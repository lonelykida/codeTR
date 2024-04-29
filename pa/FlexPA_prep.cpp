/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <sstream>
#include <chrono>
#include "FlexPA.h"
#include "db/infra/frTime.h"
#include "gc/FlexGC.h"
#include <omp.h>

using namespace std;
using namespace fr;

int gcCallCnt = 0;
//被设计用来合并来自不同层的引脚形状，这些形状可能会基于实例化的变换和缩放参数进行调整。
void FlexPA::prepPoint_pin_mergePinShapes(vector<gtl::polygon_90_set_data<frCoord> > &pinShapes, frPin* pin, frInstTerm* instTerm, bool isShrink) {
  frInst* inst = nullptr;
  if (instTerm) {// 检查是否有关联的实例化引脚，如果有则获取其所属的实例对象
    inst = instTerm->getInst();
  }

  frTransform xform;
  if (inst) {
    inst->getUpdatedXform(xform);// 如果有实例对象，则获取该实例的最新变换信息
  }

  vector<frCoord> layerWidths;// 用来存储各层宽度信息的向量
  if (isShrink) {// 如果启用了缩放功能
    layerWidths.resize(getDesign()->getTech()->getLayers().size(), 0);// 调整层宽度向量的大小，并初始化为0
    for (int i = 0; i < int(layerWidths.size()); i++) {
      layerWidths[i] = getDesign()->getTech()->getLayer(i)->getWidth();// 获取每层的宽度并存储
    }
  }

  pinShapes.clear();// 清除之前存储的形状数据
  pinShapes.resize(getDesign()->getTech()->getLayers().size());// 根据层数调整形状数据容器大小
  for (auto &shape: pin->getFigs()) {// 遍历引脚的所有几何图形
    if (shape->typeId() == frcRect) {// 如果形状是矩形
      auto obj = static_cast<frRect*>(shape.get()); // 强制转换为矩形对象
      auto layerNum = obj->getLayerNum();// 获取所在层编号
      if (getDesign()->getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::ROUTING) {
        continue;
      }
      frBox box;
      obj->getBBox(box);
      box.transform(xform);
      gtl::rectangle_data<frCoord> rect(box.left(), box.bottom(), box.right(), box.top());
      if (isShrink) {// 如果启用了缩放
        if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) {
          gtl::shrink(rect, gtl::VERTICAL, layerWidths[layerNum] / 2);// 垂直方向缩放
        } else if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frcVertPrefRoutingDir) {
          gtl::shrink(rect, gtl::HORIZONTAL, layerWidths[layerNum] / 2);// 水平方向缩放
        }
      }
      using namespace boost::polygon::operators;
      pinShapes[layerNum] += rect; // 将矩形添加到对应层的形状集合---矩形合并
    } else if (shape->typeId() == frcPolygon) {// 如果形状是多边形
      auto obj = static_cast<frPolygon*>(shape.get());// 强制转换为多边形对象
      auto layerNum = obj->getLayerNum();// 获取所在层编号
      vector<gtl::point_data<frCoord> > points;// 用于存储多边形顶点数据的向量
      // must be copied pts
      for (auto pt: obj->getPoints()) {// 遍历多边形的所有顶点
        pt.transform(xform);// 应用变换
        points.push_back(gtl::point_data<frCoord>(pt.x(), pt.y()));// 存储变换后的顶点
      }
      gtl::polygon_90_data<frCoord> poly;// 存储变换后的顶点
      poly.set(points.begin(), points.end());// 设置多边形的顶点
      using namespace boost::polygon::operators;
      pinShapes[layerNum] += poly;// 将多边形添加到对应层的形状集合
    } else {
      cout <<"Error: FlexPA mergePinShapes unsupported shape" <<endl;
      exit(1);
    }
  }
}
//在指定的坐标范围内生成网格坐标。
void FlexPA::prepPoint_pin_genPoints_rect_genGrid(map<frCoord, int> &coords, const map<frCoord, int> &trackCoords,
                                                  frCoord low, frCoord high) {
  // 使用迭代器遍历给定的轨道坐标集合，从最低有效坐标开始
  for (auto it = trackCoords.lower_bound(low); it != trackCoords.end(); it++) {
    // 解构迭代器指向的键值对，其中 coord 是坐标，cost 是与该坐标相关联的代价（例如可能的冲突或优先级）
    auto &[coord, cost] = *it;
    if (coord > high) { // 检查当前坐标是否超出了上限
      break;
    }
    coords.insert(*it);// 将当前坐标及其代价插入到结果映射中
  }
}
  
// will not generate center for wider edge
// 主要用于生成矩形区域中心点的坐标，该坐标可能用作引脚的访问点（Access Point）。
void FlexPA::prepPoint_pin_genPoints_rect_genCenter(map<frCoord, int> &coords, frLayerNum layerNum, frCoord low, frCoord high) {
  //bool enableOutput = true;
  bool enableOutput = false;
  // if touching two tracks, then no center??
  int cnt = 0;// 统计已存在坐标点的数量，如果存在两个或更多点，则可能不需要在中心再生成一个点
  for (auto it = coords.lower_bound(low); it != coords.end(); it++) {//lower_bound从左到右查一个大于等于low的数字
    auto &[c1, c2] = *it;
    if (c1 > high) {
      break;
    }
    if (c2 == 0) {
      cnt++;
    }
  }
  if (cnt >= 2) {
    return;
  }
  //if (high - low > 3 * (frCoord)(getDesign()->getTech()->getLayer(layerNum)->getWidth())) {
  //  return;
  //}

  frCoord manuGrid = getDesign()->getTech()->getManufacturingGrid();//网格尺寸
  frCoord coord = (low + high) / 2 / manuGrid * manuGrid;//计算中心点坐标，并对其进行制造网格对齐
  auto it = coords.find(coord);// 检查生成的中心点坐标是否已经存在于坐标集合中
  if (it == coords.end()) {// 如果中心点坐标尚不存在
    if (enableOutput) {
      double dbu = getDesign()->getTopBlock()->getDBUPerUU();
      cout <<"gen center pt@ " << coord / dbu <<endl;
    }
    coords.insert(make_pair(coord, 2));// 将新的中心点坐标添加到集合中，初始代价设置为2
  } else {// 如果中心点坐标已存在
    coords[coord] = std::min(coords[coord], 2);// 更新该坐标的代价为较小的值，确保使用较低代价
  }
}

void FlexPA::prepPoint_pin_genPoints_rect_ap_helper(vector<unique_ptr<frAccessPoint> > &aps, set<pair<frPoint, frLayerNum> > &apset,
                                                    const gtl::rectangle_data<frCoord> &maxrect, frCoord x, frCoord y, 
                                                    frLayerNum layerNum, bool allowPlanar, bool allowVia, 
                                                    int lowCost, int highCost) {
  gtl::point_data<frCoord> pt(x, y);//创建点对象
  if (!gtl::contains(maxrect, pt)) {
    return;// 检查点是否在给定的矩形区域内
  }
  frPoint fpt(x, y);
  if (apset.find(make_pair(fpt, layerNum)) != apset.end()) { // 检查当前层是否已经存在相同位置的访问点，如果存在则不再重复生成
    return;
  }
  auto ap = make_unique<frAccessPoint>();//新的访问点对象
  ap->setPoint(fpt);
  ap->setLayerNum(layerNum);
  if (allowPlanar) {// 根据是否允许 planar access 设置访问点的可访问方向
    auto lowerLayer = getDesign()->getTech()->getLayer(layerNum);//获取当前层信息
    ap->setAccess(frDirEnum::W, true);//设置可访问对象
    ap->setAccess(frDirEnum::E, true);
    ap->setAccess(frDirEnum::S, true);
    ap->setAccess(frDirEnum::N, true);
    // rectonly forbid wrongway planar access
    // rightway on grid only forbid off track rightway planar access
    // horz layer// 根据当前层的方向设置不同方向的访问约束
    if (lowerLayer->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir) { 
      if (lowerLayer->getLef58RectOnlyConstraint()) {// 如果有 RECTONLY 约束，则设置对应方向的访问为不可用
        ap->setAccess(frDirEnum::S, false);
        ap->setAccess(frDirEnum::N, false);
      }
      // 如果有 RIGHTWAYONGRIDONLY 约束且当前 cost 不为0，则设置对应方向的访问为不可用
      if (lowerLayer->getLef58RightWayOnGridOnlyConstraint() && lowCost != 0) {
        ap->setAccess(frDirEnum::W, false);
        ap->setAccess(frDirEnum::E, false);
      } 
    }
    // vert layer
    if (lowerLayer->getDir() == frPrefRoutingDirEnum::frcVertPrefRoutingDir) {
      if (lowerLayer->getLef58RectOnlyConstraint()) {  
        ap->setAccess(frDirEnum::W, false);
        ap->setAccess(frDirEnum::E, false);
      }
      if (lowerLayer->getLef58RightWayOnGridOnlyConstraint() && lowCost != 0) {
        ap->setAccess(frDirEnum::S, false);
        ap->setAccess(frDirEnum::N, false);
      }
    }
  } else {
    ap->setAccess(frDirEnum::W, false);
    ap->setAccess(frDirEnum::E, false);
    ap->setAccess(frDirEnum::S, false);
    ap->setAccess(frDirEnum::N, false);
  }
  ap->setAccess(frDirEnum::D, false);
  if (allowVia) {
    // if (layerNum + 1 <= getDesign()->getTech()->getTopLayerNum()) {
      ap->setAccess(frDirEnum::U, true);// 如果允许通过 via，则设置上方类型为有效
      //cout <<"@@@set U valid, check " <<ap->hasAccess(frDirEnum::U) <<endl;
    // }
  } else {
    ap->setAccess(frDirEnum::U, false); // 如果不允许通过 via，则设置上方类型为不可用
  }
  // 设置访问点的类型
  ap->setType((frAccessPointEnum)lowCost, true); // 设置下方类型
  ap->setType((frAccessPointEnum)highCost, false); // 设置上方类型
  // 将生成的访问点存储到访问点集合中
  aps.push_back(std::move(ap));
  // 将生成的访问点坐标和层号存储到访问点集合中，用于后续检查是否重复生成
  apset.insert(make_pair(fpt, layerNum));
}
//基于给定的矩形区域和坐标点生成访问点
void FlexPA::prepPoint_pin_genPoints_rect_ap(vector<unique_ptr<frAccessPoint> > &aps, set<pair<frPoint, frLayerNum> > &apset,
                                             const gtl::rectangle_data<frCoord> &rect,
                                             frLayerNum layerNum, bool allowPlanar, bool allowVia, bool isLayer1Horz,
                                             const map<frCoord, int> &xCoords, const map<frCoord, int> &yCoords, 
                                             int lowerType, int upperType) {
  // build points;
  for (auto &[xCoord, costX]: xCoords) {
    for (auto &[yCoord, costY]: yCoords) {
      // lower full/half/center
      auto &lowCost  = isLayer1Horz    ? costY : costX;
      auto &highCost = (!isLayer1Horz) ? costY : costX;
      if (lowCost == lowerType && highCost == upperType) {// 如果上方和下方操作类型匹配，则生成访问点
        prepPoint_pin_genPoints_rect_ap_helper(aps, apset, rect, xCoord, yCoord, layerNum, allowPlanar, allowVia, lowCost, highCost);
      }
    }
  }
}
// 用于为引脚在其对应矩形区域的边缘生成封装优化的访问点（Enclosure Optimized Access Points）。
void FlexPA::prepPoint_pin_genPoints_rect_genEnc(map<frCoord, int> &coords, const gtl::rectangle_data<frCoord> &rect, 
                                                 frLayerNum layerNum, bool isCurrLayerHorz) {
  auto rectWidth  = gtl::delta(rect, gtl::HORIZONTAL);
  auto rectHeight = gtl::delta(rect, gtl::VERTICAL);
  int maxNumViaTrial = 2;// 最大尝试次数为2次
  if (layerNum + 1 > getDesign()->getTech()->getTopLayerNum()) {// 如果当前层是顶层，则直接返回
    return;
  }
  // hardcode first two single vias// 存储当前层上的via定义
  vector<frViaDef*> viaDefs;
  int cnt = 0;
  for (auto &[tup, via]: layerNum2ViaDefs[layerNum + 1][1]) {// 遍历当前层上的via定义，添加到viaDefs中
    viaDefs.push_back(via);
    cnt++; 
    if (cnt >= maxNumViaTrial) {
      break;
    }
  }
  frBox box;
  for (auto &viaDef: viaDefs) {// 遍历viaDefs中的每个via定义
    frVia via(viaDef);// 创建一个via对象并获取其第一层的边界框
    via.getLayer1BBox(box);
    auto viaWidth  = box.right() - box.left();
    auto viaHeight = box.top()   - box.bottom();
    if (viaWidth > rectWidth || viaHeight > rectHeight) {// 如果via的宽度或高度大于矩形的宽度或高度，则跳过
      //cout <<"@@@" <<viaDef->getName() <<" rect " <<rectWidth <<" " <<rectHeight <<" via " <<viaWidth <<" " <<viaHeight <<endl;
      continue;
    }
    if (isCurrLayerHorz) {// 根据当前层是否水平排布来计算坐标
      auto coord = gtl::yh(rect) - (box.top() - 0);// 计算上边界的坐标
      auto it = coords.find(coord);
      if (it == coords.end()) {
        //cout << coord / 2000.0 <<endl;
        coords.insert(make_pair(coord, 3));
      } else {
        coords[coord] = std::min(coords[coord], 3);
      }
      coord = gtl::yl(rect) + (0 - box.bottom());// 计算下边界的坐标
      it = coords.find(coord);
      if (it == coords.end()) {
        //cout << coord / 2000.0 <<endl;
        coords.insert(make_pair(coord, 3));
      } else {
        coords[coord] = std::min(coords[coord], 3);
      }
    } else {// 计算右边界的坐标
      auto coord = gtl::xh(rect) - (box.right() - 0);
      auto it = coords.find(coord);
      if (it == coords.end()) {
        //cout << coord / 2000.0 <<endl;
        coords.insert(make_pair(coord, 3));
      } else {
        coords[coord] = std::min(coords[coord], 3);
      }// 计算左边界的坐标
      coord = gtl::xl(rect) + (0 - box.left());
      it = coords.find(coord);
      if (it == coords.end()) {
        //cout << coord / 2000.0 <<endl;
        coords.insert(make_pair(coord, 3));
      } else {
        coords[coord] = std::min(coords[coord], 3);
      }
    }
  }
}

// 用于生成特定矩形区域内的引脚访问点（Access Points, APs）。
void FlexPA::prepPoint_pin_genPoints_rect(vector<unique_ptr<frAccessPoint> > &aps, set<pair<frPoint, frLayerNum> > &apset,
                                          const gtl::rectangle_data<frCoord> &rect,
                                          frLayerNum layerNum, bool allowPlanar, bool allowVia, int lowerType, int upperType, bool isMacroCellPin) {
  if (std::min(gtl::delta(rect, gtl::HORIZONTAL), gtl::delta(rect, gtl::VERTICAL)) <// 如果矩形的最小尺寸小于该层的最小宽度，不生成访问点
      getDesign()->getTech()->getLayer(layerNum)->getMinWidth()) {
    return;
  }
  frLayerNum secondLayerNum = 0;// 寻找二级层，用于在需要的情况下生成跨层访问点
  if (layerNum + 2 <= getDesign()->getTech()->getTopLayerNum()) {
    secondLayerNum = layerNum + 2;//上2层
  } else if (layerNum - 2 >= getDesign()->getTech()->getBottomLayerNum()) {
    secondLayerNum = layerNum - 2;//下2层
  } else {
    cout <<"Error: prepPoint_pin_genPoints_rect cannot find secondLayerNum" <<endl;
    exit(1);
  }
// 获取当前层及二级层的轨道坐标集合
  auto &layer1TrackCoords = trackCoords[layerNum];
  auto &layer2TrackCoords = trackCoords[secondLayerNum];
  bool isLayer1Horz = (getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
// 初始化坐标映射表
  map<frCoord, int> xCoords;
  map<frCoord, int> yCoords;
// 是否使用中心线来生成访问点
  bool useCenterLine = false;
  if (isMacroCellPin) {//宏单元引脚会考虑使用中心线
    auto rectDir = gtl::guess_orientation(rect);
    if ((rectDir == gtl::HORIZONTAL && isLayer1Horz) || // 检查矩形方向是否与当前层的布线方向一致
        (rectDir == gtl::VERTICAL && !isLayer1Horz)) {
      auto layerWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth();
      // 判断矩形的宽度或高度是否小于层宽度的两倍
      // 如果矩形在宽度上很窄（小于两倍层宽），则使用中心线，这通常意味着矩形是细长的
      if ((rectDir == gtl::HORIZONTAL && gtl::delta(rect, gtl::VERTICAL) < 2 * layerWidth) ||
          (rectDir == gtl::VERTICAL && gtl::delta(rect, gtl::HORIZONTAL) < 2 * layerWidth)) {
        useCenterLine = true;
      }
    }
  }
    // 网格点：根据层的制造网格计算出的点。
    // 中心点：如果引脚区域足够窄，可能会在其几何中心放置访问点。
    // 封装优化点：在引脚的边缘附近，尤其是在可以放置通孔（Via）的情况下。
  // gen all full/half grid coords// 生成网格坐标并对它们进行分类
  // 论文图3
  if (!isMacroCellPin || !useCenterLine) {
    if (isLayer1Horz) {// 如果当前层是水平方向，生成y坐标的网格和中心线
      prepPoint_pin_genPoints_rect_genGrid(yCoords, layer1TrackCoords, gtl::yl(rect), gtl::yh(rect));
      prepPoint_pin_genPoints_rect_genGrid(xCoords, layer2TrackCoords, gtl::xl(rect), gtl::xh(rect));
      if (lowerType >= 2) {
        prepPoint_pin_genPoints_rect_genCenter(yCoords, layerNum, gtl::yl(rect), gtl::yh(rect));
      }
      if (lowerType >= 3) {
        prepPoint_pin_genPoints_rect_genEnc(yCoords, rect, layerNum, isLayer1Horz);
      }
      if (upperType >= 2) {
        prepPoint_pin_genPoints_rect_genCenter(xCoords, layerNum, gtl::xl(rect), gtl::xh(rect));
      }
      if (upperType >= 3) {
        prepPoint_pin_genPoints_rect_genEnc(xCoords, rect, layerNum, !isLayer1Horz);
      }
    } else {// 如果当前层是垂直方向，生成x坐标的网格和中心线
      prepPoint_pin_genPoints_rect_genGrid(xCoords, layer1TrackCoords, gtl::xl(rect), gtl::xh(rect));
      prepPoint_pin_genPoints_rect_genGrid(yCoords, layer2TrackCoords, gtl::yl(rect), gtl::yh(rect));
      if (lowerType >= 2) {
        prepPoint_pin_genPoints_rect_genCenter(xCoords, layerNum, gtl::xl(rect), gtl::xh(rect));
      }
      if (lowerType >= 3) {
        prepPoint_pin_genPoints_rect_genEnc(xCoords, rect, layerNum, isLayer1Horz);
      }
      if (upperType >= 2) {
        prepPoint_pin_genPoints_rect_genCenter(yCoords, layerNum, gtl::yl(rect), gtl::yh(rect));
      }
      if (upperType >= 3) {
        prepPoint_pin_genPoints_rect_genEnc(yCoords, rect, layerNum, !isLayer1Horz);
      }
    }
  } else {
    if (isLayer1Horz) {
      lowerType = 0;
      prepPoint_pin_genPoints_rect_genGrid(xCoords, layer2TrackCoords, gtl::xl(rect), gtl::xh(rect));
      if (upperType >= 2) {
        prepPoint_pin_genPoints_rect_genCenter(xCoords, layerNum, gtl::xl(rect), gtl::xh(rect));
      }
      if (upperType >= 3) {
        prepPoint_pin_genPoints_rect_genEnc(xCoords, rect, layerNum, !isLayer1Horz);
      }
      prepPoint_pin_genPoints_rect_genCenter(yCoords, layerNum, gtl::yl(rect), gtl::yh(rect));
      for (auto &[yCoord, cost]: yCoords) {
        yCoords[yCoord] = 0;
      }
    } else {
      // prepPoint_pin_genPoints_rect_genGrid(xCoords, layer1TrackCoords, gtl::xl(rect), gtl::xh(rect));
      prepPoint_pin_genPoints_rect_genGrid(yCoords, layer2TrackCoords, gtl::yl(rect), gtl::yh(rect));
      // if (lowerType >= 2) {
      //   prepPoint_pin_genPoints_rect_genCenter(xCoords, layerNum, gtl::xl(rect), gtl::xh(rect));
      // }
      // if (lowerType >= 3) {
      //   prepPoint_pin_genPoints_rect_genEnc(xCoords, rect, layerNum, isLayer1Horz);
      // }
      if (upperType >= 2) {
        prepPoint_pin_genPoints_rect_genCenter(yCoords, layerNum, gtl::yl(rect), gtl::yh(rect));
      }
      if (upperType >= 3) {
        prepPoint_pin_genPoints_rect_genEnc(yCoords, rect, layerNum, !isLayer1Horz);
      }
      prepPoint_pin_genPoints_rect_genCenter(xCoords, layerNum, gtl::xl(rect), gtl::xh(rect));
      for (auto &[xCoord, cost]: xCoords) {
        xCoords[xCoord] = 0;
      }
    }
  }
  prepPoint_pin_genPoints_rect_ap(aps, apset, rect, layerNum, allowPlanar, allowVia, 
                                  isLayer1Horz, xCoords, yCoords, lowerType, upperType);
}
// 用于在特定层上为给定的引脚生成访问点（Access Points, APs）。
// 该函数基于引脚在该层的形状数据生成潜在的访问点，并根据特定规则决定这些访问点是否合法。
void FlexPA::prepPoint_pin_genPoints_layerShapes(vector<unique_ptr<frAccessPoint> > &aps, set<pair<frPoint, frLayerNum> > &apset,
                                                 frPin* pin, frInstTerm* instTerm,
                                                 const gtl::polygon_90_set_data<frCoord> &layerShapes,
                                                 frLayerNum layerNum, bool allowVia, int lowerType, int upperType) {
  //bool enableOutput = false;
  //bool enableOutput = true;
  if (getDesign()->getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::ROUTING) {
    return;
  }
  bool allowPlanar = true;//是否允许平面访问
  bool isMacroCellPin = false;//是否为宏单元
  // bool isIOPin = false;
  if (instTerm) {
    // 如果引脚属于某个实例
    // 根据实例类型确定是否允许平面访问点
    if (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
        instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
        instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
        instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL) {
      if ((layerNum >= VIAINPIN_BOTTOMLAYERNUM && layerNum <= VIAINPIN_TOPLAYERNUM) || 
          layerNum <= VIA_ACCESS_LAYERNUM) {// 在特定层范围内可能禁止平面访问
        // if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
          allowPlanar = false;
        // }
      }
    } else if (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK ||// 如果没有实例（可能是I/O引脚），则视为宏单元引脚
               instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD ||
               instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD_POWER ||
               instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::RING) {
      isMacroCellPin = true;// 标记为宏单元引脚
      // if (layerNum >= VIAONLY_MACROCELLPIN_BOTTOMLAYERNUM && layerNum <= VIAONLY_MACROCELLPIN_TOPLAYERNUM) {
      //   allowPlanar = false;
      // }
    }
  } else {
    // isIOPin = true;
    // IO term is treated as the MacroCellPin as the top block
    isMacroCellPin = true;
    allowPlanar = true;
    allowVia = false;
  }
  // lower layer is current layer
  // righway on grid only forbid off track up via access on upper layer
  
  // 处理上层的可能限制
  auto upperLayer = (layerNum + 2 <= getDesign()->getTech()->getTopLayerNum()) ? getDesign()->getTech()->getLayer(layerNum + 2) : nullptr;
  if (!isMacroCellPin && upperLayer && upperLayer->getLef58RightWayOnGridOnlyConstraint() && upperType != 0) {
    return;
  }
  vector<gtl::rectangle_data<frCoord> > maxrects;// 生成当前层的最大矩形，用于访问点的生成
  gtl::get_max_rectangles(maxrects, layerShapes);
  for (auto &bboxRect: maxrects) {
     // 对每个矩形生成访问点
    prepPoint_pin_genPoints_rect(aps, apset, bboxRect, layerNum, allowPlanar, allowVia, lowerType, upperType, isMacroCellPin);
  }
}


// filter off-grid coordinate
// lower on-grid 0, upper on-grid 0 = 0
// lower 1/2     1, upper on-grid 0 = 1
// lower center  2, upper on-grid 0 = 2
// lower center  2, upper center  2 = 4

// 主要用于生成特定引脚的访问点（Access Points, APs）。
// 该函数根据引脚所在的各个层的形状数据生成潜在的访问点，并根据特定的规则决定这些访问点是否合法。
void FlexPA::prepPoint_pin_genPoints(vector<unique_ptr<frAccessPoint> > &aps, set<pair<frPoint, frLayerNum> > &apset,
                                     frPin* pin, frInstTerm *instTerm,
                                     const vector<gtl::polygon_90_set_data<frCoord> > &pinShapes,
                                     int lowerType, int upperType) {
  bool enableOutput = false;
  //bool enableOutput = true;
  // only VIA_ACCESS_LAYERNUM layer can have via access
  bool allowVia = true;// 控制是否允许在该层生成通孔（Via）的标志
  frLayerNum layerNum = (int)pinShapes.size() - 1;// 从最顶层开始处理
  // 逆向遍历每一层的形状集合
  for (auto it = pinShapes.rbegin(); it != pinShapes.rend(); it++) {
    if ((layerNum == VIA_ACCESS_LAYERNUM || layerNum == VIA_ACCESS_LAYERNUM + 2) && instTerm != nullptr) {
      allowVia = true;// 如果层允许通孔并且引脚属于某个实例，则设置允许通孔
    } else {
      allowVia = false;
    }// 如果当前层不为空且属于路由层
    if (!it->empty() && getDesign()->getTech()->getLayer(layerNum)->getType() == frLayerTypeEnum::ROUTING) {
      //cout <<"via layernum = " <<layerNum <<endl;// 调用函数生成当前层的访问点 
      prepPoint_pin_genPoints_layerShapes(aps, apset, pin, instTerm, *it, layerNum, allowVia, lowerType, upperType);
      // allowVia = false;
    }
    layerNum--;
  }
  if (enableOutput) {
    if (aps.empty()) {
      cout <<"Warning: prepPoint_pin_genPoints zero ap" <<endl;
    }
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    frPoint pt;
    for (auto &ap: aps) {
      ap->getPoint(pt);
      cout <<"generated ap@(" <<pt.x() / dbu <<", " <<pt.y() / dbu <<") "
           <<getDesign()->getTech()->getLayer(ap->getLayerNum())->getName() <<" , cost=" 
           <<ap->getCost() <<endl;
    }
  }
}
// 首先根据当前点的位置和方向计算下一个点的位置。然后，检查下一个点是否在任何图层多边形的内部，并返回结果。
bool FlexPA::prepPoint_pin_checkPoint_planar_ep(frPoint &ep, 
                                                const vector<gtl::polygon_90_data<frCoord> > &layerPolys,
                                                const frPoint &bp, frLayerNum layerNum, frDirEnum dir, int stepSizeMultiplier) {
  frCoord x = bp.x();//初始化变量
  frCoord y = bp.y();
  frCoord width = getDesign()->getTech()->getLayer(layerNum)->getWidth();
  frCoord stepSize = stepSizeMultiplier * width;
  switch (dir) {// 根据方向计算下一个点的位置
    case (frDirEnum::W):
      x -= stepSize;
      break;
    case (frDirEnum::E):
      x += stepSize;
      break;
    case (frDirEnum::S):
      y -= stepSize;
      break;
    case (frDirEnum::N):
      y += stepSize;
      break;
    default:
      std::cout << "unexpected direction in getPlanarEP\n";
  }
  ep.set(x, y);
  gtl::point_data<frCoord> pt(x, y);
  bool outside = true; // 检查下一个点是否在任何图层多边形内部
  for (auto &layerPoly: layerPolys) {
    if (gtl::contains(layerPoly, pt)) {
      outside = false;
      break;
    }
  }

  // width enclosure
  // bool isWidthEnclosed = false;
  // frCoord widthX1 = bp.x();
  // frCoord widthX2 = bp.x();
  // frCoord widthY1 = bp.y();
  // frCoord widthY2 = bp.y();

  // if (dir == frDirEnum::E || dir == frDirEnum::W) {
  //   widthY1 += width / 2;
  //   widthY2 -= width / 2;
  // } else if (dir == frDirEnum::N || dir == frDirEnum::S) {
  //   widthX1 += width / 2;
  //   widthX2 -= width / 2;
  // }
  // gtl::point_data<frCoord> widthPt1(widthX1, widthY1);
  // gtl::point_data<frCoord> widthPt2(widthX2, widthY2);
  // for (auto &layerPoly: layerPolys) {
  //   if (gtl::contains(layerPoly, widthPt1) && gtl::contains(layerPoly, widthPt2)) {
  //     isWidthEnclosed = true;
  //     break;
  //   }
  // }

  // check width
  // return (outside && isWidthEnclosed);
  return outside;
}
// 这个函数用于在控制台输出检查点的详细信息，包括通过哪种检查（GC或DRC）以及检查的类型（干净或污染）。
void FlexPA::prepPoint_pin_checkPoint_print_helper(frAccessPoint* ap, frPin* pin, frInstTerm* instTerm,
                                                   frDirEnum dir, int typeGC, int typeDRC,
                                                   frPoint bp, frPoint ep, frViaDef* viaDef) {
  //double dbu = getDesign()->getTopBlock()->getDBUPerUU();
  // type == 1 clean; type == 2 drc
  // typeGC == 1 表示干净，typeGC == 2 表示污染；typeDRC == 2 表示DRC错误，typeDRC == 1 表示DRC干净
  if (typeDRC != -1) { // compare mode
    if (typeGC == 1 && typeDRC == 2) {
      cout <<"  gcmisses";// 通过gc检查但未通过drc检查
    } else if (typeGC == 2 && typeDRC == 1) {
      cout <<"  gcfalsealarm";// 未通过gc检查但通过drc检查
    } else {
      return;
    }
  } else { // pure gc mode   // 纯gc模式
    if (typeGC == 1) {
      cout <<"  gcclean";
    } else if (typeGC == 2) {
      cout <<"  gcdirty";
    } else {
      return;
    }
  }
  if (viaDef) {// 如果是通过via，则打印"via"，否则打印"seg"
    cout <<"via";
  } else {
    cout <<"seg";
  }
  frTransform xform, revertCellXForm;
  if (instTerm) {
    frInst* inst = instTerm->getInst();
    inst->getTransform(xform);
    revertCellXForm.set(-xform.xOffset(), -xform.yOffset());
    revertCellXForm.set(frcR0);
    bp.transform(revertCellXForm);
    ep.transform(revertCellXForm);
  }
  frPoint transSP, transEP;
  if (bp < ep) {
    transSP = bp;
    transEP = ep;
  } else {
    transSP = ep;
    transEP = bp;
  }// 方向与名称的映射
  map<frOrientEnum, string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};
  if (viaDef) {// 如果是via
    cout <<" " << instTerm->getInst()->getRefBlock()->getName() << " "
         << instTerm->getTerm()->getName() << " " << viaDef->getName() << " "
         << bp.x() << " " << bp.y() 
         << " " << orient2Name[instTerm->getInst()->getOrient()] << "\n";
  } else {// 如果是路径段
    frCoord layerWidth = getDesign()->getTech()->getLayer(ap->getLayerNum())->getWidth();
    if (transSP.x() == transEP.x()) {
      cout << " " << instTerm->getInst()->getRefBlock()->getName() << " "
           << instTerm->getTerm()->getName() << " "
           << transSP.x() << " " << transSP.y() - layerWidth / 2 << " "
           << transEP.x() << " " << transEP.y() + layerWidth / 2 << " "
           << orient2Name[instTerm->getInst()->getOrient()]      << " "
           << getDesign()->getTech()->getLayer(ap->getLayerNum())->getName() << "\n";
    } else {
      cout << " " << instTerm->getInst()->getRefBlock()->getName() << " "
           << instTerm->getTerm()->getName() << " "
           << transSP.x() - layerWidth / 2 << " " << transSP.y() << " "
           << transEP.x() + layerWidth / 2 << " " << transEP.y() << " "
           << orient2Name[instTerm->getInst()->getOrient()]      << " "
           << getDesign()->getTech()->getLayer(ap->getLayerNum())->getName() << "\n";
    }
  }
}
// 这段代码首先获取访问点的起点，然后跳过仅允许通过垂直方向的访问点。接下来，计算交点并检查是否在多边形内部。
// 对于标准单元引脚，如果交点在多边形内部，则将该方向的访问权限设为false。
// 然后，创建路径段对象并初始化FlexGCWorker，执行路径搜索以确定访问点的访问权限。最后，根据搜索结果更新访问点的访问权限。
void FlexPA::prepPoint_pin_checkPoint_planar(frAccessPoint* ap, 
                                             const vector<gtl::polygon_90_data<frCoord> > &layerPolys,
                                             frDirEnum dir,
                                             frPin* pin, frInstTerm* instTerm) {
  bool enableOutput = false;
  //bool enableOutput = true;
  //bool compMode = false;
  frPoint bp, ep;//获取访问点的起点
  ap->getPoint(bp);
  // skip viaonly access// 跳过仅允许通过垂直方向的访问点
  if (!ap->hasAccess(dir)) {
    //if (enableOutput) {
    //  prepPoint_pin_checkPoint_print_helper(ap, pin, instTerm, dir, 0, -1, bp, ep, nullptr);
    //}
    return;
  }// 判断是否为标准单元引脚
  bool isStdCellPin = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
                                    instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
                                    instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
                                    instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL));
  // if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
     
  // } else {/ 计算交点并检查是否在多边形内部
    bool isOutSide = prepPoint_pin_checkPoint_planar_ep(ep, layerPolys, bp, ap->getLayerNum(), dir);
    // skip if two width within shape for standard cell// 对于标准单元引脚，如果交点在多边形内部，则将该方向的访问权限设为false
    if (isStdCellPin && !isOutSide) {
      ap->setAccess(dir, false);
      if (enableOutput) {
        prepPoint_pin_checkPoint_print_helper(ap, pin, instTerm, dir, 0, -1, bp, ep, nullptr);
      }
      return;
    }
  // }
// 创建路径段对象
  auto ps = make_unique<frPathSeg>();
  auto style = getDesign()->getTech()->getLayer(ap->getLayerNum())->getDefaultSegStyle();
  if (dir == frDirEnum::W || dir == frDirEnum::S) {
    ps->setPoints(ep, bp);
    style.setEndStyle(frcTruncateEndStyle, 0);
  } else {
    ps->setPoints(bp, ep);
    style.setBeginStyle(frcTruncateEndStyle, 0);
  }
  ps->setLayerNum(ap->getLayerNum());
  ps->setStyle(style);
  if (instTerm && instTerm->hasNet()) {
    ps->addToNet(instTerm->getNet());
  } else {
    ps->addToPin(pin);
  }

  // old drcWorker
  int typeDRC = -1;
  // if (compMode) {
  //   std::vector<frBlockObject*> pinObjs;
  //   std::set<frBlockObject*> pinObjSet;
  //   {
  //     frBox box;
  //     frInst* inst = nullptr;
  //     if (instTerm) {
  //       inst = instTerm->getInst();
  //       instTerm->getInst()->getBoundaryBBox(box);
  //     }
  //     auto regionQuery = design->getRegionQuery();
  //     std::vector<rq_rptr_value_t<frBlockObject> > queryResult;
  //     for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
  //       queryResult.clear();
  //       regionQuery->query(box, layerNum, queryResult);
  //       for (auto &[objBox, objPtr]: queryResult) {
  //         auto it = pinObjSet.find(objPtr);
  //         if (it == pinObjSet.end()) {
  //           if ((objPtr->typeId() == frcInstTerm && (static_cast<frInstTerm*>(objPtr))->getInst() == inst) || 
  //               (objPtr->typeId() == frcInstBlockage && (static_cast<frInstBlockage*>(objPtr)->getInst() == inst))) {
  //             pinObjs.push_back(objPtr);
  //           }
  //           pinObjSet.insert(objPtr);
  //         }
  //       }
  //     }
  //   }
  //   pinObjs.push_back(ps.get());
  //   DRCWorker drcWorker(design, pinObjs);
  //   drcWorker.init();
  //   drcWorker.setup();
  //   drcWorker.check();
  //   if (drcWorker.getViolations().empty()) {
  //     typeDRC = 1;
  //   } else {
  //     typeDRC = 2;
  //   }
  // }

  // new gcWorker// 初始化GCWorker
  FlexGCWorker gcWorker(getDesign());
  gcWorker.setIgnoreMinArea();
  frBox extBox(bp.x() - 3000, bp.y() - 3000, bp.x() + 3000, bp.y() + 3000);
  gcWorker.setExtBox(extBox);
  gcWorker.setDrcBox(extBox);
  if (instTerm) {
    gcWorker.setTargetObj(instTerm->getInst());
  } else {
    gcWorker.setTargetObj(pin->getTerm());
  }
  gcWorker.initPA0();
  if (instTerm) {
    if (instTerm->hasNet()) {
      gcWorker.addPAObj(ps.get(), instTerm->getNet());
    } else {
      gcWorker.addPAObj(ps.get(), instTerm);
    }
  } else {
    if (pin->getTerm()->hasNet()) {
      gcWorker.addPAObj(ps.get(), pin->getTerm()->getNet());
    } else {
      gcWorker.addPAObj(ps.get(), pin->getTerm());
    }
  }
  gcWorker.initPA1();
  gcWorker.main();
  gcWorker.end();

  int typeGC  = 0;
   // 更新访问点的访问权限
  if (gcWorker.getMarkers().empty()) {
    ap->setAccess(dir, true);
    // if (instTerm->getInst()->getRefBlock()->getName() == string("gf14_1rw_d128_w116_m2_bit_mod") && pin->getTerm()->getName() == string("EMAS")) {
    //     cout << " @@@ debug @@@ here1" << endl;
    //     cout << " bp (" << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
    //     cout << " ep (" << ep.x() / 2000.0 << ", " << ep.y() / 2000.0 << ")\n";
    // }
    // if (instTerm->getInst()->getRefBlock()->getName() == string("gf14_1rw_d128_w116_m2_bit_mod") && pin->getTerm()->getName() == string("CEN")) {
    //     cout << " @@@ debug @@@ here1 CEN" << endl;
    //     cout << " bp (" << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
    //     cout << " ep (" << ep.x() / 2000.0 << ", " << ep.y() / 2000.0 << ")\n";
    // }
    typeGC = 1;
  } else {
    // if (instTerm->getInst()->getRefBlock()->getName() == string("gf14_1rw_d128_w116_m2_bit_mod") && pin->getTerm()->getName() == string("EMAS")) {
    //   cout << "inst: " << instTerm->getInst()->getName() << ", isStdCellPin = " << isStdCellPin << endl;
    //   cout << " bp (" << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
    //   cout << " ep (" << ep.x() / 2000.0 << ", " << ep.y() / 2000.0 << ")\n";
    //   for (auto &marker: gcWorker.getMarkers()) {
    //     cout << " @@@ debug @@@ " << (int)marker->getConstraint()->typeId() << endl;
    //     frBox bbox;
    //     marker->getBBox(bbox);
    //     cout << "    (" << bbox.left() / 2000.0 << ", " << bbox.bottom() / 2000.0 << ") - ("
    //          << bbox.right() / 2000.0 << ", " << bbox.top() / 2000.0 << ")\n";
    //     for (auto src: marker->getSrcs()) {
    //       if (src) {
    //         cout << "      " << (int)src->typeId() << endl;
    //         if (src->typeId() == frcNet) {
    //           cout << "        " << static_cast<frNet*>(src)->getName() << endl;
    //         }
    //       } else {
    //         cout << "      nullptr\n";
    //       }
    //     }
    //   }
    // }
    ap->setAccess(dir, false);
    typeGC = 2;
  }
  if (enableOutput) {
    prepPoint_pin_checkPoint_print_helper(ap, pin, instTerm, dir, typeGC, typeDRC, bp, ep, nullptr);
  }
}
// 用于检查垂直方向的访问点是否可以通过垂直（via）访问。
// 它考虑了多个条件，包括访问点的位置、是否位于单元格边界上、是否在via中以及与多边形的交集等。
void FlexPA::prepPoint_pin_checkPoint_via(frAccessPoint* ap, 
                                          const gtl::polygon_90_set_data<frCoord> &polyset,
                                          frDirEnum dir,
                                          frPin* pin, frInstTerm* instTerm) {
  //bool enableOutput = false;
  //bool enableOutput = true;
  //bool compMode = false;
  frPoint bp;
  ap->getPoint(bp);
  auto layerNum = ap->getLayerNum();
  //cout <<"@@@mmm" <<endl;
  // skip planar only access
  if (!ap->hasAccess(dir)) {
    //cout <<"no via access, check " <<ap->hasAccess(frDirEnum::U) <<endl;
    return;
  }

  bool viainpin = false;
  // 检查是否在via中
  if (layerNum >= VIAINPIN_BOTTOMLAYERNUM && layerNum <= VIAINPIN_TOPLAYERNUM) {
    viainpin = true;
  } else if (ap->getType(true) == frAccessPointEnum::frcEncOptAP || ap->getType(false) == frAccessPointEnum::frcEncOptAP) {
    viainpin = true;
  }

  int maxNumViaTrial = 2;
  // use std:pair to ensure deterministic behavior
  vector<pair<int, frViaDef*> > viaDefs;// 用于存储通过检查的via定义
  // hardcode first two single vias// 硬编码的前两个单个via
  int cnt = 0;
  for (auto &[tup, viaDef]: layerNum2ViaDefs[layerNum + 1][1]) {
    viaDefs.push_back(make_pair(viaDefs.size(), viaDef));
    cnt++;
    if (cnt >= maxNumViaTrial) {
      break;
    }
  }

  // check if ap is on the left/right boundary of the cell
   // 检查ap是否在单元格的左/右边界上
  bool isLRBound = false;
  if (instTerm) {
    frBox boundaryBBox;
    instTerm->getInst()->getBoundaryBBox(boundaryBBox);
    frCoord width = getDesign()->getTech()->getLayer(layerNum)->getWidth();
    if (bp.x() <= boundaryBBox.left() + 3 * width || bp.x() >= boundaryBBox.right() - 3 * width) {
      isLRBound = true;
    }
  }
// 用于存储有效的via定义
  set<tuple<frCoord, int, frViaDef*> > validViaDefs;
  frBox box;
  //for (auto &[idx, viaDef]: viaDefs) {
  //  auto via = make_unique<frVia>(viaDef);
  //  via->setOrigin(bp);
  //  via->getLayer1BBox(box);
  //  gtl::rectangle_data<frCoord> viarect(box.left(), box.bottom(), box.right(), box.top());
  //  auto viarectArea = gtl::area(viarect);
  //  using namespace boost::polygon::operators;
  //  auto intersection = polyset & viarect;
  //  auto intersectionArea = gtl::area(intersection);
  //  //cout <<"@@@?????" <<endl;
  //  if (viainpin && intersectionArea < viarectArea) {
  //      continue;
  //  }
  //  auto nonEnclosedArea = viarectArea - intersectionArea;
  //  //cout <<"@@@start" <<endl;
  //  if (prepPoint_pin_checkPoint_via_helper(ap, via.get(), pin, instTerm)) {
  //    //cout <<"@@@clean" <<endl;
  //    validViaDefs.insert(make_tuple(nonEnclosedArea, idx, viaDef));
  //  }
  //}
  for (auto &[idx, viaDef]: viaDefs) {
    auto via = make_unique<frVia>(viaDef);
    via->setOrigin(bp);
    via->getLayer1BBox(box);
    gtl::rectangle_data<frCoord> viarect(box.left(), box.bottom(), box.right(), box.top());
    //auto viarectArea = gtl::area(viarect);
    using namespace boost::polygon::operators;
    gtl::polygon_90_set_data<frCoord> intersection = polyset & viarect;
    gtl::rectangle_data<frCoord> intersection_extRect;
    intersection.extents(intersection_extRect);
    frCoord leftExt   = std::max(0,   gtl::xl(intersection_extRect) - gtl::xl(viarect));
    frCoord rightExt  = std::max(0, - gtl::xh(intersection_extRect) + gtl::xh(viarect));
    frCoord bottomExt = std::max(0,   gtl::yl(intersection_extRect) - gtl::yl(viarect));
    frCoord topExt    = std::max(0, - gtl::yh(intersection_extRect) + gtl::yh(viarect));
    // via ranking criteria: max extension distance beyond pin shape
    frCoord maxExt = std::max(std::max(leftExt, rightExt), std::max(bottomExt, topExt));
    // via ranking criteria for boundary pin: max horizontal extension distance beyond pin shape
    if (isLRBound && !viainpin) {
      maxExt = std::max(leftExt, rightExt);
    }
    //cout <<"@@@?????" <<endl;
    if (viainpin && maxExt > 0) {
        continue;
    }
    if (instTerm) {
      frBox boundaryBBox;
      instTerm->getInst()->getBoundaryBBox(boundaryBBox);
      if (!boundaryBBox.contains(box)) {
        continue;
      }
    }
    
    // avoid layer2BBox outside of cell
    if (instTerm) {
      frBox layer2BBox, boundaryBBox;
      via->getLayer2BBox(layer2BBox);
      instTerm->getInst()->getBoundaryBBox(boundaryBBox);
      if (!boundaryBBox.contains(layer2BBox)) {
        continue;
      }
    }

    //cout <<"@@@start" <<endl;
    if (prepPoint_pin_checkPoint_via_helper(ap, via.get(), pin, instTerm)) {
      //cout <<"@@@clean" <<endl;
      validViaDefs.insert(make_tuple(maxExt, idx, viaDef));
    }
  }
  if (validViaDefs.empty()) {
    ap->setAccess(dir, false);
  }
  for (auto &[area, idx, viaDef]: validViaDefs) {
    ap->addViaDef(viaDef);
  }
}

bool FlexPA::prepPoint_pin_checkPoint_via_helper(frAccessPoint* ap, frVia* via, frPin* pin, frInstTerm* instTerm) {
  bool enableOutput = false;
  // bool enableOutput = true;
  //bool compMode = false;
  frPoint bp, ep;
  ap->getPoint(bp);
  //auto layerNum = ap->getLayerNum();

  if (instTerm && instTerm->hasNet()) {
    via->addToNet(instTerm->getNet());
  } else {
    via->addToPin(pin);
  }

  // old drcWorker
  int typeDRC = -1;
  // if (compMode) {
  //   std::vector<frBlockObject*> pinObjs;
  //   std::set<frBlockObject*> pinObjSet;
  //   {
  //     frBox box;
  //     frInst* inst = nullptr;
  //     if (instTerm) {
  //       inst = instTerm->getInst();
  //       instTerm->getInst()->getBoundaryBBox(box);
  //     }
  //     auto regionQuery = design->getRegionQuery();
  //     std::vector<rq_rptr_value_t<frBlockObject> > queryResult;
  //     for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
  //       queryResult.clear();
  //       regionQuery->query(box, layerNum, queryResult);
  //       for (auto &[objBox, objPtr]: queryResult) {
  //         auto it = pinObjSet.find(objPtr);
  //         if (it == pinObjSet.end()) {
  //           if ((objPtr->typeId() == frcInstTerm && (static_cast<frInstTerm*>(objPtr))->getInst() == inst) || 
  //               (objPtr->typeId() == frcInstBlockage && (static_cast<frInstBlockage*>(objPtr)->getInst() == inst))) {
  //             pinObjs.push_back(objPtr);
  //           }
  //           pinObjSet.insert(objPtr);
  //         }
  //       }
  //     }
  //   }
  //   pinObjs.push_back(via);
  //   DRCWorker drcWorker(design, pinObjs);
  //   drcWorker.addIgnoredConstraintType(frConstraintTypeEnum::frcAreaConstraint);
  //   drcWorker.init();
  //   drcWorker.setup();
  //   drcWorker.check();
  //   if (drcWorker.getViolations().empty()) {
  //     typeDRC = 1;
  //   } else {
  //     typeDRC = 2;
  //   }
  // }

  // new gcWorker
  FlexGCWorker gcWorker(getDesign());
  gcWorker.setIgnoreMinArea();
  frBox extBox(bp.x() - 3000, bp.y() - 3000, bp.x() + 3000, bp.y() + 3000);
  gcWorker.setExtBox(extBox);
  gcWorker.setDrcBox(extBox);
  if (instTerm) {
    gcWorker.setTargetObj(instTerm->getInst());
  } else {
    gcWorker.setTargetObj(pin->getTerm());
  }
  //cout <<flush;
  gcWorker.initPA0();
  if (instTerm) {
    if (instTerm->hasNet()) {
      gcWorker.addPAObj(via, instTerm->getNet());
    } else {
      gcWorker.addPAObj(via, instTerm);
    }
  } else {
    gcWorker.addPAObj(via, pin->getTerm());
  }
  gcWorker.initPA1();
  gcWorker.main();
  gcWorker.end();

  int typeGC  = 0;
  bool sol = false;
  if (gcWorker.getMarkers().empty()) {
    typeGC = 1;
    sol = true;
  } else {
    typeGC = 2;
  }
  if (enableOutput) {
    // if (instTerm->getInst()->getName() == string("_14797_") && instTerm->getTerm()->getName() == string("Y")) {
      prepPoint_pin_checkPoint_print_helper(ap, pin, instTerm, frDirEnum::U, typeGC, typeDRC, bp, ep, via->getViaDef());
      //if (typeGC == 2) {
      //  for (auto &marker: gcWorker.getMarkers()) {
      //    frBox bbox;
      //    marker.get()->getBBox(bbox);
      //    cout << "  violation on layer " << marker.get()->getLayerNum() 
      //         << ", bbox = (" << bbox.left() / 2000.0 << ", " << bbox.bottom() / 2000.0 << ") - (" << bbox.right() / 2000.0 << ", " << bbox.top() / 2000.0 << ")"
      //         << ", constraint enum = " << int(marker.get()->getConstraint()->typeId()) << "\n";
      //  }
      //}
    // }
  }
  return sol;
}

void FlexPA::prepPoint_pin_checkPoint(frAccessPoint* ap, 
                                      const gtl::polygon_90_set_data<frCoord> &polyset,
                                      const vector<gtl::polygon_90_data<frCoord> > &polys,
                                      frPin* pin, frInstTerm* instTerm) {
  bool enableOutput = false;
  // bool enableOutput = true;
  if (enableOutput) {
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    frPoint pt;
    ap->getPoint(pt);
    cout <<"checking ap@( " <<pt.x() / dbu <<" " <<pt.y() / dbu <<" ) "
         <<getDesign()->getTech()->getLayer(ap->getLayerNum())->getName() <<" , cost=" 
         <<ap->getCost() <<endl;
    //if (pt.x() != 28.8 * 2000 && pt.y() != 125.26 * 2000) {
    //  return;
    //}
  }
  /*
  frPoint bp, ep;
  ap->getPoint(bp);
  bool boundaryPlanrFound = false;
  if (!prepPoint_pin_checkPoint_planar_ep(ep, polys, bp, ap->getLayerNum(), frDirEnum::W, 1)) {
    ap->setAccess(frDirEnum::W, false);
  } else {
    boundaryPlanrFound = true;
  }
  if (!prepPoint_pin_checkPoint_planar_ep(ep, polys, bp, ap->getLayerNum(), frDirEnum::E, 1)) {
    ap->setAccess(frDirEnum::E, false);
  } else {
    boundaryPlanrFound = true;
  }
  if (!prepPoint_pin_checkPoint_planar_ep(ep, polys, bp, ap->getLayerNum(), frDirEnum::S, 1)) {
    ap->setAccess(frDirEnum::S, false);    
  } else {
    boundaryPlanrFound = true;
  }
  if (!prepPoint_pin_checkPoint_planar_ep(ep, polys, bp, ap->getLayerNum(), frDirEnum::N, 1)) {
    ap->setAccess(frDirEnum::N, false);
  } else {
    boundaryPlanrFound = true;
  }

  if (!boundaryPlanrFound) {
    ap->setAccess(frDirEnum::W, true);
    ap->setAccess(frDirEnum::E, true);
    ap->setAccess(frDirEnum::S, true);
    ap->setAccess(frDirEnum::N, true);
  }
  */

// 调用 prepPoint_pin_checkPoint_planar 函数检查平面访问方向（W、E、S、N）
  prepPoint_pin_checkPoint_planar(ap, polys, frDirEnum::W, pin, instTerm);
  prepPoint_pin_checkPoint_planar(ap, polys, frDirEnum::E, pin, instTerm);
  prepPoint_pin_checkPoint_planar(ap, polys, frDirEnum::S, pin, instTerm);
  prepPoint_pin_checkPoint_planar(ap, polys, frDirEnum::N, pin, instTerm);
  prepPoint_pin_checkPoint_via(ap, polyset, frDirEnum::U, pin, instTerm);// 调用 prepPoint_pin_checkPoint_via 函数检查垂直访问方向（U）
}
// 检查访问点是否与给定层的多边形集合相交，并进行相应的处理。
void FlexPA::prepPoint_pin_checkPoints(vector<unique_ptr<frAccessPoint> > &aps, 
                                       const vector<gtl::polygon_90_set_data<frCoord> > &layerPolysets,
                                       frPin* pin, frInstTerm* instTerm) {
  vector<vector<gtl::polygon_90_data<frCoord> > > layerPolys(layerPolysets.size());//将多边形集合转换为多边形向量
  for (int i = 0; i < (int)layerPolysets.size(); i++) {
    layerPolysets[i].get_polygons(layerPolys[i]);
  }
  for (auto &ap: aps) {// 遍历访问点集合
    auto layerNum = ap->getLayerNum();// 获取访问点所在的层号和坐标
    frPoint pt;
    ap->getPoint(pt);// 检查访问点是否与层的多边形集合相交，并进行相应处理
    //if (pt.x() == 189.3 * 2000 && pt.y() == 175.7 * 2000) {
    //  ;
    //} else {
    //  continue;
    //}
    prepPoint_pin_checkPoint(ap.get(), layerPolysets[layerNum], layerPolys[layerNum], pin, instTerm);
  }
}

void FlexPA::prepPoint_pin_updateStat(const vector<unique_ptr<frAccessPoint> > &tmpAps, frPin* pin, frInstTerm* instTerm) {
  // 检查是否是标准单元引脚
  bool isStdCellPin   = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL));
   // 检查是否是宏单元引脚
  bool isMacroCellPin = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD_POWER ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::RING));
  //bool hasAccess = false;// 遍历临时访问点集合
  for (auto &ap: tmpAps) {
    //if (ap->hasAccess()) {
    //  hasAccess = true;
    //}
    
      // 检查访问点是否有平面访问
    if (ap->hasAccess(frDirEnum::W) || ap->hasAccess(frDirEnum::E) || 
        ap->hasAccess(frDirEnum::S) || ap->hasAccess(frDirEnum::N)) { 
      if (isStdCellPin) {// 如果是标准单元引脚，增加标准单元引脚的有效平面访问点计数
        #pragma omp atomic
        stdCellPinValidPlanarApCnt++;
      }
      if (isMacroCellPin) { // 如果是宏单元引脚，增加宏单元引脚的有效平面访问点计数
        #pragma omp atomic
        macroCellPinValidPlanarApCnt++;
      }
    }
    if (ap->hasAccess(frDirEnum::U)) {  // 检查访问点是否有垂直访问
      if (isStdCellPin) {
        #pragma omp atomic
        stdCellPinValidViaApCnt++;
      }
      if (isMacroCellPin) {
        #pragma omp atomic
        macroCellPinValidViaApCnt++;
      }
    }
  }
}
//用于生成引脚访问点（Access Points, APs）的帮助函数
bool FlexPA::prepPoint_pin_helper(vector<unique_ptr<frAccessPoint> > &aps,
                                  set<pair<frPoint, frLayerNum> > &apset,
                                  vector<gtl::polygon_90_set_data<frCoord> > &pinShapes,
                                  frPin* pin, frInstTerm* instTerm, int lowerType, int upperType) {
  bool isStdCellPin   = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW || 
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL));
  bool isMacroCellPin = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD_POWER ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::RING));
  bool isIOPin = (instTerm == nullptr);// 判断是否为I/O引脚
  vector<unique_ptr<frAccessPoint> > tmpAps;// 临时存储生成的访问点
  // int prevSize = apset.size();
  // set<pair<frPoint, frLayerNum> > prevApset = apset, diffApset;
 
 // 生成引脚访问点，并将生成的访问点存储在tmpAps中
  prepPoint_pin_genPoints(tmpAps, apset, pin, instTerm, pinShapes, lowerType, upperType);
  // std::set_difference(apset.begin(), apset.end(), prevApset.begin(), prevApset.end(), std::inserter(diffApset, diffApset.end()));
  // int currSize = apset.size();
  // if (instTerm->getInst()->getName() == string("dcnt_reg_3_") && instTerm->getTerm()->getName() == string("Q")) {
  //   cout << "dcnt_reg_3_/" << instTerm->getTerm()->getName() <<" has " << currSize - prevSize 
  //        << " new access points with lowerType = " << lowerType 
  //        << " and upperType = " << upperType << "\n";
  //   // for (auto ap: diffApset) {
  //   //   cout << "  (" << ap.first.x() / 2000.0 << ", " << ap.first.y() / 2000.0 << ", " << ap.second << ")\n";
  //   // }
  // }
  // if (instTerm->getInst()->getRefBlock()->getName() == string("gf14_1rw_d128_w116_m2_bit_mod") && pin->getTerm()->getName() == string("EMAS")) {
  //   cout << "apset.size() " << apset.size() << endl;
  //   for (auto ap: apset) {
  //     cout << " ap (" << ap.first.x() / 2000.0 << ", " << ap.first.y() / 2000.0 << ")\n";
  //   }
  // }
  
  // 检查生成的访问点并添加到最终列表中
  prepPoint_pin_checkPoints(tmpAps, pinShapes, pin, instTerm);
  // 统计生成的访问点数量
  if (isStdCellPin) {
    #pragma omp atomic
    stdCellPinGenApCnt += tmpAps.size();
  }
  if (isMacroCellPin) {
    #pragma omp atomic
    macroCellPinGenApCnt += tmpAps.size();
  }
  // 根据引脚类型和生成的访问点类型添加访问点到aps
  for (auto &ap: tmpAps) {
    // for stdcell, add (i) planar access if layerNum != VIA_ACCESS_LAYERNUM, and (ii) access if exist access
    // for macro, allow pure planar ap
    if (isStdCellPin) {
      auto layerNum = ap->getLayerNum();
      // 对于标准单元格引脚，添加(1)平面访问，如果layerNum != VIA_ACCESS_LAYERNUM，以及(2)存在访问
      if ((layerNum == VIA_ACCESS_LAYERNUM && ap->hasAccess(frDirEnum::U)) ||
          (layerNum != VIA_ACCESS_LAYERNUM && ap->hasAccess())) {
      // if (ap->hasAccess(frDirEnum::U)) {
        aps.push_back(std::move(ap));
      }
    } else if ((isMacroCellPin || isIOPin) && ap->hasAccess()) {
      aps.push_back(std::move(ap));
    }
  }// 根据引脚类型和访问点数量更新统计信息，并将访问点写入引脚访问结构中
  if (isStdCellPin && (int)aps.size() >= MINNUMACCESSPOINT_STDCELLPIN) {
    prepPoint_pin_updateStat(aps, pin, instTerm);
    // write to pa
    auto it = unique2paidx.find(instTerm->getInst());
    if (it == unique2paidx.end()) {
      cout <<"Error: prepPoint_pin_helper unique2paidx not found" <<endl;
      exit(1);
    } else {
      for (auto &ap: aps) {
        pin->getPinAccess(it->second)->addAccessPoint(ap);//写入引脚访问结构
      }
    }
    return true;
  }
  if (isMacroCellPin && (int)aps.size() >= MINNUMACCESSPOINT_MACROCELLPIN) {
    prepPoint_pin_updateStat(aps, pin, instTerm);
    // write to pa
    auto it = unique2paidx.find(instTerm->getInst());
    if (it == unique2paidx.end()) {
      cout <<"Error: prepPoint_pin_helper unique2paidx not found" <<endl;
      exit(1);
    } else {
      for (auto &ap: aps) {
        pin->getPinAccess(it->second)->addAccessPoint(ap);
      }
    }
    return true;
  }
  if (isIOPin && (int)aps.size() > 0) {
    // IO term pin always only have one access
    for (auto &ap: aps) {
      pin->getPinAccess(0)->addAccessPoint(ap);
    }
    return true;
  }
  return false;
}

// first create all access points with costs用于准备每个引脚的访问点（Access Point，简称 AP）。
void FlexPA::prepPoint_pin(frPin* pin, frInstTerm* instTerm) {
  // aps are after xform
  // before checkPoints, ap->hasAccess(dir) indicates whether to check drc 
  vector<unique_ptr<frAccessPoint> > aps;// 用于存储引脚的访问点
  set<pair<frPoint, frLayerNum> > apset;// 用于避免重复的访问点
  // 判断引脚是否属于标准单元或特定类型的宏单元
  bool isStdCellPin   = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIEHIGH ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_TIELOW ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::CORE_ANTENNACELL));
  bool isMacroCellPin = (instTerm && (instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::PAD_POWER ||
                                      instTerm->getInst()->getRefBlock()->getMacroClass() == MacroClassEnum::RING));
  
  vector<gtl::polygon_90_set_data<frCoord> > pinShapes;
  if (isMacroCellPin) {// 根据是否是宏单元引脚合并引脚形状
    prepPoint_pin_mergePinShapes(pinShapes, pin, instTerm);
    // prepPoint_pin_mergePinShapes(pinShapes, pin, instTerm, true);
  } else {
    prepPoint_pin_mergePinShapes(pinShapes, pin, instTerm);
  }

  // 0 iter, gen on-grid, on-grid points
  //cout <<"iter0" <<endl;
  
  // 多个迭代尝试生成引脚访问点，不同迭代尝试不同的网格对齐策略
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 0, 0)) {
    return;
  }
  // 其余的迭代循环尝试其它配置，每次都检查是否成功生成了引脚访问点，如果成功则提前返回
  // 每个迭代改变的是引脚访问点的生成策略，比如网格对齐、中心点对齐等

  // 1st iter, gen 1/2-grid, on-grid points
  //cout <<"iter1" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 1, 0)) {
    return;
  }
  // 2nd iter, gen center, on-grid points
  //cout <<"iter2" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 2, 0)) {
    return;
  }
  // 3rd iter, gen enc-opt, on-grid points
  //cout <<"iter3" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 3, 0)) {
    return;
  }
  // 4th iter, gen on-grid, 1/2-grid points
  //cout <<"iter4" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 0, 1)) {
    return;
  }
  // 5th iter, gen 1/2-grid, 1/2-grid points
  //cout <<"iter5" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 1, 1)) {
    return;
  }
  // 6th iter, gen center, 1/2-grid points
  //cout <<"iter6" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 2, 1)) {
    return;
  }
  // 7th iter, gen enc-opt, 1/2-grid points
  //cout <<"iter7" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 3, 1)) {
    return;
  }
  // 8th iter, gen on-grid, center points
  //cout <<"iter8" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 0, 2)) {
    return;
  }
  // 9th iter, gen 1/2-grid, center points
  //cout <<"iter9" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 1, 2)) {
    return;
  }
  // 10th iter, gen center, center points
  //cout <<"iter10" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 2, 2)) {
    return;
  }
  // 11th iter, gen enc-opt, center points
  //cout <<"iter11" <<endl;
  if (prepPoint_pin_helper(aps, apset, pinShapes, pin, instTerm, 3, 2)) {
    return;
  }

  // instTerm aps are written back here if not early stopped
  // IO term aps are are written back in prepPoint_pin_helper and always early stopped
  
   // 最后，如果找到有效的访问点，更新统计信息
  prepPoint_pin_updateStat(aps, pin, instTerm);
  if (aps.empty()) {// 如果没有找到任何访问点，输出错误信息
    if (instTerm) {
      cout <<"Error: no ap for " <<instTerm->getInst()->getName() <<"/" <<instTerm->getTerm()->getName() <<endl;
    } else {
      cout <<"Error: no ap for PIN/" <<pin->getTerm()->getName() <<endl;
    }
    // 根据引脚类型增加无访问点计数
    if (isStdCellPin) {
      stdCellPinNoApCnt++;
    }
    if (isMacroCellPin) {
      macroCellPinNoApCnt++;
    }
  } else {
    // write to pa // 将找到的访问点写入对应的引脚访问对象中
    auto it = unique2paidx.find(instTerm->getInst());
    // std::map<frInst*, int,     frBlockObjectComp> unique2paidx; unique2paidx的定义
     //unique instance to pinaccess index;
    if (it == unique2paidx.end()) {
      cout <<"Error: prepPoint_pin unique2paidx not found" <<endl;
      exit(1);
    } else {
      for (auto &ap: aps) {
        // if (instTerm->getInst()->getRefBlock()->getName() == string("INVP_X1F_A9PP84TR_C14") && instTerm->getTerm()->getName() == string("Y")) {
        //  double dbu = getDesign()->getTopBlock()->getDBUPerUU();
        //  frPoint pt;
        //  for (auto &ap: aps) {
        //    ap->getPoint(pt);
        //    cout <<"checked ap@(" <<pt.x() / dbu <<", " <<pt.y() / dbu <<") "
        //         <<getDesign()->getTech()->getLayer(ap->getLayerNum())->getName() <<" , cost=" 
        //         <<ap->getCost() <<endl;
        //  }
        // }
        pin->getPinAccess(it->second)->addAccessPoint(ap);
      }
    }
  }
}
//准备设计中的所有独特实例的引脚点（pin points），并对I/O引脚进行处理。
void FlexPA::prepPoint() {
  // bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl;
  }
  int cnt = 0;

  omp_set_num_threads(MAX_THREADS);// 设置OpenMP的线程数为预定义的最大线程数
  #pragma omp parallel for schedule(dynamic)// 使用OpenMP进行并行处理，动态调度
  for (int i = 0; i < (int)uniqueInstances.size(); i++) {// 遍历所有独特的实例
    auto &inst = uniqueInstances[i];
    // only do for core and block cells // 只处理核心单元格和块单元格
    if (inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_TIEHIGH && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_TIELOW && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_ANTENNACELL && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::BLOCK &&
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::PAD &&
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::PAD_POWER &&
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::RING) {
      continue;
    }
    // temporary for testing ap
    // if (inst->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK) {
    //   continue;
    // }
    for (auto &instTerm: inst->getInstTerms()) {// 遍历实例的所有实例化引脚
      // only do for normal and clock terms// 如果是常规引脚或者时钟类型引脚应该跳过这个实例化引脚
      if (isSkipInstTerm(instTerm.get())) {
        continue;
      }
      for (auto &pin: instTerm->getTerm()->getPins()) {// 遍历每个实例化引脚的所有物理引脚
        // tmporary
        //if (inst->getName() == string(R"(u0\/u0\/U81)") && instTerm->getTerm()->getName() == string("Y")) {
        //  ;
        //} else {
        //  continue;
        //}
        if (enableOutput) {
          cout <<"pin prep for " <<inst->getName() <<" / " <<instTerm->getTerm()->getName() 
               <<" " <<inst->getRefBlock()->getName() <<" " <<inst->getOrient() <<endl;
        }
        prepPoint_pin(pin.get(), instTerm.get());// 准备每个物理引脚的引脚访问点
      }
      #pragma omp critical // 使用OpenMP的临界区，确保计数器正确更新
      {
        cnt++;
        if (VERBOSE > 0) {
          if (cnt < 1000) {
            if (cnt % 100 == 0) {// 每处理100个引脚输出一次进度
              cout <<"  complete " <<cnt <<" pins" <<endl;
            }
          } else {
            if (cnt % 1000 == 0) {// 处理数量大于1000时，每1000输出一次
              cout <<"  complete " <<cnt <<" pins" <<endl;
            }
          }
        }
      }
    }
  }

  //cout << "PA for IO terms\n" << flush;

  // PA for IO terms// 为设计中顶层块的所有I/O引脚进行处理
  omp_set_num_threads(MAX_THREADS);
  #pragma omp parallel for schedule(dynamic)// 使用动态调度并行处理
  for (unsigned i = 0; i < getDesign()->getTopBlock()->getTerms().size(); i++) {//term number 
    //for (auto &term: getDesign()->getTopBlock()->getTerms()) { // only gcc 9  support this
    auto &term = getDesign()->getTopBlock()->getTerms()[i];//遍历term
    if (term->getType() != frTermEnum::frcNormalTerm && term->getType() != frTermEnum::frcClockTerm) {
      continue;// 如果不是常规或时钟引脚，跳过
    }
    auto net = term->getNet();
    if (net == nullptr) {
      continue;// 如果没有连接的网络，跳过
    }
    // cout << term->getName() << endl;
    for (auto &pin: term->getPins()) {//对每个term的每个pin
      if (enableOutput) {
        cout << "pin prep for PIN/" << term->getName() << " " << endl;
      }
      prepPoint_pin(pin.get(), nullptr);// 准备引脚的引脚访问点--对于一个pin
    }
  }


  if (VERBOSE > 0) {
    cout <<"  complete " <<cnt <<" pins" <<endl;
  }
}
//函数为每个唯一实例准备模式，然后根据实例的位置信息生成实例行，并为每一行生成模式。最后输出完成情况和 GC 调用次数。
void FlexPA::prepPattern() {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl;
  }

  // revert access points to origin
  uniqueInstPatterns.resize(uniqueInstances.size());//？？？

  // int cnt = 0;
  // currUniqueInstIdx = 0;
  int cnt = 0;

  omp_set_num_threads(MAX_THREADS);
  // 并行处理每个唯一实例
  #pragma omp parallel for schedule(dynamic)
  for (int currUniqueInstIdx = 0; currUniqueInstIdx < (int)uniqueInstances.size(); currUniqueInstIdx++) {
    auto &inst = uniqueInstances[currUniqueInstIdx];
    // only do for core and block cells// 仅处理核心和块单元
    if (inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_TIEHIGH && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_TIELOW &&
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_ANTENNACELL) {
      // currUniqueInstIdx++;
      continue;
    }
    // 处理实例的模式
    prepPattern_inst(inst, currUniqueInstIdx);
    // currUniqueInstIdx++;
    #pragma omp critical 
    {// 输出完成情况
      cnt++;
      if (VERBOSE > 0) {
        if (cnt < 1000) {
          if (cnt % 100 == 0) {
            cout <<"  complete " <<cnt <<" unique inst patterns" <<endl;
          }
        } else {
          if (cnt % 1000 == 0) {
            cout <<"  complete " <<cnt <<" unique inst patterns" <<endl;
          }
        }
      }
    }
  }
  if (VERBOSE > 0) {
    cout <<"  complete " <<cnt <<" unique inst patterns" <<endl;
  }

  

  // prep pattern for each row// 生成每行的模式
  std::vector<frInst*> insts;//存储所有实例的列表
  std::vector<std::vector<frInst*> > instRows;//存储实例行的列表
  std::vector<frInst*> rowInsts;//存储当前行的实力列表
  // 按照实例位置排序---自定义函数
  auto instLocComp = [](frInst* const &a, frInst* const &b) {
    frPoint originA, originB;
    a->getOrigin(originA);
    b->getOrigin(originB);
    if (originA.y() == originB.y()) { // 首先按照 y 坐标进行比较
      return (originA.x() < originB.x());// 若 y 坐标相同，则按照 x 坐标升序排列
    } else {
      return (originA.y() < originB.y());
    }
  };

  getInsts(insts);
  std::sort(insts.begin(), insts.end(), instLocComp);// 使用定义的比较函数对实例列表进行排序
 // 生成实例行
  // gen rows of insts
  int prevYCoord = INT_MIN;
  int prevXEndCoord = INT_MIN;
  //if (enableOutput) {
  //  cout << "sorted insts...\n" << flush;
  //}
  frBox instBoundaryBox;
  for (auto inst: insts) {
    frPoint origin;
    inst->getOrigin(origin);
    // cout << inst->getName() << " (" << origin.x() << ", " << origin.y() << ") -- ";
    if (origin.y() != prevYCoord || origin.x() > prevXEndCoord) {
      // cout << "\n";
      if (!rowInsts.empty()) {
        instRows.push_back(rowInsts);//遍历排序后的实例列表，根据实例的y坐标和x坐标生成实例行（instRows）。
        rowInsts.clear();
      }
    }
    rowInsts.push_back(inst);
    prevYCoord = origin.y();// 更新前一个实例的y坐标
    inst->getBoundaryBBox(instBoundaryBox);// 获取实例的边界框
    prevXEndCoord = instBoundaryBox.right();// 更新行的最右端坐标
  }
  if (!rowInsts.empty()) {
    instRows.push_back(rowInsts);
  }
 // 生成每行的实例模式
  if (enableOutput) {
    cout << "gen inst row access patterns...\n" << flush;
  }
  // choose access pattern of a row of insts
  int rowIdx = 0;
  cnt = 0;
  // for (auto &instRow: instRows) {// 使用OpenMP并行处理每行实例，生成访问模式
  omp_set_num_threads(MAX_THREADS);
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < (int)instRows.size(); i++) {
    auto &instRow = instRows[i];
    genInstPattern(instRow);
    #pragma omp critical
    {
      if (enableOutput) {
        cout << "Row " << rowIdx << "\n" << flush;
      }
      rowIdx++;
      cnt++;
      if (VERBOSE > 0) {
        if (cnt < 10000) {
          if (cnt % 1000 == 0) {
            cout <<"  complete " <<cnt <<" groups" <<endl;
          }
        } else {
          if (cnt % 10000 == 0) {
            cout <<"  complete " <<cnt <<" groups" <<endl;
          }
        }
      }
    }
  }
  if (VERBOSE > 0) {
    cout <<"  complete " <<cnt <<" groups" <<endl;
  }

  if (enableOutput) {
    cout << "GC called " << gcCallCnt << " times\n";
  }
}
//用于将所有访问点的位置根据实例的当前转换信息还原为原始布局中的位置。通过将每个访问点的位置应用逆转换，从而恢复到原始实例的位置。
void FlexPA::revertAccessPoints() {
  for (auto &inst: uniqueInstances) {
    frTransform xform, revertXform;
    inst->getTransform(xform);
    revertXform.set(-xform.xOffset(), -xform.yOffset());
    revertXform.set(frcR0);

    auto paIdx = unique2paidx[inst];
    for (auto &instTerm: inst->getInstTerms()) {
      // if (isSkipInstTerm(instTerm.get())) {
      //   continue;
      // }

      for (auto &pin: instTerm->getTerm()->getPins()) {
        auto pinAccess = pin->getPinAccess(paIdx);
        for (auto &accessPoint: pinAccess->getAccessPoints()) {
          frPoint uniqueAPPoint(accessPoint->getPoint());
          uniqueAPPoint.transform(revertXform);
          accessPoint->setPoint(uniqueAPPoint);
        }
      }
    }
  }
}

// calculate which pattern to be used for each inst
// the insts must be in the same row and sorted from left to right

//作用是为一组实例（insts）生成访问模式。
void FlexPA::genInstPattern(std::vector<frInst*> &insts) {
  // bool enableOutput = false;
  if (insts.empty()) {
    return;
  }

  // TODO: change to a global constant
  // TODO: 将maxAccessPatternSize改为全局常量
  // 设置最大访问模式大小的值
  maxAccessPatternSize = 5;
  // 计算节点数量，基于实例数量和最大访问模式大小
  int numNode = (insts.size() + 2) * maxAccessPatternSize;
  // int numEdge = numNode *maxAccessPatternSize;

  std::vector<FlexDPNode> nodes(numNode); // 创建一个FlexDPNode类型的向量，大小为numNode

// 调用初始化函数，准备生成实例模式
  genInstPattern_init(nodes, insts);
  // 执行模式生成过程
  genInstPattern_perform(nodes, insts);
  // 提交模式，完成生成并进行必要的后处理
  genInstPattern_commit(nodes, insts);

}

// init dp node array for valide access patterns
// 作用是在生成实例模式的过程中初始化节点的成本。
void FlexPA::genInstPattern_init(std::vector<FlexDPNode> &nodes,
                                 const std::vector<frInst*> &insts) {
  // bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout << "  genInstPattern_init\n" << flush;
  }

  // init virutal nodes
  // 初始化虚拟节点
  // startNodeIdx表示所有实例之前的状态，即起始节点的一维索引
  int startNodeIdx = getFlatIdx(-1, 0, maxAccessPatternSize);
  int endNodeIdx = getFlatIdx(insts.size(), 0, maxAccessPatternSize);// endNodeIdx表示所有实例之后的状态，即结束节点的一维索引
   // 设置起始节点和结束节点的成本为0
  nodes[startNodeIdx].setNodeCost(0);
  nodes[startNodeIdx].setPathCost(0);
  nodes[endNodeIdx].setNodeCost(0);
  
  // init inst nodes// 初始化实例节点
  // 遍历所有实例
  for (int idx1 = 0; idx1 < (int)insts.size(); idx1++) {
    auto &inst = insts[idx1];// 获取当前实例的引用
    auto &uniqueInst = inst2unique[inst];// 获取当前实例对应的唯一实例的引用
    auto uniqueInstIdx = unique2Idx[uniqueInst];// 获取当前唯一实例的索引
    auto &instPatterns = uniqueInstPatterns[uniqueInstIdx];// 获取当前唯一实例的所有模式
    for (int idx2 = 0; idx2 < (int)instPatterns.size(); idx2++) {// 遍历当前唯一实例的所有模式
      int nodeIdx = getFlatIdx(idx1, idx2, maxAccessPatternSize);// 计算当前模式节点的一维索引
      auto accessPattern = instPatterns[idx2].get();// 获取当前模式
      nodes[nodeIdx].setNodeCost(accessPattern->getCost());// 设置当前节点的成本为当前模式的成本
    }
  }
}
//作用是在生成实例模式的过程中执行路径搜索和成本计算。
void FlexPA::genInstPattern_perform(std::vector<FlexDPNode> &nodes,
                                    const std::vector<frInst*> &insts) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //if (enableOutput) {
  //  cout << "  genInstPattern_perform\n" << flush;
  //}
  
  // 外层循环遍历所有可能的当前节点的idx1（例如，实例索引）
  for (int currIdx1 = 0; currIdx1 <= (int)insts.size(); currIdx1++) {
    bool isSet = false; // 初始化isSet标志为false，用于标记是否设置了有效路径
    if (enableOutput) {
      if (currIdx1 != int(insts.size())) {
        cout << "  genInstPattern_perform " <<insts[currIdx1]->getName() << " uniqueInst " 
             << inst2unique[insts[currIdx1]]->getName() <<endl <<flush;
      }
    }
    // 内层循环遍历所有可能的当前节点的idx2（例如，访问模式索引）
    for (int currIdx2 = 0; currIdx2 < maxAccessPatternSize; currIdx2++) {
      auto currNodeIdx = getFlatIdx(currIdx1, currIdx2, maxAccessPatternSize);// 计算当前节点的一维索引
      auto &currNode = nodes[currNodeIdx];// 获取当前节点的引用
      // 如果当前节点的成本是不可行的（例如，已经被标记为不可达），则跳过当前循环
      if (currNode.getNodeCost() == std::numeric_limits<int>::max()) {
        continue;
      }
      // 初始化prevIdx1为currIdx1的前一个值（在idx1的维度上）
      int prevIdx1 = currIdx1 - 1;
      for (int prevIdx2 = 0; prevIdx2 < maxAccessPatternSize; prevIdx2++) {// 遍历所有可能的前一个节点的idx2
        int prevNodeIdx = getFlatIdx(prevIdx1, prevIdx2, maxAccessPatternSize);
        auto &prevNode = nodes[prevNodeIdx];
        if (prevNode.getPathCost() == std::numeric_limits<int>::max()) {
          continue;
        }
      // 计算从prevNode到currNode的边的成本
        int edgeCost = getEdgeCost(prevNodeIdx, currNodeIdx, nodes, insts);
        // 如果当前节点的路径成本不可行，或者通过prevNode到达currNode的成本更低，则更新currNode的路径成本和前一个节点的索引
        if (currNode.getPathCost() == std::numeric_limits<int>::max() || 
            currNode.getPathCost() > prevNode.getPathCost() + edgeCost) {
          currNode.setPathCost(prevNode.getPathCost() + edgeCost);
          currNode.setPrevNodeIdx(prevNodeIdx);
          isSet = true;
        }
      }
    }
    if (!isSet) {
      cout << "@@@ dead end inst\n";
    }
  }
}

void FlexPA::genInstPattern_commit(std::vector<FlexDPNode> &nodes,
                                   const std::vector<frInst*> &insts) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //bool isDebugMode = true;
  bool isDebugMode = false;
  if (enableOutput) {
    cout << "  genInstPattern_commit\n" << flush;
  }
  int currNodeIdx = getFlatIdx(insts.size(), 0, maxAccessPatternSize);
  auto currNode = &(nodes[currNodeIdx]);
  int instCnt = insts.size();
  std::vector<int> instAccessPatternIdx(insts.size(), -1);
  while (currNode->getPrevNodeIdx() != -1) {
    // non-virtual node
    if (instCnt != (int)insts.size()) {
      int currIdx1, currIdx2;
      getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPatternSize);
      instAccessPatternIdx[currIdx1] = currIdx2;
    
      auto &inst = insts[currIdx1];
      if (enableOutput) {
        cout << inst->getName() << " ";
      }
      int accessPointIdx = 0;
      auto &uniqueInst = inst2unique[inst];
      auto &uniqueInstIdx = unique2Idx[uniqueInst];
      auto accessPattern = uniqueInstPatterns[uniqueInstIdx][currIdx2].get();
      auto &accessPoints = accessPattern->getPattern();
      
      // update instTerm ap
      for (auto &instTerm: inst->getInstTerms()) {
        if (isSkipInstTerm(instTerm.get())) {
          continue;
        }

        int pinIdx = 0;
        //for (auto &pin: instTerm->getTerm()->getPins()) {
        // to avoid unused variable warning in GCC
        for (int i = 0; i < (int)(instTerm->getTerm()->getPins().size()); i++) {
          auto &accessPoint = accessPoints[accessPointIdx];
          instTerm->setAccessPoint(pinIdx, accessPoint);
          pinIdx++;
          accessPointIdx++;
        }


      }

    }
    currNodeIdx = currNode->getPrevNodeIdx();
    currNode = &(nodes[currNode->getPrevNodeIdx()]);
    instCnt--;
  }
  if (enableOutput) {
    cout <<endl;
  }

  if (instCnt != -1) {
    cout << "Error: valid access pattern combination not found\n";
  }

  // for (int i = 0; i < (int)instAccessPatternIdx.size(); i++) {
  //   cout << instAccessPatternIdx[i] << " ";
  // }
  // cout << "\n";

  if (isDebugMode) {
    genInstPattern_print(nodes, insts);
  }


}

void FlexPA::genInstPattern_print(std::vector<FlexDPNode> &nodes,
                                 const std::vector<frInst*> &insts) {
  int currNodeIdx = getFlatIdx(insts.size(), 0, maxAccessPatternSize);
  auto currNode = &(nodes[currNodeIdx]);
  int instCnt = insts.size();
  std::vector<int> instAccessPatternIdx(insts.size(), -1);

  map<frOrientEnum, string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};

  while (currNode->getPrevNodeIdx() != -1) {
    // non-virtual node
    if (instCnt != (int)insts.size()) {
      int currIdx1, currIdx2;
      getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPatternSize);
      instAccessPatternIdx[currIdx1] = currIdx2;

      // print debug information
      auto &inst = insts[currIdx1];
      int accessPointIdx = 0;
      auto &uniqueInst = inst2unique[inst];
      auto &uniqueInstIdx = unique2Idx[uniqueInst];
      auto accessPattern = uniqueInstPatterns[uniqueInstIdx][currIdx2].get();
      auto &accessPoints = accessPattern->getPattern();

      for (auto &instTerm: inst->getInstTerms()) {
        if (isSkipInstTerm(instTerm.get())) {
          continue;
        }

        //for (auto &pin: instTerm->getTerm()->getPins()) {
        // to avoid unused variable warning in GCC
        for (int i = 0; i < (int)(instTerm->getTerm()->getPins().size()); i++) {
          auto &accessPoint = accessPoints[accessPointIdx];
          
          frPoint pt(accessPoint->getPoint());
          if (instTerm->hasNet()) {
            cout << " gcclean2via " << inst->getName() << " " 
                 << instTerm->getTerm()->getName() << " " << accessPoint->getViaDef()->getName() << " "
                 << pt.x() << " " << pt.y() << " "
                 << orient2Name[inst->getOrient()] << "\n";
            instTermValidViaApCnt++;
            // cout << instTermValidViaApCnt << endl;
          }
          accessPointIdx++;
        }
      }
    }
    currNodeIdx = currNode->getPrevNodeIdx();
    currNode = &(nodes[currNode->getPrevNodeIdx()]);
    instCnt--;
  }

  cout << flush;

  if (instCnt != -1) {
    cout << "Error: valid access pattern combination not found\n";
  }
}

int FlexPA::getEdgeCost(int prevNodeIdx, int currNodeIdx, 
                        std::vector<FlexDPNode> &nodes,
                        const std::vector<frInst*> &insts) {
  bool enableOutput = false;
  if (enableOutput) {
    cout << "  getEdgeCost\n";
  }
  int edgeCost = 0;
  int prevIdx1, prevIdx2, currIdx1, currIdx2;
  getNestedIdx(prevNodeIdx, prevIdx1, prevIdx2, maxAccessPatternSize);
  getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPatternSize);
  if (prevIdx1 == -1 || currIdx1 == (int)insts.size()) {
    return edgeCost;
  }

  bool hasVio = false;

  // check DRC
  std::vector<std::unique_ptr<frVia> > tempVias;
  std::vector<std::pair<frConnFig*, frBlockObject*> > objs;
  // push the vias from prev inst access pattern and curr inst access pattern
  auto prevInst = insts[prevIdx1];
  auto prevUniqueInstIdx = unique2Idx[inst2unique[prevInst]];
  auto currInst = insts[currIdx1];
  auto currUniqueInstIdx = unique2Idx[inst2unique[currInst]];
  auto prevPinAccessPattern = uniqueInstPatterns[prevUniqueInstIdx][prevIdx2].get();
  auto currPinAccessPattern = uniqueInstPatterns[currUniqueInstIdx][currIdx2].get();
  addAccessPatternObj(prevInst, prevPinAccessPattern, objs, tempVias, true);
  addAccessPatternObj(currInst, currPinAccessPattern, objs, tempVias, false);

  hasVio = !genPatterns_gc(nullptr, objs);
  if (!hasVio) {
    int prevNodeCost = nodes[prevNodeIdx].getNodeCost();
    int currNodeCost = nodes[currNodeIdx].getNodeCost();
    edgeCost = (prevNodeCost + currNodeCost) / 2;
  } else {
    edgeCost = 1000;
  }

  return edgeCost;



}

void FlexPA::addAccessPatternObj(frInst* inst,
                                 FlexPinAccessPattern *accessPattern,
                                 std::vector<std::pair<frConnFig*, frBlockObject*> > &objs,
                                 std::vector<std::unique_ptr<frVia> > &vias, bool isPrev) {
  frTransform xform;
  inst->getUpdatedXform(xform, true);
  int accessPointIdx = 0;
  auto &accessPoints = accessPattern->getPattern();

  for (auto &instTerm: inst->getInstTerms()) {
    if (isSkipInstTerm(instTerm.get())) {
      continue;
    }

    //for (auto &pin: instTerm->getTerm()->getPins()) {
    // to avoid unused variable warning in GCC
    for (int i = 0; i < (int)(instTerm->getTerm()->getPins().size()); i++) {
      auto &accessPoint = accessPoints[accessPointIdx];
      //cout <<"@@@test: isPrev = " <<isPrev <<", apoint " 
      //     <<accessPoint <<", leftAPoint " 
      //     <<accessPattern->getBoundaryAP(true) <<", rightAPoint " 
      //     <<accessPattern->getBoundaryAP(false) <<endl <<flush;
      if (isPrev && accessPoint != accessPattern->getBoundaryAP(false)) {
        accessPointIdx++;
        continue;
      }
      if ((!isPrev) && accessPoint != accessPattern->getBoundaryAP(true)) {
        accessPointIdx++;
        continue;
      }
      //cout <<"@@@find something" <<endl;
      if (accessPoint->hasAccess(frDirEnum::U)) {
        std::unique_ptr<frVia> via = std::make_unique<frVia>(accessPoint->getViaDef());
        frPoint pt(accessPoint->getPoint());
        pt.transform(xform);
        via->setOrigin(pt);
        auto rvia = via.get();
        if (instTerm->hasNet()) {
          objs.push_back(std::make_pair(rvia, instTerm->getNet()));
        } else {
          objs.push_back(std::make_pair(rvia, instTerm.get()));
        }
        vias.push_back(std::move(via));
      }
      accessPointIdx++;
    }
  }
}
//它的作用是从设计中提取特定类型的实例（frInst），并将它们添加到提供的向量insts中
void FlexPA::getInsts(std::vector<frInst*> &insts) {
  for (auto &inst: design->getTopBlock()->getInsts()) {
    // 检查实例引用的块是否属于特定的宏类
    // 如果不属于CORE、CORE_TIEHIGH、CORE_TIELOW或CORE_ANTENNACELL，则跳过该实例
    if (inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_TIEHIGH && 
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_TIELOW &&
        inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE_ANTENNACELL) {
      continue;
    }
    bool isSkip = true;// 初始化跳过标志为true
    for (auto &instTerm: inst->getInstTerms()) {// 遍历实例的所有实例终端
      if (!isSkipInstTerm(instTerm.get())) {// 检查实例终端是否应该被跳过
        isSkip = false;// 如果有任何一个实例终端不应该被跳过，则改变跳过标志为false
        break;// 找到第一个非跳过的实例终端后，跳出内层循环
      }
    }
    if (!isSkip) {// 如果存在至少一个非跳过的实例终端，则将实例添加到输出向量中
      insts.push_back(inst.get());
    }
  }
}
//决定是否应该跳过某个实例化的引脚（instTerm）的处理。
bool FlexPA::isSkipInstTerm(frInstTerm *in) { // 该函数接受一个指向frInstTerm对象的指针作为参数，并返回一个布尔值。
  if (in->getTerm()->getType() != frTermEnum::frcNormalTerm &&
      in->getTerm()->getType() != frTermEnum::frcClockTerm) {
    // 检查传入的实例化引脚的类型。
    // frInstTerm对象通过getTerm方法获取关联的frTerm对象，再通过getType方法获取该引脚的类型。
    // 如果引脚的类型既不是 frcNormalTerm（常规类型引脚）也不是 frcClockTerm（时钟类型引脚）：
    return true;
  } else {
    return false;
  }
}
// 首先遍历给定实例的每个实例术语，并对每个实例术语的每个针脚进行处理。
// 对于每个针脚，它计算其访问点的平均 x 坐标，然后将针脚及其相关信息添加到针脚列表中。
// 接着，对针脚列表按照 x 坐标进行排序，并将排序后的针脚列表传递给 genPatterns 函数以生成模式。
// the input inst must be unique instance  
  //bool enableOutput = true;
void FlexPA::prepPattern_inst(frInst *inst, int currUniqueInstIdx) {
  bool enableOutput = false;
  std::vector<std::pair<frCoord, std::pair<frPin*, frInstTerm*> > > pins;
  if (enableOutput) {
    cout << "start prepPattern " << inst->getName() << "\n" << flush;
  }
  // TODO: add assert in case input inst is not unique inst
  // 获取当前实例在unique2paidx中的索引
  int paIdx = unique2paidx[inst];
   // 遍历实例的各个instTerm
  for (auto &instTerm: inst->getInstTerms()) {
    if (isSkipInstTerm(instTerm.get())) {// 如果当前的instTerm应该被跳过，则继续下一次循环
      continue;
    }
 // 遍历instTerm中的各个pin
    for (auto &pin: instTerm->getTerm()->getPins()) {
      // container of access points // 获取当前pin的访问点集合
      auto pinAccess = pin->getPinAccess(paIdx);
      int sumXCoord = 0;// 初始化坐标和计数器
      int sumYCoord = 0;
      int cnt = 0;
      // get avg x coord for sort// 遍历访问点，计算所有访问点的平均x坐标和y坐标
      for (auto &accessPoint: pinAccess->getAccessPoints()) {
        sumXCoord += accessPoint->getPoint().x();
        sumYCoord += accessPoint->getPoint().y();
        cnt++;
      }
      if (cnt != 0) {// 如果访问点数量不为0，则将pin添加到pins中，使用计算出的平均x坐标
        // pins.push_back(std::make_pair((sumXCoord + 0.5 * sumYCoord) / cnt, std::make_pair(pin.get(), instTerm.get())));
        // pins.push_back(std::make_pair(sumXCoord / cnt, std::make_pair(pin.get(), instTerm.get())));
      } else {
        pins.push_back(std::make_pair((sumXCoord + 0.0 * sumYCoord) / cnt, std::make_pair(pin.get(), instTerm.get())));
        std::cout << "Error: pin does not have access point\n"; // 如果pin没有访问点，输出错误信息
      }
    }
  }
  std::sort(pins.begin(), pins.end(), [](const std::pair<frCoord, std::pair<frPin*, frInstTerm*> > &lhs, 
                                         const std::pair<frCoord, std::pair<frPin*, frInstTerm*> > &rhs) 
                                        {return lhs.first < rhs.first;} );// 使用lambda表达式对pins进行排序，根据平均x坐标
  if (enableOutput) {// 如果enableOutput为true，则输出排序后的pins信息
    cout <<inst->getName() <<":";
    for (auto &[x, m]: pins) {
      auto &[pin, instTerm] = m;
      cout << " " <<pin->getTerm()->getName() << "(" << pin->getPinAccess(paIdx)->getNumAccessPoints() << ")";
    }
    cout << endl;
  }

  std::vector<std::pair<frPin*, frInstTerm*> > pinInstTermPairs; // 创建一个vector用于存储排序后的pin和instTerm的pair
  for (auto &[x, m]: pins) {
    pinInstTermPairs.push_back(m);
  }

  genPatterns(pinInstTermPairs, currUniqueInstIdx); // 调用genPatterns函数，传入排序后的pin和instTerm的pair以及当前实例的索引
}
//生成引脚访问模式
void FlexPA::genPatterns(const std::vector<std::pair<frPin*, frInstTerm*> > &pins, int currUniqueInstIdx) {
  bool enableOutput = false;
  // bool enableOutput = true;

  if (pins.empty()) { // 如果传入的引脚集合为空，则直接返回
    return;
  }

  int maxAccessPointSize = 0; // 初始化最大访问点数量为0
  int paIdx = unique2paidx[pins[0].second->getInst()]; // 获取第一个引脚对应的实例索引
  for (auto &[pin, instTerm]: pins) {// 遍历所有引脚，找到具有最多访问点的引脚，赋值最大访问点数给maxAccessPointSize
    maxAccessPointSize = std::max(maxAccessPointSize, pin->getPinAccess(paIdx)->getNumAccessPoints());
  }
  int numNode = (pins.size() + 2) * maxAccessPointSize;// 计算节点和边的数量 maxAccessPointSize是其中最大的一个point数量
  int numEdge = numNode * maxAccessPointSize;//
// 创建节点和边的向量
  std::vector<FlexDPNode> nodes(numNode);//要看最大元素个数
  std::vector<int> vioEdge(numEdge, -1);
  // moved for mt
  // 创建存储实例访问模式的集合
  std::set<std::vector<int> > instAccessPatterns;
  // 创建存储已使用访问点的集合
  std::set<std::pair<int, int> > usedAccessPoints;
  // 创建存储违规访问点的集合
  std::set<std::pair<int, int> > violAccessPoints;
  if (enableOutput) {
    auto &[pin, instTerm] = pins[0];
    auto inst = instTerm->getInst();
    cout << "generate access pattern for uniqueInst" << currUniqueInstIdx << " (" << inst->getName() << ")\n";
  }
  // 初始化生成模式的过程
  genPatterns_init(nodes, pins, instAccessPatterns, usedAccessPoints, violAccessPoints, maxAccessPointSize);
  int numValidPattern = 0;// 初始化有效模式的数量
  for (int i = 0; i < ACCESS_PATTERN_END_ITERATION_NUM; i++) { // 循环尝试生成模式，直到达到预设的迭代次数
    genPatterns_reset(nodes, pins, maxAccessPointSize);// 重置节点状态
    // 执行模式生成过程
    genPatterns_perform(nodes, pins, vioEdge, usedAccessPoints, violAccessPoints, currUniqueInstIdx, maxAccessPointSize);
    bool isValid = false;// 检查是否生成了有效的模式
    if (genPatterns_commit(nodes, pins, isValid, instAccessPatterns, usedAccessPoints, violAccessPoints, currUniqueInstIdx, maxAccessPointSize)) {
      if (isValid) {
        numValidPattern++;
      } else {
        if (enableOutput) {
          genPatterns_print_debug(nodes, pins, maxAccessPointSize);
        }
      }
    } else {
      break;
    }
  }

  // try reverse order if no valid pattern // 如果没有生成有效的模式，尝试反转引脚顺序后再次尝试
  if (numValidPattern == 0) {
    auto reversedPins = pins;
    reverse(reversedPins.begin(), reversedPins.end());

    std::vector<FlexDPNode> nodes(numNode);
    std::vector<int> vioEdge(numEdge, -1);

    genPatterns_init(nodes, reversedPins, instAccessPatterns, usedAccessPoints, violAccessPoints, maxAccessPointSize);
    for (int i = 0; i < ACCESS_PATTERN_END_ITERATION_NUM; i++) {
      genPatterns_reset(nodes, reversedPins, maxAccessPointSize);
      genPatterns_perform(nodes, reversedPins, vioEdge, usedAccessPoints, violAccessPoints, currUniqueInstIdx, maxAccessPointSize);
      bool isValid = false;
      if (genPatterns_commit(nodes, reversedPins, isValid, instAccessPatterns, usedAccessPoints, violAccessPoints, currUniqueInstIdx, maxAccessPointSize)) {
        if (isValid) {
          numValidPattern++;
        } else {
          if (enableOutput) {
            genPatterns_print_debug(nodes, reversedPins, maxAccessPointSize);
          }
        }
      } else {
        break;
      }
    }
  }

  if (numValidPattern == 0) { // 如果最终没有生成有效的模式，则输出错误信息
    auto inst = pins[0].second->getInst();// 获取第一个引脚对应的实例
    cout << "Error: no valid pattern for unique instance " 
         << inst->getName() <<", refBlock is " 
         << inst->getRefBlock()->getName() <<endl;
    //int paIdx = unique2paidx[pins[0].second->getInst()];
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    frTransform shiftXform;
    inst->getTransform(shiftXform);
    shiftXform.set(frOrient(frcR0));
    cout <<"  pin ordering (with ap): " <<endl;
    for (auto &[pin, instTerm]: pins) {
      cout <<"    " <<instTerm->getTerm()->getName();
      for (auto &ap: pin->getPinAccess(paIdx)->getAccessPoints()) {
        frPoint pt;
        ap->getPoint(pt);
        //auto bNum = ap->getLayerNum();
        //auto bLayer = getDesign()->getTech()->getLayer(bNum);
        pt.transform(shiftXform);
        cout <<" (" <<pt.x() / dbu <<", " <<pt.y() / dbu <<")";
      }
      cout <<endl;
    }
    //cout << "Error: no valid pattern for unique instance " ;
  }

}

// init dp node array for valid access points初始化生成模式的过程。
void FlexPA::genPatterns_init(std::vector<FlexDPNode> &nodes,//
                              const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                              std::set<std::vector<int> > &instAccessPatterns,
                              std::set<std::pair<int, int> > &usedAccessPoints,
                              std::set<std::pair<int, int> > &violAccessPoints,
                              int maxAccessPointSize) {
  bool enableOutput = false;
  if (enableOutput) {
    cout << "  init\n" << flush;
  }
  // clear temp storage and flag// 清除临时存储和标志，为初始化做准备
  instAccessPatterns.clear();
  usedAccessPoints.clear();
  violAccessPoints.clear();

  // init virtual nodes
  // 初始化虚拟节点
  // startNodeIdx是起始节点的索引，表示所有引脚的第一个访问点之前的状态
  int startNodeIdx = getFlatIdx(-1, 0, maxAccessPointSize);// return ((idx1 + 1) * idx2Dim + idx2);
  int endNodeIdx = getFlatIdx(pins.size(), 0, maxAccessPointSize);// endNodeIdx是结束节点的索引，表示所有引脚的所有访问点之后的状态
  nodes[startNodeIdx].setNodeCost(0); // 设置起始节点和结束节点的成本为0
  nodes[startNodeIdx].setPathCost(0);
  nodes[endNodeIdx].setNodeCost(0);
  // init pin nodes// 初始化引脚节点
  int pinIdx = 0;//用于遍历引脚
  int apIdx = 0;//用于遍历一个引脚的访问点
  int paIdx = unique2paidx[pins[0].second->getInst()];//获取第一个引脚对应的实例索引

  for (auto &[pin, instTerm]: pins) {//遍历所有引脚
    apIdx = 0;// 对于当前引脚，重置访问点索引
    for (auto &ap: pin->getPinAccess(paIdx)->getAccessPoints()) { // 遍历当前引脚的所有访问点
      int nodeIdx = getFlatIdx(pinIdx, apIdx, maxAccessPointSize);// 计算当前访问点对应的节点索引--换算成nodeIdx的索引
      nodes[nodeIdx].setNodeCost(ap->getCost()); // 设置该节点的成本为访问点的成本
      apIdx++;//移动到下一个访问点
    }
    pinIdx++;//移动到下一个引脚
  }
}
//在生成模式的过程中重置节点的状态，为下一次模式搜索做准备。
void FlexPA::genPatterns_reset(std::vector<FlexDPNode> &nodes,
                               const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                               int maxAccessPointSize) {
  for (int i = 0; i < (int)nodes.size(); i++) { // 遍历所有节点
    auto node = &nodes[i];// 获取当前节点的指针
    node->setPathCost(std::numeric_limits<int>::max());// 将当前节点的路径成本设置为一个非常大的数，表示当前路径是不可达的
    node->setPrevNodeIdx(-1);// 将当前节点的前一个节点索引设置为-1，表示当前节点没有前驱节点
  }
//// 计算起始节点和结束节点的一维索引
  int startNodeIdx = getFlatIdx(-1, 0, maxAccessPointSize);
  int endNodeIdx = getFlatIdx(pins.size(), 0, maxAccessPointSize);
  nodes[startNodeIdx].setNodeCost(0);// 设置起始节点的成本为0，因为从起始节点到自身的成本是0
  nodes[startNodeIdx].setPathCost(0);// 设置起始节点的路径成本为0，因为从起始节点到自身的路径成本是0
  nodes[endNodeIdx].setNodeCost(0);// 设置结束节点的成本为0，因为到达结束节点意味着找到了一个完整的路径
}

// objs must hold at least 1 obj
// 它的作用是对给定的对象集合执行全局布线器（Global Router，简称GC）的DRC（Design Rule Check）检查。
bool FlexPA::genPatterns_gc(frBlockObject* targetObj, vector<pair<frConnFig*, frBlockObject*> > &objs, 
                            std::set<frBlockObject*> *owners) {
  gcCallCnt++;
  if (objs.empty()) {
    if (VERBOSE > 1) {
      cout <<"Warning: genPattern_gc objs empty" <<endl;
    }
    return false;
  }

  FlexGCWorker gcWorker(getDesign());
  gcWorker.setIgnoreMinArea();
  
  frCoord llx = std::numeric_limits<frCoord>::max();
  frCoord lly = std::numeric_limits<frCoord>::max();
  frCoord urx = std::numeric_limits<frCoord>::min();
  frCoord ury = std::numeric_limits<frCoord>::min();
  frBox bbox;
  for (auto &[connFig, owner]: objs) {
    connFig->getBBox(bbox);
    llx = std::min(llx, bbox.left());
    lly = std::min(llx, bbox.bottom());
    urx = std::max(llx, bbox.right());
    ury = std::max(llx, bbox.top());
  }
  frBox extBox(llx - 3000, lly - 3000, urx + 3000, ury + 3000);
  // frBox extBox(llx - 1000, lly - 1000, urx + 1000, ury + 1000);
  gcWorker.setExtBox(extBox);
  gcWorker.setDrcBox(extBox);

  gcWorker.setTargetObj(targetObj);
  gcWorker.setIgnoreDB();
  //cout <<flush;
  gcWorker.initPA0();
  for (auto &[connFig, owner]: objs) {
    gcWorker.addPAObj(connFig, owner);
  }
  gcWorker.initPA1();
  gcWorker.main();
  gcWorker.end();

  //int typeGC  = 0;
  bool sol = false;
  if (gcWorker.getMarkers().empty()) {
    //typeGC = 1;
    sol = true;
  } else {
    //typeGC = 2;
  }
  //if (enableOutput) {
    //prepPoint_pin_checkPoint_print_helper(ap, pin, instTerm, frDirEnum::U, typeGC, typeDRC, bp, ep, via->getViaDef());
  //}
  if (owners) {
    for (auto &marker: gcWorker.getMarkers()) {
      for (auto &src: marker->getSrcs()) {
        owners->insert(src);
      }
    }
  }
  return sol;
}
//作用是在生成模式的过程中执行路径搜索和成本计算。
void FlexPA::genPatterns_perform(std::vector<FlexDPNode> &nodes,
                                 const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                                 std::vector<int> &vioEdges,
                                 std::set<std::pair<int, int> > &usedAccessPoints,
                                 std::set<std::pair<int, int> > &violAccessPoints,
                                 int currUniqueInstIdx,
                                 int maxAccessPointSize) {
  bool enableOutput = false;
  if (enableOutput) {
    cout << "  perform\n" << flush;
  }
  for (int currIdx1 = 0; currIdx1 <= (int)pins.size(); currIdx1++) {// 外层循环遍历所有可能的当前节点的idx1（例如，引脚索引）
    for (int currIdx2 = 0; currIdx2 < maxAccessPointSize; currIdx2++) {// 内层循环遍历所有可能的当前节点的idx2（例如，访问点索引）
      auto currNodeIdx = getFlatIdx(currIdx1, currIdx2, maxAccessPointSize);// 计算当前节点的一维索引
      auto &currNode = nodes[currNodeIdx]; // 获取当前节点的引用
      if (currNode.getNodeCost() == std::numeric_limits<int>::max()) {// 如果当前节点的成本是不可行的（例如，已经被标记为不可达），则跳过当前循环
        continue;
      }
      int prevIdx1 = currIdx1 - 1;// 初始化prevIdx1为currIdx1的前一个值（在idx1的维度上）引脚维度
      for (int prevIdx2 = 0; prevIdx2 < maxAccessPointSize; prevIdx2++) {// 遍历所有可能的前一个节点的idx2访问点维度
        int prevNodeIdx = getFlatIdx(prevIdx1, prevIdx2, maxAccessPointSize);// 计算前一个节点的一维索引
        auto &prevNode = nodes[prevNodeIdx];// 获取前一个节点的引用
        // 如果前一个节点的路径成本是不可行的，则跳过当前循环
        if (prevNode.getPathCost() == std::numeric_limits<int>::max()) {
          continue;
        }
        // 计算从prevNode到currNode的边的成本
        int edgeCost = getEdgeCost(prevNodeIdx, currNodeIdx, nodes, pins, vioEdges, usedAccessPoints, violAccessPoints, currUniqueInstIdx, maxAccessPointSize);
        // 如果当前节点的路径成本不可行，或者通过prevNode到达currNode的成本更低，则更新currNode的路径成本和前一个节点的索引
        if (currNode.getPathCost() == std::numeric_limits<int>::max() ||
            currNode.getPathCost() > prevNode.getPathCost() + edgeCost) {
          currNode.setPathCost(prevNode.getPathCost() + edgeCost);//连接两个访问点--构建图的过程
          currNode.setPrevNodeIdx(prevNodeIdx);
        }
      }
    }
  }
}

int FlexPA::getEdgeCost(int prevNodeIdx, 
                        int currNodeIdx,
                        const std::vector<FlexDPNode> &nodes,
                        const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                        std::vector<int> &vioEdges,
                        std::set<std::pair<int, int> > &usedAccessPoints,
                        std::set<std::pair<int, int> > &violAccessPoints,
                        int currUniqueInstIdx,
                        int maxAccessPointSize) {
  int edgeCost = 0;
  int prevIdx1, prevIdx2, currIdx1, currIdx2;
  getNestedIdx(prevNodeIdx, prevIdx1, prevIdx2, maxAccessPointSize);
  getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPointSize);
  if (prevIdx1 == -1 || currIdx1 == (int)pins.size()) {
    return edgeCost;
  }

  bool hasVio = false;
  // check if the edge has been calculated
  int edgeIdx = getFlatEdgeIdx(prevIdx1, prevIdx2, currIdx2, maxAccessPointSize);
  if (vioEdges[edgeIdx] != -1) {
    hasVio = (vioEdges[edgeIdx] == 1);
  } else {
    auto &currUniqueInst = uniqueInstances[currUniqueInstIdx];
    frTransform xform;
    currUniqueInst->getUpdatedXform(xform, true);
    // check DRC
    vector<pair<frConnFig*, frBlockObject*> > objs;
    auto &[pin1, instTerm1] = pins[prevIdx1];
    auto targetObj = instTerm1->getInst();
    int paIdx = unique2paidx[targetObj];
    auto pa1 = pin1->getPinAccess(paIdx);
    // unique_ptr<frVia> via1 = make_unique<frVia>(pa1->getAccessPoint(prevIdx2)->getViaDef());
    unique_ptr<frVia> via1;
    if (pa1->getAccessPoint(prevIdx2)->hasAccess(frDirEnum::U)) {
      via1 = make_unique<frVia>(pa1->getAccessPoint(prevIdx2)->getViaDef());
      frPoint pt1(pa1->getAccessPoint(prevIdx2)->getPoint());
      pt1.transform(xform);
      via1->setOrigin(pt1);
      if (instTerm1->hasNet()) {
        objs.push_back(make_pair(via1.get(), instTerm1->getNet()));
      } else {
        objs.push_back(make_pair(via1.get(), instTerm1));
      }
    }

    auto &[pin2, instTerm2] = pins[currIdx1];
    auto pa2 = pin2->getPinAccess(paIdx);
    // unique_ptr<frVia> via2 = make_unique<frVia>(pa2->getAccessPoint(currIdx2)->getViaDef());
    unique_ptr<frVia> via2;
    if (pa2->getAccessPoint(currIdx2)->hasAccess(frDirEnum::U)) {
      via2 = make_unique<frVia>(pa2->getAccessPoint(currIdx2)->getViaDef());
      frPoint pt2(pa2->getAccessPoint(currIdx2)->getPoint());
      pt2.transform(xform);
      via2->setOrigin(pt2);
      if (instTerm2->hasNet()) {
        objs.push_back(make_pair(via2.get(), instTerm2->getNet()));
      } else {
        objs.push_back(make_pair(via2.get(), instTerm2));
      }
    }

    hasVio = !genPatterns_gc(targetObj, objs);
    vioEdges[edgeIdx] = hasVio;

    // look back for GN14
    if (!hasVio && prevNodeIdx != -1) {
      // check one more back
      auto prevPrevNodeIdx = nodes[prevNodeIdx].getPrevNodeIdx();
      if (prevPrevNodeIdx != -1) {
        int prevPrevIdx1, prevPrevIdx2;
        getNestedIdx(prevPrevNodeIdx, prevPrevIdx1, prevPrevIdx2, maxAccessPointSize);
        if (prevPrevIdx1 != -1) {
          auto &[pin3, instTerm3] = pins[prevPrevIdx1];
          auto pa3 = pin3->getPinAccess(paIdx);
          // unique_ptr<frVia> via3 = make_unique<frVia>(pa3->getAccessPoint(prevPrevIdx2)->getViaDef());
          unique_ptr<frVia> via3;
          if (pa3->getAccessPoint(prevPrevIdx2)->hasAccess(frDirEnum::U)) {
            via3 = make_unique<frVia>(pa3->getAccessPoint(prevPrevIdx2)->getViaDef());
            frPoint pt3(pa3->getAccessPoint(prevPrevIdx2)->getPoint());
            pt3.transform(xform);
            via3->setOrigin(pt3);
            if (instTerm3->hasNet()) {
              objs.push_back(make_pair(via3.get(), instTerm3->getNet()));
            } else {
              objs.push_back(make_pair(via3.get(), instTerm3));
            }
          }

          hasVio = !genPatterns_gc(targetObj, objs);
        }
      }
    }
  }

  if (!hasVio) {
  // if (true) {
    // edgeCost = std::abs(pt1.x() - pt2.x()) + std::abs(pt1.y() + pt2.y());
    // add penalty to cost if either boundary pin access points has been used before
    if ((prevIdx1 == 0 && usedAccessPoints.find(std::make_pair(prevIdx1, prevIdx2)) != usedAccessPoints.end()) || 
        (currIdx1 == (int)pins.size() - 1 && usedAccessPoints.find(std::make_pair(currIdx1, currIdx2)) != usedAccessPoints.end())) {
      edgeCost = 100;
    } else if (violAccessPoints.find(make_pair(prevIdx1, prevIdx2)) != violAccessPoints.end() || 
               violAccessPoints.find(make_pair(currIdx1, currIdx2)) != violAccessPoints.end()) {
      edgeCost = 1000;
    } else {
      int prevNodeCost = nodes[prevNodeIdx].getNodeCost();
      int currNodeCost = nodes[currNodeIdx].getNodeCost();
      edgeCost = (prevNodeCost + currNodeCost) / 2;  
    }
  } else {
    edgeCost = 1000/*violation cost*/;
  }

  return edgeCost;
}
// 作用是在生成模式的过程中提交并验证找到的路径，同时检查设计规则校验（DRC）。检查是否生成了有效的访问模式
bool FlexPA::genPatterns_commit(std::vector<FlexDPNode> &nodes,
                                const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                                bool &isValid,
                                std::set<std::vector<int> > &instAccessPatterns,
                                std::set<std::pair<int, int> > &usedAccessPoints,
                                std::set<std::pair<int, int> > &violAccessPoints,
                                int currUniqueInstIdx,
                                int maxAccessPointSize) {
  bool hasNewPattern = false;// 初始化是否找到新模式的标记为false
  bool enableOutput = false;
  //bool enableOutput = true;
  if (enableOutput) {
    cout << "  commit\n" << flush;
  }
  int currNodeIdx = getFlatIdx(pins.size(), 0, maxAccessPointSize);// 计算结束节点的一维索引
  auto currNode = &(nodes[currNodeIdx]);// 获取结束节点的引用---nodes是一维的
  int pinCnt = pins.size();// 获取引脚的数量
  std::vector<int> accessPattern(pinCnt, -1);// 初始化访问模式向量，所有元素初始值设为-1
  while (currNode->getPrevNodeIdx() != -1) {// 从结束节点开始，逆向遍历路径直到起始节点
    // non-virtual node// 如果当前节点不是虚拟节点（即属于某个引脚的访问点）
    if (pinCnt != (int)pins.size()) { 
      int currIdx1, currIdx2;
      //一维索引转多维
      getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPointSize);//idx1 = flatIdx / idx2Dim - 1; idx2 = flatIdx % idx2Dim;
      accessPattern[currIdx1] = currIdx2;// 更新访问模式向量
      usedAccessPoints.insert(std::make_pair(currIdx1, currIdx2));// 将当前访问点标记为已使用
    }

    currNodeIdx = currNode->getPrevNodeIdx(); // 移动到前一个节点
    currNode = &(nodes[currNode->getPrevNodeIdx()]);
    pinCnt--;// 减少引脚计数
  }
 // 如果遍历结束后pinCnt不等于-1，说明没有找到有效的访问模式
  if (pinCnt != -1) {
    cout << "Error: valid access pattern not found\n";
  }

  // if (enableOutput) {
  //   for (auto &apIdx: accessPattern) {
  //     cout << " " << apIdx;
  //   }
  //   cout << "\n";
  // }
  // add to pattern set if unique
  if (instAccessPatterns.find(accessPattern) == instAccessPatterns.end()) {// 如果找到的访问模式是唯一的，则添加到实例访问模式集合中
    instAccessPatterns.insert(accessPattern);// 添加访问模式 新模式
    // create new access pattern and push to uniqueInstances
    // 创建一个新的引脚访问模式对象，用于存储和管理当前的引脚访问模式
    auto pinAccessPattern = std::make_unique<FlexPinAccessPattern>();
    std::map<frPin*, frAccessPoint*> pin2AP; // 初始化一个映射，用于存储frPin和frAccessPoint之间的关联
    // check DRC for the whole pattern // 对整个模式进行DRC检查
    // 初始化objs向量和tempVias向量，用于存储连接图形对象和临时vias
    vector<pair<frConnFig*, frBlockObject*> > objs;
    vector<unique_ptr<frVia> > tempVias;
    frInst *targetObj = nullptr;// 初始化targetObj为nullptr，它将被用来指向当前处理的实例对象
    for (int idx1 = 0; idx1 < (int)pins.size(); idx1++) {// 遍历所有引脚和访问点，构建访问模式   -- 引脚数
      auto idx2 = accessPattern[idx1];// 从访问模式向量中获取当前引脚对应的访问点索引
      auto &[pin, instTerm] = pins[idx1];// 获取当前引脚和实例终端的引用
      auto inst = instTerm->getInst(); // 获取实例终端对应的实例对象
      targetObj = inst;// 更新targetObj为当前实例对象
      int paIdx = unique2paidx[inst];// 获取实例的唯一索引
      auto pa = pin->getPinAccess(paIdx);// 获取引脚的访问点集合
      auto accessPoint = pa->getAccessPoint(idx2);// 获取特定的访问点
      pin2AP[pin] = accessPoint;// 在映射中关联引脚和访问点
      // pinAccessPattern->addAccessPoint(accessPoint);

      // add objs
      unique_ptr<frVia> via;//处理通孔（Via）
      // 如果访问点具有向上的访问权限，则创建一个通孔（Via）
      if (accessPoint->hasAccess(frDirEnum::U)) {
        via = make_unique<frVia>(accessPoint->getViaDef()); // 创建一个新的通孔对象
        auto rvia = via.get();// 获取创建的通孔的裸指针
        tempVias.push_back(std::move(via));// 将通孔对象添加到临时向量中

        frTransform xform; // 获取实例的变换矩阵
        inst->getUpdatedXform(xform, true);
        // 获取访问点的位置
        frPoint pt(accessPoint->getPoint());
        // 应用变换矩阵到点上，以获取在全局坐标系中的位置
        pt.transform(xform);
        // 设置通孔的原始位置
        rvia->setOrigin(pt);
        // 检查实例终端是否有关联的网络
        if (instTerm->hasNet()) {
          objs.push_back(make_pair(rvia, instTerm->getNet()));// 如果有网络，将通孔和网络添加为一对到objs向量中
        } else {
          objs.push_back(make_pair(rvia, instTerm));
        }
      }
    }
    //初始化左右边界访问点为nullptr，并设置左右边界的坐标值
    frAccessPoint* leftAP = nullptr;
    frAccessPoint* rightAP = nullptr;
    frCoord leftPt  = std::numeric_limits<frCoord>::max();// 初始化左边界为坐标的最大值
    frCoord rightPt = std::numeric_limits<frCoord>::min();// 初始化右边界为坐标的最小值
    frPoint tmpPt;
    //从第一个引脚开始，遍历实例的所有实例终端和引脚
    auto &[pin, instTerm] = pins[0];
    auto inst = instTerm->getInst();
    for (auto &instTerm: inst->getInstTerms()) {
      if (isSkipInstTerm(instTerm.get())) {
        continue;// 如果是跳过的实例终端，则跳过当前循环
      }
      for (auto &pin: instTerm->getTerm()->getPins()) {// 检查当前引脚是否已经有有效的访问点
        if (pin2AP.find(pin.get()) == pin2AP.end()) {
          cout << "Error: pin does not have valid ap\n";
        } else {
          auto &ap = pin2AP[pin.get()]; // 获取当前引脚的访问点
          ap->getPoint(tmpPt);// 获取访问点的坐标
          if (tmpPt.x() < leftPt) {// 更新左右边界访问点和坐标值
            leftAP = ap;
            leftPt = tmpPt.x();
          }
          if (tmpPt.x() > rightPt) {
            rightAP = ap;
            rightPt = tmpPt.x();
          }
          pinAccessPattern->addAccessPoint(ap);// 将访问点添加到引脚访问模式对象中
        }
      }
    }
    //if (leftAP == nullptr) {
    //  cout <<"@@@Warning leftAP == nullptr " <<instTerm->getInst()->getName() <<endl;
    //}
    //if (rightAP == nullptr) {
    //  cout <<"@@@Warning rightAP == nullptr " <<instTerm->getInst()->getName() <<endl;
    //}
    pinAccessPattern->setBoundaryAP(true,  leftAP);// // 设置引脚访问模式对象的左右边界访问点
    pinAccessPattern->setBoundaryAP(false, rightAP);
    
    set<frBlockObject*> owners;  // 为targetObj生成所有相关对象，并检查DRC
    if (targetObj != nullptr && genPatterns_gc(targetObj, objs, &owners)) {// 如果targetObj不为空，并且DRC检查通过
      pinAccessPattern->updateCost();
      //cout <<"commit ap:";
      //for (auto &ap: pinAccessPattern->getPattern()) {
      //  cout <<" " <<ap;
      //}
      //cout <<endl <<"l/r = " 
      //     <<pinAccessPattern->getBoundaryAP(true) <<"/" 
      //     <<pinAccessPattern->getBoundaryAP(false) <<", "
      //     <<currUniqueInstIdx <<", " <<uniqueInstances[currUniqueInstIdx]->getName() <<endl;
      uniqueInstPatterns[currUniqueInstIdx].push_back(std::move(pinAccessPattern));
      // genPatterns_print(nodes, pins, maxAccessPointSize);
      isValid = true;
    } else {
      for (int idx1 = 0; idx1 < (int)pins.size(); idx1++) {
        auto idx2 = accessPattern[idx1];
        auto &[pin, instTerm] = pins[idx1];
        if (instTerm->hasNet()) {
          if (owners.find(instTerm->getNet()) != owners.end()) {
            violAccessPoints.insert(make_pair(idx1, idx2));// idx ;
          }
        } else {
          if (owners.find(instTerm) != owners.end()) {
            violAccessPoints.insert(make_pair(idx1, idx2));// idx ;
          }
        }
      }
    }

    hasNewPattern = true;
  } else {
    hasNewPattern = false;
  }

  return hasNewPattern;
}
//作用是在生成模式的过程中打印出失败的模式，以便于调试
void FlexPA::genPatterns_print_debug(std::vector<FlexDPNode> &nodes,
                               const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                               int maxAccessPointSize) {
  // bool enableOutput = false;
  int currNodeIdx = getFlatIdx(pins.size(), 0, maxAccessPointSize);
  auto currNode = &(nodes[currNodeIdx]);
  int pinCnt = pins.size();

  frTransform xform;
  auto &[pin, instTerm] = pins[0];
  if (instTerm) {
    frInst* inst = instTerm->getInst();
    inst->getTransform(xform);
  //   revertCellXForm.set(-xform.xOffset(), -xform.yOffset());
    xform.set(frcR0);
  }

  // auto inst = instTerm->getInst();
  cout << "failed pattern:";


  double dbu = getDesign()->getTopBlock()->getDBUPerUU();
  while (currNode->getPrevNodeIdx() != -1) {
    // non-virtual node
    if (pinCnt != (int)pins.size()) {
      auto &[pin, instTerm] = pins[pinCnt];
      auto inst = instTerm->getInst();
      cout <<" " <<instTerm->getTerm()->getName();
      int paIdx = unique2paidx[inst];
      auto pa = pin->getPinAccess(paIdx);
      int currIdx1, currIdx2;
      getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPointSize);
      //unique_ptr<frVia> via = make_unique<frVia>(pa->getAccessPoint(currIdx2)->getViaDef());
      frPoint pt(pa->getAccessPoint(currIdx2)->getPoint());
      pt.transform(xform);
      cout << " ("
           << pt.x() / dbu << ", " << pt.y() / dbu << ")";
    }

    currNodeIdx = currNode->getPrevNodeIdx();
    currNode = &(nodes[currNode->getPrevNodeIdx()]);
    pinCnt--;
  }
  cout <<endl;
  if (pinCnt != -1) {
    cout << "Error: valid access pattern not found\n";
  }
}

void FlexPA::genPatterns_print(std::vector<FlexDPNode> &nodes,
                               const std::vector<std::pair<frPin*, frInstTerm*> > &pins,
                               int maxAccessPointSize) {
  // bool enableOutput = false;
  int currNodeIdx = getFlatIdx(pins.size(), 0, maxAccessPointSize);
  auto currNode = &(nodes[currNodeIdx]);
  int pinCnt = pins.size();

  map<frOrientEnum, string> orient2Name = {{frcR0, "R0"}, 
                                           {frcR90, "R90"}, 
                                           {frcR180, "R180"}, 
                                           {frcR270, "R270"}, 
                                           {frcMY, "MY"}, 
                                           {frcMXR90, "MX90"},
                                           {frcMX, "MX"},
                                           {frcMYR90, "MY90"}};

  // frTransform xform, revertCellXForm;
  // auto &[pin, instTerm] = pins[0];
  // if (instTerm) {
  //   frInst* inst = instTerm->getInst();
  //   inst->getTransform(xform);
  //   revertCellXForm.set(-xform.xOffset(), -xform.yOffset());
  //   revertCellXForm.set(frcR0);
  // }

  // auto inst = instTerm->getInst();
  cout << "new pattern\n";


  while (currNode->getPrevNodeIdx() != -1) {
    // non-virtual node
    if (pinCnt != (int)pins.size()) {
      auto &[pin, instTerm] = pins[pinCnt];
      auto inst = instTerm->getInst();
      int paIdx = unique2paidx[inst];
      auto pa = pin->getPinAccess(paIdx);
      int currIdx1, currIdx2;
      getNestedIdx(currNodeIdx, currIdx1, currIdx2, maxAccessPointSize);
      unique_ptr<frVia> via = make_unique<frVia>(pa->getAccessPoint(currIdx2)->getViaDef());
      frPoint pt(pa->getAccessPoint(currIdx2)->getPoint());
      // pt.transform(revertCellXForm);
      cout << " gccleanvia " << instTerm->getInst()->getRefBlock()->getName() << " " 
           << instTerm->getTerm()->getName() << " " << via->getViaDef()->getName() << " "
           << pt.x() << " " << pt.y() << " "
           << orient2Name[instTerm->getInst()->getOrient()] << "\n";
    }

    currNodeIdx = currNode->getPrevNodeIdx();
    currNode = &(nodes[currNode->getPrevNodeIdx()]);
    pinCnt--;
  }
  if (pinCnt != -1) {
    cout << "Error: valid access pattern not found\n";
  }
}

// get flat index
// idx1 is outer index and idx2 is inner index dpNodes[idx1][idx2]
int FlexPA::getFlatIdx(int idx1, int idx2, int idx2Dim) {
  return ((idx1 + 1) * idx2Dim + idx2);
}

// get idx1 and idx2 from flat index
void FlexPA::getNestedIdx(int flatIdx, int &idx1, int &idx2, int idx2Dim) {
  idx1 = flatIdx / idx2Dim - 1;
  idx2 = flatIdx % idx2Dim;
}

// get flat edge index
int FlexPA::getFlatEdgeIdx(int prevIdx1, int prevIdx2, int currIdx2, int idx2Dim) {
  return (((prevIdx1 + 1) * idx2Dim + prevIdx2) * idx2Dim + currIdx2);
}
