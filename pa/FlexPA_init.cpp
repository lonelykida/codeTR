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

using namespace std;
using namespace fr;
//从设计中检索特定方向的轨道模式并将它们存储在提供的容器中
//作用是从设计中提取出具有特定偏好路由方向的轨道模式，并将它们存储在提供的容器中。这有助于后续的处理或优化步骤专注于特定方向的轨道模式。
void FlexPA::getPrefTrackPatterns(vector<frTrackPattern*> &prefTrackPatterns) {//获取轨道模式，参数用于储存轨道模式
  for (auto &trackPattern: design->getTopBlock()->getTrackPatterns()) {// 遍历设计顶层块中的所有轨道模式
    auto isVerticalTrack = trackPattern->isHorizontal(); /// 检查当前轨道模式的方向是否为垂直（水平轨道意味着垂直走向）
    // 获取当前轨道模式所在层的技术规范中定义的偏好路由方向并与枚举常量 frcHorzPrefRoutingDir 进行比较。
    if (design->getTech()->getLayer(trackPattern->getLayerNum())->getDir() == frcHorzPrefRoutingDir) {
    // 如果相等，表示该层的偏好路由方向为水平方向。
      // 如果该层的偏好路由方向为水平
      if (!isVerticalTrack) {// 并且当前轨道模式是水平的
        prefTrackPatterns.push_back(trackPattern);// 添加到结果向量中
      }
    } else {// 如果该层的偏好路由方向不是水平（即垂直）
      if (isVerticalTrack) {// 并且当前轨道模式是垂直的
        prefTrackPatterns.push_back(trackPattern);// 添加到结果向量中
      }
    }
  }
}

//目的是为每个参考块（RefBlock）初始化其引脚所处的层范围，并存储在一个映射中。
//这有助于后续处理，比如布局和优化步骤，通过预先知道每个块的引脚层范围，可以更有效地进行布线和其他设计优化工作。
void FlexPA::initUniqueInstance_refBlock2PinLayerRange(map<frBlock*, tuple<frLayerNum, frLayerNum>, frBlockObjectComp> &refBlock2PinLayerRange) {
  // 初始化参考块（RefBlock）的引脚层范围
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<endl <<"initUniqueInstances_refBlock2PinLayerRange ..." <<endl;
  }
  int numLayers = design->getTech()->getLayers().size();//获取层数
  for (auto &uRefBlock: design->getRefBlocks()) {//遍历块
    auto refBlock = uRefBlock.get();// 获取参考块的实际指针
    frLayerNum minLayerNum = std::numeric_limits<frLayerNum>::max();// 初始化最小层编号
    frLayerNum maxLayerNum = std::numeric_limits<frLayerNum>::min();// 初始化最大层编号
    for (auto &uTerm: refBlock->getTerms()) {// 遍历参考块中的所有引脚
      for (auto &uPin: uTerm->getPins()) {
        for (auto &uPinFig: uPin->getFigs()) {// 遍历每个引脚中的所有图形
          auto pinFig = uPinFig.get();//获取引脚图形
          if (pinFig->typeId() == frcRect) {//如果引脚是矩形
            auto lNum = static_cast<frRect*>(pinFig)->getLayerNum();//获取层号
            if (lNum >= getDesign()->getTech()->getBottomLayerNum()) {//确保层号在有效范围
              minLayerNum = std::min(minLayerNum, lNum);//更新最小层号
            }
            maxLayerNum = std::max(maxLayerNum, lNum);//更新最大层号
          } else if (pinFig->typeId() == frcPolygon) {//如果引脚是多边形
            auto lNum = static_cast<frPolygon*>(pinFig)->getLayerNum();//获取引脚层号
            if (lNum >= getDesign()->getTech()->getBottomLayerNum()) {
              minLayerNum = std::min(minLayerNum, lNum);// 更新最小层号
            }
            maxLayerNum = std::max(maxLayerNum, lNum);//获取最大层号
          } else {
            cout <<"Error: instAnalysis unsupported pinFig" <<endl;
          }
        }
      }
    }
    if (minLayerNum < getDesign()->getTech()->getBottomLayerNum() ||
        maxLayerNum > getDesign()->getTech()->getTopLayerNum()) {
      cout <<"Warning: instAnalysis skips " <<refBlock->getName() <<" due to no pin shapes" <<endl;
      continue;
    }
    maxLayerNum = std::min(maxLayerNum + 2, numLayers);// 确保最大层号不超过层数
    refBlock2PinLayerRange[refBlock] = make_tuple(minLayerNum, maxLayerNum);// 将范围存储到映射中
    if (enableOutput) {
      cout <<"  " <<refBlock->getName() <<" PIN layer ("
           <<design->getTech()->getLayer(minLayerNum)->getName() <<", "// 输出参考块的名称和引脚层范围
           <<design->getTech()->getLayer(maxLayerNum)->getName() <<")" <<endl;
    }
  }
  //cout <<"  refBlock pin layer range done" <<endl;
}

bool FlexPA::hasTrackPattern(frTrackPattern* tp, const frBox &box) {
  bool enableOutput = false;
  //bool enableOutput = true;
  auto isVerticalTrack = tp->isHorizontal(); // yes = vertical track
  frCoord low  = tp->getStartCoord();
  frCoord high = low + (frCoord)(tp->getTrackSpacing()) * ((frCoord)(tp->getNumTracks()) - 1);
  if (enableOutput) {
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    cout <<"tp low@" <<low / dbu <<", high@" <<high / dbu <<endl;
  }
  if (isVerticalTrack) {
    return !(low > box.right() || high < box.left());
  } else {
    return !(low > box.top() || high < box.bottom());
  }
}

//必须初始化使其处于活动状态
//负责处理特定参考块和其定位、方向对应的实例，并将它们按照其定位与预定义轨道的偏移进行组织，最终用于创建和更新独特实例的映射关系。
// must init all unique, including filler, macro, etc. to ensure frInst pinAccessIdx is active
void FlexPA::initUniqueInstance_main(const map<frBlock*, tuple<frLayerNum, frLayerNum>, frBlockObjectComp> &refBlock2PinLayerRange,
                                     const vector<frTrackPattern*> &prefTrackPatterns) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (enableOutput) {
    cout <<endl <<"initUniqueInstances_main ..." <<endl;
  }
  map<frBlock*, 
      map<frOrient, map<vector<frCoord>, set<frInst*, frBlockObjectComp> > >,
      frBlockObjectComp> refBlockOT2Insts; // refblock orient track-offset to instances，将轨道偏移定向到实例
  vector<frCoord> offset;//存储每个实例与轨道的偏移量
  int cnt = 0;
  for (auto &inst: design->getTopBlock()->getInsts()) {//遍历所有顶层块的所有实例
    frPoint origin;
    inst->getOrigin(origin);//获取实例的原点坐标
    frBox boundaryBBox;
    inst->getBoundaryBBox(boundaryBBox);//获取实例的原点坐标
    if (enableOutput) {
      double dbu = getDesign()->getTopBlock()->getDBUPerUU();
      //cout <<inst->getName() <<": (" 
      //     <<boundaryBBox.left()  <<", " <<boundaryBBox.bottom() <<") ("
      //     <<boundaryBBox.right() <<", " <<boundaryBBox.top()    <<")"
      //     <<endl;
      cout <<inst->getName() <<": (" 
           <<boundaryBBox.left()  / dbu <<", " <<boundaryBBox.bottom() / dbu <<") ("
           <<boundaryBBox.right() / dbu <<", " <<boundaryBBox.top()    / dbu <<")"
           <<endl;
    }
    auto orient = inst->getOrient();//获取实例的方向
    auto &[minLayerNum, maxLayerNum] = refBlock2PinLayerRange.find(inst->getRefBlock())->second;// 获取实例引脚所在层的范围
    offset.clear();
    for (auto &tp: prefTrackPatterns) {// 遍历所有偏好的轨道模式
      if (tp->getLayerNum() >= minLayerNum && tp->getLayerNum() <= maxLayerNum) {
        if (hasTrackPattern(tp, boundaryBBox)) {// 如果边界盒包含此轨道模式
          // vertical track
          if (tp->isHorizontal()) {//垂直轨道
            offset.push_back(origin.x() % tp->getTrackSpacing());//计算偏移
            //if (enableOutput) {
            //  cout <<"inst/offset/layer " <<inst->getName() <<" " <<origin.y() % tp->getTrackSpacing() 
            //       <<" " <<design->getTech()->getLayer(tp->getLayerNum())->getName() <<endl;
            //}
          } else {
            offset.push_back(origin.y() % tp->getTrackSpacing());
            //if (enableOutput) {
            //  cout <<"inst/offset/layer " <<inst->getName() <<" " <<origin.x() % tp->getTrackSpacing()
            //       <<" " <<design->getTech()->getLayer(tp->getLayerNum())->getName() <<endl;
            //}
          }
        } else {
          offset.push_back(tp->getTrackSpacing());
        }
      } else {
        offset.push_back(tp->getTrackSpacing());
      }
    }
    refBlockOT2Insts[inst->getRefBlock()][orient][offset].insert(inst.get());// 将实例按照参考块、方向和偏移组织
    cnt++;
    //if (VERBOSE > 0) {
    //  if (cnt < 100000) {
    //    if (cnt % 10000 == 0) {
    //      cout <<"  complete " <<cnt <<" instances" <<endl;
    //    }
    //  } else {
    //    if (cnt % 100000 == 0) {
    //      cout <<"  complete " <<cnt <<" instances" <<endl;
    //    }
    //  }
    //}
  }
//主要用于处理和组织集成电路设计中实例的数据，以便为布局和优化操作提供支持。
// 代码的具体目的是收集和映射实例的信息，将实例按照其参考块、方向和与预设轨道的偏移分类，并且处理独特实例的记录和索引。
  if (enableOutput) {
    cout <<endl <<"summary: " <<endl;
  }
  cnt = 0;
  frString orientName;
  for (auto &[refBlock, orientMap]: refBlockOT2Insts) {
    if (enableOutput) {
      cout <<"  " <<refBlock->getName() <<" (ORIENT/#diff patterns)" <<endl;
    }
    for (auto &[orient, offsetMap]: orientMap) {
      cnt += offsetMap.size();
      if (enableOutput) {
        orient.getName(orientName);
        cout <<"     (" <<orientName <<", " <<offsetMap.size() <<")";
      }
      for (auto &[vec, inst]: offsetMap) {
        auto uniqueInst = *(inst.begin());
        uniqueInstances.push_back(uniqueInst);
        for (auto i: inst) {
          inst2unique[i] = uniqueInst;
        }
        if (enableOutput) {
          cout <<" " <<(*(inst.begin()))->getName();
        }
      }
      if (enableOutput) {
        cout <<endl;
      }
    }
  }

  // init unique2Idx
  for (int i = 0; i < (int)uniqueInstances.size(); i++) {
    unique2Idx[uniqueInstances[i]] = i;
  }

  //if (VERBOSE > 0) {
  //  cout <<"#unique instances = " <<cnt <<endl;
  //}
}


void FlexPA::initUniqueInstance() {
  vector<frTrackPattern*> prefTrackPatterns;
  getPrefTrackPatterns(prefTrackPatterns);//// 调用 `getPrefTrackPatterns` 函数填充 `prefTrackPatterns` 向量，
  // 这个函数会筛选出与层的偏好路由方向相匹配的轨道模式。
  
  map<frBlock*, tuple<frLayerNum, frLayerNum>, frBlockObjectComp> refBlock2PinLayerRange;// 定义一个映射来存储参考块到其包含的引脚层范围的映射。
  // 这个映射的键是块的指针，值是一个包含两个层编号的元组，表示引脚的层范围。

  initUniqueInstance_refBlock2PinLayerRange(refBlock2PinLayerRange);
// 这个函数可能会分析设计中的块，确定每个块中引脚所涉及的最低层和最高层。
  initUniqueInstance_main(refBlock2PinLayerRange, prefTrackPatterns);
}
//始化所有实例和它们的引脚（pins）的引脚访问（pin access）信息。
void FlexPA::initPinAccess() {
  bool enableOutput = false;
  //bool enableOutput = true;
  // 遍历所有独特的实例
  for (auto &inst: uniqueInstances) {
    // 遍历实例的所有实例化引脚（instTerm）
    for (auto &instTerm: inst->getInstTerms()) {
      // 遍历每个实例化引脚的所有物理引脚
      for (auto &pin: instTerm->getTerm()->getPins()) {
        // 检查当前实例是否已经在 unique2paidx 映射中
        if (unique2paidx.find(inst) == unique2paidx.end()) {
          // 如果没有，则将其加入映射中，并设置对应的引脚访问索引
          unique2paidx[inst] = pin->getNumPinAccess();
        } else {
          // 如果已存在映射中，检查索引是否一致，不一致则报错
          if (unique2paidx[inst] != pin->getNumPinAccess()) {
            cout <<"Error: initPinAccess error" <<endl;
            exit(1);// 出现错误则退出
          }
        }
        // 为每个物理引脚创建一个新的引脚访问对象，并添加到引脚中
        auto pa = make_unique<frPinAccess>();
        pin->addPinAccess(pa);
      }
    }
    // 设置实例的引脚访问索引
    inst->setPinAccessIdx(unique2paidx[inst]);
  }
  // 更新其他所有实例的引脚访问索引，基于独特实例的映射
  for (auto &[inst, uniqueInst]: inst2unique) {
    inst->setPinAccessIdx(uniqueInst->getPinAccessIdx());
  }
  // 如果启用输出，打印 unique2paidx 映射的详细信息
  if (enableOutput) {
    cout <<"unique2paidx:" <<endl;
    for (auto &[inst, idx]: unique2paidx) {
      cout <<inst->getName() <<" " <<idx <<endl;
    }
  }
  // 如果启用输出，打印每个实例的引脚访问索引
  if (enableOutput) {
    cout <<"inst2paidx:" <<endl;
    for (auto &inst: getDesign()->getTopBlock()->getInsts()) {
      cout <<inst->getName() <<" " <<inst->getPinAccessIdx() <<endl;
    }
  }

  // IO terms// 为设计中的顶层块的所有I/O引脚创建新的引脚访问对象
  for (auto &term: getDesign()->getTopBlock()->getTerms()) {
    for (auto &pin: term->getPins()) {
      auto pa = make_unique<frPinAccess>();
      pin->addPinAccess(pa);
    }
  }
}
//这个函数的主要目的是为了初始化和分类存储芯片设计中所有层上的通孔（via）定义。
// 这些通孔定义根据它们所在的层级、切割数目（即在via中有多少切割图形），以及根据一些标准计算得出的原始优先级进行组织。
void FlexPA::initViaRawPriority() {
  // 遍历技术文件中定义的所有层
  for (auto layerNum = design->getTech()->getBottomLayerNum(); layerNum <= design->getTech()->getTopLayerNum(); ++layerNum) {
     // 检查当前层是否为切割层（CUT layer），切割层用于定义通孔
    if (design->getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::CUT) {
      continue; // 如果不是切割层，则跳过当前层
    }
    // 对于每个切割层，遍历该层的所有通孔定义
    for (auto &viaDef: design->getTech()->getLayer(layerNum)->getViaDefs()) {
      // 计算每个通孔定义中切割图形的数量
      int cutNum = int(viaDef->getCutFigs().size());
      viaRawPriorityTuple priority;// 创建一个用于存储优先级的元组
      // 从通孔定义中提取其原始优先级并存储在priority变量中
      getViaRawPriority(viaDef, priority);
      // 将通孔定义按层号、切割数和优先级元组分类存储
      layerNum2ViaDefs[layerNum][cutNum][priority] = viaDef;
      // layerNum2ViaDefs 是一个多级映射，其键是层号，值是另一个映射，该映射的键是切割数，值是又一个映射，
      // 最内层的映射的键是优先级元组，值是对应的通孔定义对象。
    }
  }
}
//获取并设置通孔 (via) 的原始优先级，基于一系列几何和配置特征
void FlexPA::getViaRawPriority(frViaDef* viaDef, viaRawPriorityTuple &priority) {
  bool isNotDefaultVia = !(viaDef->getDefault());//检查是否为默认的via
  bool isNotUpperAlign = false;
  bool isNotLowerAlign = false;
  gtl::polygon_90_set_data<frCoord> viaLayerPS1;

  for (auto &fig: viaDef->getLayer1Figs()) {//- 对于viaDef的第一层中的每个图形，执行以下操作：
    frBox bbox;
    fig->getBBox(bbox);//获取图像的边界框
    gtl::rectangle_data<frCoord> bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());//边界的坐标进行初始化
    using namespace boost::polygon::operators;
    viaLayerPS1 += bboxRect;// 将边界框添加到多边形集中
  }
  // 计算第一层图形的整体边界
  gtl::rectangle_data<frCoord> layer1Rect;//创建一个名为layer1Rect的gtl::rectangle_data类型的变量，
  gtl::extents(layer1Rect, viaLayerPS1);//并使用gtl::extents函数计算viaLayerPS1的边界矩形。// 从多边形集计算边界矩形
  bool isLayer1Horz = (gtl::xh(layer1Rect) - gtl::xl(layer1Rect)) > (gtl::yh(layer1Rect) - gtl::yl(layer1Rect));
  //通过比较layer1Rect的水平长度和垂直长度，确定第一层是否水平方向对齐。若水平长度大于垂直长度，则isLayer1Horz为true，否则为false。
  frCoord layer1Width = std::min((gtl::xh(layer1Rect) - gtl::xl(layer1Rect)), (gtl::yh(layer1Rect) - gtl::yl(layer1Rect)));
  //计算第一层的宽度，即layer1Rect的水平或垂直长度的较小值，并将结果存储在layer1Width变量中。
  isNotLowerAlign = (isLayer1Horz && (getDesign()->getTech()->getLayer(viaDef->getLayer1Num())->getDir() == frcVertPrefRoutingDir)) ||
                    (!isLayer1Horz && (getDesign()->getTech()->getLayer(viaDef->getLayer1Num())->getDir() == frcHorzPrefRoutingDir));
  //通过检查第一层的方向是否与垂直首选路径方向（frcVertPrefRoutingDir）或水平首选路径方向（frcHorzPrefRoutingDir）对齐，来确定isNotLowerAlign是否为true
  // 重复上述过程，处理via定义的第二层
  gtl::polygon_90_set_data<frCoord> viaLayerPS2;//储存viadef第二层的图形
  for (auto &fig: viaDef->getLayer2Figs()) {//对于第二层的图形进行下面的操作
    frBox bbox;
    fig->getBBox(bbox);
    gtl::rectangle_data<frCoord> bboxRect(bbox.left(), bbox.bottom(), bbox.right(), bbox.top());
    using namespace boost::polygon::operators;
    viaLayerPS2 += bboxRect;
  }
  gtl::rectangle_data<frCoord> layer2Rect;
  gtl::extents(layer2Rect, viaLayerPS2);
  //确定第二层是否为水平对齐
  bool isLayer2Horz = (gtl::xh(layer2Rect) - gtl::xl(layer2Rect)) > (gtl::yh(layer2Rect) - gtl::yl(layer2Rect));
  frCoord layer2Width = std::min((gtl::xh(layer2Rect) - gtl::xl(layer2Rect)), (gtl::yh(layer2Rect) - gtl::yl(layer2Rect)));
  isNotUpperAlign = (isLayer2Horz && (getDesign()->getTech()->getLayer(viaDef->getLayer2Num())->getDir() == frcVertPrefRoutingDir)) ||
                    (!isLayer2Horz && (getDesign()->getTech()->getLayer(viaDef->getLayer2Num())->getDir() == frcHorzPrefRoutingDir));
  //确定两个的面积
  frCoord layer1Area = gtl::area(viaLayerPS1);
  frCoord layer2Area = gtl::area(viaLayerPS2);
// 将计算出的各项指标组成一个元组，作为通孔的原始优先级
  priority = std::make_tuple(isNotDefaultVia, layer1Width, layer2Width, isNotUpperAlign, layer2Area, layer1Area, isNotLowerAlign);
}

void FlexPA::initTrackCoords() {//初始化轨道的坐标信息
  bool enableOutput = false;
  //bool enableOutput = true;

  if (enableOutput) {
    cout <<endl <<endl;
  }

  int     numLayers = getDesign()->getTech()->getLayers().size();//获取design中的layer数量
  frCoord manuGrid  = getDesign()->getTech()->getManufacturingGrid();//制造网格的尺寸

  // full coords
  trackCoords.clear();//清空轨道信息
  trackCoords.resize(numLayers);// 调整trackCoords的大小以匹配层数
  for (auto &trackPattern: design->getTopBlock()->getTrackPatterns()) {//遍历每个轨道模式
    auto layerNum = trackPattern->getLayerNum();//获取轨道模式层的编号
    auto isVLayer = (design->getTech()->getLayer(layerNum)->getDir() == frcVertPrefRoutingDir);//轨道水平or垂直
    auto isVTrack = trackPattern->isHorizontal(); // yes = vertical track 判断是否为水平轨道
    if ((!isVLayer && !isVTrack) || (isVLayer && isVTrack)) {//如果层的方向和轨道方向对应
      frCoord currCoord = trackPattern->getStartCoord();//获取轨道的起始坐标
      for (int i = 0; i < (int)trackPattern->getNumTracks(); i++) {//对每个轨道
        trackCoords[layerNum][currCoord] = 0; // cost 0 for full coords// 将轨道坐标加入到对应层的trackCoords映射中，成本设置为0
        currCoord += trackPattern->getTrackSpacing(); // 更新轨道坐标，增加轨道间距
      }
    }
  }
// 如果当前层不是垂直层且轨道模式不是水平轨道，或者当前层是垂直层且轨道模式是水平轨道，则执行以下操作：
// 获取轨道模式的起始坐标（currCoord）。
// 对于轨道模式中的每个轨道，将其坐标（currCoord）添加到trackCoords中，并将其对应的代价设置为0。
// 更新当前坐标，使其增加轨道间距（trackSpacing）。
  if (enableOutput) {
    cout <<"full coords: " <<endl;
    int cnt = 0;
    for (auto &m: trackCoords) {
      if (!m.empty()) {
        cout <<getDesign()->getTech()->getLayer(cnt)->getName() <<": " <<m.size() <<endl;
      }
      cnt++;
    }
  }
//半坐标时完整坐标的中点
  // half coords// 计算半轨道坐标
  vector<vector<frCoord> > halfTrackCoords(numLayers);
  for (int i = 0; i < numLayers; i++) { // 对每层
    frCoord prevFullCoord = std::numeric_limits<frCoord>::max();
    for (auto &[currFullCoord, cost]: trackCoords[i]) {// 遍历当前层的所有轨道
      if (currFullCoord > prevFullCoord) {// 如果当前轨道坐标大于前一个轨道坐标
        frCoord currHalfGrid = (currFullCoord + prevFullCoord) / 2 / manuGrid * manuGrid;// 计算半轨道坐标
        if (currHalfGrid != currFullCoord && currHalfGrid != prevFullCoord) {// 如果半轨道坐标有效
          halfTrackCoords[i].push_back(currHalfGrid);// 加入到半轨道坐标列表
        }
      }
      prevFullCoord = currFullCoord;
    }
    for (auto halfCoord: halfTrackCoords[i]) {// 对每个有效的半轨道坐标
      trackCoords[i][halfCoord] = 1; // cost 1 for half coords
    }
  }
  if (enableOutput) {
    cout <<"full+half coords: " <<endl;
    int cnt = 0;
    for (auto &m: trackCoords) {// 输出包含完整和半轨道坐标的数量
      if (!m.empty()) {
        cout <<getDesign()->getTech()->getLayer(cnt)->getName() <<": " <<m.size() <<endl;
      }
      cnt++;
    }
  }
  
  // quarter coords
  //halfTrackCoords.clear();
  //halfTrackCoords.resize(numLayers);
  //for (int i = 0; i < numLayers; i++) {
  //  frCoord prevFullCoord = std::numeric_limits<frCoord>::max();
  //  for (auto &[currFullCoord, cost]: trackCoords[i]) {
  //    if (currFullCoord > prevFullCoord) {
  //      frCoord currHalfGrid = (currFullCoord + prevFullCoord) / 2 / manuGrid * manuGrid;
  //      if (currHalfGrid != currFullCoord && currHalfGrid != prevFullCoord) {
  //        halfTrackCoords[i].push_back(currHalfGrid);
  //      }
  //    }
  //    prevFullCoord = currFullCoord;
  //  }
  //  for (auto halfCoord: halfTrackCoords[i]) {
  //    trackCoords[i][halfCoord] = 3; // cost 3 for quarter coords
  //  }
  //}
  //if (enableOutput) {
  //  cout <<"full+half+quarter coords: " <<endl;
  //  int cnt = 0;
  //  for (auto &m: trackCoords) {
  //    if (!m.empty()) {
  //      cout <<getDesign()->getTech()->getLayer(cnt)->getName() <<": " <<m.size() <<endl;
  //    }
  //    cnt++;
  //  }
  //}
}
