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
#include "gc/FlexGC.h"
#include "db/drObj/drNet.h"
#include "dr/FlexDR.h"

using namespace std;
using namespace fr;

gcNet* FlexGCWorker::getNet(frBlockObject* obj) {
  bool isFloatingVDD = false;
  bool isFloatingVSS = false;
  frBlockObject* owner = nullptr;
  if (obj->typeId() == frcTerm) {
    auto term = static_cast<frTerm*>(obj);
    if (term->hasNet()) {
      owner = term->getNet();
    } else {
      if (term->getType() == frTermEnum::frcPowerTerm) {
        isFloatingVDD = true;
      } else if (term->getType() == frTermEnum::frcGroundTerm) {
        isFloatingVSS = true;
      }
      owner = obj;
    }
  } else if (obj->typeId() == frcInstTerm) {
    auto instTerm = static_cast<frInstTerm*>(obj);
    if (instTerm->hasNet()) {
      owner = instTerm->getNet();
    } else {
      if (instTerm->getTerm()->getType() == frTermEnum::frcPowerTerm) {
        isFloatingVDD = true;
      } else if (instTerm->getTerm()->getType() == frTermEnum::frcGroundTerm) {
        isFloatingVSS = true;
      }
      owner = obj;
    }
  } else if (obj->typeId() == frcInstBlockage || obj->typeId() == frcBlockage) {
    owner = obj;
  } else if (obj->typeId() == frcPathSeg || obj->typeId() == frcVia || obj->typeId() == frcPatchWire) {
    auto shape = static_cast<frShape*>(obj);
    if (shape->hasNet()) {
      owner = shape->getNet();
    } else {
      cout <<"Error: init_design_helper shape does not have net" <<endl;
      exit(1);
    }
  } else if (obj->typeId() == drcPathSeg || obj->typeId() == drcVia || obj->typeId() == drcPatchWire) {
    auto shape = static_cast<drShape*>(obj);
    if (shape->hasNet()) {
      owner = shape->getNet()->getFrNet();
    } else {
      cout <<"Error: init_design_helper shape does not have dr net" <<endl;
      exit(1);
    }
  } else {
    cout <<"Error: init_design_helper unsupported type" <<endl;
    exit(1);
  }

  if (isFloatingVSS) {
    return nets[0].get();
  } else if (isFloatingVDD) {
    return nets[1].get();
  } else {
    auto it = owner2nets.find(owner);
    gcNet* currNet = nullptr;
    if (it == owner2nets.end()) {
      currNet = addNet(owner);
    } else {
      currNet = it->second;
    }
    return currNet;
  }
}

void FlexGCWorker::initObj(const frBox &box, frLayerNum layerNum, frBlockObject* obj, bool isFixed) {
  auto currNet = getNet(obj);
  if (getDesign()->getTech()->getLayer(layerNum)->getType() == frLayerTypeEnum::CUT) {
    currNet->addRectangle(box, layerNum, isFixed);
  } else {
    currNet->addPolygon(box, layerNum, isFixed);
  }
}
//目的是决定是否应该跳过给定的设计对象在初始化过程中的处理。
bool FlexGCWorker::initDesign_skipObj(frBlockObject* obj) {
  // 如果指定了目标对象（targetObj），则进行特定条件检查以决定是否跳过当前对象（obj）
  if (targetObj) {
    if (targetObj->typeId() == frcInst) {
      // 如果当前对象是实例端点（frInstTerm类型），且属于目标实例
      if (obj->typeId() == frcInstTerm &&
          static_cast<frInstTerm*>(obj)->getInst() == targetObj) {
        return false;//不跳过
        // 如果当前对象是实例阻挡（frInstBlockage类型），且属于目标实例
      } else if (obj->typeId() == frcInstBlockage &&
                 static_cast<frInstBlockage*>(obj)->getInst() == targetObj) {
        return false;
      } else {
        return true;
      }
      // 如果目标对象是端点（frTerm类型）
    } else if (targetObj->typeId() == frcTerm) {
      // 如果当前对象也是端点（frTerm类型），且就是目标端点
      if (obj->typeId() == frcTerm && static_cast<frTerm*>(obj) == static_cast<frTerm*>(targetObj)) {
        return false;
      } else {
        return true;
      }
    } else {
      cout <<"Error: FlexGCWorker::initDesign_skipObj type not supported" <<endl;
      exit(1);
    }
  } else {
    return false;
  }
  return false;
}

//负责初始化设计中所有的对象，包括固定对象和详细布线对象。
void FlexGCWorker::initDesign() {
  bool enableOutput = false;
  //bool enableOutput = true;

  if (ignoreDB) {//忽略数据库
    return;
  }
// 获取数据库单位每用户单位的转换系数
  double dbu = getDesign()->getTech()->getDBUPerUU();
  auto &extBox = getExtBox();// 获取外延盒（扩展区域）
  if (enableOutput) {// 如果启用了输出，显示扩展盒的坐标信息
    cout << "  extBox (" << extBox.left()  / dbu << ", " << extBox.bottom() / dbu
         << ") -- ("     << extBox.right() / dbu << ", " << extBox.top()    / dbu << "):\n";
  }
  // 创建查询框，基于外延盒的坐标
  box_t queryBox(point_t(extBox.left(), extBox.bottom()), point_t(extBox.right(), extBox.top()));
  auto regionQuery = getDesign()->getRegionQuery();//获取区域查询对象
  // 用于存储查询结果的容器
  vector<rq_rptr_value_t<frBlockObject> > queryResult;
  frBox box;
  int cnt = 0;
  // init all non-dr objs from design
  // for (auto i = getDesign()->getTech()->getBottomLayerNum(); i <= design->getTech()->getTopLayerNum(); i++) {
  //遍历所有层次，初始化设计中的非DR对象
  for (auto i = 0; i <= design->getTech()->getTopLayerNum(); i++) {
    queryResult.clear();
    // queryResult.clear();
    regionQuery->query(queryBox, i, queryResult);
    for (auto &[boostB, obj]: queryResult) {
      //跳过不需要处理的对象
      if (initDesign_skipObj(obj)) {
        continue;
      }
      // 设置对象的边界框
      box.set(boostB.min_corner().x(), boostB.min_corner().y(), boostB.max_corner().x(), boostB.max_corner().y());
      initObj(box, i, obj, true);// 初始化对象
      cnt++;
    }
  }
  if (enableOutput) {
    cout << "#fixed obj = " <<cnt << "\n";
  }
  // init all dr objs from design
  if (getDRWorker()) {
    return;
  }
  cnt = 0;
  for (auto i = getDesign()->getTech()->getBottomLayerNum(); i <= design->getTech()->getTopLayerNum(); i++) {
    queryResult.clear();
    regionQuery->queryDRObj(queryBox, i, queryResult);// 对指定的查询盒和层进行详细布线对象的查询。
    for (auto &[boostB, obj]: queryResult) {
      if (initDesign_skipObj(obj)) {
        continue;
      }
      box.set(boostB.min_corner().x(), boostB.min_corner().y(), boostB.max_corner().x(), boostB.max_corner().y());
      initObj(box, i, obj, false);
      cnt++;
    }
  }
  if (enableOutput) {
    cout << "#route obj = " <<cnt << "\n";
  }
}

void FlexGCWorker::addPAObj(frConnFig* obj, frBlockObject* owner) {
  auto it = owner2nets.find(owner);
  gcNet* currNet = nullptr;
  if (it == owner2nets.end()) {
    currNet = addNet(owner);
  } else {
    currNet = it->second;
  }

  frBox box;
  frTransform xform;
  frLayerNum layerNum;
  if (obj->typeId() == frcPathSeg) {
    auto pathSeg = static_cast<frPathSeg*>(obj);
    pathSeg->getBBox(box);
    currNet->addPolygon(box, pathSeg->getLayerNum(), false);
  } else if (obj->typeId() == frcVia) {
    auto via = static_cast<frVia*>(obj);
    layerNum = via->getViaDef()->getLayer1Num();
    via->getTransform(xform);
    for (auto &fig: via->getViaDef()->getLayer1Figs()) {
      fig->getBBox(box);
      box.transform(xform);
      currNet->addPolygon(box, layerNum, false);
    }
    // push cut layer rect
    layerNum = via->getViaDef()->getCutLayerNum();
    for (auto &fig: via->getViaDef()->getCutFigs()) {
      fig->getBBox(box);
      box.transform(xform);
      currNet->addRectangle(box, layerNum, false);
    }
    // push layer2 rect
    layerNum = via->getViaDef()->getLayer2Num();
    for (auto &fig: via->getViaDef()->getLayer2Figs()) {
      frBox bbox;
      fig->getBBox(box);
      box.transform(xform);
      currNet->addPolygon(box, layerNum, false);
    }
  } else if (obj->typeId() == frcPatchWire) {
    auto pwire = static_cast<frPatchWire*>(obj);
    pwire->getBBox(box);
    currNet->addPolygon(box, pwire->getLayerNum(), false);
  }
  
  // added 09/24/19
  if (modifiedOwnersSet.find(owner) == modifiedOwnersSet.end()) {
    modifiedOwners.push_back(owner);
    modifiedOwnersSet.insert(owner);
  }
}

//本函数主要用于初始化不同类型的设计规则检查（DRC）对象，包括路径段、通孔和贴片线。
// 它根据对象类型获取相应的边界信息和层级信息，并将这些信息添加到当前的网络对象中。
// 这样的处理确保了网络对象包含了所有与其关联的设计元素，以便进行后续的设计规则检查。
void FlexGCWorker::initDRObj(drConnFig* obj, gcNet* currNet) {
  if (currNet == nullptr) {
    currNet = getNet(obj);
  }
  frBox box;
  frTransform xform;
  frLayerNum layerNum;
  if (obj->typeId() == drcPathSeg) {
    auto pathSeg = static_cast<drPathSeg*>(obj);
    pathSeg->getBBox(box);
    // debug
    // if (pathSeg->getLayerNum() == 4 && box.left() < 470300 && box.right() > 470300 && box.bottom() < 100800 && box.top() > 100800) {
    //   cout << "  @@@ debug: initDRObj catches wire on M2 (";
    //   auto owner = currNet->getOwner();
    //   if (owner == nullptr) {
    //     cout <<" FLOATING";
    //   } else {
    //     if (owner->typeId() == frcNet) {
    //       cout <<static_cast<frNet*>(owner)->getName();
    //     } else if (owner->typeId() == frcInstTerm) {
    //       cout <<static_cast<frInstTerm*>(owner)->getInst()->getName() <<"/" 
    //            <<static_cast<frInstTerm*>(owner)->getTerm()->getName();
    //     } else if (owner->typeId() == frcTerm) {
    //       cout <<"PIN/" <<static_cast<frTerm*>(owner)->getName();
    //     } else if (owner->typeId() == frcInstBlockage) {
    //       cout <<static_cast<frInstBlockage*>(owner)->getInst()->getName() <<"/OBS";
    //     } else if (owner->typeId() == frcBlockage) {
    //       cout <<"PIN/OBS";
    //     } else {
    //       cout <<"UNKNOWN";
    //     }
    //   }
    //   cout <<  ") (" << box.left() << ", " << box.bottom() << ") - (" << box.right() << ", " << box.top() << ")\n";
    // }
    currNet->addPolygon(box, pathSeg->getLayerNum());
  } else if (obj->typeId() == drcVia) {
    auto via = static_cast<drVia*>(obj);
    layerNum = via->getViaDef()->getLayer1Num();
    via->getTransform(xform);
    for (auto &fig: via->getViaDef()->getLayer1Figs()) {
      fig->getBBox(box);
      box.transform(xform);
      currNet->addPolygon(box, layerNum);
    }
    // push cut layer rect
    layerNum = via->getViaDef()->getCutLayerNum();
    for (auto &fig: via->getViaDef()->getCutFigs()) {
      fig->getBBox(box);
      box.transform(xform);
      currNet->addRectangle(box, layerNum);
    }
    // push layer2 rect
    layerNum = via->getViaDef()->getLayer2Num();
    for (auto &fig: via->getViaDef()->getLayer2Figs()) {
      frBox bbox;
      fig->getBBox(box);
      box.transform(xform);
      currNet->addPolygon(box, layerNum);
    }
  } else if (obj->typeId() == drcPatchWire) {
    auto pwire = static_cast<drPatchWire*>(obj);
    pwire->getBBox(box);
    currNet->addPolygon(box, pwire->getLayerNum());
  }
}
//是初始化和配置来自详细布线（DRWorker）的对象。
void FlexGCWorker::initDRWorker() {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (!getDRWorker()) {//检查是否为有效的DR实例
    return;
  }
  int cnt = 0;
  for (auto &uDRNet: getDRWorker()->getNets()) {//初始化一个计数器 cnt 来跟踪处理的连接图形对象数量，并遍历 DRWorker 提供的所有网络
    // always first generate gcnet in case owner does not have any object
    auto it = owner2nets.find(uDRNet->getFrNet());
    if (it == owner2nets.end()) {
      addNet(uDRNet->getFrNet());
    }//对于每个网络，检查其对应的 frNet（基础网络对象）是否已经在 owner2nets 映射中注册。如果没有注册，调用 addNet() 将其添加进映射中。
    //auto net = uDRNet->getFrNet();
    //遍历网络中的所有连接图形对象。这包括外部连接图形 (ExtConnFigs) 和路由连接图形 (RouteConnFigs)。对每个连接图形，调用 initDRObj() 来进行初始化。每处理一个对象，计数器 cnt 自增。
    for (auto &uConnFig: uDRNet->getExtConnFigs()) {
      initDRObj(uConnFig.get());
      cnt++;
    }
    for (auto &uConnFig: uDRNet->getRouteConnFigs()) {
      initDRObj(uConnFig.get());
      cnt++;
    }
  }
  if (enableOutput) {
    cout << "#route obj = " <<cnt << "\n";
  }
}
//如何处理网络中的固定与非固定多边形和矩形，用于初始化不同类型的引脚。
void FlexGCWorker::initNet_pins_polygon(gcNet* net) {
  int numLayers = getDesign()->getTech()->getLayers().size();//获取设计的层的数量
  // init pin from polygons
  vector<gtl::polygon_90_set_data<frCoord> > layerPolys(numLayers);//初始化层的数量的多边形容器
  vector<gtl::polygon_90_with_holes_data<frCoord> > polys;/// 创建一个向量，用于临时存储具有洞的多边形 ？？？ 洞？
 // 遍历每一层，初始化来自多边形的引脚
  for (int i = 0; i < numLayers; i++) {
    polys.clear();
    using namespace gtl::operators;//// 引入gtl命名空间中的运算符，以便操作多边形
    layerPolys[i] += net->getPolygons(i, false);//返回net的 route的多边形集合
    layerPolys[i] += net->getPolygons(i, true);//返回net的fixed 的多边形集合
    layerPolys[i].get(polys);//从当前层的多边形集合中提取多边形
    for (auto &poly: polys) {// // 遍历临时多边形向量，为每个多边形创建引脚
      net->addPin(poly, i);
    }
  }
  // init pin from rectangles
  for (int i = 0; i < numLayers; i++) {//分别为不同层的fixed和route举行集合初始化pin
    for (auto &rect: net->getRectangles(i, false)) {
      net->addPin(rect, i);
    }
    for (auto &rect: net->getRectangles(i, true)) {
      net->addPin(rect, i);
    }
  }
}

void FlexGCWorker::initNet_pins_polygonEdges_getFixedPolygonEdges(gcNet* net, vector<set<pair<frPoint, frPoint> > > &fixedPolygonEdges) {
  int numLayers = getDesign()->getTech()->getLayers().size();
  vector<gtl::polygon_90_with_holes_data<frCoord> > polys;
  frPoint bp, ep, firstPt;
  // get fixed polygon edges from polygons
  for (int i = 0; i < numLayers; i++) {
    polys.clear();
    net->getPolygons(i, true).get(polys);
    for (auto &poly: polys) {
      // skip the first pt
      auto outerIt = poly.begin();
      bp.set((*outerIt).x(), (*outerIt).y());
      firstPt.set((*outerIt).x(), (*outerIt).y());
      outerIt++;
      // loop from second to last pt (n-1) edges
      for (; outerIt != poly.end(); outerIt++) {
        ep.set((*outerIt).x(), (*outerIt).y());
        fixedPolygonEdges[i].insert(make_pair(bp, ep));
        bp.set(ep);
      }
      // insert last edge
      fixedPolygonEdges[i].insert(make_pair(bp, firstPt));
      for (auto holeIt = poly.begin_holes(); holeIt != poly.end_holes(); holeIt++) {
        auto &hole_poly = *holeIt;
        // skip the first pt
        auto innerIt = hole_poly.begin();
        bp.set((*innerIt).x(), (*innerIt).y());
        firstPt.set((*innerIt).x(), (*innerIt).y());
        innerIt++;
        // loop from second to last pt (n-1) edges
        for (; innerIt != hole_poly.end(); innerIt++) {
          ep.set((*innerIt).x(), (*innerIt).y());
          fixedPolygonEdges[i].insert(make_pair(bp, ep));
          bp.set(ep);
        }
        // insert last edge
        fixedPolygonEdges[i].insert(make_pair(bp, firstPt));
      }
    }
  }
  // for rectangles input --> non-merge scenario
  for (int i = 0; i < numLayers; i++) {
    for (auto &rect: net->getRectangles(i, true)) {
      fixedPolygonEdges[i].insert(make_pair(frPoint(gtl::xl(rect), gtl::yl(rect)), 
                                            frPoint(gtl::xh(rect), gtl::yl(rect))));
      fixedPolygonEdges[i].insert(make_pair(frPoint(gtl::xh(rect), gtl::yl(rect)), 
                                            frPoint(gtl::xh(rect), gtl::yh(rect))));
      fixedPolygonEdges[i].insert(make_pair(frPoint(gtl::xh(rect), gtl::yh(rect)), 
                                            frPoint(gtl::xl(rect), gtl::yh(rect))));
      fixedPolygonEdges[i].insert(make_pair(frPoint(gtl::xl(rect), gtl::yh(rect)), 
                                            frPoint(gtl::xl(rect), gtl::yl(rect))));
    }
  }
}
//来处理网络中多边形的外边缘，并根据这些边缘是否是固定的，对它们进行标记。函数也将边缘与引脚相关联，并构建边缘间的连接关系。
void FlexGCWorker::initNet_pins_polygonEdges_helper_outer(gcNet* net, gcPin* pin, gcPolygon* poly, frLayerNum i, 
                                                          const vector<set<pair<frPoint, frPoint> > > &fixedPolygonEdges) {
  // 声明和初始化用于存储多边形顶点的变量
  frPoint bp, ep, firstPt;
  gtl::point_data<frCoord> bp1, ep1, firstPt1;
  // 存储临时边缘对象的向量
  vector<unique_ptr<gcSegment> > tmpEdges;
  // skip the first pt//跳过第一个点
  auto outerIt = poly->begin();
  bp.set((*outerIt).x(), (*outerIt).y());
  bp1 = *outerIt;
  firstPt.set((*outerIt).x(), (*outerIt).y());
  firstPt1 = *outerIt;
  outerIt++;
  // loop from second to last pt (n-1) edges//从第二个点开始遍历多边形顶点，直到最后一个点
  for (; outerIt != poly->end(); outerIt++) {
    ep.set((*outerIt).x(), (*outerIt).y());
    ep1 = *outerIt;
    auto edge = make_unique<gcSegment>();//创建一个新的边缘对象
    edge->setLayerNum(i);
    edge->addToPin(pin);
    edge->addToNet(net);
    //edge->setPoints(bp, ep);
    edge->setSegment(bp1, ep1);
    // 检查当前边缘是否在固定边缘集合中，相应地设置边缘状态
    if (fixedPolygonEdges[i].find(make_pair(bp, ep)) != fixedPolygonEdges[i].end()) {
      // fixed edge
      edge->setFixed(true);//设为 fixed
      //cntFixed++;
    } else {
      // route edge;
      edge->setFixed(false); // 非 fixed
      //cntRoute++;
    }
    // 设置当前边缘与前一个边缘的关联
    if (!tmpEdges.empty()) {
      edge->setPrevEdge(tmpEdges.back().get());//相当于将 tmpedge设置为edge 的前置边缘
      tmpEdges.back()->setNextEdge(edge.get());
    }
    tmpEdges.push_back(std::move(edge));
    bp.set(ep);
    bp1 = ep1;
    //cntOuter++;
  }
  // last edge// 处理最后一个边缘，连接第一个点和最后一个点
  auto edge = make_unique<gcSegment>();
  edge->setLayerNum(i);
  edge->addToPin(pin);
  edge->addToNet(net);
  //edge->setPoints(bp, firstPt);
  edge->setSegment(bp1, firstPt1);
  // 同样检查这个边缘是否固定
  if (fixedPolygonEdges[i].find(make_pair(bp, firstPt)) != fixedPolygonEdges[i].end()) {
    // fixed edge
    edge->setFixed(true);
    //cntFixed++;
  } else {
    // route edge;
    edge->setFixed(false);
    //cntRoute++;
  }
  edge->setPrevEdge(tmpEdges.back().get());
  tmpEdges.back()->setNextEdge(edge.get());
  // set first edge// 设置边缘链表的循环连接
  tmpEdges.front()->setPrevEdge(edge.get());
  edge->setNextEdge(tmpEdges.front().get());

  tmpEdges.push_back(std::move(edge));
  // add to polygon edges
  pin->addPolygonEdges(tmpEdges);// 将处理好的边缘列表添加到引脚的多边形边缘列表中——链表形式
}
// 定义一个处理网络引脚的内部多边形边缘的辅助函数
void FlexGCWorker::initNet_pins_polygonEdges_helper_inner(gcNet* net, gcPin* pin, 
                                                          const gtl::polygon_90_data<frCoord> &hole_poly, frLayerNum i, 
                                                          const vector<set<pair<frPoint, frPoint> > > &fixedPolygonEdges) {
  frPoint bp, ep, firstPt;// // 定义和初始化用于存储多边形顶点的变量
  gtl::point_data<frCoord> bp1, ep1, firstPt1;
  vector<unique_ptr<gcSegment> > tmpEdges; // 存储临时边缘对象的向量
  // skip the first pt跳过第一个点
  auto innerIt = hole_poly.begin();
  bp.set((*innerIt).x(), (*innerIt).y());
  bp1 = *innerIt;
  firstPt.set((*innerIt).x(), (*innerIt).y());
  firstPt1 = *innerIt;
  innerIt++;
  // loop from second to last pt (n-1) edges// 从第二个点开始遍历多边形顶点，直到最后一个点
  for (; innerIt != hole_poly.end(); innerIt++) {
    ep.set((*innerIt).x(), (*innerIt).y());
    ep1 = *innerIt;
    auto edge = make_unique<gcSegment>();
    edge->setLayerNum(i);
    edge->addToPin(pin);
    edge->addToNet(net);
    //edge->setPoints(bp, ep);
    edge->setSegment(bp1, ep1);
     // 检查当前边缘是否在固定边缘集合中，相应地设置边缘状态
    if (fixedPolygonEdges[i].find(make_pair(bp, ep)) != fixedPolygonEdges[i].end()) {
      // fixed edge
      edge->setFixed(true);
      //cntFixed++;
    } else {
      // route edge;
      edge->setFixed(false);
      //cntRoute++;
    }
    if (!tmpEdges.empty()) {
      edge->setPrevEdge(tmpEdges.back().get());
      tmpEdges.back()->setNextEdge(edge.get());
    }
    tmpEdges.push_back(std::move(edge));
    bp.set(ep);
    bp1 = ep1;
    //cntInner++;
  }
  auto edge = make_unique<gcSegment>();
  edge->setLayerNum(i);
  edge->addToPin(pin);
  edge->addToNet(net);
  //edge->setPoints(bp, firstPt);
  edge->setSegment(bp1, firstPt1);
  // last edge
  // 同样检查这个边缘是否固定
  if (fixedPolygonEdges[i].find(make_pair(bp, firstPt)) != fixedPolygonEdges[i].end()) {
    // fixed edge
    edge->setFixed(true);
    //cntFixed++;
  } else {
    // route edge;
    edge->setFixed(false);
    //cntRoute++;
  }
  edge->setPrevEdge(tmpEdges.back().get());
  tmpEdges.back()->setNextEdge(edge.get());
  // set first edge
  tmpEdges.front()->setPrevEdge(edge.get());
  edge->setNextEdge(tmpEdges.front().get());
  
  tmpEdges.push_back(std::move(edge));
  // add to polygon edges
  pin->addPolygonEdges(tmpEdges);
}
//处理网络中多边形的边缘，并将它们标记为引脚。
void FlexGCWorker::initNet_pins_polygonEdges(gcNet* net) {
  //bool enableOutput = true;
  //bool enableOutput = false;
  int numLayers = getDesign()->getTech()->getLayers().size();//层数
  // 创建一个向量，用于存储每层的固定多边形边缘（边缘以点对的形式存储）
  vector<set<pair<frPoint, frPoint> > > fixedPolygonEdges(numLayers); 
  // get all fixed polygon edges
  // 调用一个函数来初始化并获取所有层的固定多边形边缘
  initNet_pins_polygonEdges_getFixedPolygonEdges(net, fixedPolygonEdges);
// 遍历每一层的所有合并后的多边形，以构建并标记边缘
  // loop through all merged polygons and build mark edges
  for (int i = 0; i < numLayers; i++) {
    for (auto &pin: net->getPins(i)) {
      auto poly = pin->getPolygon();//获取引脚的多边形
       // 调用一个辅助函数处理外部多边形边缘
      initNet_pins_polygonEdges_helper_outer(net, pin.get(), poly, i, fixedPolygonEdges);
      // pending// 处理每个多边形中的洞（内部多边形） 不懂洞？是什么
      for (auto holeIt = poly->begin_holes(); holeIt != poly->end_holes(); holeIt++) {
        //获取内部多边形
        auto &hole_poly = *holeIt;
        //处理内部多边形 的 边缘
        initNet_pins_polygonEdges_helper_inner(net, pin.get(), hole_poly, i, fixedPolygonEdges);
      }
    }
  }
  //if (enableOutput) {
  //  cout <<"#fixed/route polygon edges = " <<cntFixed <<"/" <<cntRoute <<endl;
  //  cout <<"#outer/inner polygon edges = " <<cntOuter <<"/" <<cntInner <<endl;
  //}
}
//用来处理引脚多边形的角点。
// 它遍历每个多边形边缘，为每条边缘创建一个角点，并设置角点的类型、方向以及固定/路由状态。
// 角点的类型和方向根据边缘的凸凹性和相邻边缘的方向确定。同时，角点的固定/路由状态根据角点是否与固定的矩形重叠或在不可路由的矩形内部来确定。
// 最后，将处理好的角点添加到引脚的多边形角点列表中，以供后续的处理和分析使用。
void FlexGCWorker::initNet_pins_polygonCorners_helper(gcNet* net, gcPin* pin) {
  for (auto &edges: pin->getPolygonEdges()) {  // 遍历引脚的多边形边缘
    vector<unique_ptr<gcCorner> > tmpCorners;//临时存储角点对象的向量
    // 初始化前一边缘和层号
    auto prevEdge = edges.back().get();
    auto layerNum = prevEdge->getLayerNum();//层号
    gcCorner* prevCorner = nullptr;
    //遍历每一条边缘
    for (int i = 0; i < (int)edges.size(); i++) {
      //获取当前边缘和角
      auto nextEdge = edges[i].get();//下一边缘
      auto uCurrCorner = std::make_unique<gcCorner>();//新的角点对象
      auto currCorner = uCurrCorner.get();//获取对象指针
      tmpCorners.push_back(std::move(uCurrCorner));//存入临时变量
      // set edge attributes//设置角属性
      prevEdge->setHighCorner(currCorner);
      nextEdge->setLowCorner(currCorner);
      // set currCorner attributes 设置前后边
      currCorner->setPrevEdge(prevEdge);
      currCorner->setNextEdge(nextEdge);
      currCorner->x(prevEdge->high().x());//角点x坐标
      currCorner->y(prevEdge->high().y());//角点y坐标
      //计算并设置角点的类型
      int orient = gtl::orientation(*prevEdge, *nextEdge);// 计算两条边的相对方向
      if (orient == 1) {
        currCorner->setType(frCornerTypeEnum::CONVEX);// 方向为1表示凸角
      } else if (orient == -1) { 
        currCorner->setType(frCornerTypeEnum::CONCAVE);// 方向为-1表示凹角
      } else {
        currCorner->setType(frCornerTypeEnum::UNKNOWN);// 其他情况视为未知
      }
      //根据边的方向设置角点和方向
      if ((prevEdge->getDir() == frDirEnum::N && nextEdge->getDir() == frDirEnum::W) ||
          (prevEdge->getDir() == frDirEnum::W && nextEdge->getDir() == frDirEnum::N)) {
        currCorner->setDir(frCornerDirEnum::NE);
      } else if ((prevEdge->getDir() == frDirEnum::W && nextEdge->getDir() == frDirEnum::S) ||
                 (prevEdge->getDir() == frDirEnum::S && nextEdge->getDir() == frDirEnum::W)) {
        currCorner->setDir(frCornerDirEnum::NW);
      } else if ((prevEdge->getDir() == frDirEnum::S && nextEdge->getDir() == frDirEnum::E) ||
                 (prevEdge->getDir() == frDirEnum::E && nextEdge->getDir() == frDirEnum::S)) {
        currCorner->setDir(frCornerDirEnum::SW);
      } else if ((prevEdge->getDir() == frDirEnum::E && nextEdge->getDir() == frDirEnum::N) ||
                 (prevEdge->getDir() == frDirEnum::N && nextEdge->getDir() == frDirEnum::E)) {
        currCorner->setDir(frCornerDirEnum::SE);
      }

      // set fixed / route status
      if (currCorner->getType() == frCornerTypeEnum::CONVEX) {
        currCorner->setFixed(false);
        for (auto &rect: net->getRectangles(true)[layerNum]) {
          if (isCornerOverlap(currCorner, rect)) {
            currCorner->setFixed(true);
            break;
          }
        }
      } else if (currCorner->getType() == frCornerTypeEnum::CONCAVE) {
        currCorner->setFixed(true);
        auto cornerPt = currCorner->getNextEdge()->low();
        for (auto &rect: net->getRectangles(false)[layerNum]) {
          if (gtl::contains(rect, cornerPt, true) && !gtl::contains(rect, cornerPt, false)) {
            currCorner->setFixed(false);
            break;
          }
        }
      }
      // currCorner->setFixed(prevEdge->isFixed() && nextEdge->isFixed());

      // 设置角之间的连接关系
      if (prevCorner) {
        prevCorner->setNextCorner(currCorner); // 设置前一个角的下一个角为当前角
        currCorner->setPrevCorner(prevCorner); // 设置当前角的前一个角为前一个角
      }
      prevCorner = currCorner; // 更新前一个角为当前角
      prevEdge = nextEdge; // 更新前一个边为当前边
    }
    // 更新首尾角之间的属性-相互连接前后两个角点
    // update attributes between first and last corners
    auto currCorner = tmpCorners.front().get();
    prevCorner->setNextCorner(currCorner);
    currCorner->setPrevCorner(prevCorner);
    // add to polygon corners
    pin->addPolygonCorners(tmpCorners); // 将角点添加到引脚的多边形角列表中，添加临时存储的角点到引脚的多边形角列表
  }
}
//网络中每个引脚的多边形计算并初始化角点。通过对多边形角点的处理，可以更精确地定义多边形的形状
void FlexGCWorker::initNet_pins_polygonCorners(gcNet* net) {
  int numLayers = getDesign()->getTech()->getLayers().size();
  for (int i = 0; i < numLayers; i++) {
    for (auto &pin: net->getPins(i)) {
      /// 调用辅助函数处理该引脚的多边形角点
      initNet_pins_polygonCorners_helper(net, pin.get());
    }
  }
}
//网络中收集固定的最大矩形并将它们保存在一个结构中，用于后续处理。
void FlexGCWorker::initNet_pins_maxRectangles_getFixedMaxRectangles(gcNet* net, vector<set<pair<frPoint, frPoint> > > &fixedMaxRectangles) {
  int numLayers = getDesign()->getTech()->getLayers().size();//层数
  vector<gtl::rectangle_data<frCoord> > rects;//存储矩形的临时对象
  frPoint bp, ep;
  for (int i = 0; i < numLayers; i++) {
    //bool flag     = (getDesign()->getTech()->getLayer(i)->getType() == frLayerTypeEnum::ROUTING);
    //auto minWidth = getDesign()->getTech()->getLayer(i)->getWidth();
    rects.clear();
    gtl::get_max_rectangles(rects, net->getPolygons(i, true));//获取net对应的矩形
    for (auto &rect: rects) {
      // <minwidth max rectangle filtered to match tool and reduce NSMet
      // processed at nsmetal
      //if (flag && (gtl::delta(rect, gtl::HORIZONTAL) < minWidth || gtl::delta(rect, gtl::VERTICAL) < minWidth)) {
      //  continue;
      //}
      
      // 将计算得到的最大矩形转换为点对并插入到对应层的集合中
      fixedMaxRectangles[i].insert(make_pair(frPoint(gtl::xl(rect), gtl::yl(rect)), 
                                             frPoint(gtl::xh(rect), gtl::yh(rect))));
    }
    // for rectangles input --> non-merge scenario// 处理该层中固定的矩形，非合并情况
    for (auto &rect: net->getRectangles(i, true)) {
      // processed at nsmetal
      //if (flag && (gtl::delta(rect, gtl::HORIZONTAL) < minWidth || gtl::delta(rect, gtl::VERTICAL) < minWidth)) {
      //  continue;
      //}
      fixedMaxRectangles[i].insert(make_pair(frPoint(gtl::xl(rect), gtl::yl(rect)), 
                                             frPoint(gtl::xh(rect), gtl::yh(rect))));
    }
  }
}
//标记最大矩形是为 fixed 还是 route
void FlexGCWorker::initNet_pins_maxRectangles_helper(gcNet* net, gcPin* pin, const gtl::rectangle_data<frCoord>& rect, frLayerNum i,
                                                     const vector<set<pair<frPoint, frPoint> > > &fixedMaxRectangles) {
  auto rectangle = make_unique<gcRect>();// 创建一个新的矩形对象
  rectangle->setRect(rect);
  rectangle->setLayerNum(i);
  rectangle->addToPin(pin);
  rectangle->addToNet(net);
  // 检查此矩形是否已存在于固定最大矩形集合中
  if (fixedMaxRectangles[i].find(make_pair(frPoint(gtl::xl(rect), gtl::yl(rect)), 
                                           frPoint(gtl::xh(rect), gtl::yh(rect)))) != 
      fixedMaxRectangles[i].end()) {
    // fixed max rectangles
     // 如果矩形在固定矩形集合中，设置为固定
    rectangle->setFixed(true);
    //cntFixed++;
  } else {
    // route max rectangles
    // 如果矩形不在固定矩形集合中，设置为非固定
    rectangle->setFixed(false);
    //cntRoute++;
  }
  // 将此矩形添加到引脚的最大矩形列表中
  pin->addMaxRectangle(rectangle);
}
//是在处理网络（gcNet）中各引脚的最大矩形，这些矩形用于电路设计中的布局优化。
void FlexGCWorker::initNet_pins_maxRectangles(gcNet* net) {
  //bool enableOutput = true;
  //bool enableOutput = false;
  int numLayers = getDesign()->getTech()->getLayers().size();//层数
  vector<set<pair<frPoint, frPoint> > > fixedMaxRectangles(numLayers);
  // get all fixed max rectangles
  initNet_pins_maxRectangles_getFixedMaxRectangles(net, fixedMaxRectangles);//// 获取所有固定的最大矩形
  
  // gen all max rectangles
  //int cntFixed = 0;
  //int cntRoute = 0;
  vector<gtl::rectangle_data<frCoord> > rects;// 创建一个用于存储最大矩形的临时向量
  for (int i = 0; i < numLayers; i++) {
    //bool flag     = (getDesign()->getTech()->getLayer(i)->getType() == frLayerTypeEnum::ROUTING);
    //auto minWidth = getDesign()->getTech()->getLayer(i)->getWidth();
    for (auto &pin: net->getPins(i)) {
      rects.clear();
      gtl::get_max_rectangles(rects, *(pin->getPolygon()));//// 计算并获取引脚的多边形的最大矩形
      for (auto &rect: rects) {
        // <minwidth max rectangle filtered to match tool and reduce NSMet
        //if (flag && (gtl::delta(rect, gtl::HORIZONTAL) < minWidth || gtl::delta(rect, gtl::VERTICAL) < minWidth)) {
        //  continue;
        //}
        
        // 调用辅助函数处理每个最大矩形--标记fixed or route
        initNet_pins_maxRectangles_helper(net, pin.get(), rect, i, fixedMaxRectangles);
      }
    }
  }
  //if (enableOutput) {
  //  cout <<"#fixed/route max rectangles = " <<cntFixed <<"/" <<cntRoute <<endl;
  //}
}

void FlexGCWorker::initNet(gcNet* net) {
  initNet_pins_polygon(net);//初始化网络中的引脚多边形：
  initNet_pins_polygonEdges(net);//初始化网络中引脚多边形的边
  initNet_pins_polygonCorners(net);//初始化网络中引脚多边形的角点
  initNet_pins_maxRectangles(net);//初始化网络中引脚的最大矩形
}

void FlexGCWorker::initNets() {
  for (auto &uNet: getNets()) {//遍历所有net
    auto net = uNet.get();//获取网络对象
    initNet(net);
  }
}
//实现了在一个集成电路设计工作环境中初始化区域查询系统的功能。
void FlexGCWorker::initRegionQuery() {
  getWorkerRegionQuery().init(getDesign()->getTech()->getLayers().size());
}

// init initializes all nets from frDesign if no drWorker is provided
void FlexGCWorker::init() {
  //bool enableOutput = true;
  bool enableOutput = false;
  // 添加设计中的伪VSS（负电源）和VDD（正电源）网，这些网是浮动的，没有实际连接。
  addNet(design->getTopBlock()->getFakeVSSNet()); //[0] floating VSS
  addNet(design->getTopBlock()->getFakeVDDNet()); //[1] floating VDD
  initDesign();//初始化design相关数据
  initDRWorker();//初始化与DR 相关配置
  if (enableOutput) {
    cout <<"#gcNets = " <<nets.size() <<endl;
    int cntFloating = 0;//计数器
    int cntNet  = 0;
    int cntTerm = 0;
    int cntBlk  = 0;
    for (auto &net: nets) {// 遍历所有网络，根据所有者类型分类统计。
      if (net->getOwner() == nullptr) {//net没有所有者，就认为浮动网络
        cntFloating++;
      } else {
        if (net->getOwner()->typeId() == frcInstTerm || net->getOwner()->typeId() == frcTerm) {
          cntTerm++;//所有者 term
        } else if (net->getOwner()->typeId() == frcInstBlockage || net->getOwner()->typeId() == frcBlockage) {
          cntBlk++;//所有者 blk
        } else if (net->getOwner()->typeId() == frcNet) {
          cntNet++;//所有者 net
        } else {
          cout <<"Error: FlexGCWorker::init unknown type" <<endl;
        }
      }
    }
    cout <<"  #net/term/blk = " <<cntNet <<"/" <<cntTerm <<"/" <<cntBlk <<endl;
  }
  initNets();//初始化网络数据
  initRegionQuery();//初始化区域查询模块，用于空间数据结构和查询优化
}

// init initializes all nets from frDesign if no drWorker is provided
void FlexGCWorker::initPA0() {
  //bool enableOutput = true;
  bool enableOutput = false;
  addNet(design->getTopBlock()->getFakeVSSNet()); //[0] floating VSS
  addNet(design->getTopBlock()->getFakeVDDNet()); //[1] floating VDD
  initDesign();
  initDRWorker();
  if (enableOutput) {
    cout <<"#gcNets = " <<nets.size() <<endl;
    int cntFloating = 0;
    int cntNet  = 0;
    int cntTerm = 0;
    int cntBlk  = 0;
    for (auto &net: nets) {
      if (net->getOwner() == nullptr) {
        cntFloating++;
      } else {
        if (net->getOwner()->typeId() == frcInstTerm || net->getOwner()->typeId() == frcTerm) {
          cntTerm++;
        } else if (net->getOwner()->typeId() == frcInstBlockage || net->getOwner()->typeId() == frcBlockage) {
          cntBlk++;
        } else if (net->getOwner()->typeId() == frcNet) {
          cntNet++;
        } else {
          cout <<"Error: FlexGCWorker::init unknown type" <<endl;
        }
      }
    }
    cout <<"  #net/term/blk = " <<cntNet <<"/" <<cntTerm <<"/" <<cntBlk <<endl;
  }
}

void FlexGCWorker::initPA1() {
  initNets();
  initRegionQuery();
}

void FlexGCWorker::updateGCWorker() {
  if (!getDRWorker()) {
    cout <<"Error: updateGCWorker expects a valid DRWorker" <<endl;
    exit(1);
  }

  // get all frNets, must be sorted by id
  set<frNet*, frBlockObjectComp> fnets;
  for (auto dnet: modifiedDRNets) {
    fnets.insert(dnet->getFrNet());
  }
  modifiedDRNets.clear();

  // get GC patched net as well
  for (auto &patch: pwires) {
    if (patch->hasNet() && patch->getNet()->getFrNet()) {
      fnets.insert(patch->getNet()->getFrNet());
    }
  }

  // start init from dr objs
  for (auto fnet: fnets) {
    auto net = owner2nets[fnet];
    getWorkerRegionQuery().removeFromRegionQuery(net); // delete all region queries
    net->clear();               // delete all pins and routeXXX
    // re-init gcnet from drobjs
    auto vecptr = getDRWorker()->getDRNets(fnet);
    if (vecptr) {
      for (auto dnet: *vecptr) {
        // initialize according to drnets
        for (auto &uConnFig: dnet->getExtConnFigs()) {
          initDRObj(uConnFig.get(), net);
        }
        for (auto &uConnFig: dnet->getRouteConnFigs()) {
          initDRObj(uConnFig.get(), net);
        }
      }
    } else {
      cout <<"Error: updateGCWorker cannot find frNet in DRWorker" <<endl;
      exit(1);
    }  
  }

  // start init from GC patches
  for (auto &patch: pwires) {
    if (patch->hasNet() && patch->getNet()->getFrNet()) {
      auto fnet = patch->getNet()->getFrNet();
      auto net = owner2nets[fnet];
      initDRObj(patch.get(), net);
    }
  }

  // init
  for (auto fnet: fnets) {
    auto net = owner2nets[fnet];
    // init gc net
    initNet(net);
    getWorkerRegionQuery().addToRegionQuery(net);
  }
}

void FlexGCWorker::updateDRNet(drNet* net) {
  modifiedDRNets.push_back(net);
}

// for use incremental mode in pa
void FlexGCWorker::resetPAOwner(frBlockObject* owner) {
  auto net = owner2nets[owner];
  getWorkerRegionQuery().removeFromRegionQuery(net); // delete all region queries
  net->clear();               // delete all pins and routeXXX
  if (modifiedOwnersSet.find(owner) == modifiedOwnersSet.end()) {
    modifiedOwners.push_back(owner);
    modifiedOwnersSet.insert(owner);
  }
}

void FlexGCWorker::initPA2() {
  for (auto &owner: modifiedOwners) {
    auto net = owner2nets[owner];
    // init gc net
    initNet(net);
    getWorkerRegionQuery().addToRegionQuery(net);
  }
  modifiedOwners.clear();
  modifiedOwnersSet.clear();
}
