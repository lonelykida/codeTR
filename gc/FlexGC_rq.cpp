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

using namespace std;
using namespace fr;

void FlexGCWorkerRegionQuery::addPolygonEdge(gcSegment* edge) {
  segment_t boosts(point_t(edge->low().x(), edge->low().y()), point_t(edge->high().x(), edge->high().y()));
  polygon_edges[edge->getLayerNum()].insert(make_pair(boosts, edge));
}
//负责将一个多边形边缘（由gcSegment类表示）添加到对应层的存储结构中。
void FlexGCWorkerRegionQuery::addPolygonEdge(gcSegment* edge, vector<vector<pair<segment_t, gcSegment*> > > &allShapes) {
  segment_t boosts(point_t(edge->low().x(), edge->low().y()), point_t(edge->high().x(), edge->high().y()));
  allShapes[edge->getLayerNum()].push_back(make_pair(boosts, edge));
}

void FlexGCWorkerRegionQuery::addMaxRectangle(gcRect* rect) {
  box_t boostb(point_t(gtl::xl(*rect), gtl::yl(*rect)), point_t(gtl::xh(*rect), gtl::yh(*rect)));
  max_rectangles[rect->getLayerNum()].insert(make_pair(boostb, rect));
}
//将一个矩形（由 gcRect 类型的指针表示）添加到一个组织层级结构中，每个层包含相关矩形的集合。
void FlexGCWorkerRegionQuery::addMaxRectangle(gcRect* rect, vector<vector<rq_rptr_value_t<gcRect> > > &allShapes) {
  box_t boostb(point_t(gtl::xl(*rect), gtl::yl(*rect)), point_t(gtl::xh(*rect), gtl::yh(*rect)));
  allShapes[rect->getLayerNum()].push_back(make_pair(boostb, rect));
}

void FlexGCWorkerRegionQuery::removePolygonEdge(gcSegment* edge) {
  segment_t boosts(point_t(edge->low().x(), edge->low().y()), point_t(edge->high().x(), edge->high().y()));
  polygon_edges[edge->getLayerNum()].remove(make_pair(boosts, edge));
}

void FlexGCWorkerRegionQuery::removeMaxRectangle(gcRect* rect) {
  box_t boostb(point_t(gtl::xl(*rect), gtl::yl(*rect)), point_t(gtl::xh(*rect), gtl::yh(*rect)));
  max_rectangles[rect->getLayerNum()].remove(make_pair(boostb, rect));
}

void FlexGCWorkerRegionQuery::queryPolygonEdge(const box_t &box, frLayerNum layerNum, vector<pair<segment_t, gcSegment*> > &result) {
  polygon_edges[layerNum].query(bgi::intersects(box), back_inserter(result));
}

void FlexGCWorkerRegionQuery::queryPolygonEdge(const frBox &box, frLayerNum layerNum, vector<pair<segment_t, gcSegment*> > &result) {
  box_t boostb(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  queryPolygonEdge(boostb, layerNum, result);
}

void FlexGCWorkerRegionQuery::queryMaxRectangle(const box_t &box, frLayerNum layerNum, std::vector<rq_rptr_value_t<gcRect> > &result) {
  max_rectangles[layerNum].query(bgi::intersects(box), back_inserter(result));
}

void FlexGCWorkerRegionQuery::queryMaxRectangle(const frBox &box, frLayerNum layerNum, std::vector<rq_rptr_value_t<gcRect> > &result) {
  box_t boostb(point_t(box.left(), box.bottom()), point_t(box.right(), box.top()));
  queryMaxRectangle(boostb, layerNum, result);
}

void FlexGCWorkerRegionQuery::queryMaxRectangle(const gtl::rectangle_data<frCoord> &box, frLayerNum layerNum, std::vector<rq_rptr_value_t<gcRect> > &result) {
  box_t boostb(point_t(gtl::xl(box), gtl::yl(box)), point_t(gtl::xh(box), gtl::yh(box)));
  queryMaxRectangle(boostb, layerNum, result);
}
//初始化了一个区域查询对象，用于存储和管理集成电路设计中的多边形边和最大矩形数据。
void FlexGCWorkerRegionQuery::init(int numLayers) {
  bool enableOutput = false;
  //bool enableOutput = true;
  // 清空并重新初始化存储多边形边和最大矩形的向量
  polygon_edges.clear();
  polygon_edges.resize(numLayers);
  max_rectangles.clear();
  max_rectangles.resize(numLayers);
  // 创建用于存储所有多边形边和最大矩形的临时向量
  vector<vector<pair<segment_t, gcSegment*> > > allPolygonEdges(numLayers);
  vector<vector<rq_rptr_value_t<gcRect> > >  allMaxRectangles(numLayers);

  int cntPolygonEdge = 0;  // 计数器，用于记录多边形边的数量
  int cntMaxRectangle = 0;  // 计数器，用于记录最大矩形的数量
  for (auto &net: getGCWorker()->getNets()) {
    for (auto &pins: net->getPins()) {
      for (auto &pin: pins) {
        for (auto &edges: pin->getPolygonEdges()) {
          for (auto &edge: edges) {
            addPolygonEdge(edge.get(), allPolygonEdges);// 将多边形边添加到临时存储
            cntPolygonEdge++;
          }
        }  
        for (auto &rect: pin->getMaxRectangles()) {
          addMaxRectangle(rect.get(), allMaxRectangles);// 将最大矩形添加到临时存储
          cntMaxRectangle++;
        }
      }
    }
  }

  int cntRTPolygonEdge  = 0;
  int cntRTMaxRectangle = 0;
  // 将收集的多边形边和最大矩形构建成空间索引
  for (int i = 0; i < numLayers; i++) {
    polygon_edges[i]  = boost::move(bgi::rtree<pair<segment_t, gcSegment*>, bgi::quadratic<16> >(allPolygonEdges[i]));
    max_rectangles[i] = boost::move(bgi::rtree<rq_rptr_value_t<gcRect>, bgi::quadratic<16> >(allMaxRectangles[i]));
    cntRTPolygonEdge += polygon_edges[i].size();  // 更新空间索引多边形边的数量
    cntRTMaxRectangle += max_rectangles[i].size();  // 更新空间索引最大矩形的数量
  }
  if (enableOutput) {
    for (int i = 0; i < numLayers; i++) {
      frPoint bp, ep;
      double dbu = getGCWorker()->getDesign()->getTopBlock()->getDBUPerUU();
      for (auto &[seg, ptr]: polygon_edges[i]) {
        //ptr->getPoints(bp, ep);
        cout <<"polyEdge ";
        if (ptr->isFixed()) {
          cout <<"FIXED";
        } else {
          cout <<"ROUTE";
        }
        cout <<" @(" <<ptr->low().x() / dbu <<", " <<ptr->low().y() / dbu <<") (" <<ptr->high().x() / dbu <<", " <<ptr->high().y() / dbu <<") "
             <<getGCWorker()->getDesign()->getTech()->getLayer(i)->getName() <<" ";
        auto owner = ptr->getNet()->getOwner();
        if (owner == nullptr) {
          cout <<" FLOATING";
        } else {
          if (owner->typeId() == frcNet) {
            cout <<static_cast<frNet*>(owner)->getName();
          } else if (owner->typeId() == frcInstTerm) {
            cout <<static_cast<frInstTerm*>(owner)->getInst()->getName() <<"/" 
                 <<static_cast<frInstTerm*>(owner)->getTerm()->getName();
          } else if (owner->typeId() == frcTerm) {
            cout <<"PIN/" <<static_cast<frTerm*>(owner)->getName();
          } else if (owner->typeId() == frcInstBlockage) {
            cout <<static_cast<frInstBlockage*>(owner)->getInst()->getName() <<"/OBS";
          } else if (owner->typeId() == frcBlockage) {
            cout <<"PIN/OBS";
          } else {
            cout <<"UNKNOWN";
          }
        }
        cout <<endl;
      }
    }
  }
  
  if (enableOutput) {
    for (int i = 0; i < numLayers; i++) {
      double dbu = getGCWorker()->getDesign()->getTopBlock()->getDBUPerUU();
      for (auto &[box, ptr]: max_rectangles[i]) {
        cout <<"maxRect ";
        if (ptr->isFixed()) {
          cout <<"FIXED";
        } else {
          cout <<"ROUTE";
        }
        cout <<" @(" <<gtl::xl(*ptr) / dbu <<", " <<gtl::yl(*ptr) / dbu <<") (" <<gtl::xh(*ptr) / dbu <<", " <<gtl::yh(*ptr) / dbu <<") "
             <<getGCWorker()->getDesign()->getTech()->getLayer(i)->getName() <<" ";
        auto owner = ptr->getNet()->getOwner();
        if (owner == nullptr) {
          cout <<" FLOATING";
        } else {
          if (owner->typeId() == frcNet) {
            cout <<static_cast<frNet*>(owner)->getName();
          } else if (owner->typeId() == frcInstTerm) {
            cout <<static_cast<frInstTerm*>(owner)->getInst()->getName() <<"/" 
                 <<static_cast<frInstTerm*>(owner)->getTerm()->getName();
          } else if (owner->typeId() == frcTerm) {
            cout <<"PIN/" <<static_cast<frTerm*>(owner)->getName();
          } else if (owner->typeId() == frcInstBlockage) {
            cout <<static_cast<frInstBlockage*>(owner)->getInst()->getName() <<"/OBS";
          } else if (owner->typeId() == frcBlockage) {
            cout <<"PIN/OBS";
          } else {
            cout <<"UNKNOWN";
          }
        }
        cout <<endl;
      }
    }
  }

  if (enableOutput) {
    cout <<"gc region query #poly_edges/max_rects/rt_poly_edges/rt_max_rect = " 
         <<cntPolygonEdge <<"/" <<cntMaxRectangle <<"/" <<cntRTPolygonEdge <<"/" <<cntRTMaxRectangle 
         <<endl;
  }
}

void FlexGCWorkerRegionQuery::addToRegionQuery(gcNet* net) {
  for (auto &pins: net->getPins()) {
    for (auto &pin: pins) {
      for (auto &edges: pin->getPolygonEdges()) {
        for (auto &edge: edges) {
          addPolygonEdge(edge.get());
        }
      }  
      for (auto &rect: pin->getMaxRectangles()) {
        addMaxRectangle(rect.get());
      }
    }
  }
}

void FlexGCWorkerRegionQuery::removeFromRegionQuery(gcNet* net) {
  for (auto &pins: net->getPins()) {
    for (auto &pin: pins) {
      for (auto &edges: pin->getPolygonEdges()) {
        for (auto &edge: edges) {
          removePolygonEdge(edge.get());
        }
      }  
      for (auto &rect: pin->getMaxRectangles()) {
        removeMaxRectangle(rect.get());
      }
    }
  }
}

