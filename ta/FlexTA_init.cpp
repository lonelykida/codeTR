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

#include "ta/FlexTA.h"
#include "dr/FlexDR_init.cpp"
using namespace std;
using namespace fr;


//初始化轨道
void FlexTAWorker::initTracks() {
  //bool enableOutput = true;
  bool enableOutput = false;
  nets.clear(); //1.清空所有net
  //tracks.clear();
  //tracks.resize(getDesign()->getTech()->getLayers().size());
  trackLocs.clear();  //2.清空所有轨道
  //3.将轨道集合大小重新设置为层的数量
  trackLocs.resize(getDesign()->getTech()->getLayers().size());
  //4.在每层的轨道集合中初始化轨道坐标集，轨道数量设置为层的数量-二维数组，一行就是一层的轨道坐标
  vector<set<frCoord> > trackCoordSets(getDesign()->getTech()->getLayers().size());
  // uPtr for tp
  //对每个布线金属层，计算该层的轨道数量，并将其放入trackCoordSets集合中
  for (int lNum = 0; lNum < (int)getDesign()->getTech()->getLayers().size(); lNum++) {  //对每一个金属层
    auto layer = getDesign()->getTech()->getLayer(lNum);  //获取当前层的指针
    if (layer->getType() != frLayerTypeEnum::ROUTING) {  //如果不是布线层则跳过
      continue;
    }
    if (layer->getDir() != getDir()) {  //如果不是当前方向的布线层则跳过
      continue;
    }
    for (auto &tp: getDesign()->getTopBlock()->getTrackPatterns(lNum)) {  //对当前层的所有轨道模式
      if ((getDir() == frcHorzPrefRoutingDir && tp->isHorizontal() == false) ||
          (getDir() == frcVertPrefRoutingDir && tp->isHorizontal() == true)) {//如果不是当前方向的轨道模式则跳过
        if (enableOutput) {
          cout <<"TRACKS " <<(tp->isHorizontal() ? string("X ") : string("Y "))
               <<tp->getStartCoord() <<" DO " <<tp->getNumTracks() <<" STEP "
               <<tp->getTrackSpacing() <<" LAYER " <<tp->getLayerNum() 
               <<" ;" <<endl;
        }
        //是否是水平轨道
        bool isH = (getDir() == frcHorzPrefRoutingDir);
        //计算了基于轨道模式起始坐标和设计边界的轨道数量
        frCoord tempCoord1 = (isH ? getRouteBox().bottom() : getRouteBox().left());
        frCoord tempCoord2 = (isH ? getRouteBox().top()    : getRouteBox().right());
        int trackNum = (tempCoord1 - tp->getStartCoord()) / (int)tp->getTrackSpacing();
        if (trackNum < 0) {
          trackNum = 0;
        }
        //可能计算的时候会因为精度问题少一根轨道，就需要通过判断来将缺少的轨道补上
        if (trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < tempCoord1) {
          trackNum++;
        }
        //遍历每个轨道，并将轨道坐标插入到轨道坐标集合中
        for (; trackNum < (int)tp->getNumTracks() && trackNum * (int)tp->getTrackSpacing() + tp->getStartCoord() < tempCoord2; trackNum++) {
          frCoord trackCoord = trackNum * tp->getTrackSpacing() + tp->getStartCoord();
          //cout <<"TRACKLOC " <<trackCoord * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<endl;
          trackCoordSets[lNum].insert(trackCoord);
        }
      }
    }
  }
  //对轨道集合trackCoordSets中的每个轨道坐标放入tracks中(trackCoordSets会从小到大排序)
  for (int i = 0; i < (int)trackCoordSets.size(); i++) {
    if (enableOutput) {
      cout <<"lNum " <<i <<":";
    }
    //对每层的轨道坐标集合输出
    for (auto coord: trackCoordSets[i]) {
      if (enableOutput) {
        cout <<" " <<coord * 1.0 / getDesign()->getTopBlock()->getDBUPerUU();
      }
      //taTrack t;
      //t.setTrackLoc(coord);
      //tracks[i].push_back(std::move(t));
      trackLocs[i].push_back(coord);
    }
    if (enableOutput) {
      cout <<endl;
    }
  }
}

// use prefAp, otherwise return false
//使用优先访问点，否则返回false
bool FlexTAWorker::initIroute_helper_pin(frGuide* guide, frCoord &maxBegin, frCoord &minEnd, 
                                         set<frCoord> &downViaCoordSet, set<frCoord> &upViaCoordSet,
                                         int &wlen, frCoord &wlen2) {
  bool enableOutput = false;
  frPoint bp, ep; //获取当前guide的起终点
  guide->getPoints(bp, ep);
  if (!(bp == ep)) {  //若起终点不相等，则不符合要求 - 应该是说明不是pin
    return false;
  }

  auto net      = guide->getNet();  //获取当前guide的线网
  auto layerNum = guide->getBeginLayerNum();  //获取guide的层号
  bool isH      = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);//当前布线方向是否水平
  bool hasDown  = false;//是否有下层iroute = false
  bool hasUp    = false;//是否有上层 = false

  vector<frGuide*> nbrGuides; //nbrGuide的vector，存放的是guide
  auto rq = getRegionQuery(); //查询区域
  frBox box;  //当前guide的边界框box
  box.set(bp, bp);  //以当前guide的起终点为布线边界框

  //下边的代码在看除了当前层以外，该net是否还有下层及上层的iroute，用hasDown和hasUp来表示
  nbrGuides.clear();  //初始化nbrGuides为空
  if (layerNum - 2 >= BOTTOM_ROUTING_LAYER) { //从当前层往下查找的范围，判断当前net是否有在下层的iroute -- BOTTOM_ROUTING_LAYER = 2
    rq->queryGuide(box, layerNum - 2, nbrGuides); //查询不同层布线区域的guide，放到nbrGuides中
    for (auto &nbrGuide: nbrGuides) { //遍历nbrGuides中的其他guide
      if (nbrGuide->getNet() == net) {//在guide中找属于net的线网，若找到，则说明该线网会去下层，即有下层iroute
        hasDown = true; //有下层
        break;
      }
    }
  } 
  nbrGuides.clear();  //初始化nbrGuides为空
  if (layerNum + 2 < (int)design->getTech()->getLayers().size()) {  //判断当前net是否有上层的iroute
    rq->queryGuide(box, layerNum + 2, nbrGuides); //查询布线区域
    for (auto &nbrGuide: nbrGuides) { //遍历nbrGuides
      if (nbrGuide->getNet() == net) {//若nbrGuide的线网等于guide的线网，说明在上层也有当前net的iroute
        hasUp = true; //有上层
        break;
      }
    }
  } 

  //查询pin的结果
  vector<frBlockObject*> result;
  box.set(bp, bp);  //查询区域
  rq->queryGRPin(box, result);  //查询区域内的GRpin，并放到result中
  frTransform instXform; // (0,0), frcR0   //实例X的形式
  frTransform shiftXform;//改变X的形式
  frTerm* trueTerm = nullptr; //真实的Term指针

  //string  name;
  //下边的for循环先找存放pin的真实的Term指针，其实就是在找用于布线连接的pin的结构，将其赋值给trueTerm变量
  //因为pin的结构都是放在Term里边的
  for (auto &term: result) {  //对result中的每一个pin对象term
    //bool hasInst = false;
    frInst* inst = nullptr;   //实例指针inst，初始化为nullptr
    if (term->typeId() == frcInstTerm) {  //若当前pin的类型是InstTerm
      if (static_cast<frInstTerm*>(term)->getNet() != net) {  //若该pin不是当前net的线网，则跳过
        continue;
      }
      //hasInst = true; 否则若该pin是当前net的线网，则：
      inst = static_cast<frInstTerm*>(term)->getInst(); //先获取当前pin的实例
      inst->getTransform(shiftXform); //接着获取转换形式
      shiftXform.set(frOrient(frcR0));//将转换形式的方向设置为R0
      inst->getUpdatedXform(instXform);//对当前实例的转换形式进行更新
      trueTerm = static_cast<frInstTerm*>(term)->getTerm(); //获取真实的Term指针
    } else if (term->typeId() == frcTerm) { //若当前pin的类型是Term
      if (static_cast<frTerm*>(term)->getNet() != net) {  //若当前term不是当前net的线网，则跳过
        continue;
      }
      trueTerm = static_cast<frTerm*>(term);  //否则直接将当前pin的Term指针赋值给trueTerm
    }
    
    //若找到属于当前线网的trueTerm后
    if (trueTerm) {
      int pinIdx = 0; //pin的索引
      int pinAccessIdx = (inst) ? inst->getPinAccessIdx() : -1; //pin的连接索引，若当前pin有实例，则获取其连接索引
      for (auto &pin: trueTerm->getPins()) {  //从trueTerm中获取每一个pin
        frAccessPoint* ap = nullptr;    //pin的AccessPoint指针
        if (inst) { //若当前pin有实例，则获取其AccessPoint的第pinIdx个pin
          ap = (static_cast<frInstTerm*>(term)->getAccessPoints())[pinIdx];
        }
        //若当前pin没有访问点，则找下一个pin
        if (!pin->hasPinAccess()) {
          continue;
        }
        //若当前pin的访问点索引是-1，则找下一个pin
        if (pinAccessIdx == -1) {
          continue;
        }
        //若当前pin干脆就没有访问点，则找下一个pin
        if (ap == nullptr) {
          continue;
        }
        frPoint apBp; //AccessPoint的Begin坐标
        ap->getPoint(apBp);
        if (enableOutput) {
          cout <<" (" <<apBp.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", "
                      <<apBp.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") origin";
        }
        //访问点所在的层
        auto bNum = ap->getLayerNum();
        //根据转换形式将AccessPoint的坐标进行转换
        apBp.transform(shiftXform);
        //若访问点就在当前层的布线区域内，则更新wlen，wlen2，maxBegin，minEnd
        if (layerNum == bNum && getRouteBox().contains(apBp)) {
          wlen2 = isH ? apBp.y() : apBp.x();    //更新wlen2，其值为AccessPoint的y(若水平)坐标或x(若垂直)坐标
          maxBegin = isH ? apBp.x() : apBp.y(); //更新maxBegin，其值为AccessPoint的x(若水平)坐标或y(若垂直)坐标
          minEnd   = isH ? apBp.x() : apBp.y();
          wlen = 0; //更新wlen，其值为0
          if (hasDown) {  //若有下层的iroute，则更新downViaCoordSet(下层通孔坐标集)
            downViaCoordSet.insert(maxBegin);
          }
          if (hasUp) {    //若有上层的iroute，则更新upViaCoordSet(上层通孔坐标集)
            upViaCoordSet.insert(maxBegin);
          }
          return true;  //返回
        }
        pinIdx++;       //若访问点不在当前层，则更新pinIdx
      }
    }
  }
  //若对result中的每个term，都找不到该net的trueTerm，说明在该层上找不到该net的pin(也就是访问点)，则返回false
  return false;
}
//初始化Iroute的辅助函数
void FlexTAWorker::initIroute_helper(frGuide* guide, frCoord &maxBegin, frCoord &minEnd, 
                                     set<frCoord> &downViaCoordSet, set<frCoord> &upViaCoordSet,
                                     int &wlen, frCoord &wlen2) {
  //先初始化pin，若无pin再初始化generic
  if (!initIroute_helper_pin(guide, maxBegin, minEnd, downViaCoordSet, upViaCoordSet, wlen, wlen2)) {
    initIroute_helper_generic(guide, maxBegin, minEnd, downViaCoordSet, upViaCoordSet, wlen, wlen2);
  }
}


void FlexTAWorker::initIroute_helper_generic_helper(frGuide* guide, frCoord &wlen2) {
  bool enableOutput = false;

  frPoint bp, ep;
  guide->getPoints(bp, ep);
  auto net = guide->getNet();
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);

  auto rq = getRegionQuery();
  vector<frBlockObject*> result;

  frBox box;
  box.set(bp, bp);
  rq->queryGRPin(box, result);
  if (!(ep == bp)) {
    box.set(ep, ep);
    rq->queryGRPin(box, result);
  }
  frTransform instXform; // (0,0), frcR0
  frTransform shiftXform;
  frTerm* trueTerm = nullptr;
  //string  name;
  for (auto &term: result) {
    //bool hasInst = false;
    frInst* inst = nullptr;
    if (term->typeId() == frcInstTerm) {
      if (static_cast<frInstTerm*>(term)->getNet() != net) {
        continue;
      }
      //hasInst = true;
      inst = static_cast<frInstTerm*>(term)->getInst();
      inst->getTransform(shiftXform);
      shiftXform.set(frOrient(frcR0));
      inst->getUpdatedXform(instXform);
      trueTerm = static_cast<frInstTerm*>(term)->getTerm();
    } else if (term->typeId() == frcTerm) {
      if (static_cast<frTerm*>(term)->getNet() != net) {
        continue;
      }
      trueTerm = static_cast<frTerm*>(term);
    }
    if (trueTerm) {
      int pinIdx = 0;
      int pinAccessIdx = (inst) ? inst->getPinAccessIdx() : -1;
      for (auto &pin: trueTerm->getPins()) {
        frAccessPoint* ap = nullptr;
        if (inst) {
          ap = (static_cast<frInstTerm*>(term)->getAccessPoints())[pinIdx];
        }
        if (!pin->hasPinAccess()) {
          continue;
        }
        if (pinAccessIdx == -1) {
          continue;
        }
        if (ap == nullptr) {
          continue;
        }
        frPoint apBp;
        ap->getPoint(apBp);
        if (enableOutput) {
          cout <<" (" <<apBp.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", "
                      <<apBp.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") origin";
        }
        apBp.transform(shiftXform);
        if (getRouteBox().contains(apBp)) {
          wlen2 = isH ? apBp.y() : apBp.x();
          return;
        }
        pinIdx++;
      }
    }
    ; // to do @@@@@
    wlen2 = 0;
  }
}

void FlexTAWorker::initIroute_helper_generic(frGuide* guide, frCoord &minBegin, frCoord &maxEnd, 
                                             set<frCoord> &downViaCoordSet, set<frCoord> &upViaCoordSet,
                                             int &wlen, frCoord &wlen2) {
  auto    net         = guide->getNet();
  auto    layerNum    = guide->getBeginLayerNum();
  bool    hasMinBegin = false;
  bool    hasMaxEnd   = false;
          minBegin    = std::numeric_limits<frCoord>::max();
          maxEnd      = std::numeric_limits<frCoord>::min();
          wlen        = 0;
          //wlen2       = std::numeric_limits<frCoord>::max();
  bool    isH         = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  downViaCoordSet.clear();
  upViaCoordSet.clear();
  frPoint nbrBp, nbrEp;
  frPoint nbrSegBegin, nbrSegEnd;
  
  frPoint bp, ep;
  guide->getPoints(bp, ep);
  frPoint cp;
  // layerNum in FlexTAWorker
  vector<frGuide*> nbrGuides;
  auto rq = getRegionQuery();
  frBox box;
  for (int i = 0; i < 2; i++) {
    nbrGuides.clear();
    // check left
    if (i == 0) {
      box.set(bp, bp);
      cp = bp;
    // check right
    } else {
      box.set(ep, ep);
      cp = ep;
    }
    if (layerNum - 2 >= BOTTOM_ROUTING_LAYER) {
      rq->queryGuide(box, layerNum - 2, nbrGuides);
    } 
    if (layerNum + 2 < (int)design->getTech()->getLayers().size()) {
      rq->queryGuide(box, layerNum + 2, nbrGuides);
    } 
    for (auto &nbrGuide: nbrGuides) {
      if (nbrGuide->getNet() == net) {
        nbrGuide->getPoints(nbrBp, nbrEp);
        if (!nbrGuide->hasRoutes()) {
          // via location assumed in center
          auto psLNum = nbrGuide->getBeginLayerNum();
          if (psLNum == layerNum - 2) {
            downViaCoordSet.insert((isH ? nbrBp.x() : nbrBp.y()));
          } else {
            upViaCoordSet.insert((isH ? nbrBp.x() : nbrBp.y()));
          }
        } else {
          for (auto &connFig: nbrGuide->getRoutes()) {
            if (connFig->typeId() == frcPathSeg) {
              auto obj = static_cast<frPathSeg*>(connFig.get());
              obj->getPoints(nbrSegBegin, nbrSegEnd);
              auto psLNum = obj->getLayerNum();
              if (i == 0) {
                minBegin = min(minBegin, (isH ? nbrSegBegin.x() : nbrSegBegin.y()));
                hasMinBegin = true;
              } else {
                maxEnd = max(maxEnd, (isH ? nbrSegBegin.x() : nbrSegBegin.y()));
                hasMaxEnd = true;
              }
              //if (nbrBp == nbrEp) {
              //  wlen2 = isH ? ((nbrSegBegin.y() + nbrSegEnd().y()) / 2) : ((nbrSegBegin.x() + nbrSegEnd().x()) / 2)
              //}
              if (psLNum == layerNum - 2) {
                downViaCoordSet.insert((isH ? nbrSegBegin.x() : nbrSegBegin.y()));
              } else {
                upViaCoordSet.insert((isH ? nbrSegBegin.x() : nbrSegBegin.y()));
              }
            }
          }
        }
        if (cp == nbrEp) {
          wlen -= 1;
        }
        if (cp == nbrBp) {
          wlen += 1;
        }
      }
    }
  }

  if (!hasMinBegin) {
    minBegin = (isH ? bp.x() : bp.y());
  }
  if (!hasMaxEnd) {
    maxEnd = (isH ? ep.x() : ep.y());
  }
  if (minBegin > maxEnd) {
    swap(minBegin, maxEnd);
  }
  if (minBegin == maxEnd) {
    maxEnd += 1;
  }

  // wlen2 purely depends on ap regardless of track
  initIroute_helper_generic_helper(guide, wlen2);
}
//从guide中初始化iroute
void FlexTAWorker::initIroute(frGuide *guide) {
  bool enableOutput = false;
  //bool enableOutput = true;
  auto iroute = make_unique<taPin>(); //iroute指针
  iroute->setGuide(guide);  //将guide文件给iroute
  frBox guideBox; //存储guide区域的边框
  guide->getBBox(guideBox); //获取guide区域的边框
  auto layerNum = guide->getBeginLayerNum();  //获取当前guide所属金属层
  bool isExt = !(getRouteBox().contains(guideBox)); //判断当前guide区域是否在布线区域中
  if (isExt) {  //若guide在布线区域中
    // extIroute empty, skip
    if (guide->getRoutes().empty()) { //若当前guide没有需要布线的形状，即无布线需求，则不做任何操作
      return;
    }
    if (enableOutput) {
      double dbu = getDesign()->getTopBlock()->getDBUPerUU();
      cout <<"ext@(" <<guideBox.left() / dbu  <<", " <<guideBox.bottom() / dbu <<") ("
                     <<guideBox.right() / dbu <<", " <<guideBox.top()    / dbu <<")" <<endl;
    }
  }
  
  frCoord maxBegin, minEnd; //最大起始点和最小终止点
  //上/下通孔的坐标集合
  set<frCoord> downViaCoordSet, upViaCoordSet;
  int wlen = 0; //线长1初始化为0
  frCoord wlen2 = std::numeric_limits<frCoord>::max();  //线长2初始化为无穷大
  //根据guide更新maxBegin,minEnd,downViaCoordSet,upViaCoordSet,wlen,wlen2等参数
  initIroute_helper(guide, maxBegin, minEnd, downViaCoordSet, upViaCoordSet, wlen, wlen2);

  frCoord trackLoc = 0; //标记轨道坐标 
  frPoint segBegin, segEnd; //标记线段起点和终点
  bool    isH         = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);//判断当前布线方向是否为水平
  // set trackIdx
  if (!isInitTA()) {  //若当前iroute不是初始化iroute
    for (auto &connFig: guide->getRoutes()) { //对当前guide的布线路线进行遍历
      if (connFig->typeId() == frcPathSeg) {  //若当前布线路线为线段
        auto obj = static_cast<frPathSeg*>(connFig.get());  //获取该线段
        obj->getPoints(segBegin, segEnd);     //获取线段起点和终点
        trackLoc = (isH ? segBegin.y() : segBegin.x()); //根据线段的布线方向获取轨道坐标
        //auto psLNum = obj->getLayerNum();
        //int idx1 = 0;
        //int idx2 = 0;
        //getTrackIdx(trackLoc, psLNum, idx1, idx2);
        //iroute.setTrackIdx(idx1);
      }
    }
  } else {
    trackLoc = 0; //若当前iroute未初始化，则设置轨道坐标为0
  }

  unique_ptr<taPinFig> ps = make_unique<taPathSeg>(); //获取ta的线段对象
  auto rptr = static_cast<taPathSeg*>(ps.get());
  if (isH) {  //根据水平或垂直方向设置线段起点和终点
    rptr->setPoints(frPoint(maxBegin, trackLoc), frPoint(minEnd, trackLoc));
  } else {
    rptr->setPoints(frPoint(trackLoc, maxBegin), frPoint(trackLoc, minEnd));
  }
  rptr->setLayerNum(layerNum);  //设置轨道所属金属层
  rptr->setStyle(getDesign()->getTech()->getLayer(layerNum)->getDefaultSegStyle());//设置线段的类型
  // owner set when add to taPin - 当加入taPin时，设置owner
  iroute->addPinFig(ps);  //将轨道分配的线段对象加入iroute

  //iroute.setCoords(maxBegin, minEnd);
  //for (auto coord: upViaCoordSet) {
  //  iroute.addViaCoords(coord, true);
  //}
  //for (auto coord: downViaCoordSet) {
  //  iroute.addViaCoords(coord, false);
  //}
  for (auto coord: upViaCoordSet) { //对上层通孔的坐标进行遍历
    //获取上一层的通孔
    unique_ptr<taPinFig> via = make_unique<taVia>(getDesign()->getTech()->getLayer(layerNum + 1)->getDefaultViaDef());
    auto rViaPtr = static_cast<taVia*>(via.get());  //获取通孔指针
    rViaPtr->setOrigin(isH ? frPoint(coord, trackLoc) : frPoint(trackLoc, coord));//设置通孔的中心点
    iroute->addPinFig(via); //将当前通孔添加到当前iroute中
  }
  for (auto coord: downViaCoordSet) {//对下层通孔同样执行该操作
    unique_ptr<taPinFig> via = make_unique<taVia>(getDesign()->getTech()->getLayer(layerNum - 1)->getDefaultViaDef());
    auto rViaPtr = static_cast<taVia*>(via.get());
    rViaPtr->setOrigin(isH ? frPoint(coord, trackLoc) : frPoint(trackLoc, coord));
    iroute->addPinFig(via);
  }
  iroute->setWlenHelper(wlen);  //设置当前iroute的线长wlen
  if (wlen2 < std::numeric_limits<frCoord>::max()) {
    iroute->setWlenHelper2(wlen2);
  }
  addIroute(iroute, isExt); //将当前iroute加入iroutes
  //iroute.setId(iroutes.size()); // set id
  //iroutes.push_back(iroute);
}


//初始化iroute
void FlexTAWorker::initIroutes() {
  //bool enableOutput = true;
  bool enableOutput = false;
  vector<rq_rptr_value_t<frGuide> > result;//存放guide查询结果
  auto regionQuery = getRegionQuery();  //获取查询区域
  //对每一层进行查询
  for (int lNum = 0; lNum < (int)getDesign()->getTech()->getLayers().size(); lNum++) {
    auto layer = getDesign()->getTech()->getLayer(lNum);//获取当前层
    if (layer->getType() != frLayerTypeEnum::ROUTING) {//若当前不是布线层则跳过
      continue;
    }
    if (layer->getDir() != getDir()) {  //若当前层布线方向与目标优先布线方向不同则跳过
      continue;
    }
    result.clear(); //初始化结果集
    regionQuery->queryGuide(getExtBox(), lNum, result);//从查询区域中查询每一层的guide
    //cout <<endl <<"query1:" <<endl;
    //遍历查询结果，其中boostb是边界区域，guide是guide文件
    for (auto &[boostb, guide]: result) {
      //获取每个 frGuide 对象的两个起终点，并输出这些点的坐标和相关网络的名称
      frPoint pt1, pt2;
      guide->getPoints(pt1, pt2);
      //if (enableOutput) {
      //  cout <<"found guide (" 
      //       <<pt1.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
      //       <<pt1.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") (" 
      //       <<pt2.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<", " 
      //       <<pt2.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() <<") " 
      //       <<guide->getNet()->getName() << "\n";
      //}
      //guides.push_back(guide);
      //cout <<endl;

      //根据guide文件初始化iroute
      initIroute(guide);
    }
    //sort(guides.begin(), guides.end(), [](const frGuide *a, const frGuide *b) {return *a < *b;});
  }
  if (enableOutput) {
    bool   isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    for (auto &iroute: iroutes) {
      frPoint bp, ep;
      frCoord bc, ec, trackLoc;
      cout <<iroute->getId() <<" " <<iroute->getGuide()->getNet()->getName();
      auto guideLNum = iroute->getGuide()->getBeginLayerNum();
      for (auto &uPinFig: iroute->getFigs()) {
        if (uPinFig->typeId() == tacPathSeg) {
          auto obj = static_cast<taPathSeg*>(uPinFig.get());
          obj->getPoints(bp, ep);
          bc = isH ? bp.x() : bp.y();
          ec = isH ? ep.x() : ep.y();
          trackLoc = isH ? bp.y() : bp.x();
          cout <<" (" <<bc / dbu <<"-->" <<ec / dbu <<"), len@" <<(ec - bc) / dbu <<", track@" <<trackLoc / dbu
               <<", " <<getDesign()->getTech()->getLayer(iroute->getGuide()->getBeginLayerNum())->getName();
        } else if (uPinFig->typeId() == tacVia) {
          auto obj = static_cast<taVia*>(uPinFig.get());
          auto cutLNum = obj->getViaDef()->getCutLayerNum();
          obj->getOrigin(bp);
          bc = isH ? bp.x() : bp.y();
          cout <<string((cutLNum > guideLNum) ? ", U@" : ", D@") <<bc / dbu;
        }
      }
      cout <<", wlen_h@" <<iroute->getWlenHelper() <<endl;
    }
  }
}



void FlexTAWorker::initCosts() {
  //bool enableOutput = true;
  bool enableOutput = false;
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  frPoint bp, ep;
  frCoord bc, ec;
  // init cost
  if (isInitTA()) {
    for (auto &iroute: iroutes) {
      auto pitch = getDesign()->getTech()->getLayer(iroute->getGuide()->getBeginLayerNum())->getPitch();
      for (auto &uPinFig: iroute->getFigs()) {
        if (uPinFig->typeId() == tacPathSeg) {
          auto obj = static_cast<taPathSeg*>(uPinFig.get());
          obj->getPoints(bp, ep);
          bc = isH ? bp.x() : bp.y();
          ec = isH ? ep.x() : ep.y();
          iroute->setCost(ec - bc + iroute->hasWlenHelper2() * pitch * 1000);
        }
      }
    }
  } else {
    auto &workerRegionQuery = getWorkerRegionQuery();
    // update worker rq
    for (auto &iroute: iroutes) {
      for (auto &uPinFig: iroute->getFigs()) {
        workerRegionQuery.add(uPinFig.get());
        addCost(uPinFig.get());
      }
    }
    for (auto &iroute: extIroutes) {
      for (auto &uPinFig: iroute->getFigs()) {
        workerRegionQuery.add(uPinFig.get());
        addCost(uPinFig.get());
      }
    }
    // update iroute cost
    for (auto &iroute: iroutes) {
      frUInt4 drcCost = 0;
      frCoord trackLoc = std::numeric_limits<frCoord>::max();
      for (auto &uPinFig: iroute->getFigs()) {
        if (uPinFig->typeId() == tacPathSeg) {
          static_cast<taPathSeg*>(uPinFig.get())->getPoints(bp, ep);
          if (isH) {
            trackLoc = bp.y();
          } else {
            trackLoc = bp.x();
          }
          break;
        }
      }
      if (trackLoc == std::numeric_limits<frCoord>::max()) {
        cout <<"Error: FlexTAWorker::initCosts does not find trackLoc" <<endl;
        exit(1);
      }
      //auto tmpCost = assignIroute_getCost(iroute.get(), trackLoc, drcCost);
      assignIroute_getCost(iroute.get(), trackLoc, drcCost);
      iroute->setCost(drcCost);
      totCost += drcCost;
      //iroute->setDrcCost(drcCost);
      //totDrcCost += drcCost;
      if (enableOutput && !isInitTA()) {
        bool   isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
        double dbu = getDesign()->getTopBlock()->getDBUPerUU();
        frPoint bp, ep;
        frCoord bc, ec, trackLoc;
        cout <<iroute->getId() <<" " <<iroute->getGuide()->getNet()->getName();
        auto guideLNum = iroute->getGuide()->getBeginLayerNum();
        for (auto &uPinFig: iroute->getFigs()) {
          if (uPinFig->typeId() == tacPathSeg) {
            auto obj = static_cast<taPathSeg*>(uPinFig.get());
            obj->getPoints(bp, ep);
            bc = isH ? bp.x() : bp.y();
            ec = isH ? ep.x() : ep.y();
            trackLoc = isH ? bp.y() : bp.x();
            cout <<" (" <<bc / dbu <<"-->" <<ec / dbu <<"), len@" <<(ec - bc) / dbu <<", track@" <<trackLoc / dbu
                 <<", " <<getDesign()->getTech()->getLayer(iroute->getGuide()->getBeginLayerNum())->getName();
          } else if (uPinFig->typeId() == tacVia) {
            auto obj = static_cast<taVia*>(uPinFig.get());
            auto cutLNum = obj->getViaDef()->getCutLayerNum();
            obj->getOrigin(bp);
            bc = isH ? bp.x() : bp.y();
            cout <<string((cutLNum > guideLNum) ? ", U@" : ", D@") <<bc / dbu;
          }
        }
        //cout <<", wlen_h@" <<iroute->getWlenHelper() <<", cost@" <<iroute->getCost() <<", drcCost@" <<iroute->getDrcCost() <<endl;
        cout <<", wlen_h@" <<iroute->getWlenHelper() <<", cost@" <<iroute->getCost() <<endl;
      }
    }
  }
}

void FlexTAWorker::sortIroutes() {
  //bool enableOutput = true;
  bool enableOutput = false;
  // init cost
  if (isInitTA()) {
    for (auto &iroute: iroutes) {
      addToReassignIroutes(iroute.get());
    }
  } else {
    for (auto &iroute: iroutes) {
      if (iroute->getCost()) {
        addToReassignIroutes(iroute.get());
      }
    }
  }
  if (enableOutput && !isInitTA()) {
    bool   isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    for (auto &iroute: reassignIroutes) {
      frPoint bp, ep;
      frCoord bc, ec, trackLoc;
      cout <<iroute->getId() <<" " <<iroute->getGuide()->getNet()->getName();
      auto guideLNum = iroute->getGuide()->getBeginLayerNum();
      for (auto &uPinFig: iroute->getFigs()) {
        if (uPinFig->typeId() == tacPathSeg) {
          auto obj = static_cast<taPathSeg*>(uPinFig.get());
          obj->getPoints(bp, ep);
          bc = isH ? bp.x() : bp.y();
          ec = isH ? ep.x() : ep.y();
          trackLoc = isH ? bp.y() : bp.x();
          cout <<" (" <<bc / dbu <<"-->" <<ec / dbu <<"), len@" <<(ec - bc) / dbu <<", track@" <<trackLoc / dbu
               <<", " <<getDesign()->getTech()->getLayer(iroute->getGuide()->getBeginLayerNum())->getName();
        } else if (uPinFig->typeId() == tacVia) {
          auto obj = static_cast<taVia*>(uPinFig.get());
          auto cutLNum = obj->getViaDef()->getCutLayerNum();
          obj->getOrigin(bp);
          bc = isH ? bp.x() : bp.y();
          cout <<string((cutLNum > guideLNum) ? ", U@" : ", D@") <<bc / dbu;
        }
      }
      //cout <<", wlen_h@" <<iroute->getWlenHelper() <<", cost@" <<iroute->getCost() <<", drcCost@" <<iroute->getDrcCost() <<endl;
      cout <<", wlen_h@" <<iroute->getWlenHelper() <<", cost@" <<iroute->getCost() <<endl;
    }
  }
}

void FlexTAWorker::initFixedObjs_helper(const frBox &box, frCoord bloatDist, frLayerNum lNum, frNet* net) {
  //bool enableOutput = true;
  bool enableOutput = false;
  double dbu = getTech()->getDBUPerUU();
  frBox bloatBox;
  box.bloat(bloatDist, bloatBox);
  auto con = getDesign()->getTech()->getLayer(lNum)->getShortConstraint();
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  int idx1, idx2;
  //frCoord x1, x2;
  if (isH) {
    getTrackIdx(bloatBox.bottom(), bloatBox.top(),   lNum, idx1, idx2);
    //x1 = bloatBox.bottom();
    //x2 = bloatBox.top();
  } else {
    getTrackIdx(bloatBox.left(),   bloatBox.right(), lNum, idx1, idx2);
    //x1 = bloatBox.left();
    //x2 = bloatBox.right();
  }
  auto &trackLocs = getTrackLocs(lNum);
  //auto &tracks = getTracks(lNum);
  auto &workerRegionQuery = getWorkerRegionQuery();
  for (int i = idx1; i <= idx2; i++) {
    // new
    //auto &track = tracks[i];
    //track.addToCost(net, x1, x2, 0);
    //track.addToCost(net, x1, x2, 1);
    //track.addToCost(net, x1, x2, 2);
    // old
    auto trackLoc = trackLocs[i];
    frBox tmpBox;
    if (isH) {
      tmpBox.set(bloatBox.left(), trackLoc, bloatBox.right(), trackLoc);
    } else {
      tmpBox.set(trackLoc, bloatBox.bottom(), trackLoc, bloatBox.top());
    }
    workerRegionQuery.addCost(tmpBox, lNum, net, con);
    if (enableOutput) {
      cout <<"  add fixed obj cost ("
           <<tmpBox.left()  / dbu <<", " <<tmpBox.bottom() / dbu <<") (" 
           <<tmpBox.right() / dbu <<", " <<tmpBox.top()    / dbu <<") " 
           <<getDesign()->getTech()->getLayer(lNum)->getName();
      if (net != nullptr) {
        cout <<" " <<net->getName();
      }
      cout <<endl <<flush;
    }
  }
}

//初始化固定对象
void FlexTAWorker::initFixedObjs() {
  //bool enableOutput = false;
  ////bool enableOutput = true;
  //double dbu = getTech()->getDBUPerUU();
  vector<rq_rptr_value_t<frBlockObject> > result;
  frBox box;
  frCoord width = 0;
  frCoord bloatDist = 0;
  for (auto layerNum = getTech()->getBottomLayerNum(); layerNum <= getTech()->getTopLayerNum(); ++layerNum) {
    result.clear();
    if (getTech()->getLayer(layerNum)->getType() != frLayerTypeEnum::ROUTING ||
        getTech()->getLayer(layerNum)->getDir()  != getDir()) {
      continue;
    }
    width = getTech()->getLayer(layerNum)->getWidth();
    getRegionQuery()->query(getExtBox(), layerNum, result);
    for (auto &[boostb, obj]: result) {
      // instterm term
      if (obj->typeId() == frcTerm || obj->typeId() == frcInstTerm) {
        box.set(boostb.min_corner().x()+1, boostb.min_corner().y()+1, boostb.max_corner().x()-1, boostb.max_corner().y()-1);
        bloatDist = TASHAPEBLOATWIDTH * width;
        frNet* netPtr = nullptr;
        if (obj->typeId() == frcTerm) {
          netPtr = static_cast<frTerm*>(obj)->getNet();
        } else {
          netPtr = static_cast<frInstTerm*>(obj)->getNet();
        }
        initFixedObjs_helper(box, bloatDist, layerNum, netPtr);
      // snet
      } else if (obj->typeId() == frcPathSeg || obj->typeId() == frcVia) {
        bloatDist = initFixedObjs_calcBloatDist(obj, layerNum, boostb);
        box.set(boostb.min_corner().x()+1, boostb.min_corner().y()+1, boostb.max_corner().x()-1, boostb.max_corner().y()-1);
        frNet* netPtr = nullptr;
        if (obj->typeId() == frcPathSeg) {
          netPtr = static_cast<frPathSeg*>(obj)->getNet();
        } else {
          netPtr = static_cast<frVia*>(obj)->getNet();
        }
        initFixedObjs_helper(box, bloatDist, layerNum, netPtr);
        if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB" && getTech()->getLayer(layerNum)->getType() == frLayerTypeEnum::ROUTING) {
          // down-via
          if (layerNum - 2 >= getDesign()->getTech()->getBottomLayerNum() && 
              getTech()->getLayer(layerNum - 2)->getType() == frLayerTypeEnum::ROUTING) {
            auto cutLayer = getTech()->getLayer(layerNum - 1);
            frBox viaBox;
            auto via = make_unique<frVia>(cutLayer->getDefaultViaDef());
            via->getLayer2BBox(viaBox);
            frCoord viaWidth = viaBox.width();
            // only add for fat via
            if (viaWidth > width) {
              bloatDist = initFixedObjs_calcOBSBloatDistVia(cutLayer->getDefaultViaDef(), layerNum, boostb, false);
              initFixedObjs_helper(box, bloatDist, layerNum, netPtr);
            }
          }
          // up-via
          if (layerNum + 2 < (int)design->getTech()->getLayers().size() && 
              getTech()->getLayer(layerNum + 2)->getType() == frLayerTypeEnum::ROUTING) {
            auto cutLayer = getTech()->getLayer(layerNum + 1);
            frBox viaBox;
            auto via = make_unique<frVia>(cutLayer->getDefaultViaDef());
            via->getLayer1BBox(viaBox);
            frCoord viaWidth = viaBox.width();
            // only add for fat via
            if (viaWidth > width) {
              bloatDist = initFixedObjs_calcOBSBloatDistVia(cutLayer->getDefaultViaDef(), layerNum, boostb, false);
              initFixedObjs_helper(box, bloatDist, layerNum, netPtr);
            }
          }
        }
      } else if (obj->typeId() == frcBlockage || obj->typeId() == frcInstBlockage) {
        bloatDist = initFixedObjs_calcBloatDist(obj, layerNum, boostb);
        box.set(boostb.min_corner().x()+1, boostb.min_corner().y()+1, boostb.max_corner().x()-1, boostb.max_corner().y()-1);
        initFixedObjs_helper(box, bloatDist, layerNum, nullptr);
        // if (USEMINSPACING_OBS) {
        //   box.set(boostb.min_corner().x()+1, boostb.min_corner().y()+1, boostb.max_corner().x()-1, boostb.max_corner().y()-1);
        //   bloatDist = TASHAPEBLOATWIDTH * width;
        //   initFixedObjs_helper(box, bloatDist, layerNum, nullptr);
        // } else {
        //   box.set(boostb.min_corner().x()+1, boostb.min_corner().y()+1, boostb.max_corner().x()-1, boostb.max_corner().y()-1);
        //   bloatDist = TASHAPEBLOATWIDTH * width;
        //   initFixedObjs_helper(box, bloatDist, layerNum, nullptr);
        // }

        if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
          // block track for up-via and down-via for fat MACRO OBS          
          bool isMacro = false;
          if (obj->typeId() == frcBlockage) {
            isMacro = true;
          } else {
            auto inst = (static_cast<frInstBlockage*>(obj))->getInst();
            if (inst->getRefBlock()->getMacroClass() == MacroClassEnum::BLOCK ||
                inst->getRefBlock()->getMacroClass() == MacroClassEnum::PAD ||
                inst->getRefBlock()->getMacroClass() == MacroClassEnum::PAD_POWER ||
                inst->getRefBlock()->getMacroClass() == MacroClassEnum::RING) {
              isMacro = true;
            }
          }
          bool isFatOBS = true;
          if (min(boostb.max_corner().x() - boostb.min_corner().x(), boostb.max_corner().y() - boostb.min_corner().y()) <= 2 * width) {
            isFatOBS = false;
          } 
          if (isMacro && isFatOBS) {
            // down-via
            if (layerNum - 2 >= getDesign()->getTech()->getBottomLayerNum() && 
                getTech()->getLayer(layerNum - 2)->getType() == frLayerTypeEnum::ROUTING) {
              auto cutLayer = getTech()->getLayer(layerNum - 1);
              bloatDist = initFixedObjs_calcOBSBloatDistVia(cutLayer->getDefaultViaDef(), layerNum, boostb);
              initFixedObjs_helper(box, bloatDist, layerNum - 2, nullptr);
            }
            // up-via
            if (layerNum + 2 < (int)design->getTech()->getLayers().size() && 
                getTech()->getLayer(layerNum + 2)->getType() == frLayerTypeEnum::ROUTING) {
              auto cutLayer = getTech()->getLayer(layerNum + 1);
              bloatDist = initFixedObjs_calcOBSBloatDistVia(cutLayer->getDefaultViaDef(), layerNum, boostb);
              initFixedObjs_helper(box, bloatDist, layerNum + 2, nullptr);
            }
          }
        }
      } else {
        cout <<"Warning: unsupported type in initFixedObjs" <<endl;
      }
    }
  }
}

frCoord FlexTAWorker::initFixedObjs_calcOBSBloatDistVia(frViaDef *viaDef, const frLayerNum lNum, const box_t &boostb, bool isOBS) {
  auto layer = getTech()->getLayer(lNum);
  frBox viaBox;
  auto via = make_unique<frVia>(viaDef);
  if (viaDef->getLayer1Num() == lNum) {
    via->getLayer1BBox(viaBox);
  } else {
    via->getLayer2BBox(viaBox);
  }
  frCoord viaWidth = viaBox.width();
  frCoord viaLength = viaBox.length();

  frCoord obsWidth = min(boostb.max_corner().x() - boostb.min_corner().x(), boostb.max_corner().y() - boostb.min_corner().y());
  if (USEMINSPACING_OBS && isOBS) {
    obsWidth = layer->getWidth();
  }

  frCoord bloatDist = 0;
  auto con = getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      bloatDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      bloatDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(obsWidth, viaWidth/*prl*/);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      bloatDist = static_cast<frSpacingTableTwConstraint*>(con)->find(obsWidth, viaWidth, viaWidth/*prl*/);
    }
  }
  // at least via enclosure should not short with obs (OBS has issue with wrongway and PG has issue with prefDir)
  // TODO: generalize the following
  if (isOBS) {
    bloatDist += viaLength / 2;
  } else {
    bloatDist += viaWidth / 2;
  }
  return bloatDist;
}

frCoord FlexTAWorker::initFixedObjs_calcBloatDist(frBlockObject *obj, const frLayerNum lNum, const box_t &boostb) {
  auto layer = getTech()->getLayer(lNum);
  frCoord width = layer->getWidth();
  // use width if minSpc does not exist
  frCoord bloatDist = width;
  frCoord objWidth = min(boostb.max_corner().x() - boostb.min_corner().x(), boostb.max_corner().y() - boostb.min_corner().y());
  frCoord prl = (layer->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir) ? (boostb.max_corner().x() - boostb.min_corner().x()) :
                                                                                   (boostb.max_corner().y() - boostb.min_corner().y());
  if (obj->typeId() == frcBlockage || obj->typeId() == frcInstBlockage) {
    if (USEMINSPACING_OBS) {
      objWidth = width;
    }
  }
  auto con = getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      bloatDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      bloatDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(objWidth, prl);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      bloatDist = static_cast<frSpacingTableTwConstraint*>(con)->find(objWidth, width, prl);
    }
  }
  // assuming the wire width is width
  bloatDist += width / 2;
  return bloatDist;
}

//轨道分配之前的初始化，准备数据和结构
void FlexTAWorker::init() {
  rq.init();  //区域查询的初始化
  initTracks(); //初始化轨道
  if (getTAIter() != -1) {
    initFixedObjs();
  }
  initIroutes();//初始化iroute
  if (getTAIter() != -1) {
    initCosts();
    sortIroutes();//可能还会对iroute进行排序
  }
}
