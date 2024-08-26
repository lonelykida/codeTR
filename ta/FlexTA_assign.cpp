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

#include <cmath>
#include "ta/FlexTA.h"

using namespace std;
using namespace fr;

//box1到box2距离的平方
inline frCoord FlexTAWorker::box2boxDistSquare(const frBox &box1, const frBox &box2, frCoord &dx, frCoord &dy) {
  dx = max(max(box1.left(), box2.left())     - min(box1.right(), box2.right()), 0);
  dy = max(max(box1.bottom(), box2.bottom()) - min(box1.top(), box2.top()),     0);
  return dx * dx + dy * dy;
}

// must be current TA layer
//修改最小间距约束Planar - 传入的box是当前布线形状所在层的边界，或pin形状的上下层的边界
void FlexTAWorker::modMinSpacingCostPlanar(const frBox &box, frLayerNum lNum, taPinFig* fig, bool isAddCost, set<taPin*, frBlockObjectComp> *pinS) {
  //bool enableOutput = true;
  bool enableOutput = false;
  double dbu = getDesign()->getTopBlock()->getDBUPerUU();//用户数据单位 2000
  // obj1 = curr obj
  //获取当前所在层的宽度和长度
  frCoord width1  = box.width();
  frCoord length1 = box.length();
  // obj2 = other obj
  // layer default width
  //获取其它层的宽度和长度 - 其实应该是层的默认宽度，以及半宽度hw2
  frCoord width2     = getDesign()->getTech()->getLayer(lNum)->getWidth();
  frCoord halfwidth2 = width2 / 2;
  // spacing value needed - 需要的间距值
  //膨胀距离初始化为0
  frCoord bloatDist = 0;
  //当前层的最小间距约束类con指针
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {  //若有最小间距，则看con的类型，根据不同类型，设置不同的膨胀距离
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {  //间距约束
      //膨胀宽度设置为最小间距值
      bloatDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {//间距表
      //在间距表中查找最小PRL间距的值，将其设置为膨胀距离
      bloatDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), length1);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      //在间距表中查找最小TW间距的值，将其设置为膨胀距离
      bloatDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, length1);
    } else {
      cout <<"Warning: min spacing rule not supporterd" <<endl;
      return;
    }
  } else {  //否则出错，表示没有找到最小间距约束
    cout <<"Warning: no min spacing rule" <<endl;
    return;
  }
  //膨胀距离平方
  frCoord bloatDistSquare = bloatDist * bloatDist;
  //是否水平方向
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);

  // now assume track in H direction 此时假设轨道在水平方向
  //计算边界的上下左右坐标
  frCoord boxLow  = isH ? box.bottom() : box.left();
  frCoord boxHigh = isH ? box.top()    : box.right();
  frCoord boxLeft = isH ? box.left()   : box.bottom();
  frCoord boxRight= isH ? box.right()  : box.top();
  //边界box1的上下左右
  frBox box1(boxLeft, boxLow, boxRight, boxHigh);

  int idx1, idx2;
  //获取可用的轨道范围idx1和idx2
  getTrackIdx(boxLow - bloatDist - halfwidth2 + 1, boxHigh + bloatDist + halfwidth2 - 1, lNum, idx1, idx2);
  //由半宽度组成的矩形边界
  frBox box2(-halfwidth2, -halfwidth2, halfwidth2, halfwidth2);
  //变换矩形
  frTransform xform;
  frCoord dx, dy;
  //获取当前层的所有轨道的坐标集合
  auto &trackLocs = getTrackLocs(lNum);
  //获取区域查询工作器
  auto &workerRegionQuery = getWorkerRegionQuery();
  for (int i = idx1; i <= idx2; i++) {  //对idx1到idx2的每个可用轨道
    //cout <<"@@@ " <<i <<endl <<flush;
    auto trackLoc = trackLocs[i]; //获取当前到达的轨道的坐标
    xform.set(frPoint(boxLeft, trackLoc));  //将左边界作为X，轨道坐标作为Y，用来设置偏移的基准
    box2.transform(xform);  //根据xform的方向和box2的偏移量对box2的坐标进行变换
    box2boxDistSquare(box1, box2, dx, dy);  //计算box1和box2的距离平方，但是这儿没计算，只是把box1和box2的xy的差值放到dx和dy里了
    frCoord maxX = (frCoord)(sqrt(1.0 * bloatDistSquare - 1.0 * dy * dy));//计算最大膨胀距离平方与两盒子距离平方差值的平方根
    if (maxX * maxX + dy * dy == bloatDistSquare) { //这儿也不知道计算来干嘛的。。。。。
      maxX = max(0, maxX - 1);
    }
    frCoord blockLeft  = boxLeft  - maxX - halfwidth2;  //计算块的左边界，是当前边界-maxX再-半宽度得到的
    frCoord blockRight = boxRight + maxX + halfwidth2;  //计算块的右边界，是当前边界+maxX再+半宽度得到的
    frBox tmpBox; //临时的边界
    if (isH) {  //根据水平或竖直方向设置tmpBox
      tmpBox.set(blockLeft, trackLoc, blockRight, trackLoc);
    } else {
      tmpBox.set(trackLoc, blockLeft, trackLoc, blockRight);
    }
    if (isAddCost) {  //若是增加代价，则把代价添加到计算的tmpBox范围上去，放到costs数组中
      workerRegionQuery.addCost(tmpBox, lNum, fig, con);
      if (pinS) {     //若有pinS，则将这些东西加到pinS中
        workerRegionQuery.query(tmpBox, lNum, *pinS);
      }
      if (enableOutput) {
        cout <<"  add minSpc planar@("
             <<tmpBox.left()  / dbu <<", " <<tmpBox.bottom() / dbu <<") (" 
             <<tmpBox.right() / dbu <<", " <<tmpBox.top()    / dbu <<") " 
             <<getDesign()->getTech()->getLayer(lNum)->getName() <<endl <<flush;
      }
    } else {          //否则若是减去代价
      workerRegionQuery.removeCost(tmpBox, lNum, fig, con);
      if (pinS) {
        workerRegionQuery.query(tmpBox, lNum, *pinS);
      }
      if (enableOutput) {
        cout <<"  sub minSpc planar@(" 
             <<tmpBox.left()  / dbu <<", " <<tmpBox.bottom() / dbu <<") (" 
             <<tmpBox.right() / dbu <<", " <<tmpBox.top()    / dbu <<") " 
             <<getDesign()->getTech()->getLayer(lNum)->getName() <<endl <<flush;
      }
    }
  }
}

// given a shape on any routing layer n, block via @(n+1) if isUpperVia is true
void FlexTAWorker::modMinSpacingCostVia(const frBox &box, frLayerNum lNum, taPinFig* fig, bool isAddCost, bool isUpperVia, bool isCurrPs, 
                                        set<taPin*, frBlockObjectComp> *pinS) {
  //bool enableOutput = true;
  bool enableOutput = false;
  double dbu = getDesign()->getTopBlock()->getDBUPerUU();
  // obj1 = curr obj
  frCoord width1  = box.width();
  frCoord length1 = box.length();
  // obj2 = other obj
  // default via dimension
  frViaDef* viaDef = nullptr;
  frLayerNum cutLNum = 0;
  if (isUpperVia) {
    viaDef = (lNum < getDesign()->getTech()->getTopLayerNum()) ? 
             getDesign()->getTech()->getLayer(lNum+1)->getDefaultViaDef() : 
             nullptr;
    cutLNum = lNum + 1;
  } else {
    viaDef = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? 
             getDesign()->getTech()->getLayer(lNum-1)->getDefaultViaDef() : 
             nullptr;
    cutLNum = lNum - 1;
  }
  if (viaDef == nullptr) {
    return;
  }
  frVia via(viaDef);
  frBox viaBox(0,0,0,0);
  if (isUpperVia) {
    via.getLayer1BBox(viaBox);
  } else {
    via.getLayer2BBox(viaBox);
  }
  frCoord width2  = viaBox.width();
  frCoord length2 = viaBox.length();

  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  frLayerNum followTrackLNum = (int)getDesign()->getTech()->getBottomLayerNum() - 1;
  if (cutLNum - 1 >= getDesign()->getTech()->getBottomLayerNum() && 
      getDesign()->getTech()->getLayer(cutLNum - 1)->getType() == frLayerTypeEnum::ROUTING &&
      getDesign()->getTech()->getLayer(cutLNum - 1)->getDir() == getDir()) {
    followTrackLNum = cutLNum - 1;
  } else if (cutLNum + 1 <= getDesign()->getTech()->getTopLayerNum() && 
             getDesign()->getTech()->getLayer(cutLNum + 1)->getType() == frLayerTypeEnum::ROUTING &&
             getDesign()->getTech()->getLayer(cutLNum + 1)->getDir() == getDir()) {
    followTrackLNum = cutLNum + 1;
  } else {
    cout <<"Warning: via layer connected to non-routing layer, skipped in modMinSpacingCostVia" <<endl;
    return;
  }

  // spacing value needed
  frCoord bloatDist = 0;
  auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
  if (con) {
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      bloatDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      bloatDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), isCurrPs ? length2 : min(length1, length2));
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      bloatDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, isCurrPs ? length2 : min(length1, length2));
    } else {
      cout <<"Warning: min spacing rule not supporterd" <<endl;
      return;
    }
  } else {
    cout <<"Warning: no min spacing rule" <<endl;
    return;
  }
  // other obj eol spc to curr obj
  // no need to bloat eolWithin because eolWithin always < minSpacing
  //frCoord bloatDistEolX = 0;
  //frCoord bloatDistEolY = 0;
  //for (auto con: getDesign()->getTech()->getLayer(lNum)->getEolSpacing()) {
  //  auto eolSpace  = con->getMinSpacing();
  //  auto eolWidth  = con->getEolWidth();
  //  // eol up and down
  //  if (viaBox.right() - viaBox.left() < eolWidth) {
  //    bloatDistEolY = max(bloatDistEolY, eolSpace);
  //  } 
  //  // eol left and right
  //  if (viaBox.top() - viaBox.bottom() < eolWidth) {
  //    bloatDistEolX = max(bloatDistEolX, eolSpace);
  //  }
  //}
  //frCoord bloatDistSquare = bloatDist * bloatDist;

  int idx1, idx2;
  if (isH) {
    getTrackIdx(box.bottom() - bloatDist - (viaBox.top() - 0) + 1, 
                box.top()    + bloatDist + (0 - viaBox.bottom()) - 1,
                followTrackLNum, idx1, idx2);
  } else {
    getTrackIdx(box.left()   - bloatDist - (viaBox.right() - 0) + 1, 
                box.right()  + bloatDist + (0 - viaBox.left()) - 1,
                followTrackLNum, idx1, idx2);
  }

  auto &trackLocs = getTrackLocs(followTrackLNum);
  auto &workerRegionQuery = getWorkerRegionQuery();
  frBox tmpBx;
  frTransform xform;
  frCoord dx, dy, prl;
  //frCoord distSquare;
  frCoord reqDist = 0;
  frCoord maxX, blockLeft, blockRight;
  frBox blockBox;
  for (int i = idx1; i <= idx2; i++) {
    //cout <<"@@@ " <<i <<endl <<flush;
    auto trackLoc = trackLocs[i];
    if (isH) {
      xform.set(frPoint(box.left(), trackLoc));
    } else {
      xform.set(frPoint(trackLoc, box.bottom()));
    }
    tmpBx.set(viaBox);
    tmpBx.transform(xform);
    //distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
    box2boxDistSquare(box, tmpBx, dx, dy);
    if (isH) { // track is horizontal
      if (dy > 0) { // via at the bottom of box
        if (isCurrPs) { // prl maxed out to be viaBox
          prl = viaBox.right() - viaBox.left();
        } else { // prl maxed out to be smaller of box and viaBox
          prl = min(box.right() - box.left(), viaBox.right() - viaBox.left());
        }
      // via at the side of box
      } else {
        if (isCurrPs) { // prl maxed out to be viaBox
          prl = viaBox.top() - viaBox.bottom();
        } else { // prl maxed out to be smaller of box and viaBox
          prl = min(box.top() - box.bottom(), viaBox.top() - viaBox.bottom());
        }
      }
    } else { // track is vertical
      if (dx > 0) { // via at the bottom of box
        if (isCurrPs) { // prl maxed out to be viaBox
          prl = viaBox.top() - viaBox.bottom();
        } else { // prl maxed out to be smaller of box and viaBox
          prl = min(box.top() - box.bottom(), viaBox.top() - viaBox.bottom());
        }
      // via at the side of box
      } else {
        if (isCurrPs) { // prl maxed out to be viaBox
          prl = viaBox.right() - viaBox.left();
        } else { // prl maxed out to be smaller of box and viaBox
          prl = min(box.right() - box.left(), viaBox.right() - viaBox.left());
        }
      }
    }
    
    if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint) {
      reqDist = static_cast<frSpacingConstraint*>(con)->getMinSpacing();
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint) {
      reqDist = static_cast<frSpacingTablePrlConstraint*>(con)->find(max(width1, width2), prl);
    } else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint) {
      reqDist = static_cast<frSpacingTableTwConstraint*>(con)->find(width1, width2, prl);
    }

    if (isH) {
      if (dy >= reqDist) {
        continue;
      }
      maxX = (frCoord)(sqrt(1.0 * reqDist * reqDist - 1.0 * dy * dy));
      if (maxX * maxX + dy * dy == reqDist * reqDist) {
        maxX = max(0, maxX - 1);
      }
      blockLeft  = box.left()  - maxX - (viaBox.right() - 0);
      blockRight = box.right() + maxX + (0- viaBox.left());

      blockBox.set(blockLeft, trackLoc, blockRight, trackLoc);
    } else {
      if (dx >= reqDist) {
        continue;
      }
      maxX = (frCoord)(sqrt(1.0 * reqDist * reqDist - 1.0 * dx * dx));
      if (maxX * maxX + dx * dx == reqDist * reqDist) {
        maxX = max(0, maxX - 1);
      }
      blockLeft  = box.bottom() - maxX - (viaBox.top() - 0);
      blockRight = box.top()    + maxX + (0- viaBox.bottom());

      blockBox.set(trackLoc, blockLeft, trackLoc, blockRight);
    }

    if (isAddCost) {
      workerRegionQuery.addCost(blockBox, cutLNum, fig, con);
      if (pinS) {
        workerRegionQuery.query(blockBox, cutLNum, *pinS);
      }
      if (enableOutput) {
        cout <<"  add minSpc via@("
             <<blockBox.left()  / dbu <<", " <<blockBox.bottom() / dbu <<") (" 
             <<blockBox.right() / dbu <<", " <<blockBox.top()    / dbu <<") " 
             <<getDesign()->getTech()->getLayer(cutLNum)->getName() <<endl <<flush;
      }
    } else {
      workerRegionQuery.removeCost(blockBox, cutLNum, fig, con);
      if (pinS) {
        workerRegionQuery.query(blockBox, cutLNum, *pinS);
      }
      if (enableOutput) {
        cout <<"  sub minSpc via@(" 
             <<blockBox.left()  / dbu <<", " <<blockBox.bottom() / dbu <<") (" 
             <<blockBox.right() / dbu <<", " <<blockBox.top()    / dbu <<") " 
             <<getDesign()->getTech()->getLayer(cutLNum)->getName() <<endl <<flush;
      }
    }
  }

}

void FlexTAWorker::modCutSpacingCost(const frBox &box, frLayerNum lNum, taPinFig* fig, bool isAddCost, set<taPin*, frBlockObjectComp> *pinS) {
  bool enableOutput = false;
  //bool enableOutput = true;
  double dbu = getDesign()->getTopBlock()->getDBUPerUU();
  if (!getDesign()->getTech()->getLayer(lNum)->hasCutSpacing()) {
    return;
  }
  // obj1 = curr obj
  // obj2 = other obj
  // default via dimension
  frViaDef* viaDef = getDesign()->getTech()->getLayer(lNum)->getDefaultViaDef();
  frVia via(viaDef);
  frBox viaBox(0,0,0,0);
  via.getCutBBox(viaBox);

  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  frLayerNum followTrackLNum = (int)getDesign()->getTech()->getBottomLayerNum() - 1;
  if (lNum - 1 >= getDesign()->getTech()->getBottomLayerNum() && 
      getDesign()->getTech()->getLayer(lNum - 1)->getType() == frLayerTypeEnum::ROUTING &&
      getDesign()->getTech()->getLayer(lNum - 1)->getDir() == getDir()) {
    followTrackLNum = lNum - 1;
  } else if (lNum + 1 <= getDesign()->getTech()->getTopLayerNum() && 
             getDesign()->getTech()->getLayer(lNum + 1)->getType() == frLayerTypeEnum::ROUTING &&
             getDesign()->getTech()->getLayer(lNum + 1)->getDir() == getDir()) {
    followTrackLNum = lNum + 1;
  } else {
    cout <<"Warning: via layer connected to non-routing layer, skipped in modMinSpacingCostVia" <<endl;
    return;
  }

  // spacing value needed
  frCoord bloatDist = 0;
  for (auto con: getDesign()->getTech()->getLayer(lNum)->getCutSpacing()) {
    bloatDist = max(bloatDist, con->getCutSpacing());
  }

  int idx1, idx2;
  if (isH) {
    getTrackIdx(box.bottom() - bloatDist - (viaBox.top() - 0) + 1, 
                box.top()    + bloatDist + (0 - viaBox.bottom()) - 1,
                followTrackLNum, idx1, idx2);
  } else {
    getTrackIdx(box.left()   - bloatDist - (viaBox.right() - 0) + 1, 
                box.right()  + bloatDist + (0 - viaBox.left()) - 1,
                followTrackLNum, idx1, idx2);
  }

  auto &trackLocs = getTrackLocs(followTrackLNum);
  auto &workerRegionQuery = getWorkerRegionQuery();
  frBox tmpBx;
  frTransform xform;
  frCoord dx, dy, c2ctrackdist;
  //frCoord distSquare;
  frCoord reqDist = 0;
  frCoord maxX, blockLeft, blockRight;
  frBox blockBox;
  frPoint boxCenter, tmpBxCenter;
  boxCenter.set((box.left() + box.right()) / 2, (box.bottom() + box.top()) / 2);
  bool hasViol = false;
  for (int i = idx1; i <= idx2; i++) {
    auto trackLoc = trackLocs[i];
    //cout <<"@@@" <<trackLoc <<endl;
    if (isH) {
      xform.set(frPoint(box.left(), trackLoc));
    } else {
      xform.set(frPoint(trackLoc, box.bottom()));
    }
    tmpBx.set(viaBox);
    tmpBx.transform(xform);
    //distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
    box2boxDistSquare(box, tmpBx, dx, dy);

    for (auto con: getDesign()->getTech()->getLayer(lNum)->getCutSpacing()) {
      hasViol       = false;
      reqDist       = con->getCutSpacing();
      bool isC2C = con->hasCenterToCenter();
      if (isH) {
        c2ctrackdist = abs(boxCenter.y() - trackLoc);
      } else {
        c2ctrackdist = abs(boxCenter.x() - trackLoc);
      }

      if (isH) {
        if (isC2C) {
          if (c2ctrackdist >= reqDist) {
            continue;
          }
        } else {
          if (dy >= reqDist) {
            continue;
          }
        }
        //cout <<"@@@@" <<trackLoc <<endl;
        if (isC2C) {
          maxX = (frCoord)(sqrt(1.0 * reqDist * reqDist - 1.0 * c2ctrackdist * c2ctrackdist));
        } else {
          maxX = (frCoord)(sqrt(1.0 * reqDist * reqDist - 1.0 * dy * dy));
        }
        if (maxX * maxX + dy * dy == reqDist * reqDist) {
          maxX = max(0, maxX - 1);
        }
        if (isC2C) {
          blockLeft  = boxCenter.x() - maxX;
          blockRight = boxCenter.x() + maxX;
        } else {
          blockLeft  = box.left()  - maxX - (viaBox.right() - 0);
          blockRight = box.right() + maxX + (0- viaBox.left());
        }
        blockBox.set(blockLeft, trackLoc, blockRight, trackLoc);
      } else {
        if (isC2C) {
          if (c2ctrackdist >= reqDist) {
            continue;
          }
        } else {
          if (dx >= reqDist) {
            continue;
          }
        }
        //cout <<"@@@@" <<trackLoc <<endl;
        if (isC2C) {
          maxX = (frCoord)(sqrt(1.0 * reqDist * reqDist - 1.0 * c2ctrackdist * c2ctrackdist));
        } else {
          maxX = (frCoord)(sqrt(1.0 * reqDist * reqDist - 1.0 * dx * dx));
        }
        if (maxX * maxX + dx * dx == reqDist * reqDist) {
          maxX = max(0, maxX - 1);
        }
        if (isC2C) {
          blockLeft  = boxCenter.y() - maxX;
          blockRight = boxCenter.y() + maxX;
        } else {
          blockLeft  = box.bottom()  - maxX - (viaBox.top() - 0);
          blockRight = box.top()     + maxX + (0- viaBox.bottom());
        }

        blockBox.set(trackLoc, blockLeft, trackLoc, blockRight);
      }
      if (con->hasSameNet()) {
        continue;
      }
      if (con->isLayer()) {
        ;
      } else if (con->isAdjacentCuts()) {
        hasViol = true;
        // should disable hasViol and modify this part to new grid graph
      } else if (con->isParallelOverlap()) {
        if (isH) {
          if (dy > 0) {
            blockBox.set(max(box.left()  - (viaBox.right() - 0) + 1, blockLeft), trackLoc, 
                         min(box.right() + (0 - viaBox.left())  - 1, blockRight), trackLoc);
          }
        } else {
          if (dx > 0) {
            blockBox.set(trackLoc, max(box.bottom() - (viaBox.top() - 0)     + 1, blockLeft), 
                         trackLoc, min(box.top()    + (0 - viaBox.bottom())  - 1, blockRight));
          }
        }
        if (blockBox.left() <= blockBox.right() && blockBox.bottom() <= blockBox.top()) {
          hasViol = true;
        }
      } else if (con->isArea()) {
        auto currArea = max(box.length() * box.width(), tmpBx.length() * tmpBx.width());
        if (currArea >= con->getCutArea()) {
          hasViol = true;
        }
      } else {
        hasViol = true;
      }
      if (hasViol) {
        if (isAddCost) {
          workerRegionQuery.addCost(blockBox, lNum, fig, con);
          if (pinS) {
            workerRegionQuery.query(blockBox, lNum, *pinS);
          }
          if (enableOutput) {
            cout <<"  add cutSpc via@("
                 <<blockBox.left()  / dbu <<", " <<blockBox.bottom() / dbu <<") (" 
                 <<blockBox.right() / dbu <<", " <<blockBox.top()    / dbu <<") " 
                 <<getDesign()->getTech()->getLayer(lNum)->getName() <<endl <<flush;
          }
        } else {
          workerRegionQuery.removeCost(blockBox, lNum, fig, con);
          if (pinS) {
            workerRegionQuery.query(blockBox, lNum, *pinS);
          }
          if (enableOutput) {
            cout <<"  sub cutSpc via@(" 
                 <<blockBox.left()  / dbu <<", " <<blockBox.bottom() / dbu <<") (" 
                 <<blockBox.right() / dbu <<", " <<blockBox.top()    / dbu <<") " 
                 <<getDesign()->getTech()->getLayer(lNum)->getName() <<endl <<flush;
          }
        }
      }
    }
  }
}

//增加成本
void FlexTAWorker::addCost(taPinFig* fig, set<taPin*, frBlockObjectComp> *pinS) {
  modCost(fig, true, pinS);
}

//减少成本
void FlexTAWorker::subCost(taPinFig* fig, set<taPin*, frBlockObjectComp> *pinS) {
  modCost(fig, false, pinS);
}

//根据中间参数决定增加成本(true)或减少成本(false)
void FlexTAWorker::modCost(taPinFig* fig, bool isAddCost, set<taPin*, frBlockObjectComp> *pinS) {
  if (fig->typeId() == tacPathSeg) {  //若当前形状是路径段
    auto obj = static_cast<taPathSeg*>(fig);  //获取当前形状
    auto layerNum = obj->getLayerNum(); //获取当前形状所在层号
    frBox box;  //获取当前形状的边界
    obj->getBBox(box);  
    //修改最小间距成本等，具体的到底是修改什么还没看
    modMinSpacingCostPlanar(box, layerNum, obj, isAddCost, pinS); // must be current TA layer
    modMinSpacingCostVia(box, layerNum, obj, isAddCost, true, true, pinS);
    modMinSpacingCostVia(box, layerNum, obj, isAddCost, false, true, pinS);
  } else if (fig->typeId() == tacVia) { //若当前形状是通孔
    auto obj = static_cast<taVia*>(fig);//获取当前通孔形状
    frBox box;  //获取当前形状相关的边界
    //获取当前通孔形状上下层的边界盒及层号，假设通孔的扩展(enclosure)总是矩阵
    obj->getLayer1BBox(box); // assumes enclosure for via is always rectangle
    auto layerNum = obj->getViaDef()->getLayer1Num();
    // current TA layer  当前的TA层 - 修改一大堆东西
    if (getDir() == getDesign()->getTech()->getLayer(layerNum)->getDir()) {
      modMinSpacingCostPlanar(box, layerNum, obj, isAddCost, pinS);
    }
    modMinSpacingCostVia(box, layerNum, obj, isAddCost, true, false, pinS);
    modMinSpacingCostVia(box, layerNum, obj, isAddCost, false, false, pinS);

    //获取当前通孔形状上下层的边界盒及层号，假设通孔的扩展(enclosure)总是矩阵
    obj->getLayer2BBox(box); // assumes enclosure for via is always rectangle
    layerNum = obj->getViaDef()->getLayer2Num();
    // current TA layer
    if (getDir() == getDesign()->getTech()->getLayer(layerNum)->getDir()) {
      modMinSpacingCostPlanar(box, layerNum, obj, isAddCost, pinS);
    }
    modMinSpacingCostVia(box, layerNum, obj, isAddCost, true, false, pinS);
    modMinSpacingCostVia(box, layerNum, obj, isAddCost, false, false, pinS);

    frTransform xform;  //偏移坐标
    frPoint pt; //坐标点
    obj->getOrigin(pt); //获取当前通孔形状的原点
    xform.set(pt);    //对原点偏移
    //下边就是对形状的代价进行修改
    for (auto &uFig: obj->getViaDef()->getCutFigs()) {
      auto rect = static_cast<frRect*>(uFig.get());
      rect->getBBox(box);
      box.transform(xform);
      layerNum = obj->getViaDef()->getCutLayerNum();
      modCutSpacingCost(box, layerNum, obj, isAddCost, pinS);
    }
  } else {
    cout <<"Error: unsupported region query add" <<endl;
  }
}

//为iroute分配可用的track
void FlexTAWorker::assignIroute_availTracks(taPin* iroute, frLayerNum &lNum, int &idx1, int &idx2) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //获取iroute所在的层号
  lNum = iroute->getGuide()->getBeginLayerNum();
  frPoint gbp, gep, gIdx; //获取iroute的起点(begin)和终点(end)
  frBox gBox; //扩展盒
  iroute->getGuide()->getPoints(gbp, gep);
  //获取起点的gcell坐标(索引)
  getDesign()->getTopBlock()->getGCellIdx(gbp, gIdx);
  //根据起点的gcell坐标(索引)获取gcell的边界盒
  getDesign()->getTopBlock()->getGCellBox(gIdx, gBox);
  //判断当前iroute的方向是否为横向
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  //获取iroute的起始轨道位置和结束轨道位置
  frCoord coordLow  = isH ? gBox.bottom() : gBox.left();
  frCoord coordHigh = isH ? gBox.top()    : gBox.right();
  //防止轨道位置超出边界
  coordHigh--; // to avoid higher track == guide top/right
  //获取当前层的轨道索引范围
  getTrackIdx(coordLow, coordHigh, lNum, idx1, idx2);
  if (enableOutput) {
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    cout <<" min/max track@" <<getTrackLocs(lNum)[idx1] / dbu;
    if (idx2 > idx1) {
      cout <<"/" <<getTrackLocs(lNum)[idx2] / dbu;
    }
    cout <<endl;
  }
  /*
  min/max track@136.895/139.555
  136.895*2000 = 27300
  139.555*2000 = 27800
  min/max track@65.645/68.305
  min/max track@65.645/68.305
  min/max track@68.495/71.155
  min/max track@96.995/99.655
  min/max track@62.795/65.455
  min/max track@71.345/74.005
  min/max track@71.345/74.005
  min/max track@88.445/91.105
  min/max track@105.545/108.205
  min/max track@74.195/76.855
  min/max track@91.295/93.955
  min/max track@102.695/105.355
  min/max track@99.845/102.505
  min/max track@85.595/88.255
  min/max track@79.895/82.555
  min/max track@96.995/99.655
  */
}

frUInt4 FlexTAWorker::assignIroute_getWlenCost(taPin* iroute, frCoord trackLoc) {
  auto guide = iroute->getGuide();
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  frPoint begin, end;
  guide->getPoints(begin, end);
  frBox endBox;
  frPoint idx;
  getDesign()->getTopBlock()->getGCellIdx(end, idx);
  getDesign()->getTopBlock()->getGCellBox(idx, endBox);
  int wlen = 0;
  auto wlen_helper = iroute->getWlenHelper();
  if (wlen_helper <= 0) {
    if (isH) {
      wlen = abs(wlen_helper) * (trackLoc - endBox.bottom());
    } else {
      wlen = abs(wlen_helper) * (trackLoc - endBox.left());
    }
  } else {
    if (isH) {
      wlen = abs(wlen_helper) * (endBox.top()   - trackLoc);
    } else {
      wlen = abs(wlen_helper) * (endBox.right() - trackLoc);
    }
  }
  if (wlen < 0) {
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    cout <<"Error: getWlenCost has wlenCost < 0" <<", trackLoc@" <<trackLoc / dbu <<" box (" 
         <<endBox.left()  / dbu  <<", " <<endBox.bottom() / dbu <<") (" 
         <<endBox.right() / dbu  <<", " <<endBox.top()    / dbu <<")" <<endl;
    return (frUInt4)0;
  } else {
    return (frUInt4)wlen;
  }
}

frUInt4 FlexTAWorker::assignIroute_getPinCost(taPin* iroute, frCoord trackLoc) {
  frUInt4 sol = 0;
  if (iroute->hasWlenHelper2()) {
    sol = abs(trackLoc - iroute->getWlenHelper2());
    if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
      bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
      auto layerNum = iroute->getGuide()->getBeginLayerNum();
      int zIdx = layerNum / 2 - 1;
      if (sol) {
        if (isH) {
          // if cannot use bottom or upper layer to bridge, then add cost
          if ((getTech()->isVia2ViaForbiddenLen(zIdx, false, false, false, sol, false) || layerNum - 2 < BOTTOM_ROUTING_LAYER) &&
              (getTech()->isVia2ViaForbiddenLen(zIdx, true, true, false, sol, false) || layerNum + 2 > getTech()->getTopLayerNum())) {
            sol += TADRCCOST;
          }
        } else {
          if ((getTech()->isVia2ViaForbiddenLen(zIdx, false, false, true, sol, false) || layerNum - 2 < BOTTOM_ROUTING_LAYER) &&
              (getTech()->isVia2ViaForbiddenLen(zIdx, true, true, true, sol, false) || layerNum + 2 > getTech()->getTopLayerNum())) {
            sol += TADRCCOST;
          }
        }
      }
    }
  }
  return sol;
}

frUInt4 FlexTAWorker::assignIroute_getDRCCost_helper(taPin* iroute, const frBox &box, frLayerNum lNum) {
  auto &workerRegionQuery = getWorkerRegionQuery();
  vector<rq_generic_value_t<std::pair<frBlockObject*, frConstraint*> > > result;
  int overlap = 0;
  workerRegionQuery.queryCost(box, lNum, result);
  bool isCut = false;
  for (auto &[boostb, pr]: result) {
    auto &[obj, con] = pr;
    frCoord tmpOvlp = - max(box.left(),   boostb.min_corner().x()) + min(box.right(),  boostb.max_corner().x()) 
                      - max(box.bottom(), boostb.min_corner().y()) + min(box.top(),    boostb.max_corner().y()) + 1;
    if (tmpOvlp <= 0) {
      cout <<"Error: assignIroute_getDRCCost_helper overlap < 0" <<endl;
      exit(1);
    }
    // unknown obj, always add cost
    if (obj == nullptr) {
      overlap += tmpOvlp;
    // only add cost for diff-net
    } else if (obj->typeId() == frcNet) {
      if (iroute->getGuide()->getNet() != obj) {
        overlap += tmpOvlp;
      }
    // two taObjs
    } else if (obj->typeId() == tacPathSeg || obj->typeId() == tacVia) {
      auto taObj = static_cast<taPinFig*>(obj);
      // can exclude same iroute objs also
      if (taObj->getPin() == iroute) {
        continue;
      }
      if (iroute->getGuide()->getNet() != taObj->getPin()->getGuide()->getNet()) {
        overlap += tmpOvlp;
      }
    } else {
      cout <<"Warning: assignIroute_getDRCCost_helper unsupported type" <<endl;
    }
  }
  frCoord pitch = 0;
  if (getDesign()->getTech()->getLayer(lNum)->getType() == frLayerTypeEnum::ROUTING) {
    pitch = getDesign()->getTech()->getLayer(lNum)->getPitch();
    isCut = false;
  } else if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1 && getDesign()->getTech()->getLayer(lNum + 1)->getType() == frLayerTypeEnum::ROUTING) {
    pitch = getDesign()->getTech()->getLayer(lNum + 1)->getPitch();
    isCut = true;
  } else if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1 && getDesign()->getTech()->getLayer(lNum - 1)->getType() == frLayerTypeEnum::ROUTING) {
    pitch = getDesign()->getTech()->getLayer(lNum - 1)->getPitch();
    isCut = true;
  } else {
    cout <<"Error: assignIroute_getDRCCost_helper unknown layer type" <<endl;
    exit(1);
  }
  // always penalize two pitch per cut, regardless of cnts
  return (overlap == 0) ? 0 : (isCut ? pitch * 2: max(pitch * 2, overlap));
}

//计算iroute分配到t上的DRC代价
frUInt4 FlexTAWorker::assignIroute_getDRCCost(taPin* iroute, frCoord trackLoc) {
  frUInt4 cost = 0; //成本
  frPoint bp, ep;   //起终点
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);//iroute是否水平方向
  for (auto &uPinFig: iroute->getFigs()) {
    if (uPinFig->typeId() == tacPathSeg) {  //若当前是布线段
      auto obj = static_cast<taPathSeg*>(uPinFig.get());
      obj->getPoints(bp, ep); //获取当前iroute形状的起终点
      if (isH) {  //若是水平方向
        bp.set(bp.x(), trackLoc);
        ep.set(ep.x(), trackLoc);
      } else {
        bp.set(trackLoc, bp.y());
        ep.set(trackLoc, ep.y());
      }
      //计算其形状的cost
      frUInt4 wireCost = assignIroute_getDRCCost_helper(iroute, frBox(bp, ep), obj->getLayerNum());
      //if (!isInitTA()) {
      //  cout <<"wireCost@" <<wireCost <<endl;
      //}
      cost += wireCost;
    } else if (uPinFig->typeId() == tacVia) { //若当前是通孔
      auto obj = static_cast<taVia*>(uPinFig.get());
      obj->getOrigin(bp);
      if (isH) {
        bp.set(bp.x(), trackLoc);
      } else {
        bp.set(trackLoc, bp.y());
      }
      //cost += TAVIACOST * assignIroute_getDRCCost_helper(iroute, frBox(bp, bp), obj->getViaDef()->getCutLayerNum());
      frUInt4 viaCost = assignIroute_getDRCCost_helper(iroute, frBox(bp, bp), obj->getViaDef()->getCutLayerNum());
      //if (!isInitTA()) {
      //  cout <<"viaCost@" <<viaCost <<endl;
      //}
      cost += viaCost;
    } else {  //若是其他类型，则出错
      cout <<"Error: assignIroute_updateIroute unsupported pinFig" <<endl;
      exit(1);
    }
  }
  return cost;
}

frUInt4 FlexTAWorker::assignIroute_getAlignCost(taPin* iroute, frCoord trackLoc) {
  frUInt4 sol = 0;
  frCoord pitch = 0;
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  for (auto &uPinFig: iroute->getFigs()) {
    if (uPinFig->typeId() == tacPathSeg) {
      auto obj = static_cast<taPathSeg*>(uPinFig.get());
      frPoint bp, ep;
      obj->getPoints(bp, ep);
      auto lNum = obj->getLayerNum();
      pitch = getDesign()->getTech()->getLayer(lNum)->getPitch();
      auto &workerRegionQuery = getWorkerRegionQuery();
      set<taPin*, frBlockObjectComp> result;
      frBox box;
      if (isH) {
        box.set(bp.x(), trackLoc, ep.x(), trackLoc);
      } else {
        box.set(trackLoc, bp.y(), trackLoc, ep.y());
      }
      workerRegionQuery.query(box, lNum, result);
      for (auto &iroute2: result) {
        if (iroute2->getGuide()->getNet() == iroute->getGuide()->getNet()) {
          sol = 1;
          break;
        }
      }
    }
    if (sol == 1) {
      break;
    }
  }
  return pitch * sol;
}

//尝试将iroute分配到轨道trackLoc时的代价
frUInt4 FlexTAWorker::assignIroute_getCost(taPin* iroute, frCoord trackLoc, frUInt4 &outDrcCost) {
  //获取iroute的pitch - 应该是金属宽度？
  frCoord irouteLayerPitch = getTech()->getLayer(iroute->getGuide()->getBeginLayerNum())->getPitch();
  //bool enableOutput = true;
  bool enableOutput = false;
  //将ir分配到轨道t后的代价
  outDrcCost     = assignIroute_getDRCCost(iroute, trackLoc);
  //TADRCCOST = 32
  //若TA还未分配，则DRC代价是0.05倍，否则就是32倍
  int drcCost    = (isInitTA()) ? (0.05 * outDrcCost) : (TADRCCOST * outDrcCost);
  //将ir分配到轨道t后的线长代价
  int wlenCost   = assignIroute_getWlenCost(iroute, trackLoc);
  // int pinCost    = TAPINCOST * assignIroute_getPinCost(iroute, trackLoc);
  //将ir分配到轨道t后的pin代价的计算
  int tmpPinCost = assignIroute_getPinCost(iroute, trackLoc);
  //TAPINCOST = TAALIGNCOST = 4
  int pinCost    = (tmpPinCost == 0) ? 0 : TAPINCOST * irouteLayerPitch + tmpPinCost;
  //将ir分配到轨道t后的align代价(对齐代价？)
  int tmpAlignCost = assignIroute_getAlignCost(iroute, trackLoc);
  int alignCost  = (tmpAlignCost == 0) ? 0 : TAALIGNCOST * irouteLayerPitch + tmpAlignCost;
  // int misalignCost = assignIroute_getMisalignCost(iroute, trackLoc);
  if (enableOutput) {
    cout <<"    drc/wlen/pin/align cost = " <<drcCost <<"/" <<wlenCost <<"/" <<pinCost <<"/" <<alignCost <<endl;
  }
  //返回drc代价+wlen代价+pin代价-对齐代价和0的最大值
  return max(drcCost + wlenCost + pinCost - alignCost, 0);
}

//辅助找到iroute的最优轨道
//lNum是当前iroute所在的层号，trackIdx是当前准备分配的轨道的索引,其他参数是会随着运行而更新的
void FlexTAWorker::assignIroute_bestTrack_helper(taPin* iroute, frLayerNum lNum, int trackIdx, frUInt4 &bestCost, 
                                                 frCoord &bestTrackLoc, int &bestTrackIdx, frUInt4 &drcCost) {
  //bool enableOutput = true;
  bool enableOutput = false;
  double dbu = getDesign()->getTopBlock()->getDBUPerUU(); //用户数据单位，2000
  //获取当前轨道的实际位置
  auto trackLoc = getTrackLocs(lNum)[trackIdx];
  //尝试将当前ir分配到轨道t后的cost
  auto currCost = assignIroute_getCost(iroute, trackLoc, drcCost);
  if (isInitTA()) { //更新最优代价和最优轨道
    if (currCost < bestCost) {
      bestCost = currCost;
      bestTrackLoc = trackLoc;
      bestTrackIdx = trackIdx;
    }
  } else {
    if (drcCost < bestCost) {
      bestCost = drcCost;
      bestTrackLoc = trackLoc;
      bestTrackIdx = trackIdx;
    }
  }
  if (enableOutput) {
    cout <<"  try track@" <<trackLoc / dbu <<", cost/drc=" <<currCost <<"/" <<drcCost <<endl;
  }
}

//在lNum上找到idx1和idx2范围内的最优的track
int FlexTAWorker::assignIroute_bestTrack(taPin* iroute, frLayerNum lNum, int idx1, int idx2) {
  //bool enableOutput = true;
  bool enableOutput = false;
  double  dbu = getDesign()->getTopBlock()->getDBUPerUU();//单位转换
  frCoord bestTrackLoc = 0; //最优轨道的实际位置
  int     bestTrackIdx = -1;//最优轨道的索引
  //将最优代价更新为无穷大
  frUInt4 bestCost = std::numeric_limits<frUInt4>::max();
  frUInt4 drcCost = 0;  //将drc代价设置为0
  //while (1) {
  // if wlen2, then try from  wlen2
  // else try from wlen1 dir
  if (iroute->hasWlenHelper2()) { //判断当前的iroute是不是pin - 因为hasWlenHelper2返回的是一个bool 类型的pin变量
    //cout <<"if" <<endl;
    //如果当前iroute的形状是pin的话找到wlen2的坐标
    frCoord wlen2coord = iroute->getWlenHelper2();
    //根据下边的代码，可以判断出WlenHelper()的作用是确定找的顺序和方向的
    if (iroute->getWlenHelper() > 0) {  //若Wlen1大于0则先从起始位置往idx2方向找，未找到再从起始位置往idx1方向找
      if (enableOutput) {
        cout <<" use wlen2@" <<wlen2coord / dbu <<", wlen@" <<iroute->getWlenHelper() <<endl;
      }
      //获取轨道起始的索引
      int startTrackIdx = int(std::lower_bound(trackLocs[lNum].begin(), trackLocs[lNum].end(), wlen2coord) - trackLocs[lNum].begin());
      startTrackIdx = min(startTrackIdx, idx2);
      startTrackIdx = max(startTrackIdx, idx1);
      //其中startTrackIdx是轨道的起始索引，应该是在idx1和idx2之间的轨道
      for (int i = startTrackIdx; i <= idx2; i++) {//从当前位置到idx2方向，将iroute试探地分到每个轨道上，找最好的轨道
        assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        if (!drcCost) { //若没有DRC，则不用再继续找了
          break;
        }
      }
      if (drcCost) {  //若有DRC，则尝试从起始位置往idx1方向找
        for (int i = startTrackIdx - 1; i >= idx1; i--) {
          assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
          if (!drcCost) {
            break;
          }
        }
      }
    } else if (iroute->getWlenHelper() == 0) {  //否则若是0的话
      //则起始轨道仍然从中间某个位置开始，遍历idx2-idx1次，依次往左、往右地震荡分配，跟哈希表的二次探测类似；
      if (enableOutput) {
        cout <<" use wlen2@" <<wlen2coord / dbu <<", wlen@" <<iroute->getWlenHelper() <<endl;
      }
      //先确定起始轨道位置
      int startTrackIdx = int(std::lower_bound(trackLocs[lNum].begin(), trackLocs[lNum].end(), wlen2coord) - trackLocs[lNum].begin());
      startTrackIdx = min(startTrackIdx, idx2);
      startTrackIdx = max(startTrackIdx, idx1);
      //cout <<"startTrackIdx " <<startTrackIdx <<endl;
      for (int i = 0; i <= idx2 - idx1; i++) {  //尝试idx2-idx1次
        int currTrackIdx = startTrackIdx + i;   //直接从起始位置开始
        //下边分别尝试i+1,i-1,i+2;i-2;i+3;i-3...等位置是否符合要求
        if (currTrackIdx >= idx1 && currTrackIdx <= idx2) { 
          assignIroute_bestTrack_helper(iroute, lNum, currTrackIdx, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        }
        if (!drcCost) {
          break;
        }
        currTrackIdx = startTrackIdx - i - 1;
        if (currTrackIdx >= idx1 && currTrackIdx <= idx2) {
          assignIroute_bestTrack_helper(iroute, lNum, currTrackIdx, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        }
        if (!drcCost) {
          break;
        }
      }
    } else {  //否则若是wlen1<0的话
      //从起始位置先往idx1方向找，否则再从起始位置开始往idx2方向找
      if (enableOutput) {
        cout <<" use wlen2@" <<wlen2coord / dbu <<", wlen@" <<iroute->getWlenHelper() <<endl;
      }
      int startTrackIdx = int(std::lower_bound(trackLocs[lNum].begin(), trackLocs[lNum].end(), wlen2coord) - trackLocs[lNum].begin());
      startTrackIdx = min(startTrackIdx, idx2);
      startTrackIdx = max(startTrackIdx, idx1);
      for (int i = startTrackIdx; i >= idx1; i--) {//从开始位置往idx1的方向开始查找
        assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        if (!drcCost) {
          break;
        }
      }
      if (drcCost) {  //若有DRC，则尝试从当前位置往idx2的方向尝试
        for (int i = startTrackIdx + 1; i <= idx2; i++) {
          assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
          if (!drcCost) {
            break;
          }
        }
      }
    }
  } else {//否则若当前iroute不是pin
    //cout <<"else" <<endl;
    if (iroute->getWlenHelper() > 0) {  //当Wlen>0的时候
      //则直接从idx2往回到idx1一直找到无DRC的轨道
      if (enableOutput) {
        cout <<" use wlen@" <<iroute->getWlenHelper() <<endl;
      }
      for (int i = idx2; i >= idx1; i--) {
        assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        if (!drcCost) {
          break;
        }
      }
    } else if (iroute->getWlenHelper() == 0) {  //否则若是0的话
      //则从中间开始先往idx2方向，再往idx1方向找无DRC轨道
      if (enableOutput) {
        cout <<" use wlen@" <<iroute->getWlenHelper() <<endl;
      }
      for (int i = (idx1 + idx2) / 2; i <= idx2; i++) {
        assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        if (!drcCost) {
          break;
        }
      }
      if (drcCost) {
        for (int i = (idx1 + idx2) / 2 - 1; i >= idx1; i--) {
          assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
          if (!drcCost) {
            break;
          }
        }
      }
    } else {  //若是wlen1<0的话
      //则从idx1方向开始一直往idx2方向找无DRC轨道
      if (enableOutput) {
        cout <<" use wlen@" <<iroute->getWlenHelper() <<endl;
      }
      for (int i = idx1; i <= idx2; i++) {
        assignIroute_bestTrack_helper(iroute, lNum, i, bestCost, bestTrackLoc, bestTrackIdx, drcCost);
        if (!drcCost) {
          break;
        }
      }
    }
  }
  
  if (bestTrackIdx == -1) {
    auto guide = iroute->getGuide();
    frBox box;
    guide->getBBox(box);
    cout <<"Error: assignIroute_bestTrack select no track for " 
         <<guide->getNet()->getName() <<" @(" 
         <<box.left()  / dbu <<", " <<box.bottom() / dbu <<") ("
         <<box.right() / dbu <<", " <<box.top()    / dbu <<" "
         <<getDesign()->getTech()->getLayer(lNum)->getName() <<" idx1/2="
         <<idx1 <<"/" <<idx2
         <<endl;
    exit(1);
  }
  if (enableOutput) {
  //if (true) {
    cout <<"  select track@" <<bestTrackLoc / dbu <<", cost=" <<bestCost <<endl;
  }
  //totCost    -= iroute->getCost();
  //totDrcCost -= iroute->getDrcCost();
  //计算总的代价
  totCost    += drcCost;
  iroute->setCost(drcCost);
  //totDrcCost += drcCost;
  return bestTrackLoc;
}
  
void FlexTAWorker::assignIroute_updateIroute(taPin* iroute, frCoord bestTrackLoc, set<taPin*, frBlockObjectComp> *pinS) {
  //找到查询区域
  auto &workerRegionQuery = getWorkerRegionQuery();
  //判断当前iroute方向是否水平
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  //iroute的起终点
  frPoint bp, ep;
  // update coord
  //更新iroute中每个形状的坐标
  for (auto &uPinFig: iroute->getFigs()) {  //获取iroute中的每个形状
    if (uPinFig->typeId() == tacPathSeg) {  //若当前形状是布线段
      auto obj = static_cast<taPathSeg*>(uPinFig.get());//获取形状对象
      obj->getPoints(bp, ep); //获取布线段的起终点
      if (isH) {  //若当前形状是横向布线段
        bp.set(bp.x(), bestTrackLoc); //设置布线段的起点坐标
        ep.set(ep.x(), bestTrackLoc); //设置布线段的终点坐标
      } else {
        bp.set(bestTrackLoc, bp.y());
        ep.set(bestTrackLoc, ep.y());
      }
      obj->setPoints(bp, ep); //将起终点设置到形状对象上
    } else if (uPinFig->typeId() == tacVia) { //若当前形状是通孔
      auto obj = static_cast<taVia*>(uPinFig.get());  //获取通孔形状
      obj->getOrigin(bp); //获取通孔的中心点
      if (isH) {  //若当前形状是横向通孔
        bp.set(bp.x(), bestTrackLoc); //设置通孔的中心点
      } else {
        bp.set(bestTrackLoc, bp.y()); 
      }
      obj->setOrigin(bp); //设置通孔的中心点
    } else {  //若当前形状不是布线段和通孔，输出错误信息
      cout <<"Error: assignIroute_updateIroute unsupported pinFig" <<endl;
      exit(1);
    }
  }
  // addCost
  //将iroute中的每个形状加入到查询区域中
  for (auto &uPinFig: iroute->getFigs()) {
    addCost(uPinFig.get(), isInitTA() ? nullptr : pinS);
    workerRegionQuery.add(uPinFig.get());
  }
  iroute->addNumAssigned(); //增加iroute的被分配次数
}

//初始化
void FlexTAWorker::assignIroute_init(taPin* iroute, set<taPin*, frBlockObjectComp> *pinS) {
  //找查询区域
  auto &workerRegionQuery = getWorkerRegionQuery();
  // subCost 减少成本
  if (!isInitTA()) {  //若当前iroute已初始化，则需减少成本
    for (auto &uPinFig: iroute->getFigs()) {  //对iroute中的每个形状
      workerRegionQuery.remove(uPinFig.get());//将其从查询区域中移除
      subCost(uPinFig.get(), pinS); //减少该形状对应的成本
    }
    totCost    -= iroute->getCost();//从总成本中扣除当前iroute的成本
    //totDrcCost -= iroute->getDrcCost();
  }
}
  //更新其它iroute，
void FlexTAWorker::assignIroute_updateOthers(set<taPin*, frBlockObjectComp> &pinS) {
  //bool enableOutput = true;
  bool enableOutput = false;
  //判断iroute的方向是否水平
  bool isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
  frPoint bp, ep;//iroute的起终点
  if (isInitTA()) {//若初始化，则直接返回
    return;
  }
  for (auto &iroute: pinS) {  //对iroute集合中的每个iroute
    removeFromReassignIroutes(iroute);//将其从再分配集合中移除
    // recalculate cost
    frUInt4 drcCost = 0;  //drc成本初始化为0
    frCoord trackLoc = std::numeric_limits<frCoord>::max();//轨道位置初始化为最大值
    for (auto &uPinFig: iroute->getFigs()) {  //对iroute中的第一个布线段形状
      if (uPinFig->typeId() == tacPathSeg) {  //若当前形状是布线段
        static_cast<taPathSeg*>(uPinFig.get())->getPoints(bp, ep);//获取该布线段的起终点
        if (isH) {//判断横纵向，从而获取轨道位置
          trackLoc = bp.y();
        } else {
          trackLoc = bp.x();
        }
        break;
      }
    }
    if (trackLoc == std::numeric_limits<frCoord>::max()) {//若没有分配轨道位置，则输出错误信息
      cout <<"Error: FlexTAWorker::assignIroute_updateOthers does not find trackLoc" <<endl;
      exit(1);
    }
    totCost    -= iroute->getCost();  //总成本减去当前iroute的成本
    //totDrcCost -= iroute->getDrcCost();
    //auto tmpCost = assignIroute_getCost(iroute, trackLoc, drcCost);
    assignIroute_getCost(iroute, trackLoc, drcCost);  //计算当前iroute分配到轨道t的成本
    iroute->setCost(drcCost); //将iroute的成本设置为当前预估drcCost
    //iroute->setCost(tmpCost);
    //iroute->setDrcCost(drcCost);
    //totCost    += iroute->getCost();
    //totDrcCost += iroute->getDrcCost();
    totCost    += iroute->getCost();  //总成本加上当前iroute的预估成本
    if (drcCost && iroute->getNumAssigned() < maxRetry) {//若当前iroute分配过去后有成本，且iroute的被分配次数小于最大重试次数
      addToReassignIroutes(iroute); //将该iroute又加到再分配队列中
    }
  }
  if (enableOutput && pinS.size()) {
    cout <<"updated " <<pinS.size() <<" iroutes" <<endl;
  }
}

//分配iroute的函数
void FlexTAWorker::assignIroute(taPin* iroute) {  //传入一个iroute
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    bool   isH = (getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
    double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    frPoint bp, ep;
    frCoord bc, ec, trackLoc;
    cout <<"assigning " <<iroute->getId() <<" " <<iroute->getGuide()->getNet()->getName();
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
    cout <<", wlen_h@" <<iroute->getWlenHelper() <<", cost@" <<iroute->getCost() <<endl;
  }

  //按id从小到大排序iroute(taPin)的集合，set会自动排序
  set<taPin*, frBlockObjectComp> pinS;
  //根据当前待排序iroute初始化pinS集合
  assignIroute_init(iroute, &pinS);
  //当前iroute所在的层号和索引
  frLayerNum lNum;
  int idx1, idx2; //用来存放iroute可用轨道的索引范围
  //1.为iroute分配可用tracks
  assignIroute_availTracks(iroute, lNum, idx1, idx2);
  //2.在可用轨道idx1和idx2之间分配最佳轨道给iroute
  auto bestTrackLoc = assignIroute_bestTrack(iroute, lNum, idx1, idx2);
  //3.更新iroute的最优轨道坐标
  assignIroute_updateIroute(iroute, bestTrackLoc, &pinS);
  //4.更新pinS集合中的其他iroute的坐标和cost
  assignIroute_updateOthers(pinS);
}

//轨道分配的主过程 - FlexTA.cpp的main中调用此处函数
void FlexTAWorker::assign() {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (getTAIter() == -1) {  //taIter 是一个整数 - 但暂时不知道干嘛的
    return;
  }
  //if (isInitTA()) {
    int maxBufferSize = 20; //缓冲区尺寸设置为20
    //这个buffer是用来记录已经分配过的iroute，防止重复分配用的
    vector<taPin*> buffers(maxBufferSize, nullptr);
    //当前缓冲区已经分配过的iroute的数量
    int currBufferIdx = 0;
    //从重分配iroutes队列中取出一个iroute - 此时应该是第一次分配，但是重分配队列是重用了的，也可以拿来装第一次分配的iroute
    auto iroute = popFromReassignIroutes();
    //当仍然还有iroute时
    while (iroute != nullptr) {
      //找到当前iroute在buffers中的位置(迭代器)
      auto it = find(buffers.begin(), buffers.end(), iroute);
      // in the buffer, skip
      //若在缓冲区中存在，说明已经分配过了，或当前iroute的分配次数大于等于最大重试次数，则跳过
      if (it != buffers.end() || iroute->getNumAssigned() >= maxRetry) {
        ;
      // not in the buffer, re-assign
      } else {//否则，分配该iroute
        // re add last buffer item to reassigniroutes if drccost > 0
        assignIroute(iroute);
        //if (buffers[currBufferIdx]) {
        //  if (buffers[currBufferIdx]->getDrcCost()) {
        //    addToReassignIroutes(buffers[currBufferIdx]);
        //  }
        //}
        buffers[currBufferIdx] = iroute;  //将当前已分配的iroute放入缓冲区(相当于更新)，防止重复分配
        //iroute索引+1，并取模，防止越界
        currBufferIdx = (currBufferIdx + 1) % maxBufferSize;
        if (enableOutput && !isInitTA()) {
          //cout <<"totCost@" <<totCost <<"/" <<totDrcCost <<endl;
          cout <<"totCost@" <<totCost <<endl;
        }
        //已分配的iroute数+1
        numAssigned++;
      }
      iroute = popFromReassignIroutes();  //接着再从重分配iroutes队列中取出一个iroute
    }
  //}
}
