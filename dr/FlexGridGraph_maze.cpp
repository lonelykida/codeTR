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

#include "dr/FlexGridGraph.h"
#include "dr/FlexDR.h"

using namespace std;
using namespace fr;

//用于在迷宫搜索算法中扩展当前的波前网格
//接收当前波前网格、一个扩展的方向枚举、两个目标迷宫索引和一个中心点
/*inline*/ void FlexGridGraph::expand(FlexWavefrontGrid &currGrid, const frDirEnum &dir, 
                                      const FlexMazeIdx &dstMazeIdx1, const FlexMazeIdx &dstMazeIdx2,
                                      const frPoint &centerPt) {
  bool enableOutput = false;  //控制是否输出调试信息
  //bool enableOutput = true;
  frCost nextEstCost, nextPathCost; //存储下一个网格索引的估计成本和路径成本
  int gridX = currGrid.x();   //从currGrid获取当前网格的X、Y、Z坐标值
  int gridY = currGrid.y();
  int gridZ = currGrid.z();

  //根据当前方向dir计算下一个网格的坐标 - 此时的X,Y,Z就已经是下一个网格的坐标了
  getNextGrid(gridX, gridY, gridZ, dir);  
  
  //根据上边计算的下一个网格的坐标，新建索引nextIdx
  FlexMazeIdx nextIdx(gridX, gridY, gridZ);
  // get cost
  //计算从nextIdx到目标网格的估计成本
  nextEstCost = getEstCost(nextIdx, dstMazeIdx1, dstMazeIdx2, dir);
  //计算从当前网格沿dir方向的下一个网格的路径成本
  nextPathCost = getNextPathCost(currGrid, dir);  

  //若需要输出调试信息，输出下一个网格的估计成本和路径成本
  if (enableOutput) {
    std::cout << "  expanding from (" << currGrid.x() << ", " << currGrid.y() << ", " << currGrid.z() 
              << ") [pathCost / totalCost = " << currGrid.getPathCost() << " / " << currGrid.getCost() << "] to "
              << "(" << gridX << ", " << gridY << ", " << gridZ << ") [pathCost / totalCost = " 
              << nextPathCost << " / " << nextPathCost + nextEstCost << "]\n";
  }
  //FlexWavefrontGrid nextWavefrontGrid(gridX, gridY, gridZ, nextPathCost, nextPathCost + nextEstCost, currGrid.getBackTraceBuffer());
  //获取当前网格所在层的层号。
  auto lNum = getLayerNum(currGrid.z());
  //获取当前层的优选方向
  //auto isH  = getZDir(currGrid.z());
  //在非优选方向上对长度进行惩罚
  //frCoord via2viaLen = 1;// penalize length on non-pref dir
  //根据当前方向和层的优选方向来计算via2viaLen
  //if ((isH  && (dir == frDirEnum::W || dir == frDirEnum::E)) ||
  //    (!isH && (dir == frDirEnum::S || dir == frDirEnum::N))) {
  //  via2viaLen = getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir); 
  //}
  //如果当前网格的长度是最大值，则将via2viaLen设置为0
  //via2viaLen = (currGrid.getLength() == std::numeric_limits<frCoord>::max()) ? 0 : via2viaLen;
  //获取当前层的路径宽度
  auto pathWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();
  //通过当前网格的坐标获取对应的点
  frPoint currPt;
  getPoint(currPt, gridX, gridY);
  //计算当前点到中心点的曼哈顿距离
  frCoord currDist = abs(currPt.x() - centerPt.x()) + abs(currPt.y() - centerPt.y());

  // vlength calculation - 初始化当前网格的V长度变量
  frCoord currVLengthX = 0;
  frCoord currVLengthY = 0;
  //获取当前网格的VLength
  currGrid.getVLength(currVLengthX, currVLengthY);
  //将当前网格的VLength赋值给下一个网格的VLength
  auto nextVLengthX = currVLengthX;
  auto nextVLengthY = currVLengthY;
  //判断当前网格的上一个via是否是向上的 - 在search中传入的时候是true
  bool nextIsPrevViaUp = currGrid.isPrevViaUp();
  //如果扩展方向是向上或向下（即层间连接）
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    //重置XY方向的nextV长度，并根据移动方向设置nextIsPrevViaUp
    nextVLengthX = 0;
    nextVLengthY = 0;
    //当前扩展方向是向下，则下一个网格的"上一个扩展方向"应该是向上
    nextIsPrevViaUp = (dir == frDirEnum::D); // up via if current path goes down
  } else {  //如果扩展方向不是向上或向下，而是其他方向
    //如果当前网格X、Y方向的V长度不是最大值-根据移动方向增加下一个网格的VLength
    if (currVLengthX != std::numeric_limits<frCoord>::max() &&
        currVLengthY != std::numeric_limits<frCoord>::max()) {
      if (dir == frDirEnum::W || dir == frDirEnum::E) {
        //如果方向是西或东，X方向的长度增加当前网格到相邻网格的中心点之间的距离
        nextVLengthX += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
      } else { 
        //如果方向是南或北，Y方向的长度同上
        nextVLengthY += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
      }
    }
  }
  
  // tlength calculation - 用于计算和更新tLength，也就是“转折长度”（turn length）
  //获取当前网格的转折长度 - 用户输入时就输入的max()
  auto currTLength = currGrid.getTLength();
  //将当前网格的转折长度赋值给下一个网格的转折长度
  auto nextTLength = currTLength;
  // if there was a turn, then add tlength
  //如果当前网格的转折长度不是最大值（意味着之前有转折发生），则进入此条件块
  if (currTLength != std::numeric_limits<frCoord>::max()) {
    //在下一个网格的转折长度上加上当前方向的边长。这表示在计算路径成本时，
    //如果路径发生了转折，会增加一定的长度
    nextTLength += getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
  }
  //如果当前网格的最后一个方向不是未知，并且与当前方向不同（意味着发生了转折），
  //则进入此条件块
  // if current is a turn, then reset tlength
  if (currGrid.getLastDir() != frDirEnum::UNKNOWN && currGrid.getLastDir() != dir) {
    //重置下一个网格的转折长度为当前方向的边长。这通常表示在路径中发生了一个明显的转折，
    //因此需要重置转折长度计数
    nextTLength = getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir);
  }
  // if current is a via, then reset tlength
  //如果当前移动方向是向上或向下（通常表示通过一个via，即连接不同层的垂直连接），
  //则进入此条件块
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    //将下一个网格的转折长度设置为最大值。这通常用于重置转折长度计数，
    //因为通过via时路径的转折长度不再增加，或者表示在当前路径计算中不再考虑转折长度
    nextTLength = std::numeric_limits<frCoord>::max();
  }

  // preTurn calculation for planar turn
  // auto currGridLastDir = currGrid.getLastDir();
  // auto nextPreTurnDir = currGrid.getPreTurnDir();
  // if (dir == frDirEnum::U || dir == frDirEnum::D) {
  //   nextPreTurnDir = frDirEnum::UNKNOWN;
  // } else if (currGridLastDir != frDirEnum::UNKNOWN && currGridLastDir != dir && 
  //     dir != frDirEnum::U && dir != frDirEnum::D &&
  //     currGridLastDir != frDirEnum::U && currGridLastDir != frDirEnum::D) {
  //   nextPreTurnDir = currGridLastDir;
  // }

  //创建和更新FlexWavefrontGrid对象的过程,用于路径搜索
  FlexWavefrontGrid nextWavefrontGrid(gridX, gridY, gridZ, 
  //计算并设置波前网格的层路径面积，
  //它是当前网格的层路径面积加上沿当前方向的边长乘以路径宽度
                                      currGrid.getLayerPathArea() + getEdgeLength(currGrid.x(), currGrid.y(), currGrid.z(), dir) * pathWidth, 
                                      nextVLengthX, nextVLengthY, nextIsPrevViaUp,
                                      /*currGrid.getLength() + via2viaLen, currGrid.getLength() + via2viaLen, currGrid.isPrevViaUp(),*/
                                      nextTLength,
                                      currDist,
                                      nextPathCost, nextPathCost + nextEstCost, /*nextPreTurnDir, */currGrid.getBackTraceBuffer());
  //如果移动方向是向上或向下
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    //重置波前网格的层路径面积约束和长度
    nextWavefrontGrid.resetLayerPathArea();
    nextWavefrontGrid.resetLength();
    //根据移动方向设置下一个波前网格的上一个via的方向
    if (dir == frDirEnum::U) {
      nextWavefrontGrid.setPrevViaUp(false);
    } else {
      nextWavefrontGrid.setPrevViaUp(true);
    }
    //增加波前网格的层路径面积约束，计算上一个网格的一半via包围区域
    nextWavefrontGrid.addLayerPathArea((dir == frDirEnum::U) ? getHalfViaEncArea(currGrid.z(), false) : getHalfViaEncArea(gridZ, true));
  }
  //if (currGrid.getLastDir() != frDirEnum::UNKNOWN && currGrid.getLastDir() != dir) {
  //  nextWavefrontGrid.resetLength();
  //}
  // update wavefront buffer

  //更新波前缓冲区，将当前方向加入到波前网格的缓冲区，并获取缓冲区尾部的方向
  auto tailDir = nextWavefrontGrid.shiftAddBuffer(dir);
  // non-buffer enablement is faster for ripup all
  // if (drWorker && drWorker->getRipupMode() == 0) {
    // commit grid prev direction if needed
    
    //获取波前网格尾部的索引
    auto tailIdx = getTailIdx(nextIdx, nextWavefrontGrid);
    //如果尾部方向不是未知的
    if (tailDir != frDirEnum::UNKNOWN) {
      //如果尾部索引的上一个A*算法节点方向是未知的，或者与尾部方向相同
      if (getPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z()) == frDirEnum::UNKNOWN ||
          getPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z()) == tailDir) {
        //设置尾部索引的上一个A*算法节点方向，并把新的波前网格对象推入波前队列
        setPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z(), tailDir);
        wavefront.push(nextWavefrontGrid);
        //如果启用输出，打印提交的尾部索引和方向信息
        if (enableOutput) {
          std::cout << "    commit (" << tailIdx.x() << ", " << tailIdx.y() << ", " << tailIdx.z() << ") prev accessing dir = " << (int)tailDir << "\n";
        }
      }
    } else {  //如果尾部方向未知，则直接将新的波前网格对象推入波前队列
      // add to wavefront
      wavefront.push(nextWavefrontGrid);
    }

  // } else {
  //   // update grid astar cost if needed (non-buffer enablement)
  //   if (nextWavefrontGrid.getCost() < getAStarCost(gridX, gridY, gridZ) || !hasAStarCost(gridX, gridY, gridZ)) {
  //     setAStarCost(gridX, gridY, gridZ, nextPathCost);
  //     setPrevAstarNodeDir(gridX, gridY, gridZ, dir);
  //     wavefront.push(nextWavefrontGrid);
  //   }
  // }

  return;
}

//扩展波
/*inline*/ void FlexGridGraph::expandWavefront(FlexWavefrontGrid &currGrid, const FlexMazeIdx &dstMazeIdx1, 
                                               const FlexMazeIdx &dstMazeIdx2, const frPoint &centerPt) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (enableOutput) {
    cout << "start expand from (" << currGrid.x() << ", " << currGrid.y() << ", " << currGrid.z() << ")\n";
  }
  //if (currGrid.y() == 19 && currGrid.z() == 0) {
  //  cout <<"is expandable (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") NESWUD "
  //       <<isExpandable(currGrid, frDirEnum::N)
  //       <<isExpandable(currGrid, frDirEnum::E)
  //       <<isExpandable(currGrid, frDirEnum::S)
  //       <<isExpandable(currGrid, frDirEnum::W)
  //       <<isExpandable(currGrid, frDirEnum::U)
  //       <<isExpandable(currGrid, frDirEnum::D)
  //       <<endl;
  //  cout <<"has edge " 
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::N)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::E)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::S)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::W)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::U)
  //       <<gridGraph.hasEdge(currGrid.x(), currGrid.y(), currGrid.z(), frDirEnum::D)
  //       <<endl;
  //  int gridX = currGrid.x();
  //  int gridY = currGrid.y();
  //  int gridZ = currGrid.z();
  //  if (!gridGraph.hasEdge(gridX, gridY, gridZ, frDirEnum::E)) {
  //    ;
  //  } else {
  //    getNextGrid(gridX, gridY, gridZ, frDirEnum::E);
  //    if (gridGraph.isBlocked(gridX, gridY, gridZ)) {
  //      cout <<"blocked" <<endl;
  //    } else if (gridGraph.isSrc(gridX, gridY, gridZ)) {
  //      cout <<"src" <<endl;
  //    } else if (gridGraph.getPrevAstarNodeDir(gridX, gridY, gridZ) != frDirEnum::UNKNOWN) {
  //      cout <<"visited" <<endl;
  //    } else {
  //      ;
  //    }
  //  }
  //}
  //auto tmpGrid = currWavefrontGrid;
  //// commit grid prev direction if needed
  //auto tailIdx = getTailIdx(nextIdx, nextWavefrontGrid);
  //if (tailDir != frDirEnum::UNKNOWN) {
  //  if (getPrevAstarNodeDir(tailIdx.x(), tailIdx.y(), tailIdx.z()) == frDirEnum::UNKNOWN) {
  //    ;
  //  }
  //}
  // N
  
  //检查当前网格是否可以向北 (N) 扩展，如果可以，则调用 expand 函数在北方向上扩展波前
  if (isExpandable(currGrid, frDirEnum::N)) {
    expand(currGrid, frDirEnum::N, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no N" <<endl;
  // }
  // E

  if (isExpandable(currGrid, frDirEnum::E)) {
    expand(currGrid, frDirEnum::E, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no E" <<endl;
  // }
  // S
  if (isExpandable(currGrid, frDirEnum::S)) {
    expand(currGrid, frDirEnum::S, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no S" <<endl;
  // }
  // W
  if (isExpandable(currGrid, frDirEnum::W)) {
    expand(currGrid, frDirEnum::W, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no W" <<endl;
  // }
  // U
  if (isExpandable(currGrid, frDirEnum::U)) {
    expand(currGrid, frDirEnum::U, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no U" <<endl;
  // }
  // D
  if (isExpandable(currGrid, frDirEnum::D)) {
    expand(currGrid, frDirEnum::D, dstMazeIdx1, dstMazeIdx2, centerPt);
  }
  // else {
  //   std::cout <<"no D" <<endl;
  // }
}

//计算估计成本
/*inline*/ frCost FlexGridGraph::getEstCost(const FlexMazeIdx &src/*当前索引*/, 
                                const FlexMazeIdx &dstMazeIdx1/*最小索引*/,
                                const FlexMazeIdx &dstMazeIdx2/*最大索引*/, 
                                const frDirEnum &dir/*当前方向-初始时是UNKNOWN*/) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (enableOutput) {
    cout <<"est from (" <<src.x() <<", " <<src.y() <<", " <<src.z() <<") "
         <<"to ("       <<dstMazeIdx1.x() <<", " <<dstMazeIdx1.y() <<", " <<dstMazeIdx1.z() <<") ("
                        <<dstMazeIdx2.x() <<", " <<dstMazeIdx2.y() <<", " <<dstMazeIdx2.z() <<")";
  }
  // bend cost
  int bendCnt = 0;  //转弯计数 - 0
  int forbiddenPenalty = 0; //禁止惩罚 - 0
  frPoint srcPoint, dstPoint1/*最小*/, dstPoint2/*最大*/; //起点和(最小最大)终点1、终点2
  //设置起点的x,y坐标
  getPoint(srcPoint, src.x(), src.y());
  //设置(最小最大)终点的x,y坐标
  getPoint(dstPoint1, dstMazeIdx1.x(), dstMazeIdx1.y());
  getPoint(dstPoint2, dstMazeIdx2.x(), dstMazeIdx2.y());
  //auto minCostX = std::abs(srcPoint.x() - dstPoint.x()) * 1;
  //auto minCostY = std::abs(srcPoint.y() - dstPoint.y()) * 1;
  //auto minCostZ = std::abs(gridGraph.getZHeight(src.z()) - gridGraph.getZHeight(dst.z())) * VIACOST;
  //计算在X、Y、Z方向上的最小成本。使用max函数确保成本不会是负数
  frCoord minCostX = 
          max(max(dstPoint1.x() - srcPoint.x(), srcPoint.x() - dstPoint2.x()), 0) * 1;
  frCoord minCostY = 
          max(max(dstPoint1.y() - srcPoint.y(), srcPoint.y() - dstPoint2.y()), 0) * 1;
  frCoord minCostZ = 
          max(max(getZHeight(dstMazeIdx1.z()) - getZHeight(src.z()), 
                  getZHeight(src.z()) - getZHeight(dstMazeIdx2.z()))
                  , 0) * 1;
  if (enableOutput) {
    cout <<" x/y/z min cost = (" <<minCostX <<", " <<minCostY <<", " <<minCostZ <<") " <<endl;
  }
  //如果存在弯曲且方向不是未知的，并且方向不是与移动方向相反的，则增加弯曲计数
  bendCnt += (minCostX && dir != frDirEnum::UNKNOWN && dir != frDirEnum::E && dir != frDirEnum::W) ? 1 : 0;
  bendCnt += (minCostY && dir != frDirEnum::UNKNOWN && dir != frDirEnum::S && dir != frDirEnum::N) ? 1 : 0;
  bendCnt += (minCostZ && dir != frDirEnum::UNKNOWN && dir != frDirEnum::U && dir != frDirEnum::D) ? 1 : 0;
  //bendCnt -= bendCnt ? 1 : 0;
  // return (deltaX + deltaY + VIACOST * deltaZ + ((deltaX && deltaY) ? 1 : 0));
  if (enableOutput) {
    cout << "  est cost = " << minCostX + minCostY + minCostZ + bendCnt << endl;
  }
  //获取源点的网格坐标
  int gridX = src.x();
  int gridY = src.y();
  int gridZ = src.z();
  //获取当前点的下一个网格和方向
  getNextGrid(gridX, gridY, gridZ, dir);
  //创建下一个点的坐标
  frPoint nextPoint;
  getPoint(nextPoint, gridX, gridY);
  // avoid propagating to location that will cause fobidden via spacing to boundary pin
  //根据工艺计算禁止惩罚分数
  if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
    // if (drWorker && drWorker->getDRIter() >= 3) {
    if (drWorker && drWorker->getDRIter() >= 30 && drWorker->getRipupMode() == 0) {
      if (dstMazeIdx1 == dstMazeIdx2 && gridZ == dstMazeIdx1.z()) {
        //计算与当前网格 Z 坐标相关的层号。这里通过 (Z坐标 + 1) * 2 的公式计算
        auto layerNum = (gridZ + 1) * 2;
        //获取当前层号 layerNum 对应的层信息
        auto layer = getDesign()->getTech()->getLayer(layerNum);
        //判断当前层是否是水平优先布线方向
        bool isH = (layer->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
        // if ((isH && (dir == frDirEnum::E || dir == frDirEnum::W)) || 
            // (!isH && (dir == frDirEnum::N || dir == frDirEnum::S))) {
          if (isH) {  //是水平优先
            //计算当前点 nextPoint 与目标点 dstPoint1 在 Y 方向上的距离（间隙）
            auto gap = abs(nextPoint.y() - dstPoint1.y());
            if (gap &&
                (getDesign()->getTech()->isVia2ViaForbiddenLen(gridZ, false, false, false, gap, false) || layerNum - 2 < BOTTOM_ROUTING_LAYER) &&
                (getDesign()->getTech()->isVia2ViaForbiddenLen(gridZ, true, true, false, gap, false) || layerNum + 2 > getDesign()->getTech()->getTopLayerNum())) {
              /*计算禁止惩罚分数*/
              forbiddenPenalty = layer->getPitch() * ggDRCCost * 20;
            }
          } else {
            auto gap = abs(nextPoint.x() - dstPoint1.x());
            if (gap &&
                (getDesign()->getTech()->isVia2ViaForbiddenLen(gridZ, false, false, true, gap, false) || layerNum - 2 < BOTTOM_ROUTING_LAYER) &&
                (getDesign()->getTech()->isVia2ViaForbiddenLen(gridZ, true, true, true, gap, false) || layerNum + 2 > getDesign()->getTech()->getTopLayerNum())) {
              forbiddenPenalty = layer->getPitch() * ggDRCCost * 20;
            }
          }
        // }
      }
    }
  }

  //返回预估代价
  return (minCostX + minCostY + minCostZ + bendCnt + forbiddenPenalty);

}

//从提供的 bitset 对象中提取最后一个方向值，并将其转换为 frDirEnum 枚举类型
/*inline*/ frDirEnum FlexGridGraph::getLastDir(const std::bitset<WAVEFRONTBITSIZE> &buffer) {
  //这行代码首先调用 buffer 的 to_ulong 成员函数，
  //将 bitset 对象转换为一个无符号长整型值。然后，使用位运算 &（按位与）和掩码 0b111u
  //（二进制的 111，即十进制的 7）来提取这个长整型值的最低三位，
  //这三位代表了最后一个方向值 - 因为方向有6个，加上一个UNKNOWN，所以一共7个，需要3位存储
  auto currDirVal = buffer.to_ulong() & 0b111u;
  //将提取出的值 currDirVal 通过 static_cast 强制转换为 frDirEnum 枚举类型，并返回这个值
  return static_cast<frDirEnum>(currDirVal);
}

/*
  `0b111u` 是一个在 C++ 中用于表示数值的字面量，具体来说，
  它是一个二进制（binary）整数字面量。这种表示方法允许你直接以二进制形式写出数值。
  下面是对这种表示方法的详细解释：

- `0b` 或 `0B`：这是 C++ 中二进制字面量的前缀。
  任何以 `0b` 或 `0B` 开头的数字都将被识别为二进制数。

- `111`：这是实际的二进制数字，等同于十进制的 7。
  在二进制中，每个位的值是 2 的幂次方，从右到左分别是 2⁰、2¹、2²，以此类推。
  因此，`111` 表示的是 `1*2² + 1*2¹ + 1*2⁰`，计算结果为 7。

- `u`：这是后缀，表示这个数是一个无符号整数（unsigned integer）。
  在 C++ 中，你可以给整数字面量添加 `u` 或 `U` 后缀来明确它是一个无符号类型。
  对于二进制和八进制字面量，这是必需的，因为如果不加后缀，编译器可能会将它们误解为十进制数。

综上所述，`0b111u` 表示的是一个二进制表示的无符号整数，其值等同于十进制的 7。
这种表示方法在处理位运算或需要精确控制二进制位的场景中非常有用。
在上文的代码中，`0b111u` 被用作掩码，通过位运算提取出 `bitset` 中的最低三位。
*/


//更新网格坐标 gridX、gridY 和 gridZ 以反映在给定方向 dir 下的下一个网格位置。
void FlexGridGraph::getNextGrid(frMIdx &gridX, frMIdx &gridY, frMIdx &gridZ, const frDirEnum dir) {
  switch(dir) {
    case frDirEnum::E:
      ++gridX;
      break;
    case frDirEnum::S:
      --gridY;
      break;
    case frDirEnum::W:
      --gridX;
      break;
    case frDirEnum::N:
      ++gridY;
      break;
    case frDirEnum::U:
      ++gridZ;
      break;
    case frDirEnum::D:
      --gridZ;
      break;
    default:
      ;
  }
  return;
}

//根据当前方向dir计算前一个网格的坐标
void FlexGridGraph::getPrevGrid(frMIdx &gridX, frMIdx &gridY, frMIdx &gridZ, const frDirEnum dir) const {
  switch(dir) {
    case frDirEnum::E:
      --gridX;
      break;
    case frDirEnum::S:
      ++gridY;
      break;
    case frDirEnum::W:
      ++gridX;
      break;
    case frDirEnum::N:
      --gridY;
      break;
    case frDirEnum::U:
      --gridZ;
      break;
    case frDirEnum::D:
      ++gridZ;
      break;
    default:
      ;
  }
  return;
}

//
/*inline*/ frCost FlexGridGraph::getNextPathCost(const FlexWavefrontGrid &currGrid, const frDirEnum &dir) {
  // bool enableOutput = true;
  bool enableOutput = false;
  frMIdx gridX = currGrid.x();
  frMIdx gridY = currGrid.y();
  frMIdx gridZ = currGrid.z();
  frCost nextPathCost = currGrid.getPathCost();
  // bending cost
  //auto currDir = getLastDir(currGrid.getBackTraceBuffer());
  auto currDir = currGrid.getLastDir();
  // std::cout << "(currDir = " << (int)currDir << ") ";
  // if (currDir != dir && (int)currDir >= 1 && (int)currDir <= 4 &&
  //                       (int)dir     >= 1 && (int)dir     <= 4) {
  //   ++nextPathCost;
  // }
  auto lNum = getLayerNum(currGrid.z());
  auto pathWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();

  if (currDir != dir && currDir != frDirEnum::UNKNOWN) {
    // original
    ++nextPathCost;

    // test to give more penalty to turns
    // if (ggMarkerCost == 0) {
    //   nextPathCost += getEdgeLength(gridX, gridY, gridZ, dir);
    // } else {
    //   nextPathCost++;
    // }
  }

  //frCoord currArea = (currGrid.getLayerPathLength() + pathWidth) * pathWidth; 
  /* min-area enablement
  frCoord currArea = currGrid.getLayerPathArea();
  if (dir == frDirEnum::U) {
    currArea += getHalfViaEncArea(currGrid.z(), true);
  } else if (dir == frDirEnum::D) {
    currArea += getHalfViaEncArea(currGrid.z() - 1, false);
  }
  auto minAreaConstraint = getDesign()->getTech()->getLayer(lNum)->getAreaConstraint();
  if (minAreaConstraint && (dir == frDirEnum::U || dir == frDirEnum::D) && currArea < minAreaConstraint->getMinArea()) {
    nextPathCost += ggDRCCost * minAreaConstraint->getMinArea() / pathWidth;
    //nextPathCost += ggDRCCost * pathWidth;
    // if (TEST) {
    //   std::cout << "@@@MAR (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") dir = " << (int)dir << "\n";
    // }
  }
  */
  // via2viaminlen enablement
  //if (dir == frDirEnum::U && currGrid.getLength() != 0 && currGrid.getLength() < getVia2ViaMinLen(gridZ, currGrid.isPrevViaUp(), true)) {
  //if (dir == frDirEnum::U && 
  //    (allowVia2ViaZeroLen(gridZ, currGrid.isPrevViaUp(), true) ? !(currGrid.getLength() == 0) : true) && 
  //    currGrid.getLength() < getVia2ViaMinLen(gridZ, currGrid.isPrevViaUp(), true)) {
  //  nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //  // if (TEST) {
  //  //   std::cout << "@@@MAR (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") dir = " << (int)dir << "\n";
  //  // }
  //} else if (dir == frDirEnum::D && 
  //           (allowVia2ViaZeroLen(gridZ, currGrid.isPrevViaUp(), false) ? !(currGrid.getLength() == 0) : true) && 
  //           currGrid.getLength() < getVia2ViaMinLen(gridZ, currGrid.isPrevViaUp(), false)) {
  //  nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //  // if (TEST) {
  //  //   std::cout << "@@@MAR (" <<currGrid.x() <<", " <<currGrid.y() <<", " <<currGrid.z() <<") dir = " << (int)dir << "\n";
  //  // }
  //}

  // via2viaminlenNew enablement
  // if (dir == frDirEnum::U || dir == frDirEnum::D) {
  //   frCoord currVLengthX = 0;
  //   frCoord currVLengthY = 0;
  //   currGrid.getVLength(currVLengthX, currVLengthY);
  //   bool isCurrViaUp = (dir == frDirEnum::U);
  //   // if allow zero length and both x and y = 0 
  //   if (allowVia2ViaZeroLen(gridZ, currGrid.isPrevViaUp(), isCurrViaUp) && currVLengthX == 0 && currVLengthY == 0) {
  //     ;
  //   } else {
  //     // check only y
  //     if (currVLengthX == 0 && currVLengthY > 0 && currVLengthY < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, true)) {
  //       nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //     // check only x
  //     } else if (currVLengthX > 0 && currVLengthY == 0 && currVLengthX < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, false)) {
  //       nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //     // check both x and y
  //     } else if (currVLengthX > 0 && currVLengthY > 0 && 
  //                ((currVLengthY < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, true)) &&
  //                  currVLengthX < getVia2ViaMinLenNew(gridZ, currGrid.isPrevViaUp(), isCurrViaUp, false))) {
  //       nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //     }
  //   }
  // }

  // via2viaForbiddenLen enablement
  if (dir == frDirEnum::U || dir == frDirEnum::D) {
    frCoord currVLengthX = 0;
    frCoord currVLengthY = 0;
    currGrid.getVLength(currVLengthX, currVLengthY);
    bool isCurrViaUp = (dir == frDirEnum::U);
    bool isForbiddenVia2Via = false;
    // check only y
    if (currVLengthX == 0 && currVLengthY > 0 && getTech()->isVia2ViaForbiddenLen(gridZ, !(currGrid.isPrevViaUp()), !isCurrViaUp, false, currVLengthY, false)) {
      isForbiddenVia2Via = true;
    // check only x
    } else if (currVLengthX > 0 && currVLengthY == 0 && getTech()->isVia2ViaForbiddenLen(gridZ, !(currGrid.isPrevViaUp()), !isCurrViaUp, true, currVLengthX, false)) {
      isForbiddenVia2Via = true;
    // check both x and y
    } else if (currVLengthX > 0 && currVLengthY > 0 && 
               (getTech()->isVia2ViaForbiddenLen(gridZ, !(currGrid.isPrevViaUp()), !isCurrViaUp, false, currVLengthY) &&
                getTech()->isVia2ViaForbiddenLen(gridZ, !(currGrid.isPrevViaUp()), !isCurrViaUp, true, currVLengthX))) {
      isForbiddenVia2Via = true;
    }

    if (isForbiddenVia2Via) {
      if (drWorker && drWorker->getDRIter() >= 3) {
        nextPathCost += ggMarkerCost * getEdgeLength(gridX, gridY, gridZ, dir);
      } else {
        nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
      }
    }
  }


  // via2turnminlen enablement
  // frCoord tLength    = std::numeric_limits<frCoord>::max();
  // frCoord tLengthDummy = 0;
  // frCoord tLengthReq = 0;
  // //bool    isTLengthY = false;
  // bool    isTLengthViaUp = false;
  // if (currDir != frDirEnum::UNKNOWN && currDir != dir) {
  //   // next dir is a via
  //   if (dir == frDirEnum::U || dir == frDirEnum::D) {
  //     isTLengthViaUp = (dir == frDirEnum::U);
  //     // if there was a turn before
  //     if (tLength != std::numeric_limits<frCoord>::max()) {
  //       if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
  //         tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, false);
  //         tLength = currGrid.getTLength();
  //         //isTLengthY = false;
  //       } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
  //         tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, true);
  //         tLength = currGrid.getTLength();
  //         //isTLengthY = true;
  //       } else {
  //         ;
  //       }
  //     }
  //   // curr is a planar turn
  //   } else {
  //     isTLengthViaUp = currGrid.isPrevViaUp();
  //     if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
  //       tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, false);
  //       //isTLengthY = false;
  //       currGrid.getVLength(tLength, tLengthDummy);
  //     } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
  //       tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, true);
  //       //isTLengthY = true;
  //       currGrid.getVLength(tLengthDummy, tLength);
  //     } else {
  //       ;
  //     }
  //   }
  //   if (tLength < tLengthReq) {
  //     nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //   }
  // }

  // via2turn forbidden len enablement
  frCoord tLength    = std::numeric_limits<frCoord>::max();
  frCoord tLengthDummy = 0;
  // frCoord tLengthReq = 0;
  //bool    isTLengthY = false;
  bool    isTLengthViaUp = false;
  bool    isForbiddenTLen = false;
  if (currDir != frDirEnum::UNKNOWN && currDir != dir) {
    // next dir is a via
    if (dir == frDirEnum::U || dir == frDirEnum::D) {
      isTLengthViaUp = (dir == frDirEnum::U);
      // if there was a turn before
      if (tLength != std::numeric_limits<frCoord>::max()) {
        if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
          // tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, false);
          tLength = currGrid.getTLength();
          if (getTech()->isViaForbiddenTurnLen(gridZ, !isTLengthViaUp, true, tLength)) {
            isForbiddenTLen = true;
          }
          //isTLengthY = false;
        } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
          // tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, true);
          tLength = currGrid.getTLength();
          if (getTech()->isViaForbiddenTurnLen(gridZ, !isTLengthViaUp, false, tLength)) {
            isForbiddenTLen = true;
          }
          //isTLengthY = true;
        } else {
          ;
        }
      }
    // curr is a planar turn
    } else {
      isTLengthViaUp = currGrid.isPrevViaUp();
      if (currDir == frDirEnum::W || currDir == frDirEnum::E) {
        // tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, false);
        //isTLengthY = false;
        currGrid.getVLength(tLength, tLengthDummy);
        if (getTech()->isViaForbiddenTurnLen(gridZ, !isTLengthViaUp, true, tLength)) {
          isForbiddenTLen = true;
        }
      } else if (currDir == frDirEnum::S || currDir == frDirEnum::N) {
        // tLengthReq = getVia2TurnMinLen(gridZ, isTLengthViaUp, true);
        //isTLengthY = true;
        currGrid.getVLength(tLengthDummy, tLength);
        if (getTech()->isViaForbiddenTurnLen(gridZ, !isTLengthViaUp, false, tLength)) {
          isForbiddenTLen = true;
        }
      } else {
        ;
      }
    }
    if (isForbiddenTLen) {
      if (drWorker && drWorker->getDRIter() >= 3) {
        nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
      } else {
        nextPathCost += ggMarkerCost * getEdgeLength(gridX, gridY, gridZ, dir);
      }
    }
  }

  // min length planar U turn enablement
  // if (currDir != frDirEnum::UNKNOWN && currDir != dir &&
  //     currDir != frDirEnum::U && currDir != frDirEnum::D &&
  //     dir != frDirEnum::U && dir != frDirEnum::D) {
  //   auto preTurnDir = currGrid.getPreTurnDir();
  //   if (preTurnDir != frDirEnum::UNKNOWN && 
  //       ((int)preTurnDir + (int)dir) == OPPOSITEDIR &&
  //       currGrid.getTLength() < (int)pathWidth) {
  //     nextPathCost += ggDRCCost * getEdgeLength(gridX, gridY, gridZ, dir);
  //   }
  // }




  //if (currDir != dir && currDir != frDirEnum::UNKNOWN && currGrid.getLength() != 0 && currGrid.getLength() < (frCoord)pathWidth * 2) {
  //  nextPathCost += ggDRCCost * pathWidth * 2;
  //}
  //frMIdx nextGridX = gridX;
  //frMIdx nextGridY = gridY;
  //frMIdx nextGridZ = gridZ;
  //getNextGrid(nextGridX, nextGridY, nextGridZ, dir);
  //nextPathCost += gridGraph.getEdgeCost(gridX, gridY, gridZ, dir) * gridGraph.getEdgeLength(gridX, gridY, gridZ, dir);
  bool gridCost   = hasGridCost(gridX, gridY, gridZ, dir);
  bool drcCost    = hasDRCCost(gridX, gridY, gridZ, dir);
  bool markerCost = hasMarkerCost(gridX, gridY, gridZ, dir);
  bool shapeCost  = hasShapeCost(gridX, gridY, gridZ, dir);
  bool blockCost  = isBlocked(gridX, gridY, gridZ, dir);
  bool guideCost  = hasGuide(gridX, gridY, gridZ, dir);

  // temporarily disable guideCost
  nextPathCost += getEdgeLength(gridX, gridY, gridZ, dir)
                  + (gridCost   ? GRIDCOST         * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  + (drcCost    ? ggDRCCost        * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  + (markerCost ? ggMarkerCost     * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  // + (markerCost ? ggMarkerCost     * pathWidth                               : 0)
                  + (shapeCost  ? SHAPECOST        * getEdgeLength(gridX, gridY, gridZ, dir) : 0)
                  + (blockCost  ? BLOCKCOST        * pathWidth * 20                          : 0)
                  + (!guideCost ? GUIDECOST        * getEdgeLength(gridX, gridY, gridZ, dir) : 0);
  if (enableOutput) {
    cout <<"edge grid/shape/drc/marker/blk/length = " 
         <<hasGridCost(gridX, gridY, gridZ, dir)   <<"/"
         <<hasShapeCost(gridX, gridY, gridZ, dir)  <<"/"
         <<hasDRCCost(gridX, gridY, gridZ, dir)    <<"/"
         <<hasMarkerCost(gridX, gridY, gridZ, dir) <<"/"
         <<isBlocked(gridX, gridY, gridZ, dir) <<"/"
         //<<hasGuide(gridX, gridY, gridZ, dir) <<"/"
         <<getEdgeLength(gridX, gridY, gridZ, dir) <<endl;
  }
  return nextPathCost;

}



//通过对当前网格的回溯缓冲区进行解码，来找到路径搜索中波前尾部的网格索引
//接收两个参数：一个 FlexMazeIdx 类型的引用参数 currIdx，表示当前网格索引；
//一个 FlexWavefrontGrid 类型的引用参数 currGrid，表示当前波前网格
/*inline*/ FlexMazeIdx FlexGridGraph::getTailIdx(const FlexMazeIdx &currIdx, const FlexWavefrontGrid &currGrid) {
  //从 currIdx 中提取当前网格的 X、Y 和 Z 坐标
  int gridX = currIdx.x();
  int gridY = currIdx.y();
  int gridZ = currIdx.z();
  //从 currGrid 中获取回溯缓冲区，这个缓冲区存储了路径的回溯信息
  auto backTraceBuffer = currGrid.getBackTraceBuffer();
  //循环次数由 WAVEFRONTBUFFERSIZE 定义，这个宏定义了波前缓冲区的大小
  for (int i = 0; i < WAVEFRONTBUFFERSIZE; ++i) {
    //从回溯缓冲区中提取当前的方向值。首先将缓冲区转换为无符号长整型，
    //然后通过右移和左移操作来去除除了最低 DIRBITSIZE 位之外的所有位
    int currDirVal = backTraceBuffer.to_ulong() - ((backTraceBuffer.to_ulong() >> DIRBITSIZE) << DIRBITSIZE);
    //将提取的方向值转换为 frDirEnum 类型的枚举值
    frDirEnum currDir = static_cast<frDirEnum>(currDirVal);
    //将回溯缓冲区右移 DIRBITSIZE 位，以便在下一次循环迭代中读取下一个方向值
    backTraceBuffer >>= DIRBITSIZE;
    getPrevGrid(gridX, gridY, gridZ, currDir);
  }
  return FlexMazeIdx(gridX, gridY, gridZ);
}

/*inline*/ bool FlexGridGraph::isExpandable(const FlexWavefrontGrid &currGrid, frDirEnum dir) {
  //bool enableOutput = true;
  bool enableOutput = false;
  frMIdx gridX = currGrid.x();
  frMIdx gridY = currGrid.y();
  frMIdx gridZ = currGrid.z();
  //bool hg = hasEdge(gridX, gridY, gridZ, dir) && hasGuide(gridX, gridY, gridZ, dir);
  bool hg = hasEdge(gridX, gridY, gridZ, dir);
  if (enableOutput) {
    if (!hasEdge(gridX, gridY, gridZ, dir)) {
      cout <<"no edge@(" <<gridX <<", " <<gridY <<", " <<gridZ <<") " <<(int)dir <<endl;
    }
    if (hasEdge(gridX, gridY, gridZ, dir) && !hasGuide(gridX, gridY, gridZ, dir)) {
      cout <<"no guide@(" <<gridX <<", " <<gridY <<", " <<gridZ <<") " <<(int)dir <<endl;
    }
  }
  reverse(gridX, gridY, gridZ, dir);
  if (!hg || 
      isSrc(gridX, gridY, gridZ) || 
      (/*drWorker && drWorker->getRipupMode() == 0 &&*/ getPrevAstarNodeDir(gridX, gridY, gridZ) != frDirEnum::UNKNOWN) || // comment out for non-buffer enablement
      currGrid.getLastDir() == dir) {
    return false;
  } else {
    return true;
  }
}

//路径回溯- √
void FlexGridGraph::traceBackPath(const FlexWavefrontGrid &currGrid, vector<FlexMazeIdx> &path, vector<FlexMazeIdx> &root,
                                  FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2) {
  //bool enableOutput = true;
  bool enableOutput = false;  //调试信息
  if (enableOutput) {
    cout << "    start traceBackPath...\n";
  }
  //分别用于存储上一个方向和当前方向
  frDirEnum prevDir = frDirEnum::UNKNOWN, currDir = frDirEnum::UNKNOWN;
  //从 currGrid 获取当前网格的 X、Y 和 Z 坐标
  int currX = currGrid.x(), currY = currGrid.y(), currZ = currGrid.z();
  // pop content in buffer
  //获取当前网格的回溯缓冲区，这个缓冲区存储了路径的回溯信息 - 是存储方向的bit数组
  auto backTraceBuffer = currGrid.getBackTraceBuffer();

  //for 循环负责处理已知的、存储在缓冲区中的路径信息 - 波前缓冲区中的路径信息
  //循环遍历回溯缓冲区的大小，WAVEFRONTBUFFERSIZE 是定义波前缓冲区大小的宏
  for (int i = 0; i < WAVEFRONTBUFFERSIZE; ++i) { //WAVEFRONTBUFFERSIZE = 2
    // current grid is src
    if (isSrc(currX, currY, currZ)) { //如果当前网格是源点，则退出循环
      break;
    }         
    // get last direction
    //获取回溯缓冲区中的最后一个方向，并右移 DIRBITSIZE 位以准备读取下一个方向
    currDir = getLastDir(backTraceBuffer);
    backTraceBuffer >>= DIRBITSIZE;//右移3位
    //如果当前方向是未知的，则打印警告信息并退出循环
    if (currDir == frDirEnum::UNKNOWN) {
      cout << "Warning: unexpected direction in tracBackPath\n";
      break;
    }
    // add to root // when path makes a planar turn, dropping via on the turn is dangerous. Avoid it.
    // if (prevDir != frDirEnum::UNKNOWN && 
    //     currDir != prevDir && 
    //     (currDir != frDirEnum::U && currDir != frDirEnum::D && prevDir != frDirEnum::U && prevDir != frDirEnum::U)) {
      //将当前网格索引添加到 root 向量中
      root.push_back(FlexMazeIdx(currX, currY, currZ));
    // }
    // push point to path
    //如果当前方向与上一个方向不同，则将当前网格索引添加到 path 向量中，并如果启用输出，则打印当前网格索引
    //确保不会将相同方向的网格索引重复添加到 path 向量中，从而减少存储空间并可能简化后续处理。
    //因为WAVEFRONTBUFFERSIZE=2，所以波前缓冲区只能存储两个方向信息，存储的是最近的方向变化信息
    if (currDir != prevDir) {
      path.push_back(FlexMazeIdx(currX, currY, currZ));
      if (enableOutput) {
        cout <<" -- (" <<currX <<", " <<currY <<", " <<currZ <<")";
      }
    }
  //根据当前方向获取上一个网格的位置，并更新 prevDir
    getPrevGrid(currX, currY, currZ, currDir);
    prevDir = currDir;  //当前方向赋值给prevDir,确保在下次循环中，能检查计新计算出的当前方向currDir是否
    //与上一个记录的方向prevDir相等。若不相等则如前所述，表示路径在此处发生了转向，因此需要将这个新的位置
    //添加到路径向量path中。
  }

  //while 循环负责完成剩余的路径回溯任务，直到确保整个路径被完全重建并到达源点
  //因为for循环执行完毕后，缓冲区中的路径不一定能确保回到源点
  // trace back according to grid prev dir
  //在当前网格不是源点的情况下，继续循环，直到到达源点
  while (isSrc(currX, currY, currZ) == false) {
    // get last direction
    //获取当前网格的上一个 A* 算法节点的方向
    currDir = getPrevAstarNodeDir(currX, currY, currZ);
    // add to root
    // if (prevDir != frDirEnum::UNKNOWN && 
    //     currDir != prevDir && 
    //     (currDir != frDirEnum::U && currDir != frDirEnum::D && prevDir != frDirEnum::U && prevDir != frDirEnum::U)) {
      root.push_back(FlexMazeIdx(currX, currY, currZ));
    // }
    //如果当前方向是未知的，则打印警告信息并退出循环
    if (currDir == frDirEnum::UNKNOWN) {
      cout << "Warning: unexpected direction in tracBackPath\n";
      break;
    }
    //如果当前方向与上一个方向不同，则将当前网格索引添加到 path 向量中，并如果启用输出，则打印当前网格索引
    if (currDir != prevDir) {
      path.push_back(FlexMazeIdx(currX, currY, currZ));
      if (enableOutput) {
        cout <<" -- (" <<currX <<", " <<currY <<", " <<currZ <<")";
      }
    }
    //根据当前方向获取上一个网格的位置，并更新 prevDir
    getPrevGrid(currX, currY, currZ, currDir);
    prevDir = currDir;
  }


  // add final path to src, only add when path exists; no path exists (src = dst)
  //如果路径不为空，则将源点添加到路径中，并如果启用输出，则打印源点坐标
  //到此处时，currX、currY、currZ 一定为源点坐标(否则不会退出循环)，除非路径方向为UNKNOWN
  if (!path.empty()) {
    path.push_back(FlexMazeIdx(currX, currY, currZ));
    if (enableOutput) {
      cout <<" -- (" <<currX <<", " <<currY <<", " <<currZ <<")";
    }
  }
  //遍历路径中的每个点
  for (auto &mi: path) {
    //更新 ccMazeIdx1 和 ccMazeIdx2 以包含路径中所有点的最小和最大坐标
    ccMazeIdx1.set(min(ccMazeIdx1.x(), mi.x()),
                   min(ccMazeIdx1.y(), mi.y()),
                   min(ccMazeIdx1.z(), mi.z()));
    ccMazeIdx2.set(max(ccMazeIdx2.x(), mi.x()),
                   max(ccMazeIdx2.y(), mi.y()),
                   max(ccMazeIdx2.z(), mi.z()));
  }
  if (enableOutput) {
    cout <<endl;
  }
}

/*
  1.vector<FlexMazeIdx> &connComps:包含了每个可能的网格点索引。在布线过程中，这些索引代表起始点或已经布线的网络部分
  2.drPin* nextPin:下一个需要连接的针脚
  3.vector<FlexMazeIdx> &path:储搜索算法找到的路径，是从起始点到 nextPin 的一系列网格索引，表示了布线的路线
  4.FlexMazeIdx &ccMazeIdx1:连接组件的左下迷宫索引，连接组件的最小索引，用于确定搜索空间的边界
  5.FlexMazeIdx &ccMazeIdx2:连接组件的右上迷宫索引，连接组件的最大索引，同样用于确定搜索空间的边界
  6.frPoint &centerPt:中心点，代表了搜索区域的中心点，可能用于启发式搜索算法中的估算成本计算
*/

//路径搜索算法
bool FlexGridGraph::search(vector<FlexMazeIdx> &connComps, drPin* nextPin, vector<FlexMazeIdx> &path, 
                           FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2, const frPoint &centerPt) {
  //bool enableOutput = true;
  bool enableOutput = false; // 控制调试输出的开关
  int stepCnt = 0; // 步骤计数器

  // prep nextPinBox// 准备下一个针脚的包围盒区域
  frMIdx xDim, yDim, zDim;
  getDim(xDim, yDim, zDim);// 获取网格维度 - 其实就是数组里的int元素个数
  FlexMazeIdx dstMazeIdx1(xDim - 1, yDim - 1, zDim - 1);  //最大索引的下标
  FlexMazeIdx dstMazeIdx2(0, 0, 0); //最小索引初始化为0
  FlexMazeIdx mi; // 当前网格索引
  for (auto &ap: nextPin->getAccessPatterns()) { // 遍历针脚的所有访问模式
    ap->getMazeIdx(mi); // 获取每个访问模式的网格索引
    dstMazeIdx1.set(min(dstMazeIdx1.x(), mi.x()),   // 更新最小索引
                    min(dstMazeIdx1.y(), mi.y()),
                    min(dstMazeIdx1.z(), mi.z()));
    dstMazeIdx2.set(max(dstMazeIdx2.x(), mi.x()),   // 更新最大索引
                    max(dstMazeIdx2.y(), mi.y()),
                    max(dstMazeIdx2.z(), mi.z()));
  }

  // std::cout << "start astarSearch\n";
  //wavefront = FlexWavefront();
  wavefront.cleanup();// 清理之前的波前数据
  //cout <<"xxx1" <<endl;
  // init wavefront
  // 初始化波前
  frPoint currPt;
  //对每个索引进行遍历 - 计算其曼哈顿距离，并放入波前wavefront队列中,直到到达目标点，将目标点放入path队列
  for (auto &idx: connComps) {
    if (isDst(idx.x(), idx.y(), idx.z())) { // 如果当前取出的索引已经是目标点
      if (enableOutput) {
        cout <<"message: astarSearch dst covered (" <<idx.x() <<", " <<idx.y() <<", " <<idx.z() <<")" <<endl;
      }
      path.push_back(FlexMazeIdx(idx.x(), idx.y(), idx.z()));// 将目标点添加到路径中
      return true;  // 如果已经找到了目标点，直接返回true
    }
    // get min area min length
    auto lNum = getLayerNum(idx.z()); // 获取当前层号(即z的值)
    //auto pathWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();
    //获取当前层的最小面积约束fakeArea
    auto minAreaConstraint = getDesign()->getTech()->getLayer(lNum)->getAreaConstraint();
    frCoord fakeArea = minAreaConstraint ? minAreaConstraint->getMinArea() : 0;
    //将当前访问到的索引值放到currPt中
    getPoint(currPt, idx.x(), idx.y());
    //计算当前曼哈顿距离currDist
    frCoord currDist = abs(currPt.x() - centerPt.x()) + abs(currPt.y() - centerPt.y());
    //frCoord fakeLen  = getVia2ViaMinLen(idx.z(), true, true);
    FlexWavefrontGrid currGrid(idx.x(), idx.y(), idx.z(), fakeArea, 
                               std::numeric_limits<frCoord>::max(), std::numeric_limits<frCoord>::max(), true, 
                               /*fakeLen, fakeLen, true,*/
                               std::numeric_limits<frCoord>::max(),
                               currDist, 0, 
                               getEstCost(idx/*当前索引*/, dstMazeIdx1/*最小索引*/, dstMazeIdx2/*最大索引*/, 
                               frDirEnum::UNKNOWN/*当前方向*/)/*, frDirEnum::UNKNOWN*/);
    wavefront.push(currGrid);// 将当前网格节点推入波前
    if (enableOutput) {
      cout <<"src add to wavefront (" <<idx.x() <<", " <<idx.y() <<", " <<idx.z() <<")" <<endl;
    }
  }
  //cout <<"xxx2" <<endl;
  // 执行A*搜索
  while(!wavefront.empty()) {
    // std::cout << "here1\n";
    auto currGrid = wavefront.top();// 取出当前波前的最优点
    wavefront.pop(); // 弹出当前点
    // if (drWorker && drWorker->getRipupMode() == 0) {
      //当前网格（currGrid）在之前的A*搜索中已经有了记录的方向
      //（也就是说，这个网格已经被考虑过，并且记录了从一个方向到达它的信息）则直接跳过
      if (getPrevAstarNodeDir(currGrid.x(), currGrid.y(), currGrid.z()) != frDirEnum::UNKNOWN) {
        continue;
      }
    // } else {
    //   if (hasAStarCost(currGrid.x(), currGrid.y(), currGrid.z()) && getAStarCost(currGrid.x(), currGrid.y(), currGrid.z()) != currGrid.getPathCost()) {
    //     // cout << getAStarCost(currGrid.x(), currGrid.y(), currGrid.z()) << " != " << currGrid.getCost() << " continue\n";
    //     continue;
    //   }
    // }

    // test
    if (enableOutput) {
      ++stepCnt;// 步骤计数器增加
    }
    // if (stepCnt % 100000 == 0) {
    //   std::cout << "wavefront size = " << wavefront.size() << " at step = " << stepCnt << "\n";
    // }
    if (isDst(currGrid.x(), currGrid.y(), currGrid.z())) {// 如果到达目标点
      traceBackPath(currGrid, path, connComps, ccMazeIdx1, ccMazeIdx2);// 回溯路径
      if (enableOutput) {
        cout << "path found. stepCnt = " << stepCnt << "\n";
      }
      return true;
    } else {  //否则将当前点放入波前
      // expand and update wavefront
      expandWavefront(currGrid, dstMazeIdx1, dstMazeIdx2, centerPt);// 扩展波前
    }
    
  }
  return false;// 如果波前为空，返回失败
}

