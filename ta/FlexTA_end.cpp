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

using namespace std;
using namespace fr;

void FlexTAWorker::saveToGuides() {
  for (auto &iroute: iroutes) {  // 遍历所有内部路由对象
    for (auto &uPinFig: iroute->getFigs()) {  // 遍历路由对象中的所有图形元素
      if (uPinFig->typeId() == tacPathSeg) {  // 如果图形元素是路径段
        unique_ptr<frPathSeg> pathSeg = make_unique<frPathSeg>(*static_cast<taPathSeg*>(uPinFig.get()));  // 创建一个新的路径段对象，复制现有的taPathSeg对象
        pathSeg->addToNet(iroute->getGuide()->getNet());  // 将路径段添加到其所属网络
        auto guide = iroute->getGuide();  // 获取路由的导引对象
        vector<unique_ptr<frConnFig> > tmp;  // 创建一个容器来存储连接图形
        tmp.push_back(std::move(pathSeg));  // 将路径段移动到容器中
        guide->setRoutes(tmp);  // 设置导引的路由为新的路径段
      }
      // 可以在此处添加代码处理上下相邻的路径段，这些路径段将具有最长的线长
    }
  }
}


void FlexTAWorker::end() {
  //if (getTAIter() <= 0) {
    saveToGuides();
  //}
}
