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

void FlexPA::init() {
  initViaRawPriority();  // 初始化通孔的原始优先级
  initTrackCoords();     // 初始化轨道坐标

  initUniqueInstance();  // 初始化独特实例
  initPinAccess();       // 初始化引脚访问
}


void FlexPA::prep() {
  using namespace std::chrono;
  high_resolution_clock::time_point t0 = high_resolution_clock::now(); // 开始时间
  prepPoint();  // 准备引脚访问点
  high_resolution_clock::time_point t1 = high_resolution_clock::now(); // 准备后时间点1
  revertAccessPoints();  // 当前转换信息还原为原始布局中的位置
  high_resolution_clock::time_point t2 = high_resolution_clock::now(); // 准备后时间点2
  prepPattern();  // 准备访问模式
  high_resolution_clock::time_point t3 = high_resolution_clock::now(); // 准备后时间点3

  duration<double> time_span1 = duration_cast<duration<double>>(t1 - t0); // 计算时间段1
  duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2); // 计算时间段2
  cout << "Expt1 runtime (pin-level access point gen): " << time_span1.count() << endl; // 输出时间段1
  cout << "Expt2 runtime (design-level access pattern gen): " << time_span2.count() << endl; // 输出时间段2
}


int FlexPA::main() {
  //bool enableOutput = true;
  frTime t;
  if (VERBOSE > 0) {
    cout <<endl <<endl <<"start pin access" <<endl;
  }

  init();
  prep();

  int stdCellPinCnt = 0;  // 标准单元引脚计数
  for (auto &inst: getDesign()->getTopBlock()->getInsts()) {  // 遍历所有实例
    if (inst->getRefBlock()->getMacroClass() != MacroClassEnum::CORE) {
      continue;  // 跳过非核心宏
    }
    for (auto &instTerm: inst->getInstTerms()) {  // 遍历实例的所有引脚
      if (isSkipInstTerm(instTerm.get())) {
        continue;  // 跳过不应处理的实例
      }
      if (instTerm->hasNet()) {
        stdCellPinCnt++;  // 计数有网络连接的标准单元引脚
      }
    }
  }


  if (VERBOSE > 0) {
    cout <<"#scanned instances     = " <<inst2unique.size()     <<endl;
    cout <<"#unique  instances     = " <<uniqueInstances.size() <<endl;
    cout <<"#stdCellGenAp          = " <<stdCellPinGenApCnt           <<endl;
    cout <<"#stdCellValidPlanarAp  = " <<stdCellPinValidPlanarApCnt   <<endl;
    cout <<"#stdCellValidViaAp     = " <<stdCellPinValidViaApCnt      <<endl;
    cout <<"#stdCellPinNoAp        = " <<stdCellPinNoApCnt            <<endl;
    cout <<"#stdCellPinCnt         = " <<stdCellPinCnt                <<endl;
    cout <<"#instTermValidViaApCnt = " <<instTermValidViaApCnt        <<endl;
    cout <<"#macroGenAp            = " <<macroCellPinGenApCnt         <<endl;
    cout <<"#macroValidPlanarAp    = " <<macroCellPinValidPlanarApCnt <<endl;
    cout <<"#macroValidViaAp       = " <<macroCellPinValidViaApCnt    <<endl;
    cout <<"#macroNoAp             = " <<macroCellPinNoApCnt          <<endl;
  }

  if (VERBOSE > 0) {
    cout <<endl <<"complete pin access" <<endl;
    t.print();
    cout <<endl;
  }
  return 0;
}
