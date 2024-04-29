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
#include "global.h"
#include "FlexRoute.h"
#include "io/io.h"
#include "pa/FlexPA.h"
#include "ta/FlexTA.h"
#include "dr/FlexDR.h"
//#include "io/frPinPrep.h"
#include "gc/FlexGC.h"
#include "gr/FlexGR.h"
#include "rp/FlexRP.h"

using namespace std;
using namespace fr;

void FlexRoute::init() {    //初始化过程
  io::Parser parser(getDesign()); //将router的原始指针传给parser,parser是指向design的指针
  parser.readLefDef();      //读取LEFDEF文件
  if (GUIDE_FILE != string("")) { //若GUIDE_FILE不为空，则读取GUIDE_FILE
    parser.readGuide();
  } else {                  //否则没有GUIDE文件，VIA是否生成选项变成FALSE
    ENABLE_VIA_GEN = false;
  }
  parser.postProcess();     //设置VIA的一些属性
  FlexPA pa(getDesign());   //将design的指针传给pa，pa是指向design的指针
  pa.main();                //pa主程序 - 还没看
  if (GUIDE_FILE != string("")) { //若GUIDE文件存在
    parser.postProcessGuide();    //设置GCELL模式等等
  }
  // GR-related - 全局布线相关的东西
  parser.initRPin();  //post进程初始化Rpin区域查询
}

void FlexRoute::prep() {
  FlexRP rp(getDesign(), getDesign()->getTech());
  rp.main();
}

void FlexRoute::gr() {
  FlexGR gr(getDesign()); //全局布线
  gr.main();
}

void FlexRoute::ta() {
  FlexTA ta(getDesign());
  ta.main();    //轨道分配
  io::Writer writer(getDesign());
  writer.writeFromTA();   //最后会写一个DEF
}

void FlexRoute::dr() {
  FlexDR dr(getDesign()); //详细布线
  dr.main();
}

void FlexRoute::endFR() { //收尾工作
  io::Writer writer(getDesign());
  writer.writeFromDR();   //写入DEF
  if (REF_OUT_FILE != DEF_FILE) { //若俩名字不一样，则删除REF_OUT_FILE
    remove(REF_OUT_FILE.c_str());
  }
}

int FlexRoute::main() {
  init();   //首先初始化
  if (GUIDE_FILE == string("")) { //若GUIDE文件不存在，则直接执行GR
    gr();   //global routing结束后会生成guide文件
    io::Parser parser(getDesign()); //将router的原始指针传给parser,parser是指向design的指针
    GUIDE_FILE = OUTGUIDE_FILE; //修改GUIDE_FILE为输出的GUIDE
    ENABLE_VIA_GEN = true;      //VIA是否生成选项变成TRUE
    parser.readGuide();         //读取GUIDE
    parser.initDefaultVias();   //初始化默认VIA
    parser.writeRefDef();       //写入REFDEF
    parser.postProcessGuide();  //设置GCELL模式等等
  }
  prep();   //pin prep
  ta();     //轨道分配
  dr();     //详细布线
  endFR();  //结束布线


  /*
  // rtree test
  vector<rtree_frConnFig_value_t> result1;
  design->getTopBlock()->queryRtree4Routes(frBox(585000, 1098000, 590000, 1101000), 6, result1);
  cout <<endl <<"query1:" <<endl;
  for (auto &it: result1) {
    if (it.second->typeId() == frcPathSeg) {
      frPoint pt1, pt2;
      dynamic_pointer_cast<frPathSeg>(it.second)->getPoints(pt1, pt2);
      cout <<"found pathseg " <<pt1.x() <<" " <<pt1.y() <<" " << pt2.x() <<" " <<pt2.y() 
           <<" " <<dynamic_pointer_cast<frPathSeg>(it.second)->getNet()->getName() <<endl;
    } else if (it.second->typeId() == frcGuide) {
      frPoint pt1, pt2;
      dynamic_pointer_cast<frGuide>(it.second)->getPoints(pt1, pt2);
      cout <<"found guide   " <<pt1.x() <<" " <<pt1.y() <<" " << pt2.x() <<" " <<pt2.y()
           <<" " <<dynamic_pointer_cast<frGuide>(it.second)->getNet()->getName();
      if (dynamic_pointer_cast<frGuide>(it.second)->getBeginLayerNum() !=
          dynamic_pointer_cast<frGuide>(it.second)->getEndLayerNum()) {
        cout <<" via guide";
      }
      cout <<endl;
    }
  }
  exit(0);
  */
  return 0;
}

