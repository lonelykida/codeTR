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

#ifndef _FR_DESIGN_H_
#define _FR_DESIGN_H_

#include <memory>
#include "global.h"
#include "frBaseTypes.h"
#include "db/obj/frBlock.h"
#include "db/tech/frTechObject.h"
#include "frRegionQuery.h"

namespace fr {    //命名空间 fr
  namespace io {  //fr - io
    class Parser;
  }
  class frDesign {  //fr设计类
  public:
    // constructors //无参构造
    frDesign(): topBlock(nullptr)/*给topBlock赋值为NULL*/, tech(std::make_unique<frTechObject>()),/*初始化tech指针*/ 
                rq(std::make_unique<frRegionQuery>(this)/*初始化rq指针为当前类对象*/) {}
    // getters
    frBlock* getTopBlock() const {  //返回topBlock的原始指针
      return topBlock.get();
    }
    frTechObject* getTech() const { //返回tech的原始指针
      return tech.get();
    }
    frRegionQuery* getRegionQuery() const { //返回rq的原始指针
      return rq.get();
    }
    std::vector<std::unique_ptr<frBlock> >& getRefBlocks() {  //返回refBlocks的引用
      return refBlocks;
    }
    const std::vector<std::unique_ptr<frBlock> >& getRefBlocks() const {  //返回refBlocks的const引用
      return refBlocks;
    }
    // setters
    void setTopBlock(std::unique_ptr<frBlock> &in) {  //设置顶层块
      topBlock = std::move(in); //move转移资源所有权，确保在转移所有权后，原始的指针不再拥有资源所有权，避免资源的重复释放或内存泄漏
      //move后in变成nullptr，所有权已转移到topBlock
    }
    void addRefBlock(std::unique_ptr<frBlock> &in) {  //添加参考块
      name2refBlock[in->getName()] = in.get();  //将frBlock的名字作为key，frBlock的指针作为value添加到参考块的查询表中
      refBlocks.push_back(std::move(in));     //将in放到参考块中，move转移资源所有权，确保在转移所有权后原始指针不再拥有资源所有权，避免资源的重复释放或内存泄漏
    }
    // others
    void printAllMacros();  //打印所有宏
    void printAllComps();   //打印所有组件
    void printAllTerms();   //打印所有term
    void printCMap();       //打印CMap
    friend class io::Parser;//友元 - Parser能访问私有成员和保护成员(Parser是解析器的意思)
  protected:  //下边的很多智能指针参考T日通Route4.0文献
    std::unique_ptr<frBlock>                      topBlock;   //topBlock类型的智能指针       - 顶层块
    std::map<frString, frBlock*>                  name2refBlock;  //图结构-frString转frBlock - 参考块
    std::vector<std::unique_ptr<frBlock> >        refBlocks;  //数组，装的是frBlock类型的智能指针-参考块
    std::unique_ptr<frTechObject>                 tech;   //frTechObject类型的智能指针  - tech块类型
    std::unique_ptr<frRegionQuery>                rq;     //frRegionQuery类型的智能指针
  };
}

#endif
