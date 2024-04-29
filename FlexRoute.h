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

#ifndef _FLEXROUTE_H_
#define _FLEXROUTE_H_

#include <memory>
#include "frBaseTypes.h"
#include "frDesign.h"

namespace fr {          //命名空间 fr
  class FlexRoute {     //FlexRoute类
  public:
    FlexRoute(): design(std::make_unique<frDesign>()) {}
    frDesign* getDesign() const { //返回design的原始指针,不要在使用原始时手动释放内存，否则会导致std::unique_ptr重复释放内存，引发错误
      return design.get();
    }
    int main();
  protected:  //unique_ptr是C++11下的智能指针，可以自动释放其所管理的对象，避免内存泄漏
  /*
    声明一个std::unique_ptr并不会自动创建对象，需要通过std::make_unique或new来创建对象并将其分配给智能指针,如：
    design = std::make_unique<frDesign>();或：
    design.reset(new frDesign());
  */
    std::unique_ptr<frDesign> design;
  
    void init();
    //void pinPrep();
    void prep();
    void gr();
    void ta();
    void dr();
    void endFR();
  };
}
#endif
