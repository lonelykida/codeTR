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

#ifndef _FR_BLOCKOBJECT_H_
#define _FR_BLOCKOBJECT_H_

#include "frBaseTypes.h"

namespace fr {
  //父级块对象类
  class frBlockObject {
  public:
    // constructors
    //frBlockObject() {}
    //无参构造，初始化类型为-1:块对象类型
    frBlockObject(): id(-1) {}  
    //拷贝构造
    frBlockObject(const frBlockObject &in): id(in.id) {}
    //虚析构
    virtual ~frBlockObject() {}
    // getters
    //获取对象类型的id
    int getId() const {
      return id;
    }
    // setters
    //设置对象类型的id
    void setId(int in) {
      id = in;
    }
    // others
    //返回对象类型的id - 是个枚举
    virtual frBlockObjectEnum typeId() const {
      return frcBlockObject;
    }
    //按id从小到大排序
    bool operator<(const frBlockObject &rhs) const {  
      return id < rhs.id;
    }
  protected:
    int id;
  };
  //父级块对象的比较函数，按id从小到大排序
  struct frBlockObjectComp {
    bool operator()(const frBlockObject* lhs, const frBlockObject* rhs) const {
      return *lhs < *rhs;
    }
  };
}

#endif
