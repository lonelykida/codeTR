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

#ifndef _FR_FIG_H_
#define _FR_FIG_H_

#include <memory>
#include "db/obj/frBlockObject.h"
#include "db/infra/frBox.h"
#include "db/infra/frTransform.h"

namespace fr {
  //形状基类，继承自块，是所有形状的基类
  class frFig: public frBlockObject {
  public:
    // constructors
    frFig(): frBlockObject() {}
    // getters
    //获取形状的边界
    virtual void getBBox(frBox &box) const = 0;
    //virtual void getBBox(box_t &box) const = 0;
    // setters
    // others
    //frBlockObjectEnum typeId() const override = 0;
    virtual void move(const frTransform &xform) = 0;
    virtual bool overlaps(const frBox &box) const = 0;
  protected:
  };

  class frNet;
  //相连接的形状类
  class frConnFig: public frFig {
  public:
    // constructors
    frConnFig(): frFig() {}
    // getters
    virtual bool hasNet() const = 0;
    virtual frNet* getNet() const = 0;
    // setters
    virtual void addToNet(frNet* in) = 0;
    virtual void removeFromNet() = 0;
    // others
    //frBlockObjectEnum typeId() const override = 0;

    /* from frFig
     * getBBox
     * move
     * overlaps
     */
  protected:
  };

  class frPin;
  //pin形状类
  class frPinFig: public frConnFig {
  public:
    frPinFig(): frConnFig() {}
    // getters
    virtual bool hasPin() const = 0;
    virtual frPin* getPin() const = 0;
    // setters
    virtual void addToPin(frPin* in) = 0;
    virtual void removeFromPin() = 0;
    // others
    //frBlockObjectEnum typeId() const override = 0;

    /* from frConnFig
     * hasNet
     * getNet
     * addToNet
     * removeFromNet
     */

    /* from frFig
     * getBBox
     * move
     * overlaps
     */
  protected:
  };

}

#endif
