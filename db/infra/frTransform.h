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

#ifndef _FR_TRANSFORM_H_
#define _FR_TRANSFORM_H_

#include "db/infra/frPoint.h"
#include "db/infra/frOrient.h"

namespace fr {
  //保存"偏移点"和"方向"-并且可以对"偏移点"和"方向"进行修改
  class frTransform {
  public:
    // constructor
    frTransform(): offset(), ori() {}
    frTransform(const frPoint &pointIn, const frOrient &orientIn = frOrient(frcR0)): offset(pointIn), ori(orientIn) {}
    frTransform(frCoord xOffsetIn, frCoord yOffsetIn, const frOrient &orientIn = frOrient(frcR0)): offset(xOffsetIn, yOffsetIn), ori(orientIn) {}
    // setters
    //设置偏移点
    void set(const frPoint &pointIn) {
      offset = pointIn;
    }
    //设置方向
    void set(const frOrient &orientIn) {
      ori = orientIn;
    }
    //设置偏移点和方向 - 传入的是 frPoint 和 frOrient
    void set(const frPoint &pointIn, const frOrient &orientIn) {
      set(pointIn);
      set(orientIn);
    }
    //设置偏移点 - 传入的是两个x，y坐标
    void set(frCoord xOffsetIn, frCoord yOffsetIn) {
      set(frPoint(xOffsetIn, yOffsetIn));
    }
    //设置偏移点和方向
    void set(frCoord xOffsetIn, frCoord yOffsetIn, const frOrient &orientIn) {
      set(xOffsetIn, yOffsetIn);
      set(orientIn);
    }
    // getters
    //获取偏移点的x坐标
    frCoord xOffset() const {
      return offset.x();
    }
    //获取偏移点的y坐标
    frCoord yOffset() const {
      return offset.y();
    }
    //获取方向
    frOrient orient() const {
      return ori;
    }
    // util
    //根据偏移方向更新偏移点的x和y
    void updateXform(frPoint &size) {
      switch(orient()) {
        //case frcR0: == default
        case frcR90:
          set(xOffset() + size.y(), yOffset()           );
          break;
        case frcR180: // verified
          set(xOffset() + size.x(), yOffset() + size.y());
          break;
        case frcR270:
          set(xOffset(),            yOffset() + size.x());
          break;
        case frcMY: // verified
          set(xOffset() + size.x(), yOffset()           );
          break;
        case frcMXR90:
          set(xOffset(),            yOffset()           );
          break;
        case frcMX: // verified
          set(xOffset(),            yOffset() + size.y());
          break;
        case frcMYR90:
          set(xOffset() + size.y(), yOffset() + size.x());
          break;
        default      : // verified
          set(xOffset(),            yOffset()           );
          break;
      }
    }
    //回滚偏移
    void revert(frTransform &transformIn) {
      frCoord resXOffset, resYOffset;
      frOrient resOrient;
      switch(ori) {
        case frcR0:
          resXOffset = -offset.x();
          resYOffset = -offset.y();
          resOrient = frcR0;
          break;
        case frcR90:
          resXOffset = -offset.y();
          resYOffset = offset.x();
          resOrient = frcR270;
          break;
        case frcR180:
          resXOffset = offset.x();
          resYOffset = offset.y();
          resOrient = frcR180;
          break;
        case frcR270:
          resXOffset = offset.y();
          resYOffset = -offset.x();
          resOrient = frcR90;
          break;
        case frcMY:
          resXOffset = offset.x();
          resYOffset = -offset.y();
          resOrient = frcMY;
          break;
        case frcMYR90:
          resXOffset = offset.y();
          resYOffset = offset.x();
          resOrient = frcMYR90;
          break;
        case frcMX:
          resXOffset = -offset.x();
          resYOffset = offset.y();
          resOrient = frcMX;
          break;
        case frcMXR90:
          resXOffset = -offset.y();
          resYOffset = -offset.x();
          resOrient = frcMXR90;
          break;
        default:
          resXOffset = -offset.x();
          resYOffset = -offset.y();
          resOrient = frcR0;
          std::cout << "Error: unrecognized orient in revert\n";
      }
      transformIn.set(resXOffset, resYOffset, resOrient);
    }
  protected:
    frPoint offset; // 偏移点
    frOrient ori;   // 偏移方向
  };
}

#endif
