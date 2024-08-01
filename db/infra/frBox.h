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

#ifndef _FR_BOX_H_
#define _FR_BOX_H_

#include "db/infra/frPoint.h"
#include "frBaseTypes.h"

namespace fr
{
    // 盒子 - 保存盒子的左下角和右上角
    class frBox
    {
    public:
        // constructor
        frBox() : ll(), ur() {} // 左下和右上角坐标点
        frBox(const frBox &tmpBox) : ll(tmpBox.ll), ur(tmpBox.ur) {}
        frBox(const box_t &in)
        {
            auto minCorner = in.min_corner();
            auto maxCorner = in.max_corner();
            set(minCorner.x(), minCorner.y(), maxCorner.x(), maxCorner.y());
        }
        frBox(frCoord llx, frCoord lly, frCoord urx, frCoord ury)
        {
            ll.set((llx > urx) ? urx : llx,
                   (lly > ury) ? ury : lly);
            ur.set((llx > urx) ? llx : urx,
                   (lly > ury) ? lly : ury);
        }
        frBox(const frPoint &tmpLowerLeft, const frPoint &tmpUpperRight)
            : frBox(tmpLowerLeft.x(), tmpLowerLeft.y(), tmpUpperRight.x(), tmpUpperRight.y()) {}
        // setters
        void set(const frBox &tmpBox)
        {
            ll.set(tmpBox.ll);
            ur.set(tmpBox.ur);
        }
        //设置左下角和右上角坐标
        void set(frCoord llx, frCoord lly, frCoord urx, frCoord ury)
        {
            ll.set((llx > urx) ? urx : llx,
                   (lly > ury) ? ury : lly);
            ur.set((llx > urx) ? llx : urx,
                   (lly > ury) ? lly : ury);
        }
        void set(const frPoint &tmpLowerLeft, const frPoint &tmpUpperRight)
        {
            set(tmpLowerLeft.x(), tmpLowerLeft.y(), tmpUpperRight.x(), tmpUpperRight.y());
        }
        //不安全的设置 - 有可能左边界在右边界的更右边
        void setUnsafe(const frPoint &tmpLowerLeft, const frPoint &tmpUpperRight)
        {
            ll.set(tmpLowerLeft);
            ur.set(tmpUpperRight);
        }
        // getters
        frCoord left() const
        {
            return ll.x();
        }
        frCoord bottom() const
        {
            return ll.y();
        }
        frCoord right() const
        {
            return ur.x();
        }
        frCoord top() const
        {
            return ur.y();
        }
        frPoint &lowerLeft()
        {
            return ll;
        }
        const frPoint &lowerLeft() const
        {
            return ll;
        }
        frPoint &upperRight()
        {
            return ur;
        }
        const frPoint &upperRight() const
        {
            return ur;
        }
        //宽度 - 返回宽和高的最小值
        frCoord width() const
        {
            frCoord xSpan = right() - left();
            frCoord ySpan = top() - bottom();
            return (xSpan > ySpan) ? ySpan : xSpan;
        }
        //高度 - 返回宽和高的最大值
        frCoord length() const
        {
            frCoord xSpan = right() - left();
            frCoord ySpan = top() - bottom();
            return (xSpan > ySpan) ? xSpan : ySpan;
        }
        
        //判断box是否在盒子内
        //后边的用法中都默认incEdges = true - 表示是否包含边界
        bool contains(const frBox &box, bool incEdges = true) const
        {
            ;
            if (incEdges)
            {
                return (box.right() <= ur.x() && box.left() >= ll.x() &&
                        box.top() <= ur.y() && box.bottom() >= ll.y());
            }
            else
            {
                return (box.right() < ur.x() && box.left() > ll.x() &&
                        box.top() < ur.y() && box.bottom() > ll.y());
            }
        }
        bool contains(const frPoint &in, bool incEdges = true) const
        {
            if (incEdges)
            {
                return ll.x() <= in.x() && in.x() <= ur.x() &&
                       ll.y() <= in.y() && in.y() <= ur.y();
            }
            else
            {
                return ll.x() < in.x() && in.x() < ur.x() &&
                       ll.y() < in.y() && in.y() < ur.y();
            }
        }
        //对盒子(区域)进行偏移
        void transform(const frTransform &xform);
        //计算盒子是否重叠
        bool overlaps(const frBox &boxIn, bool incEdges = true) const;
        //扩大盒子
        void bloat(const frCoord distance, frBox &boxOut) const;
        bool operator==(const frBox &boxIn) const
        {
            return (ll == boxIn.ll) && (ur == boxIn.ur);
        }
        bool operator<(const frBox &boxIn) const
        {
            if (!(ll == boxIn.lowerLeft()))
            {
                return (ll < boxIn.lowerLeft());
            }
            else
            {
                return (ur < boxIn.upperRight());
            }
        }

    protected:
        frPoint ll, ur;
    };
}

#endif
