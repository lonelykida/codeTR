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

#ifndef _FR_MARKER_H_
#define _FR_MARKER_H_

#include "db/obj/frFig.h"
// #include "db/tech/frConstraint.h"
#include <tuple>

namespace fr
{
    class frConstraint;
    // 标记类，继承自frFig，marker用于重布线标记
    class frMarker : public frFig
    {
    public:
        // constructors - 构造器
        // 初始化constraint为NULL,bbox类,frFig类,layerNum为0
        // 标记指针的列表iter，srcs()是block对象的
        frMarker() : frFig(), constraint(nullptr), bbox(), layerNum(0), srcs(), iter(), vioHasDir(false), vioIsH(false) {}
        frMarker(const frMarker &in) : constraint(in.constraint), bbox(in.bbox), layerNum(in.layerNum),
                                       srcs(in.srcs), iter(), vioHasDir(in.vioHasDir), vioIsH(in.vioIsH) {}
        // setters - 设置器
        // 设置约束
        void setConstraint(frConstraint *constraintIn)
        {
            constraint = constraintIn;
        }

        // 设置边界
        void setBBox(const frBox &bboxIn)
        {
            bbox = bboxIn;
        }
        // 设置层号
        void setLayerNum(const frLayerNum &layerNumIn)
        {
            layerNum = layerNumIn;
        }
        // 设置是否有方向
        void setHasDir(const bool &in)
        {
            vioHasDir = in;
        }
        // 设置是否水平
        void setIsH(const bool &in)
        {
            vioIsH = in;
        }

        // void addNet(frNet* netIn) {
        //   srcs.push_back(netIn);
        // }
        // void addInstTerm(frInstTerm* instTermIn) {
        //   srcs.push_back(instTermIn);
        // }
        // void addTerm(frTerm* termIn) {
        //   srcs.push_back(termIn);
        // }

        // 添加src
        void addSrc(frBlockObject *srcIn)
        {
            // srcs.push_back(srcIn);
            srcs.insert(srcIn);
        }
        // 添加aggressor - 攻击者，是一个三元组<攻击/受害者,是否需要重布,布线次数>
        // 此时添加的攻击者对象，包含层号、边界、是否需要重布
        void addAggressor(frBlockObject *obj, const std::tuple<frLayerNum, frBox, bool> &tupleIn)
        {
            aggressors.push_back(std::make_pair(obj, tupleIn));
        }
        // 添加victims - 受害者，是一个三元组<攻击/受害者,是否需要重布,布线次数>
        // 此时添加的受害者对象，包含层号、边界、是否需要重布
        void addVictim(frBlockObject *obj, const std::tuple<frLayerNum, frBox, bool> &tupleIn)
        {
            victims.push_back(std::make_pair(obj, tupleIn));
        }
        // void addSrcId(int netId) {
        //   srcIds.insert(netId);
        // }
        // getters

        /* from frFig
         * getBBox
         * move, in .cpp
         * overlaps in .cpp
         */
        // 获取边界框
        void getBBox(frBox &bboxIn) const override
        {
            bboxIn.set(bbox);
        }
        // 获取层号
        frLayerNum getLayerNum() const
        {
            return layerNum;
        }

        // frVector<frNet*> getNets() const {
        //   return nets;
        // }

        // const frVector<frBlockObject*>& getSrcs() const {
        //   return srcs;
        // }
        // frVector<frBlockObject*>& getSrcs() {
        //   return srcs;
        // }

        // 获取const srcs
        const std::set<frBlockObject *> &getSrcs() const
        {
            return srcs;
        }
        // 获取非const srcs
        std::set<frBlockObject *> &getSrcs()
        {
            return srcs;
        }

        // const std::set<int>& getSrcIds() const {
        //   return srcIds;
        // }
        // std::set<int>& getSrcIds() {
        //   return srcIds;
        // }

        // 获取const攻击者队列<对象,<层号,边界,是否需要重布>>
        const std::vector<std::pair<frBlockObject *, std::tuple<frLayerNum, frBox, bool>>> &getAggressors() const
        {
            return aggressors;
        }

        // 获取非const攻击者队列<对象,<层号,边界,是否需要重布>>
        std::vector<std::pair<frBlockObject *, std::tuple<frLayerNum, frBox, bool>>> &getAggressors()
        {
            return aggressors;
        }

        // 获取const受害者队列<对象,<层号,边界,是否需要重布>>
        const std::vector<std::pair<frBlockObject *, std::tuple<frLayerNum, frBox, bool>>> &getVictims() const
        {
            return victims;
        }

        // 获取非const受害者队列<对象,<层号,边界,是否需要重布>>
        std::vector<std::pair<frBlockObject *, std::tuple<frLayerNum, frBox, bool>>> &getVictims()
        {
            return victims;
        }
        //获取约束
        frConstraint *getConstraint() const
        {
            return constraint;
        }
        // 获取是否有方向
        bool hasDir() const
        {
            return vioHasDir;
        }
        // 获取是否为横向
        bool isH() const
        {
            return vioIsH;
        }
        //重写move虚函数，空函数
        void move(const frTransform &xform) override
        {
        }
        //重写overlaps虚函数，不重写，直接返回false
        bool overlaps(const frBox &box) const override
        {
            return false;
        }

        // others
        //返回当前的类型编号 - 25 - 表示是一个标记
        frBlockObjectEnum typeId() const override
        {
            return frcMarker;
        }
        //设置标记指针列表
        void setIter(frListIter<std::unique_ptr<frMarker>> &in)
        {
            iter = in;
        }
        //获取标记指针列表
        frListIter<std::unique_ptr<frMarker>> getIter() const
        {
            return iter;
        }

    protected:
        frConstraint *constraint;   //约束类的指针
        frBox bbox; //边界框类变量
        frLayerNum layerNum;    //层号 - int
        // frVector<frBlockObject*> srcs; // either frNet or instTerm or term
        std::set<frBlockObject *> srcs; //block资源set
        std::vector<std::pair<frBlockObject *, std::tuple<frLayerNum, frBox, bool>>> victims;    // obj, isFixed
        std::vector<std::pair<frBlockObject *, std::tuple<frLayerNum, frBox, bool>>> aggressors; // obj, isFixed
        // std::set<int> srcIds;
        frListIter<std::unique_ptr<frMarker>> iter;
        bool vioHasDir, vioIsH; //两个方向的成员变量
    };
}

#endif
