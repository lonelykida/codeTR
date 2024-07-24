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

#ifndef _FR_FLEXGC_H_
#define _FR_FLEXGC_H_

#include <memory>
#include "frDesign.h"
#include "db/gcObj/gcNet.h"
#include "dr/FlexDR.h"

namespace fr
{
    class FlexGCWorker;
    class FlexGCWorkerRegionQuery
    {
    public:
        FlexGCWorkerRegionQuery(FlexGCWorker *in) : gcWorker(in) {}
        FlexGCWorker *getGCWorker() const
        {
            return gcWorker;
        }
        void addPolygonEdge(gcSegment *edge);
        void addPolygonEdge(gcSegment *edge, std::vector<std::vector<std::pair<segment_t, gcSegment *>>> &allShapes);
        void addMaxRectangle(gcRect *rect);
        void addMaxRectangle(gcRect *rect, std::vector<std::vector<rq_rptr_value_t<gcRect>>> &allShapes);
        void removePolygonEdge(gcSegment *connFig);
        void removeMaxRectangle(gcRect *connFig);
        void queryPolygonEdge(const box_t &box, frLayerNum layerNum, std::vector<std::pair<segment_t, gcSegment *>> &result);
        void queryPolygonEdge(const frBox &box, frLayerNum layerNum, std::vector<std::pair<segment_t, gcSegment *>> &result);
        void queryMaxRectangle(const box_t &box, frLayerNum layerNum, std::vector<rq_rptr_value_t<gcRect>> &result);
        void queryMaxRectangle(const frBox &box, frLayerNum layerNum, std::vector<rq_rptr_value_t<gcRect>> &result);
        void queryMaxRectangle(const gtl::rectangle_data<frCoord> &box, frLayerNum layerNum, std::vector<rq_rptr_value_t<gcRect>> &result);
        void init(int numLayers);
        void addToRegionQuery(gcNet *net);
        void removeFromRegionQuery(gcNet *net);

    protected:
        FlexGCWorker *gcWorker;
        std::vector<bgi::rtree<std::pair<segment_t, gcSegment *>, bgi::quadratic<16>>> polygon_edges; // merged
        std::vector<bgi::rtree<rq_rptr_value_t<gcRect>, bgi::quadratic<16>>> max_rectangles;          // merged
    };

    // class FlexDRWorker;
    class drConnFig;
    class drNet;
    // class drShape;
    // class drPatchWire;

    // 图形约束工作器
    class FlexGCWorker
    {
    public:
        // constructors - 构造器
        FlexGCWorker(frDesign *designIn, FlexDRWorker *drWorkerIn = nullptr) : design(designIn), drWorker(drWorkerIn),
                                                                               extBox(), drcBox(), owner2nets(), nets(), markers(), mapMarkers(), pwires(), rq(this), printMarker(false), modifiedDRNets(),
                                                                               modifiedOwners(), modifiedOwnersSet(),
                                                                               targetNet(nullptr), minLayerNum(std::numeric_limits<frLayerNum>::min()), maxLayerNum(std::numeric_limits<frLayerNum>::max()),
                                                                               targetObj(nullptr), ignoreDB(false), ignoreMinArea(false), surgicalFixEnabled(false) {}
        // setters - 设置器
        // 设置外部box
        void setExtBox(const frBox &in)
        {
            extBox.set(in);
        }
        // 设置DRC检查box
        void setDrcBox(const frBox &in)
        {
            drcBox.set(in);
        }
        // 添加线网
        gcNet *addNet(frBlockObject *owner = nullptr)
        {
            // 创建一个gcNet类型的指针，指针内容是层的数量
            auto uNet = std::make_unique<gcNet>(design->getTech()->getLayers().size());
            //获取gcNet的原始指针
            auto net = uNet.get();
            //将owner赋值给net
            net->setOwner(owner);
            //将新添加的net放入nets中
            nets.push_back(std::move(uNet));
            owner2nets[owner] = net;    //从owner到net的映射
            return net;
        }
        // 添加标记
        bool addMarker(std::unique_ptr<frMarker> &in)
        {
            //边界框bbox
            frBox bbox;
            in->getBBox(bbox);  //为标记添加边界框
            auto layerNum = in->getLayerNum();  //层号
            auto con = in->getConstraint();     //获取约束
            std::vector<frBlockObject *> srcs(2, nullptr);  //源net数组，两个位置的vector
            int i = 0;
            //将每个要标记的源net放入srcs中
            for (auto &src : in->getSrcs())
            {
                srcs.at(i) = src;
                i++;
            }
            //检查标记是否重复
            if (mapMarkers.find(std::make_tuple(bbox, layerNum, con, srcs[0], srcs[1])) != mapMarkers.end())
            {
                return false;
            }
            if (mapMarkers.find(std::make_tuple(bbox, layerNum, con, srcs[1], srcs[0])) != mapMarkers.end())
            {
                return false;
            }
            //标记不重复则添加标记到标记映射图中
            mapMarkers[std::make_tuple(bbox, layerNum, con, srcs[0], srcs[1])] = in.get();
            //标记不重复则添加标记到标记列表中
            markers.push_back(std::move(in));
            return true;
        }
        // 清除标记
        void clearMarkers()
        {
            mapMarkers.clear(); 
            markers.clear();    
        }
        //设置目标net
        bool setTargetNet(frBlockObject *in)
        {
            //owner就是一个线网对象(BlockObject),映射到frNet指针
            //若找到目标net则添加失败，否则添加成功
            if (owner2nets.find(in) != owner2nets.end())
            {
                targetNet = owner2nets[in];
                return true;
            }
            else
            {
                return false;
            }
        }
        //获取目标net
        gcNet *getTargetNet(frBlockObject *in)
        {
            if (owner2nets.find(in) != owner2nets.end())
            {
                return owner2nets[in];
            }
            else
            {
                return nullptr;
            }
        }
        //重置目标net为nullptr
        void resetTargetNet()
        {
            targetNet = nullptr;
        }
        //设置目标对象
        void setTargetObj(frBlockObject *in)
        {
            targetObj = in;
        }
        //设置是否忽略DB为true
        void setIgnoreDB()
        {
            ignoreDB = true;
        }
        //设置是否忽略MinArea为true
        void setIgnoreMinArea()
        {
            ignoreMinArea = true;
        }
        //设置是否开启surgical fix
        void setEnableSurgicalFix(bool in)
        {
            surgicalFixEnabled = in;
        }
        void addPAObj(frConnFig *obj, frBlockObject *owner);
        // getters - 获取器
        //获取design
        frDesign *getDesign() const
        {
            return design;
        }
        //获取drWorker
        FlexDRWorker *getDRWorker() const
        {
            return drWorker;
        }
        //获取非const外部边界框
        void getExtBox(frBox &in) const
        {
            in.set(extBox);
        }
        //获取const外部边界框
        const frBox &getExtBox() const
        {
            return extBox;
        }
        //获取非const drc边界框
        void getDrcBox(frBox &in) const
        {
            in.set(drcBox);
        }
        //获取const drc边界框
        const frBox &getDrcBox() const
        {
            return drcBox;
        }
        //获取标记列表
        const std::vector<std::unique_ptr<frMarker>> &getMarkers() const
        {
            return markers;
        }
        //获取patch wires列表
        const std::vector<std::unique_ptr<drPatchWire>> &getPWires() const
        {
            return pwires;
        }
        //获取区域查询器
        frRegionQuery *getRegionQuery() const
        {
            return design->getRegionQuery();
        }
        //获取GCWorker的区域查询器
        FlexGCWorkerRegionQuery &getWorkerRegionQuery()
        {
            return rq;
        }
        //获取net列表
        std::vector<std::unique_ptr<gcNet>> &getNets()
        {
            return nets;
        }
        // others
        void init();
        int main();
        void end();
        int test();
        // initialization from FlexPA, initPA0 --> addPAObj --> initPA1
        void initPA0();
        void initPA1();
        void resetPAOwner(frBlockObject *owner);
        void initPA2();
        void updateDRNet(drNet *net);

    protected:
        frDesign *design;
        FlexDRWorker *drWorker;

        frBox extBox; // 外部框
        frBox drcBox; // drc框

        //从net到gcNet的映射
        std::map<frBlockObject *, gcNet *> owner2nets; // no order is assumed
        std::vector<std::unique_ptr<gcNet>> nets;

        std::vector<std::unique_ptr<frMarker>> markers;
        std::map<std::tuple<frBox, frLayerNum, frConstraint *, frBlockObject *, frBlockObject *>, frMarker *> mapMarkers;
        std::vector<std::unique_ptr<drPatchWire>> pwires;

        FlexGCWorkerRegionQuery rq;
        bool printMarker;

        // temps
        std::vector<drNet *> modifiedDRNets;
        std::vector<frBlockObject *> modifiedOwners;
        std::set<frBlockObject *> modifiedOwnersSet;

        // parameters
        gcNet *targetNet;
        frLayerNum minLayerNum;
        frLayerNum maxLayerNum;

        // for pin prep
        frBlockObject *targetObj;
        bool ignoreDB;
        bool ignoreMinArea;
        bool surgicalFixEnabled;

        // init
        gcNet *getNet(frBlockObject *obj);
        void initObj(const frBox &box, frLayerNum layerNum, frBlockObject *obj, bool isFixed);
        void initDRObj(drConnFig *obj, gcNet *currNet = nullptr);
        void initDesign();
        bool initDesign_skipObj(frBlockObject *obj);
        void initDRWorker();
        void initNets();
        void initNet(gcNet *net);
        void initNet_pins_polygon(gcNet *net);
        void initNet_pins_polygonEdges(gcNet *net);
        void initNet_pins_polygonEdges_getFixedPolygonEdges(gcNet *net, std::vector<std::set<std::pair<frPoint, frPoint>>> &fixedPolygonEdges);
        void initNet_pins_polygonEdges_helper_outer(gcNet *net, gcPin *pin, gcPolygon *poly, frLayerNum i,
                                                    const std::vector<std::set<std::pair<frPoint, frPoint>>> &fixedPolygonEdges);
        void initNet_pins_polygonEdges_helper_inner(gcNet *net, gcPin *pin, const gtl::polygon_90_data<frCoord> &hole_poly, frLayerNum i,
                                                    const std::vector<std::set<std::pair<frPoint, frPoint>>> &fixedPolygonEdges);
        void initNet_pins_polygonCorners(gcNet *net);
        void initNet_pins_polygonCorners_helper(gcNet *net, gcPin *pin);
        void initNet_pins_maxRectangles(gcNet *net);
        void initNet_pins_maxRectangles_getFixedMaxRectangles(gcNet *net, std::vector<std::set<std::pair<frPoint, frPoint>>> &fixedMaxRectangles);
        void initNet_pins_maxRectangles_helper(gcNet *net, gcPin *pin, const gtl::rectangle_data<frCoord> &rect, frLayerNum i,
                                               const std::vector<std::set<std::pair<frPoint, frPoint>>> &fixedMaxRectangles);

        void initRegionQuery();

        // update
        void updateGCWorker();

        void checkMetalSpacing();
        frCoord checkMetalSpacing_getMaxSpcVal(frLayerNum layerNum);
        void myBloat(const gtl::rectangle_data<frCoord> &rect, frCoord val, box_t &box);
        void checkMetalSpacing_main(gcRect *rect);
        void checkMetalSpacing_main(gcRect *rect1, gcRect *rect2);
        void checkMetalSpacing_short(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect);
        bool checkMetalSpacing_short_skipOBSPin(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect);
        frCoord checkMetalSpacing_prl_getReqSpcVal(gcRect *rect1, gcRect *rect2, frCoord prl);
        bool checkMetalSpacing_prl_hasPolyEdge(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect, int type, frCoord prl);
        void checkMetalSpacing_prl(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect, frCoord prl, frCoord distX, frCoord distY);
        box_t checkMetalCornerSpacing_getQueryBox(gcCorner *corner, frCoord &maxSpcValX, frCoord &maxSpcValY);
        void checkMetalCornerSpacing();
        void checkMetalCornerSpacing_getMaxSpcVal(frLayerNum layerNum, frCoord &maxSpcValX, frCoord &maxSpcValY);

        void checkMetalCornerSpacing_main(gcCorner *corner);
        void checkMetalCornerSpacing_main(gcCorner *corner, gcRect *rect, frLef58CornerSpacingConstraint *con);
        void checkMetalCornerSpacing_main(gcCorner *corner, gcSegment *seg, frLef58CornerSpacingConstraint *con);

        void checkMetalShape();
        void checkMetalShape_main(gcPin *pin);
        void checkMetalShape_minWidth(const gtl::rectangle_data<frCoord> &rect, frLayerNum layerNum, gcNet *net, bool isH);
        void checkMetalShape_offGrid(gcPin *pin);
        void checkMetalShape_minEnclosedArea(gcPin *pin);
        void checkMetalShape_minStep(gcPin *pin);
        void checkMetalShape_minStep_helper(const frBox &markerBox, frLayerNum layerNum, gcNet *net, frMinStepConstraint *con,
                                            bool hasInsideCorner, bool hasOutsideCorner, bool hasStep,
                                            int currEdges, frCoord currLength, bool hasRoute);
        void checkMetalShape_rectOnly(gcPin *pin);
        void checkMetalShape_minArea(gcPin *pin);

        void checkMetalEndOfLine();
        void checkMetalEndOfLine_main(gcPin *pin);
        void checkMetalEndOfLine_eol(gcSegment *edge, frSpacingEndOfLineConstraint *con);
        bool checkMetalEndOfLine_eol_isEolEdge(gcSegment *edge, frSpacingEndOfLineConstraint *con);
        bool checkMetalEndOfLine_eol_hasParallelEdge(gcSegment *edge, frSpacingEndOfLineConstraint *con, bool &hasRoute);
        bool checkMetalEndOfLine_eol_hasParallelEdge_oneDir(gcSegment *edge, frSpacingEndOfLineConstraint *con, bool isSegLow, bool &hasRoute);
        void checkMetalEndOfLine_eol_hasParallelEdge_oneDir_getQueryBox(gcSegment *edge, frSpacingEndOfLineConstraint *con,
                                                                        bool isSegLow, box_t &queryBox, gtl::rectangle_data<frCoord> &queryRect);
        void checkMetalEndOfLine_eol_hasParallelEdge_oneDir_getParallelEdgeRect(gcSegment *edge, gtl::rectangle_data<frCoord> &rect);
        void checkMetalEndOfLine_eol_hasEol(gcSegment *edge, frSpacingEndOfLineConstraint *con, bool hasRoute);
        void checkMetalEndOfLine_eol_hasEol_getQueryBox(gcSegment *edge, frSpacingEndOfLineConstraint *con,
                                                        box_t &queryBox, gtl::rectangle_data<frCoord> &queryRect);
        void checkMetalEndOfLine_eol_hasEol_helper(gcSegment *edge1, gcSegment *edge2, frSpacingEndOfLineConstraint *con);

        void checkCutSpacing();
        void checkCutSpacing_main(gcRect *rect);
        void checkCutSpacing_main(gcRect *rect, frCutSpacingConstraint *con);
        bool checkCutSpacing_main_hasAdjCuts(gcRect *rect, frCutSpacingConstraint *con);
        frCoord checkCutSpacing_getMaxSpcVal(frCutSpacingConstraint *con);
        void checkCutSpacing_main(gcRect *ptr1, gcRect *ptr2, frCutSpacingConstraint *con);
        void checkCutSpacing_short(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect);
        void checkCutSpacing_spc(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect, frCutSpacingConstraint *con, frCoord prl);
        void checkCutSpacing_spc_diff_layer(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect, frCutSpacingConstraint *con);
        frCoord checkCutSpacing_spc_getReqSpcVal(gcRect *ptr1, gcRect *ptr2, frCutSpacingConstraint *con);

        // LEF58
        void checkLef58CutSpacing_main(gcRect *rect);
        void checkLef58CutSpacing_main(gcRect *rect, frLef58CutSpacingConstraint *con, bool skipSameNet = false);
        void checkLef58CutSpacing_main(gcRect *rect1, gcRect *rect2, frLef58CutSpacingConstraint *con);
        void checkLef58CutSpacing_spc_layer(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect, frLef58CutSpacingConstraint *con);
        void checkLef58CutSpacing_spc_adjCut(gcRect *rect1, gcRect *rect2, const gtl::rectangle_data<frCoord> &markerRect, frLef58CutSpacingConstraint *con);
        bool checkLef58CutSpacing_spc_hasAdjCuts(gcRect *rect, frLef58CutSpacingConstraint *con);
        bool checkLef58CutSpacing_spc_hasTwoCuts(gcRect *rect1, gcRect *rect2, frLef58CutSpacingConstraint *con);
        bool checkLef58CutSpacing_spc_hasTwoCuts_helper(gcRect *rect, frLef58CutSpacingConstraint *con);
        frCoord checkLef58CutSpacing_spc_getReqSpcVal(gcRect *ptr1, gcRect *ptr2, frLef58CutSpacingConstraint *con);
        frCoord checkLef58CutSpacing_getMaxSpcVal(frLef58CutSpacingConstraint *con);
        void checkMetalShape_lef58MinStep(gcPin *pin);
        void checkMetalShape_lef58MinStep_noBetweenEol(gcPin *pin, frLef58MinStepConstraint *con);

        // surgical fix
        void patchMetalShape();
        void patchMetalShape_helper();

        // utility
        bool isCornerOverlap(gcCorner *corner, const box_t &box);
        bool isCornerOverlap(gcCorner *corner, const gtl::rectangle_data<frCoord> &rect);
        bool isOppositeDir(gcCorner *corner, gcSegment *seg);
    };
}

#endif
