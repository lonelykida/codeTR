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

#ifndef _FR_BASE_TYPES_H_
#define _FR_BASE_TYPES_H_

#include <vector>
#include <list>
#include <map>
#include <string>
#include <utility>

//#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/connected_components.hpp>

#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
//#include <boost/geometry/geometries/point.hpp>
//#include <boost/geometry/geometries/box.hpp>
//#include <boost/geometry/index/rtree.hpp>

//定义一些基本类型及其别名

namespace fr {    //命名空间fr
  using frLayerNum = int; //就是int类型
  using frCoord = int;    //就是int类型
  using frUInt4 = unsigned int; //就是unsigned int类型
  using frDist  = double; //就是double类型
  using frString = std::string; //就是string类型
  using frCost = unsigned int;  //就是unsigned int类型
  using frMIdx = int; // negative value expected - 期望值为负值-就是int类型
  template <typename T> //就是vector<T>
  using frCollection = std::vector<T>;
  template <typename T> //就是vector<T>
  using frVector     = std::vector<T>;
  template <typename T> //就是list<T>
  using frList       = std::list<T>;
  template <typename T> //就是list<T>的迭代器iterator
  using frListIter   = typename std::list<T>::iterator;
  //template <typename T>
  //bool frListIterComp (const frListIter<T>& a, const frListIter<T>& b) {
  //  return &(*a) < &(*b);
  //};
  enum frOrientEnum {   //这是一个方向枚举
      frcR0       = 0, // N
      frcR90      = 1, // W
      frcR180     = 2, // S
      frcR270     = 3, // E
      frcMY       = 4, // FN
      frcMXR90    = 5, // FW
      frcMX       = 6, // FS
      frcMYR90    = 7  // FE
  };
  enum frEndStyleEnum { //这是一个EndStyle类型枚举
      frcTruncateEndStyle = 0, // ext = 0 - 截断
      frcExtendEndStyle   = 1, // ext = half width - 扩展
      frcVariableEndStyle = 2  // ext = variable  - 变量
  };
  enum frPrefRoutingDirEnum {  //这是优先布线方向的枚举
      frcNotApplicablePrefRoutingDir = 0, //不应用优先布线方向
      frcNonePrefRoutingDir          = 1, //无优先布线方向
      frcHorzPrefRoutingDir          = 2, //水平方向优先
      frcVertPrefRoutingDir          = 3  //垂直方向优先
  };
  enum frBlockObjectEnum {    //这是块对象类型枚举，一大堆类型在里边定义
      frcBlockObject   = -1,  //这是一个BlockObject类型
      frcNet           = 0,   //这是一个Net类型
      frcTerm          = 1,   //这是一个Term类型
      frcInst          = 2,   //这是一个Inst类型
      frcFig           = 3,   //这是一个Fig类型
      frcConnFig       = 4,   //这是一个ConnFig类型
      frcPinFig        = 5,   //这是一个PinFig类型
      frcShape         = 6,   //这是一个Shape类型
      frcRef           = 7,   //这是一个Ref类型
      frcVia           = 8,   //这是一个Via类型
      frcPin           = 9,   //这是一个Pin类型
      frcInstTerm      = 10,  //这是一个InstTerm类型
      frcRect          = 11,  //这是一个Rect类型
      frcPolygon       = 12,  //这是一个Polygon类型
      frcSteiner       = 13,  //这是一个Steiner类型
      frcRoute         = 14,  //这是一个Route类型
      frcPathSeg       = 15,  //这是一个PathSeg类型
      frcGuide         = 16,  //这是一个Guide类型
      frcBlockage      = 17,  //这是一个Blockage类型
      frcLayerBlockage = 18,  //这是一个LayerBlockage类型
      frcBlock         = 19,  //这是一个Block类型
      frcBoundary      = 20,  //这是一个Boundary类型
      frcFlexPathSeg   = 21,  //这是一个FlexPathSeg类型
      frcFlexVia       = 22,  //这是一个FlexVia类型
      frcInstBlockage  = 23,  //这是一个InstBlockage类型
      frcAccessPattern = 24,  //这是一个AccessPattern类型
      frcMarker        = 25,  //这是一个Marker类型
      frcPatchWire     = 26,  //这是一个PatchWire类型
      frcRPin,                //这是一个RPin类型
      grcBlockObject,         //这是一个grcBlockObject类型
      grcFig,                 //这是一个grcFig类型
      grcConnFig,             //这是一个grcConnFig类型
      grcPinFig,              //这是一个grcPinFig类型
      grcShape,               //这是一个grcShape类型
      grcNet,
      grcPin,
      grcAccessPattern,
      grcPathSeg,
      grcRef,
      grcVia,
      drcBlockObject,
      drcNet,
      drcPin,
      drcAccessPattern,
      drcPathSeg,
      drcVia,
      drcMazeMarker,
      drcFig,
      drcConnFig,
      drcRef,
      drcPinFig,
      drcPatchWire,
      tacBlockObject,
      tacTrack,
      tacPin,
      tacPathSeg,
      tacVia,
      gccBlockObject,
      gccNet,
      gccPin,
      gccEdge,
      gccRect,
      gccPolygon,
      frcAccessPoint,
      frcAccessPoints,
      frcNode,
      grcNode
  };
  //enum class drBlockObjectEnum {
  //  drcBlockObject = 0,
  //  drcNet,
  //  drcPin,
  //  drcAccessPattern,
  //  drcPathSeg,
  //  drcVia
  //};
  
  /*
    这段代码定义了一个枚举类型frGuideEnum，使用了C++11引入的强类型枚举（enum class）特性。强类型枚举与传统枚举（不带class关键字）相比，
    具有更严格的类型检查和作用域限制，可以避免命名冲突
  */
  enum class frGuideEnum {  //Guide枚举
      frcGuideX,
      frcGuideGlobal,
      frcGuideTrunk,
      frcGuideShortConn
  };
  enum class frTermEnum {   //Term枚举
    frcNormalTerm,  //普通Term
    frcClockTerm,   //时钟Term
    frcPowerTerm,   //电源Term
    frcGroundTerm   //地(Ground)Term
  };
  enum class frNetEnum {    //Net枚举
    frcNormalNet, //普通线网
    frcClockNet,  //时钟线网
    frcPowerNet,  //电源线网
    frcGroundNet  //地(Ground)线网
  };
  enum class frTermDirectionEnum {  //Term方向枚举
    UNKNOWN,    //未知
    INPUT,      //输入
    OUTPUT,     //输出
    INOUT,      //输入输出
    FEEDTHRU,   //feedThru
  };
  enum class frNodeTypeEnum { //NodeType枚举
    frcSteiner,     //斯坦纳点
    frcBoundaryPin, //边界引脚
    frcPin          //普通引脚
  };
  //enum class frLef58CutSpacingTableTypeEnum {
  //  frcCenterSpacing,
  //  frcOrthogonal,
  //  frcOther,
  //};

  enum class frConstraintTypeEnum { // check FlexDR.h fixMode - 约束类型枚举
    frcShortConstraint = 0,   //短路约束
    frcAreaConstraint = 1,    //面积约束
    frcMinWidthConstraint = 2,//最小宽度
    frcSpacingConstraint = 3, //间距
    frcSpacingEndOfLineConstraint = 4,  //EOL spacing约束
    frcSpacingEndOfLineParallelEdgeConstraint = 5, // not supported
    frcSpacingTableConstraint = 6, // not supported
    frcSpacingTablePrlConstraint = 7, 
    frcSpacingTableTwConstraint = 8,
    frcLef58SpacingTableConstraint = 9, // not supported
    frcLef58CutSpacingTableConstraint = 10, // not supported
    frcLef58CutSpacingTablePrlConstraint = 11, // not supported
    frcLef58CutSpacingTableLayerConstraint = 12, // not supported
    frcLef58CutSpacingConstraint = 13, // not supported
    frcLef58CutSpacingParallelWithinConstraint = 14, // not supported
    frcLef58CutSpacingAdjacentCutsConstraint = 15, // not supported
    frcLef58CutSpacingLayerConstraint = 16, // not supported
    frcCutSpacingConstraint = 17,
    frcMinStepConstraint,
    frcLef58MinStepConstraint,
    frcMinimumcutConstraint,
    frcOffGridConstraint,
    frcMinEnclosedAreaConstraint,
    frcLef58CornerSpacingConstraint, // not supported
    frcLef58CornerSpacingConcaveCornerConstraint, // not supported
    frcLef58CornerSpacingConvexCornerConstraint, // not supported
    frcLef58CornerSpacingSpacingConstraint, // not supported
    frcLef58CornerSpacingSpacing1DConstraint, // not supported
    frcLef58CornerSpacingSpacing2DConstraint, // not supported
    frcLef58SpacingEndOfLineConstraint, // not supported
    frcLef58SpacingEndOfLineWithinConstraint, // not supported
    frcLef58SpacingEndOfLineWithinEndToEndConstraint, // not supported
    frcLef58SpacingEndOfLineWithinParallelEdgeConstraint, // not supported
    frcLef58SpacingEndOfLineWithinMaxMinLengthConstraint, // not supported
    frcLef58CutClassConstraint, // not supported
    frcNonSufficientMetalConstraint,
    frcSpacingSamenetConstraint,
    frcLef58RightWayOnGridOnlyConstraint,
    frcLef58RectOnlyConstraint,
    frcRecheckConstraint
  };

  enum class frCornerTypeEnum { //角类型枚举
    UNKNOWN,  //未知
    CONCAVE,  //凹角
    CONVEX    //凸角
  };

  enum class frCornerDirEnum {  //角方向枚举
    UNKNOWN,  //未知
    NE,
    SE,
    SW,
    NW
  };

  enum class frMinimumcutConnectionEnum { //最小cut连接枚举
    UNKNOWN = -1,   //未知
    FROMABOVE = 0,  //从上方
    FROMBELOW = 1   //从下方
  };

  enum class frMinstepTypeEnum {  //最小step类型枚举
    UNKNOWN = -1,
    INSIDECORNER = 0,
    OUTSIDECORNER = 1,
    STEP = 2
  };

  //enum class frDirEnum { UNKNOWN = 0, E = 1, S = 2, W = 3, N = 4, U = 5, D = 6 };
  //enum class frDirEnum { UNKNOWN = 0, E = 4, S = 2, W = 3, N = 1, U = 6, D = 5 };
  #define OPPOSITEDIR 7 // used in FlexGC_main.cpp
  enum class frDirEnum { UNKNOWN = 0, D = 1, S = 2, W = 3, E = 4, N = 5, U = 6 }; //方向枚举-6个方向

  enum class frLayerTypeEnum {  //布线层类型枚举
    CUT,        //cut层
    ROUTING,    //布线层
    IMPLANT,    //implant层
    MASTERSLICE //masterslice层
  };


  enum class AccessPointTypeEnum {  //连接点类型
    Ideal,
    Good,
    Offgrid,
    None
  };

  enum class MacroClassEnum { //宏类型枚举
    UNKNOWN,  //未知
    CORE,     //CORE
    CORE_TIEHIGH,
    CORE_TIELOW,
    CORE_WELLTAP,
    CORE_SPACER,
    CORE_ANTENNACELL,
    COVER, // GF14
    ENDCAP_PRE,
    BLOCK,
    RING, // ispd19
    PAD, // ispd19
    PAD_POWER, // ispd19
    PAD_SPACER, // GF14
    ENDCAP_BOTTOMLEFT // ispd19
  };

  // note: FlexPA hardcoded the cost, don't change here
  enum class frAccessPointEnum {  //连接点类型枚举
    frcOnGridAP   = 0,  //网格点上的AP
    frcHalfGridAP = 1,  //半网格点上的AP
    frcCenterAP   = 2,  //中心点上的AP
    frcEncOptAP   = 3   //ENC优化AP
  };

  //enum class drNetOrderingEnum {
  //  NETDRIVEN,
  //  MARKERDRIVEN
  //};

  //enum class gcPinTypeEnum {
  //  FIXED,
  //  ROUTE,
  //  MERGED
  //};

  //enum frShapeEnum {
  //    frcRect    = 0,
  //    frcPolygon = 1
  //};
  class frBlockObject;  //block对象
  struct vertex_properties_t {  //顶点属性
    frBlockObject* objPtr;  //就只有一个块对象的指针
    //int index;
    //boost::default_color_type color;
    //frString name;
  };
  //class frRoute;
  //一大堆类的声明：
  class frConnFig;
  class frInstTerm;
  class frTerm;
  class frInst;
  class frBlockage;
  struct edge_properties_t {  //边属性
    //std::shared_ptr<frBlockObject> objPtr;
    //std::shared_ptr<frRoute> objPtr;
    std::shared_ptr<frConnFig> objPtr;  //只有一个连接对象的指针
    //frString name;
  };


  ////////////////////////////////////////////boost中的图结构////////////////////////////////////////////
  // boost graph
  typedef boost::adjacency_list< boost::listS, boost::listS, boost::undirectedS, vertex_properties_t, edge_properties_t > graph_t;
  //typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, vertex_properties_t, edge_properties_t > graph_t;
  // descriptor
  typedef boost::graph_traits<graph_t>::vertex_descriptor   vertex_descriptor_t;
  typedef boost::graph_traits<graph_t>::edge_descriptor     edge_descriptor_t;
  // iterator
  typedef boost::graph_traits<graph_t>::vertex_iterator     vertex_iterator_t;
  typedef boost::graph_traits<graph_t>::edge_iterator       edge_iterator_t;
  typedef boost::graph_traits<graph_t>::out_edge_iterator   out_edge_iterator_t;
  typedef boost::graph_traits<graph_t>::adjacency_iterator  adjacency_iterator_t;

  typedef std::map<vertex_descriptor_t, std::size_t>        vertex_descriptor_map_t;  //是一个map，vertex_descriptor_t是key，std::size_t是value
  //typedef boost::property_map<graph_t, &vertex_properties_t::objPtr>::type           tempPM;
  namespace bg  = boost::geometry;  //boost中的geometry
  namespace bgi = boost::geometry::index; //boost中geometry中的index

  //typedef bg::model::point<int, 2, bg::cs::cartesian> boostPoint;
  typedef bg::model::d2::point_xy<frCoord, bg::cs::cartesian> boostPoint;
  typedef bg::model::box<boostPoint> boostBox;
  typedef bg::model::polygon<boostPoint> boostPolygon;
  typedef bg::model::segment<boostPoint> boostSegment;



  //typedef boost::geometry::model::point<frCoord, 2, boost::geometry::cs::cartesian>   point_t;
  typedef bg::model::d2::point_xy<frCoord, bg::cs::cartesian>                         point_t;
  typedef bg::model::box<point_t>                                                     box_t;
  typedef bg::model::segment<point_t>                                                 segment_t;
  class frConnFig;
  typedef std::pair<box_t, std::shared_ptr<frConnFig> >                               rtree_frConnFig_value_t;
  //typedef std::pair<box_t, int* >                                                     rtree_test_t;
  typedef std::pair<box_t, std::shared_ptr<frInst> > rtree_frInst_value_t;
  typedef std::pair<box_t, std::shared_ptr<frTerm> > rtree_frTerm_value_t;
  typedef std::pair<box_t, std::pair<std::shared_ptr<frTerm>, std::shared_ptr<frInstTerm> > > rtree_frTerm_frInstTerm_value_t;
  typedef std::pair<box_t, std::shared_ptr<frBlockage> > rtree_frBlockage_value_t;
  //typedef std::pair<box_t, std::string >                                              rtree_value_t;
  template <typename T>
  using rq_iter_value_t = std::pair<box_t, frListIter<T> >;
  template <typename T>
  using rq_ptr_value_t  = std::pair<box_t, std::shared_ptr<T> >;
  template <typename T>
  using rq_rptr_value_t = std::pair<box_t, T* >;
  template <typename T>
  using rq_generic_value_t = std::pair<box_t, T>;

  // KMB data types
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, vertex_properties_t, boost::property < boost::edge_weight_t, double > > KMBGraph; 
  // typedef std::pair<int, int> KMBEdge;

  // DRC check types
  typedef std::pair<boostPoint, boostPoint> boostEdge;


  // BoostPolygon
  typedef boost::polygon::rectangle_data<int>  Rectangle;
  typedef boost::polygon::polygon_90_data<int> Polygon;
  typedef std::vector<boost::polygon::polygon_90_data<int> > PolygonSet;
  typedef boost::polygon::point_data<int> Point;
  typedef boost::polygon::interval_data<int> Interval;
  typedef boost::polygon::segment_data<int> Segment;
}

#endif
