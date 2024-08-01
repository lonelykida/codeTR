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

#include "dr/FlexDR.h"
// #include "drc/frDRC.h"
#include "gc/FlexGC.h"
#include <chrono>
#include <algorithm>
#include <random>

using namespace std;
using namespace fr;

inline frCoord FlexDRWorker::pt2boxDistSquare(const frPoint &pt, const frBox &box)
{
    frCoord dx = max(max(box.left() - pt.x(), pt.x() - box.right()), 0);
    frCoord dy = max(max(box.bottom() - pt.y(), pt.y() - box.top()), 0);
    return dx * dx + dy * dy;
}

inline frCoord FlexDRWorker::box2boxDistSquare(const frBox &box1, const frBox &box2, frCoord &dx, frCoord &dy)
{
    dx = max(max(box1.left(), box2.left()) - min(box1.right(), box2.right()), 0);
    dy = max(max(box1.bottom(), box2.bottom()) - min(box1.top(), box2.top()), 0);
    return dx * dx + dy * dy;
}

// prlx = -dx, prly = -dy
// dx > 0 : disjoint in x; dx = 0 : touching in x; dx < 0 : overlap in x
inline frCoord FlexDRWorker::box2boxDistSquareNew(const frBox &box1, const frBox &box2, frCoord &dx, frCoord &dy)
{
    dx = max(box1.left(), box2.left()) - min(box1.right(), box2.right());
    dy = max(box1.bottom(), box2.bottom()) - min(box1.top(), box2.top());
    return max(dx, 0) * max(dx, 0) + max(dy, 0) * max(dy, 0);
}

void FlexDRWorker::modViaForbiddenThrough(const FlexMazeIdx &bi, const FlexMazeIdx &ei, int type)
{
    // auto lNum = gridGraph.getLayerNum(bi.z());
    bool isHorz = (bi.y() == ei.y());

    bool isLowerViaForbidden = getTech()->isViaForbiddenThrough(bi.z(), true, isHorz);
    bool isUpperViaForbidden = getTech()->isViaForbiddenThrough(bi.z(), false, isHorz);

    if (isHorz)
    {
        for (int xIdx = bi.x(); xIdx < ei.x(); xIdx++)
        {
            if (isLowerViaForbidden)
            {
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostVia(xIdx, bi.y(), bi.z() - 1); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostVia(xIdx, bi.y(), bi.z() - 1); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostVia(xIdx, bi.y(), bi.z() - 1); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostVia(xIdx, bi.y(), bi.z() - 1); // safe access
                    break;
                default:;
                }
            }

            if (isUpperViaForbidden)
            {
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostVia(xIdx, bi.y(), bi.z()); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostVia(xIdx, bi.y(), bi.z()); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostVia(xIdx, bi.y(), bi.z()); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostVia(xIdx, bi.y(), bi.z()); // safe access
                    break;
                default:;
                }
            }
        }
    }
    else
    {
        for (int yIdx = bi.y(); yIdx < ei.y(); yIdx++)
        {
            if (isLowerViaForbidden)
            {
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostVia(bi.x(), yIdx, bi.z() - 1); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostVia(bi.x(), yIdx, bi.z() - 1); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostVia(bi.x(), yIdx, bi.z() - 1); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostVia(bi.x(), yIdx, bi.z() - 1); // safe access
                    break;
                default:;
                }
            }

            if (isUpperViaForbidden)
            {
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostVia(bi.x(), yIdx, bi.z()); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostVia(bi.x(), yIdx, bi.z()); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostVia(bi.x(), yIdx, bi.z()); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostVia(bi.x(), yIdx, bi.z()); // safe access
                    break;
                default:;
                }
            }
        }
    }
}

void FlexDRWorker::modBlockedPlanar(const frBox &box, frMIdx z, bool setBlock)
{
    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    gridGraph.getIdxBox(mIdx1, mIdx2, box);
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            if (setBlock)
            {
                gridGraph.setBlocked(i, j, z, frDirEnum::E);
                gridGraph.setBlocked(i, j, z, frDirEnum::N);
                gridGraph.setBlocked(i, j, z, frDirEnum::W);
                gridGraph.setBlocked(i, j, z, frDirEnum::S);
            }
            else
            {
                gridGraph.resetBlocked(i, j, z, frDirEnum::E);
                gridGraph.resetBlocked(i, j, z, frDirEnum::N);
                gridGraph.resetBlocked(i, j, z, frDirEnum::W);
                gridGraph.resetBlocked(i, j, z, frDirEnum::S);
            }
        }
    }
}

void FlexDRWorker::modBlockedVia(const frBox &box, frMIdx z, bool setBlock)
{
    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    gridGraph.getIdxBox(mIdx1, mIdx2, box);
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            if (setBlock)
            {
                gridGraph.setBlocked(i, j, z, frDirEnum::U);
                gridGraph.setBlocked(i, j, z, frDirEnum::D);
            }
            else
            {
                gridGraph.resetBlocked(i, j, z, frDirEnum::U);
                gridGraph.resetBlocked(i, j, z, frDirEnum::D);
            }
        }
    }
}

/*inline*/ void FlexDRWorker::modMinSpacingCostPlaner(const frBox &box, frMIdx z, int type, bool isBlockage)
{
    auto lNum = gridGraph.getLayerNum(z);
    // obj1 = curr obj
    frCoord width1 = box.width();
    frCoord length1 = box.length();
    // obj2 = other obj
    // layer default width
    frCoord width2 = getDesign()->getTech()->getLayer(lNum)->getWidth();
    frCoord halfwidth2 = width2 / 2;
    // spacing value needed
    frCoord bloatDist = 0;
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
    if (con)
    {
        if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
        {
            bloatDist = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
        }
        else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
        {
            bloatDist = (isBlockage && USEMINSPACING_OBS) ? static_cast<frSpacingTablePrlConstraint *>(con)->findMin() : static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2), length1);
        }
        else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
        {
            bloatDist = (isBlockage && USEMINSPACING_OBS) ? static_cast<frSpacingTableTwConstraint *>(con)->findMin() : static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2, length1);
        }
        else
        {
            cout << "Warning: min spacing rule not supporterd" << endl;
            return;
        }
    }
    else
    {
        cout << "Warning: no min spacing rule" << endl;
        return;
    }
    // if (isBlockage && (!USEMINSPACING_OBS)) {
    //   bloatDist = 0;
    // }
    frCoord bloatDistSquare = bloatDist * bloatDist;

    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    // assumes width always > 2
    frBox bx(box.left() - bloatDist - halfwidth2 + 1, box.bottom() - bloatDist - halfwidth2 + 1,
             box.right() + bloatDist + halfwidth2 - 1, box.top() + bloatDist + halfwidth2 - 1);
    gridGraph.getIdxBox(mIdx1, mIdx2, bx);
    // if (!isInitDR()) {
    //   cout <<" box " <<box <<" bloatDist " <<bloatDist <<" bx " <<bx <<endl;
    //   cout <<" midx1/2 (" <<mIdx1.x() <<", " <<mIdx1.y() <<") ("
    //                       <<mIdx2.x() <<", " <<mIdx2.y() <<") (" <<endl;
    // }

    frPoint pt, pt1, pt2, pt3, pt4;
    frCoord distSquare = 0;
    int cnt = 0;
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            gridGraph.getPoint(pt, i, j);
            pt1.set(pt.x() + halfwidth2, pt.y() - halfwidth2);
            pt2.set(pt.x() + halfwidth2, pt.y() + halfwidth2);
            pt3.set(pt.x() - halfwidth2, pt.y() - halfwidth2);
            pt4.set(pt.x() - halfwidth2, pt.y() + halfwidth2);
            distSquare = min(pt2boxDistSquare(pt1, box), pt2boxDistSquare(pt2, box));
            distSquare = min(pt2boxDistSquare(pt3, box), distSquare);
            distSquare = min(pt2boxDistSquare(pt4, box), distSquare);
            if (distSquare < bloatDistSquare)
            {
                // if (isAddPathCost) {
                //   gridGraph.addDRCCostPlanar(i, j, z); // safe access
                // } else {
                //   gridGraph.subDRCCostPlanar(i, j, z); // safe access
                // }
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostPlanar(i, j, z); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostPlanar(i, j, z); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostPlanar(i, j, z); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostPlanar(i, j, z); // safe access
                    break;
                default:;
                }
                if (QUICKDRCTEST)
                {
                    cout << "    (" << i << ", " << j << ", " << z << ") minSpc planer" << endl;
                }
                cnt++;
                // if (!isInitDR()) {
                //   cout <<" planer find viol mIdx (" <<i <<", " <<j <<") " <<pt <<endl;
                // }
            }
        }
    }
    // cout <<"planer mod " <<cnt <<" edges" <<endl;
}

/*inline*/ void FlexDRWorker::modMinSpacingCost(drNet *net, const frBox &box, frMIdx z, int type, bool isCurrPs)
{
    auto lNum = gridGraph.getLayerNum(z);
    // obj1 = curr obj
    frCoord width1 = box.width();
    frCoord length1 = box.length();
    // obj2 planar = other obj
    // layer default width
    frCoord width2planar = getDesign()->getTech()->getLayer(lNum)->getWidth();
    frCoord halfwidth2planar = width2planar / 2;
    // obj2 viaL = other obj
    frViaDef *viaDefL = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef() : nullptr;
    frVia viaL(viaDefL);
    frBox viaBoxL(0, 0, 0, 0);
    if (viaDefL)
    {
        viaL.getLayer2BBox(viaBoxL);
    }
    frCoord width2viaL = viaBoxL.width();
    frCoord length2viaL = viaBoxL.length();
    // obj2 viaU = other obj
    frViaDef *viaDefU = (lNum < getDesign()->getTech()->getTopLayerNum()) ? getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef() : nullptr;
    frVia viaU(viaDefU);
    frBox viaBoxU(0, 0, 0, 0);
    if (viaDefU)
    {
        viaU.getLayer1BBox(viaBoxU);
    }
    frCoord width2viaU = viaBoxU.width();
    frCoord length2viaU = viaBoxU.length();

    // spacing value needed
    frCoord bloatDistPlanar = 0;
    frCoord bloatDistViaL = 0;
    frCoord bloatDistViaU = 0;
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
    if (con)
    {
        if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
        {
            bloatDistPlanar = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
            bloatDistViaL = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
            bloatDistViaU = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
        }
        else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
        {
            bloatDistPlanar = static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2planar), length1);
            bloatDistViaL = static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2viaL), isCurrPs ? length2viaL : min(length1, length2viaL));
            bloatDistViaU = static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2viaU), isCurrPs ? length2viaU : min(length1, length2viaU));
        }
        else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
        {
            bloatDistPlanar = static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2planar, length1);
            bloatDistViaL = static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2viaL, isCurrPs ? length2viaL : min(length1, length2viaL));
            bloatDistViaU = static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2viaU, isCurrPs ? length2viaU : min(length1, length2viaU));
        }
        else
        {
            cout << "Warning: min spacing rule not supporterd" << endl;
            return;
        }
    }
    else
    {
        cout << "Warning: no min spacing rule" << endl;
        return;
    }

    // other obj eol spc to curr obj
    // no need to bloat eolWithin because eolWithin always < minSpacing
    frCoord bloatDistEolX = 0;
    frCoord bloatDistEolY = 0;
    for (auto con : getDesign()->getTech()->getLayer(lNum)->getEolSpacing())
    {
        auto eolSpace = con->getMinSpacing();
        auto eolWidth = con->getEolWidth();
        // eol up and down
        if (viaDefL && viaBoxL.right() - viaBoxL.left() < eolWidth)
        {
            bloatDistEolY = max(bloatDistEolY, eolSpace);
        }
        if (viaDefU && viaBoxU.right() - viaBoxU.left() < eolWidth)
        {
            bloatDistEolY = max(bloatDistEolY, eolSpace);
        }
        // eol left and right
        if (viaDefL && viaBoxL.top() - viaBoxL.bottom() < eolWidth)
        {
            bloatDistEolX = max(bloatDistEolX, eolSpace);
        }
        if (viaDefU && viaBoxU.top() - viaBoxU.bottom() < eolWidth)
        {
            bloatDistEolX = max(bloatDistEolX, eolSpace);
        }
    }

    frCoord bloatDist = max(max(bloatDistPlanar, bloatDistViaL), bloatDistViaU);
    // frCoord bloatDistSquare = bloatDist * bloatDist;

    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    // assumes width always > 2
    frBox bx(box.left() - max(bloatDist, bloatDistEolX) - max(max(halfwidth2planar, viaBoxL.right()), viaBoxU.right()) + 1,
             box.bottom() - max(bloatDist, bloatDistEolY) - max(max(halfwidth2planar, viaBoxL.top()), viaBoxU.top()) + 1,
             box.right() + max(bloatDist, bloatDistEolX) + max(max(halfwidth2planar, viaBoxL.left()), viaBoxU.left()) - 1,
             box.top() + max(bloatDist, bloatDistEolY) + max(max(halfwidth2planar, viaBoxL.bottom()), viaBoxU.bottom()) - 1);
    gridGraph.getIdxBox(mIdx1, mIdx2, bx);
    // if (!isInitDR()) {
    //   cout <<" box " <<box <<" bloatDist " <<bloatDist <<" bx " <<bx <<endl;
    //   cout <<" midx1/2 (" <<mIdx1.x() <<", " <<mIdx1.y() <<") ("
    //                       <<mIdx2.x() <<", " <<mIdx2.y() <<") (" <<endl;
    // }

    frPoint pt;
    frBox tmpBx;
    frCoord dx, dy, prl;
    frTransform xform;
    frCoord reqDist = 0;
    frCoord distSquare = 0;
    int cnt = 0;
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            gridGraph.getPoint(pt, i, j);
            xform.set(pt);
            // planar
            tmpBx.set(pt.x() - halfwidth2planar, pt.y() - halfwidth2planar,
                      pt.x() + halfwidth2planar, pt.y() + halfwidth2planar);
            distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
            prl = max(dx, dy);
            if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
            {
                reqDist = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
            }
            else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
            {
                reqDist = static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2planar), prl > 0 ? length1 : 0);
            }
            else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
            {
                reqDist = static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2planar, prl > 0 ? length1 : 0);
            }
            if (distSquare < reqDist * reqDist)
            {
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostPlanar(i, j, z); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostPlanar(i, j, z); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostPlanar(i, j, z);
                    break;
                case 3:
                    gridGraph.addShapeCostPlanar(i, j, z);
                    break;
                default:;
                }
                // if (isAddPathCost) {
                //   gridGraph.addDRCCostPlanar(i, j, z); // safe access
                // } else {
                //   gridGraph.subDRCCostPlanar(i, j, z); // safe access
                // }
                if (QUICKDRCTEST)
                {
                    cout << "    (" << i << ", " << j << ", " << z << ") minSpc planer" << endl;
                }
                cnt++;
            }
            // viaL
            if (viaDefL)
            {
                tmpBx.set(viaBoxL);
                tmpBx.transform(xform);
                distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
                prl = max(dx, dy);
                // curr is ps
                if (isCurrPs)
                {
                    if (dx == 0 && dy > 0)
                    {
                        prl = viaBoxL.right() - viaBoxL.left();
                    }
                    else if (dx > 0 && dy == 0)
                    {
                        prl = viaBoxL.top() - viaBoxL.bottom();
                    }
                }
                if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
                {
                    reqDist = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
                }
                else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
                {
                    reqDist = static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2viaL), prl);
                }
                else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
                {
                    reqDist = static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2viaL, prl);
                }
                if (distSquare < reqDist * reqDist)
                {
                    switch (type)
                    {
                    case 0:
                        gridGraph.subDRCCostVia(i, j, z - 1);
                        break;
                    case 1:
                        gridGraph.addDRCCostVia(i, j, z - 1);
                        break;
                    case 2:
                        gridGraph.subShapeCostVia(i, j, z - 1);
                        break;
                    case 3:
                        gridGraph.addShapeCostVia(i, j, z - 1);
                        break;
                    default:;
                    }
                    // if (isAddPathCost) {
                    //   gridGraph.addDRCCostVia(i, j, z - 1);
                    // } else {
                    //   gridGraph.subDRCCostVia(i, j, z - 1);
                    // }
                    if (QUICKDRCTEST)
                    {
                        cout << "    (" << i << ", " << j << ", " << z - 1 << ") U minSpc via" << endl;
                    }
                }
                else
                {
                    // modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, false, i, j, z);
                    modMinSpacingCostVia_eol(box, tmpBx, type, false, i, j, z);
                }
                // modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, false, i, j, z);
            }
            if (viaDefU)
            {
                tmpBx.set(viaBoxU);
                tmpBx.transform(xform);
                distSquare = box2boxDistSquare(box, tmpBx, dx, dy);
                prl = max(dx, dy);
                // curr is ps
                if (isCurrPs)
                {
                    if (dx == 0 && dy > 0)
                    {
                        prl = viaBoxU.right() - viaBoxU.left();
                    }
                    else if (dx > 0 && dy == 0)
                    {
                        prl = viaBoxU.top() - viaBoxU.bottom();
                    }
                }
                if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
                {
                    reqDist = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
                }
                else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
                {
                    reqDist = static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2viaU), prl);
                }
                else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
                {
                    reqDist = static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2viaU, prl);
                }
                if (distSquare < reqDist * reqDist)
                {
                    switch (type)
                    {
                    case 0:
                        gridGraph.subDRCCostVia(i, j, z);
                        break;
                    case 1:
                        gridGraph.addDRCCostVia(i, j, z);
                        break;
                    case 2:
                        gridGraph.subShapeCostVia(i, j, z); // safe access
                        break;
                    case 3:
                        gridGraph.addShapeCostVia(i, j, z); // safe access
                        break;
                    default:;
                    }
                    // if (isAddPathCost) {
                    //   gridGraph.addDRCCostVia(i, j, z);
                    // } else {
                    //   gridGraph.subDRCCostVia(i, j, z);
                    // }
                    if (QUICKDRCTEST)
                    {
                        cout << "    (" << i << ", " << j << ", " << z << ") U minSpc via" << endl;
                    }
                }
                else
                {
                    // modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, true, i, j, z);
                    modMinSpacingCostVia_eol(box, tmpBx, type, true, i, j, z);
                }
                // modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, true, i, j, z);
            }
        }
    }
    // cout <<"planer mod " <<cnt <<" edges" <<endl;
}

/*inline*/ void FlexDRWorker::modMinSpacingCostVia_eol_helper(const frBox &box, const frBox &testBox, int type, bool isUpperVia,
                                                              frMIdx i, frMIdx j, frMIdx z)
{
    if (testBox.overlaps(box, false))
    {
        if (isUpperVia)
        {
            switch (type)
            {
            case 0:
                gridGraph.subDRCCostVia(i, j, z);
                break;
            case 1:
                gridGraph.addDRCCostVia(i, j, z);
                break;
            case 2:
                gridGraph.subShapeCostVia(i, j, z); // safe access
                break;
            case 3:
                gridGraph.addShapeCostVia(i, j, z); // safe access
                break;
            default:;
            }
            // if (isAddPathCost) {
            //   gridGraph.addDRCCostVia(i, j, z);
            // } else {
            //   gridGraph.subDRCCostVia(i, j, z);
            // }
            if (QUICKDRCTEST)
            {
                cout << "    (" << i << ", " << j << ", " << z << ") U minSpc eol helper" << endl;
            }
        }
        else
        {
            switch (type)
            {
            case 0:
                gridGraph.subDRCCostVia(i, j, z - 1);
                break;
            case 1:
                gridGraph.addDRCCostVia(i, j, z - 1);
                break;
            case 2:
                gridGraph.subShapeCostVia(i, j, z - 1); // safe access
                break;
            case 3:
                gridGraph.addShapeCostVia(i, j, z - 1); // safe access
                break;
            default:;
            }
            // if (isAddPathCost) {
            //   gridGraph.addDRCCostVia(i, j, z - 1);
            // } else {
            //   gridGraph.subDRCCostVia(i, j, z - 1);
            // }
            if (QUICKDRCTEST)
            {
                cout << "    (" << i << ", " << j << ", " << z - 1 << ") U minSpc eol helper" << endl;
            }
        }
    }
}

/*inline*/ void FlexDRWorker::modMinSpacingCostVia_eol(const frBox &box, const frBox &tmpBx, int type, bool isUpperVia,
                                                       frMIdx i, frMIdx j, frMIdx z)
{
    auto lNum = gridGraph.getLayerNum(z);
    frBox testBox;
    if (getDesign()->getTech()->getLayer(lNum)->hasEolSpacing())
    {
        for (auto eolCon : getDesign()->getTech()->getLayer(lNum)->getEolSpacing())
        {
            auto eolSpace = eolCon->getMinSpacing();
            auto eolWidth = eolCon->getEolWidth();
            auto eolWithin = eolCon->getEolWithin();
            // eol to up and down
            if (tmpBx.right() - tmpBx.left() < eolWidth)
            {
                testBox.set(tmpBx.left() - eolWithin, tmpBx.top(), tmpBx.right() + eolWithin, tmpBx.top() + eolSpace);
                // modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);
                modMinSpacingCostVia_eol_helper(box, testBox, type, isUpperVia, i, j, z);

                testBox.set(tmpBx.left() - eolWithin, tmpBx.bottom() - eolSpace, tmpBx.right() + eolWithin, tmpBx.bottom());
                // modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);
                modMinSpacingCostVia_eol_helper(box, testBox, type, isUpperVia, i, j, z);
            }
            // eol to left and right
            if (tmpBx.top() - tmpBx.bottom() < eolWidth)
            {
                testBox.set(tmpBx.right(), tmpBx.bottom() - eolWithin, tmpBx.right() + eolSpace, tmpBx.top() + eolWithin);
                // modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);
                modMinSpacingCostVia_eol_helper(box, testBox, type, isUpperVia, i, j, z);

                testBox.set(tmpBx.left() - eolSpace, tmpBx.bottom() - eolWithin, tmpBx.left(), tmpBx.top() + eolWithin);
                // modMinSpacingCostVia_eol_helper(box, testBox, isAddPathCost, isUpperVia, i, j, z);
                modMinSpacingCostVia_eol_helper(box, testBox, type, isUpperVia, i, j, z);
            }
        }
    }
}

void FlexDRWorker::modMinimumcutCostVia(const frBox &box, frMIdx z, int type, bool isUpperVia)
{
    auto lNum = gridGraph.getLayerNum(z);
    // obj1 = curr obj
    frCoord width1 = box.width();
    frCoord length1 = box.length();
    // obj2 = other obj
    // default via dimension
    frViaDef *viaDef = nullptr;
    if (isUpperVia)
    {
        viaDef = (lNum < getDesign()->getTech()->getTopLayerNum()) ? getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef() : nullptr;
    }
    else
    {
        viaDef = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef() : nullptr;
    }
    if (viaDef == nullptr)
    {
        return;
    }
    frVia via(viaDef);
    frBox viaBox(0, 0, 0, 0);
    if (isUpperVia)
    {
        via.getCutBBox(viaBox);
    }
    else
    {
        via.getCutBBox(viaBox);
    }

    FlexMazeIdx mIdx1, mIdx2;
    frBox bx, tmpBx, sViaBox;
    frTransform xform;
    frPoint pt;
    frCoord dx, dy;
    frVia sVia;
    for (auto &con : getDesign()->getTech()->getLayer(lNum)->getMinimumcutConstraints())
    {
        // check via2cut to box
        // check whether via can be placed on the pin
        // if (!con->hasLength() && width1 > con->getWidth()) {
        if ((!con->hasLength() || (con->hasLength() && length1 > con->getLength())) && width1 > con->getWidth())
        {
            bool checkVia2 = false;
            if (!con->hasConnection())
            {
                checkVia2 = true;
            }
            else
            {
                if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isUpperVia)
                {
                    checkVia2 = true;
                }
                else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isUpperVia)
                {
                    checkVia2 = true;
                }
            }
            if (!checkVia2)
            {
                continue;
            }
            // block via on pin
            frCoord dist = 0;
            if (con->hasLength())
            {
                dist = con->getDistance();
                // conservative for macro pin
                // TODO: revert the += to be more accurate and check qor change
                dist += getDesign()->getTech()->getLayer(lNum)->getPitch();
            }
            // assumes width always > 2
            bx.set(box.left() - dist - (viaBox.right() - 0) + 1,
                   box.bottom() - dist - (viaBox.top() - 0) + 1,
                   box.right() + dist + (0 - viaBox.left()) - 1,
                   box.top() + dist + (0 - viaBox.bottom()) - 1);
            gridGraph.getIdxBox(mIdx1, mIdx2, bx);

            for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
            {
                for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
                {
                    gridGraph.getPoint(pt, i, j);
                    xform.set(pt);
                    tmpBx.set(viaBox);
                    if (gridGraph.isSVia(i, j, isUpperVia ? z : z - 1))
                    {
                        // auto sViaDef= apSVia[FlexMazeIdx(i, j, isUpperVia ? z : z - 1)];
                        auto sViaDef = apSVia[FlexMazeIdx(i, j, isUpperVia ? z : z - 1)]->getAccessViaDef();
                        sVia.setViaDef(sViaDef);
                        if (isUpperVia)
                        {
                            sVia.getCutBBox(sViaBox);
                        }
                        else
                        {
                            sVia.getCutBBox(sViaBox);
                        }
                        tmpBx.set(sViaBox);
                        // continue;
                    }
                    tmpBx.transform(xform);
                    box2boxDistSquareNew(box, tmpBx, dx, dy);
                    // prl = max(-dx, -dy);
                    if (!con->hasLength())
                    {
                        if (dx <= 0 && dy <= 0)
                        {
                            ;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    else
                    {
                        if (dx > 0 && dy > 0 && dx + dy < dist)
                        {
                            ;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    if (isUpperVia)
                    {
                        switch (type)
                        {
                        case 0:
                            gridGraph.subDRCCostVia(i, j, z); // safe access
                            break;
                        case 1:
                            gridGraph.addDRCCostVia(i, j, z); // safe access
                            break;
                        case 2:
                            gridGraph.subShapeCostVia(i, j, z); // safe access
                            break;
                        case 3:
                            gridGraph.addShapeCostVia(i, j, z); // safe access
                            break;
                        default:;
                        }
                        if (QUICKDRCTEST)
                        {
                            cout << "    (" << i << ", " << j << ", " << z << ") U minSpc via" << endl;
                        }
                    }
                    else
                    {
                        switch (type)
                        {
                        case 0:
                            gridGraph.subDRCCostVia(i, j, z - 1); // safe access
                            break;
                        case 1:
                            gridGraph.addDRCCostVia(i, j, z - 1); // safe access
                            break;
                        case 2:
                            gridGraph.subShapeCostVia(i, j, z - 1); // safe access
                            break;
                        case 3:
                            gridGraph.addShapeCostVia(i, j, z - 1); // safe access
                            break;
                        default:;
                        }
                        if (QUICKDRCTEST)
                        {
                            cout << "    (" << i << ", " << j << ", " << z - 1 << ") U minSpc via" << endl;
                        }
                    }
                }
            }
        }
        //// has length, check via out of pin
        // if (con->hasLength() && length1 > con->getLength() && width1 > con->getWidth()) {
        //   bool checkVia2 = false;
        //   if (!con->hasConnection()) {
        //     checkVia2 = true;
        //   } else {
        //     if (con->getConnection() == frMinimumcutConnectionEnum::FROMABOVE && isVia2Above) {
        //       checkVia2 = true;
        //     } else if (con->getConnection() == frMinimumcutConnectionEnum::FROMBELOW && !isVia2Above) {
        //       checkVia2 = true;
        //     }
        //   }
        //   if (!checkVia2) {
        //     continue;
        //   }
        //   // block via outside of pin
        //   frCoord dist = con->getDistance();
        //   // assumes width always > 2
        //   bx.set(box.left()   - dist - (viaBox.right() - 0) + 1,
        //          box.bottom() - dist - (viaBox.top() - 0) + 1,
        //          box.right()  + dist + (0 - viaBox.left())  - 1,
        //          box.top()    + dist + (0 - viaBox.bottom()) - 1);
        //   gridGraph.getIdxBox(mIdx1, mIdx2, bx);
        // }
    }
}

void FlexDRWorker::modMinSpacingCostVia(const frBox &box, frMIdx z, int type, bool isUpperVia, bool isCurrPs, bool isBlockage)
{
    auto lNum = gridGraph.getLayerNum(z);
    // obj1 = curr obj
    frCoord width1 = box.width();
    frCoord length1 = box.length();
    // obj2 = other obj
    // default via dimension
    frViaDef *viaDef = nullptr;
    if (isUpperVia)
    {
        viaDef = (lNum < getDesign()->getTech()->getTopLayerNum()) ? getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef() : nullptr;
    }
    else
    {
        viaDef = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef() : nullptr;
    }
    if (viaDef == nullptr)
    {
        return;
    }
    frVia via(viaDef);
    frBox viaBox(0, 0, 0, 0);
    if (isUpperVia)
    {
        via.getLayer1BBox(viaBox);
    }
    else
    {
        via.getLayer2BBox(viaBox);
    }
    frCoord width2 = viaBox.width();
    frCoord length2 = viaBox.length();

    // via prl should check min area patch metal if not fat via
    frCoord defaultWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();
    bool isH = (getDesign()->getTech()->getLayer(lNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
    bool isFatVia = (isH) ? (viaBox.top() - viaBox.bottom() > defaultWidth) : (viaBox.right() - viaBox.left() > defaultWidth);

    frCoord length2_mar = length2;
    frCoord patchLength = 0;
    if (!isFatVia)
    {
        auto minAreaConstraint = getDesign()->getTech()->getLayer(lNum)->getAreaConstraint();
        auto minArea = minAreaConstraint ? minAreaConstraint->getMinArea() : 0;
        patchLength = frCoord(ceil(1.0 * minArea / defaultWidth / getDesign()->getTech()->getManufacturingGrid())) *
                      frCoord(getDesign()->getTech()->getManufacturingGrid());
        length2_mar = max(length2_mar, patchLength);
    }

    // spacing value needed
    frCoord bloatDist = 0;
    auto con = getDesign()->getTech()->getLayer(lNum)->getMinSpacing();
    if (con)
    {
        if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
        {
            bloatDist = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
        }
        else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
        {
            bloatDist = (isBlockage && USEMINSPACING_OBS && !isFatVia) ? static_cast<frSpacingTablePrlConstraint *>(con)->findMin() : static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2), isCurrPs ? (length2_mar) : min(length1, length2_mar));
        }
        else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
        {
            bloatDist = (isBlockage && USEMINSPACING_OBS && !isFatVia) ? static_cast<frSpacingTableTwConstraint *>(con)->findMin() : static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2, isCurrPs ? (length2_mar) : min(length1, length2_mar));
        }
        else
        {
            cout << "Warning: min spacing rule not supporterd" << endl;
            return;
        }
    }
    else
    {
        cout << "Warning: no min spacing rule" << endl;
        return;
    }
    // if (isBlockage && (!USEMINSPACING_OBS)) {
    //   bloatDist = 0;
    // }
    // other obj eol spc to curr obj
    // no need to blaot eolWithin because eolWithin always < minSpacing
    frCoord bloatDistEolX = 0;
    frCoord bloatDistEolY = 0;
    for (auto con : getDesign()->getTech()->getLayer(lNum)->getEolSpacing())
    {
        auto eolSpace = con->getMinSpacing();
        auto eolWidth = con->getEolWidth();
        // eol up and down
        if (viaBox.right() - viaBox.left() < eolWidth)
        {
            bloatDistEolY = max(bloatDistEolY, eolSpace);
        }
        // eol left and right
        if (viaBox.top() - viaBox.bottom() < eolWidth)
        {
            bloatDistEolX = max(bloatDistEolX, eolSpace);
        }
    }
    // frCoord bloatDistSquare = bloatDist * bloatDist;

    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    // assumes width always > 2
    frBox bx(box.left() - max(bloatDist, bloatDistEolX) - (viaBox.right() - 0) + 1,
             box.bottom() - max(bloatDist, bloatDistEolY) - (viaBox.top() - 0) + 1,
             box.right() + max(bloatDist, bloatDistEolX) + (0 - viaBox.left()) - 1,
             box.top() + max(bloatDist, bloatDistEolY) + (0 - viaBox.bottom()) - 1);
    gridGraph.getIdxBox(mIdx1, mIdx2, bx);

    frPoint pt;
    frBox tmpBx;
    frCoord distSquare = 0;
    frCoord dx, dy, prl;
    frTransform xform;
    frCoord reqDist = 0;
    frBox sViaBox;
    frVia sVia;
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            gridGraph.getPoint(pt, i, j);
            xform.set(pt);
            tmpBx.set(viaBox);
            if (gridGraph.isSVia(i, j, isUpperVia ? z : z - 1))
            {
                // auto sViaDef= apSVia[FlexMazeIdx(i, j, isUpperVia ? z : z - 1)];
                auto sViaDef = apSVia[FlexMazeIdx(i, j, isUpperVia ? z : z - 1)]->getAccessViaDef();
                sVia.setViaDef(sViaDef);
                if (isUpperVia)
                {
                    sVia.getLayer1BBox(sViaBox);
                }
                else
                {
                    sVia.getLayer2BBox(sViaBox);
                }
                tmpBx.set(sViaBox);
                // continue;
            }
            tmpBx.transform(xform);
            distSquare = box2boxDistSquareNew(box, tmpBx, dx, dy);
            prl = max(-dx, -dy);
            // curr is ps
            if (isCurrPs)
            {
                if (-dy >= 0 && prl == -dy)
                {
                    prl = viaBox.top() - viaBox.bottom();
                    // ignore svia effect here...
                    if (!isH && !isFatVia)
                    {
                        prl = max(prl, patchLength);
                    }
                }
                else if (-dx >= 0 && prl == -dx)
                {
                    prl = viaBox.right() - viaBox.left();
                    if (isH && !isFatVia)
                    {
                        prl = max(prl, patchLength);
                    }
                }
            }
            else
            {
                ;
            }
            if (con->typeId() == frConstraintTypeEnum::frcSpacingConstraint)
            {
                reqDist = static_cast<frSpacingConstraint *>(con)->getMinSpacing();
            }
            else if (con->typeId() == frConstraintTypeEnum::frcSpacingTablePrlConstraint)
            {
                reqDist = (isBlockage && USEMINSPACING_OBS && !isFatVia) ? static_cast<frSpacingTablePrlConstraint *>(con)->findMin() : static_cast<frSpacingTablePrlConstraint *>(con)->find(max(width1, width2), prl);
            }
            else if (con->typeId() == frConstraintTypeEnum::frcSpacingTableTwConstraint)
            {
                reqDist = (isBlockage && USEMINSPACING_OBS && !isFatVia) ? static_cast<frSpacingTableTwConstraint *>(con)->findMin() : static_cast<frSpacingTableTwConstraint *>(con)->find(width1, width2, prl);
            }
            // if (isBlockage && (!USEMINSPACING_OBS)) {
            //   reqDist = 0;
            // }
            if (distSquare < reqDist * reqDist)
            {
                if (isUpperVia)
                {
                    switch (type)
                    {
                    case 0:
                        gridGraph.subDRCCostVia(i, j, z); // safe access
                        break;
                    case 1:
                        gridGraph.addDRCCostVia(i, j, z); // safe access
                        break;
                    case 2:
                        gridGraph.subShapeCostVia(i, j, z); // safe access
                        break;
                    case 3:
                        gridGraph.addShapeCostVia(i, j, z); // safe access
                        break;
                    default:;
                    }
                    // if (isAddPathCost) {
                    //   gridGraph.addDRCCostVia(i, j, z);
                    // } else {
                    //   gridGraph.subDRCCostVia(i, j, z);
                    // }
                    if (QUICKDRCTEST)
                    {
                        cout << "    (" << i << ", " << j << ", " << z << ") U minSpc via" << endl;
                    }
                }
                else
                {
                    switch (type)
                    {
                    case 0:
                        gridGraph.subDRCCostVia(i, j, z - 1); // safe access
                        break;
                    case 1:
                        gridGraph.addDRCCostVia(i, j, z - 1); // safe access
                        break;
                    case 2:
                        gridGraph.subShapeCostVia(i, j, z - 1); // safe access
                        break;
                    case 3:
                        gridGraph.addShapeCostVia(i, j, z - 1); // safe access
                        break;
                    default:;
                    }
                    // if (isAddPathCost) {
                    //   gridGraph.addDRCCostVia(i, j, z - 1);
                    // } else {
                    //   gridGraph.subDRCCostVia(i, j, z - 1);
                    // }
                    if (QUICKDRCTEST)
                    {
                        cout << "    (" << i << ", " << j << ", " << z - 1 << ") U minSpc via" << endl;
                    }
                }
            }
            // eol, other obj to curr obj
            // modMinSpacingCostVia_eol(box, tmpBx, isAddPathCost, isUpperVia, i, j, z);
            modMinSpacingCostVia_eol(box, tmpBx, type, isUpperVia, i, j, z);
        }
    }
}

// eolType == 0: planer
// eolType == 1: down
// eolType == 2: up
/*inline*/ void FlexDRWorker::modEolSpacingCost_helper(const frBox &testbox, frMIdx z, int type, int eolType)
{
    auto lNum = gridGraph.getLayerNum(z);
    frBox bx;
    if (eolType == 0)
    {
        // layer default width
        frCoord width2 = getDesign()->getTech()->getLayer(lNum)->getWidth();
        frCoord halfwidth2 = width2 / 2;
        // assumes width always > 2
        bx.set(testbox.left() - halfwidth2 + 1, testbox.bottom() - halfwidth2 + 1,
               testbox.right() + halfwidth2 - 1, testbox.top() + halfwidth2 - 1);
    }
    else
    {
        // default via dimension
        frViaDef *viaDef = nullptr;
        if (eolType == 1)
        {
            viaDef = (lNum > getDesign()->getTech()->getBottomLayerNum()) ? getDesign()->getTech()->getLayer(lNum - 1)->getDefaultViaDef() : nullptr;
        }
        else if (eolType == 2)
        {
            viaDef = (lNum < getDesign()->getTech()->getTopLayerNum()) ? getDesign()->getTech()->getLayer(lNum + 1)->getDefaultViaDef() : nullptr;
        }
        if (viaDef == nullptr)
        {
            return;
        }
        frVia via(viaDef);
        frBox viaBox(0, 0, 0, 0);
        if (eolType == 2)
        { // upper via
            via.getLayer1BBox(viaBox);
        }
        else
        {
            via.getLayer2BBox(viaBox);
        }
        // assumes via bbox always > 2
        bx.set(testbox.left() - (viaBox.right() - 0) + 1, testbox.bottom() - (viaBox.top() - 0) + 1,
               testbox.right() + (0 - viaBox.left()) - 1, testbox.top() + (0 - viaBox.bottom()) - 1);
    }

    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    gridGraph.getIdxBox(mIdx1, mIdx2, bx); // >= bx

    frVia sVia;
    frBox sViaBox;
    frTransform xform;
    frPoint pt;

    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            if (eolType == 0)
            {
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostPlanar(i, j, z); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostPlanar(i, j, z); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostPlanar(i, j, z); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostPlanar(i, j, z); // safe access
                    break;
                default:;
                }
                // if (isAddPathCost) {
                //   gridGraph.addDRCCostPlanar(i, j, z); // safe access
                // } else {
                //   gridGraph.subDRCCostPlanar(i, j, z); // safe access
                // }
                if (QUICKDRCTEST)
                {
                    cout << "    (" << i << ", " << j << ", " << z << ") N eolSpc" << endl;
                }
            }
            else if (eolType == 1)
            {
                if (gridGraph.isSVia(i, j, z - 1))
                {
                    gridGraph.getPoint(pt, i, j);
                    auto sViaDef = apSVia[FlexMazeIdx(i, j, z - 1)]->getAccessViaDef();
                    sVia.setViaDef(sViaDef);
                    sVia.setOrigin(pt);
                    sVia.getLayer2BBox(sViaBox);
                    if (!sViaBox.overlaps(testbox, false))
                    {
                        continue;
                    }
                }
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostVia(i, j, z - 1); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostVia(i, j, z - 1); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostVia(i, j, z - 1); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostVia(i, j, z - 1); // safe access
                    break;
                default:;
                }
                // if (isAddPathCost) {
                //   gridGraph.addDRCCostVia(i, j, z - 1); // safe access
                // } else {
                //   gridGraph.subDRCCostVia(i, j, z - 1); // safe access
                // }
                if (QUICKDRCTEST)
                {
                    cout << "    (" << i << ", " << j << ", " << z - 1 << ") U eolSpc" << endl;
                }
            }
            else if (eolType == 2)
            {
                if (gridGraph.isSVia(i, j, z))
                {
                    gridGraph.getPoint(pt, i, j);
                    auto sViaDef = apSVia[FlexMazeIdx(i, j, z)]->getAccessViaDef();
                    sVia.setViaDef(sViaDef);
                    sVia.setOrigin(pt);
                    sVia.getLayer1BBox(sViaBox);
                    if (!sViaBox.overlaps(testbox, false))
                    {
                        continue;
                    }
                }
                switch (type)
                {
                case 0:
                    gridGraph.subDRCCostVia(i, j, z); // safe access
                    break;
                case 1:
                    gridGraph.addDRCCostVia(i, j, z); // safe access
                    break;
                case 2:
                    gridGraph.subShapeCostVia(i, j, z); // safe access
                    break;
                case 3:
                    gridGraph.addShapeCostVia(i, j, z); // safe access
                    break;
                default:;
                }
                // if (isAddPathCost) {
                //   gridGraph.addDRCCostVia(i, j, z); // safe access
                // } else {
                //   gridGraph.subDRCCostVia(i, j, z); // safe access
                // }
                if (QUICKDRCTEST)
                {
                    cout << "    (" << i << ", " << j << ", " << z << ") U eolSpc" << endl;
                }
            }
        }
    }
}

/*inline*/ void FlexDRWorker::modEolSpacingCost(const frBox &box, frMIdx z, int type, bool isSkipVia)
{
    auto lNum = gridGraph.getLayerNum(z);
    frBox testBox;
    if (getDesign()->getTech()->getLayer(lNum)->hasEolSpacing())
    {
        for (auto con : getDesign()->getTech()->getLayer(lNum)->getEolSpacing())
        {
            auto eolSpace = con->getMinSpacing();
            auto eolWidth = con->getEolWidth();
            auto eolWithin = con->getEolWithin();
            // curr obj to other obj eol
            // if (!isInitDR()) {
            //  cout <<"eolSpace/within = " <<eolSpace <<" " <<eolWithin <<endl;
            //}
            // eol to up and down
            if (box.right() - box.left() < eolWidth)
            {
                testBox.set(box.left() - eolWithin, box.top(), box.right() + eolWithin, box.top() + eolSpace);
                // if (!isInitDR()) {
                //   cout <<"  topBox " <<testBox <<endl;
                // }
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
                modEolSpacingCost_helper(testBox, z, type, 0);
                if (!isSkipVia)
                {
                    modEolSpacingCost_helper(testBox, z, type, 1);
                    modEolSpacingCost_helper(testBox, z, type, 2);
                }
                testBox.set(box.left() - eolWithin, box.bottom() - eolSpace, box.right() + eolWithin, box.bottom());
                // if (!isInitDR()) {
                //   cout <<"  bottomBox " <<testBox <<endl;
                // }
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
                modEolSpacingCost_helper(testBox, z, type, 0);
                if (!isSkipVia)
                {
                    modEolSpacingCost_helper(testBox, z, type, 1);
                    modEolSpacingCost_helper(testBox, z, type, 2);
                }
            }
            // eol to left and right
            if (box.top() - box.bottom() < eolWidth)
            {
                testBox.set(box.right(), box.bottom() - eolWithin, box.right() + eolSpace, box.top() + eolWithin);
                // if (!isInitDR()) {
                //   cout <<"  rightBox " <<testBox <<endl;
                // }
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
                modEolSpacingCost_helper(testBox, z, type, 0);
                if (!isSkipVia)
                {
                    modEolSpacingCost_helper(testBox, z, type, 1);
                    modEolSpacingCost_helper(testBox, z, type, 2);
                }
                testBox.set(box.left() - eolSpace, box.bottom() - eolWithin, box.left(), box.top() + eolWithin);
                // if (!isInitDR()) {
                //   cout <<"  leftBox " <<testBox <<endl;
                // }
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 0);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 1);
                // modEolSpacingCost_helper(testBox, z, isAddPathCost, 2);
                modEolSpacingCost_helper(testBox, z, type, 0);
                if (!isSkipVia)
                {
                    modEolSpacingCost_helper(testBox, z, type, 1);
                    modEolSpacingCost_helper(testBox, z, type, 2);
                }
            }
            // other obj to curr obj eol
        }
    }
}

// forbid via if it would trigger violation
void FlexDRWorker::modAdjCutSpacingCost_fixedObj(const frBox &origCutBox, frVia *origVia)
{
    if (origVia->getNet()->getType() != frNetEnum::frcPowerNet && origVia->getNet()->getType() != frNetEnum::frcGroundNet)
    {
        return;
    }
    auto lNum = origVia->getViaDef()->getCutLayerNum();
    for (auto con : getDesign()->getTech()->getLayer(lNum)->getCutSpacing())
    {
        if (con->getAdjacentCuts() == -1)
        {
            continue;
        }
        bool hasFixedViol = false;

        gtl::point_data<frCoord> origCenter((origCutBox.left() + origCutBox.right()) / 2, (origCutBox.bottom() + origCutBox.top()) / 2);
        gtl::rectangle_data<frCoord> origCutRect(origCutBox.left(), origCutBox.bottom(), origCutBox.right(), origCutBox.top());

        frBox viaBox;
        origVia->getCutBBox(viaBox);

        auto reqDistSquare = con->getCutSpacing();
        reqDistSquare *= reqDistSquare;

        auto cutWithin = con->getCutWithin();
        frBox queryBox;
        viaBox.bloat(cutWithin, queryBox);

        vector<rq_rptr_value_t<frBlockObject>> result;
        getRegionQuery()->query(queryBox, lNum, result);

        frBox box;
        for (auto &[boostb, obj] : result)
        {
            box.set(boostb.min_corner().x(), boostb.min_corner().y(), boostb.max_corner().x(), boostb.max_corner().y());
            if (obj->typeId() == frcVia)
            {
                auto via = static_cast<frVia *>(obj);
                if (via->getNet()->getType() != frNetEnum::frcPowerNet && via->getNet()->getType() != frNetEnum::frcGroundNet)
                {
                    continue;
                }
                if (origCutBox == box)
                {
                    continue;
                }

                gtl::rectangle_data<frCoord> cutRect(box.left(), box.bottom(), box.right(), box.top());
                gtl::point_data<frCoord> cutCenterPt((box.left() + box.right()) / 2, (box.bottom() + box.top()) / 2);

                frCoord distSquare = 0;
                if (con->hasCenterToCenter())
                {
                    distSquare = gtl::distance_squared(origCenter, cutCenterPt);
                }
                else
                {
                    distSquare = gtl::square_euclidean_distance(origCutRect, cutRect);
                }

                if (distSquare < reqDistSquare)
                {
                    hasFixedViol = true;
                    break;
                }
            }
        }

        // block adjacent via idx if will trigger violation
        // pessimistic since block a box
        if (hasFixedViol)
        {
            FlexMazeIdx mIdx1, mIdx2;
            frBox spacingBox;
            auto reqDist = con->getCutSpacing();
            auto cutWidth = getDesign()->getTech()->getLayer(lNum)->getWidth();
            if (con->hasCenterToCenter())
            {
                spacingBox.set(origCenter.x() - reqDist, origCenter.y() - reqDist, origCenter.x() + reqDist, origCenter.y() + reqDist);
            }
            else
            {
                origCutBox.bloat(reqDist + cutWidth / 2, spacingBox);
            }
            // cout << "  @@@ debug: blocking for adj (" << spacingBox.left() / 2000.0 << ", " << spacingBox.bottom() / 2000.0
            //      << ") -- (" << spacingBox.right() / 2000.0 << ", " << spacingBox.top() / 2000.0 << ")\n";
            gridGraph.getIdxBox(mIdx1, mIdx2, spacingBox);

            frMIdx zIdx = gridGraph.getMazeZIdx(origVia->getViaDef()->getLayer1Num());
            for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
            {
                for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
                {
                    gridGraph.setBlocked(i, j, zIdx, frDirEnum::U);
                }
            }
        }
    }
}

/*inline*/ void FlexDRWorker::modCutSpacingCost(const frBox &box, frMIdx z, int type, bool isBlockage)
{
    auto lNum = gridGraph.getLayerNum(z) + 1;
    if (!getDesign()->getTech()->getLayer(lNum)->hasCutSpacing())
    {
        return;
    }
    // obj1 = curr obj
    // obj2 = other obj
    // default via dimension
    frViaDef *viaDef = getDesign()->getTech()->getLayer(lNum)->getDefaultViaDef();
    frVia via(viaDef);
    frBox viaBox(0, 0, 0, 0);
    via.getCutBBox(viaBox);

    // spacing value needed
    frCoord bloatDist = 0;
    for (auto con : getDesign()->getTech()->getLayer(lNum)->getCutSpacing())
    {
        bloatDist = max(bloatDist, con->getCutSpacing());
        if (con->getAdjacentCuts() != -1 && isBlockage)
        {
            bloatDist = max(bloatDist, con->getCutWithin());
        }
    }
    // frCoord bloatDistSquare = bloatDist * bloatDist;

    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    // assumes width always > 2
    frBox bx(box.left() - bloatDist - (viaBox.right() - 0) + 1,
             box.bottom() - bloatDist - (viaBox.top() - 0) + 1,
             box.right() + bloatDist + (0 - viaBox.left()) - 1,
             box.top() + bloatDist + (0 - viaBox.bottom()) - 1);
    gridGraph.getIdxBox(mIdx1, mIdx2, bx);

    frPoint pt;
    frBox tmpBx;
    frCoord distSquare = 0;
    frCoord c2cSquare = 0;
    frCoord dx, dy, prl;
    frTransform xform;
    // frCoord reqDist = 0;
    frCoord reqDistSquare = 0;
    frPoint boxCenter, tmpBxCenter;
    boxCenter.set((box.left() + box.right()) / 2, (box.bottom() + box.top()) / 2);
    frCoord currDistSquare = 0;
    bool hasViol = false;
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            for (auto &uFig : via.getViaDef()->getCutFigs())
            {
                auto obj = static_cast<frRect *>(uFig.get());
                gridGraph.getPoint(pt, i, j);
                xform.set(pt);
                obj->getBBox(tmpBx);
                // tmpBx.set(viaBox);
                tmpBx.transform(xform);
                tmpBxCenter.set((tmpBx.left() + tmpBx.right()) / 2, (tmpBx.bottom() + tmpBx.top()) / 2);
                distSquare = box2boxDistSquareNew(box, tmpBx, dx, dy);
                c2cSquare = (boxCenter.x() - tmpBxCenter.x()) * (boxCenter.x() - tmpBxCenter.x()) +
                            (boxCenter.y() - tmpBxCenter.y()) * (boxCenter.y() - tmpBxCenter.y());
                prl = max(-dx, -dy);
                for (auto con : getDesign()->getTech()->getLayer(lNum)->getCutSpacing())
                {
                    hasViol = false;
                    // reqDist = con->getCutSpacing();
                    reqDistSquare = con->getCutSpacing() * con->getCutSpacing();
                    currDistSquare = con->hasCenterToCenter() ? c2cSquare : distSquare;
                    if (con->hasSameNet())
                    {
                        continue;
                    }
                    if (con->isLayer())
                    {
                        ;
                    }
                    else if (con->isAdjacentCuts())
                    {
                        // OBS always count as within distance instead of cut spacing
                        if (isBlockage)
                        {
                            reqDistSquare = con->getCutWithin() * con->getCutWithin();
                        }
                        if (currDistSquare < reqDistSquare)
                        {
                            hasViol = true;
                            // should disable hasViol and modify this part to new grid graph
                        }
                    }
                    else if (con->isParallelOverlap())
                    {
                        if (prl > 0 && currDistSquare < reqDistSquare)
                        {
                            hasViol = true;
                        }
                    }
                    else if (con->isArea())
                    {
                        auto currArea = max(box.length() * box.width(), tmpBx.length() * tmpBx.width());
                        if (currArea >= con->getCutArea() && currDistSquare < reqDistSquare)
                        {
                            hasViol = true;
                        }
                    }
                    else
                    {
                        if (currDistSquare < reqDistSquare)
                        {
                            hasViol = true;
                        }
                    }
                    if (hasViol)
                    {
                        switch (type)
                        {
                        case 0:
                            gridGraph.subDRCCostVia(i, j, z); // safe access
                            break;
                        case 1:
                            gridGraph.addDRCCostVia(i, j, z); // safe access
                            break;
                        case 2:
                            gridGraph.subShapeCostVia(i, j, z); // safe access
                            break;
                        case 3:
                            gridGraph.addShapeCostVia(i, j, z); // safe access
                            break;
                        default:;
                        }
                        // if (isAddPathCost) {
                        //   gridGraph.addDRCCostVia(i, j, z);
                        // } else {
                        //   gridGraph.subDRCCostVia(i, j, z);
                        // }
                        if (QUICKDRCTEST)
                        {
                            cout << "    (" << i << ", " << j << ", " << z << ") U cutSpc" << endl;
                        }
                        break;
                    }
                }
            }
        }
    }
}

/*inline*/ void FlexDRWorker::modInterLayerCutSpacingCost(const frBox &box, frMIdx z, int type, bool isUpperVia, bool isBlockage)
{
    auto cutLayerNum1 = gridGraph.getLayerNum(z) + 1;
    auto cutLayerNum2 = isUpperVia ? cutLayerNum1 + 2 : cutLayerNum1 - 2;
    auto z2 = isUpperVia ? z + 1 : z - 1;

    frViaDef *viaDef = nullptr;
    if (isUpperVia)
    {
        viaDef = (cutLayerNum2 <= getDesign()->getTech()->getTopLayerNum()) ? getDesign()->getTech()->getLayer(cutLayerNum2)->getDefaultViaDef() : nullptr;
    }
    else
    {
        viaDef = (cutLayerNum2 >= getDesign()->getTech()->getBottomLayerNum()) ? getDesign()->getTech()->getLayer(cutLayerNum2)->getDefaultViaDef() : nullptr;
    }
    if (viaDef == nullptr)
    {
        return;
    }
    frCutSpacingConstraint *con = getDesign()->getTech()->getLayer(cutLayerNum1)->getInterLayerCutSpacing(cutLayerNum2, false);
    if (con == nullptr)
    {
        con = getDesign()->getTech()->getLayer(cutLayerNum2)->getInterLayerCutSpacing(cutLayerNum1, false);
    }
    if (con == nullptr)
    {
        return;
    }
    // obj1 = curr obj
    // obj2 = other obj
    // default via dimension
    frVia via(viaDef);
    frBox viaBox(0, 0, 0, 0);
    via.getCutBBox(viaBox);

    // spacing value needed
    frCoord bloatDist = con->getCutSpacing();
    // frCoord bloatDistSquare = bloatDist * bloatDist;

    FlexMazeIdx mIdx1;
    FlexMazeIdx mIdx2;
    // assumes width always > 2
    frBox bx(box.left() - bloatDist - (viaBox.right() - 0) + 1,
             box.bottom() - bloatDist - (viaBox.top() - 0) + 1,
             box.right() + bloatDist + (0 - viaBox.left()) - 1,
             box.top() + bloatDist + (0 - viaBox.bottom()) - 1);
    gridGraph.getIdxBox(mIdx1, mIdx2, bx);

    frPoint pt;
    frBox tmpBx;
    frCoord distSquare = 0;
    frCoord c2cSquare = 0;
    frCoord dx, dy /*, prl*/;
    frTransform xform;
    // frCoord reqDist = 0;
    frCoord reqDistSquare = 0;
    frPoint boxCenter, tmpBxCenter;
    boxCenter.set((box.left() + box.right()) / 2, (box.bottom() + box.top()) / 2);
    frCoord currDistSquare = 0;
    bool hasViol = false;
    for (int i = mIdx1.x(); i <= mIdx2.x(); i++)
    {
        for (int j = mIdx1.y(); j <= mIdx2.y(); j++)
        {
            for (auto &uFig : via.getViaDef()->getCutFigs())
            {
                auto obj = static_cast<frRect *>(uFig.get());
                gridGraph.getPoint(pt, i, j);
                xform.set(pt);
                obj->getBBox(tmpBx);
                // tmpBx.set(viaBox);
                tmpBx.transform(xform);
                tmpBxCenter.set((tmpBx.left() + tmpBx.right()) / 2, (tmpBx.bottom() + tmpBx.top()) / 2);
                distSquare = box2boxDistSquareNew(box, tmpBx, dx, dy);
                c2cSquare = (boxCenter.x() - tmpBxCenter.x()) * (boxCenter.x() - tmpBxCenter.x()) +
                            (boxCenter.y() - tmpBxCenter.y()) * (boxCenter.y() - tmpBxCenter.y());
                // prl = max(-dx, -dy);
                hasViol = false;
                // reqDist = con->getCutSpacing();
                reqDistSquare = con->getCutSpacing() * con->getCutSpacing();
                currDistSquare = con->hasCenterToCenter() ? c2cSquare : distSquare;
                if (currDistSquare < reqDistSquare)
                {
                    hasViol = true;
                }
                if (hasViol)
                {
                    switch (type)
                    {
                    case 0:
                        gridGraph.subDRCCostVia(i, j, z2); // safe access
                        break;
                    case 1:
                        gridGraph.addDRCCostVia(i, j, z2); // safe access
                        break;
                    case 2:
                        gridGraph.subShapeCostVia(i, j, z2); // safe access
                        break;
                    case 3:
                        gridGraph.addShapeCostVia(i, j, z2); // safe access
                        break;
                    default:;
                    }
                    // if (isAddPathCost) {
                    //   gridGraph.addDRCCostVia(i, j, z);
                    // } else {
                    //   gridGraph.subDRCCostVia(i, j, z);
                    // }
                    if (QUICKDRCTEST)
                    {
                        // if (true) {
                        if (isUpperVia)
                        {
                            cout << "    (" << i << ", " << j << ", " << z << ") U inter layer cutSpc" << endl;
                        }
                        else
                        {
                            cout << "    (" << i << ", " << j << ", " << z << ") D inter layer cutSpc" << endl;
                        }
                    }
                    break;
                }
            }
        }
    }
}

void FlexDRWorker::addPathCost(drConnFig *connFig)
{
    modPathCost(connFig, 1);
}

void FlexDRWorker::subPathCost(drConnFig *connFig)
{
    modPathCost(connFig, 0);
}

/*inline*/ void FlexDRWorker::modPathCost(drConnFig *connFig, int type)
{
    if (connFig->typeId() == drcPathSeg)
    {
        auto obj = static_cast<drPathSeg *>(connFig);
        // auto net = obj->getNet();
        FlexMazeIdx bi, ei;
        obj->getMazeIdx(bi, ei);
        if (QUICKDRCTEST)
        {
            cout << "  ";
            if (type)
            {
                cout << "add";
            }
            else
            {
                cout << "sub";
            }
            cout << "PsCost for " << bi << " -- " << ei << endl;
        }
        // new
        frBox box;
        obj->getBBox(box);

        // modMinSpacingCostPlaner(net, box, bi.z(), isAddPathCost);
        // modMinSpacingCostVia(box, bi.z(), isAddPathCost, true,  true);
        // modMinSpacingCostVia(box, bi.z(), isAddPathCost, false, true);
        modMinSpacingCostPlaner(box, bi.z(), type);
        modMinSpacingCostVia(box, bi.z(), type, true, true);
        modMinSpacingCostVia(box, bi.z(), type, false, true);
        modViaForbiddenThrough(bi, ei, type);
        // modMinSpacingCost(net, box, bi.z(), isAddPathCost, true);
        // modEolSpacingCost(box, bi.z(), isAddPathCost);
        //  wrong way wire cannot have eol problem: (1) with via at end, then via will add eol cost; (2) with pref-dir wire, then not eol edge
        bool isHLayer = (getDesign()->getTech()->getLayer(gridGraph.getLayerNum(bi.z()))->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir);
        if (isHLayer == (bi.y() == ei.y()))
        {
            modEolSpacingCost(box, bi.z(), type);
        }
    }
    else if (connFig->typeId() == drcPatchWire)
    {
        auto obj = static_cast<drPatchWire *>(connFig);
        // auto net = obj->getNet();
        frMIdx zIdx = gridGraph.getMazeZIdx(obj->getLayerNum());
        // FlexMazeIdx bi, ei;
        // obj->getMazeIdx(bi, ei);
        // if (TEST) {
        //   cout <<"  ";
        //   if (isAddPathCost) {
        //     cout <<"add";
        //   } else {
        //     cout <<"sub";
        //   }
        //   cout <<"PsCost for " <<bi <<" -- " <<ei <<endl;
        // }
        // new
        frBox box;
        obj->getBBox(box);

        // modMinSpacingCostPlaner(net, box, zIdx, isAddPathCost);
        // modMinSpacingCostVia(box, zIdx, isAddPathCost, true,  true);
        // modMinSpacingCostVia(box, zIdx, isAddPathCost, false, true);
        modMinSpacingCostPlaner(box, zIdx, type);
        modMinSpacingCostVia(box, zIdx, type, true, true);
        modMinSpacingCostVia(box, zIdx, type, false, true);
        // modMinSpacingCost(net, box, bi.z(), isAddPathCost, true);
        // modEolSpacingCost(box, zIdx, isAddPathCost);
        modEolSpacingCost(box, zIdx, type);
    }
    else if (connFig->typeId() == drcVia)
    {
        auto obj = static_cast<drVia *>(connFig);
        // auto net = obj->getNet();
        FlexMazeIdx bi, ei;
        obj->getMazeIdx(bi, ei);
        if (QUICKDRCTEST)
        {
            cout << "  ";
            if (type)
            {
                cout << "add";
            }
            else
            {
                cout << "sub";
            }
            cout << "ViaCost for " << bi << " -- " << ei << endl;
        }
        // new

        frBox box;
        obj->getLayer1BBox(box); // assumes enclosure for via is always rectangle

        // modMinSpacingCostPlaner(net, box, bi.z(), isAddPathCost);
        // modMinSpacingCostVia(box, bi.z(), isAddPathCost, true,  false);
        // modMinSpacingCostVia(box, bi.z(), isAddPathCost, false, false);
        modMinSpacingCostPlaner(box, bi.z(), type);
        modMinSpacingCostVia(box, bi.z(), type, true, false);
        modMinSpacingCostVia(box, bi.z(), type, false, false);
        // modMinSpacingCost(net, box, bi.z(), isAddPathCost, false);
        // modEolSpacingCost(box, bi.z(), isAddPathCost);
        modEolSpacingCost(box, bi.z(), type);

        obj->getLayer2BBox(box); // assumes enclosure for via is always rectangle

        // modMinSpacingCostPlaner(net, box, ei.z(), isAddPathCost);
        // modMinSpacingCostVia(box, ei.z(), isAddPathCost, true,  false);
        // modMinSpacingCostVia(box, ei.z(), isAddPathCost, false, false);
        modMinSpacingCostPlaner(box, ei.z(), type);
        modMinSpacingCostVia(box, ei.z(), type, true, false);
        modMinSpacingCostVia(box, ei.z(), type, false, false);
        // modMinSpacingCost(net, box, ei.z(), isAddPathCost, false);
        // modEolSpacingCost(box, ei.z(), isAddPathCost);
        modEolSpacingCost(box, ei.z(), type);

        // obj->getCutBBox(box);
        frTransform xform;
        frPoint pt;
        obj->getOrigin(pt);
        xform.set(pt);
        for (auto &uFig : obj->getViaDef()->getCutFigs())
        {
            // if (uFig->typeId() == frcRect) {
            auto rect = static_cast<frRect *>(uFig.get());
            rect->getBBox(box);
            box.transform(xform);
            // modCutSpacingCost(box, bi.z(), isAddPathCost);
            modCutSpacingCost(box, bi.z(), type);
            modInterLayerCutSpacingCost(box, bi.z(), type, true);
            modInterLayerCutSpacingCost(box, bi.z(), type, false);
            //}
        }
    }
}
// 用于迭代初始化和根据特定规则对需要重新路由的网络进行排序的函数
bool FlexDRWorker::mazeIterInit_sortRerouteNets(int mazeIter, vector<drNet *> &rerouteNets)
{
    // 定义第一种排序比较器，根据网络的引脚数量和占据区域大小进行比较 - 升序
    auto rerouteNetsComp1 = [](drNet *const &a, drNet *const &b)
    {
        frBox boxA, boxB;
        a->getPinBox(boxA); // 获取A的引脚盒子
        b->getPinBox(boxB); // 获取b的引脚盒子
        auto areaA = (boxA.right() - boxA.left()) * (boxA.top() - boxA.bottom());
        auto areaB = (boxB.right() - boxB.left()) * (boxB.top() - boxB.bottom());
        // auto areaA = (boxA.right() - boxA.left()) + (boxA.top() - boxA.bottom());
        // auto areaB = (boxB.right() - boxB.left()) + (boxB.top() - boxB.bottom());
        //  根据引脚数和面积来排序，如果引脚数相同，则比较面积，如果面积也相同，比较ID，引脚从小到大，面积从小到达，ID从小到大
        return (a->getNumPinsIn() == b->getNumPinsIn() ? (areaA == areaB ? a->getId() < b->getId() : areaA < areaB) : a->getNumPinsIn() < b->getNumPinsIn());
        // return (areaA == areaB ? a->getId() < b->getId() : areaA > areaB);
    };
    // 定义第二种排序比较器，根据网络的标记距离进行比较，标记距离相同则比较ID，都是从小到大
    auto rerouteNetsComp2 = [](drNet *const &a, drNet *const &b)
    {
        return (a->getMarkerDist() == b->getMarkerDist() ? a->getId() < b->getId() : a->getMarkerDist() < b->getMarkerDist());
    };
    // 定义第三种排序比较器，组合了引脚数、面积以及标记距离的比较-若标记距离同则比较引脚数，若引脚数同则比较面积，若面积同则比较ID，皆从小到大
    auto rerouteNetsComp3 = [mazeIter](drNet *const &a, drNet *const &b)
    {
        frBox boxA, boxB;
        a->getPinBox(boxA);
        b->getPinBox(boxB);
        auto areaA = (boxA.right() - boxA.left()) * (boxA.top() - boxA.bottom());
        auto areaB = (boxB.right() - boxB.left()) * (boxB.top() - boxB.bottom());
        bool sol = (a->getNumPinsIn() == b->getNumPinsIn() ? (areaA == areaB ? a->getId() < b->getId() : areaA < areaB) : a->getNumPinsIn() < b->getNumPinsIn());
        // if (mazeIter % 2 == 1) {
        //   sol = !sol;
        // }
        return (a->getMarkerDist() == b->getMarkerDist() ? sol : a->getMarkerDist() < b->getMarkerDist());
    };
    bool sol = false; // 初始化排序结果标志为假
    // 根据迭代次数和修复模式选择合适的排序策略
    // sort
    if (mazeIter == 0)
    {
        //修复模式0和9用第一种比较器，12用第二种，345用第三种
        switch (getFixMode())
        {
        case 0:
        case 9:
            sort(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp1); // 使用第一种比较器进行排序
            break;
        case 1:
        case 2:
            sort(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp2);
            break;
        case 3:
        case 4:
        case 5:
            sort(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp3);
            break;
        default:;
        }
        // to be removed// 实施基于种子的随机交换，增加排序的随机性 OR_SEED初始定义为-1
        if (OR_SEED != -1 && rerouteNets.size() >= 2)
        {
            uniform_int_distribution<int> distribution(0, rerouteNets.size() - 1);
            default_random_engine generator(OR_SEED);
            int numSwap = (double)(rerouteNets.size()) * OR_K;  //OR_K初始化定义为0
            for (int i = 0; i < numSwap; i++)
            {
                int idx = distribution(generator);
                swap(rerouteNets[idx], rerouteNets[(idx + 1) % rerouteNets.size()]);
            }
        }

        sol = true; // 设置排序结果标志为真
    }
    else
    {
        switch (getFixMode())
        {
        case 0:
            //若不是第一次迭代，则按照第一种比较器进行排序-根据网络的引脚数量和占据区域大小进行比较
            sol = next_permutation(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp1);
            // shuffle(rerouteNets.begin(), rerouteNets.end(), default_random_engine(0));
            break;
        case 1://第2-3次迭代什么都不做
        case 2:
            break;
        case 3://第4-6次迭代也什么都不做
        case 4:
        case 5:
            // sol = next_permutation(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp1);
            break;
        default:;
        }
        sol = true; // 设置排序结果标志为真
    }
    return sol; // 返回排序是否成功的结果
                // return true;
}

/*
void FlexDRWorker::mazeIterInit_initDR(vector<drNet*> &rerouteNets) {
  for (auto &net: nets) {
    rerouteNets.push_back(net.get());
  }
  mazeIterInit_sortRerouteNets(rerouteNets);
  for (auto &net: rerouteNets) {
    net->setModified(true);
    net->setNumMarkers(0);
    // add via access cost when net is not routed
    if (RESERVE_VIA_ACCESS) {
      initMazeCost_via_helper(net, true);
    }
    //net->clear();
  }
}
*/
// 本函数主要用于初始化网络的迷宫搜索和修复过程。
//  它根据不同的迭代和撕裂模式，选择哪些网络需要重新布线，并对这些网络进行排序和初始化处理。
//  如果是首次迭代，可能会处理所有网络或仅处理标记为撕裂的网络。在后续迭代中，可能会根据修复模式清空并重新选择网络。
//  此外，还会对网络进行排序，并根据排序结果初始化网络，包括标记网络、重置标记数量、移除旧的路径成本、设置新的迷宫成本等。
//  temporary settings to test search and repair// 临时设置用于测试搜索和修复
bool FlexDRWorker::mazeIterInit_searchRepair(int mazeIter, vector<drNet *> &rerouteNets)
{
    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象
    int cnt = 0;                                      // 用于计数的变量
    // 如果是第一次迭代
    if (mazeIter == 0)
    {
        if (getRipupMode() == 0)
        { // 根据撕裂模式（RipupMode）决定是否重新布线
            for (auto &net : nets)
            { // 只重新布线标记为撕裂的网络
                if (net->isRipup())
                {
                    rerouteNets.push_back(net.get());
                }
            }
        }
        else if (getRipupMode() == 1)
        {
            for (auto &net : nets)
            { // 重新布线所有网络
                rerouteNets.push_back(net.get());
            }
        }
        else if (getRipupMode() == 2)
        {
            for (auto &net : nets)
            {
                rerouteNets.push_back(net.get());
            }
        }
    }
    else
    { // 在后续迭代中，根据修复模式（FixMode）可能会清除并选择网络
        if (getFixMode() == 1 || getFixMode() == 2 || getFixMode() == 3 || getFixMode() == 4 || getFixMode() == 5)
        {
            rerouteNets.clear();
            for (auto &net : nets)
            {
                if (net->isRipup())
                {
                    rerouteNets.push_back(net.get());
                }
            }
        }
    }
    // change the drNet magical sorting bit here
    // to do
    // sort(rerouteNets.begin(), rerouteNets.end(),
    //     [](drNet* const &a, drNet* const &b) {return *a < *b;});// 调用排序函数对需要重新布线的网络进行排序（具体实现省略
    auto sol = mazeIterInit_sortRerouteNets(mazeIter, rerouteNets); /////
    if (sol)
    { // 如果排序成功，则对每个网络进行初始化处理
        for (auto &net : rerouteNets)
        {
            net->setModified(true); // 标记网络为已修改
            if (net->getFrNet())
            {
                net->getFrNet()->setModified(true); // modified 被改进的// 标记对应的前向网络为已修改
            }
            net->setNumMarkers(0); // 重置标记数量
            for (auto &uConnFig : net->getRouteConnFigs())
            {
                subPathCost(uConnFig.get());              // 减少路径成本
                workerRegionQuery.remove(uConnFig.get()); // worker region query// 从工作区域查询中移除路径
                cnt++;
            }
            // add via access cost when net is not routed
            if (RESERVE_VIA_ACCESS)
            { // 如果设置了保留通道访问成本，初始化迷宫成本
                initMazeCost_via_helper(net, true);
            }
            net->clear(); // 清楚网络信息
        }
    }
    // cout <<"sr sub " <<cnt <<" connfig costs" <<endl;// 输出处理的连接成本数量
    return sol;
}

void FlexDRWorker::mazeIterInit_drcCost()
{
    switch (getFixMode())
    {       // 根据修复模式调整设计规则检查（DRC）成本
    case 0: // 模式0：不作改变
        break;
    case 1:
    case 2:
        workerDRCCost *= 2; // 模式1和2：将DRC成本加倍
        break;
    default: // 其他模式不作处理
        ;
    }
}

void FlexDRWorker::mazeIterInit_resetRipup()
{ // 如果修复模式是1到5，重置所有网络的撕裂状态和标记距离
    if (getFixMode() == 1 || getFixMode() == 2 || getFixMode() == 3 || getFixMode() == 4 || getFixMode() == 5)
    {
        for (auto &net : nets)
        {
            net->resetRipup();
            net->resetMarkerDist();
        }
    }
}

bool FlexDRWorker::mazeIterInit(int mazeIter, vector<drNet *> &rerouteNets)
{
    mazeIterInit_resetRipup();                               // reset net ripup// 重置网络的撕裂状态
    initMazeCost_marker();                                   // add marker cost, set net ripup// 初始化迷宫成本并设置网络撕裂
    mazeIterInit_drcCost();                                  // 初始化DRC成本
    return mazeIterInit_searchRepair(mazeIter, rerouteNets); // add to rerouteNets// 执行搜索和修复，添加到重新路由的网络列表中
}

void FlexDRWorker::route_2_init_getNets_sort(vector<drNet *> &rerouteNets)
{
    auto rerouteNetsComp1 = [](drNet *const &a, drNet *const &b) { // 定义两种比较器，用于根据不同的网络特性进行排序
        frBox boxA, boxB;
        a->getPinBox(boxA);
        b->getPinBox(boxB);
        auto areaA = (boxA.right() - boxA.left()) * (boxA.top() - boxA.bottom());
        auto areaB = (boxB.right() - boxB.left()) * (boxB.top() - boxB.bottom());
        // auto areaA = (boxA.right() - boxA.left()) + (boxA.top() - boxA.bottom());
        // auto areaB = (boxB.right() - boxB.left()) + (boxB.top() - boxB.bottom());
        return (a->getNumPinsIn() == b->getNumPinsIn() ? (areaA == areaB ? a->getId() < b->getId() : areaA < areaB) : a->getNumPinsIn() < b->getNumPinsIn());
        // return (areaA == areaB ? a->getId() < b->getId() : areaA > areaB);
    };
    auto rerouteNetsComp2 = [](drNet *const &a, drNet *const &b)
    {
        return (a->getMarkerDist() == b->getMarkerDist() ? a->getId() < b->getId() : a->getMarkerDist() < b->getMarkerDist());
    };
    // sort
    if (getRipupMode() == 1)
    { // 根据撕裂模式选择合适的排序策略
        sort(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp1);
    }
    else
    {
        sort(rerouteNets.begin(), rerouteNets.end(), rerouteNetsComp2);
    }
}

void FlexDRWorker::route_2_init_getNets(vector<drNet *> &tmpNets)
{
    initMazeCost_marker(); // 初始化迷宫成本标记
    for (auto &net : nets)
    { // 遍历所有网络
        if (getRipupMode() == 1 || net->isRipup())
        { // 如果撕裂模式为1或网络需要被撕裂，则加入到临时网络列表
            tmpNets.push_back(net.get());
        }
    }
    // 对临时网络列表进行排序处理
    route_2_init_getNets_sort(tmpNets);
}

void FlexDRWorker::route_2_ripupNet(drNet *net)
{
    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象
    for (auto &uConnFig : net->getRouteConnFigs())
    {                                             // 遍历网络中的所有连接图形
        subPathCost(uConnFig.get());              // sub quick drc cost// 减少路径成本
        workerRegionQuery.remove(uConnFig.get()); // sub worker region query// 从工作区域查询中移除连接图形
    }
    // add via access cost when net is not routed
    if (RESERVE_VIA_ACCESS)
    { // 如果设置了保留通道访问成本，并且网络未被布线，初始化迷宫成本
        initMazeCost_via_helper(net, true);
    }
    net->clear(); // delete connfigs// 清除网络中的连接图形
}

void FlexDRWorker::route_2_pushNet(deque<drNet *> &rerouteNets, drNet *net, bool ripUp, bool isPushFront)
{
    // 检查网络是否已经在队列中或已达到最大迭代次数
    if (net->isInQueue() || net->getNumReroutes() >= getMazeEndIter())
    {
        return; // 如果是，不做任何操作
    }
    if (isPushFront)
    {                                // 根据isPushFront标志，决定是将网络推到队列前端还是后端
        rerouteNets.push_front(net); // 推到前端
    }
    else
    {
        rerouteNets.push_back(net); // 推到后端
    }
    if (ripUp)
    { // 如果指定了ripUp，执行撕裂操作
        route_2_ripupNet(net);
    } // 设置网络为已加入队列状态
    net->setInQueue();
}

drNet *FlexDRWorker::route_2_popNet(deque<drNet *> &rerouteNets)
{ // 从队列前端取出网络
    auto net = rerouteNets.front();
    rerouteNets.pop_front(); // 移除队列中的第一个网络
    // 如果网络已被路由，则进行撕裂操作
    if (net->isRouted())
    {
        route_2_ripupNet(net);
    }
    net->resetInQueue(); // 重置网络的队列状态
    if (net->getNumReroutes() < getMazeEndIter())
    { // 检查并更新网络的迭代次数
        net->addNumReroutes();
        net->setRouted();
    }
    else
    {
        net = nullptr; // 如果超过迭代次数，将网络设置为null
    }
    return net; // 返回处理后的网络
}

void FlexDRWorker::route_2_init(deque<drNet *> &rerouteNets)
{
    vector<drNet *> tmpNets;
    route_2_init_getNets(tmpNets); // 初始化获取需要重新路由的网络
    // 将这些网络加入到重新路由的队列中，并标记为需要撕裂
    for (auto &net : tmpNets)
    {
        route_2_pushNet(rerouteNets, net, true);
    }
}

//迷宫线网初始化
void FlexDRWorker::mazeNetInit(drNet *net)
{
    gridGraph.resetStatus(); // 重置网格图状态，准备新的布线尝试
    // sub term / instterm cost when net is about to route
    initMazeCost_terms(net->getFrNetTerms(), false, true); // 当网络即将布线时，减少术语/实例术语成本
    // sub via access cost when net is about to route
    // route_queue does not need to reserve
    if (getFixMode() < 9 && RESERVE_VIA_ACCESS)
    { // 如果修复模式小于9且设置了通过访问保留，则减少通过访问成本
        initMazeCost_via_helper(net, false);
    }
    if (isFollowGuide())
    { // 如果设置了跟随导引线，则初始化导引线成本
        initMazeCost_guide_helper(net, true);
    }
    // add minimum cut cost from objs in ext ring when the net is about to route
    initMazeCost_minCut_helper(net, true);    // 当网络即将布线时，添加最小切割成本
    initMazeCost_ap_helper(net, false);       // 初始化辅助点成本
    initMazeCost_boundary_helper(net, false); // 初始化边界成本
}

void FlexDRWorker::mazeNetEnd(drNet *net)
{
    // add term / instterm cost back when net is about to end
    initMazeCost_terms(net->getFrNetTerms(), true, true); // 当网络布线结束时，添加术语/实例术语成本回来
    if (isFollowGuide())
    { // 如果设置了跟随导引线，在布线结束时重置导引线成本
        initMazeCost_guide_helper(net, false);
    }
    // 当网络布线结束时，减少最小切割成本
    // sub minimum cut cost from vias in ext ring when the net is about to end
    initMazeCost_minCut_helper(net, false);
    initMazeCost_ap_helper(net, true);       // 结束时添加辅助点成本
    initMazeCost_boundary_helper(net, true); // 结束时添加边界成本
}

void FlexDRWorker::route_drc()
{ // 从初始化到最终阶段的设计规则检查流程，包括处理潜在的DRC错误和应用几何修补。
    // DRCWorker drcWorker(getDesign(), fixedObjs);
    // drcWorker.addDRNets(nets);
    using namespace std::chrono;
    // high_resolution_clock::time_point t0 = high_resolution_clock::now();
    // drcWorker.init();
    // high_resolution_clock::time_point t1 = high_resolution_clock::now();
    // drcWorker.setup();
    // high_resolution_clock::time_point t2 = high_resolution_clock::now();
    //// drcWorker.main();
    // drcWorker.check();
    ////setMarkers(drcWorker.getViolations());
    // high_resolution_clock::time_point t3 = high_resolution_clock::now();
    ////drcWorker.report();
    //
    // duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);

    // duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);

    // duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
    // if (VERBOSE > 1) {
    //   stringstream ss;
    //   ss   <<"DRC  (INIT/SETUP/MAIN) " <<time_span0.count() <<" "
    //                                   <<time_span1.count() <<" "
    //                                   <<time_span2.count() <<" "
    //                                   <<endl;
    //   //ss <<"#viol = " <<markers.size() <<endl;
    //   ss <<"#viol(DRC) = " <<drcWorker.getViolations().size() <<endl;
    //   cout <<ss.str() <<flush;
    // }

    // new gcWorker
    FlexGCWorker gcWorker(getDesign(), this);                             // 初始化 FlexGCWorker，用于执行几何检查
    gcWorker.setExtBox(getExtBox());                                      // 设置外部盒子，用于定义检查区域
    gcWorker.setDrcBox(getDrcBox());                                      // 设置DRC盒子，可能用于限定DRC检查的特定区域
    gcWorker.setEnableSurgicalFix(true);                                  // 启用详细的修复功能，可能用于自动修复DRC错误
    high_resolution_clock::time_point t0x = high_resolution_clock::now(); // 记录初始化的时间点
    gcWorker.init();                                                      // 初始化几何检查工作器
    high_resolution_clock::time_point t1x = high_resolution_clock::now();
    gcWorker.main(); // 执行主要的几何检查流程
    high_resolution_clock::time_point t2x = high_resolution_clock::now();

    // write back GC patches// 处理所有生成的修补线（Patch Wire）
    for (auto &pwire : gcWorker.getPWires())
    {
        auto net = pwire->getNet();
        if (!net)
        {
            cout << "Error: pwire with no net\n";
            exit(1); // 如果没有网络与修补线相关联，则输出错误并退出
        }
        auto tmpPWire = make_unique<drPatchWire>(); // 创建一个新的修补线对象，并设置其属性
        tmpPWire->setLayerNum(pwire->getLayerNum());
        frPoint origin;
        pwire->getOrigin(origin);
        tmpPWire->setOrigin(origin);
        frBox box;
        pwire->getOffsetBox(box);
        tmpPWire->setOffsetBox(box);
        tmpPWire->addToNet(net);
        // 将新的修补线对象加入到网络和区域查询中
        unique_ptr<drConnFig> tmp(std::move(tmpPWire));
        auto &workerRegionQuery = getWorkerRegionQuery();
        workerRegionQuery.add(tmp.get());
        net->addRoute(tmp);
    }

    // drcWorker.main();
    // 结束几何检查的流程
    gcWorker.end();
    // 设置检测到的标记
    setMarkers(gcWorker.getMarkers());
    high_resolution_clock::time_point t3x = high_resolution_clock::now();
    // drcWorker.report();
    //  计算并输出各阶段所花费的时间
    duration<double> time_span0x = duration_cast<duration<double>>(t1x - t0x);

    duration<double> time_span1x = duration_cast<duration<double>>(t2x - t1x);

    duration<double> time_span2x = duration_cast<duration<double>>(t3x - t2x);
    if (VERBOSE > 1)
    {
        stringstream ss;
        ss << "GC  (INIT/MAIN/END) " << time_span0x.count() << " "
           << time_span1x.count() << " "
           << time_span2x.count() << " "
           << endl;
        ss << "#viol = " << markers.size() << endl; // 输出冲突数
        cout << ss.str() << flush;
    }
}

void FlexDRWorker::route_postRouteViaSwap()
{
    // 该函数 route_postRouteViaSwap() 是在详细布线过程中执行的后处理步骤，专门用于在布线后对连通图中的通孔（vias）进行调整。
    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象
    set<FlexMazeIdx> modifiedViaIdx;                  // 用于记录被修改过的通孔索引
    frBox box;                                        // 用于存储查询框
    vector<drConnFig *> results;                      // 存放查询结果的数组
    frPoint bp;                                       // 用于存储通孔的原点坐标
    FlexMazeIdx bi, ei;                               // 通孔的开始和结束索引
    bool flag = false;                                // 标志位，用于检测是否进行了修改
    for (auto &marker : getMarkers())
    { // 遍历所有标记（通常是DRC错误相关）
        results.clear();
        marker.getBBox(box);                         // 获取标记的边界盒
        auto lNum = marker.getLayerNum();            // 获取标记的层编号
        workerRegionQuery.query(box, lNum, results); // 在区域查询中查询此范围内的连接图形
        for (auto &connFig : results)
        { // 遍历查询结果，寻找通孔
            if (connFig->typeId() == drcVia)
            { // 确认类型为通孔
                auto obj = static_cast<drVia *>(connFig);
                obj->getMazeIdx(bi, ei); // 获取通孔的迷宫索引
                obj->getOrigin(bp);      // 获取通孔的原点
                // 根据是否是初始设计规则检查确定条件
                bool condition1 = isInitDR() ? (bp.x() < getRouteBox().right()) : (bp.x() <= getRouteBox().right());
                bool condition2 = isInitDR() ? (bp.y() < getRouteBox().top()) : (bp.y() <= getRouteBox().top());
                // only swap vias when the net is marked modified
                // 只有当通孔在路由框内且网络标记为已修改时，才考虑交换通孔
                if (bp.x() >= getRouteBox().left() && bp.y() >= getRouteBox().bottom() && condition1 && condition2 &&
                    obj->getNet()->isModified())
                {
                    auto it = apSVia.find(bi);
                    // 如果找到了可以交换的通孔，并且此通孔还未被修改
                    if (modifiedViaIdx.find(bi) == modifiedViaIdx.end() && it != apSVia.end())
                    {
                        auto ap = it->second;
                        if (ap->nextAccessViaDef())
                        {
                            auto newViaDef = ap->getAccessViaDef();
                            workerRegionQuery.remove(obj); // 从查询区域中移除当前通孔
                            obj->setViaDef(newViaDef);     // 设置新的通孔定义
                            workerRegionQuery.add(obj);    // 将新的通孔加入查询区域
                            modifiedViaIdx.insert(bi);     // 记录该索引为已修改
                            flag = true;
                        }
                    }
                }
            }
        }
    }
    if (flag)
    {
        route_drc();
    }
}
// 是一个专门为处理与终端间距（End-of-Line, EOL）约束有关的DRC问题而设计的后处理步骤，
// 主要针对出现在设计中的阻挡物（blockages）附近的通孔（via）进行调整
void FlexDRWorker::route_postRouteViaSwap_PA()
{
    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象
    frBox box;                                        // 用于查询的盒子
    vector<drConnFig *> results;                      // 用于存放查询结果
    frPoint bp;                                       // 通孔原点坐标
    FlexMazeIdx bi, ei;                               // 通孔的起始和结束迷宫索引

    vector<pair<drVia *, drAccessPattern *>> vioAPs; // 存放违规的通孔和访问模式

    for (auto &marker : gcWorker->getMarkers())
    { // 遍历几何约束工作器中的标记
        // bool hasAPVia = false;
        bool hasOBS = false; // 标记是否有阻挡物
        auto con = marker->getConstraint();
        // only handle EOL// 只处理终端间距的约束
        if (con->typeId() != frConstraintTypeEnum::frcSpacingEndOfLineConstraint)
        {
            continue;
        }
        // check OBS// 检查标记源是否包含阻挡物
        for (auto src : marker->getSrcs())
        {
            if (src)
            {
                if (src->typeId() == frcInstBlockage || src->typeId() == frcBlockage)
                {
                    hasOBS = true;
                    break;
                }
            }
        }
        if (!hasOBS)
        {
            continue;
        }
        // check APVia
        results.clear(); // 清空结果并查询相关区域
        marker->getBBox(box);
        auto lNum = marker->getLayerNum();
        workerRegionQuery.query(box, lNum, results);
        for (auto &connFig : results)
        {
            if (connFig->typeId() == drcVia)
            {
                auto obj = static_cast<drVia *>(connFig);
                obj->getMazeIdx(bi, ei);
                obj->getOrigin(bp);
                bool condition1 = isInitDR() ? (bp.x() < getRouteBox().right()) : (bp.x() <= getRouteBox().right());
                bool condition2 = isInitDR() ? (bp.y() < getRouteBox().top()) : (bp.y() <= getRouteBox().top());
                // only swap vias when the net is marked modified// 只在通孔的网络被标记修改时进行交换
                if (bp.x() >= getRouteBox().left() && bp.y() >= getRouteBox().bottom() && condition1 && condition2 &&
                    obj->getNet()->isModified())
                {
                    auto it = apSVia.find(bi);
                    if (it != apSVia.end())
                    {
                        auto ap = it->second;
                        vioAPs.push_back(make_pair(obj, ap));
                        break;
                    }
                }
            }
        }
    }

    // iterate violation ap and try// 遍历所有违规的通孔和访问模式，尝试找到更好的通孔设计
    for (auto &[via, ap] : vioAPs)
    {
        // cout << "@@@ here\n";
        int bestNumVios = INT_MAX;
        auto origAccessViaDef = ap->getAccessViaDef();
        auto bestAccessViaDef = ap->getAccessViaDef();
        auto cutLayerNum = bestAccessViaDef->getCutLayerNum();
        gcWorker->setTargetNet(via->getNet()->getFrNet());
        gcWorker->setEnableSurgicalFix(false);
        gcWorker->main();
        bestNumVios = gcWorker->getMarkers().size();

        via->getOrigin(bp);
        // cout << "@@@ attemping via at (" << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
        // for (int i = 0; i < ap->getNumAccessViaDef(); i++) {
        //   if (ap->nextAccessViaDef()) {
        //     auto newViaDef = ap->getAccessViaDef();
        //     workerRegionQuery.remove(via);
        //     via->setViaDef(newViaDef);
        //     workerRegionQuery.add(via);
        //     gcWorker->updateDRNet(via->getNet());
        //     gcWorker->main();
        //     if ((int)gcWorker->getMarkers().size() < bestNumVios) {
        //       bestNumVios = gcWorker->getMarkers().size();
        //       bestAccessViaDef = newViaDef;
        //     }
        //     // cout << "@@@   trying " << newViaDef->getName() << "\n";
        //   }
        // }

        for (auto newViaDef : getDesign()->getTech()->getLayer(cutLayerNum)->getViaDefs())
        {
            if (newViaDef->getNumCut() != 1)
            {
                continue;
            }
            if (newViaDef == origAccessViaDef)
            {
                continue;
            }
            workerRegionQuery.remove(via);
            via->setViaDef(newViaDef);
            workerRegionQuery.add(via);
            gcWorker->updateDRNet(via->getNet());
            gcWorker->main();
            if ((int)gcWorker->getMarkers().size() < bestNumVios)
            {
                bestNumVios = gcWorker->getMarkers().size();
                bestAccessViaDef = newViaDef;
            }
        }

        // set to best one // 应用最佳的通孔定义
        workerRegionQuery.remove(via);
        via->setViaDef(bestAccessViaDef);
        workerRegionQuery.add(via);
        gcWorker->updateDRNet(via->getNet());

        // if (origAccessViaDef != bestAccessViaDef) {
        //   cout << "@@@ route_postRouteViaSwap_PA find better via for PA\n";
        // }
    }
}

bool FlexDRWorker::route_2_x2_addHistoryCost(const frMarker &marker)
{
    // 用于在布线后处理阶段，将历史成本（一种惩罚分数）添加到引起设计规则冲突（DRC）的布线元素上。
    // 这有助于在后续布线过程中避免这些区域，改善布线质量。
    bool enableOutput = false; // 是否输出调试信息的标志
    // bool enableOutput = true;

    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象
    vector<rq_rptr_value_t<drConnFig>> results;       // 用于存储查询结果
    frBox mBox, bloatBox;                             // 存储标记的边界框和扩展后的边界框
    FlexMazeIdx mIdx1, mIdx2;                         // 边界框在迷宫中的索引

    marker.getBBox(mBox);             // 获取标记的边界框
    auto lNum = marker.getLayerNum(); // 获取标记所在层的编号

    workerRegionQuery.query(mBox, lNum, results); // 在给定层上查询边界框内的连接图形
    frPoint bp, ep;                               // 起点和终点坐标
    frBox objBox;                                 // 连接图形的边界框
    frCoord width;                                // 连接图形的宽度
    frSegStyle segStyle;                          // 连接图形的样式
    FlexMazeIdx objMIdx1, objMIdx2;               // 连接图形的起始和结束迷宫索引
    bool fixable = false;                         // 是否可以修复的标志

    for (auto &[boostB, connFig] : results)
    { // 遍历查询到的所有连接图形
        objBox.set(boostB.min_corner().x(), boostB.min_corner().y(), boostB.max_corner().x(), boostB.max_corner().y());
        if (connFig->typeId() == drcPathSeg)
        { // 如果是路径段
            auto obj = static_cast<drPathSeg *>(connFig);
            // skip if unfixable obj
            obj->getPoints(bp, ep); // 获取路径段的起点和终点
            // 检查路径段是否在路由框内，不在则跳过
            if (!(getRouteBox().contains(bp) && getRouteBox().contains(ep)))
            {
                continue;
            }
            fixable = true; // 设置可以修复标志

            // skip if curr is irrelavant spacing violation
            // if (marker.hasDir() && marker.isH() && objBox.top() != mBox.bottom() && objBox.bottom() != mBox.top()) {
            //  continue;
            //} else if (marker.hasDir() && !marker.isH() && objBox.right() != mBox.left() && objBox.left() != mBox.right()) {
            //  continue;
            //}
            // add history cost
            // get points to mark up, markup up to "width" grid points to the left and right of pathseg
            obj->getStyle(segStyle);                     // 获取路径段的样式
            width = segStyle.getWidth();                 // 获取宽度
            mBox.bloat(width, bloatBox);                 // 根据宽度膨胀边界框
            gridGraph.getIdxBox(mIdx1, mIdx2, bloatBox); // 获取膨胀后边界框的迷宫索引
            obj->getMazeIdx(objMIdx1, objMIdx2);         // 获取路径段的迷宫索引
            bool isH = (objMIdx1.y() == objMIdx2.y());   // 判断是否为水平路径
                                                         // 根据路径方向增加历史成本
            if (isH)
            {
                for (int i = max(objMIdx1.x(), mIdx1.x()); i <= min(objMIdx2.x(), mIdx2.x()); i++)
                {
                    gridGraph.addMarkerCostPlanar(i, objMIdx1.y(), objMIdx1.z()); // 增加平面历史成本
                    if (enableOutput)
                    {
                        cout << "add marker cost planar @(" << i << ", " << objMIdx1.y() << ", " << objMIdx1.z() << ")" << endl;
                    }
                    planarHistoryMarkers.insert(FlexMazeIdx(i, objMIdx1.y(), objMIdx1.z())); // 记录历史标记位置
                }
            }
            else
            {
                for (int i = max(objMIdx1.y(), mIdx1.y()); i <= min(objMIdx2.y(), mIdx2.y()); i++)
                {
                    gridGraph.addMarkerCostPlanar(objMIdx1.x(), i, objMIdx1.z()); // 增加平面历史成本
                    if (enableOutput)
                    {
                        cout << "add marker cost planar @(" << objMIdx1.x() << ", " << i << ", " << objMIdx1.z() << ")" << endl;
                    }
                    planarHistoryMarkers.insert(FlexMazeIdx(objMIdx1.x(), i, objMIdx1.z())); // 记录历史标记位置
                }
            }
        }
        else if (connFig->typeId() == drcVia)
        { // 如果是通孔
            auto obj = static_cast<drVia *>(connFig);
            obj->getOrigin(bp); // 获取通孔原点
            // skip if unfixable obj//检查通孔是否在路由框内，不在则跳过
            if (!getRouteBox().contains(bp))
            {
                continue;
            }
            fixable = true; // 设置可以修复标志
            // skip if curr is irrelavant spacing violation
            // if (marker.hasDir() && marker.isH() && objBox.top() != mBox.bottom() && objBox.bottom() != mBox.top()) {
            //  continue;
            //} else if (marker.hasDir() && !marker.isH() && objBox.right() != mBox.left() && objBox.left() != mBox.right()) {
            //  continue;
            //}
            // add history cost
            obj->getMazeIdx(objMIdx1, objMIdx2);                                  // 获取通孔的迷宫索引
            gridGraph.addMarkerCostVia(objMIdx1.x(), objMIdx1.y(), objMIdx1.z()); // 增加通孔历史成本
            if (enableOutput)
            {
                cout << "add marker cost via @(" << objMIdx1.x() << ", " << objMIdx1.y() << ", " << objMIdx1.z() << ")" << endl;
            }
            viaHistoryMarkers.insert(objMIdx1); // 记录历史标记位置
        }
        else if (connFig->typeId() == drcPatchWire)
        {   // 如果是修补线
            // 当前未处理修补线相关逻辑
            ;
        }
    }
    return fixable; // 返回是否可以修复
}

void FlexDRWorker::route_2_x2_ripupNets(const frMarker &marker, drNet *net)
{
    bool enableOutput = false;

    auto &workerRegionQuery = getWorkerRegionQuery();
    vector<rq_rptr_value_t<drConnFig>> results;
    frBox mBox, bloatBox;
    FlexMazeIdx mIdx1, mIdx2;

    marker.getBBox(mBox);
    auto lNum = marker.getLayerNum();

    // ripup all nets within bloatbox
    frCoord bloatDist = 0;
    if (getDesign()->getTech()->getLayer(lNum)->getType() == frLayerTypeEnum::CUT)
    {
        if (getDesign()->getTech()->getTopLayerNum() >= lNum + 1 &&
            getDesign()->getTech()->getLayer(lNum + 1)->getType() == frLayerTypeEnum::ROUTING)
        {
            bloatDist = getDesign()->getTech()->getLayer(lNum + 1)->getWidth() * workerMarkerBloatWidth;
        }
        else if (getDesign()->getTech()->getBottomLayerNum() <= lNum - 1 &&
                 getDesign()->getTech()->getLayer(lNum - 1)->getType() == frLayerTypeEnum::ROUTING)
        {
            bloatDist = getDesign()->getTech()->getLayer(lNum - 1)->getWidth() * workerMarkerBloatWidth;
        }
    }
    else if (getDesign()->getTech()->getLayer(lNum)->getType() == frLayerTypeEnum::ROUTING)
    {
        bloatDist = getDesign()->getTech()->getLayer(lNum)->getWidth() * workerMarkerBloatWidth;
    }
    mBox.bloat(bloatDist, bloatBox);
    if (enableOutput)
    {
        double dbu = getDesign()->getTopBlock()->getDBUPerUU();
        cout << "marker @(" << mBox.left() / dbu << ", " << mBox.bottom() / dbu << ") ("
             << mBox.right() / dbu << ", " << mBox.top() / dbu << ") "
             << getDesign()->getTech()->getLayer(lNum)->getName() << " " << bloatDist
             << endl;
    }

    workerRegionQuery.query(bloatBox, lNum, results);
    frBox objBox;
    for (auto &[boostB, connFig] : results)
    {
        objBox.set(boostB.min_corner().x(), boostB.min_corner().y(), boostB.max_corner().x(), boostB.max_corner().y());
        // for pathseg-related marker, bloat marker by half width and add marker cost planar
        if (connFig->typeId() == drcPathSeg)
        {
            // cout <<"@@pathseg" <<endl;
            //  update marker dist
            auto dx = max(max(objBox.left(), mBox.left()) - min(objBox.right(), mBox.right()), 0);
            auto dy = max(max(objBox.bottom(), mBox.bottom()) - min(objBox.top(), mBox.top()), 0);
            connFig->getNet()->updateMarkerDist(dx * dx + dy * dy);

            if (connFig->getNet() != net)
            {
                connFig->getNet()->setRipup();
                if (enableOutput)
                {
                    cout << "ripup pathseg from " << connFig->getNet()->getFrNet()->getName() << endl;
                }
            }
            // for via-related marker, add marker cost via
        }
        else if (connFig->typeId() == drcVia)
        {
            // cout <<"@@via" <<endl;
            //  update marker dist
            auto dx = max(max(objBox.left(), mBox.left()) - min(objBox.right(), mBox.right()), 0);
            auto dy = max(max(objBox.bottom(), mBox.bottom()) - min(objBox.top(), mBox.top()), 0);
            connFig->getNet()->updateMarkerDist(dx * dx + dy * dy);

            auto obj = static_cast<drVia *>(connFig);
            obj->getMazeIdx(mIdx1, mIdx2);
            if (connFig->getNet() != net)
            {
                connFig->getNet()->setRipup();
                if (enableOutput)
                {
                    cout << "ripup via from " << connFig->getNet()->getFrNet()->getName() << endl;
                }
            }
        }
        else if (connFig->typeId() == drcPatchWire)
        {
            // TODO: could add marker // for now we think the other part in the violation would not be patchWire
        }
        else
        {
            cout << "Error: unsupporterd dr type" << endl;
        }
    }
}

/*
void FlexDRWorker::route_2_x2(drNet* net, deque<drNet*> &rerouteNets) {
  bool enableOutput = true;
  for (auto &net: nets) {
    net->resetRipup();
  }
  // old
  for (auto it = planarHistoryMarkers.begin(); it != planarHistoryMarkers.end();) {
    auto currIt = it;
    auto &mi = *currIt;
    ++it;
    if (gridGraph.decayMarkerCostPlanar(mi.x(), mi.y(), mi.z(), MARKERDECAY)) {
      planarHistoryMarkers.erase(currIt);
    }
  }
  for (auto it = viaHistoryMarkers.begin(); it != viaHistoryMarkers.end();) {
    auto currIt = it;
    auto &mi = *currIt;
    ++it;
    if (gridGraph.decayMarkerCostVia(mi.x(), mi.y(), mi.z(), MARKERDECAY)) {
      viaHistoryMarkers.erase(currIt);
    }
  }
  for (auto &marker: getMarkers()) {
    bool fixable = route_2_x2_addHistoryCost(marker);
    // skip non-fixable markers
    if (!fixable) {
      return;
    }
    route_2_x2_ripupNets(marker, net);
  }
  if (enableOutput) {
    cout <<"#viol = " <<getMarkers().size() <<endl;
  }
  for (auto &currNet: nets) {
    if (currNet->isRipup()) {
      if (enableOutput) {
        cout <<"ripup " <<currNet->getFrNet()->getName() <<endl;
      }
      route_2_pushNet(rerouteNets, currNet.get(), true, true);
      drcWorker.updateDRNet(currNet.get());
    }
  }
}
*/

/*
void FlexDRWorker::route_2_x1(drNet* net, deque<drNet*> &rerouteNets) {
  bool enableOutput = true;
  drcWorker.updateDRNet(net);
  drcWorker.check(net);
  setMarkers(drcWorker.getIncreViolations());
  route_2_x2(net, rerouteNets);
  if (enableOutput) {
    ;
  }

}
*/

/*
void FlexDRWorker::route_2() {
  bool enableOutput = true;
  //bool enableOutput = false;
  if (enableOutput) {
    cout << "start Maze route #nets = " <<nets.size() <<endl;
  }
  if (!DRCTEST && isEnableDRC() && getRipupMode() == 0 && getInitNumMarkers() == 0) {
    return;
  }
  if (DRCTEST) {
    DRCWorker drcWorker(getDesign(), fixedObjs);
    drcWorker.addDRNets(nets);
    using namespace std::chrono;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    drcWorker.init();
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    drcWorker.setup();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    //drcWorker.main();
    drcWorker.check();
    high_resolution_clock::time_point t3 = high_resolution_clock::now();
    drcWorker.report();

    duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);

    duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);

    duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
    stringstream ss;
    ss   <<"time (INIT/SETUP/MAIN) " <<time_span0.count() <<" "
                                     <<time_span1.count() <<" "
                                     <<time_span2.count() <<" "
                                     <<endl;
    cout <<ss.str() <<flush;


  } else {
    // queue of nets;
    deque<drNet*> rerouteNets;
    route_2_init(rerouteNets);
    // drcWorker
    drcWorker.setObjs(fixedObjs);
    drcWorker.addDRNets(nets);
    drcWorker.init();
    drcWorker.setup();
    // 0st iter ripupMode: 0 -- identify all drc net; 1 -- queue of all nets
    while (!rerouteNets.empty()) {
      // pop net
      auto net = route_2_popNet(rerouteNets);
      // nullptr net exceed mazeEndIter
      if (!net) {
        continue;
      }
      if (enableOutput) {
        cout <<"route " <<net->getFrNet()->getName() <<endl;
      }
      mazeNetInit(net);
      bool isRouted = routeNet(net);
      if (isRouted == false) {
        // TODO: output maze area
        cout << "Fatal error: Maze Route cannot find path (" << net->getFrNet()->getName() << "). Connectivity Changed.\n";
        if (OUT_MAZE_FILE != string("")) {
          gridGraph.print();
        }
        exit(1);
      }
      mazeNetEnd(net);
      // incr drc
      route_2_x1(net, rerouteNets);
      if (VERBOSE > 1) {
        ;
      }
    }
    if (VERBOSE > 1) {
      cout <<"#quick viol = " <<getNumQuickMarkers() <<endl;
    }
    route_drc();
    for (auto &net: nets) {
      net->setBestRouteConnFigs();
    }
    setBestMarkers();
  }
}
*/
// 复杂的网络路由处理流程，涉及多个步骤。
// 首先初始化与几何约束（GC）相关的工具和设置，然后处理线网的拆线重布队列，执行路由，再对路由结果进行修复和优化。
// 其中包括贴片线的处理，最后完成后处理步骤，如交换通孔以解决特定的布线违规问题。此外，还涉及多次调用GC工作器以确保设计符合DRC规范。
void FlexDRWorker::route_queue()
{ // 用于处理网络的队列布线
    // bool enableOutput = true;
    bool enableOutput = false; // 是否要使用输出功能
    // deque<pair<drNet*, int> > rerouteNets; // drNet*, #reroute pair
    // <drNet*, <isRoute, #reroute cnt> >// 初始化一个用于存储重新路由的网络队列。
    // <攻击/受害者，是否要重布，推入时的布线轮次>
    deque<pair<frBlockObject *, pair<bool, int>>> rerouteQueue; 
    // 如果设置了跳过路由，则直接返回。
    if (skipRouting)
    {
        return;
    }

    // init GC // 初始化几何约束工作器，设置扩展和DRC检查框。
    FlexGCWorker gcWorker(getDesign(), this);
    gcWorker.setExtBox(getExtBox());
    gcWorker.setDrcBox(getDrcBox());
    // if (needRecheck) {
    //   gcWorker.init();
    //   gcWorker.setEnableSurgicalFix(true);
    //   gcWorker.main();
    //   setMarkers(gcWorker.getMarkers());
    //   if (getDRIter() > 2 && gcWorker.getMarkers().size() == 0 && getRipupMode() == 0) {
    //     return;
    //   }
    // } else {
    //   if (getDRIter() > 2 && getInitNumMarkers() == 0 && getRipupMode() == 0) {
    //     return;
    //   }
    // }

    // if (!needRecheck) {// 初始化 GC 工作器，并启用手术式修复。
    gcWorker.init(); // 继续初始化工作 net seg pin ……
    gcWorker.setEnableSurgicalFix(true);
    // }

    setGCWorker(&gcWorker); // 将 GC 工作器设置给当前工作区。

    // init net status// 重置网络的撕裂状态 - 都变成0
    route_queue_resetRipup();
    // init marker cost// 添加标记成本到队列。
    route_queue_addMarkerCost();
    // init reroute queue
    // route_queue_init_queue(rerouteNets);
    // 初始化重新路由队列。
    route_queue_init_queue(rerouteQueue);

    // 如果启用输出，打印重新路由队列中的网络数量。
    if (enableOutput)
    {
        cout << "init. #nets in rerouteQueue = " << rerouteQueue.size() << "\n";
    }

    // route// 执行主要的路由函数。

    route_queue_main(rerouteQueue);
    // end// 结束路由，重置目标网络和启用手术式修复。
    gcWorker.resetTargetNet();
    gcWorker.setEnableSurgicalFix(true);
    gcWorker.main();

    // write back GC patches
    // 具体用于处理由gcWorker（几何约束工作器）提供的修正线（pwire，即Patch Wire）
    for (auto &pwire : gcWorker.getPWires())
    {                               // 遍历从几何约束工作器获取的所有修正线（Patch Wires）
        auto net = pwire->getNet(); // 获取与当前修正线关联的网络
        if (!net)
        {
            cout << "Error: pwire with no net\n";
            exit(1);
        }
        auto tmpPWire = make_unique<drPatchWire>();
        tmpPWire->setLayerNum(pwire->getLayerNum());
        frPoint origin;
        pwire->getOrigin(origin);
        tmpPWire->setOrigin(origin);
        frBox box;
        pwire->getOffsetBox(box);
        tmpPWire->setOffsetBox(box);
        tmpPWire->addToNet(net);

        unique_ptr<drConnFig> tmp(std::move(tmpPWire));
        auto &workerRegionQuery = getWorkerRegionQuery();
        workerRegionQuery.add(tmp.get());
        net->addRoute(tmp); // 将新的修正线添加到网络的路由列表中
    }

    // post route via swap (currently only for PA-related violation)// 如果迭代次数超过20，进行后处理，例如交换通孔。
    if (getDRIter() >= 20)
    {
        route_postRouteViaSwap_PA();
    }

    gcWorker.resetTargetNet(); // 重置目标网络和禁用手术式修复，然后再次执行几何约束工作器。
    gcWorker.setEnableSurgicalFix(false);
    gcWorker.main();

    gcWorker.end();

    setMarkers(gcWorker.getMarkers());

    for (auto &net : nets)
    {
        net->setBestRouteConnFigs();
    }
    setBestMarkers();
}

// 整个过程是控制布线队列处理的逻辑，从而完成电路设计的重新布线任务。
void FlexDRWorker::route_queue_main(deque<pair<frBlockObject *, pair<bool, int>>> &rerouteQueue)
{
    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象，用于管理设计中的空间查询
    // bool isRouteSkipped = true;
    //重布的线网数量
    auto numqueue = rerouteQueue.size();
    // int i=0;
    while (!rerouteQueue.empty())
    { // 处理队列直到为空
        // std::cout<<i++<<endl;
        // cout << "rerouteQueue size = " << rerouteQueue.size() << endl;
        auto [obj, statusPair] = rerouteQueue.front(); // 从队列前端获取对象，包括对象本身和其状态对
        bool doRoute = statusPair.first;               // 提取是否需要布线以及重布线尝试的次数
        int numReroute = statusPair.second;
        rerouteQueue.pop_front(); // 将对象从队列中移除
        bool didRoute = false;    // 标志位，用来检查是否完成了布线和检查
        bool didCheck = false;

        if (obj->typeId() == drcNet && doRoute)
        { // 仅当对象是网络且需要布线时才进行处理
            auto net = static_cast<drNet *>(obj);   //将右值转换为左值对象
            if (numReroute != net->getNumReroutes())
            { // 如果重布线计数与网络的计数不匹配则跳过布线
                // isRouteSkipped = true;
                // cout << "  skip route " << net->getFrNet()->getName() << "\n";
                continue;
            }
            // cout << "  do route " << net->getFrNet()->getName() << "\n";
            // isRouteSkipped = false;
            // init
            net->setModified(true); // 为布线做准备
            if (net->getFrNet())
            {
                net->getFrNet()->setModified(true);
            }
            net->setNumMarkers(0);
            for (auto &uConnFig : net->getRouteConnFigs())
            { // 移除现有的布线元素及其相关成本
                subPathCost(uConnFig.get());
                workerRegionQuery.remove(uConnFig.get()); // worker region query// 更新工作区域查询以反映更改
            }
            // route_queue need to unreserve via access if all nets are ripupped (i.e., not routed)
            // see route_queue_init_queue RESERVE_VIA_ACCESS
            // this is unreserve via
            // via is reserved only when drWorker starts from nothing and via is reserved
            // 如果需要，处理特殊情况，可能会解除通孔位置的保留
            if (RESERVE_VIA_ACCESS && net->getNumReroutes() == 0 && (getRipupMode() == 1 || getRipupMode() == 2))
            {
                initMazeCost_via_helper(net, false);
            }
            net->clear(); // 清除现有的布线数据，为新的布线做准备

            // route
            mazeNetInit(net); // 通过迷宫路由器对网络进行布线，先初始化
            bool isRouted = routeNet(net);
            if (isRouted == false)
            {
                frBox routeBox = getRouteBox(); // 处理致命的布线错误：未找到路径
                // TODO: output maze area
                cout << "Fatal error: Maze Route cannot find path (" << net->getFrNet()->getName() << ") in "
                     << "(" << routeBox.left() / 2000.0 << ", " << routeBox.bottom() / 2000.0
                     << ") - (" << routeBox.right() / 2000.0 << ", " << routeBox.top() / 2000.0
                     << "). Connectivity Changed.\n";
                if (OUT_MAZE_FILE == string(""))
                {
                    if (VERBOSE > 0)
                    {
                        cout << "Waring: no output maze log specified, skipped writing maze log" << endl;
                    }
                }
                else
                {
                    gridGraph.print(); // 可选地打印网格图以进行调试
                }
                exit(1);
            }
            mazeNetEnd(net); // 完成网络的布线
            net->addNumReroutes();
            didRoute = true;

            // if (routeBox.left() == 462000 && routeBox.bottom() == 81100) {
            //   cout << "@@@ debug net: " << net->getFrNet()->getName() << ", numPins = " << net->getNumPinsIn() << "\n";
            //   for (auto &uConnFig: net->getRouteConnFigs()) {
            //     if (uConnFig->typeId() == drcPathSeg) {
            //       auto ps = static_cast<drPathSeg*>(uConnFig.get());
            //       frPoint bp , ep;
            //       ps->getPoints(bp, ep);
            //       cout << "  pathseg: (" << bp.x() / 1000.0 << ", " << bp.y() / 1000.0 << ") - ("
            //                              << ep.x() / 1000.0 << ", " << ep.y() / 1000.0 << ") layerNum = "
            //                              << ps->getLayerNum() << "\n";
            //     }
            //   }
            //   // sanity check
            //   auto vptr = getDRNets(net->getFrNet());
            //   if (vptr) {
            //     bool isFound = false;
            //     for (auto dnet: *vptr) {
            //       if (dnet == net) {
            //         isFound = true;
            //       }
            //     }
            //     if (!isFound) {
            //       cout << "Error: drNet does not exist in frNet-to-drNets map\n";
            //     }
            //   }
            // }

            // gc// 如果网络被布线，执行几何约束检查
            if (gcWorker->setTargetNet(net->getFrNet()))
            {
                gcWorker->updateDRNet(net);
                gcWorker->setEnableSurgicalFix(true);
                gcWorker->main();

                // write back GC patches// 应用几何修正补丁
                for (auto &pwire : gcWorker->getPWires())
                {
                    auto net = pwire->getNet();
                    auto tmpPWire = make_unique<drPatchWire>();
                    tmpPWire->setLayerNum(pwire->getLayerNum());
                    frPoint origin;
                    pwire->getOrigin(origin);
                    tmpPWire->setOrigin(origin);
                    frBox box;
                    pwire->getOffsetBox(box);
                    tmpPWire->setOffsetBox(box);
                    tmpPWire->addToNet(net);

                    unique_ptr<drConnFig> tmp(std::move(tmpPWire));
                    auto &workerRegionQuery = getWorkerRegionQuery();
                    workerRegionQuery.add(tmp.get());
                    net->addRoute(tmp);
                }

                didCheck = true;
            }
            else
            {
                cout << "Error: fail to setTargetNet\n";
            }
        }
        else
        {
            // if (isRouteSkipped == false) { // 如果不进行布线，根据对象类型可能进行检查
            gcWorker->setEnableSurgicalFix(false);
            if (obj->typeId() == frcNet)
            {
                auto net = static_cast<frNet *>(obj);
                if (gcWorker->setTargetNet(net))
                {
                    gcWorker->main();
                    didCheck = true;
                    // cout << "do check " << net->getName() << "\n";
                }
            }
            else
            {
                if (gcWorker->setTargetNet(obj))
                {
                    gcWorker->main();
                    didCheck = true;
                    // cout << "do check\n";
                }
            }
            // }
        }

        // end
        if (didCheck)
        {
            route_queue_update_queue(gcWorker->getMarkers(), rerouteQueue);
        }
        if (didRoute)
        {
            route_queue_markerCostDecay();
        }
        if (didCheck)
        {
            route_queue_addMarkerCost(gcWorker->getMarkers());
        }
    }
}

/*

// void FlexDRWorker::route_queue_main(deque<pair<drNet*, int> > &rerouteNets,
//                                     FlexGCWorker *gcWorker) {
//   auto &workerRegionQuery = getWorkerRegionQuery();
//   while (!rerouteNets.empty()) {
//     auto [net, numReroute] = rerouteNets.front();
//     rerouteNets.pop_front();
//     if (numReroute != net->getNumReroutes()) {
//       continue;
//     }
//     // init
//     net->setModified(true);
//     net->setNumMarkers(0);
//     for (auto &uConnFig: net->getRouteConnFigs()) {
//       subPathCost(uConnFig.get());
//       workerRegionQuery.remove(uConnFig.get()); // worker region query
//     }
//     if (RESERVE_VIA_ACCESS) {
//       initMazeCost_via_helper(net, true);
//     }
//     net->clear();


//     // route
//     mazeNetInit(net);
//     bool isRouted = routeNet(net);
//     if (isRouted == false) {
//       // TODO: output maze area
//       cout << "Fatal error: Maze Route cannot find path (" << net->getFrNet()->getName() << "). Connectivity Changed.\n";
//       if (OUT_MAZE_FILE != string("")) {
//         if (VERBOSE > 0) {
//           cout <<"Waring: no output maze log specified, skipped writing maze log" <<endl;
//         }
//       } else {
//         gridGraph.print();
//       }
//       exit(1);
//     }
//     mazeNetEnd(net);
//     net->addNumReroutes();

//     // gc
//     if (gcWorker->setTargetNet(net->getFrNet())) {
//       gcWorker->updateDRNet(net);
//       gcWorker->main();
//     }

//     // end
//     route_queue_update_queue(gcWorker->getMarkers(), net, rerouteNets);
//     route_queue_markerCostDecay();
//     route_queue_addMarkerCost(gcWorker->getMarkers());
//   }
// }

// void FlexDRWorker::route_snr() {
//   bool enableOutput = false;
//   map<drNet*, int> drNet2Idx;
//   for (auto &net: nets) {
//     drNet2Idx[net.get()] = drNet2Idx.size();
//   }
//   // vector<int> drNetNumViols(drNet2Idx.size(), 0);
//   // vector<int> drNetDeltaVios(drNet2Idx.size(), 0);
//   auto localMarkers = markers;
//   vector<set<int> > drNet2ViolIdx(drNet2Idx.size());
//   vector<bool> isMarkerValid(localMarkers.size(), true);
//   vector<set<int> > viol2DrNetIdx(localMarkers.size());

//   // init mapping from net to markers and marker to nets
//   auto &workerRegionQuery = getWorkerRegionQuery();
//   for (int markerIdx = 0; markerIdx < (int)localMarkers.size(); markerIdx++) {
//     auto &marker = localMarkers[markerIdx];
//     vector<rq_rptr_value_t<drConnFig> > results;
//     auto lNum = marker.getLayerNum();
//     frBox mBox;
//     marker.getBBox(mBox);
//     workerRegionQuery.query(mBox, lNum, results);
//     for (auto &[boostB, connFig]: results) {
//       if (drNet2Idx.find(connFig->getNet()) == drNet2Idx.end()) {
//         cout << "Error: unknown net in route_snr\n";
//         continue;
//       }
//       auto netIdx = drNet2Idx(connFig->getNet());
//       drNet2ViolIdx[netIdx].insert(markerIdx);
//       viol2DrNetIdx[markerIdx].insert(netIdx);
//     }
//   }

//   // init for iteration
//   for (int i = 0; i < mazeNetInit; i++) {
//     FlexGCWorker gcWorker(getDesign(), this);
//     gcWorker.setExtBox(getExtBox());
//     gcWorker.setDrcBox(getDrcBox());
//     gcWorker.init();

//     init_snr_iter();

//   }


// }

// void FlexDRWorker::route_snr_net(drNet* net,
//                                  FlexGCWorker* gcWorker,
//                                  const map<drNet*, int> &drNet2Idx,
//                                  vector<frMarker> &markers,
//                                  vector<set<int> > &drNet2ViolIdx,
//                                  vector<bool> &isMarkerValid,
//                                  vector<set<int> > &viol2DrNetIdx,
//                                  priority_queue<int> &nextMarkerIdx) {
//   vector<int> victimMarkerIdx;
//   // preparation
//   route_snr_ripupNet(net, drNet2Idx, drNet2ViolIdx, isMarkerValid, viol2DrNetIdx, nextMarkerIdx, victimMarkerIdx);

//   // route net
//   bool isRouted = routeNet(net);
//   if (isRouted == false) {
//     cout << "Fatal error: Maze Route cannot find path (" << net->getFrNet()->getName() << "). Connectivity Changed.\n";
//     if (OUT_MAZE_FILE != string("")) {
//       gridGraph.print();
//     }
//     exit(1);
//   }

//   // end
//   mazeNetEnd(net);
//   route_snr_endNet_modMarkerCost(markers, victimMarkerIdx, false);  // sub resolved marker cost
//   // get violations caused by this net
//   vector<frMarker> newMarkers;
//   if (gcWorker->setTargetNet(net->getFrNet())) {
//     gcWorker->updateDRNet(net);
//     gcWorker->main();
//     for (auto &uMarker: gcWorker->getMarkers()) {
//       auto &marker = *uMarker;
//       frBox box;
//       marker.getBBox(box);
//       if (getDrcBox().overlaps(box)) {
//         newMarkers.push_back(marker)
//       }
//     }
//   }


// }

// void FlexDRWorker::route_snr_checkNet(drNet *net) {

// }

// void FlexDRWorker::route_snr_endNet_modMarkerCost(const vector<frMarker> &markers,
//                                                   const vector<int> &markerIdxs,
//                                                   bool isAddCost) {
//   for (auto markerIdx: markerIdxs) {
//     auto &marker = markers[markerIdx];
//     initMazeCost_marker_fixMode_9_addHistoryCost(marker, isAddCost);
//   }
// }

// void FlexDRWorker::route_snr_ripupNet(drNet* net,
//                                       const map<drNet*, int> &drNet2Idx,
//                                       vector<set<int> > drNet2ViolIdx,
//                                       vector<bool> $isMarkerValid,
//                                       vector<set<int> > &viol2DrNetIdx,
//                                       priority_queue<int> &nextMarkerIdx,
//                                       vector<int> &victimMarkerIdx) {
//   auto &workerRegionQuery = getWorkerRegionQuery();
//   for (auto &uConnFig: net->getRouteConnFigs()) {
//     subPathCost(uConnFig.get());
//     workerRegionQuery.remove(uConnFig.get());
//   }
//   if (RESERVE_VIA_ACCESS) {
//     initMazeCost_via_helper(net, true);
//   }
//   net->clear();
//   // vector<int> victimMarkerIdx;
//   for (auto markerIdx: drNet2ViolIdx[drNet2Idx[net]]) {
//     victimMarkerIdx.push_back(markerIdx);
//     for (auto netIdx: viol2DrNetIdx[markerIdx]) {
//       drNet2ViolIdx[netIdx].erase(markerIdx);
//     }
//   }
//   for (auto markerIdx: victimMarkerIdx) {
//     isMarkerValid[markerIdx] = false;
//     nextMarkerIdx.push(markerIdx);
//   }
// }


*/

void FlexDRWorker::route()
{
    // bool enableOutput = true;
    bool enableOutput = false; // 控制是否输出调试信息的标志，这里默认关闭
    if (enableOutput)
    {
        cout << "start Maze route #nets = " << nets.size() << endl; // 如果开启输出，打印开始布线的信息
    }
    // 如果不在测试模式且启用了DRC且ripup模式为0且初始标记数为0，则直接返回
    // DRCTEST默认传入false
    if (!DRCTEST && isEnableDRC() && getRipupMode() == 0 && getInitNumMarkers() == 0)
    {
        return;
    }
    // 如果是DRC测试模式
    if (DRCTEST)
    {
        // DRCWorker drcWorker(getDesign(), fixedObjs);
        // drcWorker.addDRNets(nets);
        using namespace std::chrono;
        // high_resolution_clock::time_point t0 = high_resolution_clock::now();
        // drcWorker.init();
        // high_resolution_clock::time_point t1 = high_resolution_clock::now();
        // drcWorker.setup();
        // high_resolution_clock::time_point t2 = high_resolution_clock::now();
        //// drcWorker.main();
        // drcWorker.check();
        // high_resolution_clock::time_point t3 = high_resolution_clock::now();
        // drcWorker.report();
        //
        // duration<double> time_span0 = duration_cast<duration<double>>(t1 - t0);

        // duration<double> time_span1 = duration_cast<duration<double>>(t2 - t1);

        // duration<double> time_span2 = duration_cast<duration<double>>(t3 - t2);
        // stringstream ss;
        // ss   <<"time (INIT/SETUP/MAIN) " <<time_span0.count() <<" "
        //                                  <<time_span1.count() <<" "
        //                                  <<time_span2.count() <<" "
        //                                  <<endl;
        // cout <<ss.str() <<flush;

        FlexGCWorker gcWorker(getDesign(), this);                             // 创建一个几何约束工作器实例
        gcWorker.setExtBox(getExtBox());                                      // 设置外部框
        gcWorker.setDrcBox(getDrcBox());                                      // 设置DRC检查框
        high_resolution_clock::time_point t0x = high_resolution_clock::now(); // 记录开始时间
        gcWorker.init();                                                      // 初始化几何约束工作器
        high_resolution_clock::time_point t1x = high_resolution_clock::now();
        gcWorker.main(); // 执行主要的检查任务
        high_resolution_clock::time_point t2x = high_resolution_clock::now();
        // drcWorker.main();
        gcWorker.end();                    // 结束几何约束工作器的任务
        setMarkers(gcWorker.getMarkers()); // 设置标记

        high_resolution_clock::time_point t3x = high_resolution_clock::now();
        // drcWorker.report();
        //  计算和打印时间间隔
        duration<double> time_span0x = duration_cast<duration<double>>(t1x - t0x);

        duration<double> time_span1x = duration_cast<duration<double>>(t2x - t1x);

        duration<double> time_span2x = duration_cast<duration<double>>(t3x - t2x);

        if (VERBOSE > 1)
        {
            stringstream ss;
            ss << "GC  (INIT/MAIN/END) " << time_span0x.count() << " "
               << time_span1x.count() << " "
               << time_span2x.count() << " "
               << endl;
            ss << "#viol = " << markers.size() << endl;
            cout << ss.str() << flush;
        }
    }
    else
    {
        // FlexGCWorker gcWorker(getDesign(), this);
        // gcWorker.setExtBox(getExtBox());
        // gcWorker.setDrcBox(getDrcBox());
        // gcWorker.init();
        // gcWorker.main();

        // setMarkers(gcWorker.getMarkers());

        vector<drNet *> rerouteNets; // 存储需要重新布线的网络列表
        for (int i = 0; i < mazeEndIter; ++i)
        { // 进行多次迭代
            if (!mazeIterInit(i, rerouteNets))
            { // 初始化迭代，如果失败则返回
                return;
            }
            // minAreaVios.clear();
            // if (i == 0) {
            //   workerDRCCost    = DRCCOST;
            //   workerMarkerCost = MARKERCOST;
            // } else {
            //   workerDRCCost *= 2;
            //   workerMarkerCost *= 2;
            // }
            for (auto net : rerouteNets)
            { // 对每个需要重新布线的网络
                // for (auto &pin: net->getPins()) {
                //   for (auto &ap: )
                mazeNetInit(net);              // 初始化网络的布线
                bool isRouted = routeNet(net); // 进行网络布线
                if (isRouted == false)
                { // 如果布线失败
                    // TODO: output maze area
                    cout << "Fatal error: Maze Route cannot find path (" << net->getFrNet()->getName() << ") in "
                         << "(" << routeBox.left() / 2000.0 << ", " << routeBox.bottom() / 2000.0
                         << ") - (" << routeBox.right() / 2000.0 << ", " << routeBox.top() / 2000.0
                         << "). Connectivity Changed.\n";
                    cout << "#local pin = " << net->getPins().size() << endl;
                    for (auto &pin : net->getPins())
                    { // 遍历网络中的所有引脚，每个引脚都是`net`对象的`getPins()`方法返回的集合中的一部分。
                        if (pin->hasFrTerm())
                        { // 检查当前引脚是否有关联的 'frTerm'。'frTerm' 通常代表引脚上的一个功能性接口，比如电子设计中的连接点。

                            if (pin->getFrTerm()->typeId() == frcInstTerm)
                            {   // 如果 'frTerm' 的类型是 'frcInstTerm'，这表示该引脚连接到一个实例终端。
                                // 'frcInstTerm' 通常用于表示电路设计中组件实例化部分的终端。

                                auto instTerm = static_cast<frInstTerm *>(pin->getFrTerm()); // 安全地将 'frTerm' 强制转换为 'frInstTerm' 类型，以访问适用于实例终端的特定方法和属性。

                                cout << "  instTerm " << instTerm->getInst()->getName() << "/" << instTerm->getTerm()->getName() << "\n";
                                // 输出连接到此引脚的实例名称和终端名称。
                                // 'instTerm->getInst()->getName()' 获取实例的名称，
                                // 'instTerm->getTerm()->getName()' 获取该实例内的终端名称。
                            }
                            else
                            {
                                // 如果 'frTerm' 不是类型 'frcInstTerm'，则将其视为普通终端。
                                cout << "  term\n";
                                // 输出 "term" 表示这个引脚有一个终端，但它不是特定于实例的终端。
                            }
                        }
                        else
                        {
                            // 如果引脚没有关联的 'frTerm'，这意味着可能是用于设计边界或特殊用途的引脚。
                            cout << "  boundary pin\n";
                            // 输出 "boundary pin" 表示这个引脚可能用于边界或外部连接，
                            // 这些连接不直接连接到电路组件的标准功能终端。
                        }
                    }
                    if (OUT_MAZE_FILE == string(""))
                    {
                        if (VERBOSE > 0)
                        {
                            cout << "Waring: no output maze log specified, skipped writing maze log" << endl;
                        }
                    }
                    else
                    {
                        gridGraph.print();
                    }
                    exit(1);
                }
                mazeNetEnd(net); // 结束网络布线
            }
            // drc worker here// 检查并保存最佳的DRC结果
            if (!rerouteNets.empty() && isEnableDRC())
            {
                route_drc(); // 进行DRC检查
                // route_postRouteViaSwap(); // blindly swap via cause more violations in tsmc65lp_aes
            }

            // quick drc // 检查是否有违规，并决定是否结束迭代
            int violNum = getNumQuickMarkers(); // 获取快速标记数
            if (VERBOSE > 1)
            {
                cout << "#quick viol = " << getNumQuickMarkers() << endl; // 如果启用了详细输出，则打印违规数
            }

            // save to best drc
            // if (i == 0 || (isEnableDRC() && getMarkers().size() < getBestMarkers().size())) {
            if (i == 0 || (isEnableDRC() && (getMarkers().size() < getBestMarkers().size() || workerMarkerBloatWidth > 0)))
            {
                // if (true) {
                for (auto &net : nets)
                {
                    net->setBestRouteConnFigs();
                }
                // double dbu = getDesign()->getTopBlock()->getDBUPerUU();
                // if (getRouteBox().left()    == 63     * dbu &&
                //     getRouteBox().right()   == 84     * dbu &&
                //     getRouteBox().bottom()  == 139.65 * dbu &&
                //     getRouteBox().top()     == 159.6  * dbu) {
                //   for (auto &net: nets) {
                //     if (net->getFrNet()->getName() == string("net144") || net->getFrNet()->getName() == string("net221")) {
                //       cout <<net->getFrNet()->getName() <<": " <<endl;
                //       cout <<"routeConnFigs" <<endl;
                //       for (auto &uConnFig: net->getRouteConnFigs()) {
                //         if (uConnFig->typeId() == drcPathSeg) {
                //           auto obj = static_cast<drPathSeg*>(uConnFig.get());
                //           frPoint bp, ep;
                //           obj->getPoints(bp, ep);
                //           cout <<"  ps ("
                //                <<bp.x() / dbu <<", " <<bp.y() / dbu <<") ("
                //                <<ep.x() / dbu <<", " <<ep.y() / dbu <<") "
                //                <<getDesign()->getTech()->getLayer(obj->getLayerNum())->getName()
                //                <<endl;

                //        } else if (uConnFig->typeId() == drcVia) {
                //          auto obj = static_cast<drVia*>(uConnFig.get());
                //          frPoint pt;
                //          obj->getOrigin(pt);
                //          cout <<"  via ("
                //               <<pt.x() / dbu <<", " <<pt.y() / dbu <<") "
                //               <<obj->getViaDef()->getName()
                //               <<endl;
                //        } else if (uConnFig->typeId() == drcPatchWire) {
                //          auto obj = static_cast<drPatchWire*>(uConnFig.get());
                //          frPoint pt;
                //          obj->getOrigin(pt);
                //          frBox offsetBox;
                //          obj->getOffsetBox(offsetBox);
                //          cout <<"  pWire (" <<pt.x() / dbu <<", " <<pt.y() / dbu <<") RECT ("
                //               <<offsetBox.left()   / dbu <<" "
                //               <<offsetBox.bottom() / dbu <<" "
                //               <<offsetBox.right()  / dbu <<" "
                //               <<offsetBox.top()    / dbu <<") "
                //               <<getDesign()->getTech()->getLayer(obj->getLayerNum())->getName()
                //               <<endl;
                //        }
                //      }
                //      cout <<"extConnFigs" <<endl;
                //      for (auto &uConnFig: net->getExtConnFigs()) {
                //        if (uConnFig->typeId() == drcPathSeg) {
                //          auto obj = static_cast<drPathSeg*>(uConnFig.get());
                //          frPoint bp, ep;
                //          obj->getPoints(bp, ep);
                //          cout <<"  ps ("
                //               <<bp.x() / dbu <<", " <<bp.y() / dbu <<") ("
                //               <<ep.x() / dbu <<", " <<ep.y() / dbu <<") "
                //               <<getDesign()->getTech()->getLayer(obj->getLayerNum())->getName()
                //               <<endl;

                //        } else if (uConnFig->typeId() == drcVia) {
                //          auto obj = static_cast<drVia*>(uConnFig.get());
                //          frPoint pt;
                //          obj->getOrigin(pt);
                //          cout <<"  via ("
                //               <<pt.x() / dbu <<", " <<pt.y() / dbu <<") "
                //               <<obj->getViaDef()->getName()
                //               <<endl;
                //        } else if (uConnFig->typeId() == drcPatchWire) {
                //          auto obj = static_cast<drPatchWire*>(uConnFig.get());
                //          frPoint pt;
                //          obj->getOrigin(pt);
                //          frBox offsetBox;
                //          obj->getOffsetBox(offsetBox);
                //          cout <<"  pWire (" <<pt.x() / dbu <<", " <<pt.y() / dbu <<") RECT ("
                //               <<offsetBox.left()   / dbu <<" "
                //               <<offsetBox.bottom() / dbu <<" "
                //               <<offsetBox.right()  / dbu <<" "
                //               <<offsetBox.top()    / dbu <<") "
                //               <<getDesign()->getTech()->getLayer(obj->getLayerNum())->getName()
                //               <<endl;
                //        }
                //      }
                //    }
                //  }
                //}
                setBestMarkers();
                if (VERBOSE > 1 && i > 0)
                {
                    cout << "best iter = " << i << endl;
                }
            }

            if (isEnableDRC() && getMarkers().empty())
            { // 如果启用了DRC并且没有标记，结束迭代
                break;
            }
            else if (!isEnableDRC() && violNum == 0)
            { // 如果未启用DRC并且没有违规，结束迭代
                break;
            }
        }
    }
}

// 这个函数用于布线准备阶段，设置未连接的引脚、访问模式的迷宫索引，并更新迷宫图中的目标点。
void FlexDRWorker::routeNet_prep(drNet *net, set<drPin *, frBlockObjectComp> &unConnPins,
                                 map<FlexMazeIdx, set<drPin *, frBlockObjectComp>> &mazeIdx2unConnPins,
                                 set<FlexMazeIdx> &apMazeIdx,
                                 set<FlexMazeIdx> &realPinAPMazeIdx /*,
                                  map<FlexMazeIdx, frViaDef*> &apSVia*/
)
{
    // bool enableOutput = true;
    bool enableOutput = false; // 控制是否输出调试信息的标志，这里默认关闭
    for (auto &pin : net->getPins())
    { // 遍历网络中的所有引脚
        if (enableOutput)
        {
            cout << "pin set target@"; // 如果启用输出，打印引脚设置目标的前缀
        }
        unConnPins.insert(pin.get()); // 将引脚插入到未连接引脚集合中
        for (auto &ap : pin->getAccessPatterns())
        {                                             // 遍历引脚的所有访问模式
            FlexMazeIdx mi;                           // 创建一个迷宫索引变量
            ap->getMazeIdx(mi);                       // 从访问模式中获取迷宫索引
            mazeIdx2unConnPins[mi].insert(pin.get()); // 将引脚插入到对应迷宫索引的未连接引脚集合中
            if (pin->hasFrTerm())
            {                                // 如果引脚有功能术语（即连接到实际电路元件的引脚）
                realPinAPMazeIdx.insert(mi); // 将此迷宫索引插入到实际引脚访问模式的迷宫索引集合中

                // if (net->getFrNet()->getName() == string("pci_devsel_oe_o")) {
                //   cout <<"apMazeIdx (" <<mi.x() <<", " <<mi.y() <<", " <<mi.z() <<")\n";
                //   auto routeBox = getRouteBox();
                //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
                //   std::cout <<"routeBox (" <<routeBox.left() / dbu <<", " <<routeBox.bottom() / dbu <<") ("
                //                            <<routeBox.right()/ dbu <<", " <<routeBox.top()    / dbu <<")" <<std::endl;
                // }
            }
            apMazeIdx.insert(mi); // 将迷宫索引插入到访问模式迷宫索引集合中
            gridGraph.setDst(mi); // 在网格图中设置此迷宫索引为目标点
            if (enableOutput)
            {
                cout << " (" << mi.x() << ", " << mi.y() << ", " << mi.z() << ")";
            }
        }
        if (enableOutput)
        {
            cout << endl;
        }
    }
}

// 该函数 routeNet_setSrc 主要用于选择源引脚并初始化连接组件。这是在详细布线过程中，设置起始点为布线算法的准备步骤。
void FlexDRWorker::routeNet_setSrc(set<drPin *, frBlockObjectComp> &unConnPins,
                                   map<FlexMazeIdx, set<drPin *, frBlockObjectComp>> &mazeIdx2unConnPins,
                                   vector<FlexMazeIdx> &connComps,
                                   FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2, frPoint &centerPt)
{
    frMIdx xDim, yDim, zDim;
    gridGraph.getDim(xDim, yDim, zDim);           // 获取网格图的维度
    ccMazeIdx1.set(xDim - 1, yDim - 1, zDim - 1); // 初始化连接组件的最大范围索引
    ccMazeIdx2.set(0, 0, 0);                      // 初始化连接组件的最小范围索引
    // first pin selection algorithm goes here
    // choose the center pin
    // 初始化选择源引脚算法
    centerPt.set(0, 0); // 初始化中心点坐标
    int totAPCnt = 0;   // 初始化总的访问模式计数
    frCoord totX = 0;   // 总的x坐标
    frCoord totY = 0;   // 总的y坐标
    frCoord totZ = 0;   // 总的z坐标
    FlexMazeIdx mi;     // 迷宫索引变量
    frPoint bp;         // 基本点变量
    for (auto &pin : unConnPins)
    { // 遍历未连接的引脚
        for (auto &ap : pin->getAccessPatterns())
        {                                                               // 遍历引脚的访问模式
            ap->getMazeIdx(mi);                                         // 获取访问模式的迷宫索引
            ap->getPoint(bp);                                           // 获取访问模式的点坐标
            totX += bp.x();                                             // 累加x坐标
            totY += bp.y();                                             // 累加y坐标
            centerPt.set(centerPt.x() + bp.x(), centerPt.y() + bp.y()); // 更新中心点坐标
            totZ += gridGraph.getZHeight(mi.z());                       // 累加z坐标高度
            totAPCnt++;                                                 // 增加访问模式计数
            break;                                                      // 只处理每个引脚的第一个访问模式
        }
    }
    totX /= totAPCnt;                                               // 计算x坐标平均值
    totY /= totAPCnt;                                               // 计算y坐标平均值
    totZ /= totAPCnt;                                               // 计算z坐标平均值
    centerPt.set(centerPt.x() / totAPCnt, centerPt.y() / totAPCnt); // 计算中心点坐标

    // frCoord currDist = std::numeric_limits<frCoord>::max();
    //  select the farmost pin

    drPin *currPin = nullptr; // 当前选中的引脚
    // if (unConnPins.size() == 2) {
    //   int minAPCnt = std::numeric_limits<int>::max();
    //   for (auto &pin: unConnPins) {
    //     if (int(pin->getAccessPatterns().size()) < minAPCnt) {
    //       currPin = pin;
    //       minAPCnt = int(pin->getAccessPatterns().size());
    //     }
    //   }
    // } else {

    // if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
    //   map<frCoord, set<int> > apYCoord2PinIdx;
    //   int pinIdx = 0;
    //   for (auto &pin: unConnPins) {
    //     for (auto &ap: pin->getAccessPatterns()) {
    //       ap->getMazeIdx(mi);
    //       ap->getPoint(bp);
    //       if (apYCoord2PinIdx.find(bp.y()) != apYCoord2PinIdx.end() && apYCoord2PinIdx[bp.y()].find(pinIdx) == apYCoord2PinIdx[bp.y()].end()) {
    //         currPin = pin;
    //         break;
    //       }
    //       apYCoord2PinIdx[bp.y()].insert(pinIdx);
    //     }
    //     if (currPin) {
    //       break;
    //     }
    //     pinIdx++;
    //   }
    // }

    // // first try non-boundary pins
    // if (getRipupMode() == 0 && getDRIter() >= 20 && getDRIter() % 3 == 0 && !currPin) {
    //   frCoord currDist = 0;
    //   for (auto &pin: unConnPins) {
    //     if (pin->hasFrTerm() == false) {
    //       continue;
    //     }
    //     for (auto &ap: pin->getAccessPatterns()) {
    //       ap->getMazeIdx(mi);
    //       ap->getPoint(bp);
    //       frCoord dist = abs(totX - bp.x()) + abs(totY - bp.y()) + abs(totZ - gridGraph.getZHeight(mi.z()));
    //       if (dist >= currDist) {
    //         currDist = dist;
    //         currPin  = pin;
    //       }
    //     }
    //   }
    // }

    if (!currPin)
    {
        frCoord currDist = 0; // 当前距离，用于选择最远引脚
        for (auto &pin : unConnPins)
        { // 遍历未连接的引脚
            for (auto &ap : pin->getAccessPatterns())
            {                                                                                                      // 遍历引脚的访问模式
                ap->getMazeIdx(mi);                                                                                // 获取迷宫索引
                ap->getPoint(bp);                                                                                  // 获取点坐标
                frCoord dist = abs(totX - bp.x()) + abs(totY - bp.y()) + abs(totZ - gridGraph.getZHeight(mi.z())); // 计算距离
                if (dist >= currDist)
                {                    // 如果当前距离大于已知的最大距离
                    currDist = dist; // 更新最大距离
                    currPin = pin;   // 更新当前引脚
                }
            }
        }
    }
    // }

    // if (currPin->hasFrTerm()) {
    //   if (currPin->getFrTerm()->typeId() == frcTerm) {
    //     cout << "src pin (frTermName) = " << ((frTerm*)currPin->getFrTerm())->getName() << "\n";
    //   } else {
    //     cout << "src pin (frTermName) = " << ((frInstTerm*)currPin->getFrTerm())->getInst()->getName() << "/" << ((frInstTerm*)currPin->getFrTerm())->getTerm()->getName() << "\n";
    //   }
    // } else {
    //   cout << "src pin is boundary pin\n";
    // }
    // int apCnt = 0;
    // for (auto &ap: currPin->getAccessPatterns()) {
    //  auto apPtr = ap.get();
    //  //auto mazeIdx = apPtr->getMazeIdx();
    //  frPoint bp;
    //  apPtr->getPoint(bp);
    //  // cout << "  ap" << apCnt << " has MazeIdx (" << mazeIdx.x() << ", " << mazeIdx.y() << ", " << mazeIdx.z() << "), real coord = ("
    //  //      << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
    //  ++apCnt;
    //}

    unConnPins.erase(currPin); // 从未连接引脚集合中移除当前引脚

    // auto currPin = *(unConnPins.begin());
    // unConnPins.erase(unConnPins.begin());
    //  first pin selection algorithm ends here
    for (auto &ap : currPin->getAccessPatterns())
    {                                               // 遍历当前引脚的访问模式
        ap->getMazeIdx(mi);                         // 获取迷宫索引
        connComps.push_back(mi);                    // 将迷宫索引加入到连接组件集合
        ccMazeIdx1.set(min(ccMazeIdx1.x(), mi.x()), // 更新连接组件的最小范围索引
                       min(ccMazeIdx1.y(), mi.y()),
                       min(ccMazeIdx1.z(), mi.z()));
        ccMazeIdx2.set(max(ccMazeIdx2.x(), mi.x()), // 更新连接组件的最大范围索引
                       max(ccMazeIdx2.y(), mi.y()),
                       max(ccMazeIdx2.z(), mi.z()));
        //it = set<fr::drPin * , fr:: frBlockObjectComp>
        auto it = mazeIdx2unConnPins.find(mi); // 查找当前迷宫索引在未连接引脚的映射中的位置
        if (it == mazeIdx2unConnPins.end()) // 如果未找到，继续下一循环
        {
            continue;
        }
        auto it2 = it->second.find(currPin); // 在未连接引脚集合中查找当前引脚
        if (it2 == it->second.end())        // 如果未找到，继续下一循环
        {
            continue;
        }
        it->second.erase(it2); // 从集合中移除当前引脚

        gridGraph.setSrc(mi); // 在网格图中设置当前迷宫索引为源点
        // remove dst label only when no other pins share the same loc
        if (it->second.empty())
        {                                 // 如果当前迷宫索引对应的未连接引脚集合为空
            mazeIdx2unConnPins.erase(it); // 从映射中移除当前迷宫索引
            gridGraph.resetDst(mi);       // 重置网格图中的目标点
        }
    }
}
// 该函数 routeNet_getNextDst 是用于从给定的未连接引脚集合中找出最佳的下一个目标引脚进行布线。
// 这个选择基于引脚位置与当前关注区域（由 ccMazeIdx1 和 ccMazeIdx2 确定的边界框）的距离。
drPin *FlexDRWorker::routeNet_getNextDst(FlexMazeIdx &ccMazeIdx1, FlexMazeIdx &ccMazeIdx2,
                                         map<FlexMazeIdx, set<drPin *, frBlockObjectComp>> &mazeIdx2unConnPins)
{
    frPoint pt;                                             // 用来临时存储点坐标的变量。
    frPoint ll, ur;                                         // 分别代表边界框的左下角和右上角的坐标点。
    gridGraph.getPoint(ll, ccMazeIdx1.x(), ccMazeIdx1.y()); // 获取边界框左下角的坐标。
    gridGraph.getPoint(ur, ccMazeIdx2.x(), ccMazeIdx2.y()); // 获取边界框右上角的坐标。
    frCoord currDist = std::numeric_limits<frCoord>::max(); // 初始化当前距离为最大值，用于找出最近的引脚。
    drPin *nextDst = nullptr;                               // 初始化下一个目标引脚为nullptr。

    // for (auto &[mazeIdx, setS]: mazeIdx2unConnPins) {
    //   gridGraph.getPoint(pt, mazeIdx.x(), mazeIdx.y());
    //   if (pt.y() >= ll.y() && pt.y() <= ur.y()) {
    //     nextDst = *(setS.begin());
    //     break;
    //   }
    // }
    // 如果之前的代码段（被注释掉的部分）没有找到合适的目标引脚
    if (!nextDst)   // 如果目标引脚为NULL
        for (auto &[mazeIdx, setS] : mazeIdx2unConnPins)
        {                                                               // 遍历所有未连接引脚的迷宫索引和对应的引脚集合
            gridGraph.getPoint(pt, mazeIdx.x(), mazeIdx.y());           // 获取当前迷宫索引对应的点坐标。
            frCoord dx = max(max(ll.x() - pt.x(), pt.x() - ur.x()), 0); // 计算x方向的距离差。
            frCoord dy = max(max(ll.y() - pt.y(), pt.y() - ur.y()), 0); // 计算y方向的距离差。
            frCoord dz = max(max(gridGraph.getZHeight(ccMazeIdx1.z()) - gridGraph.getZHeight(mazeIdx.z()),
                                 gridGraph.getZHeight(mazeIdx.z()) - gridGraph.getZHeight(ccMazeIdx2.z())),
                             0); // 计算z方向的高度差。
            
            // if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
            //   if (dx + MISALIGNMENTCOST * dy + dz < currDist) {
            //     currDist = dx + MISALIGNMENTCOST * dy + dz;
            //     nextDst = *(setS.begin());
            //   }
            // } else {
            if (dx + dy + dz < currDist)
            {                              // 如果当前引脚到边界框的总距离小于已知的最小距离
                currDist = dx + dy + dz;   // 更新最小距离
                nextDst = *(setS.begin()); // 设置当前引脚为下一个目标引脚
            }
            // }
            if (currDist == 0)
            { // 如果距离为0，即找到了边界框内的引脚
                break;
            }
        }
    return nextDst; // 返回找到的下一个目标引脚
}

void FlexDRWorker::mazePinInit()
{
    gridGraph.resetAStarCosts();  // 重置迷宫图中的A*算法的成本值，为新的搜索准备
    gridGraph.resetPrevNodeDir(); // 重置迷宫图中每个节点的前驱节点方向，用于新的路径搜索
}
// 函数 routeNet_postAstarUpdate 主要用于在完成 A* 算法后更新布线路径的状态，特别是处理路径中的点和更新未连接引脚的状态。
void FlexDRWorker::routeNet_postAstarUpdate(vector<FlexMazeIdx> &path, vector<FlexMazeIdx> &connComps,
                                            set<drPin *, frBlockObjectComp> &unConnPins,
                                            map<FlexMazeIdx, set<drPin *, frBlockObjectComp>> &mazeIdx2unConnPins,
                                            bool isFirstConn)
{
    // first point is dst// 首先检查路径是否为空
    set<FlexMazeIdx> localConnComps; // 本地连接组件索引集合
    if (!path.empty())
    {
        auto mi = path[0];       // 获取路径的第一个点，通常是目标点
        vector<drPin *> tmpPins; // 临时存储要处理的引脚
        for (auto pin : mazeIdx2unConnPins[mi])
        { // 遍历在第一个迷宫索引处的所有未连接引脚
            // unConnPins.erase(pin);
            tmpPins.push_back(pin); // 将引脚添加到临时向量
        }
        for (auto pin : tmpPins)
        {                          // 处理每个临时存储的引脚
            unConnPins.erase(pin); // 从未连接引脚集合中移除该引脚
            for (auto &ap : pin->getAccessPatterns())
            { // 遍历引脚的所有访问模式
                FlexMazeIdx mi;
                ap->getMazeIdx(mi);                    // 获取访问模式的迷宫索引
                auto it = mazeIdx2unConnPins.find(mi); // 在未连接引脚映射中找到该迷宫索引
                if (it == mazeIdx2unConnPins.end())
                {
                    continue; // 如果未找到，则继续下一循环
                }
                auto it2 = it->second.find(pin); // 在对应的引脚集合中找到当前引脚
                if (it2 == it->second.end())
                {
                    continue; // 如果未找到，则继续下一循环
                }
                it->second.erase(it2); // 从集合中移除引脚
                if (it->second.empty())
                {                                 // 如果当前迷宫索引下的引脚集合为空
                    mazeIdx2unConnPins.erase(it); // 从映射中移除该迷宫索引
                    gridGraph.resetDst(mi);       // 重置对应迷宫索引的目标状态
                }
                if (ALLOW_PIN_AS_FEEDTHROUGH)
                {                              // 如果允许引脚作为走线通道
                    localConnComps.insert(mi); // 将迷宫索引添加到本地连接组件集合
                    gridGraph.setSrc(mi);      // 设置迷宫索引为起点
                }
            }
        }
    }
    else
    {
        cout << "Error: routeNet_postAstarUpdate path is empty" << endl; // 如果路径为空，则输出错误信息
    }
    // must be before comment line ABC so that the used actual src is set in gridgraph
    if (isFirstConn && (!ALLOW_PIN_AS_FEEDTHROUGH))
    { // 如果是第一次连接并且不允许引脚作为走线通道
        for (auto &mi : connComps)
        {
            gridGraph.resetSrc(mi); // 重置所有连接组件索引的起点状态
        }
        connComps.clear(); // 清空连接组件索引集合
        if ((int)path.size() == 1)
        {                                 // 如果路径长度为1
            connComps.push_back(path[0]); // 将路径的唯一索引添加到连接组件集合
            gridGraph.setSrc(path[0]);    // 设置该索引为起点
        }
    }
    // line ABC
    // must have >0 length
    // 处理路径中除起点外的所有点
    for (int i = 0; i < (int)path.size() - 1; ++i)
    {
        auto start = path[i];   // 起始点
        auto end = path[i + 1]; // 终点
        auto startX = start.x(), startY = start.y(), startZ = start.z();
        auto endX = end.x(), endY = end.y(), endZ = end.z();
        // horizontal wire// 处理水平线
        if (startX != endX && startY == endY && startZ == endZ)
        {
            for (auto currX = std::min(startX, endX); currX <= std::max(startX, endX); ++currX)
            {
                localConnComps.insert(FlexMazeIdx(currX, startY, startZ)); // 添加到本地连接组件集合
                gridGraph.setSrc(currX, startY, startZ);                   // 设置为起点
                                                                           // gridGraph.resetDst(currX, startY, startZ);
            }
            // vertical wire// 处理垂直线
        }
        else if (startX == endX && startY != endY && startZ == endZ)
        {
            for (auto currY = std::min(startY, endY); currY <= std::max(startY, endY); ++currY)
            {
                localConnComps.insert(FlexMazeIdx(startX, currY, startZ)); // 添加到本地连接组件集合
                gridGraph.setSrc(startX, currY, startZ);                   // 设置为起点
                                                                           // gridGraph.resetDst(startX, currY, startZ);
            }
            // via// 处理通孔
        }
        else if (startX == endX && startY == endY && startZ != endZ)
        {
            for (auto currZ = std::min(startZ, endZ); currZ <= std::max(startZ, endZ); ++currZ)
            {
                localConnComps.insert(FlexMazeIdx(startX, startY, currZ)); // 添加到本地连接组件集合
                gridGraph.setSrc(startX, startY, currZ);                   // 设置为起点
                                                                           // gridGraph.resetDst(startX, startY, currZ);
            }
            // zero length // 处理零长度路径
        }
        else if (startX == endX && startY == endY && startZ == endZ)
        {
            // root.addPinGrid(startX, startY, startZ);
            std::cout << "Warning: zero-length path in updateFlexPin\n"; // 输出零长度路径警告
        }
        else
        {
            std::cout << "Error: non-colinear path in updateFlexPin\n"; // 输出路径不连续错误
        }
    }
    // 更新连接组件索引集合
    for (auto &mi : localConnComps)
    {
        if (isFirstConn && !ALLOW_PIN_AS_FEEDTHROUGH)
        {
            connComps.push_back(mi); // 添加到连接组件集合
        }
        else
        {
            if (!(mi == *(path.cbegin())))
            {
                connComps.push_back(mi); // 添加到连接组件集合
            }
        }
    }
}
// 主要用于在 A* 路径搜索后将路径写入实际的物理设计中，该函数处理路径点，生成布线路径并添加到网络中。
void FlexDRWorker::routeNet_postAstarWritePath(drNet *net, vector<FlexMazeIdx> &points,
                                               const set<FlexMazeIdx> &apMazeIdx /*,
                                                const map<FlexMazeIdx, frViaDef*> &apSVia*/
)
{
    // bool enableOutput = true;
    bool enableOutput = false; // 控制调试输出的开关，默认关闭
    if (points.empty())
    { // 检查路径点是否为空
        if (enableOutput)
        {                                                             // 如果启用了输出
            std::cout << "Warning: path is empty in writeMazePath\n"; // 打印路径为空的警告信息
        }
        return; // 由于路径为空，函数提前返回
    }
    if (TEST && points.size())
    {                   // 如果是测试模式且路径点不为空
        cout << "path"; // 输出路径开始
        for (auto &mIdx : points)
        {                                                                            // 遍历路径点
            cout << " (" << mIdx.x() << ", " << mIdx.y() << ", " << mIdx.z() << ")"; // 打印每个路径点的坐标
        }
        cout << endl; // 输出换行符
    }
    auto &workerRegionQuery = getWorkerRegionQuery(); // 获取工作区域查询对象，用于添加和查询设计区域内的元素
    for (int i = 0; i < (int)points.size() - 1; ++i)
    {                           // 遍历路径点，除了最后一个点
        FlexMazeIdx start, end; // 定义起始和结束索引
        if (points[i + 1] < points[i])
        { // 确定路径的方向，使得 start 总是指向较小的索引
            start = points[i + 1];
            end = points[i];
        }
        else
        {
            start = points[i];
            end = points[i + 1];
        }
        auto startX = start.x(), startY = start.y(), startZ = start.z();
        auto endX = end.x(), endY = end.y(), endZ = end.z();
        // horizontal wire
        if (startX != endX && startY == endY && startZ == endZ)
        {
            frPoint startLoc, endLoc;
            frLayerNum currLayerNum = gridGraph.getLayerNum(startZ);
            gridGraph.getPoint(startLoc, startX, startY);
            gridGraph.getPoint(endLoc, endX, endY);
            auto currPathSeg = make_unique<drPathSeg>();
            currPathSeg->setPoints(startLoc, endLoc);
            currPathSeg->setLayerNum(currLayerNum);
            currPathSeg->addToNet(net);
            auto currStyle = getTech()->getLayer(currLayerNum)->getDefaultSegStyle();
            if (apMazeIdx.find(start) != apMazeIdx.end())
            {
                currStyle.setBeginStyle(frcTruncateEndStyle, 0);
            }
            if (apMazeIdx.find(end) != apMazeIdx.end())
            {
                currStyle.setEndStyle(frcTruncateEndStyle, 0);
            }
            currPathSeg->setStyle(currStyle);
            currPathSeg->setMazeIdx(start, end);
            unique_ptr<drConnFig> tmp(std::move(currPathSeg));
            workerRegionQuery.add(tmp.get());
            net->addRoute(tmp);
            // if (/*startLoc.x() == 1834400 && */startLoc.y() == 2095500 && currLayerNum == 6 && net->getFrNet()->getName() == string("net74729")) {
            //   if (currStyle.getBeginStyle() == frEndStyle(frcTruncateEndStyle)) {
            //     cout << "@@@ DEBUG: begin point has 0 ext at x = " << startLoc.x() / 2000.0 << "\n";
            //   } else {
            //     cout << "@@@ DEBUG: begin point has non-0 ext at x = " << startLoc.x() / 2000.0 << "\n";
            //   }
            //   auto routeBox = getRouteBox();
            //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
            //   std::cout <<"routeBox (" <<routeBox.left() / dbu <<", " <<routeBox.bottom() / dbu <<") ("
            //                            <<routeBox.right()/ dbu <<", " <<routeBox.top()    / dbu <<")" <<std::endl;
            // }
            // if (/*endLoc.x() == 1834400 && */endLoc.y() == 2095500 && currLayerNum == 6 && net->getFrNet()->getName() == string("net74729")) {
            //   if (currStyle.getEndStyle() == frEndStyle(frcTruncateEndStyle)) {
            //     cout << "@@@ DEBUG: end point has 0 ext at x = " << endLoc.x() / 2000.0 << "\n";
            //   } else {
            //     cout << "@@@ DEBUG: end point has non-0 ext at x = " << endLoc.x() / 2000.0 << "\n";
            //   }
            //   auto routeBox = getRouteBox();
            //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
            //   std::cout <<"routeBox (" <<routeBox.left() / dbu <<", " <<routeBox.bottom() / dbu <<") ("
            //                            <<routeBox.right()/ dbu <<", " <<routeBox.top()    / dbu <<")" <<std::endl;
            // }
            // quick drc cnt
            bool prevHasCost = false;
            for (int i = startX; i < endX; i++)
            {
                if (gridGraph.hasDRCCost(i, startY, startZ, frDirEnum::E))
                {
                    if (!prevHasCost)
                    {
                        net->addMarker();
                        prevHasCost = true;
                    }
                    if (TEST)
                    {
                        cout << " pass marker @(" << i << ", " << startY << ", " << startZ << ") E" << endl;
                    }
                }
                else
                {
                    prevHasCost = false;
                }
            }
            if (enableOutput)
            {
                cout << " write horz pathseg ("
                     << startLoc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", "
                     << startLoc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") ("
                     << endLoc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", "
                     << endLoc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") "
                     << getTech()->getLayer(currLayerNum)->getName() << endl;
            }
            // vertical wire
        }
        else if (startX == endX && startY != endY && startZ == endZ)
        {
            frPoint startLoc, endLoc;
            frLayerNum currLayerNum = gridGraph.getLayerNum(startZ);
            gridGraph.getPoint(startLoc, startX, startY);
            gridGraph.getPoint(endLoc, endX, endY);
            auto currPathSeg = make_unique<drPathSeg>();
            currPathSeg->setPoints(startLoc, endLoc);
            currPathSeg->setLayerNum(currLayerNum);
            currPathSeg->addToNet(net);
            auto currStyle = getTech()->getLayer(currLayerNum)->getDefaultSegStyle();
            if (apMazeIdx.find(start) != apMazeIdx.end())
            {
                currStyle.setBeginStyle(frcTruncateEndStyle, 0);
            }
            if (apMazeIdx.find(end) != apMazeIdx.end())
            {
                currStyle.setEndStyle(frcTruncateEndStyle, 0);
            }
            currPathSeg->setStyle(currStyle);
            currPathSeg->setMazeIdx(start, end);
            unique_ptr<drConnFig> tmp(std::move(currPathSeg));
            workerRegionQuery.add(tmp.get());
            net->addRoute(tmp);
            // if (startLoc.x() == 127300 && currLayerNum == 2 && net->getFrNet()->getName() == string("pci_devsel_oe_o")) {
            //   if (currStyle.getBeginStyle() == frEndStyle(frcTruncateEndStyle)) {
            //     cout << "@@@ DEBUG: begin point has 0 ext at y = " << startLoc.y() / 1000.0 << "(mazeIdx (x, y) = (" << startX << "," << startY << ")\n";
            //   } else {
            //     cout << "@@@ DEBUG: begin point has non-0 ext at y = " << startLoc.y() / 1000.0 << "(mazeIdx (x, y) = (" << startX << "," << startY << ")\n";
            //   }
            //   auto routeBox = getRouteBox();
            //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
            //   std::cout <<"routeBox (" <<routeBox.left() / dbu <<", " <<routeBox.bottom() / dbu <<") ("
            //                            <<routeBox.right()/ dbu <<", " <<routeBox.top()    / dbu <<")" <<std::endl;
            // }
            // if (endLoc.x() == 127300 && currLayerNum == 2 && net->getFrNet()->getName() == string("pci_devsel_oe_o")) {
            //   if (currStyle.getEndStyle() == frEndStyle(frcTruncateEndStyle)) {
            //     cout << "@@@ DEBUG: end point has 0 ext at y = " << endLoc.y() / 1000.0 << "(mazeIdx (x, y) = (" << endX << "," << endY << ")\n";
            //   } else {
            //     cout << "@@@ DEBUG: end point has non-0 ext at x = " << endLoc.y() / 1000.0 << "(mazeIdx (x, y) = (" << endX << "," << endY << ")\n";
            //   }
            //   auto routeBox = getRouteBox();
            //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
            //   std::cout <<"routeBox (" <<routeBox.left() / dbu <<", " <<routeBox.bottom() / dbu <<") ("
            //                            <<routeBox.right()/ dbu <<", " <<routeBox.top()    / dbu <<")" <<std::endl;
            // }
            // quick drc cnt
            bool prevHasCost = false;
            for (int i = startY; i < endY; i++)
            {
                if (gridGraph.hasDRCCost(startX, i, startZ, frDirEnum::E))
                {
                    if (!prevHasCost)
                    {
                        net->addMarker();
                        prevHasCost = true;
                    }
                    if (TEST)
                    {
                        cout << " pass marker @(" << startX << ", " << i << ", " << startZ << ") N" << endl;
                    }
                }
                else
                {
                    prevHasCost = false;
                }
            }
            if (enableOutput)
            {
                cout << " write vert pathseg ("
                     << startLoc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", "
                     << startLoc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") ("
                     << endLoc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", "
                     << endLoc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") "
                     << getTech()->getLayer(currLayerNum)->getName() << endl;
            }
            // via
        }
        else if (startX == endX && startY == endY && startZ != endZ)
        {
            for (auto currZ = startZ; currZ < endZ; ++currZ)
            {
                frPoint loc;
                frLayerNum startLayerNum = gridGraph.getLayerNum(currZ);
                // frLayerNum endLayerNum = gridGraph.getLayerNum(currZ + 1);
                gridGraph.getPoint(loc, startX, startY);
                FlexMazeIdx mi(startX, startY, currZ);
                auto cutLayerDefaultVia = getTech()->getLayer(startLayerNum + 1)->getDefaultViaDef();
                if (gridGraph.isSVia(startX, startY, currZ))
                {
                    cutLayerDefaultVia = apSVia.find(mi)->second->getAccessViaDef();
                }
                auto currVia = make_unique<drVia>(cutLayerDefaultVia);
                currVia->setOrigin(loc);
                currVia->setMazeIdx(FlexMazeIdx(startX, startY, currZ), FlexMazeIdx(startX, startY, currZ + 1));
                unique_ptr<drConnFig> tmp(std::move(currVia));
                workerRegionQuery.add(tmp.get());
                net->addRoute(tmp);
                if (gridGraph.hasDRCCost(startX, startY, currZ, frDirEnum::U))
                {
                    net->addMarker();
                    if (TEST)
                    {
                        cout << " pass marker @(" << startX << ", " << startY << ", " << currZ << ") U" << endl;
                    }
                }
                if (enableOutput)
                {
                    cout << " write via ("
                         << loc.x() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ", "
                         << loc.y() * 1.0 / getDesign()->getTopBlock()->getDBUPerUU() << ") "
                         << cutLayerDefaultVia->getName() << endl;
                }
            }
            // zero length
        }
        else if (startX == endX && startY == endY && startZ == endZ)
        {
            std::cout << "Warning: zero-length path in updateFlexPin\n";
        }
        else
        {
            std::cout << "Error: non-colinear path in updateFlexPin\n";
        }
    }
}
// 对每个 net的路径添加路径成本
void FlexDRWorker::routeNet_postRouteAddPathCost(drNet *net)
{
    int cnt = 0;
    for (auto &connFig : net->getRouteConnFigs())
    {
        addPathCost(connFig.get());
        cnt++;
    }
    // cout <<"updated " <<cnt <<" connfig costs" <<endl;
}
// 目的是为了准备一个面积映射，这个映射记录了网络中每个引脚接入点（通过迷宫索引标识）的最大初始面积。没明白什么要来看这个映射点面积
void FlexDRWorker::routeNet_prepAreaMap(drNet *net, map<FlexMazeIdx, frCoord> &areaMap)
{
    FlexMazeIdx mIdx;
    for (auto &pin : net->getPins())
    {
        for (auto &ap : pin->getAccessPatterns())
        {
            ap->getMazeIdx(mIdx);         // 获取接入模式的迷宫索引
            auto it = areaMap.find(mIdx); // 尝试在面积映射中查找该索引
            if (it != areaMap.end())
            {                                                     // 如果找到已存在的记录
                it->second = max(it->second, ap->getBeginArea()); // 更新最大面积值
            }
            else
            {
                areaMap[mIdx] = ap->getBeginArea(); // 未找到作为新记录插入
            }
        }
    }
}
// 该函数负责一个网络内的所有引脚布线
bool FlexDRWorker::routeNet(drNet *net)
{
    // bool enableOutput = true;
    bool enableOutput = false;
    if (net->getPins().size() <= 1)
    { // 如果网络中的引脚数小于等于1，不需要布线，直接返回true
        return true;
    }

    if (TEST || enableOutput)
    {
        cout << "route " << net->getFrNet()->getName() << endl;
    }
    // 准备未连接引脚集合和它们在迷宫中的索引
    set<drPin *, frBlockObjectComp> unConnPins; //未连接引脚集合
    map<FlexMazeIdx, set<drPin *, frBlockObjectComp>> mazeIdx2unConnPins;   //迷宫布线索引到未连接引脚的映射
    set<FlexMazeIdx> apMazeIdx; // 访问模式的迷宫索引
    set<FlexMazeIdx> realPinAPMazeIdx; // 实际引脚访问模式的迷宫索引
    // map<FlexMazeIdx, frViaDef*> apSVia;// 进行布线准备，包括设置未连接引脚集合等
    routeNet_prep(net, unConnPins, mazeIdx2unConnPins, apMazeIdx, realPinAPMazeIdx /*, apSVia*/);
    // prep for area map
    map<FlexMazeIdx, frCoord> areaMap; // 准备面积映射，用于边界修复
    if (ENABLE_BOUNDARY_MAR_FIX)
    {
        routeNet_prepAreaMap(net, areaMap);
    }
    // 存储连接组件的左下和右上迷宫索引，以及中心点
    //Idx1 - 最大范围索引 , Idx2 - 最小范围索引
    FlexMazeIdx ccMazeIdx1, ccMazeIdx2; // connComps ll, ur flexmazeidx
    frPoint centerPt;
    vector<FlexMazeIdx> connComps;  //相连的组件
                                                                                                  // 存储路径
    routeNet_setSrc(unConnPins, mazeIdx2unConnPins, connComps, ccMazeIdx1, ccMazeIdx2, centerPt); // 设置源引脚和更新连接组件信息
    vector<FlexMazeIdx> path;                                                                     // astar must return with >= 1 idx
    bool isFirstConn = true;        //第一次连接 - 初始化为true
    while (!unConnPins.empty())
    { // 当还有未连接的引脚时
        //1.先找该区域第一个待连接的引脚
        auto nextPin = routeNet_getNextDst(ccMazeIdx1, ccMazeIdx2, mazeIdx2unConnPins);
        mazePinInit(); // 初始化迷宫布线的引脚成本和每个结点的前驱
        path.clear();  //清空当前path路径矩阵，为当前pin的寻路做好准备
        // if (nextPin->hasFrTerm()) {
        //   if (nextPin->getFrTerm()->typeId() == frcTerm) {
        //     cout << "next pin (frTermName) = " << ((frTerm*)nextPin->getFrTerm())->getName() << "\n";
        //   } else {
        //     cout << "next pin (frTermName) = " << ((frInstTerm*)nextPin->getFrTerm())->getInst()->getName() << "/" << ((frInstTerm*)nextPin->getFrTerm())->getTerm()->getName() << "\n";
        //   }
        // } else {
        //   cout << "next pin is boundary pin\n";
        // }

        // 如果找到路径
        if (gridGraph.search(connComps, nextPin, path, ccMazeIdx1, ccMazeIdx2, centerPt))
        {
            routeNet_postAstarUpdate(path, connComps, unConnPins, mazeIdx2unConnPins, isFirstConn);
            routeNet_postAstarWritePath(net, path, realPinAPMazeIdx /*, apSVia*/);
            routeNet_postAstarPatchMinAreaVio(net, path, areaMap);
            isFirstConn = false;
        }
        else
        {
            // int apCnt = 0;
            // for (auto &ap: nextPin->getAccessPatterns()) {
            //   auto apPtr = ap.get();
            //   auto mazeIdx = apPtr->getMazeIdx();
            //   frPoint bp;
            //   apPtr->getPoint(bp);
            //   // cout << "  ap" << apCnt << " has MazeIdx (" << mazeIdx.x() << ", " << mazeIdx.y() << ", " << mazeIdx.z() << "), real coord = ("
            //   //      << bp.x() / 2000.0 << ", " << bp.y() / 2000.0 << ")\n";
            //   ++apCnt;
            // }
            return false;
        }
    }
    // 添加路径成本
    routeNet_postRouteAddPathCost(net);
    return true;
}

// 在路由过程后处理最小面积违规问题。当连线长度小于规定的最小面积时，需要对路径进行补丁处理以满足设计规则。
void FlexDRWorker::routeNet_postAstarPatchMinAreaVio(drNet *net, const vector<FlexMazeIdx> &path, const map<FlexMazeIdx, frCoord> &areaMap)
{
    if (path.empty())
    {
        return;
    }
    // get path with separated (stacked vias)
    vector<FlexMazeIdx> points;
    for (int i = 0; i < (int)path.size() - 1; ++i)
    {
        auto currIdx = path[i];
        auto nextIdx = path[i + 1];
        if (currIdx.z() == nextIdx.z())
        {
            points.push_back(currIdx);
        }
        else
        {
            if (currIdx.z() < nextIdx.z())
            {
                for (auto z = currIdx.z(); z < nextIdx.z(); ++z)
                {
                    FlexMazeIdx tmpIdx(currIdx.x(), currIdx.y(), z);
                    points.push_back(tmpIdx);
                }
            }
            else
            {
                for (auto z = currIdx.z(); z > nextIdx.z(); --z)
                {
                    FlexMazeIdx tmpIdx(currIdx.x(), currIdx.y(), z);
                    points.push_back(tmpIdx);
                }
            }
        }
    }
    points.push_back(path.back());

    auto layerNum = gridGraph.getLayerNum(points.front().z());
    auto minAreaConstraint = getDesign()->getTech()->getLayer(layerNum)->getAreaConstraint();
    // frCoord currArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
    frCoord currArea = 0;
    if (ENABLE_BOUNDARY_MAR_FIX)
    {
        if (areaMap.find(points[0]) != areaMap.end())
        {
            currArea = areaMap.find(points[0])->second;
            // if (TEST) {
            //   cout <<"currArea[0] = " <<currArea <<", from areaMap" <<endl;
            // }
        }
        else
        {
            currArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
            // if (TEST) {
            //   cout <<"currArea[0] = " <<currArea <<", from rule" <<endl;
            // }
        }
    }
    else
    {
        currArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
    }
    frCoord startViaHalfEncArea = 0, endViaHalfEncArea = 0;
    FlexMazeIdx prevIdx = points[0], currIdx;
    int i;
    int prev_i = 0; // path start point
    for (i = 1; i < (int)points.size(); ++i)
    {
        currIdx = points[i];
        // check minAreaViolation when change layer, or last segment
        if (currIdx.z() != prevIdx.z())
        {
            layerNum = gridGraph.getLayerNum(prevIdx.z());
            minAreaConstraint = getDesign()->getTech()->getLayer(layerNum)->getAreaConstraint();
            frCoord reqArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
            // add next via enclosure
            if (currIdx.z() < prevIdx.z())
            {
                currArea += gridGraph.getHalfViaEncArea(prevIdx.z() - 1, false);
                endViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z() - 1, false);
            }
            else
            {
                currArea += gridGraph.getHalfViaEncArea(prevIdx.z(), true);
                endViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z(), true);
            }
            // push to minArea violation
            if (currArea < reqArea)
            {
                // if (TEST) {
                //   cout <<"currArea[" <<i <<"] = " <<currArea <<", add pwire " <<i - 2 <<", " <<i - 1 <<endl;
                // }
                FlexMazeIdx bp, ep;
                frCoord gapArea = reqArea - (currArea - startViaHalfEncArea - endViaHalfEncArea) - std::min(startViaHalfEncArea, endViaHalfEncArea);
                // bp = std::min(prevIdx, currIdx);
                // ep = std::max(prevIdx, currIdx);
                // new
                bool bpPatchStyle = true; // style 1: left only; 0: right only
                bool epPatchStyle = false;
                // stack via
                if (i - 1 == prev_i)
                {
                    bp = points[i - 1];
                    ep = points[i - 1];
                    bpPatchStyle = true;
                    epPatchStyle = false;
                    // planar
                }
                else
                {
                    // bp = points[prev_i];
                    // ep = points[i-1];
                    // if (points[prev_i] < points[prev_i+1]) {
                    //   bpPatchStyle = true;
                    // } else {
                    //   bpPatchStyle = false;
                    // }
                    // if (points[i-1] < points[i-2]) {
                    //   epPatchStyle = true;
                    // } else {
                    //   epPatchStyle = false;
                    // }
                    bp = points[prev_i];
                    ep = points[i - 1];
                    if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir)
                    {
                        if (points[prev_i].x() < points[prev_i + 1].x())
                        {
                            bpPatchStyle = true;
                        }
                        else if (points[prev_i].x() > points[prev_i + 1].x())
                        {
                            bpPatchStyle = false;
                        }
                        else
                        {
                            if (points[prev_i].x() < points[i - 1].x())
                            {
                                bpPatchStyle = true;
                            }
                            else
                            {
                                bpPatchStyle = false;
                            }
                        }
                        if (points[i - 1].x() < points[i - 2].x())
                        {
                            epPatchStyle = true;
                        }
                        else if (points[i - 1].x() > points[i - 2].x())
                        {
                            epPatchStyle = false;
                        }
                        else
                        {
                            if (points[i - 1].x() < points[prev_i].x())
                            {
                                epPatchStyle = true;
                            }
                            else
                            {
                                epPatchStyle = false;
                            }
                        }
                    }
                    else
                    {
                        if (points[prev_i].y() < points[prev_i + 1].y())
                        {
                            bpPatchStyle = true;
                        }
                        else if (points[prev_i].y() > points[prev_i + 1].y())
                        {
                            bpPatchStyle = false;
                        }
                        else
                        {
                            if (points[prev_i].y() < points[i - 1].y())
                            {
                                bpPatchStyle = true;
                            }
                            else
                            {
                                bpPatchStyle = false;
                            }
                        }
                        if (points[i - 1].y() < points[i - 2].y())
                        {
                            epPatchStyle = true;
                        }
                        else if (points[i - 1].y() > points[i - 2].y())
                        {
                            epPatchStyle = false;
                        }
                        else
                        {
                            if (points[i - 1].y() < points[prev_i].y())
                            {
                                epPatchStyle = true;
                            }
                            else
                            {
                                epPatchStyle = false;
                            }
                        }
                    }
                }
                // old
                // if (i - 2 >= 0 && points[i-1].z() == points[i-2].z()) {
                //  bp = std::min(points[i-1], points[i-2]);
                //  ep = std::max(points[i-1], points[i-2]);
                //} else {
                //  bp = points[i-1];
                //  ep = points[i-1];
                //}
                auto patchWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth();
                // FlexDRMinAreaVio minAreaVio(net, bp, ep, reqArea - (currArea - startViaHalfEncArea) - std::min(startViaHalfEncArea, endViaHalfEncArea));
                // minAreaVios.push_back(minAreaVio);
                // non-pref dir boundary add patch on other end
                // if ((getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir &&
                //     bp.x() == ep.x() && bp.y() != ep.y()) ||
                //    (getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcVertPrefRoutingDir &&
                //     bp.y() == ep.y() && bp.x() != ep.x())
                //    ) {
                //  frPoint pt1, pt2;
                //  gridGraph.getPoint(pt1, bp.x(), bp.y());
                //  gridGraph.getPoint(pt2, ep.x(), ep.y());
                //  if (!getRouteBox().contains(pt1, false)) {
                //    bp.set(ep);
                //  } else if (!getRouteBox().contains(pt2, false)) {
                //    ep.set(bp);
                //  }
                //}
                // old
                // routeNet_postAstarAddPatchMetal(net, bp, ep, gapArea, patchWidth);
                // new
                routeNet_postAstarAddPatchMetal(net, bp, ep, gapArea, patchWidth, bpPatchStyle, epPatchStyle);
            }
            else
            {
                // if (TEST) {
                //   cout <<"currArea[" <<i <<"] = " <<currArea <<", no pwire" <<endl;
                // }
            }
            // init for next path
            if (currIdx.z() < prevIdx.z())
            {
                currArea = gridGraph.getHalfViaEncArea(prevIdx.z() - 1, true);
                startViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z() - 1, true);
            }
            else
            {
                currArea = gridGraph.getHalfViaEncArea(prevIdx.z(), false);
                startViaHalfEncArea = gridGraph.getHalfViaEncArea(prevIdx.z(), false);
            }
            prev_i = i;
        }
        // add the wire area
        else
        {
            layerNum = gridGraph.getLayerNum(prevIdx.z());
            minAreaConstraint = getDesign()->getTech()->getLayer(layerNum)->getAreaConstraint();
            frCoord reqArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
            auto pathWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth();
            frPoint bp, ep;
            gridGraph.getPoint(bp, prevIdx.x(), prevIdx.y());
            gridGraph.getPoint(ep, currIdx.x(), currIdx.y());
            frCoord pathLength = abs(bp.x() - ep.x()) + abs(bp.y() - ep.y());
            if (currArea < reqArea)
            {
                currArea += pathLength * pathWidth;
            }
            // if (TEST) {
            //   cout <<"currArea[" <<i <<"] = " <<currArea <<", no pwire planar" <<endl;
            // }
        }
        prevIdx = currIdx;
    }
    // add boundary area for last segment
    if (ENABLE_BOUNDARY_MAR_FIX)
    {
        layerNum = gridGraph.getLayerNum(prevIdx.z());
        minAreaConstraint = getDesign()->getTech()->getLayer(layerNum)->getAreaConstraint();
        frCoord reqArea = (minAreaConstraint) ? minAreaConstraint->getMinArea() : 0;
        if (areaMap.find(prevIdx) != areaMap.end())
        {
            currArea += areaMap.find(prevIdx)->second;
        }
        endViaHalfEncArea = 0;
        if (currArea < reqArea)
        {
            // if (TEST) {
            //   cout <<"currArea[" <<i <<"] = " <<currArea <<", add pwire end" <<endl;
            // }
            FlexMazeIdx bp, ep;
            frCoord gapArea = reqArea - (currArea - startViaHalfEncArea - endViaHalfEncArea) - std::min(startViaHalfEncArea, endViaHalfEncArea);
            // bp = std::min(prevIdx, currIdx);
            // ep = std::max(prevIdx, currIdx);
            // new
            bool bpPatchStyle = true; // style 1: left only; 0: right only
            bool epPatchStyle = false;
            // stack via
            if (i - 1 == prev_i)
            {
                bp = points[i - 1];
                ep = points[i - 1];
                bpPatchStyle = true;
                epPatchStyle = false;
                // planar
            }
            else
            {
                bp = points[prev_i];
                ep = points[i - 1];
                if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir)
                {
                    if (points[prev_i].x() < points[prev_i + 1].x())
                    {
                        bpPatchStyle = true;
                    }
                    else if (points[prev_i].x() > points[prev_i + 1].x())
                    {
                        bpPatchStyle = false;
                    }
                    else
                    {
                        if (points[prev_i].x() < points[i - 1].x())
                        {
                            bpPatchStyle = true;
                        }
                        else
                        {
                            bpPatchStyle = false;
                        }
                    }
                    if (points[i - 1].x() < points[i - 2].x())
                    {
                        epPatchStyle = true;
                    }
                    else if (points[i - 1].x() > points[i - 2].x())
                    {
                        epPatchStyle = false;
                    }
                    else
                    {
                        if (points[i - 1].x() < points[prev_i].x())
                        {
                            epPatchStyle = true;
                        }
                        else
                        {
                            epPatchStyle = false;
                        }
                    }
                }
                else
                {
                    if (points[prev_i].y() < points[prev_i + 1].y())
                    {
                        bpPatchStyle = true;
                    }
                    else if (points[prev_i].y() > points[prev_i + 1].y())
                    {
                        bpPatchStyle = false;
                    }
                    else
                    {
                        if (points[prev_i].y() < points[i - 1].y())
                        {
                            bpPatchStyle = true;
                        }
                        else
                        {
                            bpPatchStyle = false;
                        }
                    }
                    if (points[i - 1].y() < points[i - 2].y())
                    {
                        epPatchStyle = true;
                    }
                    else if (points[i - 1].y() > points[i - 2].y())
                    {
                        epPatchStyle = false;
                    }
                    else
                    {
                        if (points[i - 1].y() < points[prev_i].y())
                        {
                            epPatchStyle = true;
                        }
                        else
                        {
                            epPatchStyle = false;
                        }
                    }
                }
            }
            // if (i - 2 >= 0 && points[i-1].z() == points[i-2].z()) {
            //   bp = std::min(points[i-1], points[i-2]);
            //   ep = std::max(points[i-1], points[i-2]);
            // } else {
            //   bp = points[i-1];
            //   ep = points[i-1];
            // }
            auto patchWidth = getDesign()->getTech()->getLayer(layerNum)->getWidth();
            // FlexDRMinAreaVio minAreaVio(net, bp, ep, reqArea - (currArea - startViaHalfEncArea) - std::min(startViaHalfEncArea, endViaHalfEncArea));
            // minAreaVios.push_back(minAreaVio);
            // non-pref dir boundary add patch on other end to avoid pwire in the middle of non-pref dir routing segment
            // if ((getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcHorzPrefRoutingDir &&
            //     bp.x() == ep.x() && bp.y() != ep.y()) ||
            //    (getDesign()->getTech()->getLayer(layerNum)->getDir() == frPrefRoutingDirEnum::frcVertPrefRoutingDir &&
            //     bp.y() == ep.y() && bp.x() != ep.x())
            //    ) {
            //  frPoint pt1, pt2;
            //  gridGraph.getPoint(pt1, bp.x(), bp.y());
            //  gridGraph.getPoint(pt2, ep.x(), ep.y());
            //  if (!getRouteBox().contains(pt1, false)) {
            //    bp.set(ep);
            //  } else if (!getRouteBox().contains(pt2, false)) {
            //    ep.set(bp);
            //  }
            //}
            routeNet_postAstarAddPatchMetal(net, bp, ep, gapArea, patchWidth, bpPatchStyle, epPatchStyle);
        }
        else
        {
            // if (TEST) {
            //   cout <<"currArea[" <<i <<"] = " <<currArea <<", no pwire end" <<endl;
            // }
        }
    }
}

// void FlexDRWorker::routeNet_postRouteAddPatchMetalCost(drNet* net) {
//   for (auto &patch: net->getRoutePatchConnFigs()) {
//     addPathCost(patch.get());
//   }
// }

// assumes patchWidth == defaultWidth
// the cost checking part is sensitive to how cost is stored (1) planar + via; or (2) N;E;U
int FlexDRWorker::routeNet_postAstarAddPathMetal_isClean(const FlexMazeIdx &bpIdx,
                                                         bool isPatchHorz, bool isPatchLeft,
                                                         frCoord patchLength)
{
    // bool enableOutput = true;
    bool enableOutput = false;
    int cost = 0;
    frPoint origin, patchEnd;
    gridGraph.getPoint(origin, bpIdx.x(), bpIdx.y());
    frLayerNum layerNum = gridGraph.getLayerNum(bpIdx.z());
    if (isPatchHorz)
    {
        if (isPatchLeft)
        {
            patchEnd.set(origin.x() - patchLength, origin.y());
        }
        else
        {
            patchEnd.set(origin.x() + patchLength, origin.y());
        }
    }
    else
    {
        if (isPatchLeft)
        {
            patchEnd.set(origin.x(), origin.y() - patchLength);
        }
        else
        {
            patchEnd.set(origin.x(), origin.y() + patchLength);
        }
    }
    if (enableOutput)
    {
        double dbu = getDesign()->getTopBlock()->getDBUPerUU();
        cout << "    patchOri@(" << origin.x() / dbu << ", " << origin.y() / dbu << ")" << endl;
        cout << "    patchEnd@(" << patchEnd.x() / dbu << ", " << patchEnd.y() / dbu << ")" << endl;
    }
    // for wire, no need to bloat width
    frPoint patchLL = min(origin, patchEnd);
    frPoint patchUR = max(origin, patchEnd);
    if (!getRouteBox().contains(patchEnd))
    {
        cost = std::numeric_limits<int>::max();
    }
    else
    {
        FlexMazeIdx startIdx, endIdx;
        startIdx.set(0, 0, layerNum);
        endIdx.set(0, 0, layerNum);
        // gridGraph.getMazeIdx(startIdx, patchLL, layerNum);
        // gridGraph.getMazeIdx(endIdx, patchUR, layerNum);
        frBox patchBox(patchLL, patchUR);
        gridGraph.getIdxBox(startIdx, endIdx, patchBox);
        if (enableOutput)
        {
            // double dbu = getDesign()->getTopBlock()->getDBUPerUU();
            cout << "    patchOriIdx@(" << startIdx.x() << ", " << startIdx.y() << ")" << endl;
            cout << "    patchEndIdx@(" << endIdx.x() << ", " << endIdx.y() << ")" << endl;
        }
        if (isPatchHorz)
        {
            // in gridgraph, the planar cost is checked for xIdx + 1
            for (auto xIdx = max(0, startIdx.x() - 1); xIdx < endIdx.x(); ++xIdx)
            {
                if (gridGraph.hasDRCCost(xIdx, bpIdx.y(), bpIdx.z(), frDirEnum::E))
                {
                    cost += gridGraph.getEdgeLength(xIdx, bpIdx.y(), bpIdx.z(), frDirEnum::E) * workerDRCCost;
                    if (enableOutput)
                    {
                        cout << "    (" << xIdx << ", " << bpIdx.y() << ", " << bpIdx.z() << ") drc cost" << endl;
                    }
                }
                if (gridGraph.hasShapeCost(xIdx, bpIdx.y(), bpIdx.z(), frDirEnum::E))
                {
                    cost += gridGraph.getEdgeLength(xIdx, bpIdx.y(), bpIdx.z(), frDirEnum::E) * SHAPECOST;
                    if (enableOutput)
                    {
                        cout << "    (" << xIdx << ", " << bpIdx.y() << ", " << bpIdx.z() << ") shape cost" << endl;
                    }
                }
                if (gridGraph.hasMarkerCost(xIdx, bpIdx.y(), bpIdx.z(), frDirEnum::E))
                {
                    cost += gridGraph.getEdgeLength(xIdx, bpIdx.y(), bpIdx.z(), frDirEnum::E) * workerMarkerCost;
                    if (enableOutput)
                    {
                        cout << "    (" << xIdx << ", " << bpIdx.y() << ", " << bpIdx.z() << ") marker cost" << endl;
                    }
                }
                // if (apSVia.find(FlexMazeIdx(xIdx, bpIdx.y(), bpIdx.z() - 1)) != apSVia.end()) { // avoid covering upper layer of apSVia
                //   cost += gridGraph.getEdgeLength(xIdx, bpIdx.y(), bpIdx.z() - 1, frDirEnum::U) * workerDRCCost;
                // }
            }
        }
        else
        {
            // in gridgraph, the planar cost is checked for yIdx + 1
            for (auto yIdx = max(0, startIdx.y() - 1); yIdx < endIdx.y(); ++yIdx)
            {
                if (enableOutput)
                {
                    cout << "    check (" << bpIdx.x() << ", " << yIdx << ", " << bpIdx.z() << ") N" << endl;
                }
                if (gridGraph.hasDRCCost(bpIdx.x(), yIdx, bpIdx.z(), frDirEnum::N))
                {
                    cost += gridGraph.getEdgeLength(bpIdx.x(), yIdx, bpIdx.z(), frDirEnum::N) * workerDRCCost;
                    if (enableOutput)
                    {
                        cout << "    (" << bpIdx.x() << ", " << yIdx << ", " << bpIdx.z() << ") drc cost" << endl;
                    }
                }
                if (gridGraph.hasShapeCost(bpIdx.x(), yIdx, bpIdx.z(), frDirEnum::N))
                {
                    cost += gridGraph.getEdgeLength(bpIdx.x(), yIdx, bpIdx.z(), frDirEnum::N) * SHAPECOST;
                    if (enableOutput)
                    {
                        cout << "    (" << bpIdx.x() << ", " << yIdx << ", " << bpIdx.z() << ") shape cost" << endl;
                    }
                }
                if (gridGraph.hasMarkerCost(bpIdx.x(), yIdx, bpIdx.z(), frDirEnum::N))
                {
                    cost += gridGraph.getEdgeLength(bpIdx.x(), yIdx, bpIdx.z(), frDirEnum::N) * workerMarkerCost;
                    if (enableOutput)
                    {
                        cout << "    (" << bpIdx.x() << ", " << yIdx << ", " << bpIdx.z() << ") marker cost" << endl;
                    }
                }
                // if (apSVia.find(FlexMazeIdx(bpIdx.x(), yIdx, bpIdx.z() - 1)) != apSVia.end()) { // avoid covering upper layer of apSVia
                //   cost += gridGraph.getEdgeLength(bpIdx.x(), yIdx, bpIdx.z() - 1, frDirEnum::U) * workerDRCCost;
                // }
            }
        }
    }
    return cost;
}

void FlexDRWorker::routeNet_postAstarAddPatchMetal_addPWire(drNet *net, const FlexMazeIdx &bpIdx,
                                                            bool isPatchHorz, bool isPatchLeft,
                                                            frCoord patchLength, frCoord patchWidth)
{
    frPoint origin, patchEnd;
    gridGraph.getPoint(origin, bpIdx.x(), bpIdx.y());
    frLayerNum layerNum = gridGraph.getLayerNum(bpIdx.z());
    // actual offsetbox
    frPoint patchLL, patchUR;
    if (isPatchHorz)
    {
        if (isPatchLeft)
        {
            patchLL.set(0 - patchLength, 0 - patchWidth / 2);
            patchUR.set(0, 0 + patchWidth / 2);
        }
        else
        {
            patchLL.set(0, 0 - patchWidth / 2);
            patchUR.set(0 + patchLength, 0 + patchWidth / 2);
        }
    }
    else
    {
        if (isPatchLeft)
        {
            patchLL.set(0 - patchWidth / 2, 0 - patchLength);
            patchUR.set(0 + patchWidth / 2, 0);
        }
        else
        {
            patchLL.set(0 - patchWidth / 2, 0);
            patchUR.set(0 + patchWidth / 2, 0 + patchLength);
        }
    }

    auto tmpPatch = make_unique<drPatchWire>();
    tmpPatch->setLayerNum(layerNum);
    tmpPatch->setOrigin(origin);
    tmpPatch->setOffsetBox(frBox(patchLL, patchUR));
    tmpPatch->addToNet(net);
    unique_ptr<drConnFig> tmp(std::move(tmpPatch));
    auto &workerRegionQuery = getWorkerRegionQuery();
    workerRegionQuery.add(tmp.get());
    net->addRoute(tmp);
    // if (TEST) {
    //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    //   cout <<"pwire@(" <<origin.x() / dbu <<", " <<origin.y() / dbu <<"), ("
    //        <<patchLL.x() / dbu <<", " <<patchLL.y() / dbu <<", "
    //        <<patchUR.x() / dbu <<", " <<patchUR.y() / dbu
    //        <<endl;
    // }
}

void FlexDRWorker::routeNet_postAstarAddPatchMetal(drNet *net,
                                                   const FlexMazeIdx &bpIdx,
                                                   const FlexMazeIdx &epIdx,
                                                   frCoord gapArea,
                                                   frCoord patchWidth,
                                                   bool bpPatchStyle, bool epPatchStyle)
{
    bool isPatchHorz;
    // bool isLeftClean = true;
    frLayerNum layerNum = gridGraph.getLayerNum(bpIdx.z());
    frCoord patchLength = frCoord(ceil(1.0 * gapArea / patchWidth / getDesign()->getTech()->getManufacturingGrid())) *
                          getDesign()->getTech()->getManufacturingGrid();
    // if (gapArea % patchWidth != 0) {
    //   ++patchLength;
    // }
    // frPoint origin;
    // auto &workerRegionQuery = getWorkerRegionQuery();
    //// stacked via
    // if (bpIdx.x() == epIdx.x() && bpIdx.y() == epIdx.y()) {
    //   if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) {
    //     isPatchHorz = true;
    //   } else {
    //     isPatchHorz = false;
    //   }
    // }
    //// vertical patch
    // else if (bpIdx.x() == epIdx.x()) {
    //   isPatchHorz = false;
    // }
    //// horizontal patch
    // else {
    //   isPatchHorz = true;
    // }

    // always patch to pref dir
    if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir)
    {
        isPatchHorz = true;
    }
    else
    {
        isPatchHorz = false;
    }

    // if (QUICKDRCTEST) {
    //   cout <<"  pwire L" <<endl;
    // }
    auto costL = routeNet_postAstarAddPathMetal_isClean(bpIdx, isPatchHorz, bpPatchStyle, patchLength);
    // if (QUICKDRCTEST) {
    //   cout <<"  pwire R" <<endl;
    // }
    auto costR = routeNet_postAstarAddPathMetal_isClean(epIdx, isPatchHorz, epPatchStyle, patchLength);
    // if (QUICKDRCTEST) {
    //   double dbu = getDesign()->getTopBlock()->getDBUPerUU();
    //   frPoint bp, ep;
    //   gridGraph.getPoint(bp, bpIdx.x(), bpIdx.y());
    //   gridGraph.getPoint(ep, epIdx.x(), epIdx.y());
    //   cout <<"  pwire L@(" <<bpIdx.x()    <<", " <<bpIdx.y()           <<", " <<bpIdx.z() <<") ("
    //                        <<bp.x() / dbu <<", " <<bp.y() / dbu <<") " <<getDesign()->getTech()->getLayer(layerNum)->getName() <<", cost="
    //                        <<costL <<endl;
    //   cout <<"  pwire R@(" <<epIdx.x()    <<", " <<epIdx.y()           <<", " <<epIdx.z() <<") ("
    //                        <<ep.x() / dbu <<", " <<ep.y() / dbu <<") " <<getDesign()->getTech()->getLayer(layerNum)->getName() <<", cost="
    //                        <<costR <<endl;
    // }
    if (costL <= costR)
    {
        routeNet_postAstarAddPatchMetal_addPWire(net, bpIdx, isPatchHorz, bpPatchStyle, patchLength, patchWidth);
        // if (TEST) {
        //   cout <<"pwire added L" <<endl;
        // }
    }
    else
    {
        routeNet_postAstarAddPatchMetal_addPWire(net, epIdx, isPatchHorz, epPatchStyle, patchLength, patchWidth);
        // if (TEST) {
        //   cout <<"pwire added R" <<endl;
        // }
    }
}

// void FlexDRWorker::routeNet_postAstarAddPatchMetal(drNet* net,
//                                                    const FlexMazeIdx &bpIdx,
//                                                    const FlexMazeIdx &epIdx,
//                                                    frCoord gapArea,
//                                                    frCoord patchWidth) {
//   bool isPatchHorz;
//   bool isLeftClean = true;
//   frLayerNum layerNum = gridGraph.getLayerNum(bpIdx.z());
//   frCoord patchLength = frCoord(ceil(1.0 * gapArea / patchWidth / getDesign()->getTech()->getManufacturingGrid())) *
//                         getDesign()->getTech()->getManufacturingGrid();
//   //if (gapArea % patchWidth != 0) {
//   //  ++patchLength;
//   //}
//   frPoint origin;
//   auto &workerRegionQuery = getWorkerRegionQuery();
//   // stacked via
//   if (bpIdx.x() == epIdx.x() && bpIdx.y() == epIdx.y()) {
//     if (getDesign()->getTech()->getLayer(layerNum)->getDir() == frcHorzPrefRoutingDir) {
//       isPatchHorz = true;
//     } else {
//       isPatchHorz = false;
//     }
//   }
//   // vertical patch
//   else if (bpIdx.x() == epIdx.x()) {
//     isPatchHorz = false;
//   }
//   // horizontal patch
//   else {
//     isPatchHorz = true;
//   }
//
//   // try bottom / left option
//   if (isPatchHorz) {
//     gridGraph.getPoint(origin, bpIdx.x(), bpIdx.y());
//     frPoint patchEnd(origin.x() - patchLength, origin.y());
//     if (!getRouteBox().contains(patchEnd)) {
//       isLeftClean = false;
//     } else {
//       // patch metal length is manufacturing grid, additional half widht is required
//       frPoint patchLL(origin.x() - patchLength/* - patchWidth / 2*/, origin.y() - patchWidth / 2);
//       //frPoint patchLL(origin.x() - patchLength - patchWidth / 2, origin.y() - patchWidth / 2);
//       frPoint patchUR(origin.x(), origin.y() + patchWidth / 2);
//       FlexMazeIdx startIdx, endIdx;
//       gridGraph.getMazeIdx(startIdx, patchLL, layerNum);
//       gridGraph.getMazeIdx(endIdx, patchUR, layerNum);
//       for (auto xIdx = startIdx.x(); xIdx < endIdx.x(); ++xIdx) {
//         for (auto yIdx = startIdx.y(); yIdx < endIdx.y(); ++yIdx) {
//           // origin should not be checked cost
//           if (xIdx == bpIdx.x() && yIdx == bpIdx.y()) {
//             continue;
//           }
//           if (gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
//               gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N) ||
//               gridGraph.hasShapeCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
//               gridGraph.hasShapeCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N) ||
//               gridGraph.hasMarkerCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
//               gridGraph.hasMarkerCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N) //||
//               //apSVia.find(FlexMazeIdx(xIdx, yIdx, bpIdx.z() - 1)) != apSVia.end() // avoid covering upper layer of apSVia
//               ) {
//             isLeftClean = false;
//             break;
//           }
//         }
//       }
//     }
//     // add patch if clean
//     if (isLeftClean) {
//       frPoint patchLL(origin.x() - patchLength, origin.y() - patchWidth / 2);
//       frPoint patchUR(origin.x(), origin.y() + patchWidth / 2);
//       auto tmpPatch = make_unique<drPatchWire>();
//       tmpPatch->setLayerNum(layerNum);
//       tmpPatch->setOrigin(origin);
//       // tmpPatch->setBBox(frBox(patchLL, patchUR));
//       tmpPatch->setOffsetBox(frBox(-patchLength, -patchWidth / 2, 0, patchWidth / 2));
//       tmpPatch->addToNet(net);
//       unique_ptr<drConnFig> tmp(std::move(tmpPatch));
//       workerRegionQuery.add(tmp.get());
//       net->addRoute(tmp);
//     }
//   } else {
//     gridGraph.getPoint(origin, bpIdx.x(), bpIdx.y());
//     frPoint patchEnd(origin.x(), origin.y() - patchLength);
//     if (!getRouteBox().contains(patchEnd)) {
//       isLeftClean = false;
//     } else {
//       // patch metal length is manufacturing grid, additional half widht is required
//       frPoint patchLL(origin.x() - patchWidth / 2, origin.y() - patchLength/* - patchWidth / 2*/);
//       //frPoint patchLL(origin.x() - patchWidth / 2, origin.y() - patchLength - patchWidth / 2);
//       frPoint patchUR(origin.x() + patchWidth / 2, origin.y());
//       FlexMazeIdx startIdx, endIdx;
//       gridGraph.getMazeIdx(startIdx, patchLL, layerNum);
//       gridGraph.getMazeIdx(endIdx, patchUR, layerNum);
//       for (auto xIdx = startIdx.x(); xIdx < endIdx.x(); ++xIdx) {
//         for (auto yIdx = startIdx.y(); yIdx < endIdx.y(); ++yIdx) {
//           // origin should not be checked cost
//           if (xIdx == bpIdx.x() && yIdx == bpIdx.y()) {
//             continue;
//           }
//           if (gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
//               gridGraph.hasDRCCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N) ||
//               gridGraph.hasShapeCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
//               gridGraph.hasShapeCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N) ||
//               gridGraph.hasMarkerCost(xIdx, yIdx, bpIdx.z(), frDirEnum::E) ||
//               gridGraph.hasMarkerCost(xIdx, yIdx, bpIdx.z(), frDirEnum::N) //||
//               //apSVia.find(FlexMazeIdx(xIdx, yIdx, bpIdx.z() - 1)) != apSVia.end() // avoid covering upper layer of apSVia
//              ) {
//             isLeftClean = false;
//             break;
//           }
//         }
//       }
//     }
//     // add patch if clean
//     if (isLeftClean) {
//       frPoint patchLL(origin.x() - patchWidth / 2, origin.y() - patchLength);
//       frPoint patchUR(origin.x() + patchWidth / 2, origin.y());
//       auto tmpPatch = make_unique<drPatchWire>();
//       tmpPatch->setLayerNum(layerNum);
//       tmpPatch->setOrigin(origin);
//       // tmpPatch->setBBox(frBox(patchLL, patchUR));
//       tmpPatch->setOffsetBox(frBox(-patchWidth / 2, -patchLength, patchWidth / 2, 0));
//       tmpPatch->addToNet(net);
//       unique_ptr<drConnFig> tmp(std::move(tmpPatch));
//       workerRegionQuery.add(tmp.get());
//       net->addRoute(tmp);
//     }
//   }
//   // use top / right option if bottom / left is not usable
//   if (!isLeftClean) {
//     gridGraph.getPoint(origin, epIdx.x(), epIdx.y());
//     if (isPatchHorz) {
//       frPoint patchLL(origin.x(), origin.y() - patchWidth / 2);
//       frPoint patchUR(origin.x() + patchLength, origin.y() + patchWidth / 2);
//
//       auto tmpPatch = make_unique<drPatchWire>();
//       tmpPatch->setLayerNum(layerNum);
//       tmpPatch->setOrigin(origin);
//       // tmpPatch->setBBox(frBox(patchLL, patchUR));
//       tmpPatch->setOffsetBox(frBox(0, -patchWidth / 2, patchLength, patchWidth / 2));
//       tmpPatch->addToNet(net);
//       unique_ptr<drConnFig> tmp(std::move(tmpPatch));
//       workerRegionQuery.add(tmp.get());
//       net->addRoute(tmp);
//     } else {
//       frPoint patchLL(origin.x() - patchWidth / 2, origin.y());
//       frPoint patchUR(origin.x() + patchWidth / 2, origin.y() + patchLength);
//
//       auto tmpPatch = make_unique<drPatchWire>();
//       tmpPatch->setLayerNum(layerNum);
//       tmpPatch->setOrigin(origin);
//       // tmpPatch->setBBox(frBox(patchLL, patchUR));
//       tmpPatch->setOffsetBox(frBox(-patchWidth / 2, 0, patchWidth / 2, patchLength));
//       tmpPatch->addToNet(net);
//       unique_ptr<drConnFig> tmp(std::move(tmpPatch));
//       workerRegionQuery.add(tmp.get());
//       net->addRoute(tmp);
//     }
//   }
//
// }
