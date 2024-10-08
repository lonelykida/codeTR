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

#ifndef _GC_NET_H_
#define _GC_NET_H_

#include <memory>
#include "db/gcObj/gcBlockObject.h"
#include "db/gcObj/gcPin.h"

namespace fr {
  class frNet;
  class gcNet: public gcBlockObject {
  public:
    // constructors
    gcNet(int numLayers): gcBlockObject(), fixedPolygons(numLayers), routePolygons(numLayers), 
                          fixedRectangles(numLayers), routeRectangles(numLayers), pins(numLayers), owner(nullptr) {}
    // setters
    void addPolygon(const frBox &box, frLayerNum layerNum, bool isFixed = false) {
      gtl::rectangle_data<frCoord> rect(box.left(), box.bottom(), box.right(), box.top());
      using namespace gtl::operators;
      if (isFixed) {
        fixedPolygons[layerNum] += rect;
      } else {
        routePolygons[layerNum] += rect;
      }
    }
    void addRectangle(const frBox &box, frLayerNum layerNum, bool isFixed = false) {
      gtl::rectangle_data<frCoord> rect(box.left(), box.bottom(), box.right(), box.top());
      if (isFixed) {
        fixedRectangles[layerNum].push_back(rect);
      } else {
        routeRectangles[layerNum].push_back(rect);
      }
    }
    void addPin(const gtl::polygon_90_with_holes_data<frCoord> &shape, frLayerNum layerNum) {
      auto pin = std::make_unique<gcPin>(shape, layerNum, this);
      pin->setId(pins[layerNum].size());
      pins[layerNum].push_back(std::move(pin));
    }
    void addPin(const gtl::rectangle_data<frCoord> &rect, frLayerNum layerNum) {
      gtl::polygon_90_with_holes_data<frCoord> shape;
      std::vector<frCoord> coords = {gtl::xl(rect), gtl::yl(rect), gtl::xh(rect), gtl::yh(rect)};
      shape.set_compact(coords.begin(), coords.end());
      //std::cout <<"rect (" <<gtl::xl(rect) <<", " <<gtl::yl(rect) <<") (" <<gtl::xh(rect) <<", " <<gtl::yh(rect) <<")" <<std::endl;
      //std::cout <<"poly";
      //for (auto it = shape.begin(); it != shape.end(); it++) {
      //  std::cout <<" (" <<(*it).x() <<", " <<(*it).y() <<")";
      //}
      //std::cout <<std::endl;
      auto pin = std::make_unique<gcPin>(shape, layerNum, this);
      pin->setId(pins[layerNum].size());
      pins[layerNum].push_back(std::move(pin));
    }
    //设置owner
    void setOwner(frBlockObject* in) {
      owner = in;
    }
    void clear() {
      auto size = routePolygons.size();
      routePolygons.clear();
      routePolygons.resize(size);
      routeRectangles.clear();
      routeRectangles.resize(size);
      for (auto &layerPins: pins) {
        layerPins.clear();
      }
    }
    // getters
    const std::vector<gtl::polygon_90_set_data<frCoord> >& getPolygons(bool isFixed = false) const {
      if (isFixed) {
        return fixedPolygons;
      } else {
        return routePolygons;
      }
    }
    const gtl::polygon_90_set_data<frCoord>& getPolygons(frLayerNum layerNum, bool isFixed = false) const {
      if (isFixed) {
        return fixedPolygons[layerNum];
      } else {
        return routePolygons[layerNum];
      }
    }
    const std::vector<std::vector<gtl::rectangle_data<frCoord> > >& getRectangles(bool isFixed = false) const {
      if (isFixed) {
        return fixedRectangles;
      } else {
        return routeRectangles;
      }
    }
    const std::vector<gtl::rectangle_data<frCoord> >& getRectangles(frLayerNum layerNum, bool isFixed = false) const {
      if (isFixed) {
        return fixedRectangles[layerNum];
      } else {
        return routeRectangles[layerNum];
      }
    }
    const std::vector<std::vector<std::unique_ptr<gcPin> > >& getPins() const {
      return pins;
    }
    const std::vector<std::unique_ptr<gcPin> >& getPins(frLayerNum layerNum) const {
      return pins[layerNum];
    }
    std::vector<std::unique_ptr<gcPin> >& getPins(frLayerNum layerNum) {
      return pins[layerNum];
    }
    bool hasOwner() const {
      return (owner);
    }
    frBlockObject* getOwner() const {
      return owner;
    }
    // others
    frBlockObjectEnum typeId() const override {
      return gccNet;
    }
    //void init(int numLayers) {
    //  fixedPolygons.clear();
    //  routePolygons.clear();
    //  pins.clear();
  protected:
    std::vector<gtl::polygon_90_set_data<frCoord> >          fixedPolygons; // only routing layer
    std::vector<gtl::polygon_90_set_data<frCoord> >          routePolygons; // only routing layer
    std::vector<std::vector<gtl::rectangle_data<frCoord> > > fixedRectangles; // only cut layer
    std::vector<std::vector<gtl::rectangle_data<frCoord> > > routeRectangles; // only cut layer
    std::vector<std::vector<std::unique_ptr<gcPin> > >       pins;
    frBlockObject*                                           owner;
    //bool                                               dirty;
    // assisting structures
    //std::vector<frBlockObject*>                        fixedObjs;
    //std::vector<frBlockObject*>                        routeObjs;
    //gtl::polygon_90_set_data<frCoord>                  fixedPs;
    //gtl::polygon_90_set_data<frCoord>                  mergedPs;

    void init();
  };
}

#endif
