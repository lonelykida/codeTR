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

#ifndef _FR_NET_H_
#define _FR_NET_H_

//#include <boost/graph/adjacency_list.hpp>
#include "frBaseTypes.h"
#include "db/obj/frBlockObject.h"
#include "db/obj/frGuide.h"
#include "db/obj/frShape.h"
#include "db/obj/frVia.h"
#include "db/obj/frNode.h"
#include "db/obj/frRPin.h"
#include "db/grObj/grShape.h"
#include "db/grObj/grVia.h"

namespace fr {
  class frInstTerm;
  class frTerm;

  //线网类
  class frNet: public frBlockObject {
  public:
    // constructors
    //frNet(): frBlockObject() {}
    frNet(const frString &in): frBlockObject(), name(in), instTerms(), terms(), shapes(), vias(), 
                               pwires(), grShapes(), grVias(), nodes(), root(nullptr), rootGCellNode(nullptr), 
                               firstNonRPinNode(nullptr), rpins(), guides(), type(frNetEnum::frcNormalNet), 
                               modified(false), isFakeNet(false) {}
    // getters
    //获取线网的名字
    const frString& getName() const {
      return name;
    }
    //获取线网中的const实例(inst)term队列
    const std::vector<frInstTerm*>& getInstTerms() const {
      return instTerms;
    }
    //获取线网中的实例(inst)term队列
    std::vector<frInstTerm*>& getInstTerms() {
      return instTerms;
    }
    //获取线网中的const term队列
    const std::vector<frTerm*>& getTerms() const {
      return terms;
    }
    //获取线网中的term队列
    std::vector<frTerm*>& getTerms() {
      return terms;
    }
    //获取线网中的shape队列
    std::list<std::unique_ptr<frShape> >& getShapes() {
      return shapes;
    }
    //获取线网中的const shape队列
    const std::list<std::unique_ptr<frShape> >& getShapes() const {
      return shapes;
    }
    //获取线网中的via队列
    std::list<std::unique_ptr<frVia> >& getVias() {
      return vias;
    }
    //获取线网中的const via队列
    const std::list<std::unique_ptr<frVia> >& getVias() const {
      return vias;
    }
    //获取线网中的patchWire队列
    std::list<std::unique_ptr<frShape> >& getPatchWires() {
      return pwires;
    }
    //获取线网中的const patchWire队列
    const std::list<std::unique_ptr<frShape> >& getPatchWires() const {
      return pwires;
    }
    //获取线网中的GRShape队列
    std::list<std::unique_ptr<grShape> >& getGRShapes() {
      return grShapes;
    }
    //获取线网中的const GRShape队列
    const std::list<std::unique_ptr<grShape> >& getGRShapes() const {
      return grShapes;
    }
    //获取线网中的GRVia队列
    std::list<std::unique_ptr<grVia> >& getGRVias() {
      return grVias;
    }
    //获取线网中的const GRVia队列
    const std::list<std::unique_ptr<grVia> >& getGRVias() const {
      return grVias;
    }
    //获取线网中的node队列
    std::list<std::unique_ptr<frNode> >& getNodes() {
      return nodes;
    }
    //获取线网中的const node队列
    const std::list<std::unique_ptr<frNode> >& getNodes() const {
      return nodes;
    }
    //获取线网的根节点
    frNode* getRoot() {
      return root;
    }
    //获取线网的根GCell节点
    frNode* getRootGCellNode() {
      return rootGCellNode;
    }
    //获取线网中的第一个非R Pin节点
    frNode* getFirstNonRPinNode() {
      return firstNonRPinNode;
    }
    //获取线网中的R Pin队列
    std::vector<std::unique_ptr<frRPin> >& getRPins() {
      return rpins;
    }
    //获取线网中的const R Pin队列
    const std::vector<std::unique_ptr<frRPin> >& getRPins() const {
      return rpins;
    }
    //获取线网中的Guide队列
    std::vector<std::unique_ptr<frGuide> >& getGuides() {
      return guides;
    }
    //获取线网中的const Guide队列
    const std::vector<std::unique_ptr<frGuide> >& getGuides() const {
      return guides;
    }
    //获取线网是否被修改
    const bool isModified() const {
      return modified;
    }
    //获取线网是否是虚拟网(FakeNet)
    const bool isFake() const {
      return isFakeNet;
    }

    // setters
    //添加实例(inst)term
    void addInstTerm(frInstTerm* in) {
      instTerms.push_back(in);
    }
    //添加term
    void addTerm(frTerm* in) {
      terms.push_back(in);
    }
    //设置线网的名字
    void setName(const frString &stringIn) {
      name = stringIn;
    }
    //添加dr中的shape
    void addShape(std::unique_ptr<frShape> &in) {
      in->addToNet(this);
      auto rptr = in.get(); //rptr = repeater - 中继器，转发器
      shapes.push_back(std::move(in));
      rptr->setIter(--shapes.end());  //设置repeater的指针
    }
    void addVia(std::unique_ptr<frVia> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      vias.push_back(std::move(in));
      rptr->setIter(--vias.end());
    }
    void addPatchWire(std::unique_ptr<frShape> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      pwires.push_back(std::move(in));
      rptr->setIter(--pwires.end());
    }
    void addGRShape(std::unique_ptr<grShape> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      grShapes.push_back(std::move(in));
      rptr->setIter(--grShapes.end());
    }
    void addGRVia(std::unique_ptr<grVia> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      grVias.push_back(std::move(in));
      rptr->setIter(--grVias.end());
    }
    void addNode(std::unique_ptr<frNode> &in) {
      in->addToNet(this);
      auto rptr = in.get();
      if (nodes.empty()) {
        rptr->setId(0);
      } else {
        rptr->setId(nodes.back()->getId() + 1);
      }
      nodes.push_back(std::move(in));
      rptr->setIter(--nodes.end());
    }
    void setRoot(frNode* in) {
      root = in;
    }
    void setRootGCellNode(frNode* in) {
      rootGCellNode = in;
    }
    void setFirstNonRPinNode(frNode* in) {
      firstNonRPinNode = in;
    }
    void addRPin(std::unique_ptr<frRPin> &in) {
      in->addToNet(this);
      rpins.push_back(std::move(in));      
    }
    void addGuide(std::unique_ptr<frGuide> &in) {
      auto rptr = in.get();
      rptr->addToNet(this);
      guides.push_back(std::move(in));
      //rptr->setIter(--vias.end());
    }
    void clearGuides() {
      guides.clear();
    }
    void removeShape(frShape* in) {
      shapes.erase(in->getIter());
    }
    void removeVia(frVia* in) {
      vias.erase(in->getIter());
    }
    void removePatchWire(frShape* in) {
      pwires.erase(in->getIter());
    }
    void removeGRShape(grShape* in) {
      grShapes.erase(in->getIter());
    }
    void clearGRShapes() {
      grShapes.clear();
    }
    void removeGRVia(grVia *in) {
      grVias.erase(in->getIter());
    }
    void clearGRVias() {
      grVias.clear();
    }
    void removeNode(frNode* in) {
      nodes.erase(in->getIter());
    }
    void setModified(bool in) {
      modified = in;
    }
    void setIsFake(bool in) {
      isFakeNet = in;
    }
    // others
    // 获取线网的类型 - 普通线网、时钟线网、地线网等
    frNetEnum getType() const {
      return type;
    }
    // 设置线网的类型
    void setType(frNetEnum in) {
      type = in;
    }
    // 获取线网的块类型
    virtual frBlockObjectEnum typeId() const override {
      return frcNet;
    }
  protected:
    frString                                  name;       //线网名
    std::vector<frInstTerm*>                  instTerms;  //实例的Term队列
    std::vector<frTerm*>                      terms;     // terms is IO
    // dr
    std::list<std::unique_ptr<frShape> >      shapes;   //dr的形状队列
    std::list<std::unique_ptr<frVia> >        vias;     //dr的通孔列表
    std::list<std::unique_ptr<frShape> >      pwires;   //dr的补丁线(patch wire)列表
    // gr
    std::list<std::unique_ptr<grShape> >      grShapes; //gr的形状队列
    std::list<std::unique_ptr<grVia> >        grVias;   //gr的通孔列表
    //
    std::list<std::unique_ptr<frNode> >       nodes; // the nodes at the beginning of the list correspond to rpins
                                                     // there is no guarantee that first element is root
    frNode*                                   root;
    frNode*                                   rootGCellNode;
    frNode*                                   firstNonRPinNode;
    std::vector<std::unique_ptr<frRPin> >     rpins;
    //std::list<std::unique_ptr<frGuide> >    guides;
    std::vector<std::unique_ptr<frGuide> >    guides;
    frNetEnum                                 type;
    bool                                      modified;
    bool                                      isFakeNet; // indicate floating PG nets
  };
}

#endif
