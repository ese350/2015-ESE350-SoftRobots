#include "mbed.h"
#include "soft.hpp"
#include <string>
#include <vector>
#include <unordered_map>

typedef typename Soft::Control Control;
typedef typename Soft::Control::Frame Frame;
typedef typename Soft::Monitor::Monitor Monitor;
typedef typename Soft::Monitor::Pressure Pressure;
typedef typename Soft::Actuator Actuator;
    

Frame& Control::operator[](unsigned int idx) {
  if(idx >= pnCycle.size())
    pnCycle.resize(idx+1, 0.0);
  return pnCycle[idx];
}
  
const Frame& Control::operator[](unsigned int idx) const {
  return pnCycle[idx];
}

void Control::run() {
  while(1) {
    for(Frame& frame : pnCycle)
      frame.run();
  }
}

void Control::run(unsigned int steps) {
  for(unsigned int i = 0; i < steps; i++) {
    for(Frame& frame : pnCycle)
      frame.run();
  }
}

void Control::run(std::string name) {
  auto res = alias.find(name);
  if(res == alias.end())
    return;
  KeyRange r = res->second;
  run(r.first, r.second);
}

void Control::run(unsigned int a, unsigned int b) {
  for(unsigned int i = a; i <= b; i++)
    pnCycle[i].run();
}

Frame& Frame::operator=(const Frame *f) {
  *this = *f;
  delete f;
  return *this;
}

Frame& Frame::operator+=(const Actuator& step) {
  steps.push_back(step);
  return *this;
}

void Frame::run() {
  wait(preDelay);
  for(Actuator& step : steps)
    step.enable();
  wait(onTime);
  for(Actuator& step : steps)
    step.disable();
  wait(postDelay);
}

Actuator& Actuator::operator=(PinName pin) {
  if(this->dout)
    delete this->dout;
  this->dout = new DigitalOut(pin);
  return *this;
}

Actuator& Actuator::operator+=(Pressure *p) {
  if(pressure) {
    pressure->disable();
    delete pressure;
  }
  pressure = p;
  p->enable(this, 0.01);
  return *this;
}

void Actuator::enable() {
  *dout = 1;
}

void Actuator::disable() {
  *dout = 0;
}
void Monitor::enable(Actuator *act, float dt) {
  ticker.detach();
  this->act = act;
  ticker.attach(this, &Monitor::monitor, dt);
}
        
void Monitor::disable() {
  ticker.detach();
}

void Pressure::monitor() {
  if(!act)
    return;
    
  float p = 150.0 * pMon.read() / (2 * 9.15);
  if(p < pMin)
    act->enable();
  if(p > pMax)
    act->disable();
}
