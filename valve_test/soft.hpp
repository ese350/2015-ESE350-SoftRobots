#include "mbed.h"
#include <string>
#include <vector>
#include <float.h>
#include <unordered_map>

namespace std {
  template <> struct hash<PinName>{
    size_t operator()(const PinName& x) const {
      return x;
    }
  };
}

namespace Soft {
  class Actuator;
  
  typedef std::pair<unsigned int, unsigned int> KeyRange;
  
  namespace Monitor {
    class Monitor {
      public:
        void enable(Actuator *act, float dt);
        void disable();
        virtual void monitor() = 0;
        virtual ~Monitor() {}
      
      protected:
        Actuator *act;
        Ticker ticker;
    };
    
    class Pressure : public Monitor {
      public:
        Pressure(PinName pin, float max = FLT_MAX, float min = FLT_MIN)
            : pMin(min), pMax(max), pMon(pin) {}
        virtual ~Pressure() {}
        virtual void monitor();
        
      protected:
        float pMin, pMax;
        AnalogIn pMon;
    };
  }
  
  class Actuator {
    public:
      Actuator& operator=(PinName pin);
      Actuator& operator+=(Monitor::Pressure *pressure);
      void enable();
      void disable();
    private:
      DigitalOut *dout;
      Monitor::Pressure *pressure;
      Ticker ticker;
  };
  
  class Control {
    public:
      class Frame {
        public:
          Frame(float on, float pre = 0.0, float post = 0.0)
            : onTime(on), preDelay(pre), postDelay(post) {}
            
          Frame& operator=(const Frame *f);
          Frame& operator+=(const Actuator& step);
          
          void run();
        
        private:
          float onTime, preDelay, postDelay;
          std::vector<Actuator> steps;
      };
      
      Frame& operator[](unsigned int idx);
      const Frame& operator[](unsigned int idx) const;
      
      void run();
      void run(unsigned int steps);
      void run(std::string alias);
      void run(unsigned int a, unsigned int b);
          
      std::unordered_map<std::string, KeyRange> alias;
      std::unordered_map<std::string, Actuator> ctl;
      
    protected:
      std::vector<Frame> pnCycle;
  };
}
