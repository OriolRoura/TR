#ifndef __PASBAIX__
#define __PASBAIX__

class PasBaix {
  private:
    float oldMean;
    float alfa;

  public:
    PasBaix (float a) {
      alfa = a;
      oldMean = 0;  
    }

    float calc (float f) {
      oldMean = alfa*f+(1-alfa)*oldMean;
      return oldMean;
    }

    void clean() {
      oldMean = 0;
    }

    void setAlfa(float a) {
      alfa = a;
    };
};

#endif
