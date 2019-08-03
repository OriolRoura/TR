#ifndef __COMSSTRUCT__

#define __COMSSTRUCT__

struct ComsStruct {
  private:
    double id;
    float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    bool lightLeft, lightBreak, ligthRight;


  public:
    ComsStruct() {
      clean();
    }

    void clean(){
      set (0,0,0,0,0,0,0,0,0,0);
    }
    
    void set (double lId, float lGyroX, float lGyroY, float lGyroZ, float lAccelX, float lAccelY, float lAccelZ, bool lLightLeft, bool lLightBreak, bool lLigthRight) {
      id = lId;
      gyroX = lGyroX;
      gyroY = lGyroY;
      gyroZ = lGyroZ;
      accelX = lAccelX;
      accelY = lAccelY;
      accelZ = lAccelZ;
      lightLeft = lLightLeft;
      lightBreak = lLightBreak;
      ligthRight = lLigthRight;
   
    }

    void get (double *lId, float *lGyroX, float *lGyroY, float *lGyroZ, float *lAccelX, float *lAccelY, float *lAccelZ, bool *lLightLeft, bool *lLightBreak, bool *lLigthRight) {
      *lId = id;
      *lGyroX = gyroX;
      *lGyroY = gyroY;
      *lGyroZ = gyroZ;
      *lAccelX = accelX;
      *lAccelY = accelY;
      *lAccelZ = accelZ;
      *lLightLeft = lightLeft;
      *lLightBreak = lightBreak;
      *lLigthRight = ligthRight;
    }



};

#endif
