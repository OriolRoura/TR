#ifndef __COMSSTRUCT__

#define __COMSSTRUCT__

struct ComsStruct {
  private:
    double id;
    float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    double checksum;

  public:
    ComsStruct() {
      clean();
    }

    void clean(){
      set (0,0,0,0,0,0,0,0);
    }
    
    void set (double lId, float lGyroX, float lGyroY, float lGyroZ, float lAccelX, float lAccelY, float lAccelZ, double lChecksum) {
      id = lId;
      gyroX = lGyroX;
      gyroY = lGyroY;
      gyroZ = lGyroZ;
      accelX = lAccelX;
      accelY = lAccelY;
      accelZ = lAccelZ;
      checksum = lChecksum;    
    }

    void get (double *lId, float *lGyroX, float *lGyroY, float *lGyroZ, float *lAccelX, float *lAccelY, float *lAccelZ, double *lChecksum) {
      *lId = id;
      *lGyroX = gyroX;
      *lGyroY = gyroY;
      *lGyroZ = gyroZ;
      *lAccelX = accelX;
      *lAccelY = accelY;
      *lAccelZ = accelZ;
      *lChecksum = checksum;
    }

    double calcChecksum () {
      int i;
      double lchecksum;

      for (i=0; i<4; i++) {
        *((byte *) (&checksum)+i)=*((byte *) (&id)+i)^*((byte *) (&gyroX)+i)^*((byte *) (&gyroY)+i)^*((byte *) (&gyroZ)+i)^*((byte *) (&accelX)+i)^*((byte *) (&accelY)+i)^*((byte *) (&accelZ)+i);
      }
    
      return checksum;
    }

    void setChecksum (double lChecksum) {
      checksum = lChecksum; 
    }

    double getChecksum () {
      return checksum;
    }
    
    bool verifyChecksum () {
      return checksum == calcChecksum();
    }

};

#endif
