#pragma once

#include "encoder.hpp"
//#include "sensor.h"
//#include "motor.h"

class ALGO{
 private:
  float dir[2];

 public:
  float pos[2];
  float tarPos[2];
  void init();

  //void calcTargetVelRH(float senFC, float senFL, float senFR, float senLF, float senLR, float senRF, float senRR);
  void calcTargetVelDR(Encoder *encl, Encoder *encr);

  float tarVelL, tarVelR;
};

