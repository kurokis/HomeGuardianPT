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

  void calcTargetVelRH(float senFC_dist, float senFL_dist, float senFR_dist, float senLF_dist, float senLR_dist, float senRF_dist, float senRR_dist);
  void calcTargetVelDR(Encoder *encl, Encoder *encr);

  float tarVelL, tarVelR;
};

