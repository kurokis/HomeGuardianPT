#include "algo.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#define VEL 50.0
#define TREAD 0.098
#define STRAIGHT 0
#define LEFT 1
#define RIGHT -1

//using namespace std;


void ALGO::calcTargetVelRH(float senFC_dist, float senFL_dist, float senFR_dist, float senLF_dist, float senLR_dist, float senRF_dist, float senRR_dist){
  float curvature, ratio;
  const float thres = 440;
  bool senFC_res, senFL_res, senFR_res, senLF_res, senLR_res, senRF_res, senRR_res;
  if(senFC_dist < thres){senFC_res = true;}else{senFC_res = false;}
  if(senFL_dist < thres){senFL_res = true;}else{senFL_res = false;}
  if(senFR_dist < thres){senFR_res = true;}else{senFR_res = false;}
  if(senLF_dist < thres){senLF_res = true;}else{senLF_res = false;}
  if(senLR_dist < thres){senLR_res = true;}else{senLR_res = false;}
  if(senRF_dist < thres){senRF_res = true;}else{senRF_res = false;}
  if(senRR_dist < thres){senRR_res = true;}else{senRR_res = false;}

  //curvature is positive when turning right
  if (!senRF_res)
    curvature = 4;
  else
    curvature = (senRF_dist - senRR_dist) * 40.0 + ((senRF_dist + senRR_dist)-0.35) * 15.0;

  //ratio = L/R
  if(curvature>0)
    ratio = 1/(1 - 0.1*curvature); //tread width 0.1m
  else
    ratio = 1 + 0.1*curvature; //tread width 0.1m

  tarVelL = VEL * ratio / (ratio + 1.0);
  tarVelR = VEL - tarVelL;

  if(senFC_res && senFC_dist < 0.2){
    if(senFL_res && senFL_dist < 0.2 && senFR_res && senFR_dist < 0.2){
      tarVelR = - VEL / 2.0;
      tarVelL = - VEL / 2.0;
    }
    else if(!senLR_res || senLR_dist > senRR_dist){
      tarVelL = 10.0;
      tarVelR = VEL - tarVelL;
    }
    else{
      tarVelR = 10.0;
      tarVelL = VEL - tarVelR;
    }
  }
}


void ALGO::calcTargetVelDR(Encoder *encl, Encoder *encr){

	/*
  // float prevOdoLF = motLF->odo;
  // float prevOdoLR = motLR->odo;
  // float prevOdoRF = motRF->odo;
  // float prevOdoRR = motRR->odo;



  motLF->prevOdo = motLF->odo;
  motLR->prevOdo = motLR->odo;
  motRF->prevOdo = motRF->odo;
  motRR->prevOdo = motRR->odo;

  motLF->readOdo(clientID);
  motLR->readOdo(clientID);
  motRF->readOdo(clientID);
  motRR->readOdo(clientID);

  float diffOdoLF = -(motLF->odo - motLF->prevOdo);
  float diffOdoLR = -(motLR->odo - motLR->prevOdo);
  float diffOdoRF = -(motRF->odo - motRF->prevOdo);
  float diffOdoRR = -(motRR->odo - motRR->prevOdo);

  if(diffOdoLF < - M_PI)
    diffOdoLF = diffOdoLF+2*M_PI;
  if(diffOdoLR < - M_PI)
    diffOdoLR = diffOdoLR+2*M_PI;
  if(diffOdoRF < - M_PI)
    diffOdoRF = diffOdoRF+2*M_PI;
  if(diffOdoRR < - M_PI)
    diffOdoRR = diffOdoRR+2*M_PI;


  // if (diffOdoLF>=0)
  //   diffOdoLF = fmod(diffOdoLF + M_PI, 2*M_PI)-M_PI;
  // else
  //   diffOdoLF = fmod(diffOdoLF - M_PI, 2*M_PI)+M_PI;

  // if (diffOdoLR>=0)
  //   diffOdoLR = fmod(diffOdoLR + M_PI, 2*M_PI)-M_PI;
  // else
  //   diffOdoLR = fmod(diffOdoLR - M_PI, 2*M_PI)+M_PI;

  // if (diffOdoRF>=0)
  //   diffOdoRF = fmod(diffOdoRF + M_PI, 2*M_PI)-M_PI;
  // else
  //   diffOdoRF = fmod(diffOdoRF - M_PI, 2*M_PI)+M_PI;

  // if (diffOdoRR>=0)
  //   diffOdoRR = fmod(diffOdoRR + M_PI, 2*M_PI)-M_PI;
  // else
  //   diffOdoRR = fmod(diffOdoRR - M_PI, 2*M_PI)+M_PI;

  float OdoL = WHEEL_R * (diffOdoLF + diffOdoLR) / 2;
  float OdoR = WHEEL_R * (diffOdoRF + diffOdoRR) / 2;*/

  float OdoL = encl->deltaMm();
  float OdoR = encr->deltaMm();

  float R = 0.0;
  int curv;
  if (OdoL > OdoR){
    R = (TREAD * OdoL) / (OdoL - OdoR);
    curv = RIGHT; //-1
  }
  else if (OdoL == OdoR){
    curv = STRAIGHT; //0
  }
  else{
    R = (TREAD * OdoR) / (OdoR - OdoL);
    curv = LEFT; //1
  }


  float theta = curv * ((OdoL + OdoR) / 2.0) / (R - TREAD/2.0);

  if(curv > 0){
    pos[0] = pos[0] + (R - TREAD/2) * sin(theta) * dir[0] + (R - TREAD/2) * (1-cos(theta)) * (-1) * dir[1];
    pos[1] = pos[1] + (R - TREAD/2) * sin(theta) * dir[1] + (R - TREAD/2) * (1-cos(theta)) * dir[0];
  }

  else if(curv == 0){
    pos[0] = pos[0] + dir[0] * (OdoL + OdoR) / 2;
    pos[1] = pos[1] + dir[1] * (OdoL + OdoR) / 2;
  }

  else{
    pos[0] = pos[0] + (R - TREAD/2) * sin(-theta) * dir[0] + (R - TREAD/2) * (1-cos(-theta)) * dir[1];
    pos[1] = pos[1] + (R - TREAD/2) * sin(-theta) * dir[1] + (R - TREAD/2) * (1-cos(-theta)) * (-1) * dir[0];
  }


  float temp = dir[0];
  dir[0] = dir[0]*cos(theta) - dir[1]*sin(theta);
  dir[1] = temp*sin(theta) + dir[1]*cos(theta);


  float tarDir[2];
  tarDir[0] = tarPos[0] - pos[0];
  tarDir[1] = tarPos[1] - pos[1];

  //angle of tarDir from dir (tangent)

  float innerProd = (dir[0] * tarDir[0] + dir[1] * tarDir[1]);
  float determinant = (dir[0] * tarDir[1] - dir[1] * tarDir[0]);
  float diffAngTan, curvature;
  if (innerProd > 0){
    diffAngTan = determinant / innerProd;
    curvature  = - std::max<float>(-1.0, std::min<float>(diffAngTan, 1.0)) * 6;
  }
  else if (determinant > 0){
    curvature = -6;
  }
  else{
    curvature = 6;
  }

  float ratio;
  if(curvature>0)
    ratio = 1/(1 - 0.1*curvature); //tread width 0.1m
  else
    ratio = 1 + 0.1*curvature; //tread width 0.1m

  tarVelL = VEL * ratio / (ratio + 1.0);
  tarVelR = VEL - tarVelL;
  //  tarVelL = 5.0;
  //  tarVelR = 5.0;

  /*
  cout<<"pos   : "<<pos[0]<<' '<<pos[1]<<endl;
  cout<<"tarpos: "<<tarPos[0]<<' '<<tarPos[1]<<endl;
  cout<<"theta : "<<theta<<endl;
  cout<<"dir   : "<<dir[0]<<' '<<dir[1]<<endl;
  cout<<"tardir: "<<tarDir[0]<<' '<<tarDir[1]<<endl;
  cout<<"diffAngTan: "<<diffAngTan<<endl;
  cout<<"prevodo: "<<motLF->prevOdo<<' '<<motLR->prevOdo<<' '<<motRF->prevOdo<<' '<<motRR->prevOdo<<endl;
  cout<<"odo    : "<<motLF->odo<<' '<<motLR->odo<<' '<<motRF->odo<<' '<<motRR->odo<<endl;
  //  cout<<"diffodo: "<<diffOdoLF<<' '<<diffOdoLR<<' '<<diffOdoRF<<' '<<diffOdoRR<<endl;
  cout<<"diffodo: "<<OdoL<<' '<<OdoR<<endl;
  cout<<"curvature: "<<curvature<<' '<<endl;
  cout<<"tarVel   : "<<tarVelL<<' '<<tarVelR<<endl;
  cout<<"curv, R: "<<curv<<' '<<R<<endl;
  cout<<endl;
  */


}


void ALGO::init(){
  dir[0] = 0;
  dir[1] = 1;
  pos[0] = 0;
  pos[1] = 0;
  tarPos[0] = 0;
  tarPos[1] = 1;
}
