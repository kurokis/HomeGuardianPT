#include "transition.hpp"

#include "iostream"
using namespace std;

void TRANS::takeTrans(float x, float y, ALGO *algo){
  switch (st){
  case 0:
    algo->tarPos[0] = 0;
    algo->tarPos[1] = 1.5;
    if (y >= 1.45){
      st = 1;
    }
    break;
  case 1:
    algo->tarPos[0] = 6.6;
    algo->tarPos[1] = 1.3;
    if (x >= 6.5){
      st = 2;
    }
    break;
  case 2:
    algo->tarPos[0] = 6.8;
    algo->tarPos[1] = -2.4;
    if (y <= -2.3){
      st = 3;
    }
    break;
  case 3:
    algo->tarPos[0] = 0.3;
    algo->tarPos[1] = -2.4;
    if (x <= 0.4){
      st = 4;
    }
    break;
  case 4:
    algo->tarPos[0] = -0.3;
    algo->tarPos[1] = 0.1;
    if (y >= 0){
      st = 0;
    }
    break;
  }
  cout<<"st: "<<st<<endl;
}

void TRANS::init(){
  st = 0;
}



  
