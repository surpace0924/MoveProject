//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// @Outline: メカナムホイールの出力値を計算する
// @Author: Ryoga Sato
// @Description:
// 行列を用いた計算を行い、なめらかな移動を実現する
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _MECANUM_
#define _MECANUM_
#include <Arduino.h>
#include "Macro.h"

class Mecanum
{
  public:
    void calculate(int velocityVector[3], int maxOutputRate, double nowAngle, int absOutputRate[4]);

  private:
};

#endif
