//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// @Outline: メカナムホイールの出力値を計算する
// @Author: Ryoga Sato
// @Description:
// 行列を用いた計算を行い、なめらかな移動を実現する
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _STEERING_
#define _STEERING_
#include "Macro.h"

class Steering
{
  public:
    void rotationConversion(double result[2], int x, int y, double angle);
    void calculate(int _velocityVector[3], int maxOutputRate, double nowAngle, int _pwm[4], int _arg[4]);
};

#endif
