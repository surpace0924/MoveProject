#include "Steering.hpp"

void Steering::calculate(int _velocityVector[3], int maxOutputRate, double nowAngle, int _pwm[4], int _arg[4])
{
    int xElement = _velocityVector[0];
    int yElement = _velocityVector[1];
    int spinElement = _velocityVector[2];

    int angularVelocity = map(spinElement, -255, 255, -90, 90);

    double power[4][2] = {0};
    int x0 = xElement - angularVelocity * sin((nowAngle + 45) * M_PI / 180.0);
    int y0 = yElement + angularVelocity * cos((nowAngle + 45) * M_PI / 180.0);

    int x1 = xElement - angularVelocity * cos((nowAngle + 45) * M_PI / 180.0);
    int y1 = yElement - angularVelocity * sin((nowAngle + 45) * M_PI / 180.0);

    int x2 = xElement + angularVelocity * sin((nowAngle + 45) * M_PI / 180.0);
    int y2 = yElement - angularVelocity * cos((nowAngle + 45) * M_PI / 180.0);

    int x3 = xElement + angularVelocity * cos((nowAngle + 45) * M_PI / 180.0);
    int y3 = yElement + angularVelocity * sin((nowAngle + 45) * M_PI / 180.0);

    rotationConversion(power[0], x0, y0, -nowAngle);
    rotationConversion(power[1], x1, y1, -nowAngle);
    rotationConversion(power[2], x2, y2, -nowAngle);
    rotationConversion(power[3], x3, y3, -nowAngle);

    // 出力のガード
    bool isOverflowOfPwm = false;
    double maxPwm = 0;
    int bufPwm[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        _arg[i] = angles(power[i][0], power[i][1]);
        _arg[i] = (_arg[i] < 180) ? _arg[i] : _arg[i] - 180;

        bufPwm[i] = abs(radiuses(abs(power[i][0]), abs(power[i][1])));

        // 最大値を格納
        maxPwm = (bufPwm[i] > maxPwm) ? bufPwm[i] : maxPwm;

        // いずれかのモータが出力倍率を超過していればフラグを立てる
        if (bufPwm[i] > maxOutputRate)
            isOverflowOfPwm = true;
    }

    for (int i = 0; i < 4; i++)
    {
        _pwm[i] = (isOverflowOfPwm) ? map(bufPwm[i], 0, maxPwm, 0, maxOutputRate) : bufPwm[i];
        _pwm[i] = (angles(power[i][0], power[i][1]) < 180) ? _pwm[i] : -1 * _pwm[i];
    }
}

void Steering::rotationConversion(double result[2], int x, int y, double angle)
{
    result[0] = x * cos(angle * M_PI / 180.0) - y * sin(angle * M_PI / 180.0);
    result[1] = x * sin(angle * M_PI / 180.0) + y * cos(angle * M_PI / 180.0);
}
