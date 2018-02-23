#include "Mecanum.hpp"

// 線形変換を表す行列（機体上部から見たホイールの回転方向に対応）
// 機体の各性能・重心に応じて任意に変更（フルで出力したければ1.0）
double g_coefMatrix[4][3] = {{-1.0, 1.0, 0.85},
                             {1.0, 1.0, -0.85},
                             {-1.0, 1.0, -0.85},
                             {1.0, 1.0, 0.85}};

void Mecanum::calculate(int velocityVector[3], int maxOutputRate, double nowAngle, int _pwm[4])
{
    int velocity[3];
    int x = velocityVector[0];
    int y = velocityVector[1];
    
    velocity[0] = x * cos(-nowAngle * M_PI / 180.0) - y * sin(-nowAngle * M_PI / 180.0);
    velocity[1] = x * sin(-nowAngle * M_PI / 180.0) + y * cos(-nowAngle * M_PI / 180.0);
    velocity[2] = velocityVector[2];

    // 行列計算を簡単にするために配列（線形変換前のベクトル）に代入する
    int outputRate[4];

    // スティックの要素と出力倍率や方向を勘案し、各モータの出力値を決める
    for (int motorNum = 0; motorNum < 4; motorNum++)
    {
        // 出力のレートを格納するバッファ変数  [-300.0 - 300.0]
        double bufOutputRate[4] = {0};

        // 理論出力レート値が実際の範囲越えを起こしたかを格納するフラグ変数
        bool isOverflowOfPwm = false;

        // 最大値を格納 [0-100]
        double MaxAbsOutputRate = 0;

        // 行列の積を求める
        for (int motorNum = 0; motorNum < 4; motorNum++)
        {
            for (int element = 0; element < 3; element++)
            {
                // 3つの要素の計算結果の総和をとる
                bufOutputRate[motorNum] += g_coefMatrix[motorNum][element] * velocity[element];
            }

            // 絶対値の最大値を格納
            MaxAbsOutputRate = (abs(bufOutputRate[motorNum]) > MaxAbsOutputRate) ? abs(bufOutputRate[motorNum]) : MaxAbsOutputRate;

            // いずれかのモータが出力倍率を超過していればフラグを立てる
            if (abs(bufOutputRate[motorNum]) > 100)
            {
                isOverflowOfPwm = true;
            }
        }

        // 出力値の最終調整
        for (int motorNum = 0; motorNum < 4; motorNum++)
            _pwm[motorNum] = (isOverflowOfPwm == true) ? constrain((bufOutputRate[motorNum] * (100 / MaxAbsOutputRate)), -100, 100) * maxOutputRate / 100 : bufOutputRate[motorNum] * maxOutputRate / 100;
    }
}
