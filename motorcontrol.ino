#include <Servo.h>
#include "Config.h"

Servo servoL;
Servo servoR;
Servo servoZ;
// 初期化
void initServos() {
  servoL.attach(PIN_SERVO_L);
  servoR.attach(PIN_SERVO_R);
  servoZ.attach(PIN_SERVO_Z);
  penUp();
}
// ペン上下制御
void penUp() {
  servoZ.write(PEN_UP_ANGLE);
  delay(100);
}

void penDown() {
  servoZ.write(PEN_DOWN_ANGLE);
  delay(100);
}

// 位置制御
void movePen(float x, float y) {
  float degL, degR;

  //逆運動学
  solveIK(x, y, degL, degR);

  //計算結果の評価
  if (isnan(degL) || isnan(degR)) {
    return; //無効な結果
  }

  // 基本式: 出力角度 = 90 + (計算角度 - 90) * 回転方向 + ズレ補正
  // ※ SG90は90度が中心なので、そこからの差分で計算します
  
  int outL = 90 + (degL - 90) * DIR_L + ZERO_OFFSET_L;
  int outR = 90 + (degR - 90) * DIR_R + ZERO_OFFSET_R;

  //可動域制限
  outL = constrain(outL, 0, 180);
  outR = constrain(outR, 0, 180);

  //モータに指令を送る
  servoL.write(outL);
  servoR.write(outR);

  //移動待ち時間
  delay(300); 
}

// 設定した初期位置に戻る
void Homeposition() {
  movePen(HOME_X, HOME_Y);
}