#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

const int PIN_SERVO_L = 9;   // 左モータ
const int PIN_SERVO_R = 10;  // 右モータ
const int PIN_SERVO_Z = 11;  // ペン上下

const float LINK_D  = 60.0;   // 左右モータ間の距離 (mm)
const float LINK_1  = 80.0;   // 青部分の長さ (mm)
const float LINK_2  = 100.0;  // ペン先長さ (mm)

const float EXT_X   = 40.0;   // ペン先までのオフセット (mm)
const float EXT_Y   = 0.0;    // 横方向のズレ
// ホーム角度: プログラムが「90度」と命じたとき、サーボが取るべき角度
// アームを真上(90度)に向けた状態で固定し、ズレていればここを微調整します
const int ZERO_OFFSET_L = 0;  // 左サーボのズレ (例: +5, -3 など)
const int ZERO_OFFSET_R = 0;  // 右サーボのズレ

// 回転方向: SG90の取り付け向きによって + / - が逆になる場合の設定
// 1 = 正転, -1 = 逆転
const int DIR_L = 1;  
const int DIR_R = -1; // 右側は鏡像配置なので逆転させることが多い
// ペンの上下
// ペンを上げる/下げる時のサーボ角度
const int PEN_UP_ANGLE   = 90;
const int PEN_DOWN_ANGLE = 0; 
// 文字を書く基準位置
const float HOME_X = LINK_D / 2.0; // 左右中央 (30.0mm)
const float HOME_Y = 150.0;        // モータ軸からの距離 (mm)
#endif