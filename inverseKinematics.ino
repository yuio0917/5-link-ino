#include <math.h>
#include "Config.h"

void solveIK(float x, float y, float &degL, float &degR) {
  
  // 初期化
  degL = NAN; 
  degR = NAN;

  // リンク長さの調整
  float l_virt = sqrt(pow(LINK_2 + EXT_X, 2) + pow(EXT_Y, 2));
  float phi    = atan2(EXT_Y, LINK_2 + EXT_X);

  // STEP 1: 左側の計算 (ペン先をターゲットにする)
  float dist_L = sqrt(x * x + y * y);

  // 実現不可の場合
  if (dist_L > (LINK_1 + l_virt) || dist_L < abs(LINK_1 - l_virt)) {
    return; // 計算中止
  }

  // 余弦定理 (Cosine Rule)
  // cos(alpha) = (a^2 + b^2 - c^2) / 2ab
  float valL = (pow(x, 2) + pow(y, 2) + pow(LINK_1, 2) - pow(l_virt, 2)) / (2 * LINK_1 * dist_L);
  
  // 数値計算誤差で 1.00001 とかになると acos がエラーになるので制限する
  valL = constrain(valL, -1.0, 1.0); 
  
  float alphaL = acos(valL);   // 肘の内角
  float baseAngleL = atan2(y, x); // ターゲットの方角
  
  float thetaL_rad = baseAngleL + alphaL; // 左モータの角度(ラジアン)
  // ※ロボットの左腕は外側に折れる形状なので「+ alphaL」

  // ==========================================================
  // STEP 2: ボルト関節位置 (Joint) の逆算
  // ==========================================================
  // まず、左肘(Elbow)の座標を出す
  float elbowL_x = LINK_1 * cos(thetaL_rad);
  float elbowL_y = LINK_1 * sin(thetaL_rad);
  
  // 左肘からペン先へのベクトル角度
  float vec_x = x - elbowL_x;
  float vec_y = y - elbowL_y;
  float angle_virt = atan2(vec_y, vec_x);
  
  // プレートの形状ズレを引いて、本来のLink2の角度を出す
  float angle_real = angle_virt - phi;
  
  // これでボルト関節(Joint)の位置が確定
  float jx = elbowL_x + LINK_2 * cos(angle_real);
  float jy = elbowL_y + LINK_2 * sin(angle_real);
  // 右側の計算
  // 右モータ(LINK_D, 0) から 関節J(jx, jy) へのIK
  float rx = jx - LINK_D;
  float ry = jy;
  float dist_R = sqrt(rx * rx + ry * ry);

  // 実現ふかの場合
  if (dist_R > (LINK_1 + LINK_2) || dist_R < abs(LINK_1 - LINK_2)) {
    return; // 計算中止
  }

  float valR = (pow(rx, 2) + pow(ry, 2) + pow(LINK_1, 2) - pow(LINK_2, 2)) / (2 * LINK_1 * dist_R);
  valR = constrain(valR, -1.0, 1.0);
  
  float alphaR = acos(valR);
  float baseAngleR = atan2(ry, rx);
  
  float thetaR_rad = baseAngleR - alphaR;
  // ラジアンー度数法変換
  degL = thetaL_rad * 180.0 / PI;
  degR = thetaR_rad * 180.0 / PI;
}