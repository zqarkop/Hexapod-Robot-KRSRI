//Udah Paling Bener
#include <SoftwareSerial.h>

namespace legs {
bool isNormalized = false;
bool isStandby = false;
bool kakiNapak = false;
bool diamSemprot = false;
float sensitivitasMPU = 0.5;
float konstantaKalibrasiJarak = 176.27;

enum MoveType {
  STANDBY,
  NORMALIZE,
  FORWARD,
  FORWARD_LOW,
  FORWARD_HIGHER,
  CAPIT_TURUN,
  CAPIT_ANGKAT,
  CAPIT_LEPAS,
  GYRO_VERTIKAL,
  GYRO_HORIZONTAL,
  JALAN_KEPITING,
  JALAN_KEPITING_MIRROR,
  BACKWARD,
  SHIFT_RIGHT,
  SHIFT_LEFT,
  SHIFT_LEFTHIGHER,
  ROTATE_CW,
  ROTATE_CCW,
  ROTATE_CW_LESS,
  ROTATE_CCW_LESS,
  TURN_RIGHT,
  TURN_LEFT,
  SHIFT_RIGHT_LESS,
  SHIFT_LEFT_LESS,
  ROTATE_CWKEPITING,
  ROTATE_CCWKEPITING,
  FORWARD_GYRO,
  SEMPROT_GOYANG,
  BACKWARD_KORBAN,
  SHIFT_LEFT_KORBAN,
  SHIFT_RIGHT_KORBAN,
  ROTATE_CCW_KORBAN,
  ROTATE_CW_KORBAN,
  CAPIT_NAIK
};

MoveType state_currentMove = NORMALIZE;
unsigned short int state_step2keep = 0;
unsigned long int state_lastMoveRecord = 0;
unsigned short int state_nextStep = 0;
bool state_isComboAUp = true;
bool state_isComboBUp = true;
unsigned short int state_endStep = 3;

SoftwareSerial com(19, 18); // RX, TX

void setup () {
  com.begin(9600);
}

int aTetaL1 = -370; int bTetaL1 = 2370;
int aBetaL1 = 490; int bBetaL1 = 1480;
int aAlphaL1 = -450; int bAlphaL1 = 2350;

int aTetaL2 = -220; int bTetaL2 = 1550;
int aBetaL2 = 470; int bBetaL2 = 1420;
int aAlphaL2 = -460; int bAlphaL2 = 1900;

int aTetaL3 = 370; int bTetaL3 = 980;
int aBetaL3 = 470; int bBetaL3 = 1850;
int aAlphaL3 = -380; int bAlphaL3 = 2690;

int aTetaR1 = 430; int bTetaR1 = 1300;
int aBetaR1 = -420; int bBetaR1 = 1260;
int aAlphaR1 = 440; int bAlphaR1 = 560;

int aTetaR2 = 310; int bTetaR2 = 1350;
int aBetaR2 = -450; int bBetaR2 = 1340;
int aAlphaR2 = 470; int bAlphaR2 = 120;

int aTetaR3 = -440; int bTetaR3 = 1450;
int aBetaR3 = -500; int bBetaR3 = 1380;
int aAlphaR3 = 500; int bAlphaR3 = 570;
 
  float kakiNaik = -5;//max 5 tadinya 5.5
  float kakiTurun = -9;//max 13 tadinya 11 Tengahnya 9

  float calL1R3 = 1;
  
  float calL1x = 0;
  float calL1y = 0;
  float calL1z = 0;
  
  float calL2x = 0;
  float calL2y = 0;
  float calL2z = 0;
  
  float calL3x = 0;
  float calL3y = 0;
  float calL3z = 0;
  
  float calR1x = 0;
  float calR1y = 0;
  float calR1z = 0;
  
  float calR2x = 0;
  float calR2y = 0;
  float calR2z = 0;
  
  float calR3x = 0;
  float calR3y = 0;
  float calR3z = 0;
  

float getR(float y, float x){
  return sqrt(pow(y, 2) + pow(x, 2));
}

float getTeta(float y, float x){
  return atan(y/x);
}

float getP(float r, float c){
  return (r - c);
}

float getU(float p, float z){
  return sqrt(pow(p, 2) + pow(z, 2));
}

float getAlpha (float t, float f, float u) {
  return acos((pow(t, 2) + pow(f, 2) - pow(u, 2))/(2 * t * f));
}

float getGama (float p, float z) {
  return atan(-z/p);
}

float getBeta (float t, float f, float u, float alpha, float gama) {
  return acos((pow(u, 2) + pow(f, 2) - pow(t, 2))/(2 * f * u)) - gama;
}

float getAngularOfCartesianTeta (float x, float y, float z) {
  float r = getR(y, x);
  float teta = getTeta(y, x);
  float p = getP(r, 3.5);
  float u = getU(p, z);
  float alpha = getAlpha(9.5, 5, u);
  float gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
  float beta = getBeta(9.5, 5, u, alpha, gama);

  return teta; 
}

float getAngularOfCartesianBeta (float x, float y, float z) {
  float r = getR(y, x);
  float teta = getTeta(y, x);
  float p = getP(r, 3.5);
  float u = getU(p, z);
  float alpha = getAlpha(9.5, 5, u);
  float gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
  float beta = getBeta(9.5, 5, u, alpha, gama);

  return beta; 
}

float getAngularOfCartesianAlpha (float x, float y, float z) {
  float r = getR(y, x);
  float teta = getTeta(y, x);
  float p = getP(r, 3.5);
  float u = getU(p, z);
  float alpha = getAlpha(9.5, 5, u);
  float gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
  float beta = getBeta(9.5, 5, u, alpha, gama);

  return alpha; 
}

float getAngularOfCartesianTetaR2R3 (float x, float y, float z) {
  float r = getR(y, x);
  float teta = getTeta(y, x);
  float p = getP(r, 3.5);
  float u = getU(p, z);
  float alpha = getAlpha(9.5, 5, u);
  float gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
  float beta = getBeta(9.5, 5, u, alpha, gama);

  return teta; 
}

float getAngularOfCartesianBetaR2R3 (float x, float y, float z) {
  float r = getR(y, x);
  float teta = getTeta(y, x);
  float p = getP(r, 3.5);
  float u = getU(p, z);
  float alpha = getAlpha(9.5, 5, u);
  float gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
  float beta = getBeta(9.5, 5, u, alpha, gama);

  return beta; 
}

float getAngularOfCartesianAlphaR2R3 (float x, float y, float z) {
  float r = getR(y, x);
  float teta = getTeta(y, x);
  float p = getP(r, 3.5);
  float u = getU(p, z);
  float alpha = getAlpha(9.5, 5, u);
  float gama = p > 0 ? getGama(p, z) : M_PI + getGama(p, z);
  float beta = getBeta(9.5, 5, u, alpha, gama);

  return alpha; 
}

float radian2Deg (float rad) {
  return rad * 180 / M_PI;
}

float convert (float teta, float inDeg, float inIO, float offset, bool reversed) {
  float result = (inIO/inDeg) * teta;
  return reversed ? offset - result : offset + result;
}

float getPitch(){
  for(int i = 0; i<30; i++){
    mpu.update();
  }
  return mpu.getAngleX()*-1;
}
float getRoll(){
  for(int i = 0; i<30; i++){
    mpu.update();
  }
  return mpu.getAngleY();
}

String konversi_rotate_cw_korban(int x){
  kakiTurun = -11;
  kakiNaik = -5.5;
  
  float L1[4][3] = {
    {6.5 + calL1x, 3.5 + calL1y, kakiTurun + calL1z},
    {6.5 + calL1x, 3.5 + calL1y, kakiTurun + calL1z},
    {5.5 + calL1x, 4.5 + calL1y, kakiNaik + calL1z},
    {5.5 + calL1x, 4.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, 0.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, 0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -0.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {5.5 + calL3x, 4.5 + calL3y, kakiTurun + calL3z},
    {5.5 + calL3x, 4.5 + calL3y, kakiTurun + calL3z},
    {6.5 + calL3x, 3.5 + calL3y, kakiNaik + calL3z},
    {6.5 + calL3x, 3.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6.5 + calR1x, 3.5 + calR1y, kakiNaik + calR1z},
    {6.5 + calR1x, 3.5 + calR1y, kakiTurun + calR1z},
    {5.5 + calR1x, 4.5 + calR1y, kakiTurun + calR1z},
    {5.5 + calR1x, 4.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, 0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -0.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, -0.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {5.5 + calR3x, 4.5 + calR3y, kakiNaik + calR3z},
    {5.5 + calR3x, 4.5 + calR3y, kakiTurun + calR3z},
    {6.5 + calR3x, 3.5 + calR3y, kakiTurun + calR3z},
    {6.5 + calR3x, 3.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T250";
  return hasil;
}

String konversi_rotate_ccw_korban(int x){
  kakiTurun = -11;
  kakiNaik = -5.5;
  
  float L1[4][3] = {
    {5.5 + calL1x, 4.5 + calL1y, kakiTurun + calL1z},
    {5.5 + calL1x, 4.5 + calL1y, kakiTurun + calL1z},
    {6.5 + calL1x, 3.5 + calL1y, kakiNaik + calL1z},
    {6.5 + calL1x, 3.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, -0.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, -0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6.5 + calL3x, 3.5 + calL3y, kakiTurun + calL3z},
    {6.5 + calL3x, 3.5 + calL3y, kakiTurun + calL3z},
    {5.5 + calL3x, 4.5 + calL3y, kakiNaik + calL3z},
    {5.5 + calL3x, 4.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {5.5 + calR1x, 4.5 + calR1y, kakiNaik + calR1z},
    {5.5 + calR1x, 4.5 + calR1y, kakiTurun + calR1z},
    {6.5 + calR1x, 3.5 + calR1y, kakiTurun + calR1z},
    {6.5 + calR1x, 3.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, 0.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6.5 + calR3x, 3.5 + calR3y, kakiNaik + calR3z},
    {6.5 + calR3x, 3.5 + calR3y, kakiTurun + calR3z},
    {5.5 + calR3x, 4.5 + calR3y, kakiTurun + calR3z},
    {5.5 + calR3x, 4.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T250";
  return hasil;
}

String konversi_capitAngkat(int x){  
  float L1[4][3] = {
    {6 + calL1x, 2.5 + calL1y, (-6 + calL1z) * calL1R3},
    {6 + calL1x, 2.5 + calL1y, (-6 + calL1z) * calL1R3},
    {6 + calL1x, 5.5 + calL1y, (-6 + calL1z) * calL1R3},
    {6 + calL1x, 5.5 + calL1y, (-6 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6 + calL2x,  1.5 + calL2y, -6 + calL2z},
    {6.6 + calL2x,  1.5 + calL2y, -6 + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, -6 + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, -6 + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 5.5 + calL3y, -6 + calL3z},
    {6 + calL3x, 5.5 + calL3y, -6 + calL3z},
    {6 + calL3x, 2.5 + calL3y, -6 + calL3z},
    {6 + calL3x, 2.5 + calL3y, -6 + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 5.5 + calR1y, -6 + calR1z},
    {6 + calR1x, 5.5 + calR1y, -6 + calR1z},
    {6 + calR1x, 2.5 + calR1y, -6 + calR1z},
    {6 + calR1x, 2.5 + calR1y, -6 + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -1.5 + calR2y, -6 + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, -6 + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, -6 + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, -6 + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 2.5 + calR3y, (-6 + calR3z) * calL1R3},
    {6 + calR3x, 2.5 + calR3y, (-6 + calR3z) * calL1R3},
    {6 + calR3x, 5.5 + calR3y, (-6 + calR3z) * calL1R3},
    {6 + calR3x, 5.5 + calR3y, (-6 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" #11P1340 #23P1100 T200";
  return hasil;
}

String konversi_backwardKorban(int x){  
  float mpuL1 = -11;
  float mpuL2 = -11;
  float mpuL3 = -11;
  float mpuR1 = -11;
  float mpuR2 = -11;
  float mpuR3 = -11;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  fx = 0;
  fy = 0;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {6-travel + calL1x, 4.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6-travel + calL1x, 4.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6+travel + calL1x, 3.5 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {6+travel + calL1x, 3.5 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6+travel + calL2x, -0.5 + calL2y, kakiNaik + calL2z},
    {6.6+travel + calL2x, -0.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0.5 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {6-travel + calL3x, 3.5 + calL3y, zL3 + calL3z},
    {6-travel + calL3x, 3.5 + calL3y, zL3 + calL3z},
    {6+travel + calL3x, 4.5 + calL3y, kakiNaik + calL3z},
    {6+travel + calL3x, 4.5 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {6-travel + calR1x, 3.5 + calR1y, kakiNaik + calR1z},
    {6-travel + calR1x, 3.5 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 4.5 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 4.5 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6+travel + calR2x, 0.5 + calR2y, zR2 + calR2z},
    {6.6+travel + calR2x, 0.5 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, -0.5 + calR2y, kakiNaik + calR2z},
    {6.6-travel + calR2x, -0.5 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {6-travel + calR3x, 4.5 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {6-travel + calR3x, 4.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 3.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 3.5 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T250";
  return hasil;
}

String konversi_shiftLeftKorban(int x){  
  float mpuL1 = -11;
  float mpuL2 = -11;
  float mpuL3 = -11;
  float mpuR1 = -11;
  float mpuR2 = -11;
  float mpuR3 = -11;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  fx = 0;
  fy = 0;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {6-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {6-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {7+travel + calL1x, 4 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {7+travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {7.6+travel + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {7.6+travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {6-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {6-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {7+travel + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {7+travel + calL3x, 4 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {6-travel + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {6-travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {7+travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {7+travel + calR1x, 4 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {7.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {7.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {6.6-travel + calR2x, 0 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {6-travel + calR3x, 4 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {6-travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {7+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {7+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T250";
  return hasil;
}

String konversi_shiftRightKorban(int x){  
  float mpuL1 = -11;
  float mpuL2 = -11;
  float mpuL3 = -11;
  float mpuR1 = -11;
  float mpuR2 = -11;
  float mpuR3 = -11;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  fx = 0;
  fy = 0;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {7-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {7-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {6+travel + calL1x, 4 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {6+travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6+travel + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {6.6+travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {7.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {7.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {7-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {7-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {6+travel + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {6+travel + calL3x, 4 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {7-travel + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {7-travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 4 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {6.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {7.6-travel + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {7.6-travel + calR2x, 0 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {7-travel + calR3x, 4 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {7-travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T250";
  return hasil;
}

String konversi_SemprotGoyang(int x){
  float mpuL1 = -8;
  float mpuL2 = -8;
  float mpuL3 = -8;
  float mpuR1 = -8;
  float mpuR2 = -8;
  float mpuR3 = -8;
  float kakiNaik = -8;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  if(!diamSemprot){
    fx = getRoll()/15*2.5*sensitivitasMPU;
    fy = getPitch()/15*2.5*sensitivitasMPU;
    diamSemprot = true;
  }
  fx = 0;
  fy = 0;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {7.5-travel + calL1x, 2.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {4.5+travel + calL1x, 5.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6+travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6+travel + calL2x, -1.5 + calL2y, zL2 + calL2z},
    {6.6+travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 1.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {4.5-travel + calL3x, 5.5 + calL3y, zL3 + calL3z},
    {6-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {7.5+travel + calL3x, 2.5 + calL3y, zL3 + calL3z},
    {6+travel + calL3x, 4 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {4.5-travel + calR1x, 5.5 + calR1y, zR1 + calR1z},
    {6-travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {7.5+travel + calR1x, 2.5 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 4 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6+travel + calR2x, 1.5 + calR2y, zR2 + calR2z},
    {6.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, -1.5 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, 0 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {7.5-travel + calR3x, 2.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6-travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {4.5+travel + calR3x, 5.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_ForwardGyro(int x){  
  float mpuL1 = -9;
  float mpuL2 = -9;
  float mpuL3 = -9;
  float mpuR1 = -9;
  float mpuR2 = -9;
  float mpuR3 = -9;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  fx = getRoll()/15*2.5*sensitivitasMPU;
  fy = getPitch()/15*2.5*sensitivitasMPU;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {6-travel + calL1x, 2.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6-travel + calL1x, 2.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6+travel + calL1x, 5.5 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {6+travel + calL1x, 5.5 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6+travel + calL2x, 1.5 + calL2y, kakiNaik + calL2z},
    {6.6+travel + calL2x, 1.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, -1.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, -1.5 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {6-travel + calL3x, 5.5 + calL3y, zL3 + calL3z},
    {6-travel + calL3x, 5.5 + calL3y, zL3 + calL3z},
    {6+travel + calL3x, 2.5 + calL3y, kakiNaik + calL3z},
    {6+travel + calL3x, 2.5 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {6-travel + calR1x, 5.5 + calR1y, kakiNaik + calR1z},
    {6-travel + calR1x, 5.5 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 2.5 + calR1y, zR1 + calR1z},
    {6+travel + calR1x, 2.5 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6+travel + calR2x, -1.5 + calR2y, zR2 + calR2z},
    {6.6+travel + calR2x, -1.5 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, 1.5 + calR2y, kakiNaik + calR2z},
    {6.6-travel + calR2x, 1.5 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {6-travel + calR3x, 2.5 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {6-travel + calR3x, 2.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 5.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6+travel + calR3x, 5.5 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_rotateCWKepiting(int x){  
  float mpuL1 = -9;
  float mpuL2 = -9;
  float mpuL3 = -9;
  float mpuR1 = -9;
  float mpuR2 = -9;
  float mpuR3 = -9;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  fx = 0;
  fy = getPitch()/15*2.5*sensitivitasMPU;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {6.5-travel + calL1x, 3.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6.5-travel + calL1x, 3.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {5.5+travel + calL1x, 4.5 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {5.5+travel + calL1x, 4.5 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6+travel + calL2x, 0.5 + calL2y, kakiNaik + calL2z},
    {6.6+travel + calL2x, 0.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, -0.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, -0.5 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {5.5-travel + calL3x, 4.5 + calL3y, zL3 + calL3z},
    {5.5-travel + calL3x, 4.5 + calL3y, zL3 + calL3z},
    {6.5+travel + calL3x, 3.5 + calL3y, kakiNaik + calL3z},
    {6.5+travel + calL3x, 3.5 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {6.5-travel + calR1x, 3.5 + calR1y, kakiNaik + calR1z},
    {6.5-travel + calR1x, 3.5 + calR1y, zR1 + calR1z},
    {5.5+travel + calR1x, 4.5 + calR1y, zR1 + calR1z},
    {5.5+travel + calR1x, 4.5 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6+travel + calR2x, 0.5 + calR2y, zR2 + calR2z},
    {6.6+travel + calR2x, 0.5 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, -0.5 + calR2y, kakiNaik + calR2z},
    {6.6-travel + calR2x, -0.5 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {5.5-travel + calR3x, 4.5 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {5.5-travel + calR3x, 4.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6.5+travel + calR3x, 3.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {6.5+travel + calR3x, 3.5 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_rotateCCWKepiting(int x){  
  float mpuL1 = -9;
  float mpuL2 = -9;
  float mpuL3 = -9;
  float mpuR1 = -9;
  float mpuR2 = -9;
  float mpuR3 = -9;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  fx = 0;
  fy = getPitch()/15*2.5*sensitivitasMPU;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-12){zL1 = -12;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-12){zL2 = -12;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-12){zL3 = -12;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-12){zR1 = -12;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-12){zR2 = -12;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-12){zR3 = -12;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 0;
  
  float L1[4][3] = {
    {5.5-travel + calL1x, 4.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {5.5-travel + calL1x, 4.5 + calL1y, (zL1 + calL1z) * calL1R3},
    {6.5+travel + calL1x, 3.5 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {6.5+travel + calL1x, 3.5 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6+travel + calL2x, -0.5 + calL2y, kakiNaik + calL2z},
    {6.6+travel + calL2x, -0.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0.5 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0.5 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {6.5-travel + calL3x, 3.5 + calL3y, zL3 + calL3z},
    {6.5-travel + calL3x, 3.5 + calL3y, zL3 + calL3z},
    {5.5+travel + calL3x, 4.5 + calL3y, kakiNaik + calL3z},
    {5.5+travel + calL3x, 4.5 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {5.5-travel + calR1x, 4.5 + calR1y, kakiNaik + calR1z},
    {5.5-travel + calR1x, 4.5 + calR1y, zR1 + calR1z},
    {6.5+travel + calR1x, 3.5 + calR1y, zR1 + calR1z},
    {6.5+travel + calR1x, 3.5 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6+travel + calR2x, -0.5 + calR2y, zR2 + calR2z},
    {6.6+travel + calR2x, -0.5 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, 0.5 + calR2y, kakiNaik + calR2z},
    {6.6-travel + calR2x, 0.5 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {6.5-travel + calR3x, 3.5 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {6.5-travel + calR3x, 3.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {5.5+travel + calR3x, 4.5 + calR3y, (zR3 + calR3z) * calL1R3},
    {5.5+travel + calR3x, 4.5 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_JalanKepitingMirror(int x){
  float mpuL1 = -9;
  float mpuL2 = -9;
  float mpuL3 = -9;
  float mpuR1 = -9;
  float mpuR2 = -9;
  float mpuR3 = -9;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  
  fy = getPitch()/15*2.5*sensitivitasMPU;
  if(fy < 0){
    fx = getRoll()/15*2.5*sensitivitasMPU;
  }
  else{
    fx = 0;
  }
//  fx = 0;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-11){zL1 = -11;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-11){zL2 = -11;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-11){zL3 = -11;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-11){zR1 = -11;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-11){zR2 = -11;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-11){zR3 = -11;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 1;
  
  float L1[4][3] = {
    {7+travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {7+travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {6-travel + calL1x, 4 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {6-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6-travel + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {6.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {7.6+travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {7.6+travel + calL2x, 0 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {7+travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {7+travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {6-travel + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {6-travel + calL3x, 4 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {7+travel + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {7+travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {6-travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {6-travel + calR1x, 4 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {6.6-travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {7.6+travel + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {7.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {7+travel + calR3x, 3 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {7+travel + calR3x, 3 + calR3y, (zR3 + calR3z) * calL1R3},
    {6-travel + calR3x, 3 + calR3y, (zR3 + calR3z) * calL1R3},
    {6-travel + calR3x, 3 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_JalanKepiting(int x){
  float mpuL1 = -9;
  float mpuL2 = -9;
  float mpuL3 = -9;
  float mpuR1 = -9;
  float mpuR2 = -9;
  float mpuR3 = -9;
  float kakiNaik = -5.5;
  float fx,fy,zL1,zL2,zL3,zR1,zR2,zR3;
  
//  fx = getPitch()/17*5*sensitivitasMPU;
//  fy = getRoll()/10*3*sensitivitasMPU;
  
  fy = getPitch()/15*2.5*sensitivitasMPU;
  if(fy < 0){
    fx = getRoll()/15*2.5*sensitivitasMPU;
  }
  else{
    fx = 0;
  }
//  fx = 0;
  zL1 = mpuL1 + fx + fy;
  if(zL1<-11){zL1 = -11;}
  if(zL1>-6){zL1 = -6;}
  zL1 = zL1*1;
  zL2 = mpuL2 + fy;
  if(zL2<-11){zL2 = -11;}
  if(zL2>-6){zL2 = -6;}
  zL3 = mpuL3 - fx + fy;
  zL3 = zL3*1;
  if(zL3<-11){zL3 = -11;}
  if(zL3>-6){zL3 = -6;}
  
  zR1 = mpuR1 + fx - fy;
  zR1 = zR1*1;
  if(zR1<-11){zR1 = -11;}
  if(zR1>-6){zR1 = -6;}
  zR2 = mpuR2 - fy;
  if(zR2<-11){zR2 = -11;}
  if(zR2>-6){zR2 = -6;}
  zR3 = mpuR3 - fx - fy; 
  if(zR3<-11){zR3 = -11;}
  if(zR3>-6){zR3 = -6;}
  zR3 = zR3*1;

  float travel = 1;
  
  float L1[4][3] = {
    {6-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {6-travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
    {7+travel + calL1x, 4 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {7+travel + calL1x, 4 + calL1y, (zL1 + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {7.6+travel + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {7.6+travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
    {6.6-travel + calL2x, 0 + calL2y, zL2 + calL2z},
  };
  float L3[4][3] = {
    {6-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {6-travel + calL3x, 4 + calL3y, zL3 + calL3z},
    {7+travel + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {7+travel + calL3x, 4 + calL3y, zL3 + calL3z},
  };
  float R1[4][3] = {
    {6-travel + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {6-travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {7+travel + calR1x, 4 + calR1y, zR1 + calR1z},
    {7+travel + calR1x, 4 + calR1y, zR1 + calR1z},
  };
  float R2[4][3] = {
    {7.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {7.6+travel + calR2x, 0 + calR2y, zR2 + calR2z},
    {6.6-travel + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {6.6-travel + calR2x, 0 + calR2y, zR2 + calR2z},
  };
  float R3[4][3] = {
    {6-travel + calR3x, 4 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {6-travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {7+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
    {7+travel + calR3x, 4 + calR3y, (zR3 + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_forward(int x){  
  float L1[4][3] = {
    {6 + calL1x, 2.5 + calL1y, (kakiTurun + calL1z) * calL1R3},
    {6 + calL1x, 2.5 + calL1y, (kakiTurun + calL1z) * calL1R3},
    {6 + calL1x, 5.5 + calL1y, (kakiNaik + calL1z) * calL1R3},
    {6 + calL1x, 5.5 + calL1y, (kakiTurun + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6 + calL2x,  1.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x,  1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 2.5 + calL3y, kakiNaik + calL3z},
    {6 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 5.5 + calR1y, kakiNaik + calR1z},
    {6 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 2.5 + calR3y, (kakiNaik + calR3z) * calL1R3},
    {6 + calR3x, 2.5 + calR3y, (kakiTurun + calR3z) * calL1R3},
    {6 + calR3x, 5.5 + calR3y, (kakiTurun + calR3z) * calL1R3},
    {6 + calR3x, 5.5 + calR3y, (kakiTurun + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_standby(int x){  
  float L1[4][3] = {
    {6 + calL1x, 4 + calL1y, (kakiTurun + calL1z) * calL1R3},
    {6 + calL1x, 4 + calL1y, (kakiTurun + calL1z) * calL1R3},
    {6 + calL1x, 4 + calL1y, (kakiTurun + calL1z) * calL1R3},
    {6 + calL1x, 4 + calL1y, (kakiTurun + calL1z) * calL1R3},
  };
  float L2[4][3] = {
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 4 + calR3y, (kakiTurun + calR3z) * calL1R3},
    {6 + calR3x, 4 + calR3y, (kakiTurun + calR3z) * calL1R3},
    {6 + calR3x, 4 + calR3y, (kakiTurun + calR3z) * calL1R3},
    {6 + calR3x, 4 + calR3y, (kakiTurun + calR3z) * calL1R3},
  };
  
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_rotate_cw(int x){
  float L1[4][3] = {
    {7.5 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
    {7.5 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
    {4.5 + calL1x, 5.5 + calL1y, kakiNaik + calL1z},
    {4.5 + calL1x, 5.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, 1.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, 1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {4.5 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {4.5 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {7.5 + calL3x, 2.5 + calL3y, kakiNaik + calL3z},
    {7.5 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {7.5 + calR1x, 2.5 + calR1y, kakiNaik + calR1z},
    {7.5 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
    {4.5 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {4.5 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, 1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {4.5 + calR3x, 5.5 + calR3y, kakiNaik + calR3z},
    {4.5 + calR3x, 5.5 + calR3y, kakiTurun + calR3z},
    {7.5 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
    {7.5 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_rotate_ccw(int x){
  float L1[4][3] = {
    {4.5 + calL1x, 5.5 + calL1y, kakiTurun + calL1z},
    {4.5 + calL1x, 5.5 + calL1y, kakiTurun + calL1z},
    {7.5 + calL1x, 2.5 + calL1y, kakiNaik + calL1z},
    {7.5 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, -1.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 1.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {7.5 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
    {7.5 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
    {4.5 + calL3x, 5.5 + calL3y, kakiNaik + calL3z},
    {4.5 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {4.5 + calR1x, 5.5 + calR1y, kakiNaik + calR1z},
    {4.5 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {7.5 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
    {7.5 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 1.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, 1.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {7.5 + calR3x, 2.5 + calR3y, kakiNaik + calR3z},
    {7.5 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
    {4.5 + calR3x, 5.5 + calR3y, kakiTurun + calR3z},
    {4.5 + calR3x, 5.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_backward(int x){
  float L1[4][3] = {
    {6 + calL1x, 5.5 + calL1y, kakiTurun + calL1z},
    {6 + calL1x, 5.5 + calL1y, kakiTurun + calL1z},
    {6 + calL1x, 2.5 + calL1y, kakiNaik + calL1z},
    {6 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, -1.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x,  1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x,  1.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 5.5 + calL3y, kakiNaik + calL3z},
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 2.5 + calR1y, kakiNaik + calR1z},
    {6 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x,  1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 5.5 + calR3y, kakiNaik + calR3z},
    {6 + calR3x, 5.5 + calR3y, kakiTurun + calR3z},
    {6 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
    {6 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_rotate_cw_less(int x){
  float L1[4][3] = {
    {6.5 + calL1x, 3.5 + calL1y, kakiTurun + calL1z},
    {6.5 + calL1x, 3.5 + calL1y, kakiTurun + calL1z},
    {5.5 + calL1x, 4.5 + calL1y, kakiNaik + calL1z},
    {5.5 + calL1x, 4.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, 0.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, 0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -0.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {5.5 + calL3x, 4.5 + calL3y, kakiTurun + calL3z},
    {5.5 + calL3x, 4.5 + calL3y, kakiTurun + calL3z},
    {6.5 + calL3x, 3.5 + calL3y, kakiNaik + calL3z},
    {6.5 + calL3x, 3.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6.5 + calR1x, 3.5 + calR1y, kakiNaik + calR1z},
    {6.5 + calR1x, 3.5 + calR1y, kakiTurun + calR1z},
    {5.5 + calR1x, 4.5 + calR1y, kakiTurun + calR1z},
    {5.5 + calR1x, 4.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, 0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -0.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, -0.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {5.5 + calR3x, 4.5 + calR3y, kakiNaik + calR3z},
    {5.5 + calR3x, 4.5 + calR3y, kakiTurun + calR3z},
    {6.5 + calR3x, 3.5 + calR3y, kakiTurun + calR3z},
    {6.5 + calR3x, 3.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}


String konversi_rotate_ccw_less(int x){
  float L1[4][3] = {
    {5.5 + calL1x, 4.5 + calL1y, kakiTurun + calL1z},
    {5.5 + calL1x, 4.5 + calL1y, kakiTurun + calL1z},
    {6.5 + calL1x, 3.5 + calL1y, kakiNaik + calL1z},
    {6.5 + calL1x, 3.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, -0.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, -0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6.5 + calL3x, 3.5 + calL3y, kakiTurun + calL3z},
    {6.5 + calL3x, 3.5 + calL3y, kakiTurun + calL3z},
    {5.5 + calL3x, 4.5 + calL3y, kakiNaik + calL3z},
    {5.5 + calL3x, 4.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {5.5 + calR1x, 4.5 + calR1y, kakiNaik + calR1z},
    {5.5 + calR1x, 4.5 + calR1y, kakiTurun + calR1z},
    {6.5 + calR1x, 3.5 + calR1y, kakiTurun + calR1z},
    {6.5 + calR1x, 3.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -0.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, 0.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6.5 + calR3x, 3.5 + calR3y, kakiNaik + calR3z},
    {6.5 + calR3x, 3.5 + calR3y, kakiTurun + calR3z},
    {5.5 + calR3x, 4.5 + calR3y, kakiTurun + calR3z},
    {5.5 + calR3x, 4.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_right(int x){
  float L1[4][3] = {
    {8 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {8 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {5 + calL1x, 4 + calL1y, kakiNaik + calL1z},
    {5 + calL1x, 4 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {5.6 + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {5.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {8.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {8.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {8 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {8 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {5 + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {5 + calL3x, 4 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {8 + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {8 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {5 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {5 + calR1x, 4 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {5.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {5.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {8.6 + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {8.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {8 + calR3x, 4 + calR3y, kakiNaik + calR3z},
    {8 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {5 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {5 + calR3x, 4 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_left(int x){
  float L1[4][3] = {
    {5 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {5 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {8 + calL1x, 4 + calL1y, kakiNaik + calL1z},
    {8 + calL1x, 4 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {8.6 + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {8.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {5.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {5.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {5 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {5 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {8 + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {8 + calL3x, 4 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {5 + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {5 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {8 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {8 + calR1x, 4 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {8.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {8.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {5.6 + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {5.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {5 + calR3x, 4 + calR3y, kakiNaik + calR3z},
    {5 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {8 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {8 + calR3x, 4 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}


String konversi_turn_right(int x){
  float L1[4][3] = {
    {6 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
    {6 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
    {4.5 + calL1x, 5.5 + calL1y, kakiNaik + calL1z},
    {4.5 + calL1x, 5.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x,  1.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x,  1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, -1.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {7.5 + calL3x, 2.5 + calL3y, kakiNaik + calL3z},
    {7.5 + calL3x, 2.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 5.5 + calR1y, kakiNaik + calR1z},
    {6 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {4.5 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {4.5 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 2.5 + calR3y, kakiNaik + calR3z},
    {6 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
    {7.5 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
    {7.5 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_turn_left(int x){
  float L1[4][3] = {
    {6 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
    {6 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
    {7.5 + calL1x, 2.5 + calL1y, kakiNaik + calL1z},
    {7.5 + calL1x, 2.5 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, 1.5 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, 1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 1.5 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 1.5 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
    {4.5 + calL3x, 5.5 + calL3y, kakiNaik + calL3z},
    {4.5 + calL3x, 5.5 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 5.5 + calR1y, kakiNaik + calR1z},
    {6 + calR1x, 5.5 + calR1y, kakiTurun + calR1z},
    {7.5 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
    {7.5 + calR1x, 2.5 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, -1.5 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x,  1.5 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 2.5 + calR3y, kakiNaik + calR3z},
    {6 + calR3x, 2.5 + calR3y, kakiTurun + calR3z},
    {4.5 + calR3x, 5.5 + calR3y, kakiTurun + calR3z},
    {4.5 + calR3x, 5.5 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_ShiftRightLess(int x){ 
  float L1[4][3] = {
    {7 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {7 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {6 + calL1x, 4 + calL1y, kakiNaik + calL1z},
    {6 + calL1x, 4 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {6.6 + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {7.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {7.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {7 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {7 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {7 + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {7 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {7.6 + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {7.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {7 + calR3x, 4 + calR3y, kakiNaik + calR3z},
    {7 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {6 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {6 + calR3x, 4 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

String konversi_ShiftLeftLess(int x){ 
  float L1[4][3] = {
    {6 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {6 + calL1x, 4 + calL1y, kakiTurun + calL1z},
    {7 + calL1x, 4 + calL1y, kakiNaik + calL1z},
    {7 + calL1x, 4 + calL1y, kakiTurun + calL1z},
  };
  float L2[4][3] = {
    {7.6 + calL2x, 0 + calL2y, kakiNaik + calL2z},
    {7.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
    {6.6 + calL2x, 0 + calL2y, kakiTurun + calL2z},
  };
  float L3[4][3] = {
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {6 + calL3x, 4 + calL3y, kakiTurun + calL3z},
    {7 + calL3x, 4 + calL3y, kakiNaik + calL3z},
    {7 + calL3x, 4 + calL3y, kakiTurun + calL3z},
  };
  float R1[4][3] = {
    {6 + calR1x, 4 + calR1y, kakiNaik + calR1z},
    {6 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {7 + calR1x, 4 + calR1y, kakiTurun + calR1z},
    {7 + calR1x, 4 + calR1y, kakiTurun + calR1z},
  };
  float R2[4][3] = {
    {7.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {7.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
    {6.6 + calR2x, 0 + calR2y, kakiNaik + calR2z},
    {6.6 + calR2x, 0 + calR2y, kakiTurun + calR2z},
  };
  float R3[4][3] = {
    {6 + calR3x, 4 + calR3y, kakiNaik + calR3z},
    {6 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {7 + calR3x, 4 + calR3y, kakiTurun + calR3z},
    {7 + calR3x, 4 + calR3y, kakiTurun + calR3z},
  };
  //================================================== L1 ==================================================
  int tetaL1 = round(convert(getAngularOfCartesianTeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aTetaL1, bTetaL1, false));
  int betaL1 = round(convert(getAngularOfCartesianBeta(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aBetaL1, bBetaL1, false));
  int alphaL1 = round(convert(getAngularOfCartesianAlpha(L1[x][0],L1[x][1],L1[x][2]), M_PI/4, aAlphaL1, bAlphaL1, false));
  //================================================== L2 ==================================================
  int tetaL2 = round(convert(getAngularOfCartesianTeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aTetaL2, bTetaL2, false));
  int betaL2 = round(convert(getAngularOfCartesianBeta(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aBetaL2, bBetaL2, false));
  int alphaL2 = round(convert(getAngularOfCartesianAlpha(L2[x][0],L2[x][1],L2[x][2]), M_PI/4, aAlphaL2, bAlphaL2, false));
  //================================================== L3 ==================================================
  int tetaL3 = round(convert(getAngularOfCartesianTeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aTetaL3, bTetaL3, false));
  int betaL3 = round(convert(getAngularOfCartesianBeta(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aBetaL3, bBetaL3, false));
  int alphaL3 = round(convert(getAngularOfCartesianAlpha(L3[x][0],L3[x][1],L3[x][2]), M_PI/4, aAlphaL3, bAlphaL3, false));
  //================================================== R1 ==================================================
  int tetaR1 = round(convert(getAngularOfCartesianTeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aTetaR1, bTetaR1, false));
  int betaR1 = round(convert(getAngularOfCartesianBeta(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aBetaR1, bBetaR1, false));
  int alphaR1 = round(convert(getAngularOfCartesianAlpha(R1[x][0],R1[x][1],R1[x][2]), M_PI/4, aAlphaR1, bAlphaR1, false));
  //================================================== R2 ==================================================
  int tetaR2 = round(convert(getAngularOfCartesianTetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aTetaR2, bTetaR2, false));
  int betaR2 = round(convert(getAngularOfCartesianBetaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aBetaR2, bBetaR2, false));
  int alphaR2 = round(convert(getAngularOfCartesianAlphaR2R3(R2[x][0],R2[x][1],R2[x][2]), M_PI/4, aAlphaR2, bAlphaR2, false));
  //================================================== R3 ==================================================
  int tetaR3 = round(convert(getAngularOfCartesianTetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aTetaR3, bTetaR3, false));
  int betaR3 = round(convert(getAngularOfCartesianBetaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aBetaR3, bBetaR3, false));
  int alphaR3 = round(convert(getAngularOfCartesianAlphaR2R3(R3[x][0],R3[x][1],R3[x][2]), M_PI/4, aAlphaR3, bAlphaR3, false));

  String spasi = " ";
  String hasil = "#4P"+String(tetaL1)+spasi+"#5P"+String(betaL1)+spasi+"#6P"+String(alphaL1)+spasi+"#8P"+String(tetaL2)+spasi+"#9P"+String(betaL2)+spasi+"#10P"+String(alphaL2)+spasi+"#12P"+String(tetaL3)+spasi+"#13P"+String(betaL3)+spasi+"#14P"+String(alphaL3)+spasi+"#20P"+String(tetaR1)+spasi+"#21P"+String(betaR1)+spasi+"#22P"+String(alphaR1)+spasi+"#24P"+String(tetaR2)+spasi+"#25P"+String(betaR2)+spasi+"#26P"+String(alphaR2)+spasi+"#28P"+String(tetaR3)+spasi+"#29P"+String(betaR3)+spasi+"#30P"+String(alphaR3)+" T200";
  return hasil;
}

   // START OF CODE GENERATED

  void ssc_standby () {
    com.println(F("#23P2400 #11P940 T200"));
    switch (state_nextStep) {
      case 0:
         com.println(konversi_standby(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
         com.println(konversi_standby(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
         com.println(konversi_standby(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
         com.println(konversi_standby(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_normalize () {
    com.println(F("#23P1100 #11P1340 T200"));
    switch (state_nextStep) {
      case 0:
        com.println(F("#4P1593 #5P2308 #6P2137 #12P1334 #13P2251 #14P2445 #24P1300 #25P1028 #26P500 T200"));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(F("#4P1593 #5P1953 #6P1925 #12P1334 #13P1950 #14P2242 #24P1300 #25P1395 #26P718 T200"));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(F("#8P1300 #9P2597 #10P2302 #20P1484 #21P752 #22P750 #28P1248 #29P786 #30P894 T200"));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(F("#8P1300 #9P2259 #10P2079 #20P1484 #21P1107 #22P974 #28P1248 #29P1073 #30P1057 T200"));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCCWKorban () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotate_ccw_korban(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotate_ccw_korban(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotate_ccw_korban(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotate_ccw_korban(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCWKorban () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotate_cw_korban(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotate_cw_korban(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotate_cw_korban(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotate_cw_korban(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_backwardKorban () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_backwardKorban(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_backwardKorban(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_backwardKorban(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_backwardKorban(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_shiftLeftKorban () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_shiftLeftKorban(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_shiftLeftKorban(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_shiftLeftKorban(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_shiftLeftKorban(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_shiftRightKorban () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_shiftRightKorban(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_shiftRightKorban(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_shiftRightKorban(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_shiftRightKorban(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_semprotGoyang () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_SemprotGoyang(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_SemprotGoyang(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_SemprotGoyang(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_SemprotGoyang(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_forwardGyro () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_ForwardGyro(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_ForwardGyro(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_ForwardGyro(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_ForwardGyro(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_forward () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_forward(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_forward(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_forward(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_forward(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_jalanKepitingMirror () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_JalanKepitingMirror(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_JalanKepitingMirror(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_JalanKepitingMirror(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_JalanKepitingMirror(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_jalanKepiting () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_JalanKepiting(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_JalanKepiting(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_JalanKepiting(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_JalanKepiting(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCWKepiting () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotateCWKepiting(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotateCWKepiting(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotateCWKepiting(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotateCWKepiting(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCCWKepiting () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotateCCWKepiting(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotateCCWKepiting(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotateCCWKepiting(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotateCCWKepiting(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_rotateCW () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotate_cw(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotate_cw(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotate_cw(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotate_cw(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCCW () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotate_ccw(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotate_ccw(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotate_ccw(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotate_ccw(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCWLess () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotate_cw_less(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotate_cw_less(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotate_cw_less(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotate_cw_less(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_rotateCCWLess () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_rotate_ccw_less(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_rotate_ccw_less(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_rotate_ccw_less(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_rotate_ccw_less(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_backward () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_backward(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_backward(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_backward(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_backward(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_shiftRight () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_right(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_right(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_right(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_right(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_shiftLeft () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_left(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_left(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_left(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_left(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_turnRight () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_turn_right(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_turn_right(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_turn_right(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_turn_right(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_turnLeft () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_turn_left(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_turn_left(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_turn_left(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_turn_left(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_shiftRightLess () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_ShiftRightLess(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_ShiftRightLess(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_ShiftRightLess(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_ShiftRightLess(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_shiftLefttLess () {
    switch (state_nextStep) {
      case 0:
        com.println(konversi_ShiftLeftLess(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_ShiftLeftLess(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_ShiftLeftLess(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_ShiftLeftLess(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }

  void ssc_capitNaik () {
    com.println(F("#11P1340 T200"));
    delay(200);
    com.println(F("#23P1100 T200"));
    delay(200);
  }

  void ssc_capitTurun () {
    com.println(F("#11P940 T200"));
    delay(200);
    com.println(F("#23P2300 T200"));
    delay(200);
  }
  
  void ssc_capitAngkat () {
    com.println(F("#11P1340 T200"));
    delay(200);
    com.println(F("#23P1100 T200"));
    delay(200);
    switch (state_nextStep) {
      case 0:
        com.println(konversi_capitAngkat(0));
        state_isComboBUp = true;
        state_nextStep = 1;
        break;
      case 1:
        com.println(konversi_capitAngkat(1));
        state_isComboBUp = false;
        state_nextStep = 2;
        break;
      case 2:
        com.println(konversi_capitAngkat(2));
        state_isComboAUp = true;
        state_nextStep = 3;
        break;
      case 3:
        com.println(konversi_capitAngkat(3));
        state_isComboAUp = false;
        state_nextStep = 0;
        break;
    }
  }
  
  void ssc_capitLepas () {
    com.println(F("#23P2400 T200"));
    delay(200);
    com.println(F("#11P940 T200"));
    delay(2000);
    com.println(F("#23P1100 T200"));
    delay(200);
    com.println(F("#11P1340 T200"));
    delay(200);
  }

  
  // END OF GENERATED

  
void moveAndi (MoveType id) {
  if (millis() - state_lastMoveRecord > 249) {
    state_lastMoveRecord = millis();

    if (state_currentMove != NORMALIZE || (state_currentMove == NORMALIZE && (state_isComboAUp || state_isComboBUp))) {
      isNormalized = false;
    }

    if (state_currentMove != STANDBY) {
      isStandby = false;
    }

    if (state_step2keep > 0) {
      state_step2keep = state_step2keep - 1;
    } else {
      state_currentMove = id;
    }

    switch (state_currentMove) {
      case BACKWARD_KORBAN:
        ssc_backwardKorban();
        break;
      case SHIFT_LEFT_KORBAN:
        ssc_shiftLeftKorban();
        break;
      case SHIFT_RIGHT_KORBAN:
        ssc_shiftRightKorban();
        break;
      case ROTATE_CW_KORBAN:
        ssc_rotateCWKorban();
        break;
      case ROTATE_CCW_KORBAN:
        ssc_rotateCCWKorban();
        break;
    }
  }
}

void move (MoveType id) {
  if (millis() - state_lastMoveRecord > 199) {
    state_lastMoveRecord = millis();

    if (state_currentMove != NORMALIZE || (state_currentMove == NORMALIZE && (state_isComboAUp || state_isComboBUp))) {
      isNormalized = false;
    }

    if (state_currentMove != STANDBY) {
      isStandby = false;
    }

    if (state_step2keep > 0) {
      state_step2keep = state_step2keep - 1;
    } else {
      state_currentMove = id;
    }

    switch (state_currentMove) {
      case STANDBY:
        ssc_standby();
        break;
      case NORMALIZE:
        ssc_normalize();
        break;
      case FORWARD:
        ssc_forward();
        break;
      case ROTATE_CW:
        ssc_rotateCW();
        break;
      case ROTATE_CCW:
        ssc_rotateCCW();
        break;
      case ROTATE_CW_LESS:
        ssc_rotateCWLess();
        break;
      case ROTATE_CCW_LESS:
        ssc_rotateCCWLess();
        break;
      case BACKWARD:
        ssc_backward();
        break;
      case SHIFT_RIGHT:
        ssc_shiftRight();
        break;
      case SHIFT_LEFT:
        ssc_shiftLeft();
        break;
      case TURN_RIGHT:
        ssc_turnRight();
        break;
      case TURN_LEFT:
        ssc_turnLeft();
        break;
      case JALAN_KEPITING:
        ssc_jalanKepiting();
        break;
      case SHIFT_RIGHT_LESS:
        ssc_shiftRightLess();
        break;
      case SHIFT_LEFT_LESS:
        ssc_shiftLefttLess();
        break;
      case CAPIT_TURUN:
        ssc_capitTurun();
        break;
      case CAPIT_ANGKAT:
        ssc_capitAngkat();
        break;
      case CAPIT_LEPAS:
        ssc_capitLepas();
        break;
      case ROTATE_CWKEPITING:
        ssc_rotateCWKepiting();
        break;
      case ROTATE_CCWKEPITING:
        ssc_rotateCCWKepiting();
        break;
      case FORWARD_GYRO:
        ssc_forwardGyro();
        break;
      case SEMPROT_GOYANG:
        ssc_semprotGoyang();
        break;
      case BACKWARD_KORBAN:
        ssc_backwardKorban();
        break;
      case SHIFT_LEFT_KORBAN:
        ssc_shiftLeftKorban();
        break;
      case SHIFT_RIGHT_KORBAN:
        ssc_shiftRightKorban();
        break;
      case ROTATE_CW_KORBAN:
        ssc_rotateCWKorban();
        break;
      case ROTATE_CCW_KORBAN:
        ssc_rotateCCWKorban();
        break;
      case JALAN_KEPITING_MIRROR:
        ssc_jalanKepitingMirror();
        break;
      case CAPIT_NAIK:
        ssc_capitNaik();
        break;
    }
  }
}

void keep (int num) {
  if (state_step2keep < 1) {
    state_step2keep = num;
  }
}

void force () {
  state_step2keep = 0;
}

void standby () {
  move(STANDBY);
  if (state_currentMove == STANDBY) isStandby = true;
}

void normalize () {
  if (state_currentMove != NORMALIZE) {
    if (state_isComboAUp) {
      state_nextStep = 1;
    } else if (state_isComboBUp) {
      state_nextStep = 3;
    }
  }

  move(NORMALIZE);

  if (state_currentMove == NORMALIZE && !state_isComboAUp && !state_isComboBUp) {
    isNormalized = true;
  }
}

void capitNaik () {
    move(CAPIT_NAIK);
}

void backwardKorban () {
    moveAndi(BACKWARD_KORBAN);
}

void shiftLeftKorban () {
    moveAndi(SHIFT_LEFT_KORBAN);
}

void shiftRightKorban () {
    moveAndi(SHIFT_RIGHT_KORBAN);
}

void semprotGoyang () {
    move(SEMPROT_GOYANG);
}

void forwardGyro () {
    move(FORWARD_GYRO);
}

void jalanKepitingMirror () {
    move(JALAN_KEPITING_MIRROR);
}

void jalanKepiting () {
    move(JALAN_KEPITING);
}

void rotateCWKepiting () {
    move(ROTATE_CWKEPITING);
}

void rotateCCWKepiting () {
    move(ROTATE_CCWKEPITING);
}

float forward (float distance = 0) {
  distance = distance*konstantaKalibrasiJarak;
  if (distance > 0) {
    unsigned int startCounter = millis();
    unsigned int currentCounter = millis();
    while ((currentCounter - startCounter) < distance) {
      move(FORWARD);
      currentCounter = millis();
    }
  } else {
    move(FORWARD);
  }
}

void rotateCW (unsigned int keep = 0) {
  if (keep > 0) {
    unsigned int startCounter = millis();
    unsigned int currentCounter = millis();
    while ((currentCounter - startCounter) < keep) {
      move(ROTATE_CW);
      currentCounter = millis();
    }
  } else {
    move(ROTATE_CW);
  }
}

void rotateCCW (unsigned int keep = 0) {
  if (keep > 0) {
    unsigned int startCounter = millis();
    unsigned int currentCounter = millis();
    while ((currentCounter - startCounter) < keep) {
      move(ROTATE_CCW);
      currentCounter = millis();
    }
  } else {
    move(ROTATE_CCW);
  }
}

float backward (float distance = 3) {
  distance = round(distance/3)*2;
  for(int i=0; i<distance; i++){
    move(BACKWARD);
  }
  return false;
}

void shiftRight (float distance = 0) {
  distance = distance*konstantaKalibrasiJarak;
  if (distance > 0) {
    unsigned int startCounter = millis();
    unsigned int currentCounter = millis();
    while ((currentCounter - startCounter) < distance) {
      move(SHIFT_RIGHT);
      currentCounter = millis();
    }
  } else {
    move(SHIFT_RIGHT);
  }
}

void shiftLeft () {
  move(SHIFT_LEFT);
}

void rotateCWLess () {
  move(ROTATE_CW_LESS);
}

void rotateCCWLess () {
  move(ROTATE_CCW_LESS);
}

void turnRight () {
  move(TURN_RIGHT);
}

void turnLeft () {
  move(TURN_LEFT);
}

void gyroVertikal(){
  move(GYRO_VERTIKAL);
}

void rotateCCWKorban(){
  moveAndi(ROTATE_CW_KORBAN);
}

void rotateCWKorban(){
  moveAndi(ROTATE_CCW_KORBAN);
}

void gyroHorizontal(){
  move(GYRO_HORIZONTAL);
}

float shiftRightLess (float distance = 2) {
  distance = round(distance/2)*2;
  for(int i=0; i<distance; i++){
    move(SHIFT_RIGHT_LESS);
  }
  return false;
}

float shiftLeftLess (float distance = 2) {
  distance = round(distance/2)*2;
  for(int i=0; i<distance; i++){
    move(SHIFT_LEFT_LESS);
  }
  return false;
}

float capitTurun (float distance = 4) {
  distance = round(distance/2)*2;
  for(int i=0; i<distance; i++){
    move(CAPIT_TURUN);
  }
  return false;
}

float capitAngkat () {
  move(CAPIT_ANGKAT);
}

float capitLepas (float distance = 1) {
//  distance = round(distance/2)*2;
  for(int i=0; i<distance; i++){
    move(CAPIT_LEPAS);
  }
  return false;
}
}
