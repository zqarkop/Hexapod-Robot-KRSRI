#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
#include "ping.h"
#include "proxy.h"
#include "flame.h"
#include "legs.h"
#include "pump.h"
#include "activation.h"
#include "lcd.h"
#include "line.h"


#include <Pixy2.h>                                     //kodeku
int i, width, height;
float x, y;
float cx, cy;
float fullx = 315;
float fully = 207;
bool penguncifase1 = false;
bool penguncifase2 = false;
bool penguncifase3= false;
bool stopbang=false;
bool capitlangsung=false;
bool isBaruTurunCapit=false;
bool penguncicapitlangsung=false;
bool fakemovecapit=false;
Pixy2 pixy;                                             //kodeku

bool homeState = true;  // cek kondisi apakah di home buat deteksi jalan di awal  default true
bool state_isInversed = false; //true = SLWR, false = SRWR // default false
bool state_isInitialized = false;  // default false
bool naikHR1 = false;  // default false
bool naikHR2 = false;  // default false
bool turunHR1 = false; // default false
bool turunHR2 = false; // default false
bool masukPertigaan = false; // default false
bool sebelumRuang1 = true;  // default true
bool turunKakiSebelumRuang1 = false; // default false
bool udahKeluarRuang1 = false; // default false
bool udahLewatHR2 = false; // default false
bool masukRuang1 = false; // default false
bool udahMasukRuang1 = false; // default false
bool nyalainFlame = false; // default false
bool siapPadamApiR1 = false; // default false
bool pindahArahHR1 = false; //default false
bool ruang1Selesai = false; //default false
bool kucing = false;
bool pulang = false; //default false
bool otwMatiinSensor = false; //default false
bool turuninKorbanR1 = false; //default false
bool fixKeluarR1 = false; //default false
bool capit1 = false; //default false
bool capit2 = false; //default false
bool capit3 = false; //default false
bool keluarR1LewatGaris = false; //default false
bool isCrabWalksOn = false; //default false
bool isHR1 = false; //default false
bool isHR2Sebelum = false; //default false
bool isHR2Setelah = false; //default false
bool isLurusinMauJalanKepiting = false; //default false
bool udahTurunTanjakan = false; //default false
bool isTimeTurunTanjakan = false; //default false
bool isLurusinSetelahJalanKepiting = false; //default false
bool bolehTurunTanjakan = false; //default false
bool isLurusinMauMasukRuang1 = false; //default false
bool isSelesaiTanjakan = false; //default false
bool isOtwRuang1 = false; //default false
bool isUdahMasukRuang1 = false; //default false
bool isDepanRuang1 = false; //default false
bool isGeserDikitSetelahTanjakan = false; //default false
bool isNyalainFlame = false; //default false
bool isSelamatinKorban = false; //default false
bool isLurusinMauKeluarRuang1 = false; //default false
bool isBuangAnak = false; //default false
bool isDepanRuang2 = false; //default false
bool isUdahMasukRuang2 = false; //default false
bool isSelesaiMasukRuang2 = false; //default false
bool kodeMajid = false; //default false
bool isLurusinAbisNyemprot = false; //default false
bool isLurusinMajid = false; //default false
bool mirror = false; //default false
bool pengunciHome = false; //default false
bool isMentokRuang2 = false; //default false
bool awalBanget = false; //default false

bool andiMatiin = false; // default false

float konstantaKalibrasi = 1.4;
float konstantaKalibrasiJarak = 176.27;
float sensitivitasMPU = 0.5;

unsigned int state_startTime = 0;
unsigned int state_lastSWR = 0;
unsigned long waktu;
unsigned long waktuNaikHR1;
unsigned long waktuAndi;
unsigned long waktuKeluarR1;
unsigned long timeTurunTanjakanStart;
unsigned long timeTurunTanjakanStop;

int r2ToHome = 0;
int CurrentState = 0;
int CounterRead = 0;
int CounterFire = 0;
int toleransiKompas = 5;

int kalibrasiArahBelakang = 296; //296
int kalibrasiArahKiri = 53;      //53
int kalibrasiArahDepan = 105;    //105
int kalibrasiArahKanan = 153;    //153

int arahBelakang,arahKiri,arahDepan,arahKanan;

bool avoidWall(bool inverse = false);
bool flameDetection();
bool avoid3Ladder(bool inverse = false);
bool getCloser2SRWR(bool inverse = false);
void traceRoute();
void traceRouteInversed();

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
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

float getCompass(){
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  if(heading < 0)
    heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;
   
  float headingDegrees = heading * 180/M_PI;

  // if(headingDegrees >= 355){ // ada yang mendekati 5
  //   headingDegrees = headingDegrees-360;
  // }
  // if(headingDegrees <= 5){ // ada yang mendekati 355
  //   headingDegrees = 360+headingDegrees;
  // }

  return headingDegrees;
}

void setup () {
  Serial.begin(9600);
  ping::setup();
  proxy::setup();
  flame::setup();
  legs::setup();
  pump::setup();
  activation::setup();
  lcd::setup();
  line::setup();
  legs::standby();
  pixy.init();
  lcd::clean();
  lcd::justPrint("Kalibrasi", "Dulu Gan !!!");
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  delay(500);
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  delay(500);
}
void loop () {
  activation::update();
  waktu = millis();
  if (activation::isON) {

    // ======================= Masukin di penentuan ==========
    if(!mirror){
      arahBelakang = kalibrasiArahBelakang; 
      arahKiri = kalibrasiArahKiri; 
      arahDepan = kalibrasiArahDepan; 
      arahKanan = kalibrasiArahKanan; 
    }
    else{
      arahBelakang = kalibrasiArahDepan; 
      arahKiri = kalibrasiArahKiri; 
      arahDepan = kalibrasiArahBelakang; 
      arahKanan = kalibrasiArahKanan; 
    }
    
    ping::update();
    proxy::update();
    flame::update();
    line::update();
    if (homeState == true){
      if(!awalBanget){
        legs::capitNaik();
        awalBanget = true;
      }
      // unsigned int startCounterr = millis();
      // unsigned int currentCounterr = millis();
      // while ((currentCounterr - startCounterr) < 200) {
      //   legs::capitAngkat();
      //   currentCounterr = millis();
      // }

     if(((getCompass() < arahKiri-5) || (getCompass() > arahKiri+5)) && (pengunciHome == false)){
       legs::rotateCCWLess();
     }
     if(((getCompass() > arahKiri-5) && (getCompass() < arahKiri+5))){
       legs::forward(50);
       pengunciHome = true;
       delay(200);
       pingupdate();
       if(!(ping::jarakA > 40)){
         mirror = true;
         homeState = false;
         state_isInversed = true;
         delay(200);
         while(!((getCompass() > arahKanan-5) && (getCompass() < arahKanan+5))){
           legs::rotateCCW();
         }
         lcd::clean();
         lcd::justPrint("Pepet", "Kiri");
         delay(1000);
         unsigned int startCounter = millis();
         unsigned int currentCounter = millis();
         while ((currentCounter - startCounter) < 130*konstantaKalibrasiJarak) {
           legs::forward();
           if (!avoidWall()) return;
           currentCounter = millis();
         }
       }
       else{
         mirror = false;
         homeState = false;
         state_isInversed = false;
         delay(200);
         while(!((getCompass() > arahKanan-5) && (getCompass() < arahKanan+5))){
           legs::rotateCCW();
         }
         lcd::clean();
         lcd::justPrint("Pepet", "Kanan");
         delay(1000);
         unsigned int startCounterd = millis();
         unsigned int currentCounterd = millis();
         while ((currentCounterd - startCounterd) < 130*konstantaKalibrasiJarak) {
           legs::forward();
           if (!avoidWall()) return;
           currentCounterd = millis();
         }
       }
     }
    }
    else{
      // Kodenya
      ping::update();
      proxy::update();
      flame::update();
      line::update();
      if((isHR1 == false)  && (line::isDetectedPuing == true)){
        isHR1 = true;
        delay(500);
      }
      if((isHR1 == true) && (isHR2Sebelum == false)){
        ping::update();
        if((ping::jarakC <= 20) && (isLurusinMauJalanKepiting == false)){
          delay(500);
          if(getCompass() < arahDepan-5){
            if((getCompass() < arahDepan-5)){  //Ganti misal ada perubahan kompas    barakuda
              legs::rotateCW();
              lcd::clean();
              lcd::justPrint("Putar", "Kanan");
              // delay(500);
            }
          }
          else if(getCompass() > arahDepan+5){  //Ganti misal ada perubahan kompas    barakuda
            if((getCompass() > arahDepan+5)){
              legs::rotateCCW();
              lcd::clean();
              lcd::justPrint("Putar", "Kiri");
              // delay(500);
            }
          }
          if((getCompass() >= arahDepan-5) && (getCompass() <= arahDepan+5) && (isLurusinMauJalanKepiting == false)){
            lcd::clean();
            lcd::justPrint("Udah", "Lurus");
            // delay(500);
            isHR2Sebelum = true;
            isLurusinMauJalanKepiting = true;
          }
        }
        if((ping::jarakC > 20) && (isLurusinMauJalanKepiting == false)){
          if (!avoidWall(true)) return;
          legs::forward();
          lcd::clean();
          lcd::justPrint("Menuju", "HR 1");
        }
      }
      else{
        line::update();
        pingupdate();
        if((isHR2Sebelum == true) && (isCrabWalksOn == false) && (isHR2Setelah == false)){
          if(getCompass() < arahDepan-35){   //Ganti misal ada perubahan kompas    barakuda
            for(int i = 0; i<3; i++){
              legs::rotateCWKepiting();
              delay(25);
            }
          }
          if(getCompass() > arahDepan+15){   //Ganti misal ada perubahan kompas    barakuda
            for(int i = 0; i<3; i++){
              legs::rotateCCWKepiting();
              delay(25);
            }
          }
          if(!mirror){
            legs::jalanKepiting();
          }
          else{
            legs::jalanKepitingMirror();
          }
          
          lcd::clean();
          lcd::justPrint("Mode", "Kepiting");
          if(!mirror){
            if((line::isDetectedHalangRintang == true) && (getPitch() < -5) && (udahTurunTanjakan == false) && (bolehTurunTanjakan == false)){
              bolehTurunTanjakan = true;
              lcd::clean();
              lcd::justPrint("Boleh", "Turun");
              delay(1000);
            }
            if((line::isDetectedLantai == true) && (getPitch() < 5) && (getPitch() > -5) && (udahTurunTanjakan == false) && (bolehTurunTanjakan == true)){
              udahTurunTanjakan = true;
              lcd::clean();
              lcd::justPrint("Turun", "Tanjakan");
              // delay(2000);
            }
          }
          if(mirror){
            if((line::isDetectedHalangRintang == true) && (getPitch() > 5) && (udahTurunTanjakan == false) && (bolehTurunTanjakan == false)){
              bolehTurunTanjakan = true;
              lcd::clean();
              lcd::justPrint("Boleh", "Turun");
              delay(1000);
            }
            if((line::isDetectedLantai == true) && (getPitch() < 5) && (getPitch() > -5) && (udahTurunTanjakan == false) && (bolehTurunTanjakan == true)){
              udahTurunTanjakan = true;
              lcd::clean();
              lcd::justPrint("Turun", "Tanjakan");
              // delay(2000);
            }
          }
          if((udahTurunTanjakan == true) && (isTimeTurunTanjakan == false)){
            timeTurunTanjakanStart = millis();
            isTimeTurunTanjakan = true;
          }
          timeTurunTanjakanStop = millis();
          if(((timeTurunTanjakanStop - timeTurunTanjakanStart) > 4500) && (isTimeTurunTanjakan == true)){
            isCrabWalksOn = true;
            isHR2Setelah = true;
          }
        }
        else{
          if(((getCompass() <= arahBelakang-5) || (getCompass() >= arahBelakang+5)) && (isHR2Setelah == true) && (isLurusinSetelahJalanKepiting == false) && (isSelesaiTanjakan == false)){
            legs::rotateCW();
            if((getCompass() >= arahBelakang-5) && (getCompass() <= arahBelakang+5)){
              isLurusinSetelahJalanKepiting = true;
              isSelesaiTanjakan = true;
              lcd::clean();
              lcd::justPrint("Udah", "Lurus");
              // delay(3000);
            }
          }
          else{
            if((isSelesaiTanjakan == true) && (isOtwRuang1 == false)){
              if(isGeserDikitSetelahTanjakan == false){
                // legs::shiftRight(10);
                isGeserDikitSetelahTanjakan = true;
              }
              if(isDepanRuang1 == false){
                unsigned int startCounter = millis();
                unsigned int currentCounter = millis();
                while ((currentCounter - startCounter) < 75*konstantaKalibrasiJarak) {
                  legs::forward();
                  if(getCompass() < arahBelakang-5){   //Ganti misal ada perubahan kompas    barakuda
                    legs::rotateCW();
                  }
                  if(getCompass() > arahBelakang+5){   //Ganti misal ada perubahan kompas    barakuda
                    legs::rotateCCW();
                  }
                  if (!avoidWall()) return;
                  currentCounter = millis();
                }
                lcd::clean();
                lcd::justPrint("Depan", "Ruang 1");
                // delay(2000);
                isDepanRuang1 = true;
              }
              if(!((getCompass() >= arahKanan-5) && (getCompass() <= arahKanan+5) && (isLurusinMauMasukRuang1 == false))){ 
//                legs::rotateCCW();
                if(!mirror){
                  legs::rotateCCWLess();
                }
                else{
                  legs::rotateCWLess();
                }
                lcd::clean();
                lcd::justPrint("Gaboleh", "Muter");
              }
              if((getCompass() >= arahKanan-5) && (getCompass() <= arahKanan+5) && (isLurusinMauMasukRuang1 == false)){
                unsigned int startCounter = millis();
                unsigned int currentCounter = millis();
                while ((currentCounter - startCounter) < 65*konstantaKalibrasiJarak) {
                  legs::forward();
                  if (!avoidWall()) return;
                  currentCounter = millis();
                }
                isLurusinMauMasukRuang1 = true;
                isOtwRuang1 = true;
                lcd::clean();
                lcd::justPrint("Berhasil", "Masuk Ruang 1");
//                delay(2000);
                isNyalainFlame = true;
              }
            }
            else{
              if(isSelamatinKorban == true){
                // kode majid
                //==================================== Start Kode Korban ======================================
//                while(kodeMajid == false){
//                  lcd::clean();
//                  lcd::justPrint("Kode Majid", "while");
//                  delay(2000);
//                  unsigned int startCounter = millis();
//                  unsigned int currentCounter = millis();
//                  while ((currentCounter - startCounter) < 200) {   //Ganti misal ada perubahan kompas    barakuda
//                    legs::capitTurun();
//                    currentCounter = millis();
//                  }
//                  delay(100);
//                  legs::capitTurun();
//                  if(isBaruTurunCapit == false){
//                    delay(2000);
//                    isBaruTurunCapit = true;
//                  }
//                  pixy.ccc.getBlocks();
//                  delay(10);
//                  if (pixy.ccc.numBlocks){
//                    for (i = 0; i < pixy.ccc.numBlocks; i++){
//                      x = pixy.ccc.blocks[i].m_x;                    //get x position
//                      y = pixy.ccc.blocks[i].m_y;                    //get y position
//                      width = pixy.ccc.blocks[i].m_width;            //get width
//                      height = pixy.ccc.blocks[i].m_height;          //get height
//                      cx = (x + (width / 2));
//                      cy = (y + (height / 2));
//                      lcd::clean();
//                      lcd::justPrintint(cx, cy);
//                      if(cy < 205+1 && penguncifase1== false){
//                        lcd::clean();
//                        lcd::justPrint("fase1", "mundur");
//                        legs::backwardKorban();
//                        if(height<4 && penguncicapitlangsung==false){
//                          capitlangsung=true;
//                          penguncicapitlangsung=true;
//                        }
//                      }
//                      if(cy > 205-1){
//                        penguncifase1=true;
//                      }
//                      if(cx < 195-4  && penguncifase1 == true && stopbang == false){              
//                        if(cx<180){
//                          legs::rotateCWKorban();
//                          lcd::clean();
//                          lcd::justPrint("fase2", "rotatecw");
//                          if(height<2 && penguncicapitlangsung==false){
//                            capitlangsung=true;
//                            penguncicapitlangsung=true;
//                          }
//                        }
//                        legs::shiftRightKorban();
//                        lcd::clean();
//                        lcd::justPrint("fase2", "ShiftRight");
//                        if(height<2 && penguncicapitlangsung==false){
//                          capitlangsung=true;
//                          penguncicapitlangsung=true;
//                        }  
//                      }
//                      //142
//                      if(cx > 195+12  && penguncifase1 ==true && stopbang == false){
//                        if(cx>220){ 
//                          legs::rotateCWKorban();
//                          lcd::clean();
//                          lcd::justPrint("fase2", "rotatecw");
//                          if(height<2 && penguncicapitlangsung==false){
//                            capitlangsung=true;
//                            penguncicapitlangsung=true;
//                          }
//                        }
//                        else{
//                          legs::shiftLeftKorban();
//                          lcd::clean();
//                          lcd::justPrint("fase2", "shiftlleft");
//                          if(height<2 && penguncicapitlangsung==false){
//                            capitlangsung=true;
//                            penguncicapitlangsung=true;
//                          }
//                        }
//                      }
//                      if (cx >= 193-4 && cx <= 197+4 && penguncifase1 == true && capitlangsung==false){         
//                        if(height<2 && penguncicapitlangsung==false){
//                          capitlangsung=true;
//                          penguncicapitlangsung=true;
//                        }
//                        for (i = 0; i < 8; i++){
//                          legs::backwardKorban();
//                          lcd::clean();
//                          lcd::justPrint("fase3", "Mundur");
//                          penguncifase3=true;
//                          if(height<2 && penguncicapitlangsung==false){
//                            i=8;
//                            capitlangsung=true;
//                            penguncicapitlangsung=true;
//                          }
//                        } 
//                        stopbang = true;
//                      }
//                      if (penguncifase1 == true && penguncicapitlangsung==false && penguncifase3==true){
//                        for (i = 0; i < 1; i++){
//                          legs::shiftRightKorban();
//                        }
//                        if(height<2 && penguncicapitlangsung==false){
//                          capitlangsung=true;
//                          penguncicapitlangsung=true;
//                        }
//                      }
//                      while((penguncifase1 == true && penguncifase3==true)||(capitlangsung==true)||(height<2)){
//                        if(height<2 && penguncicapitlangsung==false){
//                          capitlangsung=true;
//                          penguncicapitlangsung=true;
//                        }
//                        lcd::clean();
//                        lcd::justPrint("fasefinal", "Angkat capit");
//                        penguncifase3=false;
//                        fakemovecapit=true;
//                        delay(500);
//                        if(fakemovecapit==true){
//                          for (i = 0; i < 10; i++){
//                            legs::backwardKorban();
//                            lcd::clean();
//                            lcd::justPrint("fasefinal", "fakemove");
//                            penguncifase3=true;
//                          }
//                          fakemovecapit=false;
//                        }
//                        delay(100);  
//                        unsigned int startCounterr = millis();
//                        unsigned int currentCounterr = millis();
//                        while ((currentCounterr - startCounterr) < 200) {
//                          legs::capitAngkat();
//                          currentCounterr = millis();
//                        }
//                        delay(500);
//                        kodeMajid = true;
//                        lcd::clean();
//                        lcd::justPrint("selesai", "otw keluar");
//                      }
//                    }
//                  }
//                  if(!pixy.ccc.numBlocks){
//                    lcd::clean();
//                    lcd::justPrint("tidak ada korban", "Angkat capit");
//                    penguncifase3=false;
//                    delay(100);  
//                    unsigned int startCounterrr = millis();
//                    unsigned int currentCounterrr = millis();
//                    while ((currentCounterrr - startCounterrr) < 200) {
//                      legs::capitAngkat();
//                      currentCounterrr = millis();
//                    }
//                    delay(500);
//                    kodeMajid = true;
//                  }
//                }
                //==================================== End Kode Korban ======================================
                if(((getCompass() < arahKiri-5) || (getCompass() > arahKiri+5)) && (isLurusinMauKeluarRuang1 == false)){
                  legs::rotateCCW();
                  lcd::clean();
                  lcd::justPrint("Kode Majid", "Korban");
                  // delay(2000);
                }
                if((getCompass() > arahKiri-5) && (getCompass() < arahKiri+5) && (isLurusinMauKeluarRuang1 == false)){
                  isLurusinMauKeluarRuang1 = true;
                }
                if(isLurusinMauKeluarRuang1 == true){
                  legs::forward();
                  ping::update();
                  if (!avoidWall()) return;
                  if(getCompass() < arahKiri-5){
                    unsigned int startCounter = millis();
                    unsigned int currentCounter = millis();
                    while ((currentCounter - startCounter) < 200) {   //Ganti misal ada perubahan kompas    barakuda
                      legs::rotateCW();
                      currentCounter = millis();
                    }
                  }
                  if(getCompass() > arahKiri+5){
                    unsigned int startCounter = millis();
                    unsigned int currentCounter = millis();
                    while ((currentCounter - startCounter) < 200) {   //Ganti misal ada perubahan kompas    barakuda
                      legs::rotateCCW();
                      currentCounter = millis();
                    }
                  }
                  line::update();
                  ping::update();
                  if((ping::jarakC < 35) && (line::isDetectedKarpet == true)){
                    while(!((getCompass() > arahKanan-5) && (getCompass() < arahKanan+5))){
                      legs::rotateCCWLess();
                    }
                    unsigned int startCounter = millis();
                    unsigned int currentCounter = millis();
                    while ((currentCounter - startCounter) < 200) {
                      legs::capitLepas();
                      currentCounter = millis();
                    }
                    unsigned int startCounterBuangAnak = millis();
                    unsigned int currentCounterBuangAnak = millis();
                    while ((currentCounterBuangAnak - startCounterBuangAnak) < 25*konstantaKalibrasiJarak) {
                      legs::forward();
                      currentCounterBuangAnak = millis();
                    }
                    lcd::clean();
                    lcd::justPrint("Abis Buang", "Anak");
                    delay(500);
                    isSelamatinKorban = false;
                    while(!((getCompass() > arahDepan-5) && (getCompass() < arahDepan+5))){
                      legs::rotateCCWLess();
                    }
                    isBuangAnak = true;
                  }
                }
              }
              else{
                if((isBuangAnak == true) && (isDepanRuang2 == false)){
                  unsigned int startCounterBuangAnak = millis();
                  unsigned int currentCounterBuangAnak = millis();
                  while ((currentCounterBuangAnak - startCounterBuangAnak) < 140*konstantaKalibrasiJarak) {
                    legs::forward();
                    ping::update();
                    if (!avoidWall()) return;
                    if(getCompass() < arahDepan-5){   //Ganti misal ada perubahan kompas    barakuda
                      legs::rotateCW();
                    }
                    if(getCompass() > arahDepan+5){   //Ganti misal ada perubahan kompas    barakuda
                      legs::rotateCCW();
                    }
                    currentCounterBuangAnak = millis();
                    lcd::clean();
                    lcd::justPrint("OTW", "Ruang 2");
                  }
                  delay(500);
                  isDepanRuang2 = true;
                }
                else{
                  if((isDepanRuang2 == true) && (isSelesaiMasukRuang2 == false)){
                    lcd::clean();
                    lcd::justPrint("Masuk", "Ruaang 2");
                    if(!((getCompass() >= arahKanan-5) && (getCompass() <= arahKanan+5) && (isUdahMasukRuang2 == false))){
                      legs::rotateCWLess();
                      lcd::clean();
                      lcd::justPrint("Gaboleh", "Muter");
                    }
                    if((getCompass() >= arahKanan-5) && (getCompass() <= arahKanan+5) && (isUdahMasukRuang2 == false)){
                      unsigned int startCounter = millis();
                      unsigned int currentCounter = millis();
                      while ((currentCounter - startCounter) < 100*konstantaKalibrasiJarak) {
                        legs::forwardGyro();
                        if (!avoidWall()) return;
                        currentCounter = millis();
                      }
                      isUdahMasukRuang2 = true;
                      isSelesaiMasukRuang2 = true;
                      lcd::clean();
                      lcd::justPrint("Berhasil", "Masuk Ruang 2");
                      delay(1000);
                      isNyalainFlame = true;
                    }
                  }
                  else{
                    if((isSelesaiMasukRuang2 == true) && (isMentokRuang2 == false)){
                      ping::update();
                      proxy::update();
                      flame::update();
                      line::update();
                      legs::forwardGyro();
                      if (flameDetection()) return;
                      if(getCompass() < arahKanan-5){
                        legs::rotateCWLess();
                      }
                      if((getCompass() > arahKanan+5)){
                        legs::rotateCCWLess();
                      }
                      if (!avoidWall()) return;
                      if(line::isDetectedLantai == true){
                        isNyalainFlame = true;
                        isMentokRuang2 = true;
                        if(mirror == true){
                          state_isInversed = true;
                        }
                        else{
                          state_isInversed = false;
                        }
                      }
                    }
                    else{
                      ping::update();
                      proxy::update();
                      flame::update();
                      line::update();
                      if (state_isInversed) {          // Pepet Kiri
                        if (ping::isOnSLWR) {
                          state_lastSWR = millis();
                        }
                        if (!avoidWall(true)) return;
                        if(isNyalainFlame == true){
                          if (flameDetection()) return;
                        }
                        if (!getCloser2SRWR(true)) return;
                        traceRouteInverse();
                      } else {                         // Pepet Kanan
                        if (ping::isOnSRWR) {
                          state_lastSWR = millis();
                        }
                        if (!avoidWall()) return;
                        if(isNyalainFlame == true){
                          if (flameDetection()) return;
                        }
                        if (!getCloser2SRWR()) return;
                        traceRoute();
                      }
                    }
                  }
                }
              }
            }
          }
        }
     }
    } // Tutup Else
  } else {
    if (state_isInitialized) state_isInitialized = false;

    if (activation::isMenu) {           // Debug
      if (activation::isMenuChanged) lcd::clean();
      switch (activation::activeMenu) {
        case 0:
          lcd::justPrint(ping::debug(), ping::debug1());
          break;
        case 1:
          lcd::justPrint(flame::debug(), flame::debug1());
          break;
        case 2:
          lcd::justPrint(proxy::debug(), activation::debugSoundActivation());
          break;
        case 3:
          lcd::justPrint(line::debug(), line::debug1());
          break;
        case 4:
          lcd::justPrint("Press start to", "extinguish");
          pump::activate(activation::isStartPushed);
          break;
        case 5:
          lcd::clean();
          lcd::justPrintint(getCompass(), getCompass());
          delay(500);
          break;
        default:
          lcd::justPrint("== DEBUG MODE ==", "Press stop again");
      }
    } else {
      standBy();
    }
  }
}

void standBy () {
  if (legs::isStandby) {
    lcd::message(0, lcd::STANDBY);
    lcd::message(1, lcd::BLANK);
    return;
  }

  if (legs::isNormalized) {
    legs::standby();
    return;
  }

  lcd::message(0, lcd::NORMALIZING);
  lcd::message(1, lcd::BLANK);
  legs::normalize();
}

bool avoidWall (bool inverse = false) {
  short int minPos = 0;
  short int maxPos = 8;
  bool minPosFound = false;
  bool maxPosFound = false;

  if (!minPosFound && ping::near_a) {
    minPos = 0;
    minPosFound = true;
  }
  if (!minPosFound && ping::near_b) {
    minPos = 2;
    minPosFound = true;
  }
  if (!minPosFound && ping::near_c) {
    minPos = 4;
    minPosFound = true;
  }
  if (!minPosFound && ping::near_d) {
    minPos = 6;
    minPosFound = true;
  }
  if (!minPosFound && ping::near_e) {
    minPos = 8;
    minPosFound = true;
  }

  if (!maxPosFound && ping::near_e) {
    maxPos = 8;
    maxPosFound = true;
  }
  if (!maxPosFound && ping::near_d) {
    maxPos = 6;
    maxPosFound = true;
  }
  if (!maxPosFound && ping::near_c) {
    maxPos = 4;
    maxPosFound = true;
  }
  if (!maxPosFound && ping::near_b) {
    maxPos = 2;
    maxPosFound = true;
  }
  if (!maxPosFound && ping::near_a) {
    maxPos = 0;
    maxPosFound = true;
  }

  if (!minPosFound || !maxPosFound) return true; // this means wall is successfully avoided, if it's not then continue below

  short int avg = (maxPos + minPos) / 2;

  if (avg < 1) {
    lcd::message(0, lcd::WALL_ON_RIGHT);
    lcd::message(1, lcd::SHIFTING_LEFT);
    legs::shiftLeft();
  } else if (1 <= avg && avg <= 3) {
    lcd::message(0, lcd::WALL_ON_RIGHT);
    lcd::message(1, lcd::ROTATING_CCW);
    legs::rotateCCW();
  } else if (3 < avg && avg < 5) {
    lcd::message(0, lcd::WALL_ON_FRONT);

    if ((maxPos - minPos) == 0) {
      lcd::message(1, lcd::MOVING_BACKWARD);
      legs::backward(); // wall surface is flat
    } else {
      lcd::message(1, lcd::ROTATING_CW);

      if (inverse) {
        legs::rotateCCW(); // wall surface detected is not flat
      } else {
        legs::rotateCW(); // wall surface detected is not flat
      }
    }
  } else if (5 <= avg && avg <= 7) {
    lcd::message(0, lcd::WALL_ON_LEFT);
    lcd::message(1, lcd::ROTATING_CW);
    legs::rotateCW();
  } else if (avg > 7) {
    lcd::message(0, lcd::WALL_ON_LEFT);
    lcd::message(1, lcd::SHIFTING_RIGHT);
    legs::shiftRight();
  }

  return false;
}

bool flameDetection () {
  if (flame::is_right && (ping::near_b || ping::near_a)) {
    lcd::message(0, lcd::FIRE_ON_RIGHT);
    lcd::message(1, lcd::SHIFTING_LEFT);
    legs::shiftLeft();
    return true;
  }

  if (flame::is_left && (ping::near_d || ping::near_e)) {
    lcd::message(0, lcd::FIRE_ON_LEFT);
    lcd::message(1, lcd::SHIFTING_RIGHT);
    legs::shiftRight();
    return true;
  }

  if (flame::is_right && !flame::is_left && !flame::is_center) {
    lcd::message(0, lcd::FIRE_ON_RIGHT);
    lcd::message(1, lcd::ROTATING_CW);
    legs::rotateCW();
    return true;
  }

  if (flame::is_left && !flame::is_right && !flame::is_center) {
    lcd::message(0, lcd::FIRE_ON_LEFT);
    lcd::message(1, lcd::ROTATING_CCW);
    legs::rotateCCW();
    return true;
  }

  if (flame::is_right && !flame::is_left && flame::is_center) {
    lcd::message(0, lcd::FIRE_ON_RIGHT);
    lcd::message(1, lcd::ROTATING_CW);
    legs::rotateCWLess();
    return true;
  }

  if (flame::is_left && !flame::is_right && flame::is_center) {
    lcd::message(0, lcd::FIRE_ON_LEFT);
    lcd::message(1, lcd::ROTATING_CCW);
    legs::rotateCCWLess();
    return true;
  }

  //Pemadaman Api Ruang 1
  if (flame::is_center && CounterFire == 0) {
    lcd::message(0, lcd::FIRE_ON_CENTER);
    if (proxy::isDetectingSomething2) {
      legs::rotateCWLess();
      lcd::message(1, lcd::EXTINGUISHING);
      lcd::clean();
      lcd::justPrint("Lagi", "Nyemprot");
//      delay(2000);
      unsigned int startCounter = millis();
      unsigned int currentCounter = millis();
      while ((currentCounter - startCounter) < (2000*konstantaKalibrasi)) { // def 400
        lcd::message(1, lcd::ROTATING_CW);
        pump::menyebar();
        legs::semprotGoyang();
        currentCounter = millis();
      }
      pump::menyebarstop();
      if (flame::state_isIndicatorOn) {
        digitalWrite(PIN_FLAME_INDICATOR, HIGH);
      }
      delay(500);
      while(!((getCompass() > arahKiri-5) && (getCompass() < arahKiri+5))){
        legs::rotateCCWLess();
      }
      // legs::rotateCCW(4825*konstantaKalibrasi); //180 derajat  ======================================= ubah ke gyro


      
                unsigned int startCounterx = millis();
                unsigned int currentCounterx = millis();
                while ((currentCounterx - startCounterx) < 30*konstantaKalibrasiJarak) {
                  lcd::clean();
                  lcd::justPrint("Kode Majid", "Maju");
                  // delay(2000);
                  if((getCompass() >= arahKiri-8) && (getCompass() <= arahKiri+8)){
                    unsigned int startCounter = millis();
                    unsigned int currentCounter = millis();
                    while ((currentCounter - startCounter) < 200) {   //Ganti misal ada perubahan kompas    barakuda
                      legs::forward();
                      currentCounter = millis();
                    }
                  }
                  if(getCompass() < arahKiri-8){
                    unsigned int startCounter = millis();
                    unsigned int currentCounter = millis();
                    while ((currentCounter - startCounter) < 200) {   //Ganti misal ada perubahan kompas    barakuda
                      legs::rotateCW();
                      currentCounter = millis();
                    }
                  }
                  if(getCompass() > arahKiri+8){
                    unsigned int startCounter = millis();
                    unsigned int currentCounter = millis();
                    while ((currentCounter - startCounter) < 200) {   //Ganti misal ada perubahan kompas    barakuda
                      legs::rotateCCW();
                      currentCounter = millis();
                    }
                  }
                  if (!avoidWall()) return;
                  currentCounter = millis();
                }


                
      CounterFire += 1;
      isSelamatinKorban = true;
      isNyalainFlame = false;
      line::update();
    }
    else {
      lcd::message(1, lcd::MOVING_FORWARD);
      legs::forward();
    }
    return true;
  }
  //Pemadaman Api Ruang 2
  if (flame::is_center && CounterFire == 1) {
    lcd::message(0, lcd::FIRE_ON_CENTER);
    if (proxy::isDetectingSomething2) {
      lcd::message(1, lcd::EXTINGUISHING);
      lcd::clean();
      lcd::justPrint("Lagi", "Nyemprot");
      // delay(5000);
      unsigned int startCounter = millis();
      unsigned int currentCounter = millis();
      while ((currentCounter - startCounter) < (2000*konstantaKalibrasi)) { // def 400
        lcd::message(1, lcd::ROTATING_CW);
        pump::menyebar();
        legs::semprotGoyang();
        currentCounter = millis();
      }
      pump::menyebarstop();
      if (flame::state_isIndicatorOn) {
        digitalWrite(PIN_FLAME_INDICATOR, HIGH);
      }
      unsigned int startCounterMaju = millis();
      unsigned int currentCounterMaju = millis();
      while ((currentCounterMaju - startCounterMaju) < (15*konstantaKalibrasiJarak)) { // def 400
        legs::forwardGyro();
        currentCounterMaju = millis();
      }
      while(!((getCompass() > arahKiri-5) && (getCompass() < arahKiri+5))){
        legs::rotateCCWLess();
      }
      // legs::rotateCCW(4825*konstantaKalibrasi); //180 derajat
      CounterFire += 1;
      nyalainFlame = false;
      state_isInversed = true;
      delay(500);
      unsigned int startCounterPulang = millis();
      unsigned int currentCounterPulang = millis();
      while ((currentCounterPulang - startCounterPulang) < (200*konstantaKalibrasiJarak)) { // def 400
        legs::forwardGyro();
        currentCounterPulang = millis();
      }
    }
    else {
      lcd::message(1, lcd::MOVING_FORWARD);
      legs::forward();
    }
    return true;
  }
  return false;
}

bool getCloser2SRWR (bool inverse = false) {
  if (inverse) {
    if (ping::far_e && !ping::far_d && !ping::isOnSLWR) {
      lcd::message(0, lcd::FOUND_SLWR);
      lcd::message(1, lcd::SHIFTING_LEFT);
      legs::shiftLeft();
      return false;
    }
  } else {
    if (ping::far_a && !ping::far_b && !ping::isOnSRWR) {
      lcd::message(0, lcd::FOUND_SRWR);
      lcd::message(1, lcd::SHIFTING_RIGHT);
      legs::shiftRight();
      return false;
    }
  }

  return true;
}

void traceRoute () {
  if (!ping::far_a && !ping::far_b) {
    lcd::message(0, lcd::PATH_ON_RIGHT);
    lcd::message(1, lcd::TURNING_RIGHT);
    legs::turnRight();
  } else if (ping::far_a && !ping::far_c) {
    lcd::message(0, lcd::PATH_ON_FRONT);
    lcd::message(1, lcd::MOVING_FORWARD);

    if((turunKakiSebelumRuang1 == true) || (naikHR1 == true) || (CounterFire == 2) || ((naikHR2 == true) && (turunHR2 == false))){
      legs::forward();
//      legs::forwardHigher(); //Maju Kaki tinggi
    }
    else{
      legs::forward();
    }
  } else if (ping::far_a && ping::far_c && !ping::far_e) {
    lcd::message(0, lcd::PATH_ON_LEFT);
    lcd::message(1, lcd::TURNING_LEFT);
    legs::turnLeft();
  } else {
    lcd::message(0, lcd::NO_PATH);
    lcd::message(1, lcd::ROTATING_CCW);
    legs::rotateCCW(700*konstantaKalibrasi);
    pingupdate();
  }
}

void traceRouteInverse () {
  if (!ping::far_e && !ping::far_d) {
    lcd::message(0, lcd::PATH_ON_LEFT);
    lcd::message(1, lcd::TURNING_LEFT);
    legs::turnLeft();
  } else if (ping::far_e && !ping::far_c) {
    lcd::message(0, lcd::PATH_ON_FRONT);
    lcd::message(1, lcd::MOVING_FORWARD);

    if((turunKakiSebelumRuang1 == true) || (naikHR1 == true) || (CounterFire == 2) || ((naikHR2 == true) && (turunHR2 == false))){
//      legs::forwardHigher(); //Maju Kaki tinggi
      legs::forward();
    }
    else{
      legs::forward();
    }
  } else if (ping::far_e && ping::far_c && !ping::far_a) {
    lcd::message(0, lcd::PATH_ON_RIGHT);
    lcd::message(1, lcd::TURNING_RIGHT);
    legs::turnRight();
  } else {
    lcd::message(0, lcd::NO_PATH);
    lcd::message(1, lcd::ROTATING_CW);
    legs::rotateCW(700*konstantaKalibrasi);
    pingupdate();
  }
}

void pingupdate() {
  ping::update();
  ping::update();
  ping::update();
  ping::update();
  ping::update();
}
