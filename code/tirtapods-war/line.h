#define PIN_LINE_SENSOR A8
#define PIN_LINE_SENSOR2 A9

namespace line {
  bool isDetectedKarpet = false;
  bool isDetectedHalangRintang = false;
  bool isDetectedStiker = false;
  bool isDetectedLantai = false;
  bool isDetectedPuing = false;
  unsigned int nilaiF = 0;
  unsigned int nilaiC = 0;

  void setup () {
    pinMode(PIN_LINE_SENSOR, INPUT);
    pinMode(PIN_LINE_SENSOR2, INPUT);
  }

  void update () {
//    ================ Lantai ================
    if (analogRead(PIN_LINE_SENSOR) <= 80){ // 70 - 80
      isDetectedLantai = true;
    } else {
      isDetectedLantai = false;
    }
//    ================ Karpet ================
    if ((analogRead(PIN_LINE_SENSOR) >= 85) && (analogRead(PIN_LINE_SENSOR) < 100)){ //85 - 95
      isDetectedKarpet = true;
    } else {
      isDetectedKarpet = false;
    }
//    ================ Puing ================
    if ((analogRead(PIN_LINE_SENSOR) >= 150)){
      isDetectedPuing = true;
    } else {
      isDetectedPuing = false;
    }
//    ================ Halang Rintang ================
    if (analogRead(PIN_LINE_SENSOR) >= 200){ //200 atas
      isDetectedHalangRintang = true;
    } else {
      isDetectedHalangRintang = false;
    }

    nilaiF = analogRead(PIN_LINE_SENSOR);
    nilaiC = analogRead(PIN_LINE_SENSOR2);
  }

  String debug () {
    String text = "Line F: ";
    text.concat(analogRead(PIN_LINE_SENSOR));
    text.concat("         ");
    return text;
  }

  String debug1 () {
    String text = "Line C: ";
    text.concat(analogRead(PIN_LINE_SENSOR2));
    text.concat("         ");
    return text;
  }
}
