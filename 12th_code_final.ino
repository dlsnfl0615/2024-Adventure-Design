#include <SoftwareSerial.h>
#include <Servo.h>
#include <OneWire.h>
//블루투스
#define BT_RXD 7
#define BT_TXD 6
//수질측정
#define TdsSensorPin A1
#define SCOUNT 30
#define TdsDelayTime 1307U
//수온측정
#define TemperaturePin 2
#define TemperatureDelayTime 1401U
//서보모터
#define servoPin 13
#define ServoActivatingTime 2100U
//워터펌프
#define ADD_AA 10
#define ADD_AB 11
#define SUB_AA 8
#define SUB_AB 9
//그래프 값 전달하기
#define GraphDelayTime 3000U
//블루투스
SoftwareSerial bluetooth(BT_RXD, BT_TXD);
//수질측정
unsigned long printTimepoint = 0;
unsigned long analogSampleTimepoint = 0;
float analogBuffer[SCOUNT];
float analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, temperature = 25;
int previousTds = 0, currentTds = 0;  //블루투스로 도배 못하게
float TdsStandardValue = -1;
float upperTds = -1;
//수온
unsigned long previousTemp = 0;
OneWire ds(TemperaturePin);
float upperTemperature = 0, lowerTemperature = 0;
int previousTemperature = -1, currentTemperature = 0;  //블루투스로 도배 못하게
//서보모터
unsigned long previousServo = 0;
Servo servo;
bool doorOpen = false;
//워터펌프
bool isPumpActivating = false, turnOn = false;  //물이 더러워지면 워터펌프 작동. 워터 펌프가 작동되는 동안 해당 함수를 다시 실행시키면 안됨
unsigned long previousPump = 0;
//버튼 모드
String mode;
bool isInitialized = false;
//그래프 그리기를 위한 값 전송
unsigned long previousGraph = 0;
float tdsGraph = -1, temperatureGraph = -1;
extern volatile unsigned long timer0_millis; //타이머변수
unsigned long timeVal; //이전시간
unsigned long readTime; //현재타이머시간
int hour, min, sec;

void setup() {
  Serial.begin(9600);  //115200으로 하면 앱에서 보낸 값 못 읽어옴
  //수질
  pinMode(TdsSensorPin, INPUT);
  //블루투스
  bluetooth.begin(9600);
  //서보모터
  servo.attach(servoPin);
  servo.write(145); //서보모터 닫아두기
  //워터펌프
  pinMode(ADD_AA, OUTPUT);
  pinMode(ADD_AB, OUTPUT);
  pinMode(SUB_AA, OUTPUT);
  pinMode(SUB_AB, OUTPUT);
}

void loop() {
  if (!isInitialized) {
    if (bluetooth.available()) {
      mode = bluetooth.readStringUntil('@');
      if (mode == "set") {
        String from = bluetooth.readStringUntil('@');
        String initValues[4];
        split(from, initValues, ',');
        lowerTemperature = initValues[0].toFloat();
        upperTemperature = initValues[1].toFloat();
        TdsStandardValue = initValues[2].toFloat();

        String startedTime[3];
        split(initValues[3], startedTime, ' ');
        hour = startedTime[0].toInt();
        min = startedTime[1].toInt();
        sec = startedTime[2].toInt();
        Serial.print(hour); Serial.print(" "); Serial.print(min); Serial.print(" "); Serial.println(sec);
        timer0_millis = ((long)hour*3600+min*60+sec)*1000;
        printTimepoint = analogSampleTimepoint = previousTemp = previousServo = previousPump = previousGraph = timer0_millis;
        timeVal=millis();

        upperTds = TdsStandardValue * 1.3;
        // Serial.println(lowerTemperature);
        // Serial.println(upperTemperature);
        // Serial.println(TdsStandardValue);
        isInitialized = true;
        mode = "";
        Serial.println("설정 끝.");
      }
    }
  }
  else {
    start();
  }
}

void start() {
  if (bluetooth.available()) {
    mode = bluetooth.readStringUntil('@');  //set, tds, temperature
    Serial.print("mode : ");
    Serial.println(mode);
  }

  //밥주기
  if (mode == "servo") {
    doorOpen = true;
    mode = "";
  }
  if (doorOpen) {  //밖으로 빼줌
    activateServo();
  }

  //수질관리
  float tdsValue = getTdsValue();
  if (tdsValue != -1) {
    tdsGraph = tdsValue;
    Serial.print("tdsValue : "); Serial.println(tdsValue);
    if (mode == "tds") {
      String str = "tds@";
      str += String(tdsValue);
      str += '@';
      bluetooth.print(str);
      mode = "";
    }

    String str = "water@";
    if (!isPumpActivating && tdsValue > upperTds) {  //워터 펌프 작동 X && 기준치보다 높으면
      Serial.print("tdsValue("); Serial.print(tdsValue); Serial.print(") > upperTds("); Serial.print(upperTds); ;Serial.println(")이므로 펌프 작동");
      turnOn = true;
      isPumpActivating = true;
      str += "정수중...@";
      currentTds = 1;
    }

    if (tdsValue < upperTds) {  //탁도 안정화되면
      if (isPumpActivating) Serial.println("수질 안정화 됨");
      turnOn = false;
      isPumpActivating = false;
      str += "수질 유지중...@";
      currentTds = 2;
    }

    if (currentTds != previousTds) {
      bluetooth.print(str);
      Serial.print("최대 탁도 값 : "); Serial.println(upperTds);
      Serial.print("현재 탁도 값 : "); Serial.println(tdsValue);
      previousTds = currentTds;
    }
  }

  activatePump();

  //수온관리
  float temperatureValue = getTemperature();  //수질 측정값 계속
  if (temperatureValue == -1000) {
    Serial.println("센서 연결 불량.");
  }
  else if (temperatureValue != -1) {
    temperatureGraph = temperatureValue;
    Serial.print("온도 : "); Serial.println(temperatureValue);
    if (mode == "temperature") {
      String str = "temperature@";
      str += String(temperatureValue);
      str += '@';
      bluetooth.print(str);
      mode = "";
    }

    String str1 = "uplow@";
    if (temperatureValue < lowerTemperature) {  //1
      str1 += "수온이 너무 낮습니다.@";
      currentTemperature = 1;
    }
    else if (temperatureValue > upperTemperature) {  //2
      str1 += "수온이 너무 높습니다.@";
      currentTemperature = 2;
    }
    else if (temperatureValue <= upperTemperature && temperatureValue >= lowerTemperature) {  //3
      str1 += "수온 유지중...@";
      currentTemperature = 3;
    }
    if (currentTemperature != previousTemperature) {  //상시 측정되는 수온값이 블루투스로 너무 많이 전송되는 현상. 다른 값들이 블루투스로 전달될 기회까지 뺏음.
      bluetooth.print(str1);
      if (currentTemperature == 1) {
        Serial.print("현재값("); Serial.print(temperatureValue); Serial.print(") < 최솟값("); Serial.print(lowerTemperature); Serial.println(")");
      }
      else if (currentTemperature == 2) {
        Serial.print("현재값("); Serial.print(temperatureValue); Serial.print(") > 최댓값("); Serial.print(upperTemperature); Serial.println(")");
      }
      else {
        Serial.print("최솟값("); Serial.print(lowerTemperature); Serial.print(") < "); Serial.print("현재값("); Serial.print(temperatureValue); Serial.print(") < 최댓값("); Serial.print(upperTemperature); Serial.println(")");
      }
      previousTemperature = currentTemperature;
    }
  }

  setCurrentTime();
  if (tdsGraph != -1 && temperatureGraph != -1) {
    graphValues();
  }
}

void activateServo() {
  const int angles[2] = {45, 150};
  static int index = 0;
  unsigned long currentServo = millis();

  if (currentServo - previousServo >= ServoActivatingTime) {
    previousServo = currentServo;
    servo.write(angles[index]);
    // Serial.print("서보모터 각도 : "); Serial.println(angles[index]);
    if (index == 1) doorOpen = false;
    index = !index;
  }
}

void activatePump() {
  if (!turnOn) {
    digitalWrite(ADD_AA, LOW);
    digitalWrite(ADD_AB, LOW);
    digitalWrite(SUB_AA, LOW);
    digitalWrite(SUB_AB, LOW);
    return;
  }
  
  if (isPumpActivating) {
    digitalWrite(ADD_AA, HIGH);
    digitalWrite(ADD_AB, LOW);
  }

  unsigned long currentPump = millis();
  static bool pumpFlag = true;
  if (currentPump - previousPump >= 1000U && currentPump - previousPump <= 600000U) {
    previousPump = currentPump;
    if (pumpFlag) {
      digitalWrite(SUB_AA, HIGH);
      digitalWrite(SUB_AB, LOW);
    }
    else {
      digitalWrite(SUB_AA, LOW);
      digitalWrite(SUB_AB, LOW);
    }
    pumpFlag = !pumpFlag;
  }
}

void setCurrentTime() {
  if(millis()-timeVal>=1000) { //1초 단위 출력
    readTime = millis()/1000;
    
    if(millis()>=86400000){
      timer0_millis=0;
    }
    timeVal = millis();
  
    sec = readTime%60;
    min = (readTime/60)%60;
    hour = (readTime/(60*60))%24;
  }
}

void graphValues() {
  String values = "graph@";
  unsigned long currentGraph = millis();
  
  if (currentGraph - previousGraph >= GraphDelayTime && currentGraph - previousGraph <= GraphDelayTime + 600000U) {
    previousGraph = currentGraph;
    unsigned long time = hour * 10000L + min * 100L + sec;
    values += String(tdsGraph) + "@" + String(temperatureGraph) + "@" + time + "@";
    bluetooth.print(values);
    Serial.println("그래프 그리기");
    tdsGraph = -1; temperatureGraph = -1;
  }
}

float getTemperature() {
  unsigned long currentTemp = millis();
  
  if (currentTemp - previousTemp >= TemperatureDelayTime && currentTemp - previousTemp <= TemperatureDelayTime + 600000U) {
    previousTemp = currentTemp;
    return measureTemperature();
  }

  return -1;
}

float measureTemperature() {  //온도 측정 후 반환하는 함수
  byte data[12];
  byte addr[8];
  if (!ds.search(addr)) {
    ds.reset_search();
    return -1000;
  }
  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }
  if (addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for (int i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB);
  float TemperatureSum = tempRead / 16;
  return TemperatureSum;
}


float getTdsValue() {  //수질 측정
  const int VREF = 5.0;

  unsigned long currentAnalogSampleTimepoint = millis();
  if (currentAnalogSampleTimepoint - analogSampleTimepoint > 40U)  //U는 unsigned int
  {
    analogSampleTimepoint = currentAnalogSampleTimepoint;
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }
  unsigned long currentPrintTimepoint = millis();
  if (currentPrintTimepoint - printTimepoint >= TdsDelayTime && currentPrintTimepoint - printTimepoint <= TdsDelayTime + 600000U) {
    printTimepoint = currentPrintTimepoint;
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVolatge = averageVoltage / compensationCoefficient;
    float tds = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;

    return tds;
  }

  return -1;
}

int getMedianNum(float bArray[], int iFilterLen) {  //튀는 값 제거
  int bTab[iFilterLen];
  int bTemp;

  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];

  qsort(bTab, iFilterLen, sizeof(int), sort_asc);

  if ((iFilterLen & 1) > 0)  //이진수에서 가장 하위 비트로 짝수 홀수 판별 가능
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;

  return bTemp;
}

int sort_asc(const void* item1, const void* item2) {  //퀵정렬을 위한 비교 함수
  // int 비교라고 가정하고 int 값으로 캐스팅
  int a = *((int*)item1);
  int b = *((int*)item2);

  return a - b;
}

void split(String from, String values[], const char token) {
  int count = 0, findToken = 0;
  String sTemp = "";
  String sCopy = from;

  while (true) {
    findToken = sCopy.indexOf(token);

    if (findToken != -1) {
      sTemp = sCopy.substring(0, findToken);
      Serial.println(sTemp);
      sCopy = sCopy.substring(findToken + 1);
    }
    else {
      values[count] = sCopy;
      break;
    }
    values[count++] = sTemp;
  }
}
