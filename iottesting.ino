#include <esp_sleep.h>  // Deep Sleep 제어 라이브러리
#include <WiFi.h>       // Wi-Fi 연결 라이브러리
#include <Arduino_JSON.h>
#include <AWS_IOT.h>
#include "driver/rtc_io.h"  //rtc 핀을 사용하기 위한 헤더파일 추가

AWS_IOT binstate;

#define ANALOG_IN_PIN 35

const char* ssid = "SSIBSARE";
const char* password = "song1472";

char HOST_ADDRESS[] = "a7fbxwr7ybl3j-ats.iot.ap-northeast-2.amazonaws.com";
char CLIENT_ID[] = "bin_1";
char sTOPIC_NAME[] = "$aws/things/ThrashModule1/shadow/update/accepted";  // subscribe topic name
char pTOPIC_NAME[] = "$aws/things/ThrashModule1/shadow/update";           // publish topic name

int status = WL_IDLE_STATUS;
int msgCount = 0, msgReceived = 0;

char payload[512];
char rcvdPayload[512];

unsigned long preMil = 0;
const long intMil = 5000;

int maind, subd, flames, bat;  //json 형식으로 보내기위한 최종 전역변수

//아래는 전압측정을 위한 변수
float adc_voltage = 0.0;
float in_voltage = 0.0;

float R1 = 30000.0;
float R2 = 7500.0;

float ref_voltage = 3.3;

int adc_value = 0;

void mySubCallBackHandler(char* topicName, int payloadLen, char* payLoad) {
  strncpy(rcvdPayload, payLoad, payloadLen);
  rcvdPayload[payloadLen] = 0;
  msgReceived = 1;
}  //메세지를 받았을때의 인터럽트


// 10분(600초) * 1초당 마이크로초 Deep sleep 시간 설정
#define SLEEP_DURATION 10 * 1000000ULL

volatile bool flameDetected = false;

// 인터럽트 서비스 루틴 (Deep Sleep 모드 해제)
void IRAM_ATTR flameInterrupt() {
  flameDetected = true;  // 인터럽트 플래그 설정
  flames = 1;
}

void setup() {
  Serial.begin(115200);

  // 화염 감지 센서 인터럽트 핀 설정
  pinMode(34, INPUT_PULLUP);                                            // 화염 감지 센서 연결 핀
  attachInterrupt(digitalPinToInterrupt(34), flameInterrupt, FALLING);  // FALLING 엣지에서 인터럽트

  WiFi.disconnect(true);  // Wi-Fi 초기화
  delay(100);             // 모듈 리셋을 위한 시간 대기
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 6) {  // 최대 6초 대기
    delay(1000);
    Serial.println("Reconnecting to WiFi...");
    retryCount++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi");
    esp_restart();  // 연결 실패 시 강제 재부팅
  } else {
    Serial.println("Connected to WiFi");
  }


  if (binstate.connect(HOST_ADDRESS, CLIENT_ID) == 0) {
    Serial.println("Connected to AWS");
    delay(1000);
    if (0 == binstate.subscribe(sTOPIC_NAME, mySubCallBackHandler)) {
      Serial.println("Subscribe Successfull");
    } else {
      Serial.println("Subscribe Failed, Check the Thing Name and Certificates");
      while (1)
      esp_restart();
        ;
    }
  } else {
    Serial.println("AWS connection failed, Check the HOST Address");
    while (1)
    esp_restart();
      ;
  }

  pinMode(27, OUTPUT);  // led 핀 설정, Deep sleep 모드로 들어가는경우 모든 핀이 LOW로 전환되기때문에 RTC 핀을 활용하였음
  rtc_gpio_init(GPIO_NUM_4);
  rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_hold_en(GPIO_NUM_4);  // Deep Sleep에서도 상태 유지

  // 초음파 센서 핀 설정
  pinMode(25, OUTPUT);  // Sonar 1 Trig
  pinMode(33, OUTPUT);  // Sonar 2 Trig
  pinMode(4, OUTPUT);
  pinMode(26, INPUT);  // Sonar 1 Echo
  pinMode(32, INPUT);  // Sonar 2 Echo

  // 이전 웨이크업 원인 확인
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("화염 감지 센서로 인해 깨어났습니다!");
    flames = 1;
  } else if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("타이머로 인해 깨어났습니다!");
  } else {
    Serial.println("정상 부팅 중...");
  }

  // 타이머 웨이크업 및 외부 GPIO 웨이크업 설정
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION);  // 10분 타이머 웨이크업 설정
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);   // GPIO34 (화염 감지 센서)에서 LOW 신호로 웨이크업
}

void loop() {

  // 화염 감지 인터럽트 발생 여부 확인
  if (flameDetected) {
    flames = 1;
    flameDetected = false;  // 플래그 리셋
    Serial.println("화염 감지: 긴급 상황!");
  }

  // 초음파 센서 측정 및 출력
  sonar_1();
  sonar_2();
  //배터리 잔량계산
  batt_con();
  send_data_aws();
  flames = 0;

  // Deep Sleep으로 전환
  Serial.println("Deep Sleep 모드로 전환...");
  delay(100);

  esp_deep_sleep_start();
}

// 초음파 센서 1 측정
void sonar_1() {
  long duration;
  int trigPin = 25;
  int echoPin = 26;

  do {
    int retryCount = 0;
    const int maxRetries = 10;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 에코 신호 측정
    duration = pulseIn(echoPin, HIGH, 10000);

    retryCount++;
    if (retryCount >= maxRetries) {
      Serial.println("Failed to measure distance after retries");
      return;
    }

    if (duration <= 0) {
      Serial.println("No echo received (retrying)");
    }
  } while (duration <= 0);  // 유효한 값이 나올 때까지 반복

  // 거리 계산
  long distance = duration * 0.034 / 2;
  if (distance < 200 && distance > 2) {  // 유효 거리 필터
    Serial.print("Measured distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    maind = distance;
  } else {
    Serial.println("Out of range");
  }
}

// 초음파 센서 2 측정, 완전 찼는지 확인하는 초음파센서
void sonar_2() {
  long duration;
  int trigPin = 33;
  int echoPin = 32;

  do {
    int retryCount = 0;
    const int maxRetries = 10;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 에코 신호 측정
    duration = pulseIn(echoPin, HIGH, 10000);

    retryCount++;
    if (retryCount >= maxRetries) {
      Serial.println("Failed to measure distance after retries");
      return;
    }

    if (duration <= 0) {
      Serial.println("No echo received (retrying)");
    }
  } while (duration <= 0);  // 유효한 값이 나올 때까지 반복

  // 거리 계산
  long distance = duration * 0.034 / 2;
  if (distance < 200 && distance > 2) {  // 유효 거리 필터
    Serial.print("Measured distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    subd = distance;
  } else {
    Serial.println("Out of range");
  }

  if (distance < 10) {              // 쓰레기통이 꽉 찼을 때 켜는 LED
    rtc_gpio_hold_dis(GPIO_NUM_4);  // 제어 해제
    digitalWrite(4, HIGH);
    rtc_gpio_hold_en(GPIO_NUM_4);  // 다시 홀드
    Serial.println("LED ON");
  } else {
    rtc_gpio_hold_dis(GPIO_NUM_4);  // 제어 해제
    digitalWrite(4, LOW);
    rtc_gpio_hold_en(GPIO_NUM_4);  // 다시 홀드
  }


  delay(50);  // 측정 간격
}


void send_data_aws() {
  sprintf(payload, "{\"state\":{\"reported\":{\"name\": \"ThrashModule1\", \"mainUltrasonicSensor\":%d, \"subUltrasonicSensor\":%d, \"detectFlame\":%d, \"battery\":\"%d\"}}}",
          maind, subd, flames, bat);

  if (binstate.publish(pTOPIC_NAME, payload) == 0) {
    Serial.print("Publish Message:");
    Serial.println(payload);
  } else
    Serial.println("Publish failed");
}

void batt_con() {                           //배터리의 상태를 모니터링하는 함수
  for (int i = 0; i < 6; i++) {             //안정화를 위해 5번 반복측정
    adc_value = analogRead(ANALOG_IN_PIN);  //15번 핀으로 통해 모니터링

    adc_voltage = (adc_value * ref_voltage) / 4096.0;  //분해능이 12비트기에 4096

    in_voltage = adc_voltage / (R2 / (R1 + R2));
    in_voltage = in_voltage + 0.84;

    Serial.print("Input Voltage = ");
    Serial.println(in_voltage, 2);

    delay(500);
  }

  float output = mapFloat(in_voltage, 3.5, 4.3, 0.0, 100.0);  // 0~5를 0~100으로 매핑
  bat = (int)output;
  Serial.print("배터리 잔량");
  Serial.println(bat);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {  //배터리 잔량계산을 위한 float 형 map 함수
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
