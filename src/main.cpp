#include <WiFi.h>
#include <SPI.h>
#include <WebSocketsServer.h>

// [해결] 1. STM32와 맞추기 위한 시스템 상태 정의
typedef enum {
    STATE_SAFE = 0,
    STATE_WARNING = 1,
    STATE_ALERT = 2,
    STATE_DANGER = 3
} SystemState;

// Wi-Fi 및 웹소켓 설정
const char* ssid = "YOUR_WIFI_SSID";         // 와이파이 이름
const char* password = "YOUR_WIFI_PASSWORD"; // 와이파이 비밀번호
WebSocketsServer webSocket = WebSocketsServer(81);

// SPI 핀 정의
#define VSPI_MISO   19
#define VSPI_MOSI   23
#define VSPI_SCLK   18
#define VSPI_SS     5

// 시계용 변수 (정상 컴파일을 위해 전역 변수로 관리)
uint8_t current_hour = 12;
uint8_t current_minute = 30;
SystemState current_state = STATE_SAFE; // 수위 상태 저장용 변수

// 웹소켓 이벤트 처리기
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED: // [수정] WSType -> WStype (t가 소문자)
      Serial.printf("[%u] Disconnected!\n", num);
      break;
      
    case WStype_CONNECTED:    // [수정] WSType -> WStype
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
      
    case WStype_TEXT:         // [수정] WSType -> WStype
      Serial.printf("[%u] Received text: %s\n", num, payload);
      break;
      
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // SPI 핀 설정 및 초기화
  pinMode(VSPI_SS, OUTPUT);
  digitalWrite(VSPI_SS, HIGH);
  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 통신 속도 안정화

  // Wi-Fi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP()); 

  // 웹소켓 시작
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();

  // 1초마다 동기화 및 데이터 송수신 수행
  static unsigned long lastTime = 0;
  if (millis() - lastTime > 1000) {
    lastTime = millis();

    // 임시 시간 갱신
    current_minute++;
    if(current_minute >= 60) {
      current_minute = 0;
      current_hour = (current_hour + 1) % 24;
    }

    // --- [해결] 2. SPI 1회 통신으로 송수신 병합 (Full-Duplex) ---
    // 송신 데이터 버퍼 설정 (이번 루프에서 결정해둔 상태와 시간을 STM32에 전달)
    uint8_t txData[3];
    txData[0] = (uint8_t)current_state; 
    txData[1] = current_hour;           
    txData[2] = current_minute;         

    uint8_t rxData[3] = {0,};
    uint16_t waterValue = 0;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(VSPI_SS, LOW); // CS Active
    
    // 송수신 동시에 진행 (클럭 유실이나 싱크 밀림 원천 차단)
    rxData[0] = SPI.transfer(txData[0]); // STM32의 마커(0xAA) 가져오면서 상태 송신
    rxData[1] = SPI.transfer(txData[1]); // STM32의 ADC MSB 가져오면서 시간(시) 송신
    rxData[2] = SPI.transfer(txData[2]); // STM32의 ADC LSB 가져오면서 시간(분) 송신

    digitalWrite(VSPI_SS, HIGH); // CS Inactive
    SPI.endTransaction();

    // STM32로부터 정상 데이터가 왔는지 판별 (시작 마커 0xAA 확인)
    if (rxData[0] == 0xAA) {
      waterValue = (rxData[1] << 8) | rxData[2];
      Serial.printf("Received Water Value: %d\n", waterValue);
    } else {
      Serial.println("SPI Sync Error: Marker not matched.");
    }

    // --- 3. 수위 판단 알고리즘 (다음 주기에 전달됨) ---
    if (waterValue >= 3500)      current_state = STATE_DANGER;
    else if (waterValue >= 2500) current_state = STATE_ALERT;
    else if (waterValue >= 1200) current_state = STATE_WARNING;
    else                         current_state = STATE_SAFE;

    // --- 4. 웹소켓 전송 단계 ---
    String jsonPayload = "{\"waterValue\":" + String(waterValue) + 
                         ",\"state\":" + String((uint8_t)current_state) + 
                         ",\"time\":\"" + String(current_hour) + ":" + String(current_minute) + "\"}";
    
    webSocket.broadcastTXT(jsonPayload); // 웹소켓 대시보드로 실시간 갱신!
  }
}