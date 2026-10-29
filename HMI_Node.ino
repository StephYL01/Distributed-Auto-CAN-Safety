#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
//  HMI TX NODE - Reads user inputs and publishes to CAN Bus
//  Features: FreeRTOS, CSMA/CR Arbitration, AEB Safety Logic
// ============================================================

// --- Hardware Pin Configuration ---
#define PIN_STEERING_POT   4    // ADC2_CH0 (Steering Wheel)
#define PIN_HORN_SENSOR    5    // TTP223 Capacitive Sensor (Horn)
#define PIN_WIPER_SWITCH   21   // Homing switch (Wipers)
#define PIN_CAN_TX         22   // SN65HVD230 CTX
#define PIN_CAN_RX         23   // SN65HVD230 CRX

// New Pins for Environment/Safety Sensors
#define PIN_DOOR_HALL      15   // A3144 Hall Effect Sensor (Door Status)
#define PIN_US_TRIG        19   // HC-SR04 Trigger
#define PIN_US_ECHO        18   // HC-SR04 Echo (Requires 5V to 3.3V voltage divider)

// --- CAN IDs (Hardware Arbitration Priority) ---
#define CAN_ID_AEB_CRITICAL  0x010  // Highest Priority: Autonomous Emergency Braking
#define CAN_ID_HMI_CONTROLS  0x120  // Medium Priority: User inputs (Steering, Wipers)
#define CAN_ID_ENV_DATA      0x121  // Medium Priority: Environment (Distance, Door)

// --- System Thresholds & Timings ---
#define AEB_CRITICAL_DIST_CM 15   // Distance threshold to trigger Emergency Braking
#define SENSOR_PERIOD_MS     50   // 50ms to allow ultrasonic echo return
#define CAN_PERIOD_MS        20   // 50Hz CAN Bus refresh rate
#define LOG_PERIOD_MS        2500 // Console log frequency

#define DEBOUNCE_SAMPLES     3     

// ------------------------------------------------------------
//  Frame Payloads (5 Bytes each for E2E Protection):
//  0x010 (AEB): [0]=AEB_Active(1) [1]=Dist_cm [2]=0 [3]=AliveCounter [4]=CRC8
//  0x120 (HMI): [0]=SteerAngle [1]=Wiper [2]=Horn [3]=AliveCounter [4]=CRC8
//  0x121 (ENV): [0]=Dist_MSB [1]=Dist_LSB [2]=Door(1=Open/0=Closed) [3]=Alive [4]=CRC8
//  CRC8: poly 0x07, init 0xFF, calculated over ID LSB + Bytes 0..3
// ------------------------------------------------------------

struct SharedData {
  uint8_t  steerAngle;
  uint8_t  wiperActive;
  uint8_t  hornActive;
  uint16_t obstacleDist;
  uint8_t  doorOpen;
};

// Global shared state with RTOS Mutex protection
static SharedData  g_systemState = {90, 0, 0, 999, 0};
static portMUX_TYPE g_stateMutex = portMUX_INITIALIZER_UNLOCKED;

// Flag pentru depanare: blocheaza suprascrierea distantei de catre senzorul fizic
static bool g_manualOverride = false;

// Diagnostics counters (Written only by CAN TX Task)
static uint32_t g_txErrorCount = 0;
static uint32_t g_busRecoveryCount = 0;

// --- Utilities ---
// CRC8 Calculation for E2E (End-to-End) Protection
static uint8_t calculateCRC8(uint16_t id, const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  uint8_t first = (uint8_t)(id & 0xFF);
  for (int i = -1; i < len; i++) {
    crc ^= (i < 0) ? first : data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

struct Debouncer {
  bool    stableState;
  bool    lastRawState;
  uint8_t sampleCount;
};

static bool applyDebounce(Debouncer &db, bool rawState);

static bool applyDebounce(Debouncer &db, bool rawState) {
  if (rawState == db.lastRawState) {
    if (db.sampleCount < DEBOUNCE_SAMPLES) db.sampleCount++;
  } else {
    db.lastRawState = rawState;
    db.sampleCount = 1;
  }
  if (db.sampleCount >= DEBOUNCE_SAMPLES) db.stableState = db.lastRawState;
  return db.stableState;
}

// Monitor TWAI driver status and recover from BUS-OFF states
static void checkAndRecoverCANBus() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    g_busRecoveryCount++;
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();   // Recovery complete, restart driver
  }
}

// Construct and transmit a CAN Frame with Alive Counter and CRC
static void transmitCANFrame(uint16_t id, uint8_t *payload, uint8_t &aliveCounter) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 0; // Standard 11-bit ID
  msg.rtr = 0;  // Data frame
  msg.data_length_code = 5;
  
  for (int i = 0; i < 4; i++) {
    msg.data[i] = payload[i];
  }
  
  msg.data[3] = aliveCounter++; // E2E Alive Counter
  msg.data[4] = calculateCRC8(id, msg.data, 4); // E2E CRC8

  if (twai_transmit(&msg, 0) != ESP_OK) {
    g_txErrorCount++;
  }
}

// --- Task Prototypes ---
void TaskPollSensors(void *pvParameters);
void TaskTransmitCAN(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- [HMI] Starting HMI Node (FreeRTOS) ---");

  // Configure Hardware Inputs
  pinMode(PIN_WIPER_SWITCH, INPUT_PULLUP);
  pinMode(PIN_STEERING_POT, INPUT);
  pinMode(PIN_HORN_SENSOR, INPUT);
  analogReadResolution(12);

  // Configure Environment Sensors
  pinMode(PIN_DOOR_HALL, INPUT_PULLUP);
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW); 

  // Initialize TWAI Driver (CAN 500 kbps)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("[HMI] FATAL ERROR: TWAI driver failed to start. Rebooting...");
    delay(2000);
    ESP.restart();
  }
  Serial.println("[HMI] TWAI driver initialized at 500 kbps.");

  // RTOS Task Creation (Priority 2 for CAN TX ensures deterministic bus timing)
  xTaskCreate(TaskPollSensors, "SensorTask", 4096, NULL, 1, NULL);
  xTaskCreate(TaskTransmitCAN, "CanTxTask",  4096, NULL, 2, NULL);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'D' || cmd == 'd') {
      // 1. Citim numărul ÎNAINTE de a bloca resursele
      int dist = Serial.parseInt();

      // 2. Blocăm resursele strict pentru scrierea variabilelor
      portENTER_CRITICAL(&g_stateMutex);
      g_systemState.obstacleDist = dist;
      g_manualOverride = true;
      portEXIT_CRITICAL(&g_stateMutex);

      Serial.printf("\n[DEBUG] Distanta setata MANUAL la %d cm.\n", dist);
    } 
    else if (cmd == 'R' || cmd == 'r') {
      portENTER_CRITICAL(&g_stateMutex);
      g_manualOverride = false;
      portEXIT_CRITICAL(&g_stateMutex);
      
      Serial.println("\n[DEBUG] Control redat senzorului fizic.");
    }
    else if (cmd == 'U' || cmd == 'u') {
      portENTER_CRITICAL(&g_stateMutex);
      g_systemState.doorOpen = !g_systemState.doorOpen;
      bool currentState = g_systemState.doorOpen; // Salvăm starea pentru printare sigură
      portEXIT_CRITICAL(&g_stateMutex);
      
      Serial.printf("\n[DEBUG] Usa %s din Serial.\n", currentState ? "DESCHISA" : "INCHISA");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(100)); // Previne blocarea procesorului
}

// --- Task: Sensor Polling ---
void TaskPollSensors(void *pvParameters) {
  Debouncer wiperDebounce = {false, false, 0};
  Debouncer hornDebounce  = {false, false, 0};
  int filteredPotValue = analogRead(PIN_STEERING_POT);

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // 1. Read HC-SR04 Ultrasonic (AEB Sensor)
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
    
    // pulseIn blocks task until echo returns. Max timeout: 20000us (~3.4 meters)
    unsigned long echoDuration = pulseIn(PIN_US_ECHO, HIGH, 20000);
    uint16_t distanceCm = (echoDuration == 0) ? 999 : (echoDuration * 0.0343) / 2;

    // 2. Read Hall Effect Door Sensor
    // PULLUP active: HIGH = No magnet (Door Open), LOW = Magnet (Door Closed)
    bool isDoorOpen = (digitalRead(PIN_DOOR_HALL) == HIGH);

    // 3. Read Wiper Switch (Active LOW)
    bool isWiperActive = applyDebounce(wiperDebounce, digitalRead(PIN_WIPER_SWITCH) == LOW);

    // 4. Read Horn Sensor (Active HIGH)
    bool isHornActive = applyDebounce(hornDebounce, digitalRead(PIN_HORN_SENSOR) == HIGH);

    // 5. Read Steering Potentiometer (Exponential Moving Average Filter)
    int rawPotValue = analogRead(PIN_STEERING_POT);
    filteredPotValue += (rawPotValue - filteredPotValue) / 4;
    uint8_t steeringAngle = (uint8_t)constrain((filteredPotValue * 180) / 4095, 0, 180);

    // Atomically update the global state
    portENTER_CRITICAL(&g_stateMutex);
    
    // Actualizăm distanța doar dacă NU suntem în modul de override manual
    if (!g_manualOverride) {
      g_systemState.obstacleDist = distanceCm;
    }
    
    // Suprascriem ușa din senzor doar dacă dorim comportamentul fizic, 
    // altfel o lăsăm sub controlul toggle-ului 'U' din Serial
    if (g_systemState.doorOpen == 0 || isDoorOpen) {
      g_systemState.doorOpen = isDoorOpen ? 1 : 0;
    }
    
    g_systemState.steerAngle = steeringAngle;
    g_systemState.wiperActive = isWiperActive ? 1 : 0;
    g_systemState.hornActive = isHornActive ? 1 : 0;
    
    portEXIT_CRITICAL(&g_stateMutex);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// --- Task: CAN Transmission & Safety Logic ---
void TaskTransmitCAN(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  uint8_t aliveAeb = 0;
  uint8_t aliveControls = 0;
  uint8_t aliveEnv = 0;
  uint32_t lastLogTime = 0;

  for (;;) {
    // Take a consistent snapshot of the current state
    SharedData currentState;
    portENTER_CRITICAL(&g_stateMutex);
    currentState = g_systemState;
    portEXIT_CRITICAL(&g_stateMutex);

    checkAndRecoverCANBus();

    // --- SAFETY LOGIC (AEB) ---
    bool isAebActive = (currentState.obstacleDist < AEB_CRITICAL_DIST_CM);

    if (isAebActive) {
      // HAZARD DETECTED: Transmit high-priority AEB frame.
      // We deliberately DO NOT transmit HMI controls, simulating a safe-state override.
      uint8_t aebPayload[4] = { 1, (uint8_t)currentState.obstacleDist, 0, 0 };
      transmitCANFrame(CAN_ID_AEB_CRITICAL, aebPayload, aliveAeb);
    } else {
      // NORMAL OPERATION: Transmit HMI steering and wiper controls.
      uint8_t ctrlPayload[4] = { currentState.steerAngle, currentState.wiperActive, currentState.hornActive, 0 };
      transmitCANFrame(CAN_ID_HMI_CONTROLS, ctrlPayload, aliveControls);
    }

    // Always transmit environment data (Door status is also a critical safety factor)
    uint8_t envPayload[4] = { (uint8_t)(currentState.obstacleDist >> 8), (uint8_t)(currentState.obstacleDist & 0xFF), currentState.doorOpen, 0 };
    transmitCANFrame(CAN_ID_ENV_DATA, envPayload, aliveEnv);

    // Periodic Console Logging
    uint32_t now = millis();
    if (now - lastLogTime >= LOG_PERIOD_MS) {
      lastLogTime = now;
      if (isAebActive) {
        Serial.printf("[HMI] !!! AEB ACTIVE !!! Obstacle at %d cm. Steering disabled.\n", currentState.obstacleDist);
      } else {
        Serial.printf("[HMI] Steer:%3d | Wiper:%d | Horn:%d | Dist:%3dcm | Door:%s | txErr:%lu\n",
                      currentState.steerAngle, currentState.wiperActive, currentState.hornActive, 
                      currentState.obstacleDist, (currentState.doorOpen ? "OPEN" : "CLOSED"),
                      (unsigned long)g_txErrorCount);
      }
    }

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CAN_PERIOD_MS));
  }
}
