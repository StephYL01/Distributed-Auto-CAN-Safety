#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/twai.h"

// ============================================================
//  BCM RX NODE - Body Control Module & Actuator Management
//  Features: FreeRTOS, E2E Protection (CRC), AEB Safe State
// ============================================================

// --- Hardware Pin Configuration ---
#define PIN_SERVO      19 
#define PIN_BUZZER     21 
#define PIN_DOME_LIGHT 5    // Interior dome light (Active when door is open)
#define PIN_CAN_TX     22 
#define PIN_CAN_RX     23 

// --- CAN IDs (Hardware Arbitration Priority) ---
#define CAN_ID_AEB_CRITICAL  0x010  // Highest Priority: Autonomous Emergency Braking
#define CAN_ID_HMI_CONTROLS  0x120  // Medium Priority: User inputs
#define CAN_ID_ENV_DATA      0x121  // Medium Priority: Environment

// --- System Timings ---
#define AEB_TIMEOUT_MS       500    // Time before auto-recovering from AEB if hazard clears

// --- Shared Global State ---
struct SystemState {
  uint8_t  steerAngle;
  uint8_t  wiperActive;
  uint8_t  hornActive;
  uint16_t obstacleDist;
  uint8_t  doorOpen;
  
  // Safety specific flags
  bool     aebActive;
  uint32_t lastAebTimestamp;
};

static SystemState g_currentState = {90, 0, 0, 999, 0, false, 0};
static portMUX_TYPE g_stateMutex = portMUX_INITIALIZER_UNLOCKED;

Servo steeringServo;

// --- Utilities (E2E Protection) ---
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

// --- Task Prototypes ---
void TaskReceiveCAN(void *pvParameters);
void TaskActuatorControl(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- [BCM] Starting BCM Node (FreeRTOS) ---");

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_DOME_LIGHT, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_DOME_LIGHT, LOW);

  // Allocate hardware timers for stable PWM generation
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(PIN_SERVO, 500, 2400);
  steeringServo.write(g_currentState.steerAngle);

  // Initialize TWAI Driver (CAN 500 kbps)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("[BCM] TWAI driver initialized at 500 kbps.");
  } else {
    Serial.println("[BCM] FATAL ERROR: TWAI driver failed to start.");
  }

  // RTOS Task Creation
  xTaskCreate(TaskReceiveCAN, "CanRxTask", 4096, NULL, 2, NULL);   
  xTaskCreate(TaskActuatorControl, "ActuatorTask", 4096, NULL, 1, NULL); 
}

void loop() {
  // Main loop suspended. FreeRTOS handles execution.
  vTaskDelay(portMAX_DELAY); 
}

// --- Task: CAN Reception & E2E Validation ---
void TaskReceiveCAN(void *pvParameters) {
  for (;;) {
    twai_message_t rx_msg;
    
    // Block task until a message is received
    if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
      
      // End-to-End (E2E) Protection: Validate DLC and CRC8 payload
      if (rx_msg.data_length_code != 5) continue; 
      
      uint8_t receivedCRC = rx_msg.data[4];
      uint8_t calculatedCRC = calculateCRC8(rx_msg.identifier, rx_msg.data, 4);
      
      if (receivedCRC != calculatedCRC) {
        Serial.println("[BCM] E2E Error: CRC mismatch. Frame dropped.");
        continue;
      }

      portENTER_CRITICAL(&g_stateMutex);
      
      // 1. Process Highest Priority Frame: AEB Hazard
      if (rx_msg.identifier == CAN_ID_AEB_CRITICAL) {
        g_currentState.aebActive = (rx_msg.data[0] == 1);
        g_currentState.obstacleDist = rx_msg.data[1];
        g_currentState.lastAebTimestamp = millis();
      } 
      // 2. Process Standard HMI Controls
      else if (rx_msg.identifier == CAN_ID_HMI_CONTROLS) {
        g_currentState.steerAngle = rx_msg.data[0];
        g_currentState.wiperActive = rx_msg.data[1];
        g_currentState.hornActive = rx_msg.data[2];
      } 
      // 3. Process Environment Data
      else if (rx_msg.identifier == CAN_ID_ENV_DATA) {
        g_currentState.obstacleDist = (rx_msg.data[0] << 8) | rx_msg.data[1]; 
        g_currentState.doorOpen = rx_msg.data[2];
      }
      
      portEXIT_CRITICAL(&g_stateMutex);
    }
  }
}

// --- Task: Actuator Logic & Safe State Execution ---
void TaskActuatorControl(void *pvParameters) {
  bool sweepDirection = true;
  int wiperAngle = 0;
  uint8_t lastDoorState = 255;

  for (;;) {
    SystemState localState;
    portENTER_CRITICAL(&g_stateMutex);
    localState = g_currentState;
    portEXIT_CRITICAL(&g_stateMutex);

    // Auto-recover from AEB Safe State if timeout exceeded (hazard cleared)
    if (localState.aebActive && (millis() - localState.lastAebTimestamp > AEB_TIMEOUT_MS)) {
      portENTER_CRITICAL(&g_stateMutex);
      g_currentState.aebActive = false;
      portEXIT_CRITICAL(&g_stateMutex);
      localState.aebActive = false;
      Serial.println("[BCM] Hazard cleared. Resuming normal operation.");
    }

    // 1. DOME LIGHT LOGIC (Safety Interlock feature)
    if (localState.doorOpen != lastDoorState) {
      digitalWrite(PIN_DOME_LIGHT, localState.doorOpen ? HIGH : LOW);
      Serial.printf("[BCM] Door status changed: %s\n", localState.doorOpen ? "OPEN" : "CLOSED");
      lastDoorState = localState.doorOpen;
    }

    // 2. SAFE STATE EXECUTION (AEB Hazard override)
    if (localState.aebActive) {
      // Ignore user inputs. Lock steering to center and sound continuous alarm.
      digitalWrite(PIN_BUZZER, HIGH);
      steeringServo.write(90); 
      vTaskDelay(pdMS_TO_TICKS(50));
      continue; // Skip normal actuator execution
    }

    // 3. NORMAL OPERATION: Buzzer & Horn Logic
    bool proximityAlert = (localState.obstacleDist > 0 && localState.obstacleDist < 50); 
    digitalWrite(PIN_BUZZER, (localState.hornActive == 1 || proximityAlert) ? HIGH : LOW);

    // 4. NORMAL OPERATION: Servo Control (Wiper vs Steering)
    if (localState.wiperActive == 1) {
      steeringServo.write(wiperAngle);
      if (sweepDirection) {
        wiperAngle += 15;
        if (wiperAngle >= 180) { wiperAngle = 180; sweepDirection = false; }
      } else {
        wiperAngle -= 15;
        if (wiperAngle <= 0) { wiperAngle = 0; sweepDirection = true; }
      }
      vTaskDelay(pdMS_TO_TICKS(30)); 
    } else {
      steeringServo.write(localState.steerAngle);
      vTaskDelay(pdMS_TO_TICKS(20)); 
    }
  }
}