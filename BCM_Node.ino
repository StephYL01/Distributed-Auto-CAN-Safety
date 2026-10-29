#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/twai.h"

// ============================================================
//  BCM RX NODE - Body Control Module & Actuator Management
//  Features: FreeRTOS, E2E Protection (CRC8 + Alive Counter),
//            AEB Safe State, Communication-Loss Supervision
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

// --- System Timings & Thresholds ---
#define AEB_TIMEOUT_MS       500    // Time before auto-recovering from AEB if hazard clears
#define COMM_TIMEOUT_MS      150    // No valid frame for this long => link lost => safe outputs
                                    // (HMI sends 0x121 every 20 ms, so it acts as a heartbeat)
#define CAN_RX_TIMEOUT_MS    100    // twai_receive() wake-up period (keeps bus supervision alive)
#define BUS_CHECK_PERIOD_MS  1000   // How often the TWAI driver state is checked
#define LOG_PERIOD_MS        2000   // Diagnostic log period
#define PROXIMITY_ALERT_CM   50     // Buzzer threshold in normal operation
#define SAFE_STEER_ANGLE     90     // Steering position in any safe state

// --- E2E frame bookkeeping (one alive counter per CAN ID) ---
#define NUM_FRAME_TYPES      3
enum FrameType { FRAME_AEB = 0, FRAME_CONTROLS = 1, FRAME_ENV = 2 };

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

  // Link supervision (updated on every valid, non-duplicate frame)
  bool     everRx;
  uint32_t lastFrameMs;

  // Diagnostics counters
  uint32_t crcErrors;
  uint32_t lostFrames;
  uint32_t duplicateFrames;
  uint32_t busRecoveries;
};

static SystemState g_currentState = {
  90, 0, 0, 999, 0,     // steer, wiper, horn, distance, door
  false, 0,             // aebActive, lastAebTimestamp
  false, 0,             // everRx, lastFrameMs
  0, 0, 0, 0            // crcErrors, lostFrames, duplicateFrames, busRecoveries
};
// Spinlock (portMUX) protecting g_currentState across both tasks/cores
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

// Maps a CAN ID to its alive-counter slot, or -1 for unknown IDs
static int frameTypeFromId(uint32_t id) {
  switch (id) {
    case CAN_ID_AEB_CRITICAL: return FRAME_AEB;
    case CAN_ID_HMI_CONTROLS: return FRAME_CONTROLS;
    case CAN_ID_ENV_DATA:     return FRAME_ENV;
    default:                  return -1;
  }
}

// Monitor TWAI driver status and recover from BUS-OFF / STOPPED states
static void checkAndRecoverCANBus() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    portENTER_CRITICAL(&g_stateMutex);
    g_currentState.busRecoveries++;
    portEXIT_CRITICAL(&g_stateMutex);
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();   // Recovery complete, restart driver
  }
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
  steeringServo.write(SAFE_STEER_ANGLE);

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
  uint8_t  lastAlive[NUM_FRAME_TYPES] = {0, 0, 0};
  bool     haveAlive[NUM_FRAME_TYPES] = {false, false, false};
  uint32_t lastValidMs = 0;
  uint32_t lastBusCheckMs = 0;

  for (;;) {
    twai_message_t rx_msg;

    // Wake up periodically even without traffic, so bus supervision keeps running
    esp_err_t rxResult = twai_receive(&rx_msg, pdMS_TO_TICKS(CAN_RX_TIMEOUT_MS));
    uint32_t now = millis();

    if (now - lastBusCheckMs >= BUS_CHECK_PERIOD_MS) {
      lastBusCheckMs = now;
      checkAndRecoverCANBus();
    }

    // After a link outage the sender may have rebooted: resynchronise alive counters
    // so the first frames afterwards are not reported as "lost".
    if (now - lastValidMs > COMM_TIMEOUT_MS) {
      for (int i = 0; i < NUM_FRAME_TYPES; i++) haveAlive[i] = false;
    }

    if (rxResult != ESP_OK) continue;   // No frame: link supervision happens in the actuator task

    // --- E2E Protection, step 1: format (11-bit data frame, DLC = 5) ---
    if (rx_msg.extd || rx_msg.rtr || rx_msg.data_length_code != 5) continue;

    // --- E2E Protection, step 2: CRC8 over ID LSB + payload bytes 0..3 ---
    uint8_t receivedCRC = rx_msg.data[4];
    uint8_t calculatedCRC = calculateCRC8(rx_msg.identifier, rx_msg.data, 4);
    if (receivedCRC != calculatedCRC) {
      portENTER_CRITICAL(&g_stateMutex);
      g_currentState.crcErrors++;
      portEXIT_CRITICAL(&g_stateMutex);
      continue;                         // Frame dropped (counted, logged periodically)
    }

    int type = frameTypeFromId(rx_msg.identifier);
    if (type < 0) continue;             // Unknown ID: ignore

    // --- E2E Protection, step 3: alive counter (one counter per CAN ID) ---
    uint8_t  alive = rx_msg.data[3];
    uint32_t lost = 0;
    if (haveAlive[type]) {
      uint8_t delta = (uint8_t)(alive - lastAlive[type]);   // wraps at 256
      if (delta == 0) {
        // Same counter twice: repeated / stale frame -> reject, do not refresh link timer
        portENTER_CRITICAL(&g_stateMutex);
        g_currentState.duplicateFrames++;
        portEXIT_CRITICAL(&g_stateMutex);
        continue;
      }
      lost = (uint32_t)(delta - 1);     // delta > 1 means frames were dropped in between
    }
    lastAlive[type] = alive;
    haveAlive[type] = true;

    // --- Frame accepted: update shared state atomically ---
    portENTER_CRITICAL(&g_stateMutex);
    uint32_t stamp = millis();
    g_currentState.everRx = true;
    g_currentState.lastFrameMs = stamp;
    g_currentState.lostFrames += lost;

    switch (type) {
      case FRAME_AEB:        // 1. Highest priority frame: AEB hazard
        g_currentState.aebActive = (rx_msg.data[0] == 1);
        g_currentState.obstacleDist = rx_msg.data[1];
        g_currentState.lastAebTimestamp = stamp;
        break;
      case FRAME_CONTROLS:   // 2. Standard HMI controls
        g_currentState.steerAngle = (rx_msg.data[0] > 180) ? 180 : rx_msg.data[0];
        g_currentState.wiperActive = rx_msg.data[1] ? 1 : 0;
        g_currentState.hornActive = rx_msg.data[2] ? 1 : 0;
        break;
      case FRAME_ENV:        // 3. Environment data
        g_currentState.obstacleDist = ((uint16_t)rx_msg.data[0] << 8) | rx_msg.data[1];
        g_currentState.doorOpen = rx_msg.data[2] ? 1 : 0;
        break;
    }
    portEXIT_CRITICAL(&g_stateMutex);

    lastValidMs = stamp;
  }
}

// --- Task: Actuator Logic & Safe State Execution ---
void TaskActuatorControl(void *pvParameters) {
  bool sweepDirection = true;
  int wiperAngle = 0;
  uint8_t lastDoorState = 255;
  bool prevAebActive = false;
  bool prevCommsOk = false;
  uint32_t lastLogMs = 0;

  for (;;) {
    // Timeout check and snapshot in ONE critical section: a hazard frame arriving
    // meanwhile cannot be overwritten, and millis() can never be older than the
    // timestamps written by the receive task.
    SystemState localState;
    portENTER_CRITICAL(&g_stateMutex);
    if (g_currentState.aebActive && (millis() - g_currentState.lastAebTimestamp > AEB_TIMEOUT_MS)) {
      g_currentState.aebActive = false;   // Auto-recover: no hazard frame for AEB_TIMEOUT_MS
    }
    localState = g_currentState;
    portEXIT_CRITICAL(&g_stateMutex);

    uint32_t now = millis();
    bool commsOk = localState.everRx && ((now - localState.lastFrameMs) < COMM_TIMEOUT_MS);

    // State-change logging
    if (localState.aebActive != prevAebActive) {
      Serial.println(localState.aebActive ? "[BCM] AEB hazard received. SAFE STATE active."
                                          : "[BCM] Hazard cleared. Resuming normal operation.");
      prevAebActive = localState.aebActive;
    }
    if (commsOk != prevCommsOk) {
      Serial.println(commsOk ? "[BCM] HMI link established."
                             : "[BCM] HMI link LOST (or not started). Safe outputs applied.");
      prevCommsOk = commsOk;
    }

    // Periodic diagnostics
    if (now - lastLogMs >= LOG_PERIOD_MS) {
      lastLogMs = now;
      Serial.printf("[BCM] link:%s aeb:%d dist:%ucm door:%s | crcErr:%lu lost:%lu dup:%lu busRec:%lu\n",
                    commsOk ? "OK" : "LOST", localState.aebActive, localState.obstacleDist,
                    localState.doorOpen ? "OPEN" : "CLOSED",
                    (unsigned long)localState.crcErrors, (unsigned long)localState.lostFrames,
                    (unsigned long)localState.duplicateFrames, (unsigned long)localState.busRecoveries);
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
      steeringServo.write(SAFE_STEER_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue; // Skip normal actuator execution
    }

    // 3. COMMUNICATION LOSS: stale commands must never keep actuators running.
    //    Steering returns to center; horn/wiper/proximity buzzer are not driven.
    if (!commsOk) {
      digitalWrite(PIN_BUZZER, LOW);
      steeringServo.write(SAFE_STEER_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // 4. NORMAL OPERATION: Buzzer & Horn Logic
    bool proximityAlert = (localState.obstacleDist > 0 && localState.obstacleDist < PROXIMITY_ALERT_CM);
    digitalWrite(PIN_BUZZER, (localState.hornActive == 1 || proximityAlert) ? HIGH : LOW);

    // 5. NORMAL OPERATION: Servo Control (Wiper vs Steering)
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
