# Distributed-Auto-CAN-Safety
C++ / FreeRTOS demonstrator for a distributed automotive CAN network (HMI &amp; BCM) featuring AEB, hardware arbitration, and E2E Functional Safety.
# Distributed Automotive Control System (HMI & BCM) via CAN Bus

This project is a hardware and software demonstrator for a distributed automotive network based on the CAN 2.0B protocol. The system emulates the interaction between a Human-Machine Interface (HMI) module and a Body Control Module (BCM), emphasizing modern **Functional Safety** concepts conceptually aligned with the ISO 26262 standard.

The firmware is developed in C++ (Arduino Core for ESP32) and runs on **FreeRTOS**, ensuring deterministic execution and preemptive multitasking of network communication, sensor polling, and actuator control tasks.

## ⚙️ System Architecture

The system consists of two distinct Electronic Control Units (ECUs), implemented on ESP32 microcontrollers and interconnected via CAN transceivers (SN65HVD230) operating at **500 kbps**:

<img width="2040" height="1530" alt="WhatsApp Image 2026-10-29 at 13 30 20" src="https://github.com/user-attachments/assets/a95281a8-664b-4d30-855e-b47bdd679c00" />

### 1. HMI Node (Transmitter / Driver Interface)
This node processes physical inputs and environmental sensors, broadcasting the data on the CAN bus:
* **Steering Control:** Potentiometer (ADC with an exponential moving average filter) for the steering angle.
* **Wiper Control:** Micro-switch (active LOW, with software debounce).
* **Horn Control:** TTP223 capacitive sensor (active HIGH).
* **Door Monitoring:** A3144 Hall Effect sensor for door status detection (open/closed).
* **AEB Sensor:** HC-SR04 ultrasonic sensor for frontal distance measurement and Autonomous Emergency Braking triggers.

### 2. BCM Node (Receiver / Actuator Management)
This node validates received packets and controls the vehicle's hardware:
* **Dual-Mode Actuator:** A servo motor that controls either the steering (based on the received angle) or executes a sweeping motion (when the wipers are activated).
* **Acoustic Warning:** Active buzzer triggered either manually (horn) or automatically (obstacle under 50 cm or hazard state).
* **Dome Light (Safety Interlock):** LED automatically activated when the door is opened.

## 🛡️ Functional Safety

The project goes beyond standard remote control by integrating critical safety mechanisms:

* **End-to-End (E2E) Protection:** To prevent the execution of electrically corrupted commands, each CAN frame (DLC = 5) includes an *Alive Counter* (to detect dropped or delayed packets) and a *CRC8* (Cyclic Redundancy Check) calculated over the Payload and ID. The BCM node automatically drops any invalid frame.
* **Hardware Arbitration (CSMA/CR):** The system relies on CAN IDs to prioritize traffic. Hazard messages (AEB) are assigned ID `0x010` (absolute maximum priority), while standard telemetry receives `0x120` and `0x121`.
* **Autonomous Emergency Braking (AEB) & Safe State:** If the ultrasonic sensor detects an obstacle under 15 cm, the HMI node intentionally suppresses the transmission of steering commands and prioritizes the critical alert frame. Upon reception, the BCM node enters a *Safe State*: it ignores user inputs, locks the steering to the center position (90 degrees), and activates the acoustic alarm until the hazard clears (features an integrated Auto-Recovery timeout).

## 🛠️ Debugging & Fault Injection Mode

To facilitate edge-case testing (Hardware-in-the-Loop simulation), the HMI node supports interactive commands via the Serial Monitor (115200 baud) that override physical sensor readings:
* `D<value>` (e.g., `D14`): Manually sets the distance to 14 cm, forcing the system into the AEB hazard state.
* `R`: Restores control to the physical ultrasonic sensor (Resets manual override).
* `U`: Simulates opening or closing the door via software (Toggle).

## 🔌 Hardware Pinout (ESP32)

### 1. HMI Node (Transmitter)
| Component | ESP32 Pin | Additional Connection Notes |
| :--- | :--- | :--- |
| **Steering Potentiometer** | `GPIO 4` | Connect outer pins to 3.3V and GND. |
| **Horn Sensor (TTP223)** | `GPIO 5` | Active HIGH. VCC to 3.3V. |
| **Wiper Switch** | `GPIO 21` | Active LOW (uses internal PULLUP). Connect to GND when pressed. |
| **Door Sensor (A3144 Hall)** | `GPIO 15` | Active LOW (uses internal PULLUP). Connect to GND via magnet. |
| **HC-SR04 Ultrasonic (TRIG)** | `GPIO 19` | Connect VCC to 5V (VIN). |
| **HC-SR04 Ultrasonic (ECHO)** | `GPIO 18` | ⚠️ **Requires Voltage Divider!** 5V to 3.3V (R1=1kΩ to Echo, R2=2kΩ to GND). |
| **CAN Transceiver (CTX)** | `GPIO 22` | SN65HVD230 (3.3V logic). |
| **CAN Transceiver (CRX)** | `GPIO 23` | SN65HVD230 (3.3V logic). |

### 2. BCM Node (Receiver)
| Component | ESP32 Pin | Additional Connection Notes |
| :--- | :--- | :--- |
| **Steering/Wiper Servo** | `GPIO 19` | Power via 5V (VIN) or external BEC depending on current draw. |
| **Active Buzzer** | `GPIO 21` | Connect positive pin to GPIO, negative to GND. |
| **Dome Light (LED)** | `GPIO 5` | Connect in series with a current-limiting resistor (e.g., 220Ω). |
| **CAN Transceiver (CTX)** | `GPIO 22` | SN65HVD230 (3.3V logic). |
| **CAN Transceiver (CRX)** | `GPIO 23` | SN65HVD230 (3.3V logic). |

### ⚠️ Important Wiring Notes:
1. **CAN Bus Backbone:** Connect `CAN_H` to `CAN_H` and `CAN_L` to `CAN_L` between the two transceivers.
2. **Termination Resistors:** Ensure the CAN bus is terminated with a 120Ω resistor at both physical ends of the network.
3. **Common Ground:** If the ECUs are powered from different sources, ensure a common GND wire is shared between the modules to prevent ground loop offsets.

## 🚀 Roadmap & Future Work

As the project evolves from a proof-of-concept prototype to a robust, industry-aligned functional safety demonstrator, the following milestones are planned:

### 1. Hardware Transition (Custom PCB Design)
* **KiCad Schematic & Routing:** Migrate the current breadboard prototype to a professional schematic using KiCad.
* **Automotive-Grade PCB:** Design and manufacture custom PCBs featuring proper ground planes, EMI/RFI filtering, and ESD protection for the CAN bus.
* **Power Management:** Integrate robust Low-Dropout Regulators (LDOs) and galvanic isolation to mimic real-world ECU power distribution.

### 2. Validation & Bus Analysis
* **Packet Sniffing & Diagnostics:** Utilize a dedicated USB-CAN analyzer to perform real-time packet sniffing, bus load monitoring, and hardware arbitration validation.
* **Latency Profiling:** Measure the precise End-to-End (E2E) latency from the moment the ultrasonic sensor detects an obstacle to the physical locking of the actuator (FTTI - Fault Tolerant Time Interval).

### 3. Safety & Architecture Enhancements
* **FMEA Documentation:** Complete a formal Failure Mode and Effects Analysis (FMEA) to document system responses to node losses, sensor disconnects, and bus-off conditions.
* **AUTOSAR Alignment:** Standardize the current CRC8 and Alive Counter implementation to fully comply with AUTOSAR E2E Profile 1 specifications.
* **Advanced Fault Injection:** Expand the Serial Monitor debugging tool into an automated Hardware-in-the-Loop (HIL) test script for stress-testing the AEB logic.****
