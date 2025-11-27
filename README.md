# ESP32 Radar System with FreeRTOS

A real-time embedded radar system developed on the **ESP32** microcontroller utilizing the **VL53L4CX Time-of-Flight (ToF) sensor**, a servo motor for 180° sweeping, a buzzer, and an **RGB LED**, all coordinated by the **FreeRTOS** operating system.

This project implements the required multi-threaded architecture using **six** distinct tasks, one Mutex, and one Binary Semaphore to ensure concurrent operation, safe access to shared data, and timely response to interrupts.

## Features

* **Multithreaded Design:** Uses **six FreeRTOS tasks** (Startup, Button, ToF, Servo, LED, Buzzer) for true concurrency.
* **Synchronization:** Uses a **Mutex** (`distanceMutex`) to protect shared sensor data and a **Binary Semaphore** (`buttonSem`) for interrupt-driven button handling.
* **100°/s Sweep:** Servo motor sweeps from 0° to 180° at a precise speed of **100 degrees per second**.
* **Alarm System:** Triggers a fast **Red LED** blink and **Buzzer** alarm when an object is detected within **1 meter** (1000 mm). The alarm holds for a minimum of 3 seconds.
* **State Control:** Push button on Pin 37 toggles the system between **READY** (stopped) and **SWEEPING** (active).
* **Concurrent Boot Sequence:** The dedicated `StartupTask` runs the LED flash, buzzer tone, and servo sweep simultaneously upon boot.


<img width="500" height="468" alt="image" src="https://github.com/user-attachments/assets/e169e6d7-a9c4-47a0-8e26-5ffbb3b17171" />
<img width="500" height="468" alt="image" src="https://github.com/user-attachments/assets/5ee28670-bfcb-4122-8eae-bd00c2263cf9" />
<img width="500" height="468" alt="image" src="https://github.com/user-attachments/assets/6b4aadee-08c9-470d-af97-f7c34393b1e7" />

## 🛠️ Hardware & Pin Assignment

The system uses the following required and utilized pin assignments:

| Device | MCU Pin (GPIO) | Function |
| :--- | :--- | :--- |
| **Time-of-Flight (SDA)** | **41** | I²C Data |
| **Time-of-Flight (SCL)** | **42** | I²C Clock |
| **Servo Motor** | **40** | PWM Output |
| **Buzzer** | **39** | Tone Output (LEDC Channel) |
| **Push button** | **37** | Input (Falling Edge Interrupt) |
| **RGB LED - Red** | **2** | Status Indicator (Physical LED) |
| **RGB LED - Green** | **4** | Status Indicator (Physical LED) |
| **RGB LED - Blue** | **5** | Status Indicator (Physical LED) |

## ⚙️ Software Architecture (FreeRTOS Tasks)

The architecture is divided into six cooperative tasks to handle system logic, I/O, and motion.

| Task Name | Type | Priority | Role & Logic |
| :--- | :--- | :--- | :--- |
| **StartupTask** | Run-to-Completion | Medium (3) | Executes the concurrent **Boot Sequence**: LED flash (Red/Green), Buzzer tone, and 90 $\to$ 180 $\to$ 0 $\to$ 90 servo sweep. Deletes itself on completion. |
| **ButtonTask** | Event-Driven | High (2) | Waits on the `buttonSem`. **Toggles the `sweeping` flag** (START/STOP) and manages the `alarmArmed` state. Clears alarm flags when stopping. |
| **ToFTask** | Periodic (100ms) | Medium (2) | Reads range data from ToF. **Updates `distanceMM`** and sets `objectDetected` and **`alarmHoldUntilMs`** under Mutex protection. |
| **ServoTask** | Periodic (10ms) | Medium (2) | Reads `sweeping`/`alarm` state. Implements the **100°/s sweep** when active. Holds the current angle when an alarm is active. |
| **LEDTask** | Periodic (20ms) | Low (1) | Reads the system state. Implements the required LED patterns: Solid Green (Ready), Slow Green Blink (Sweeping), Fast Red Blink (Alarm). |
| **BuzzerTask**| Periodic (50ms) | Low (1) | Reads the alarm state. **Activates the alarm tone** when `alarmArmed` is true AND either `objectDetected` or the 3-second hold is active. |

### **FreeRTOS Primitives Usage**

| Primitive | Variable | Purpose (Where and Why) |
| :--- | :--- | :--- |
| **Mutex** | `distanceMutex` | **Shared Resource Protection:** Used to lock access when `ToFTask` updates the shared `distanceMM`, `objectDetected`, and `alarmHoldUntilMs` flags, preventing other tasks from reading inconsistent data mid-update. |
| **Binary Semaphore** | `buttonSem` | **Synchronization (ISR $\to$ Task):** Given in the **Push Button ISR** and taken by the `ButtonTask`. This decouples the time-sensitive interrupt handler from the main button processing logic (debouncing, state changes). |

## 📐 System States and Logic

| State | LED Pattern | Buzzer | Servo | Transitions |
| :--- | :--- | :--- | :--- | :--- |
| **BOOT** | Alt. Red/Green (Fast) | ON (Tone) | Initial Sweep (90 $\to$ 180 $\to$ 0 $\to$ 90) | Auto $\to$ **READY\_IDLE** when sweep complete. |
| **READY\_IDLE** | Solid Green | OFF | Parked (90°) | Button Press $\to$ **SWEEPING** |
| **SWEEPING** | Green Blink (Slow, 500ms) | OFF | Sweeping (100°/s) | Button Press $\to$ **READY\_IDLE** OR Distance $\le 1000$ mm $\to$ **ALARM** |
| **ALARM** | Red Blink (Fast, 100ms) | ON (Tone, 2200Hz) | HELD (Angle Frozen) | Button Press $\to$ **READY\_IDLE** OR Distance $> 1000$ mm for $\ge 3$s $\to$ **SWEEPING** |
