# 🤖 STM32_Robotic_Arm

An industrial-style robotic arm controller built on an STM32 ARM Cortex-M4 microcontroller. This project features a fully autonomous hardware DMA pipeline, real-time 3D Inverse Kinematics (IK), and trajectory smoothing, all managed concurrently using FreeRTOS.

## 📖 Overview

This firmware translates dual-analog joystick inputs into smooth, Cartesian (X, Y, Z) coordinate movements for a 4 Degree-of-Freedom (4-DOF) robotic arm. 

Unlike basic joint-by-joint controllers, this system uses velocity control and the **Law of Cosines** to calculate joint angles dynamically. A custom FreeRTOS architecture ensures that mathematical calculations, hardware polling, and motor interpolation run independently without blocking the CPU.

### Core Features
*   **Real-time Inverse Kinematics:** Computes base, shoulder, and elbow angles from Cartesian XYZ coordinates.
*   **Hardware DMA Pipeline:** ADC reads four analog joystick axes continuously in the background via Circular DMA.
*   **Mathematical Boundaries:** Spherical collision detection prevents the IK math from overflowing if the user attempts to drive the arm past its physical reach.
*   **Trajectory Interpolation:** Servos ramp smoothly to their target positions rather than snapping violently.
*   **RTOS Architecture:** Decoupled `Input`, `Kinematics`, and `Servo` tasks communicating via thread-safe Queues.

---

## ⚡ Hardware & Wiring

This project was built using an STM32 Nucleo G474Re board and standard SG90/MG996R servos.

### Analog Inputs (Joysticks)
The joysticks are connected to `ADC1` configured in Scan Mode with Continuous Conversion enabled.

| Axis | Function | STM32 Pin | ADC Channel |
| :--- | :--- | :--- | :--- |
| **Joy 1 X** | Forward / Backward (X) | `PA0` | ADC1_CH1 |
| **Joy 1 Y** | Left / Right Base (Y) | `PA1` | ADC1_CH2 |
| **Joy 2 Y** | Up / Down Height (Z) | `PC1` | ADC1_CH7 |
| **Joy 2 X** | Claw Open / Close (C) | `PC0` | ADC1_CH6 |

### PWM Outputs (Servos)
The servo timers are configured to output a 50Hz (20ms) signal, with pulse widths ranging from 500µs to 2500µs to achieve a full 180° range of motion.

| Joint | Function | STM32 Pin | Timer & Channel |
| :--- | :--- | :--- | :--- |
| **Base** | Rotation | `PB8` (D15) | TIM4_CH3 |
| **Shoulder** | Elevation | `PA6` (D12) | TIM3_CH1 |
| **Elbow** | Extension | `PC7` (D9) | TIM8_CH2 |
| **Claw** | Grip | `PA7` (D11) | TIM3_CH2 |

---

## 🧠 Software Architecture

The FreeRTOS scheduler manages three primary tasks that form a continuous data pipeline:

1.  **`Input_Task` (50Hz):** 
    *   Reads the latest raw voltages from the DMA `adc_buffer`.
    *   Applies deadbands and velocity control to update the target XYZ coordinate.
    *   Validates the coordinate against the `max_distance` (185mm) boundary sphere.
    *   Dispatches the valid `TargetPosition` struct to `TargetPositionQueue`.
2.  **`Kinematics_Task` (Event-Driven):** 
    *   Wakes up immediately when new XYZ data arrives.
    *   Calculates the Base angle via `atan2f`.
    *   Solves the Shoulder and Elbow angles using 2D geometric projection and the Law of Cosines.
    *   Dispatches the resulting `TargetAngles` struct to `TargetAnglesQueue`.
3.  **`Servo_Task` (50Hz):** 
    *   Maps floating-point degrees (0-180) to hardware timer PWM ticks (500-2500).
    *   Steps the current PWM value toward the target PWM value by a fixed maximum increment (10µs per 20ms) to ensure smooth mechanical motion.
    *   Writes directly to the timer `CCR` registers.

---

## 🚀 Getting Started

1.  **PlatformIO Setup:** This is a PlatformIO project so you need to install the extension on VSCode. Once you git clone the project, open the extension, click on open project, and then open the NucleoSTM32_G474RE_FreeRTOS_Template directory. Then build and flash onto the board.
2.  **CubeMX Configuration:** 
    *   Ensure your High-Speed Clock (HSE/HSI) is configured properly so your timers output exactly 50Hz.
    *   Set ADC1 to `Scan Conversion Mode`, `Continuous Conversion Mode`, and enable `DMA Continuous Requests`.
    *   Allocate at least `3000` bytes of FreeRTOS heap to support the queues and task stacks.
3.  **Flashing:** Compile and flash using STM32CubeIDE or your preferred ARM GCC toolchain.
4.  **Debugging:** A UART interface is initialized at 115200 baud to output real-time joystick ADC values for hardware calibration.
