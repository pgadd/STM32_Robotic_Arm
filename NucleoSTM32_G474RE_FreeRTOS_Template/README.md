# STM32 Bare-Metal FreeRTOS (PlatformIO Template)

A clean, production-ready boilerplate for developing FreeRTOS applications on STM32 microcontrollers using **VS Code + PlatformIO**, while retaining the ability to use **STM32CubeMX** as a graphical hardware configurator.

---

## 🏛️ The "DMZ" Architecture (Crucial Read)

The biggest trap in STM32 development is **CubeMX Code Overwrites**. When you use STM32CubeMX to turn on a new sensor pin and hit *Generate Code*, ST ruthlessly deletes any un-flagged code written inside `main.c`.

To solve this, this repository uses a **Demilitarized Zone (DMZ)**:

```text
src/
├── [ST QUARANTINE - DO NOT EDIT]
│   ├── app_freertos.c
│   ├── main.c                  <-- Auto-generated Hardware Janitor
│   ├── stm32g4xx_hal_msp.c
│   └── stm32g4xx_it.c
│
└── app/                        <-- YOUR KINGDOM (Write code here)
    ├── main_app.c              <-- Your true main()
    └── main_app.h
```


## 🚀 Quick Start

1. Click the green **Use this template** button at the top right of this repository → **Create a new repository**.
2. Clone your newly generated repository to your local machine:
   ```bash
   git clone [https://github.com/YOUR-USERNAME/YOUR-REPO-NAME.git](https://github.com/YOUR-USERNAME/YOUR-REPO-NAME.git)
   ```
3. Open the cloned directory in **VS Code**.
4. Open `src/app/main_app.c` and write your application code inside `App_Main()`.
5. Connect your STM32 via USB and click the **PlatformIO: Upload** arrow in the bottom-left status bar.

---

## 🔌 Adding Peripherals (SPI, I2C, UART, ADC)

When your firmware evolves and you need to talk to external hardware, **do not configure the registers manually.** Use STM32CubeMX as a code generator:

1. Open standalone **STM32CubeMX** and load your specific MCU.
2. Enable your desired peripheral (e.g., set `I2C1` to *I2C*).
3. Navigate to the **Project Manager** tab:
   * Set **Toolchain / IDE** to `Makefile`.
   * Under the **Code Generator** tab, check: *“Copy only the necessary library files”*.
4. Click **Generate Code** into a temporary dummy folder on your desktop.
5. Open that temporary folder and copy **only these 3 files**:
   * `Core/Src/main.c`
   * `Core/Src/stm32g4xx_hal_msp.c`
   * `Core/Src/stm32g4xx_it.c`
6. **Paste them into this repository's `src/` directory**, overwriting the old ones.
7. Open your newly pasted `src/main.c` and re-inject the DMZ bridge *(CubeMX wipes this upon regeneration)*:

```c
/* USER CODE BEGIN Includes */
#include "app/main_app.h"
/* USER CODE END Includes */

/* ... inside main() ... */

/* USER CODE BEGIN 2 */
App_Main();
/* USER CODE END 2 */
```

### Accessing newly added hardware in your code

ST instantiates your peripheral handles globally inside `main.c` (e.g., `I2C_HandleTypeDef hi2c1;`). 

To use that peripheral inside your clean workspace (`src/app/my_sensor.c`), reach across the DMZ using an `extern` declaration:

```c
#include "stm32g4xx_hal.h"

/* Tell the linker: ST created this instance over in main.c */
extern I2C_HandleTypeDef hi2c1;

void ReadSensorTask(void *params) {
    uint8_t rx_buffer[2];
    for (;;) {
        // Read 2 bytes from an I2C sensor at address 0x42
        HAL_I2C_Master_Receive(&hi2c1, (0x42 << 1), rx_buffer, 2, HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## ⚠️ The 3 Golden Rules of STM32 FreeRTOS

* **The Stack Size Trap:** In FreeRTOS `xTaskCreate()`, the stack depth argument is measured in **Words (4 Bytes)**, not raw bytes. Passing `256` allocates `1024 bytes` of RAM to that task.
* **Never call `HAL_Delay()` inside a Task:** `HAL_Delay()` relies on a hardware polling loop and freezes the entire CPU. Always use `vTaskDelay(pdMS_TO_TICKS(ms))` so the RTOS kernel knows it can put the current task to sleep and yield the CPU to another thread.
* **The `printf()` Black Hole:** Standard C string formatting consumes massive amounts of stack memory. Calling `sprintf()` on a `float` inside a standard thread will instantly blow past a 128-word stack limit and trigger a Hardware Fault. Give heavy calculation tasks a stack of at least `512`.

---

## 🔄 Porting to another STM32 Microcontroller

This template ships pre-configured for the **STM32G474RE** (Cortex-M4F @ 170MHz). To adapt it for an STM32F4, F1, L4, or H7:

1. **`platformio.ini`**: Change `board = nucleo_g474re` to your target PlatformIO board identifier.
2. **`platformio.ini`**: Update the `-mfpu=` build flag to match your silicon's exact Floating Point Unit (or delete it and change `softfp` to `soft` if the target MCU lacks a hardware FPU).
3. **`platformio.ini` & `lib/FreeRTOS/library.json`**: Update the architecture include paths pointing to `portable/GCC/ARM_CM4F` to match your chip's exact Cortex core (e.g., `ARM_CM3` for an STM32F103, or `ARM_CM7` for an STM32H7).
