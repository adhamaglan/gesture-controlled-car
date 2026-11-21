# Gesture-Controlled Car

A wireless, hand-gesture-controlled differential drive car powered by dual **ATmega32** microcontrollers and **HC-05 Bluetooth** modules. A sensor glove reads finger flex angles via buffered ADC channels, encodes posture and throttle into a 3-byte UART packet, and transmits it over Bluetooth to the rover—which drives dual DC motors through an **L293D** H-bridge and displays live status on a **16×2 Character LCD**.

---

## System Overview

* **Hand Module (Transmitter):** Reads 5× flex sensors buffered through **LM358** op-amp voltage followers (`PA0`–`PA4`) and transmits encoded gesture commands via **HC-05** Bluetooth at `~40 Hz`. Powered by a `3.7V` Li-ion battery with a **TP4056** charger and **MT3608** `5V` boost converter.
* **Car Module (Receiver):** Receives Bluetooth commands over UART, controls left/right DC motors using an **L293D** driver with Timer1 PWM speed control, and shows real-time direction status on a **16×2 LCD** (4-bit mode). Powered by a `7.4V` (2S) Li-ion battery pack with a **2S 5A BMS** and an **LM7805** `5V` regulator.
* **Layered Embedded C Architecture:** Firmware follows a modular **MCAL → HAL → service → Application** design pattern to separate register-level hardware drivers from application control logic.

---

## Hardware Pinout

### Hand Module (`ATmega32` @ 16 MHz)
> Full schematic: [`Schematics/hand_module.pdf`](Schematics/hand_module.pdf)

| ATmega32 Pin | Function | Connected Component | Notes |
| :--- | :--- | :--- | :--- |
| `PA0` | `ADC0` | Thumb Flex Sensor (`FINGER_0`) | Proportional PWM throttle (`0–100%`) |
| `PA1` – `PA4` | `ADC1` – `ADC4` | Index, Middle, Ring, Pinky (`FINGER_1`–`FINGER_4`) | Buffered via `LM358` unity-gain followers |
| `PD0` | `RXD` | HC-05 `TX` | UART Receive (`9600 bps`) |
| `PD1` | `TXD` | HC-05 `RX` | Through `1 kΩ / 2 kΩ` (`5V → 3.3V`) voltage divider |

### Car Module (`ATmega32` @ 16 MHz)
> Full schematic: [`Schematics/car_module.pdf`](Schematics/car_module.pdf)

| ATmega32 Pin | Function | Connected Component | Notes |
| :--- | :--- | :--- | :--- |
| `PA4` – `PA7` | GPIO Output | 16×2 LCD `DB4` – `DB7` | 4-bit parallel data bus |
| `PB1` / `PB2` | GPIO Output | 16×2 LCD `RS` / `E` | `R/W` tied to `GND`; `10 kΩ` trim pot on `V0` |
| `PB4` – `PB7` | GPIO Output | L293D `1A`, `2A`, `3A`, `4A` | Left & right motor direction control |
| `PD0` / `PD1` | `RXD` / `TXD` | HC-05 `TX` / `RX` | `1 kΩ / 2 kΩ` voltage divider on `RX` |
| `PD4` / `PD5` | `OC1B` / `OC1A` | L293D `EN1,2` / `EN3,4` | Hardware PWM motor speed control |

---

## Application Firmware (`main.c`)

### 1. Hand Module (`Source code/Hand Module (Transmitter)/Application/main.c`)
Runs a `~40 Hz` (`22 ms` cycle) sensing-and-transmission loop on the glove's **ATmega32** (`16 MHz`):

* **Power-On Flat-Hand Calibration (`~310 ms`):** Waits `300 ms` on boot for the `5V` boost rail to stabilize, then captures a **16-sample baseline average** (`Global_u16InitVal[5]`) across `PA0`–`PA4` while the hand is held flat.
* **Clean ADC Multiplexing (`APP_u16ReadAdcClean`):** Discards the first conversion after switching `ADMUX` and waits `20 µs` so the Sample & Hold capacitor settles cleanly before averaging runtime samples.
* **Proportional Thumb Throttle (`PA0`):** Calculates `Init - Current` ADC drop, ignores jitter below `30` counts (`THUMB_DEADZONE_DROP`), saturates at `135+` counts (`THUMB_MAX_DROP = 100%`), and linearly scales intermediate bend angles into `0–100%` PWM duty cycle.
* **4-Bit Finger Posture Mask (`PA1`–`PA4`):** Evaluates Index, Middle, Ring, and Pinky drops (`>= 70` counts) into a 4-bit mask `[Pinky | Ring | Middle | Index]`:

  | Hand Gesture | 4-Bit Mask (`P R M I`) | Command | Rover Action |
  | :--- | :---: | :---: | :--- |
  | Closed Fist (all 4 bent) | `0b1111` | `'F'` | Move Forward |
  | Peace Sign (Index & Middle straight) | `0b1100` | `'B'` | Move Backward |
  | Point Index (Index straight) | `0b1110` | `'R'` | Turn Right |
  | Pinky Out (Pinky straight) | `0b0111` | `'L'` | Turn Left |
  | Open Palm / Neutral | `default` | `'S'` | Stop |

* **Safety Override & 3-Byte UART Frame (`9600 bps`):** Forces `Cmd = 'S'` and `PWM = 0%` whenever the thumb is straight (`0%` throttle) or the gesture is neutral, then transmits `[0xAA (Header)] [Command] [PWM %]`.

### 2. Car Module (`Source code/Car Module (Receiver)/Application/main.c`)
Runs the rover's control loop on the car's **ATmega32** (`16 MHz`), decoding incoming Bluetooth packets:

* **Peripheral Setup:** Initializes **USART** (`9600 bps`) for HC-05 reception, **CLCD** (`16×2` LCD in 4-bit mode), and **DCMOTOR** (`L293D` direction pins `PB4`–`PB7` + **Timer1** PWM on `OC1A`/`OC1B`).
* **3-Byte Frame Synchronization (`0xAA` Header):** Monitors incoming UART bytes for the `0xAA` start-of-packet header before reading the **Command** byte (`'F'`, `'B'`, `'R'`, `'L'`, `'S'`) and **PWM Speed** (`0–100%`), preventing frame misalignment if a wireless byte is dropped.
* **Differential Drive & LCD Feedback:** Executes the validated command via the `DCMOTOR` driver (`MoveForward`, `MoveBackward`, `TurnRight`, `TurnLeft`, or `StopAll`) and updates the `16×2` LCD with the active direction and speed.

---

## 📂 Repository Structure

<details open>
<summary><b>📦 Gesture-Controlled-Car</b></summary>

* 📄 **`README.md`** — Project documentation, pinouts, firmware breakdown, and build guide.

<blockquote>

<details>
<summary>📂 <b>Datasheets/</b> (Component Reference PDFs)</summary>

* 📄 **`Atmega32.pdf`** — 8-bit AVR Microcontroller datasheet.
* 📄 **`Character LCD Datasheet.pdf`** — 16×2 alphanumeric LCD controller reference.
* 📄 **`Flex_Sensor_short.pdf`** — Resistive bend sensor electrical specifications.
* 📄 **`HC-05 and HC-06.pdf`** — Bluetooth serial module AT commands & pinout.
* 📄 **`L293D.pdf`** / **`LM358.pdf`** / **`LM7805.pdf`** — Motor driver, op-amp, and voltage regulator datasheets.
</details>

<details>
<summary>📂 <b>Media/</b> (Hardware Photos & Driving Demos)</summary>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>Pictures/</b></summary>

* Hardware assembly photos of the flex-sensor glove transmitter and differential drive car.
</details>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>Videos/</b></summary>

* Live wireless gesture-controlled driving demonstration clips.
</details>

</details>

<details>
<summary>📂 <b>Schematics/</b> (KiCad Circuit Diagrams)</summary>

* 📄 **`hand_module.pdf`** — Glove transmitter circuit schematic.
* 📄 **`car_module.pdf`** — Differential drive car receiver circuit schematic.
</details>

<details open>
<summary>📂 <b>Source code/</b> (AVR Layered C Firmware & HEX Binaries)</summary>

<details open>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>Hand Module (Transmitter)/</b></summary>

* 📄 **`Hand Module (Transmitter).hex`** — Ready-to-flash transmitter firmware binary.

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>Application/</b></summary>

* 📄 **`main.c`** — Startup calibration, thumb throttle mapping, 4-finger gesture decoding, and UART packet transmission.
</details>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>MCAL/</b> (Microcontroller Abstraction Layer)</summary>

* 📄 **`regdef.h`** — Direct volatile memory-mapped register definitions for ATmega32.
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>ADC/</b> (Analog-to-Digital Converter Driver)</summary>

* 📄 **`adc.c`** — Implementation of 10-bit ADC initialization and channel conversion.
* 📄 **`adc.h`** — ADC function declarations and channel/reference macros.
</details>
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>USART/</b> (Serial UART Driver)</summary>

* 📄 **`usart.c`** — Implementation of USART initialization and byte/string transmission.
* 📄 **`usart.h`** — USART function declarations.
* 📄 **`usart_config.h`** — Baud rate (`9600 bps`) and frame format configurations.
* 📄 **`usart_priv.h`** — USART private register and bit macros.
</details>
</details>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>service/</b> (Shared Utilities Layer)</summary>

* 📄 **`bit_math.h`** — Bitwise manipulation macros (`SET_BIT`, `CLR_BIT`, `GET_BIT`, `TOG_BIT`).
* 📄 **`std_types.h`** — Platform-independent strict-width primitive definitions (`u8`, `s16`, `u32`, `f32`).
</details>

</details>

<details open>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>Car Module (Receiver)/</b></summary>

* 📄 **`Car Module (Receiver).hex`** — Ready-to-flash receiver firmware binary.

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>Application/</b></summary>

* 📄 **`main.c`** — UART packet synchronization, motor direction/speed execution, and LCD status display.
</details>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>MCAL/</b> (Microcontroller Abstraction Layer)</summary>

* 📄 **`regdef.h`** — Direct volatile memory-mapped register definitions for ATmega32.
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>DIO/</b> (Digital Input/Output Driver)</summary>

* 📄 **`dio.c`** — Implementation of pin/port direction, value, toggle, and pull-up controls.
* 📄 **`dio.h`** — Pin/Port direction macros and function declarations.
</details>
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>GIE/</b> (Global Interrupt Enable Driver)</summary>

* 📄 **`gie.c`** — Implementation of Global Interrupt Enable/Disable functions.
* 📄 **`gie.h`** — Global Interrupt Enable function declarations.
</details>
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>TIMER1/</b> (Timer/Counter 1 & PWM Driver)</summary>

* 📄 **`timer1.c`** — Implementation of Timer1 and dual-channel hardware PWM (`OC1A`/`OC1B`).
* 📄 **`timer1.h`** — Timer1 function declarations and mode macros.
* 📄 **`timer1_cfg.h`** — Timer1 prescaler and waveform mode configurations.
* 📄 **`timer1_priv.h`** — Timer1 private register and bit definitions.
</details>
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>USART/</b> (Serial UART Driver)</summary>

* 📄 **`usart.c`** — Implementation of USART initialization, transmission, and reception.
* 📄 **`usart.h`** — USART function declarations.
* 📄 **`usart_config.h`** — Baud rate (`9600 bps`) and frame format configurations.
* 📄 **`usart_priv.h`** — USART private register and bit macros.
</details>
</details>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>HAL/</b> (Hardware Abstraction Layer)</summary>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>CLCD/</b> (Character LCD Driver)</summary>

* 📄 **`CLCD_prog.c`** — 16×2 Character LCD function implementations.
* 📄 **`CLCD_int.h`** — Character LCD function declarations.
* 📄 **`CLCD_config.h`** — 4-bit interface mode (`CLCD_4_BITS`) and GPIO pin routing layout.
* 📄 **`CLCD_priv.h`** — LCD instruction opcodes and private macros.
</details>
<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>DCMOTOR/</b> (DC Motor H-Bridge Driver)</summary>

* 📄 **`dcmotor.c`** — Differential drive motor direction and speed implementations.
* 📄 **`dcmotor.h`** — DC Motor function declarations.
* 📄 **`dcmotor_cfg.h`** — L293D direction pins and Timer1 PWM channel configurations.
* 📄 **`dcmotor_priv.h`** — DC Motor private macros.
</details>
</details>

<details>
<summary>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;📂 <b>service/</b> (Shared Utilities Layer)</summary>

* 📄 **`bit_math.h`** — Bitwise manipulation macros (`SET_BIT`, `CLR_BIT`, `GET_BIT`, `TOG_BIT`).
* 📄 **`std_types.h`** — Platform-independent strict-width primitive definitions (`u8`, `s16`, `u32`, `f32`).
</details>

</details>

</details>

</blockquote>
</details>