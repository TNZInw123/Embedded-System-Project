# RFID Access System with EEPROM Database and User Interface
Firmware and CAD Design for FRA421: Embedded System Course Project
<img width="1530" height="1152" alt="image" src="https://github.com/user-attachments/assets/91e254c1-0b4e-4e2e-a9e8-0bae24151184" />

# RFID Access System with EEPROM Database and User Interface

![System CAD Design](https://github.com/your-username/your-repo/blob/main/path-to-your-image.png?raw=true)
*(Replace the link above with the actual path to your CAD image if available, or keep your existing image upload)*

## 📌 Project Overview
**Course:** FRA421 - Embedded Systems  
[cite_start]**Author:** Pongnapat Arayangkul (66340500032) [cite: 2]  

This project is a standalone **RFID Access Control System** powered by an **STM32L010** microcontroller. It features a fully integrated database system using an **EEPROM** chip to store, add, and delete authorized user cards (UIDs) without needing a connection to a PC.

[cite_start]The system is designed with **Modular Programming** principles and utilizes a **Finite State Machine (FSM)** to manage user inputs, display logic, and hardware communication seamlessly[cite: 4, 16].

## 🛠 Features
* **Standalone Operation:** Verify User Cards against an internal database.
* **Admin Mode:** Add or Delete users directly using the device interface (LCD + Buttons).
* [cite_start]**Persistent Storage:** Uses I2C EEPROM to save users even when power is lost[cite: 10].
* [cite_start]**Real-Time Clock (RTC):** Displays current Date and Time on the LCD[cite: 15].
* **Interactive UI:**
    * **LCD 1602 (I2C):** Shows status, menus, and custom icons (♥, ✔, ✖).
    * **RGB LED & Buzzer:** Provides visual and audio feedback for access events.
* [cite_start]**Custom Firmware Drivers:** Manually implemented libraries for MFRC522 (SPI) and LCD (I2C) to demonstrate low-level understanding[cite: 7].
* [cite_start]**PCB Design:** Custom-designed PCB integrating MCU, RFID, and Power delivery[cite: 6].

## ⚙️ Hardware Specifications
| Component | Function | Communication |
| :--- | :--- | :--- |
| **STM32L010RB** | Main Microcontroller (NUCLEO-L010RB) | - |
| **MFRC522** | RFID Reader Module | **SPI** |
| **EEPROM** | User Database Storage | **I2C** |
| **LCD 1602** | User Display (w/ PCF8574) | **I2C** |
| **Buttons (x3)** | Navigation (Next, OK, Back) | GPIO (Active Low) |
| **RTC** | Internal Real-Time Clock | Internal |

## 🧩 Software Architecture
[cite_start]The firmware is built upon a **Non-Blocking Finite State Machine (FSM)** structure[cite: 8]. This ensures the system can poll for cards, update the clock, and listen for button presses simultaneously without "freezing."

### System States
1.  **IDLE_WAIT:** Polls for cards every 50ms while updating the RTC clock on the LCD.
2.  **CHECK_CARD:** Compares scanned UID against the **Master Card** (Admin) or EEPROM Database (User).
3.  **ACCESS_GRANTED/DENIED:** Triggers Relay/LED/Buzzer feedback.
4.  **ADMIN_MENU:** Allows the Master Card holder to select "Add User" or "Delete User."
5.  **ADMIN_WRITE:** Handles the logic to save/remove UIDs from EEPROM memory.

### Key Algorithmic Features
* **Debouncing:** Software-based filtering for button inputs.
* **EEPROM "Swap & Pop":** Optimized deletion algorithm that replaces a deleted user with the last user in memory to avoid data fragmentation.
* **Custom Characters:** Custom bitmaps uploaded to LCD CGRAM for UI icons.

## 📂 Repository Structure
```text
📦 RFID-Access-System
 ┣ 📂 Firmware               # Source code (STM32CubeIDE Project)
 ┃ ┣ 📂 Core/Src             # Main Logic (main.c, fsm.c, drivers)
 ┃ ┣ 📂 Core/Inc             # Headers (rc522.h, eeprom.h, lcd.h)
 ┃ ┗ ...
 ┣ 📜 RFID_System-CAD.zip    # CAD/3D Model files for the enclosure
 ┗ 📜 README.md              # Project Documentation
