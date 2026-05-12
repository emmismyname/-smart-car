# Smart Car Project (Electromagnetic)

A high-speed intelligent vehicle based on the **STC32G** microcontroller, designed for electromagnetic field tracking competitions.

## 🚗 Overview
This project features an autonomous car that detects alternating magnetic fields from a track. It utilizes PID control for steering and speed management.

* **MCU**: STC32G12K128 (8051 Architecture)
* **Tools**: Altium Designer (Hardware), Keil uVision5 (Software)
* **Key Features**: Inductive sensing, PID control, Brushless ESC support.

---

## 🛠️ Hardware Design
The hardware files are located in the `/hardware` directory.

### 1. Driver Board
Handles signal processing from induction coils and controls the motors.
![Driver Board 3D](hardware/Images/Driver_Board_V2.png)

### 2. Power Module
Regulates 2S/3S LiPo battery input to stable 5V and 3.3V rails.
![Power Module 3D](hardware/Images/Power_Module.png)

---

## 📂 Repository Structure
* `hardware/`: Altium Designer schematics and PCB layouts.
* `USER/`: Core application logic and interrupt handlers.
* `Libraries/`: Seekfree STC32 open-source library and drivers.
* `Docs/`: Pinout maps and sensor calibration data.

---

## 🚀 Getting Started
1.  **Hardware**: Open `.PcbDoc` files in Altium Designer to view layouts or generate Gerber files for manufacturing.
2.  **Software**:
    * Open the project in **Keil5**.
    * Compile and flash the firmware via STC-ISP.
    * Monitor sensor values using the serial debugger.

---

## 📈 Project Status
- [x] Power module design & testing
- [x] Driver board PCB layout
- [ ] PID algorithm optimization for high-speed cornering
- [ ] Integration of Brushless ESC
