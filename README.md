1. Abstract

This project presents an ESP32-based environmental data logger capable of real-time monitoring, automated control, and IoT-enabled remote access. 
It measures temperature, humidity, soil moisture, pressure, and light intensity, storing data locally and on the cloud. The system integrates with Google Sheets and the Blynk platform for visualization and remote control. 
Dual connectivity modes (WiFi + GSM) ensure continuous operation. Designed with scalability and reliability in mind, this device is suited for agriculture, research, and smart environment applications.

2. Introduction

This report details the design and development of a real-time monitoring and control ESP32-based environmental data logger. 
It is suitable for agriculture, horticulture, and environmental studies, with automated control, robust local and cloud data logging, and full remote access through IoT integration.

Objectives

Measure temperature, humidity, soil moisture, atmospheric pressure, and light intensity

Record environmental data locally on a microSD card

Transfer sensor data to Google Sheets over the internet

Control connected devices automatically (e.g., pump, grow light)

Allow remote monitoring and control through the Blynk IoT platform

Offer network redundancy via WiFi and GSM

3. System Overview

The data logger uses the ESP32 microcontroller to acquire sensor readings at set intervals. Data is:

Displayed locally

Stored on an SD card

Uploaded to the cloud via WiFi or GSM (SIM800L)

A real-time clock (RTC) timestamps each reading. Both manual and automatic actuator control is supported via relays. 
Thresholds and modes are configurable through the Blynk app. The hardware is enclosed in a 3D-printed case for durability and semi-outdoor protection.

4. Project and System Details
4.1 Sensor Integration

DHT22: Temperature & Humidity

Soil Moisture Sensor: Analog measurement

BMP180: Atmospheric Pressure

BH1750: Light Intensity

4.2 Data Logging

CSV format on microSD card

Cloud logging to Google Sheets via HTTPS

Logging intervals adjustable via Blynk app

4.3 Automated Control

Automatic pump operation when soil moisture falls below threshold

Grow light automation based on light intensity

Thresholds and automation configurable remotely

Manual override available for relays

4.4 Connectivity

WiFi as primary network interface

GSM (SIM800L) as fallback

WiFiManager for dynamic credential configuration

4.5 User Interface

OLED display cycles through system status and sensor readings

Two buttons for navigation and system reset

LEDs indicate operation and connectivity status

4.6 Software Architecture

Arduino-based, modular, and maintainable

Uses FreeRTOS for parallel task execution

Functional Modules

Sensor reading & verification

OLED display refresh & navigation

SD card logging

WiFi and GSM communication

Blynk IoT interface

Relay control logic

Buck converter

RTC for timestamping

Cloud Integration

Google Sheets receives data via HTTP POST through Apps Script

Data includes timestamp, temperature, humidity, soil moisture, light intensity, and pressure

Configurable intervals with retry logic

4.7 Blynk IoT Platform Features

Real-time data display

Set moisture and light thresholds

Enable/disable automation

Manual relay control

Adjust logging intervals

Force data uploads

5. Backup and Fail-Safes

SD Card Backup: Temporary logging if primary file fails

Network Fallback: GSM used if WiFi fails

Logging Continuity: Local logs maintained even if cloud unavailable

System Recovery: Buttons allow reset of WiFi or system

6. Operational Workflow

Startup: Initializes sensors, SD card, display, connectivity, and RTC sync
Loop Operation:

Read sensors at intervals

Update OLED display

Apply automation logic to relays

Log data to SD card

Send data to Google Sheets via WiFi/GSM

Update Blynk values

Manual Functions: Relay control, threshold changes, forced logging, and reset via buttons

7. Applications

Precision Agriculture

Greenhouse Automation

Weather Stations

Remote Environmental Monitoring

Educational & Research Projects

8. Test Results

Pump & light automation triggers correctly

GSM fallback verified when WiFi disabled

Cloud logging works reliably with expected intervals

SD card logging maintained during internet loss

Blynk app provides real-time access to all data

9. Conclusion

The ESP32-based environmental data logger is robust, scalable, and reliable. With dual connectivity, strong local and cloud logging, and IoT integration, it ensures continuous operation. 
Its modular design and remote configurability make it ideal for agriculture, research, and smart environment applications.
