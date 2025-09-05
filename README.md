ESP32 Environmental Data Logger

Real-time IoT-based monitoring, automated control, and cloud backup for environmental parameters.

This project features an ESP32-based environmental data logger capable of measuring temperature, humidity, soil moisture, atmospheric pressure, and light intensity, storing data locally on an SD card, and syncing it to Google Sheets. Dual connectivity (WiFi + GSM fallback) ensures uninterrupted operation, while relays allow automated control of connected devices like pumps or grow lights. Remote monitoring and control are available through the Blynk IoT platform.

Features

Sensor Monitoring: DHT22 (Temp & Humidity), Soil Moisture Sensor, BMP180 (Pressure), BH1750 (Light)

Data Logging: Local CSV storage on microSD card and cloud sync via Google Sheets

Automated Control: Threshold-based relay operations for pumps and grow lights

Connectivity: WiFi as primary interface, GSM (SIM800L) as failover

User Interface: OLED display for real-time readings, buttons for navigation, and LED indicators

IoT Integration: Full control and monitoring via Blynk app

Backup & Fail-Safes: SD card logging backup, network fallback, uninterrupted data logging, system recovery options
