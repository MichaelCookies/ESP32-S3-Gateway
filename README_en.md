# ESP32-S3 IoT Home Gateway - v4.2

[中文](README.md) | [English](README_en.md)

This project is a highly available IoT gateway system developed based on the ESP-IDF v5.5 framework. The system is designed with an event-driven architecture and integrates Huawei Cloud IoTDA access, local LAN bidirectional control, offline breakpoint resumption, and hardware fault self-healing mechanisms. It aims to provide a reference design that can operate stably even in weak network environments and under hardware interference.
This project is also one of the projects included in my portfolio when applying for jobs.

## Core Features

### 1. Robust System Architecture
* **Event-Driven Architecture**: The system internally builds a unified message bus through FreeRTOS queues, achieving complete decoupling between business logic controllers and peripheral drivers.
* **Dual-core Concurrent Scheduling**: Fully leveraging the dual-core feature of ESP32-S3, network communication stacks and sensor sampling tasks are separated to ensure real-time performance.
* **Lifecycle Management**: Strictly links the MQTT client with network status, eliminating resource leaks and DNS resolution deadlocks in the event of network disconnection.

### 2. Industrial-grade Reliability Design
* **I2C Bus Self-healing**: To address the issue of I2C slave deadlock, a 9-pulse recovery sequence and manual STOP signal mechanism have been implemented, and physical layer hard reset is supported.
* **Hardware Fault Isolation**: The OLED display and AHT20 sensor are connected to independent I2C hardware controllers (Port 0/1), ensuring that a single point failure does not affect the main system process.
* **Smart Watchdog**: Integrated task-level hardware watchdog, covering all critical business threads, to prevent deadlock.
* **Exponential Backoff for Network Reconnection**: WiFi reconnection uses an exponential backoff algorithm to prevent resource exhaustion during network oscillations.

### 3. Data Integrity Assurance
* **Offline Resume Transmission**: Based on the SPIFFS file system, local data persistence is achieved. Telemetry data is automatically cached during network interruptions, and traffic shaping synchronization is automatically executed when the network is restored.
* **Full State Synchronization**: Supports bidirectional synchronization of device shadows (Device Shadow) to ensure real-time consistency of configurations, thresholds, and switch states between the cloud and the local device.

### 4. Multi-dimensional Interaction Mode
* **Cloud Control**: Supports property reporting, command issuance, and synchronous response of Huawei Cloud IoTDA.
* **Local Area Network Control**: Equipped with an internal UDP service, it supports device discovery, real-time data broadcasting, and local command control, enabling emergency management in environments without public networks.
* **Seamless UI Refresh**: Based on the local refresh algorithm of OLED, it eliminates screen flickering during data jumps.

## Engineering Structures

The project adopts a component-based design, with the core logic located in the `components` directory.

```text
├── components/
│   ├── aht20/           # AHT20 temperature and humidity sensor driver
│   ├── ssd1306/         # OLED display driver (I2C)
│   ├── common/          # Common data types and message definitions
│   ├── logs/           # Offline log storage module (Data Logger)
│   ├── mqtt_huawei/    # Huawei Cloud MQTT protocol stack and device model encapsulation
│   ├── state_machine/  # Core state management component
│   │   ├── settings.c  # NVS configuration persistence and global mutex lock
│   │   └── hw_recovery.c # Hardware fault detection and physical layer recovery logic
│   └── wifi/           # Network communication component
│       ├── wifi_station.c # WiFi connection and reconnection strategy management
│       └── lan_service.c # UDP local area network broadcast and command service
├── main/
│   ├── main.c          # Business logic controller (Controller)
│   └── Kconfig.projbuild # Project-level KConfig configuration
├── partitions.csv       # Custom partition table (including storage partitions)
└── CMakeLists.txt      # Build script
```

## Environment Configuration and Compilation

This project relies on ESP-IDF v5.2 or a later version.

1. **Configuration Project**

Run the configuration menu to set up WiFi credentials, MQTT triplet, and hardware pins:

```bash
idf.py menuconfig
```

* **WiFi Configuration**: Set SSID and password.
* **Huawei Cloud IoT**: Set Broker address, Device ID, Client ID and Password.
* **AHT20/SSD1306**: Configure corresponding SDA/SCL pins (it is recommended to use different ports).
* **Local Network**: Set the target address for UDP broadcast.

2. **Compilation and Burning**

```bash
idf.py build flash monitor
```

## Hardware Connection

Suggested connection scheme (can be adjusted according to KConfig):

| Module | Signal | ESP32-S3 Pin | Description |
| :--- | :--- | :--- | :--- |
| **SSD1306 OLED** | SDA | GPIO 8 | I2C Port 0 |
| | SCL | GPIO 9 | I2C Port 0 |
| **AHT20 Sensor** | SDA | GPIO 10 | I2C Port 1 |
| | SCL | GPIO 11 | I2C Port 1 |

## Upper-level Computer Accessories

This project provides an upper computer console `ESP_UCP` developed based on **Qt 6 / QML**, which supports:

* Automatic discovery of gateway devices within the local area network.
* Real-time display of temperature and humidity curves and device status.
* Local issuance of control commands (alarms, threshold settings, network reset).
* Reception and display of remote debugging logs.

(A new folder has been created.)

## Version History

* **v4.2**: Integrated hardware self-healing module, optimized I2C driver timeout mechanism, and achieved full-link hot-swapping support.
* **v4.0**: Added local area network UDP service and offline data storage function.
* **v3.0**: Reconstructed into an event bus architecture and introduced NVS configuration management.
* **v2.0**: Implemented basic WiFi/MQTT communication and multi-task scheduling.

---

**License Type**: MIT  
**Author**: Michael Cookies  

## Third-party Components

1. **SSD1306 Display Driver**
   - **Source**: https://github.com/nopnop2002/esp-idf-ssd1306
   - **License**: MIT License
   - **Usage**: Copied the `component/ssd1306` directory to this project
   - **Modifications**: Optimized the I2C initialization logic
   - **Copyright**: Copyright (c) 2022 nopnop2002