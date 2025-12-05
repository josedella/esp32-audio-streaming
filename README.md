# Real-Time AI Gunshot Detection System

![License](https://img.shields.io/badge/license-MIT-blue.svg) ![Platform](https://img.shields.io/badge/platform-ESP32--S3-green) ![Edge Impulse](https://img.shields.io/badge/AI-Edge%20Impulse-orange) ![Status](https://img.shields.io/badge/status-Active-brightgreen)

**A deterministic, edge-computing security device capable of detecting ballistic audio signatures with sub-second latency using the ESP32-S3 and W5500 Ethernet.**

**🔗 Live Demo:** [Project Website](https://josedella.github.io/esp32-audio-streaming/)
**🔗 Edge Impulse Model:** [Public Model Studio](https://studio.edgeimpulse.com/studio/842824)

---

## 📖 Overview

This project addresses the critical challenge of latency in safety systems . Traditional cloud-based audio analysis introduces network delays and privacy concerns . This system solves those issues by performing **1D Convolutional Neural Network (CNN)** inference locally on an **ESP32-S3**, using a custom **W5500 Ethernet** driver for deterministic data transmission .

The system simultaneously streams high-fidelity audio (16kHz PCM) to a central server while running continuous background inference . Upon detecting a gunshot, it injects a high-priority "Magic Packet" signature into the stream to trigger immediate visual and audible alerts at the receiver.

### Key Features
* **Edge AI Inference:** Runs a quantized int8 CNN model locally (FreeRTOS separate task) to distinguish gunshots from background noise (speech, door slams, clapping) .
* **Zero-Cloud Dependency:** All processing happens on the device; only results and audio streams are transmitted .
* **Deterministic Networking:** Uses hardwired Ethernet (SPI via W5500) to avoid Wi-Fi packet loss and jitter .
* **Dual-Core Architecture:** Utilizes the ESP32-S3's dual cores to separate audio acquisition (Core 0) from heavy AI processing (Core 1) .
* **Custom Receiver App:** A C# Windows Forms application that visualizes waveforms, records data, and handles alerts.

---

## 🛠️ Hardware Architecture

The system is built on the **Producer-Consumer** model using FreeRTOS .

| Component | Model | Purpose | Connection |
| :--- | :--- | :--- | :--- |
| **MCU** | ESP32-S3 | Main processing unit (240MHz, Dual Core) | N/A |
| **Microphone** | INMP441 | High-fidelity audio capture (MEMS, 16kHz) | I2S Interface |
| **Ethernet** | W5500 | Network transmission (UDP/RTP) | SPI Interface |
| **User Interface** | LED & Button | Status indication and recording control | GPIO |

### Pinout Configuration
*Defined in `config.h`*

| Pin Name | GPIO (ESP32-S3) | Description |
| :--- | :--- | :--- |
| **ETH_MISO** | 5 | W5500 SPI MISO |
| **ETH_MOSI** | 6 | W5500 SPI MOSI |
| **ETH_SCLK** | 7 | W5500 SPI Clock |
| **ETH_CS** | 10 | W5500 Chip Select |
| **ETH_RST** | 2 | W5500 Reset |
| **I2S_BCLK** | 47 | Mic Bit Clock |
| **I2S_WS** | 21 | Mic Word Select (L/R) |
| **I2S_DATA** | 48 | Mic Data Out |

---

## 🧠 AI & DSP Pipeline

The AI model was trained using **Edge Impulse** on a custom dataset comprising three distinct classes .

### 1. Dataset Strategy
* **Gunshot:** Acoustic proxies (balloon pops, heavy book slams, ruler snaps) to simulate high-pressure release and ballistic shockwaves .
* **Background:** Ambient noise from quiet rooms, empty hallways, and busy corridors .
* **Noise/Interference:** Adversarial examples including shouting, clapping, and door slams to prevent false positives .

### 2. Signal Processing (DSP)
* **Input:** 16kHz Audio .
* **Window Size:** 1000ms .
* **Feature Extraction:** Mel-filterbank Energy (MFE) to generate spectrograms representing frequency energy over time .

### 3. Neural Network (1D ConvNet)
The model uses a 1D Convolutional architecture optimized for the ESP32-S3 vector instructions .
* **Architecture:** Two Conv1D layers (8 and 16 filters) followed by Dropout (0.25) .
* **Output:** 3 Classes (Background, Gunshot, Noise) .

---

## 💻 Software Implementation

### Firmware (ESP-IDF & FreeRTOS)
The firmware is divided into concurrent FreeRTOS tasks :
1.  **`mic_task` (High Prio):** Reads I2S DMA buffers, processes stereo-to-mono conversion, and applies gain .
2.  **`net_task` (Medium Prio):** Consumes audio queue and packages data into RTP/UDP packets via the W5500 .
3.  **`ai_task` (Low Prio - Core 1):** Buffers audio into a ring buffer. When a noise threshold is breached, it runs the Edge Impulse classifier .
    * *Alerting:* If confidence > 0.75, it injects a "Magic Key" (`0x7F01`, `0x7F02`) into the UDP stream.

### Receiver Application (C# / .NET)
A Windows Forms application acts as the base station.
* **Audio Engine:** Uses `NAudio` for low-latency playback.
* **Visuals:** Uses `OxyPlot` for real-time waveform rendering.
* **Alert System:** Monitors the incoming UDP byte stream. If the "Magic Key" sequence is detected, the screen flashes red and logs the timestamp.

---

## 🚀 Getting Started

### Prerequisites
* **Hardware:** ESP32-S3 DevKit, W5500 Module, INMP441 Mic.
* **Software:**
    * ESP-IDF v5.x
    * Visual Studio 2019/2022 (for C# App)
    * Edge Impulse CLI (optional for model retraining)

### 1. Firmware Setup
```bash
# Clone the repository
git clone [https://github.com/josedella/esp32-audio-streaming.git](https://github.com/josedella/esp32-audio-streaming.git)
cd esp32-audio-streaming


# Build and Flash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
