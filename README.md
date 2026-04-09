# ECU LPG DPI400 Reverse Engineering

## Disclaimer
Most of the content in this repository was created with heavy use of LLMs and may contain mistakes.

## Overview

This repository contains research and experimental work related to reverse engineering the **.osc logger file format** used by the diagnostic software for **DPI400 series LPG ECUs**, as well as partial reverse engineering of the **ECU communication protocol**.

The goal of this project is to better understand how the official diagnostic tools operate and to enable development of custom logging and analysis tools for LPG ECU systems.

---

## Contents

### 🔍 OSC File Format Reverse Engineering
- Analysis of `.osc` log file structure
- Experimental parsers and decoding attempts
- Observations about data encoding and layout

### 📡 ECU Communication Protocol
- Reverse engineering of LPG ECU communication (CAN / OBD2-based)
- Identification of messages and parameters
- Mapping of ECU runtime data

### 🧪 Example Logger (OBD2 + LPG)
- Sample application for OBD2 (CAN) data logging
- Parallel logging of LPG ECU parameters
- Higher sampling rate compared to official diagnostic software
- Basic synchronization of multi-source data streams

---

## ⚠️ Disclaimer

This project is experimental and research-oriented.

- The `.osc` format is not fully documented
- ECU communication protocol is partially reverse engineered
- Some interpretations may be incomplete or inaccurate

Use at your own risk.

---

## 🎯 Purpose

- Understand DPI400 LPG ECU internals
- Enable independent logging and diagnostics
- Improve data acquisition frequency and flexibility compared to official tools

---