# Modbus Driver for ARM Cortex-M

A portable, bare-metal Modbus RTU/TCP driver for ARM Cortex-M 
microcontrollers (STM32, TI C2000, NXP).

## What it does
- Modbus RTU master/slave over UART/RS-485
- Modbus TCP over Ethernet (LwIP-compatible)
- Hardware CRC-16 and register mapping

## Why
I needed a lightweight, dependency-free Modbus stack for multi-MCU 
distributed systems (motor control, industrial instrumentation).
