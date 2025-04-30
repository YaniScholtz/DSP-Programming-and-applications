# STM32 FFT Spectrum Analyzer

This repository contains an STM32 practical project developed using **STM32CubeIDE**.

The project performs a **Fast Fourier Transform (FFT)** on ADC signals sampled by triple-interleaved ADCs and displays the resulting **frequency spectrum** on the onboard LCD.

## Project Description

- ADC data is collected using **three ADCs** in triple interleaved mode.
- The ADC samples are processed using a **recursive FFT algorithm**.
- The frequency spectrum is mapped and displayed graphically on the **STM32 Discovery Board's LCD**.
- Various utility functions like `DrawAxis()` and `DrawFFTSpectrum()` assist in visualizing the signal.

## Main Features

- Triple ADC DMA capture
- Floating-point FFT implementation
- Real-time frequency spectrum visualization
- LCD rendering using STM32 BSP libraries

## Requirements

- STM32F429I Discovery Board
- STM32CubeIDE
- STM32 HAL drivers

## How to Run

1. Open the project in **STM32CubeIDE**.
2. Flash the MCU with the code.
3. Observe the live frequency spectrum of the input ADC signal on the LCD.

## File Structure

- `Core/Src/main.c` — Main application code (ADC sampling, FFT, LCD drawing).
- `Core/Inc/main.h` — (Optional) Header file for application prototypes.



