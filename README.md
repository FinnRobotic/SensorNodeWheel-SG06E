# SensorNodeWheel-SG06E

## Overview
This repository contains C code for reading various sensors including:
- Hall Sensor (Wheel Speed)
- Strain Gauges
- Damper Displacement Sensor

The firmware is designed to run on the SENSOR_BOARD_WHEEL_DMA platform and leverages DMA for efficient data handling.

## Features
- Real-time sensor data acquisition
- DMA-based ADC conversion
- Interrupt-driven wheel speed calculation
- Configurable system parameters

## File Structure

```bash
SENSOR_BOARD_WHEEL_DMA/
├── Core/
│   ├── Inc/
│   │   └── main.h
│   ├── Src/
│   │   └── main.c
├── Drivers/
│   ├── CMSIS/
│   └── HAL_Driver/
```
## Getting Started

### Prerequisites
- STM32CubeMX: For project configuration and code generation
- STM32CubeIDE: For compiling and debugging the code
- Hardware: SENSOR_BOARD_WHEEL_DMA

### Setup
1. Clone the repository:
    ```sh
    git clone https://github.com/FinnRobotic/SensorNodeWheel-SG06E.git
    ```
2. Open the project in STM32CubeIDE.
3. Connect your SENSOR_BOARD_WHEEL_DMA to your development environment.
4. Build and flash the project to the board.

### Usage
1. Power on the SENSOR_BOARD_WHEEL_DMA.
2. The system will initialize and start reading sensor data.
3. Sensor data can be accessed through the configured UART or CAN interface.

## Detailed Description

### main.c
The `main.c` file contains the main program logic, including:
- System initialization
- Peripheral configuration (ADC, DMA, TIM, FDCAN)
- Main loop for sensor data processing

### Error Handling
The `Error_Handler` function is used to capture and handle errors during execution. It can be extended to provide more specific error messages.

### Interrupts
- `HAL_GPIO_EXTI_Callback`: Handles external GPIO interrupts for wheel speed calculation.
- `HAL_ADC_ConvCpltCallback`: Handles ADC conversion complete interrupt for DMA transfers.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## Contact
For any queries, please contact [FinnRobotic](mailto:finn.ole.flemming@gmail.com).
