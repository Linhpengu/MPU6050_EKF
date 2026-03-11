# MPU6050_EKF
Extended Kalman Filter Implementation for MPU6050.

## The EKF runs in 500Hz I2C DMA interrupt. 

## Set up STM32 as follows:
    - I2C DMA for transmit and receive
    - External Interrupt Pin connects with MPU INT Pin.

## How does it work?
    - The biases and calibration matrix of the accelerometer have to be measured manually. Follow this link: https://www.mathworks.com/help/nav/ref/accelcal.html
    - The biases of the gyroscope are the mean of the measured values of the gyroscope for about 10s.
    - Whenever MPU data is ready, the INT pin will trigger the external interrupt of the STM32. Inside the interrupt handler, call the mpu_get_data(*ekf) function. Because of the DMA configuration for I2C, the process will run simultaneously with the MCU of stm32.
    - When the MPU receives data from the MPU, using the void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {} to process the data. Inside the interrupt, call mpu_process_data(*mpu) and mpu_ekf_process(* ekf) to process MPU data and run the EKF.

## Result: No drifting angles + MPU reacts quickly to the changes of rotation.

<video src="/workspaces/MPU6050_EKF/mpu_ekf.mp4" controls width="100%"></video>
