# MPU6050_EKF
Extended Kalman Filer Implementation for MPU6050.

The EKF runs in 500Hz I2C DMA interrupt. 

Set up STM32 as follows:
    - I2C DMA for transmit and recieve
    - External Interrupt Pin connect with MPU INT Pin.

How it works ?
    - Whenever MPU data is ready, the INT pin will trigger the external interrupt of STM32. Inside the interrupt handler, call the mpu_get_data(*ekf) function. Because of the DMA configuration for I2C, the process will run simultanously with the MCU of stm32.
    - When the MPU receive data from MPU, using the void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {} to process the data. Inside the interrupt, call mpu_process_data(*mpu) and mpu_ekf_process(* ekf) to proceess MPU data and run the EKF.

Result: No drifting angles + MPU react quickly to the changes of rotation.

<video src="/workspaces/MPU6050_EKF/mpu_ekf.mp4" controls width="100%"></video>
