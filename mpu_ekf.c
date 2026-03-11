#include "mpu_ekf.h"

#define WHO_AM_I_REG     0x75
#define PWR_MGMT_1_REG   0x6B
#define SMPLRT_DIV_REG   0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG   0x41
#define GYRO_CONFIG_REG  0x1B
#define GYRO_XOUT_H_REG  0x43
#define REG_INT_ENABLE   0x38
#define REG_INT_PIN_CFG  0x37
#define REG_CONFIG       0x1A

#define MPU6050_ADDR 0xD0   // 0x68<<1 Due to 7-Bit transfer.

// CONSTANTS OF PROJECT //
// Using +-2g for Accel and +-500deg/s for Gyro
// #define MPU_ACCEL_SCALE   0.00119750976f   // (1.0f / 8192.0f.0f)*9.81. ( +- 4g)

#define MPU_ACCEL_SCALE_G      0.00006103515f  // 1.0f / 16384.0f (Yields G's)
#define MPU_ACCEL_SCALE_MPS2   0.00059875488f  // (1.0f / 16384.0f) * 9.81f (Yields m/s^2)

#define MPU_GYRO_SCALE_DEGS    0.01526717557f  // 1.0f / 65.5f (Yields Deg/s)
#define MPU_GYRO_SCALE_RADS    0.00026646287f  // (1.0f / 65.5f) * (PI / 180.0f) (Yields Rad/s)

#define RAD_TO_DEG_F           57.2957795131f  // Use a float version of this macro
#define DEG_TO_RAD_F           0.0174533f     



uint8_t mpu_init(mpu_struct *mpu)
{
    for (uint8_t i =0; i < 14; i++)
    {
        mpu -> rxData[i] = 0;
    }
    mpu -> rxFlag = 0;
    mpu -> dataReadyFlag =0;
    uint8_t check;  
    uint8_t Data;
    uint8_t numErrors  = 0;
    HAL_StatusTypeDef status;

    // check device ID WHO_AM_I
    status     = HAL_I2C_Mem_Read(mpu -> I2Cx, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &check, 1, HAL_MAX_DELAY);
    numErrors += (status != HAL_OK);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // Wake up Sensor
        Data = 0x00;
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);
        
        // Disable FSYNC, enable Digital Low-pass Filter (fs = 1kHz, bandwidth: acc= 94 Hz, gyr = 98 Hz)
        Data = 0x02;
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);

        // Enable Interrupt and interrupt status bits are cleared on any read operation //
        Data = 0x10;
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, REG_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);

        Data = 0x01;
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        // Data = 0x07; // Without Low pass filter -> the internal rate is 8Khz 
        // Sample Rate = Internal Sample Rate / (1 + SMPLRT_DIV) = 1Khz
        Data = 0x01; // With the Low pass filter enable -> the internal rate is 1Khz
        // Sample Rate = Internal Sample Rate / (1 + SMPLRT_DIV) = 1kHz/2 = 500Hz ~ 0.002ms
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +-2g
        Data = 0x00;
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        //  +-500deg/s
        Data = 0x08;
        status = HAL_I2C_Mem_Write(mpu -> I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
        numErrors += (status != HAL_OK);
        // return 0;
    }
    if (numErrors > 0) return 0;
    else return 1;
}

void  mpu_get_data(mpu_struct *mpu){    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(mpu -> I2Cx, MPU6050_ADDR,ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, mpu -> rxData, 14);
    // Enable for Debugging
    // if (status != HAL_OK) {
    //     // IF YOU HIT THIS PRINT/BREAKPOINT, THE DMA NEVER STARTED!
    //     // It is likely returning HAL_BUSY (0x02)
    //     while(1); // Trap the CPU here so you can see it fail during debugging
    // }
    mpu -> rxFlag = 0;
}

void mpu_process_data( mpu_struct *mpu){
    // acc ={ax_raw}
    int16_t acc[3] = { (int16_t)(mpu -> rxData[0] << 8 | mpu -> rxData[1]),
                       (int16_t)(mpu -> rxData[2] << 8 | mpu -> rxData[3]), 
                       (int16_t)(mpu -> rxData[4] << 8 | mpu -> rxData[5])};

    // int16_t temp   = (int16_t)(mpu -> rxData[6] << 8 | mpu -> rxData[7]);
    
    int16_t gyr[3] = { (int16_t)(mpu -> rxData[8]  << 8 | mpu -> rxData[9]),
                       (int16_t)(mpu -> rxData[10] << 8 | mpu -> rxData[11]), 
                       (int16_t)(mpu -> rxData[12] << 8 | mpu -> rxData[13])};
    
    // Turn data to float - use for experiment
    mpu -> raw_acc_mps2[0] = acc[0] * MPU_ACCEL_SCALE_MPS2;
    mpu -> raw_acc_mps2[1] = acc[1] * MPU_ACCEL_SCALE_MPS2;
    mpu -> raw_acc_mps2[2] = acc[2] * MPU_ACCEL_SCALE_MPS2;

    mpu -> raw_gyr_rps[0] = gyr[0]  * MPU_GYRO_SCALE_RADS;
    mpu -> raw_gyr_rps[1] = gyr[1]  * MPU_GYRO_SCALE_RADS;
    mpu -> raw_gyr_rps[2] = gyr[2]  * MPU_GYRO_SCALE_RADS;
        
    // Accelerometer data Caliration:  T*raw - bias.
    mpu -> acc_mps2[0] = (mpu->Matrix_A[0][0]*mpu -> raw_acc_mps2[0] + mpu->Matrix_A[0][1]*mpu -> raw_acc_mps2[1] + mpu->Matrix_A[0][2]*mpu -> raw_acc_mps2[2]) + mpu -> acc_bias [0];
    mpu -> acc_mps2[1] = (mpu->Matrix_A[1][0]*mpu -> raw_acc_mps2[0] + mpu->Matrix_A[1][1]*mpu -> raw_acc_mps2[1] + mpu->Matrix_A[1][2]*mpu -> raw_acc_mps2[2]) + mpu -> acc_bias [1];
    mpu -> acc_mps2[2] = (mpu->Matrix_A[2][0]*mpu -> raw_acc_mps2[0] + mpu->Matrix_A[2][1]*mpu -> raw_acc_mps2[1] + mpu->Matrix_A[2][2]*mpu -> raw_acc_mps2[2]) + mpu -> acc_bias [2]; 
    
    // Gyroscope Data Cabliration
    mpu -> gyr_rps[0] = mpu -> raw_gyr_rps[0] - mpu -> gyr_bias[0];
    mpu -> gyr_rps[1] = mpu -> raw_gyr_rps[1] - mpu -> gyr_bias[1];
    mpu -> gyr_rps[2] = mpu -> raw_gyr_rps[2] - mpu -> gyr_bias[2];

    mpu -> rxFlag = 1; // Finish process data.
}

#define GRAVITY  9.81f

/* --- CMSIS-DSP Matrix Instances --- */
static arm_matrix_instance_f32 mat_x, mat_x_pred, mat_P, mat_P_pred;
static arm_matrix_instance_f32 mat_F, mat_F_T, mat_Q;
static arm_matrix_instance_f32 mat_H, mat_H_T, mat_R;
static arm_matrix_instance_f32 mat_S, mat_S_inv, mat_K;
static arm_matrix_instance_f32 mat_z, mat_h_x, mat_y;
static arm_matrix_instance_f32 mat_I;
static arm_matrix_instance_f32 mat_temp3x3_1, mat_temp3x3_2, mat_temp3x1;

/* --- Matrix Memory Arrays --- */
// static float32_t x_data[3] = {0, 0, 0};  // States data
static float32_t x_pred_data[3] = {0, 0, 0}; // Predicted States Data
static float32_t P_data[9] = {1,0,0,  0,1,0,  0,0,1}; // Covariance matrix Data
static float32_t P_pred_data[9]; // Predicted Covariance Matrix P
static float32_t F_data[9]; 
static float32_t F_T_data[9];
static float32_t H_data[9];
static float32_t H_T_data[9];
static float32_t S_data[9], S_inv_data[9], K_data[9];
static float32_t z_data[3], h_x_data[3], y_data[3];
static float32_t I_data[9] = {1,0,0,  0,1,0,  0,0,1}; 

static float32_t temp3x3_1_data[9], temp3x3_2_data[9], temp3x1_data[3];

void mpu_ekf_init(mpu_ekf_t *ekf)
{   
    //// 3x1 Vectors
    arm_mat_init_f32(&mat_x,       3, 1, ekf -> angles_rad);
    arm_mat_init_f32(&mat_x_pred,  3, 1, x_pred_data);
    arm_mat_init_f32(&mat_z,       3, 1, z_data);
    arm_mat_init_f32(&mat_h_x,     3, 1, h_x_data);
    arm_mat_init_f32(&mat_y,       3, 1, y_data);
    arm_mat_init_f32(&mat_temp3x1, 3, 1, temp3x1_data);

    // 3x3 Matrices
    arm_mat_init_f32(&mat_P,         3, 3, P_data);
    arm_mat_init_f32(&mat_P_pred,    3, 3, P_pred_data);
    arm_mat_init_f32(&mat_F,         3, 3, F_data);
    arm_mat_init_f32(&mat_F_T,       3, 3, F_T_data);
    arm_mat_init_f32(&mat_Q,         3, 3, ekf->Q_data);
    arm_mat_init_f32(&mat_R,         3, 3, ekf->R_data);
    arm_mat_init_f32(&mat_H,         3, 3, H_data);
    arm_mat_init_f32(&mat_H_T,       3, 3, H_T_data);
    arm_mat_init_f32(&mat_S,         3, 3, S_data);
    arm_mat_init_f32(&mat_S_inv,     3, 3, S_inv_data);
    arm_mat_init_f32(&mat_K,         3, 3, K_data);
    arm_mat_init_f32(&mat_I,         3, 3, I_data);
    arm_mat_init_f32(&mat_temp3x3_1, 3, 3, temp3x3_1_data);
    arm_mat_init_f32(&mat_temp3x3_2, 3, 3, temp3x3_2_data);
    ekf->is_initialised = 1;

}

void mpu_ekf_process(mpu_ekf_t *ekf)
{   
    if (!ekf->is_initialised) return;
    // Some Notes: angles_rad = {roll, pitch, yaw} = {phi, theta, psi} 
    // ==============================================================
    // STEP 1: PREDICT
    // ==============================================================
    float32_t gx = ekf->mpu->gyr_rps[0];
    float32_t gy = ekf->mpu->gyr_rps[1];
    float32_t gz = ekf->mpu->gyr_rps[2];

    float32_t phi   = ekf->angles_rad[0];
    float32_t theta = ekf->angles_rad[1];
    float32_t psi   = ekf->angles_rad[2];

    float32_t tan_theta = tanf(theta);
    float32_t sec_theta = 1.0f / cosf(theta);
    float32_t sin_phi   = sinf(phi);
    float32_t cos_phi   = cosf(phi);
    
    // // 1a. State Transition
    float32_t phi_dot   = gx + sin_phi * tan_theta * gy + cos_phi * tan_theta * gz;
    float32_t theta_dot = cos_phi * gy - sin_phi * gz;
    float32_t psi_dot   = (sin_phi * sec_theta * gy) + (cos_phi * sec_theta * gz);

    // Predict state: x_p = x + x_dot * dt
    x_pred_data[0] = phi   + (phi_dot   * ekf->dt);
    x_pred_data[1] = theta + (theta_dot * ekf->dt);
    x_pred_data[2] = psi   + (psi_dot   * ekf->dt);

    // 1b. Compute Jacobian F_x (Process model jacobian by states)  --- Taken From MATLAB 
    F_data[0] = 1.0f;  F_data[1] = sin_phi * sec_theta * sec_theta * ekf -> dt *  ekf -> mpu -> gyr_rps[1];  F_data[2] = 0.0f;
    F_data[3] = 0.0f;  F_data[4] = 1.0f;                                                                     F_data[5] = 0.0f;
    F_data[6] = 0.0f;  F_data[7] = sin_phi * sec_theta * tan_theta * ekf -> mpu -> gyr_rps[1];               F_data[8] = 1.0f;
    
    // 1c. Compute Jacobian F_w (Process model jacobian by noise)  --- Taken From MATLAB Q_k =  F_w*Q*F_w^T 
    float32_t Q_temp_data[9] = {0};
    Q_temp_data[0] = ekf->Q_data[0] * ekf->dt * ekf->dt; // Scaling by dt is standard for covariance
    Q_temp_data[4] = ekf->Q_data[4] * ekf->dt * ekf->dt;
    Q_temp_data[8] = ekf->Q_data[8] * ekf->dt * ekf->dt;

    arm_matrix_instance_f32 mat_Q_temp;
    arm_mat_init_f32(&mat_Q_temp, 3, 3, Q_temp_data);

    // 1d. Predict Covariance: P_pred = F * P * F^T + F_w*Q*F_w^T
    arm_mat_trans_f32(&mat_F, &mat_F_T);                       // Tranpose of Matrix F
    arm_mat_mult_f32(&mat_F, &mat_P, &mat_temp3x3_1);          // F *P -> store in "mat_temp3x3_1"         
    arm_mat_mult_f32(&mat_temp3x3_1, &mat_F_T, &mat_P_pred);   // F *P * F^T ->  store in "mat_P_pred"  
    arm_mat_add_f32(&mat_P_pred, &mat_Q, &mat_P_pred);         // F * P * F^T + Q -> store in "mat_P_pred" for prediction of Covariance.
    
    // ==============================================================
    // STEP 2: UPDATE
    // ==============================================================
    // Using prediction values to Update EKF x_pred_data = = {roll_pred, pitch_pred, yaw_pred} = {phi_pred, theta_pred, psi_pred} 
    float32_t sin_theta_p = sinf(x_pred_data[1]);
    float32_t cos_theta_p = cosf(x_pred_data[1]);
    float32_t sin_phi_p   = sinf(x_pred_data[0]);
    float32_t cos_phi_p   = cosf(x_pred_data[0]);

    // h_x matrix (from Math model)
    h_x_data[0] = -GRAVITY * sin_theta_p;
    h_x_data[1] =  GRAVITY * cos_theta_p * sin_phi_p;
    h_x_data[2] =  GRAVITY * cos_theta_p * cos_phi_p;

    // H_x - Jacobian Matrix from MATLAB & H_v is a Identity matrix -> H_v*P*H_v^T = P
    H_data[0] = 0.0f;                                 H_data[1] = -GRAVITY * cos_theta_p;             H_data[2] = 0.0f;
    H_data[3] = GRAVITY * cos_theta_p * cos_phi_p;    H_data[4] = -GRAVITY * sin_theta_p * sin_phi_p; H_data[5] = 0.0f;
    H_data[6] = -GRAVITY * cos_theta_p * sin_phi_p;   H_data[7] = -GRAVITY * sin_theta_p * cos_phi_p; H_data[8] = 0.0f;

    // 2c. Innovation Covariance S = H * P_pred * H^T + R
    arm_mat_trans_f32(&mat_H, &mat_H_T);                   // Tranpose of H 
    arm_mat_mult_f32(&mat_H, &mat_P_pred, &mat_temp3x3_1); // H * P_pre -> Store into  mat_temp3x3_1
    arm_mat_mult_f32(&mat_temp3x3_1, &mat_H_T, &mat_S);    //  H * P_pre * H^T -> Store into S     
    arm_mat_add_f32(&mat_S, &mat_R, &mat_S);               // S = S + R -> Add covariance matrix R

    // 2d. Kalman Gain K = P_pred * H^T * inv(S)
    arm_mat_inverse_f32(&mat_S, &mat_S_inv);     // Inverse of S matrix               
    arm_mat_mult_f32(&mat_P_pred, &mat_H_T, &mat_temp3x3_1);    // P_pred * H^T
    arm_mat_mult_f32(&mat_temp3x3_1, &mat_S_inv, &mat_K); // K = P_pred * H^T * inv(S)

    // 2e. Innovation y = z - h_x
    z_data[0] = ekf -> mpu -> acc_mps2[0]; z_data[1] = ekf -> mpu -> acc_mps2[1]; z_data[2] = ekf -> mpu -> acc_mps2[2]; // Ax Ay, Az
    arm_mat_sub_f32(&mat_z, &mat_h_x, &mat_y);
    
    // 2f. State Update x = x_pred + K * y
    arm_mat_mult_f32(&mat_K, &mat_y, &mat_temp3x1);             
    arm_mat_add_f32(&mat_x_pred, &mat_temp3x1, &mat_x);   

    // Update Angles 
    // ekf->angles_rad[0] = ekf->angles_rad[0]; 
    // ekf->angles_rad[1] = ekf->angles_rad[1]; 
    // ekf->angles_rad[2] = ekf->angles_rad[2];
    
    // Change to Degree      
    ekf -> angles_deg[0] = ekf->angles_rad[0] * RAD_TO_DEG_F;
    ekf -> angles_deg[1] = ekf->angles_rad[1] * RAD_TO_DEG_F;
    ekf -> angles_deg[2] = ekf->angles_rad[2] * RAD_TO_DEG_F;

    // 2g. Covariance Update P = (I - K * H) * P_pred
    arm_mat_mult_f32(&mat_K, &mat_H, &mat_temp3x3_1);           
    arm_mat_sub_f32(&mat_I, &mat_temp3x3_1, &mat_temp3x3_2);    
    arm_mat_mult_f32(&mat_temp3x3_2, &mat_P_pred, &mat_P);
}


//--- END OF THE FILE ---// 