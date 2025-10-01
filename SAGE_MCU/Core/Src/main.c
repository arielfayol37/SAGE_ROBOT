/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR1_DIR_PORT GPIOA
#define MOTOR1_DIR_PIN  GPIO_PIN_9
#define MOTOR1_PWM_CCR  TIM1->CCR4

#define MOTOR2_DIR_PORT GPIOA
#define MOTOR2_DIR_PIN  GPIO_PIN_8
#define MOTOR2_PWM_CCR  TIM2->CCR4

#define Lw  0.465f // Distance between wheels (m)
#define WHEEL_DIAMETER  0.101f // Wheel size (m)
#define GEAR_RATIO  7.3333f // Ratio from the motor to the wheel
#define Max_RPM  3000.0f // motor speed at 100% speed (guess from aliexpress listing, this needs to be calculated)

#define ENCODER_CPR        100       // counts per motor shaft rev (A or B channel edges before x4)
#define ENC_XMULT          2          // 1,2,4 depending on your timer encoder mode
#define ENCODER_COUNTS_PER_MOTOR_REV  ((ENCODER_CPR)*(ENC_XMULT))

// Control loop rate
#define CTRL_HZ            100.0f
#define CTRL_DT            (1.0f/CTRL_HZ)

// PI gains (start conservative; tune on robot)
#define KP                 1.5f
#define KI                 0.0f
#define KD                 0.0f       // optional; start at 0

// PWM output limit (percent)
#define DUTY_MAX_PCT       100.0f
#define DUTY_MIN_PCT       0.0f

// Sign conventions (flip if your wiring is opposite)
#define DIR_FWD_STATE      GPIO_PIN_SET
#define DIR_REV_STATE      GPIO_PIN_RESET

//#define MPU9250_ADDR 	 0x68  // left-shifted for HAL
#define MPU9250_ADDR_7BIT  0x68
#define MPU9250_ADDR       (MPU9250_ADDR_7BIT << 1)  // HAL expects 8-bit

#define MPU9250_REG_DATA 0x3B
#define PWR_MGMT_1 		 0x6B
#define WHO_AM_I   		 0x75

#define MAX_ACCEL_MPS2   8.0f   // example: 0.5 m/s^2

// --- Serial framing ---
#define SOF            0x7E

#define TYPE_CMD       0x01  // Host -> MCU: v,w
#define TYPE_ODOM      0x02  // MCU -> Host: x,y,th,v,w
#define TYPE_IMU       0x03  // MCU -> Host: gx,gy,gz, ax,ay,az

// Payload sizes (bytes)
#define LEN_CMD        (sizeof(float)*2)   // 8
#define LEN_ODOM       (sizeof(float)*5)   // 20
#define LEN_IMU        (sizeof(float)*6)   // 24

#define MPU9250_ACCEL_XOUT_H 0x3B



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile float desired_speed_L = 0.0f;     // m/s  (left wheel)
volatile float desired_speed_R = 0.0f;     // m/s  (right wheel)

volatile float speed_L = 0.0f;     // m/s  (left wheel)
volatile float speed_R = 0.0f;     // m/s  (right wheel)


//static float cmd_speed_L = 0.0f; // m/s  (left wheel) These are used for acceleration limiting
//static float cmd_speed_R = 0.0f; // m/s  (right wheel)

static int32_t enc_prev_L = 0; // delta counts L
static int32_t enc_prev_R = 0; // delta counts R

float robot_X = 0.0f; // m
float robot_Y = 0.0f; // m
float robot_Theta = 0.0f; // rad

// PI integrators
static float i_term_L = 0.0f;
static float i_term_R = 0.0f;

// ----- RX state -----
static uint8_t  rx_state = 0;      // 0: wait SOF, 1: TYPE, 2: LEN, 3: PAYLOAD
static uint8_t  rx_type = 0;
static uint8_t  rx_len  = 0;
static uint8_t  rx_pay[32];        // enough for our largest payload
static uint8_t  rx_pay_idx = 0;

// kick off single-byte interrupt read at init: HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
static uint8_t rx_byte;

// ±2 g, ±250 dps on MPU‑9250
static const float ACCEL_SENSE = 16384.0f;   // LSB/g
static const float GYRO_SENSE  = 131.0f;     // LSB/(deg/s)

// uint8_t rxPacket[9];  // 1 header + 8 bytes for two floats

float v = 0.0f; // velocity m/s
float w = 0.0f; // angular velocity rad/s

uint8_t IM_buf[14]; // IMU data buffer


typedef struct { // IMU data structure
	float ax, ay, az;    // m/s^2
	float gx, gy, gz;    // rad/s
} IMU_Data_t;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void Set_Motor_Speed(uint8_t motor, float duty_percent);
IMU_Data_t Read_IMU(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static inline float clampf(float x, float lo, float hi) {
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

static inline float ramp_to_target(float current, float target, float max_delta)
{
	if (target > current + max_delta)  return current + max_delta;
	if (target < current - max_delta)  return current - max_delta;
	return target;
}

void Motors_Init(void)
{
	// Start PWM channels
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   // Right
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   // Left

	// Start encoder interfaces
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	Set_Motor_Speed(1, 0.0f);   // Right motor
	Set_Motor_Speed(2, 0.0f);   // Left  motor

	// Reset counter
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);

	enc_prev_L = 0;
	enc_prev_R = 0;
}


static void IMU_Init(void)
{
	uint8_t data;

	// Check WHO_AM_I (should return 0x71 for MPU9250)
	if (HAL_I2C_Mem_Read(&hi2c3, MPU9250_ADDR, WHO_AM_I, 1, &data, 1, 100) == HAL_OK) {
		char msg[32];
		int len = snprintf(msg, sizeof(msg), "WHO_AM_I=0x%02X\r\n", data);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
	} else {
		const char *err = "IMU not found!\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
	}

	// Wake up the device (clear sleep bit)
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c3, MPU9250_ADDR, PWR_MGMT_1, 1, &data, 1, 100);

}


IMU_Data_t Read_IMU(void)
{
    IMU_Data_t d;
    uint8_t buf[14];

    if (HAL_I2C_Mem_Read(&hi2c3, MPU9250_ADDR, MPU9250_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 100) != HAL_OK)
    {
        // If read failed, zero values
        d.ax = d.ay = d.az = 0.0f;
        d.gx = d.gy = d.gz = 0.0f;
        return d;
    }

    // Convert raw values
    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    // Accel → m/s^2
    d.ax = (raw_ax / ACCEL_SENSE) * 9.80665f;
    d.ay = (raw_ay / ACCEL_SENSE) * 9.80665f;
    d.az = (raw_az / ACCEL_SENSE) * 9.80665f; // weird ass persistent bias removal

    // Gyro → rad/s
    float gx_dps = raw_gx / GYRO_SENSE;
    float gy_dps = raw_gy / GYRO_SENSE;
    float gz_dps = raw_gz / GYRO_SENSE;

    d.gx = gx_dps * (float)M_PI / 180.0f;
    d.gy = gy_dps * (float)M_PI / 180.0f;
    d.gz = gz_dps * (float)M_PI / 180.0f;

    return d;
}



// Returns counts since last call; handles 16-bit wrap. Bind this to your actual encoder timers.
int32_t encoder_delta_counts(TIM_HandleTypeDef *ht, int32_t *prev_accum)
{
	// Read 16-bit counter (most STM32 timers default to 16b in encoder mode)
	uint16_t now = __HAL_TIM_GET_COUNTER(ht);
	static uint16_t lastL = 0, lastR = 0;

	uint16_t *plast = (ht == &htim3) ? &lastL : &lastR;

	int16_t diff = (int16_t)(now - *plast);   // signed wrap-safe delta
	*plast = now;

	// Accumulate into 32-bit (optional)
	*prev_accum += diff;

	return diff;
}

float counts_to_wheel_mps(int32_t delta_counts)
{
	// Motor revs during dt
	float motor_revs = (float)delta_counts / (float)ENCODER_COUNTS_PER_MOTOR_REV;
	float wheel_revs = motor_revs / GEAR_RATIO;

	float wheel_rps  = wheel_revs / CTRL_DT;
	float circumference = (float)M_PI * WHEEL_DIAMETER;

	return wheel_rps * circumference;  // m/s
}


void Set_Motor_Speed(uint8_t motor, float duty_percent)
{
	GPIO_TypeDef *dirPort;
	uint16_t dirPin;
	volatile uint32_t *pwmCcr;
	uint32_t arr;

	if (motor == 1) {
		dirPort = MOTOR1_DIR_PORT;
		dirPin  = MOTOR1_DIR_PIN;
		pwmCcr  = &TIM1->CCR4;
		arr     = __HAL_TIM_GET_AUTORELOAD(&htim1);
	} else if (motor == 2) {
		dirPort = MOTOR2_DIR_PORT;
		dirPin  = MOTOR2_DIR_PIN;
		pwmCcr  = &TIM2->CCR4;
		arr     = __HAL_TIM_GET_AUTORELOAD(&htim2);
	} else return;

	// direction
	if (duty_percent >= 0)
		HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(dirPort, dirPin, GPIO_PIN_RESET);

	// duty in timer counts (0–ARR)
	float abs_percent = fabsf(duty_percent);
	if (abs_percent > 100.0f) abs_percent = 100.0f;

	*pwmCcr = 100-(uint32_t)((abs_percent / 100.0f) * arr);
}

float Velocity_To_PWM(float v)
{
	// 1. wheel circumference
	float circumference = (float)M_PI * WHEEL_DIAMETER;

	// 2. wheel rpm needed
	float wheel_rpm = (v / circumference) * 60.0f;

	// 3. motor rpm required
	float motor_rpm = wheel_rpm * GEAR_RATIO;

	// 4. duty cycle (0–100 %)
	float duty = (motor_rpm / Max_RPM) * 100.0f;


	return (duty);
}

// v  = forward speed (m/s)
// w  = angular speed (rad/s)
// Lw = distance between wheels (m)
void Robot_Set_Velocity(float v, float w)
{
	desired_speed_L = v - (w * Lw / 2.0f);
	desired_speed_R = v + (w * Lw / 2.0f);
}

void Control_Update(void)
{
	// 1) Measure wheel speeds from encoders
	int32_t dL = encoder_delta_counts(&htim4, &enc_prev_L);
	int32_t dR = -encoder_delta_counts(&htim3, &enc_prev_R);

	float vL_meas = counts_to_wheel_mps(dL); // m/s
	float vR_meas = counts_to_wheel_mps(dR); // m/s inverted because of the encoder direction

	float v_robot = (vL_meas + vR_meas) / 2.0f; // m/s
	float w_robot = (vR_meas - vL_meas) / Lw; // rad/s

	// Integrate to update position and orientation
	robot_Theta += w_robot * CTRL_DT;
	robot_Theta = fmodf(robot_Theta, 2.0f * (float)M_PI); // keep theta in [0, 2pi)

	robot_X += v_robot * cosf(robot_Theta) * CTRL_DT;
	robot_Y += v_robot * sinf(robot_Theta) * CTRL_DT;

	v = v_robot;
	w = w_robot;

	// Errors (m/s)
	float eL = desired_speed_L - vL_meas;
	float eR = desired_speed_R - vR_meas;

	/*
    // PI control (same as before)
    i_term_L += KI * eL * CTRL_DT;
    i_term_R += KI * eR * CTRL_DT;
    i_term_L = clampf(i_term_L, -100.0f, 100.0f);
    i_term_R = clampf(i_term_R, -100.0f, 100.0f);
	 */

	float uL = KP * eL + i_term_L;
	float uR = KP * eR + i_term_R;

	speed_L = clampf(Velocity_To_PWM(desired_speed_L + uL), -DUTY_MAX_PCT, DUTY_MAX_PCT);
	speed_R = clampf(Velocity_To_PWM(desired_speed_R + uR), -DUTY_MAX_PCT, DUTY_MAX_PCT);

	Set_Motor_Speed(1,  speed_R);
	Set_Motor_Speed(2, -speed_L);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t b = rx_byte;

        switch (rx_state)
        {
            case 0: // wait SOF
                if (b == SOF) { rx_state = 1; }
                break;

            case 1: // TYPE
                rx_type = b;
                rx_state = 2;
                break;

            case 2: // LEN
                rx_len = b;
                if (rx_len > sizeof(rx_pay)) {
                    // invalid length -> reset
                    rx_state = 0;
                } else {
                    rx_pay_idx = 0;
                    rx_state = (rx_len == 0) ? 0 : 3;
                }
                break;

            case 3: // PAYLOAD
                rx_pay[rx_pay_idx++] = b;
                if (rx_pay_idx >= rx_len) {
                    // full frame received -> dispatch
                    if (rx_type == TYPE_CMD && rx_len == LEN_CMD) {
                        float v_in, w_in;
                        memcpy(&v_in, &rx_pay[0], 4);
                        memcpy(&w_in, &rx_pay[4], 4);
                        // (optional) limit here if you want:
                        // v_in = clampf(v_in, -V_MAX, V_MAX);
                        // w_in = clampf(w_in, -W_MAX, W_MAX);
                        Robot_Set_Velocity(v_in, w_in);
                    }
                    // else: ignore unknown type/len

                    // reset for next frame
                    rx_state = 0;
                }
                break;
        }

        // re-arm 1-byte RX
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    rx_state = 0;
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

static void TX_Odom(void)
{
    uint8_t buf[3 + LEN_ODOM]; // SOF, TYPE, LEN + payload
    buf[0] = SOF;
    buf[1] = TYPE_ODOM;
    buf[2] = LEN_ODOM;

    // payload: x, y, theta, v, w (float32 LE)
    memcpy(&buf[3],      &robot_X,   4);
    memcpy(&buf[3 + 4],  &robot_Y,   4);
    memcpy(&buf[3 + 8],  &robot_Theta, 4);
    memcpy(&buf[3 + 12], &v, 4);
    memcpy(&buf[3 + 16], &w, 4);

    HAL_UART_Transmit(&huart2, buf, sizeof(buf), HAL_MAX_DELAY);
}

static void TX_Imu(const IMU_Data_t* d)
{
    uint8_t buf[3 + LEN_IMU];
    buf[0] = SOF;
    buf[1] = TYPE_IMU;
    buf[2] = LEN_IMU;

    memcpy(&buf[3],      &d->gx, 4);
    memcpy(&buf[3 + 4],  &d->gy, 4);
    memcpy(&buf[3 + 8],  &d->gz, 4);
    memcpy(&buf[3 + 12], &d->ax, 4);
    memcpy(&buf[3 + 16], &d->ay, 4);
    memcpy(&buf[3 + 20], &d->az, 4);

    HAL_UART_Transmit(&huart2, buf, sizeof(buf), HAL_MAX_DELAY);
}

void Control_Packet(uint8_t Control_Byte)
{
	switch (Control_Byte)
	{
		case 69: // Stop
			NVIC_SystemReset(); // reset the MCU
			break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

	Motors_Init(); // Start these motors dih
	IMU_Init(); // ts pmo sybau


	const uint32_t tick_period   = 1000U / CTRL_HZ; // 10 ms
	const uint32_t odom_period   = 1000U / 50;      // 20 ms -> 50 Hz
	const uint32_t imu_period    = 1000U / 100;     // 10 ms  -> 100 Hz

	uint32_t currentTime;
	uint32_t prevUpdateTime = 0;
	uint32_t prevTXTime = 0;
	uint32_t prevIMUTime = 0;

	HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // start reciever


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		currentTime = HAL_GetTick();

		if( (currentTime - prevUpdateTime) >= tick_period){
			Control_Update();
			prevUpdateTime = currentTime;
		}

		if( (currentTime - prevTXTime) >= odom_period){
			TX_Odom();
			prevTXTime = currentTime;
		}

		if( (currentTime - prevIMUTime) >= imu_period){
			IMU_Data_t d = Read_IMU();   // consider bias removal here
			TX_Imu(&d);
			prevIMUTime = currentTime;
		}

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10B17DB5;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
