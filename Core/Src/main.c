/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : To�n b? c?u tr�c AMR du?c d?n d?p v� t?i uu h�a
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050/mpu6050.h"
#include <stdio.h>
#include "encoder.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float alpha_d;
} PID_Config_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARR_MAX_L     4199
#define PID_LIMIT_L   ARR_MAX_L

#define ARR_MAX_R     2099
#define PID_LIMIT_R   ARR_MAX_R

#define Ts            0.01f
#define RAD_S_L       28.6f
#define RAD_S_R       29.0f
#define ARR_MIN       10
#define Base_Link     0.31548f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const PID_Config_t PID_Straight = { .Kp = 0.3f, .Ki = 1.0f, .Kd = 0.09f, .alpha_d = 0.2f };
const PID_Config_t PID_Turn     = { .Kp = 0.2f, .Ki = 0.05f, .Kd = 0.06f, .alpha_d = 0.2f };
const PID_Config_t PID_PostTurn = { .Kp = 0.3f, .Ki = 1.00f, .Kd = 0.09f, .alpha_d = 0.25f };

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t buzzer_done_flag = 0;

/*=========================== MPU6050 EXT PARAM ==============================*/
extern float Real_Accel_X; 
extern float Real_Accel_Y; 
extern float Real_Accel_Z;

extern float acc_calibrated_x; 
extern float acc_calibrated_y; 
extern float acc_calibrated_z;

extern volatile float Final_Accel_X; 
extern volatile float Final_Accel_Y; 
extern volatile float Final_Gyro_Z;

extern float gy_calibrated_x; 
extern float gy_calibrated_y; 
extern float gy_calibrated_z;

extern float Odom_Gyro_X;
extern signed char Is_Calibrated;
extern float Robot_Yaw;
volatile signed char mpu_data_ready_flag = 0;

/*=========================== ENCODER PARAM ====================================*/
Encoder_t LeftEncoder;
Encoder_t RightEncoder;
volatile uint32_t prev_tick = 0;
const uint32_t sample_period_ms = 10;

int32_t pulse_left;
float left_angle; 
float left_rad_raw; 
float left_vel_raw;
float left_rad_filt; 
float left_vel_filt; 
float alpha_vel_left = 0.3f;

volatile uint16_t pulse_right;
float right_angle; 
float right_rad_raw; 
float right_vel_raw;
float right_rad_filt; 
float right_vel_filt; 
float alpha_vel_right = 0.2f;

uint32_t step_tick = 0;
uint16_t current_pwm = 0;

/*=========================== PI_VEL PARAMETER ===============================*/
volatile uint8_t pid_trigger = 0;

// Left side velocity control
float Kp_L = 50.0f; 
float Ki_L = 1000.0f; 
float Kb_L = 0.0f;
float error_sat_p_L = 0.0f; 
float error_sat_L = 0.0f; 
float ui_p_L = 0.0f; 
float error_integral_L = 0.0f;
volatile int16_t motor_L_output = 0; 
volatile float setpoint_L = 0.0f;

// Right side velocity control
float Kp_R = 120.0f; 
float Ki_R = 900.0f; float 
Kb_R = 0.0f;
float error_sat_p_R = 0.0f; 
float error_sat_R = 0.0f; 
float ui_p_R = 0.0f; float 
error_integral_R = 0.0f;
volatile int16_t motor_R_output = 0; 
volatile float setpoint_R = 0.0f;

/*=========================== PID_ANGLE PARAMETER ============================*/
float Kp_angle = 0.1f; 
float Ki_angle = 0.0f; 
float Kd_angle = 0.0f; 
float Kb_angle = 0.0f;
float alpha_d_angle = 0.15f;
float angle_error = 0.0f; 
float angle_last_error = 0.0f; 
float angle_integral = 0.0f;
const float PID_LIMIT_ANGLE = 20.0f;
const float ANGLE_I_LIMIT   = 5.0f;
float error_sat_p_angle = 0.0f; 
float Setpoint_Yaw = 0.0f; 
float last_ud_f_angle = 0.0f;
float target_speed_rad_s = 0.0f;

/*=========================== STATE MACHINE PARAM ============================*/
uint32_t run_time_counter = 0;
float target_linear_speed = 0.0f;
float Target_Yaw = 0.0f;

/*=========================== AUTO CALIB GYRO_CORRECTION =====================*/
const float WHEEL_RADIUS = 0.05f;
const float DEG_TO_RAD   = 0.0174532925f;

float calib_wheel_yaw_rad = 0.0f; 
float calib_wheel_yaw_deg = 0.0f;
uint8_t calib_step_state = 0; 
float recommended_gyro_scale = 0.0f; 
float recommended_base_link = 0.0f;

/*=========================== UART ===========================================*/

// Received data
uint8_t rxFrame[10];
uint8_t uart_flag;
float linear_x, angular_z;
volatile uint16_t uart_timeout_counter = 0;

// Transmitted data
uint8_t txFrame[14];
float x = 0.0f;
float y = 0.0f;
float omega = 0.0f;
float theta = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void encoder_process(void);
void PI_vel(float rad_L, float rad_R);
float PID_angle(float setpoint, float feedback, float dt);
void Set_Motor(int16_t left_output, int16_t right_output);
void AMR_Trajectory_Process(void);
void Auto_Calib_Turn_Place(void);
void Read_DMP(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*==================== MPU_INTERUPT ===========================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin)
    {      
        mpu_data_ready_flag = 1; 
    }
}

/*================== CACULATATE ENCODER ======================*/
void encoder_process(void)
{
    Encoder_Update(&LeftEncoder);
    Encoder_Update(&RightEncoder);
    
    left_rad_raw  = LeftEncoder.speed_rad_s;
    left_vel_raw  = LeftEncoder.velocity_m_s;
    left_rad_filt = (alpha_vel_left * left_rad_raw) + ((1.0f - alpha_vel_left) * left_rad_filt);
    left_vel_filt = (alpha_vel_left * left_vel_raw) + ((1.0f - alpha_vel_left) * left_vel_filt);
    
    right_rad_raw  = RightEncoder.speed_rad_s;    
    right_vel_raw  = RightEncoder.velocity_m_s;    
    right_rad_filt = (alpha_vel_right * right_rad_raw) + ((1.0f - alpha_vel_right) * right_rad_filt);
    right_vel_filt = (alpha_vel_right * right_vel_raw) + ((1.0f - alpha_vel_right) * right_vel_filt);
}

/*=================== PI VELOCITY =========================*/
void PI_vel(float rad_L, float rad_R) 
{
    float error_L = (float)setpoint_L - rad_L;
    float up_L = Kp_L * error_L;
    float ui_L = ui_p_L + Ts * (Ki_L * error_L + Kb_L * error_sat_p_L);
    float out_L_raw = up_L + ui_L;

    if (out_L_raw > PID_LIMIT_L) {
        motor_L_output = PID_LIMIT_L;
        error_sat_L = PID_LIMIT_L - out_L_raw;
    }
    else if (out_L_raw < -PID_LIMIT_L) {
        motor_L_output = -PID_LIMIT_L;
        error_sat_L = -PID_LIMIT_L - out_L_raw;
    }
    else {
        motor_L_output = (int16_t)out_L_raw;
        error_sat_L = 0.0f;
    }
    ui_p_L = ui_L;
    error_sat_p_L = error_sat_L;

    float error_R = (float)setpoint_R - rad_R;
    float up_R = Kp_R * error_R;
    float ui_R = ui_p_R + Ts * (Ki_R * error_R + Kb_R * error_sat_p_R);
    float out_R_raw = up_R + ui_R;

    if (out_R_raw > PID_LIMIT_R) {
        motor_R_output = PID_LIMIT_R;
        error_sat_R = PID_LIMIT_R - out_R_raw;
    } 
    else if (out_R_raw < -PID_LIMIT_R) {
        motor_R_output = -PID_LIMIT_R;
        error_sat_R = -PID_LIMIT_R - out_R_raw;
    } 
    else {
        motor_R_output = (int16_t)out_R_raw;
        error_sat_R = 0.0f;
    }
    ui_p_R = ui_R;
    error_sat_p_R = error_sat_R;
}

/*======================== PID ANGLE =========================*/
float PID_angle(float setpoint, float feedback, float dt)
{
    angle_error = setpoint - feedback;

    while (angle_error > 180.0f)  
			angle_error -= 360.0f;
    while (angle_error < -180.0f) 
			angle_error += 360.0f;

    float up_angle = Kp_angle * angle_error;
    float ui_angle = angle_integral + dt * (Ki_angle * angle_error + Kb_angle * error_sat_p_angle);
    float ud_angle_raw = Kd_angle * (angle_error - angle_last_error) / dt;
    
    float ud_angle_filtered = (alpha_d_angle * ud_angle_raw) + ((1.0f - alpha_d_angle) * last_ud_f_angle);
    last_ud_f_angle = ud_angle_filtered;

    float out_angle_raw = up_angle + ui_angle + ud_angle_filtered;
    float angle_output = 0.0f;
    float error_sat_angle = 0.0f;
    
    if (out_angle_raw > PID_LIMIT_ANGLE) {
        angle_output = PID_LIMIT_ANGLE;
        error_sat_angle = PID_LIMIT_ANGLE - out_angle_raw;
    }
    else if (out_angle_raw < -PID_LIMIT_ANGLE) {
        angle_output = -PID_LIMIT_ANGLE;
        error_sat_angle = -PID_LIMIT_ANGLE - out_angle_raw;
    }
    else {
        angle_output = out_angle_raw;
        error_sat_angle = 0.0f;
    }

    angle_integral = ui_angle; 
    error_sat_p_angle = error_sat_angle;
    angle_last_error = angle_error;
    
    return angle_output;
}

/*======================== MOTOR CONTROL ========================*/
void Set_Motor(int16_t left_output, int16_t right_output)
{
    if (left_output >= 0) {
        if (left_output > ARR_MAX_L) left_output = ARR_MAX_L;
        TIM8->CCR3 = ARR_MIN;
        TIM8->CCR4 = left_output;
    }
    else {
        left_output = -left_output;
        if (left_output > ARR_MAX_L) left_output = ARR_MAX_L;
        TIM8->CCR3 = left_output;
        TIM8->CCR4 = ARR_MIN;
    }

    if (right_output >= 0) {
        if (right_output > ARR_MAX_R) right_output = ARR_MAX_R;
        TIM4->CCR3 = right_output;
        TIM4->CCR4 = ARR_MIN;
    }
    else {
        right_output = -right_output;
        if (right_output > ARR_MAX_R) right_output = ARR_MAX_R;
        TIM4->CCR3 = ARR_MIN;
        TIM4->CCR4 = right_output;
    }
}
/*============================ CODE CALIB ANGLE ====================================*/
void AMR_Trajectory_Process(void)
{
    static float dynamic_linear_speed = 0.0f; 
    
    float raw_target_speed = linear_x / WHEEL_RADIUS;
    
    float target_speed = fabsf(raw_target_speed);
    
    if (target_speed > 5.0f) {
        target_speed = 5.0f;
    }

    // 2. TÍNH CHUẨN ACCEL_RATE (Gia tốc cộng thêm mỗi chu kỳ 10ms)
    float accel_time_s = 1.50f;                    
    float accel_ticks  = accel_time_s / Ts;       
    float accel_rate   = target_speed / accel_ticks; 

    float current_target_yaw = Target_Yaw;
    float max_turn_speed = 2.0f;    
    PID_Config_t active_pid; 
    
    if (run_time_counter < 500) 
    {
        active_pid = PID_Straight;
        current_target_yaw = Target_Yaw;
        
        if (run_time_counter < 150) { 
            dynamic_linear_speed += accel_rate;
            if (dynamic_linear_speed > target_speed) dynamic_linear_speed = target_speed;
        }
				
        else if (run_time_counter < 350) {
            dynamic_linear_speed = target_speed;
        }

        else { 
            dynamic_linear_speed -= accel_rate;
            if (dynamic_linear_speed < 0.0f) dynamic_linear_speed = 0.0f;
        }
    }
		
    else if (run_time_counter >= 500 && run_time_counter < 650) 
    {
        active_pid = PID_Turn;
        dynamic_linear_speed = 0.0f; 
        current_target_yaw = Target_Yaw + 180.0f; 
        angle_integral = 0.0f; 
    }

    else if (run_time_counter >= 650 && run_time_counter < 1150) 
    {
        current_target_yaw = Target_Yaw + 180.0f;
        
        if (run_time_counter < 800) { 
            active_pid = PID_PostTurn;
            angle_integral = 0.0f; 
        }
        else {
            active_pid = PID_Straight;
        }
        
        // Tăng tốc trong 1.5s đầu lượt về (650 -> 800 ticks)
        if (run_time_counter < 800) { 
            dynamic_linear_speed += accel_rate;
            if (dynamic_linear_speed > target_speed) dynamic_linear_speed = target_speed;
        }
        else if (run_time_counter < 1000) {
            dynamic_linear_speed = target_speed;
        }
        else { 
            dynamic_linear_speed -= accel_rate;
            if (dynamic_linear_speed < 0.0f) dynamic_linear_speed = 0.0f;
        }
    }
		
    else 
    {
        active_pid = PID_Straight;
        dynamic_linear_speed = 0.0f;
    }

    // Nạp tham số PID góc hướng
    Kp_angle      = active_pid.Kp;
    Ki_angle      = active_pid.Ki;
    Kd_angle      = active_pid.Kd;
    alpha_d_angle = active_pid.alpha_d;
    Kb_angle      = (Kp_angle > 0.0f) ? (Ki_angle / Kp_angle) : 0.0f;

    // Tính toán đầu ra chỉnh góc
    float delta_omega = PID_angle(current_target_yaw, Robot_Yaw, Ts);
    if (delta_omega > max_turn_speed)  delta_omega = max_turn_speed;
    if (delta_omega < -max_turn_speed) delta_omega = -max_turn_speed;
    
    float dir = (linear_x >= 0.0f) ? 1.0f : -1.0f;
    
    setpoint_L = (dynamic_linear_speed * dir) - delta_omega;
    setpoint_R = (dynamic_linear_speed * dir) + delta_omega;
}

/*==================== PROCESS RECEIVED DATA =====================*/
void AMR_Teleop_Process(void)
{
    static float current_speed_rad_s = 0.0f;
    
    uart_timeout_counter += 10;
    if (uart_timeout_counter > 200)
    {
			linear_x  = 0.0f;
			angular_z = 0.0f;
    }

    target_speed_rad_s = linear_x / WHEEL_RADIUS;
    
    if (target_speed_rad_s > 5.0f)   
			target_speed_rad_s = 5.0f;
    if (target_speed_rad_s < -5.0f)  
			target_speed_rad_s = -5.0f;

    const float max_change_per_tick = 0.03333f;
    float speed_error = target_speed_rad_s - current_speed_rad_s;
    
    if (speed_error > max_change_per_tick) {
      current_speed_rad_s += max_change_per_tick;
    }
    else if (speed_error < -max_change_per_tick) {
      current_speed_rad_s -= max_change_per_tick;
    }
    else {
      current_speed_rad_s = target_speed_rad_s;
    }
		
    if (fabsf(angular_z) > 0.01f)
    {
			Kp_angle      = PID_Turn.Kp;
			Ki_angle      = PID_Turn.Ki;
			Kd_angle      = PID_Turn.Kd;
			alpha_d_angle = PID_Turn.alpha_d;
			Kb_angle      = (Kp_angle > 0.0f) ? (Ki_angle / Kp_angle) : 0.0f;

			float wheel_turn_rate = (angular_z * Base_Link) / (2.0f * WHEEL_RADIUS);

			setpoint_L = -wheel_turn_rate;
			setpoint_R =  wheel_turn_rate;

			Target_Yaw = Robot_Yaw;
			angle_integral = 0.0f;
    }

    else
    {
			// Chuyển sang thông số PID Chạy thẳng
			Kp_angle      = PID_Straight.Kp;
			Ki_angle      = PID_Straight.Ki;
			Kd_angle      = PID_Straight.Kd;
			alpha_d_angle = PID_Straight.alpha_d;
			Kb_angle      = (Kp_angle > 0.0f) ? (Ki_angle / Kp_angle) : 0.0f;

			// Bù lệch góc hướng dựa trên Target_Yaw đã lưu
			float delta_omega = PID_angle(Target_Yaw, Robot_Yaw, Ts);
			
			float max_turn_speed = 2.0f;
			if (delta_omega > max_turn_speed)  delta_omega = max_turn_speed;
			if (delta_omega < -max_turn_speed) delta_omega = -max_turn_speed;

			// Chiều đi tiến hoặc lùi
			setpoint_L = current_speed_rad_s - delta_omega;
			setpoint_R = current_speed_rad_s + delta_omega;
    }
}

/*=========================== UART1 RECEIVE===========================*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	 if(huart->Instance == USART1){
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, 10);
		 __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		uart_flag = 1;
		if(rxFrame[0] == 0xAA && rxFrame[9] == 0x0D)
		{
			uint8_t *p;

			p = (uint8_t *)&linear_x;
			p[0] = rxFrame[1];
			p[1] = rxFrame[2];
			p[2] = rxFrame[3];
			p[3] = rxFrame[4];

			p = (uint8_t *)&angular_z;
			p[0] = rxFrame[5];
			p[1] = rxFrame[6];
			p[2] = rxFrame[7];
			p[3] = rxFrame[8];
			
			uart_timeout_counter = 0;
		}
	}
}

/*======================= PID ACTIVATE ========================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        pid_trigger = 1;
        run_time_counter++;
    }
}

/*=========================== AUTO CALIB GYRO_CORRECTION =====================*/
void Auto_Calib_Turn_Place(void)
{
    if (calib_step_state == 0) {
        setpoint_L = -2.0f; 
				setpoint_R = 2.0f;
        calib_wheel_yaw_rad = 0.0f; calib_step_state = 1;
    }
    else if (calib_step_state == 1) {
        float current_w_wheel = (right_vel_filt - left_vel_filt) / Base_Link;
        calib_wheel_yaw_rad += current_w_wheel * Ts;
        calib_wheel_yaw_deg = calib_wheel_yaw_rad * 57.29577951f;
            
        if (fabs(Robot_Yaw) >= 1800.0f) {
            setpoint_L = 0.0f; setpoint_R = 0.0f;
            motor_L_output = 0; motor_R_output = 0;
            Set_Motor(0, 0); calib_step_state = 2; 
        }
    }
    else if (calib_step_state == 2) {
        GPIOE->ODR |= (1<<2); 
				HAL_Delay(200); 
				GPIOE->ODR &=~ (1<<2);
        recommended_base_link = Base_Link * (calib_wheel_yaw_deg / 1800.0f);
        calib_step_state = 3; 
    }
}

/*========================= ODOM TRANSMIT ======================*/
void UART6_Send_Odom(float x_pos, float y_pos, float omega_vel)
{
    
    txFrame[0] = 0xAA;
    
    // 2. Payload: Copy 12 bytes của (x, y, omega)
    *(float*)&txFrame[1] = x_pos;
    *(float*)&txFrame[5] = y_pos;
    *(float*)&txFrame[9] = omega_vel;
    
    // 3. Byte Stop
    txFrame[13] = 0x0D;

		HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, txFrame, sizeof(txFrame), 10);
    if (status != HAL_OK) {
        GPIOA->ODR |= (1<<2);
    }
}

//void UART6_Send_Odom(float x_pos, float y_pos, float omega_vel)
//{
//		const char *msg = "DA NHAN UART6\r\n";
//    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 10);
//}

/*========================= CACULATE ODOM ======================*/
void Update_Odometry(void)
{
    // 1. Lấy vận tốc góc omega (rad/s) từ Gyro Z MPU6050 để gửi lên PC/ROS
    omega = Final_Gyro_Z * DEG_TO_RAD;

    // 2. Cập nhật góc theta (rad) TRỰC TIẾP từ Robot_Yaw (DMP MPU6050)
    // Giúp loại bỏ hoàn toàn trôi góc (drift) do tích phân Gyro
    theta = Robot_Yaw * DEG_TO_RAD;

    // 3. Tính vận tốc dài v_robot (m/s) từ Encoder 2 bánh
    float v_robot = (left_vel_filt + right_vel_filt) / 2.0f;

    // 4. Cập nhật tọa độ X, Y
    x += v_robot * cosf(theta) * Ts;
    y += v_robot * sinf(theta) * Ts;

    // 5. Chuẩn hóa góc theta về khoảng [-PI, PI] (chỉ để truyền/hiển thị)
    while (theta >  3.14159265f) 
			theta -= 2.0f * 3.14159265f;
    while (theta < -3.14159265f) 
			theta += 2.0f * 3.14159265f;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  GPIOA->ODR &=~ (1<<2);   // LED b�o ngu?n ON
  GPIOE->ODR &=~ (1<<1);   // Cho ph�p chip MPU6050 ch?y
  
  MPU6050_initialize(); 
  DMP_Init();           
  
  // B?t xung di?u khi?n d?ng co H-Bridge
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  
  // B?t ph?n c?ng d?c xung Encoder b�nh xe
  Encoder_Init(&LeftEncoder, TIM2);
  Encoder_Init(&RightEncoder, TIM3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Kh?i d?ng ng?t c?ng chu k? di?u khi?n di?u t?c 10ms (Timer 5)
  HAL_TIM_Base_Start_IT(&htim5);
  
  // Kh?i t?o h? s? Anti-windup ban d?u cho b�nh v� v? tr�
  Kb_L = Ki_L / Kp_L;
  Kb_R = Ki_R / Kp_R;
  Kb_angle = Ki_angle / Kp_angle;
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, 10);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (mpu_data_ready_flag == 1)
    {
        mpu_data_ready_flag = 0;
        Read_DMP(); 
    }
		
    if (Is_Calibrated == 1 && buzzer_done_flag == 0)
    {
			buzzer_done_flag = 1;
			for(int i = 0; i < 3; i++) {   
				GPIOE->ODR |= (1<<2); 
				HAL_Delay(50);            
				GPIOE->ODR &=~ (1<<2); 
				HAL_Delay(50);
			}
			run_time_counter = 0; 
    }

    if (Is_Calibrated == 0) 
    {
			setpoint_L = 0.0f; 
			setpoint_R = 0.0f;
			motor_L_output = 0; 
			motor_R_output = 0;
			Set_Motor(0, 0);
			
			if (pid_trigger == 1) {
					pid_trigger = 0;
					encoder_process(); 
			}
			continue; 
    }

    if (pid_trigger == 1)
    {
      pid_trigger = 0;
    
			encoder_process();
			Update_Odometry();
			AMR_Teleop_Process();

			PI_vel(left_rad_filt, right_rad_filt);
			Set_Motor(motor_L_output, motor_R_output);

			static uint8_t send_odom_counter = 0;
			if (++send_odom_counter >= 2) {
					send_odom_counter = 0;
					// Gọi lại hàm này để vừa ĐÓNG GÓI mảng txFrame vừa GỬI ĐI!
					UART6_Send_Odom(x, y, omega); 
			}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|MPU_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin MPU_Enable_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|MPU_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
