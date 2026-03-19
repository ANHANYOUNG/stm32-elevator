/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Elevator System with LCD & Overload Protection (F103RB)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_IDLE,
    STATE_MOVE_UP,
    STATE_MOVE_DOWN,
    STATE_DOOR_OPEN,
    STATE_DOOR_CLOSE
} ElevatorState_t;

// 엘리베이터 구조체 정의
typedef struct {
    int cur_floor;
    int current_dir;
    ElevatorState_t state;
    uint32_t door_open_time;
    uint32_t fsrValue;          // 과적 센서 (바닥)
    uint32_t doorPressure;      // 문 압력 센서
    bool is_overloaded;
    bool door_blocked;          // 문에 장애물 감지
    bool is_A;                  // true = A호기, false = B호기
    
    // 요청 배열
    int Call_Up[4];      // MAX_FLOOR + 1
    int Call_Down[4];
    int Call_In[4];
    
    // 디버그용 (실제 PWM 값)
    uint16_t motor_ccr;     // 현재 모터 PWM 값 (1360~1550)
    uint16_t servo_ccr;     // 현재 서보 PWM 값 (500~1500)
    int sensed_floor;       // 현재 감지된 층 (0=없음, 1~3=층)
    int target_floor;       // 목표 층
} Elevator_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- LCD 설정 ---
#define LCD_ADDR (0x27 << 1) // I2C 주소 (안되면 0x3F << 1 로 변경)

// --- 과적 설정 ---
#define OVERLOAD_THRESHOLD 1000 // 0~4095 중 이 값 넘으면 과적 (조절 필요)
#define DOOR_PRESSURE_THRESHOLD 10 // 문 압력 임계값 (손가락 끼임 감지)

// --- A호기 센서 핀 (PB12, 13, 14) ---
#define SENSOR_1F_Pin GPIO_PIN_12
#define SENSOR_1F_GPIO_Port GPIOB
#define SENSOR_2F_Pin GPIO_PIN_13
#define SENSOR_2F_GPIO_Port GPIOB
#define SENSOR_3F_Pin GPIO_PIN_14
#define SENSOR_3F_GPIO_Port GPIOB

// --- B호기 센서 핀 (PB1, 2, 10) ---
#define SENSOR2_1F_Pin GPIO_PIN_1
#define SENSOR2_1F_GPIO_Port GPIOB
#define SENSOR2_2F_Pin GPIO_PIN_2
#define SENSOR2_2F_GPIO_Port GPIOB
#define SENSOR2_3F_Pin GPIO_PIN_10
#define SENSOR2_3F_GPIO_Port GPIOB

// --- 음계 ---
const int C4=262, G4=392, C5=523, E5=659, G5=784, C6=1047;

// --- 상수 ---
#define MAX_FLOOR 3
#define UP 1
#define DOWN -1
#define STOP 0
#define DOOR_WAIT_TIME 5000

// --- PWM (TIM1) ---
#define SERVO_0_DEG    1500    // 문 닫힘
#define SERVO_90_DEG   500     // 문 열림
#define SERVO_STEP     10      // 부드러운 이동을 위한 스텝 크기
#define SERVO_DELAY    20      // 각 스텝 사이 딜레이 (ms)
// --- 엘리베이터 모터 PWM 설정 (TIM1 Channel 1) ---
#define MOTOR_SPEED_UP   1550  // 상승 속도 (1500(slow)~2300(fast))
#define MOTOR_SPEED_DOWN 1400  // 하강 속도 (700(fast)~1500(slow))
#define MOTOR_STOP       1480     // 정지
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// 엘리베이터 인스턴스
Elevator_t elevator_A;
Elevator_t elevator_B;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// LCD 함수
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Print(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void Update_LCD_Status_Dual(Elevator_t *elev_A, Elevator_t *elev_B);

// 엘리베이터 초기화
void Init_Elevator_A(void);
void Init_Elevator_B(void);

// 엘리베이터 함수
void play_sound(int frequency, int duration);
void play_open_sound(void);
void emergency_sound(void);
void button_sound(void);
void arrive_sound(void);
void delay_us(uint16_t us);
void elevator_state_machine(Elevator_t *elev);
void process_keypad_input(void);
char scan_keypad(void);
bool Has_Requests_Above(Elevator_t *elev, int floor);
bool Has_Requests_Below(Elevator_t *elev, int floor);
void Motor_Drive(Elevator_t *elev, int direction);
void Motor_Stop(Elevator_t *elev);
void Door_Motor(Elevator_t *elev, int action);
bool Door_Motor_Smooth(Elevator_t *elev, int action);
int Check_Hall_Sensor(Elevator_t *elev);
Elevator_t* Find_Nearest_Elevator(int target_floor, int direction);
uint32_t Read_ADC_Channel(uint32_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ADC 채널 하나씩 읽는 함수
uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // 채널 설정
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0;
    }
    
    // ADC 시작
    HAL_ADC_Start(&hadc1);
    
    // 변환 완료 대기 (최대 100ms)
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t value = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return value;
    }
    
    HAL_ADC_Stop(&hadc1);
    return 0;
}

// --- LCD 구현 ---
void LCD_Write_Byte(uint8_t val, uint8_t mode) {
    uint8_t high_nibble = val & 0xF0;
    uint8_t low_nibble = (val << 4) & 0xF0;
    uint8_t rs_bit = mode ? 0x01 : 0x00;
    uint8_t backlight = 0x08;
    uint8_t data[4];

    data[0] = high_nibble | backlight | rs_bit | 0x04;
    data[1] = high_nibble | backlight | rs_bit | 0x00;
    data[2] = low_nibble  | backlight | rs_bit | 0x04;
    data[3] = low_nibble  | backlight | rs_bit | 0x00;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data, 4, 10);
}
void LCD_SendCommand(uint8_t cmd) { LCD_Write_Byte(cmd, 0); }
void LCD_SendData(uint8_t data) { LCD_Write_Byte(data, 1); }
void LCD_Init(void) {
    HAL_Delay(50);
    LCD_SendCommand(0x30); HAL_Delay(5);
    LCD_SendCommand(0x30); HAL_Delay(1);
    LCD_SendCommand(0x30); HAL_Delay(1);
    LCD_SendCommand(0x20); HAL_Delay(1);
    LCD_SendCommand(0x28);
    LCD_SendCommand(0x08);
    LCD_SendCommand(0x01); HAL_Delay(2);
    LCD_SendCommand(0x06);
    LCD_SendCommand(0x0C);
}
void LCD_Print(char *str) { while (*str) LCD_SendData(*str++); }
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    LCD_SendCommand(addr + col);
}

// 엘리베이터 초기화 함수
void Init_Elevator_A(void) {
    elevator_A.cur_floor = 1;
    elevator_A.current_dir = STOP;
    elevator_A.state = STATE_IDLE;
    elevator_A.is_overloaded = false;
    elevator_A.door_blocked = false;
    elevator_A.is_A = true;  // A호기
    elevator_A.door_open_time = 0;
    elevator_A.fsrValue = 0;
    elevator_A.doorPressure = 0;
    
    // 디버그 초기화
    elevator_A.motor_ccr = MOTOR_STOP;
    elevator_A.servo_ccr = SERVO_0_DEG;
    elevator_A.sensed_floor = 0;
    elevator_A.target_floor = 0;
    
    memset(elevator_A.Call_Up, 0, sizeof(elevator_A.Call_Up));
    memset(elevator_A.Call_Down, 0, sizeof(elevator_A.Call_Down));
    memset(elevator_A.Call_In, 0, sizeof(elevator_A.Call_In));
}

void Init_Elevator_B(void) {
    elevator_B.cur_floor = 1;
    elevator_B.current_dir = STOP;
    elevator_B.state = STATE_IDLE;
    elevator_B.is_overloaded = false;
    elevator_B.door_blocked = false;
    elevator_B.is_A = false;  // B호기
    elevator_B.door_open_time = 0;
    elevator_B.fsrValue = 0;
    elevator_B.doorPressure = 0;
    
    // 디버그 초기화
    elevator_B.motor_ccr = MOTOR_STOP;
    elevator_B.servo_ccr = SERVO_0_DEG;
    elevator_B.sensed_floor = 0;
    elevator_B.target_floor = 0;
    
    memset(elevator_B.Call_Up, 0, sizeof(elevator_B.Call_Up));
    memset(elevator_B.Call_Down, 0, sizeof(elevator_B.Call_Down));
    memset(elevator_B.Call_In, 0, sizeof(elevator_B.Call_In));
}

// LCD 2대 동시 표시
void Update_LCD_Status_Dual(Elevator_t *elev_A, Elevator_t *elev_B) {
    static int prev_A_floor = -1, prev_B_floor = -1;
    static ElevatorState_t prev_A_state = STATE_IDLE, prev_B_state = STATE_IDLE;
    static bool prev_A_ovl = false, prev_B_ovl = false;
    
    if (elev_A->cur_floor != prev_A_floor || elev_A->state != prev_A_state ||
        elev_A->is_overloaded != prev_A_ovl ||
        elev_B->cur_floor != prev_B_floor || elev_B->state != prev_B_state ||
        elev_B->is_overloaded != prev_B_ovl) {
        
        char buf[17];
        
        // 1줄: A호기
        LCD_SetCursor(0, 0);
        if (elev_A->is_overloaded) {
            sprintf(buf, "A:%dF !!OVL!!   ", elev_A->cur_floor);
        } else {
            const char *state_A = (elev_A->state == STATE_MOVE_UP) ? "UP" : 
                                   (elev_A->state == STATE_MOVE_DOWN) ? "DN" : "ST";
            sprintf(buf, "A:%dF %-3s       ", elev_A->cur_floor, state_A);
        }
        LCD_Print(buf);
        
        // 2줄: B호기
        LCD_SetCursor(1, 0);
        if (elev_B->is_overloaded) {
            sprintf(buf, "B:%dF !!OVL!!   ", elev_B->cur_floor);
        } else {
            const char *state_B = (elev_B->state == STATE_MOVE_UP) ? "UP" : 
                                   (elev_B->state == STATE_MOVE_DOWN) ? "DN" : "ST";
            sprintf(buf, "B:%dF %-3s       ", elev_B->cur_floor, state_B);
        }
        LCD_Print(buf);
        
        prev_A_floor = elev_A->cur_floor;
        prev_A_state = elev_A->state;
        prev_A_ovl = elev_A->is_overloaded;
        prev_B_floor = elev_B->cur_floor;
        prev_B_state = elev_B->state;
        prev_B_ovl = elev_B->is_overloaded;
    }
}

// --- 엘리베이터 공통 함수 (구조체 포인터 사용) ---
int Check_Hall_Sensor(Elevator_t *elev) {
    if (elev->is_A) {
        // A호기: PB12, 13, 14
        if (HAL_GPIO_ReadPin(GPIOB, SENSOR_1F_Pin) == GPIO_PIN_RESET) return 1;
        if (HAL_GPIO_ReadPin(GPIOB, SENSOR_2F_Pin) == GPIO_PIN_RESET) return 2;
        if (HAL_GPIO_ReadPin(GPIOB, SENSOR_3F_Pin) == GPIO_PIN_RESET) return 3;
    } else {
        // B호기: PB1, 2, 10
        if (HAL_GPIO_ReadPin(GPIOB, SENSOR2_1F_Pin) == GPIO_PIN_RESET) return 1;
        if (HAL_GPIO_ReadPin(GPIOB, SENSOR2_2F_Pin) == GPIO_PIN_RESET) return 2;
        if (HAL_GPIO_ReadPin(GPIOB, SENSOR2_3F_Pin) == GPIO_PIN_RESET) return 3;
    }
    return 0;
}

bool Has_Requests_Above(Elevator_t *elev, int floor) {
    for (int i = floor + 1; i <= MAX_FLOOR; i++) 
        if (elev->Call_Up[i] || elev->Call_Down[i] || elev->Call_In[i]) return true;
    return false;
}

bool Has_Requests_Below(Elevator_t *elev, int floor) {
    for (int i = floor - 1; i >= 1; i--) 
        if (elev->Call_Up[i] || elev->Call_Down[i] || elev->Call_In[i]) return true;
    return false;
}

void Motor_Drive(Elevator_t *elev, int direction) {
    uint16_t speed = (direction == UP) ? MOTOR_SPEED_UP : MOTOR_SPEED_DOWN;
    elev->motor_ccr = speed;  // 디버그용 저장
    if (elev->is_A) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);  // A호기: CH1
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);  // B호기: CH3
    }
}

void Motor_Stop(Elevator_t *elev) {
    elev->motor_ccr = MOTOR_STOP;  // 디버그용 저장
    if (elev->is_A) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_STOP);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MOTOR_STOP);
    }
}

void Door_Motor(Elevator_t *elev, int action) {
    uint16_t pos = (action == 1) ? SERVO_90_DEG : SERVO_0_DEG;
    elev->servo_ccr = pos;  // 디버그용 저장
    if (elev->is_A) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pos);  // A호기: CH2
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pos);  // B호기: CH4
    }
}

// 부드러운 문 동작 (압력 감지하면서 이동)
// return: true = 정상 완료, false = 압력 감지로 중단
bool Door_Motor_Smooth(Elevator_t *elev, int action) {
    uint16_t target_pos = (action == 1) ? SERVO_90_DEG : SERVO_0_DEG;  // 1=열림, 0=닫힘
    uint16_t current_pos = elev->servo_ccr;
    
    // 이동 방향 결정
    int step = (target_pos > current_pos) ? SERVO_STEP : -SERVO_STEP;
    
    // 천천히 이동하면서 압력 체크
    while (abs((int)current_pos - (int)target_pos) > SERVO_STEP) {
        current_pos += step;
        
        // 서보 위치 업데이트
        if (elev->is_A) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_pos);  // A호기: CH2
        } else {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, current_pos);  // B호기: CH4
        }
        elev->servo_ccr = current_pos;
        
        HAL_Delay(SERVO_DELAY);
        
        // 문 닫는 중에만 압력 체크
        if (action == 0) {
            // ADC 값 실시간 업데이트
            elev->doorPressure = Read_ADC_Channel(
                elev->is_A ? ADC_CHANNEL_4 : ADC_CHANNEL_5
            );
            
            // 압력 감지 시 즉시 중단
            if (elev->doorPressure > DOOR_PRESSURE_THRESHOLD) {
                elev->door_blocked = true;
                return false;  // 압력 감지로 중단
            }
        }
    }
    
    // 최종 위치로 이동
    if (elev->is_A) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, target_pos);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, target_pos);
    }
    elev->servo_ccr = target_pos;
    
    return true;  // 정상 완료
}

void delay_us(uint16_t us) { volatile uint32_t i = us * 10; while(i--); }
void play_sound(int frequency, int duration) {
    if (frequency == 0) return;
    long period_us = 1000000 / frequency;
    long cycles = (long)((double)duration * 1000 / period_us);
    for (long i = 0; i < cycles; i++) {
        HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET);
        delay_us(period_us / 2);
        HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);
        delay_us(period_us / 2);
    }
}
void play_open_sound(void) { play_sound(G5, 120); play_sound(C6, 300); }
void emergency_sound(void) { play_sound(C4, 300); HAL_Delay(50); play_sound(C4, 300); }
void arrive_sound(void) { play_sound(C6, 300); HAL_Delay(50); play_sound(G5, 300); }
void button_sound(void) { play_sound(E5, 100); }

// 엘리베이터의 최종 목적지 계산 (현재 방향으로 가장 먼 요청)
int Get_Final_Destination(Elevator_t *elev) {
    int final_dest = elev->cur_floor;
    
    if (elev->current_dir == UP) {
        // 상승 중: 가장 높은 요청 찾기
        for (int i = MAX_FLOOR; i >= elev->cur_floor; i--) {
            if (elev->Call_Up[i] || elev->Call_Down[i] || elev->Call_In[i]) {
                final_dest = i;
                break;
            }
        }
    } else if (elev->current_dir == DOWN) {
        // 하강 중: 가장 낮은 요청 찾기
        for (int i = 1; i <= elev->cur_floor; i++) {
            if (elev->Call_Up[i] || elev->Call_Down[i] || elev->Call_In[i]) {
                final_dest = i;
                break;
            }
        }
    }
    return final_dest;
}

// 요청을 경유할 수 있는지 체크 (현재 이동 방향에서)
bool Can_Serve_On_Way(Elevator_t *elev, int target_floor, int req_direction) {
    if (elev->state == STATE_IDLE) return true;  // IDLE이면 어디든 가능
    
    // 상승 중인 엘리베이터
    if (elev->current_dir == UP) {
        // 현재 위치보다 위에 있고, UP 요청이면 경유 가능
        if (target_floor > elev->cur_floor && req_direction == UP) {
            return true;
        }
    }
    // 하강 중인 엘리베이터  
    else if (elev->current_dir == DOWN) {
        // 현재 위치보다 아래에 있고, DOWN 요청이면 경유 가능
        if (target_floor < elev->cur_floor && req_direction == DOWN) {
            return true;
        }
    }
    return false;
}

// 요청까지의 실제 이동 거리 계산 (최종 목적지 경유 포함)
int Calculate_Real_Distance(Elevator_t *elev, int target_floor, int req_direction) {
    if (elev->state == STATE_IDLE) {
        // IDLE이면 직선 거리
        return abs(elev->cur_floor - target_floor);
    }
    
    // 경유 가능하면 직선 거리
    if (Can_Serve_On_Way(elev, target_floor, req_direction)) {
        return abs(elev->cur_floor - target_floor);
    }
    
    // 경유 불가능: 최종 목적지까지 갔다가 target까지
    int final_dest = Get_Final_Destination(elev);
    int dist_to_final = abs(elev->cur_floor - final_dest);
    int dist_final_to_target = abs(final_dest - target_floor);
    
    return dist_to_final + dist_final_to_target;
}

// 스마트 배차: 실제 이동 거리 기반 선택 (IDLE/MOVING 상관없이 동작)
Elevator_t* Find_Nearest_Elevator(int target_floor, int direction) {
    // 실제 이동 거리 비교 (경유 가능 여부 + 최종 목적지 고려)
    int dist_A = Calculate_Real_Distance(&elevator_A, target_floor, direction);
    int dist_B = Calculate_Real_Distance(&elevator_B, target_floor, direction);
    
    // 거리가 같으면 경유 가능한 쪽 우선
    if (dist_A == dist_B) {
        bool A_on_way = Can_Serve_On_Way(&elevator_A, target_floor, direction);
        bool B_on_way = Can_Serve_On_Way(&elevator_B, target_floor, direction);
        if (A_on_way && !B_on_way) return &elevator_A;
        if (B_on_way && !A_on_way) return &elevator_B;
    }
    
    return (dist_A <= dist_B) ? &elevator_A : &elevator_B;
}

char scan_keypad(void) {
    const char keymap[4][4] = { {'1', '2', '3', 'A'}, {'4', '5', '6', 'B'}, {'7', '8', '9', 'C'}, {'*', '0', '#', 'D'} };
    GPIO_TypeDef* col_ports[] = {KEY_C1_GPIO_Port, KEY_C2_GPIO_Port, KEY_C3_GPIO_Port, KEY_C4_GPIO_Port};
    uint16_t col_pins[] = {KEY_C1_Pin, KEY_C2_Pin, KEY_C3_Pin, KEY_C4_Pin};
    GPIO_TypeDef* row_ports[] = {KEY_R1_GPIO_Port, KEY_R2_GPIO_Port, KEY_R3_GPIO_Port, KEY_R4_GPIO_Port};
    uint16_t row_pins[] = {KEY_R1_Pin, KEY_R2_Pin, KEY_R3_Pin, KEY_R4_Pin};

    for(int c=0; c<4; c++) {
        HAL_GPIO_WritePin(col_ports[c], col_pins[c], GPIO_PIN_RESET);
        for(int r=0; r<4; r++) {
            if(HAL_GPIO_ReadPin(row_ports[r], row_pins[r]) == GPIO_PIN_RESET) {
                while(HAL_GPIO_ReadPin(row_ports[r], row_pins[r]) == GPIO_PIN_RESET);
                HAL_GPIO_WritePin(col_ports[c], col_pins[c], GPIO_PIN_SET);
                return keymap[r][c];
            }
        }
        HAL_GPIO_WritePin(col_ports[c], col_pins[c], GPIO_PIN_SET);
    }
    return 0;
}
void process_keypad_input(void) {
    char key = scan_keypad();
    if (key != 0) {
        button_sound();
        
        Elevator_t *nearest = NULL;
        switch(key) {
            // === [첫 열: 외부 호출 - 스마트 배차] ===
            case '1': // 외부 3층 Down
                nearest = Find_Nearest_Elevator(3, DOWN);
                nearest->Call_Down[3] = 1;
                break;
            case '4': // 외부 2층 Up
                nearest = Find_Nearest_Elevator(2, UP);
                nearest->Call_Up[2] = 1;
                break;
            case '7': // 외부 2층 Down
                nearest = Find_Nearest_Elevator(2, DOWN);
                nearest->Call_Down[2] = 1;
                break;
            case '*': // 외부 1층 Up
                nearest = Find_Nearest_Elevator(1, UP);
                nearest->Call_Up[1] = 1;
                break;

            // === [두번째 열: 1호기 내부] ===
            case '2': // 1호기 3층
                elevator_A.Call_In[3] = 1;
                break;
            case '5': // 1호기 2층
                elevator_A.Call_In[2] = 1;
                break;
            case '8': // 1호기 1층
                elevator_A.Call_In[1] = 1;
                break;
            case '0': // N/A
                break;

            // === [세번째 열: 2호기 내부] ===
            case '3': // 2호기 3층
                elevator_B.Call_In[3] = 1;
                break;
            case '6': // 2호기 2층
                elevator_B.Call_In[2] = 1;
                break;
            case '9': // 2호기 1층
                elevator_B.Call_In[1] = 1;
                break;
            case '#': // N/A
                break;

            // === [네번째 열: 문 제어] ===
            case 'A': // 1호기 문 닫힘
                if (elevator_A.state == STATE_DOOR_OPEN && !elevator_A.is_overloaded) {
                    elevator_A.state = STATE_DOOR_CLOSE;
                }
                break;
            case 'B': // 1호기 문 열림
                if (elevator_A.state == STATE_IDLE || elevator_A.state == STATE_DOOR_OPEN) {
                    elevator_A.state = STATE_DOOR_OPEN;
                    elevator_A.door_open_time = HAL_GetTick();
                    arrive_sound();
                }
                break;
            case 'C': // 2호기 문 닫힘
                if (elevator_B.state == STATE_DOOR_OPEN && !elevator_B.is_overloaded) {
                    elevator_B.state = STATE_DOOR_CLOSE;
                }
                break;
            case 'D': // 2호기 문 열림
                if (elevator_B.state == STATE_IDLE || elevator_B.state == STATE_DOOR_OPEN) {
                    elevator_B.state = STATE_DOOR_OPEN;
                    elevator_B.door_open_time = HAL_GetTick();
                    arrive_sound();
                }
                break;
            default: 
                break;
        }
        
        HAL_Delay(200); // 디바운싱 TODO 이동 중 키 입력해서 홀센서 무시되면 수정
    }
}

// --- 상태 머신 (구조체 기반) ---
void elevator_state_machine(Elevator_t *elev) {
    // 센서 감지 상태 업데이트
    elev->sensed_floor = Check_Hall_Sensor(elev);
    
    // 목표층 계산 (가장 가까운 요청)
    elev->target_floor = 0;
    if (elev->current_dir == UP) {
        for (int i = elev->cur_floor; i <= MAX_FLOOR; i++) {
            if (elev->Call_Up[i] || elev->Call_Down[i] || elev->Call_In[i]) {
                elev->target_floor = i;
                break;
            }
        }
    } else if (elev->current_dir == DOWN) {
        for (int i = elev->cur_floor; i >= 1; i--) {
            if (elev->Call_Up[i] || elev->Call_Down[i] || elev->Call_In[i]) {
                elev->target_floor = i;
                break;
            }
        }
    }
    
    switch(elev->state) {
        case STATE_IDLE:
            if (elev->Call_Up[elev->cur_floor] || elev->Call_Down[elev->cur_floor] || elev->Call_In[elev->cur_floor]) {
                elev->Call_Up[elev->cur_floor] = 0; 
                elev->Call_Down[elev->cur_floor] = 0; 
                elev->Call_In[elev->cur_floor] = 0;
                elev->target_floor = elev->cur_floor;
                elev->state = STATE_DOOR_OPEN;
                elev->door_open_time = HAL_GetTick();
                arrive_sound();
            }
            else if (Has_Requests_Above(elev, elev->cur_floor)) { 
                elev->current_dir = UP; 
                elev->state = STATE_MOVE_UP; 
            }
            else if (Has_Requests_Below(elev, elev->cur_floor)) { 
                elev->current_dir = DOWN; 
                elev->state = STATE_MOVE_DOWN; 
            }
            break;

        case STATE_MOVE_UP:
            Motor_Drive(elev, UP);
            if (elev->cur_floor >= MAX_FLOOR) { 
                Motor_Stop(elev); 
                elev->cur_floor = MAX_FLOOR; 
                elev->state = STATE_IDLE; 
                emergency_sound(); 
                break; 
            }
            
            // 홀센서 체크 (층이 바뀌었을 때만)
            if (elev->sensed_floor > 0 && elev->sensed_floor != elev->cur_floor) {
                elev->cur_floor = elev->sensed_floor;
                bool stop_flag = false;
                if (elev->Call_In[elev->cur_floor] || elev->Call_Up[elev->cur_floor] || 
                    (!Has_Requests_Above(elev, elev->cur_floor) && elev->Call_Down[elev->cur_floor])) 
                    stop_flag = true;
                if (stop_flag) {
                    Motor_Stop(elev);
                    elev->Call_In[elev->cur_floor] = 0; 
                    elev->Call_Up[elev->cur_floor] = 0;
                    if (!Has_Requests_Above(elev, elev->cur_floor)) { 
                        elev->Call_Down[elev->cur_floor] = 0; 
                        elev->current_dir = DOWN; 
                    }
                    elev->target_floor = elev->cur_floor;
                    elev->state = STATE_DOOR_OPEN;
                    elev->door_open_time = HAL_GetTick();
                    arrive_sound();
                }
            }
            break;

        case STATE_MOVE_DOWN:
            Motor_Drive(elev, DOWN);
            if (elev->cur_floor <= 1) { 
                Motor_Stop(elev); 
                elev->cur_floor = 1; 
                elev->state = STATE_IDLE; 
                emergency_sound(); 
                break; 
            }
            if (elev->sensed_floor > 0 && elev->sensed_floor != elev->cur_floor) {
                elev->cur_floor = elev->sensed_floor;
                bool stop_flag = false;
                if (elev->Call_In[elev->cur_floor] || elev->Call_Down[elev->cur_floor] || 
                    (!Has_Requests_Below(elev, elev->cur_floor) && elev->Call_Up[elev->cur_floor])) 
                    stop_flag = true;
                if (stop_flag) {
                    Motor_Stop(elev);
                    elev->Call_In[elev->cur_floor] = 0; 
                    elev->Call_Down[elev->cur_floor] = 0;
                    if (!Has_Requests_Below(elev, elev->cur_floor)) { 
                        elev->Call_Up[elev->cur_floor] = 0; 
                        elev->current_dir = UP; 
                    }
                    elev->target_floor = elev->cur_floor;
                    elev->state = STATE_DOOR_OPEN;
                    elev->door_open_time = HAL_GetTick();
                    arrive_sound();
                }
            }
            break;

        case STATE_DOOR_OPEN:
            Door_Motor_Smooth(elev, 1);  // 부드럽게 열기
            
            // 과적 감지
            if (elev->fsrValue > OVERLOAD_THRESHOLD) {
                elev->is_overloaded = true;
                elev->door_open_time = HAL_GetTick();
                emergency_sound();
            } else {
                elev->is_overloaded = false;
            }
            
            if (!elev->is_overloaded && (HAL_GetTick() - elev->door_open_time >= DOOR_WAIT_TIME)) {
                elev->state = STATE_DOOR_CLOSE;
            }
            break;

        case STATE_DOOR_CLOSE:
            // 부드럽게 닫으면서 압력 감지
            if (!Door_Motor_Smooth(elev, 0)) {
                // 압력 감지로 중단됨 (Door_Motor_Smooth가 false 반환)
                elev->door_blocked = true;
                elev->state = STATE_DOOR_OPEN;
                elev->door_open_time = HAL_GetTick();
                emergency_sound();  // 경고음
                break;
            }
            
            // 문이 정상적으로 닫혔음
            elev->door_blocked = false;
            
            // 현재 층에 새 요청 발생 시 문 다시 열기
            if (elev->Call_In[elev->cur_floor] || elev->Call_Up[elev->cur_floor] || elev->Call_Down[elev->cur_floor]) {
                elev->Call_In[elev->cur_floor] = 0; 
                elev->Call_Up[elev->cur_floor] = 0; 
                elev->Call_Down[elev->cur_floor] = 0;
                elev->state = STATE_DOOR_OPEN;
                elev->door_open_time = HAL_GetTick();
                break;
            }
            
            // 다음 동작 결정
            if (elev->current_dir == UP) {
                if (Has_Requests_Above(elev, elev->cur_floor)) elev->state = STATE_MOVE_UP;
                else if (Has_Requests_Below(elev, elev->cur_floor)) { 
                    elev->current_dir = DOWN; 
                    elev->state = STATE_MOVE_DOWN; 
                }
                else { elev->current_dir = STOP; elev->state = STATE_IDLE; }
            }
            else if (elev->current_dir == DOWN) {
                if (Has_Requests_Below(elev, elev->cur_floor)) elev->state = STATE_MOVE_DOWN;
                else if (Has_Requests_Above(elev, elev->cur_floor)) { 
                    elev->current_dir = UP; 
                    elev->state = STATE_MOVE_UP; 
                }
                else { elev->current_dir = STOP; elev->state = STATE_IDLE; }
            }
            else {
                if (Has_Requests_Above(elev, elev->cur_floor)) { 
                    elev->current_dir = UP; 
                    elev->state = STATE_MOVE_UP; 
                }
                else if (Has_Requests_Below(elev, elev->cur_floor)) { 
                    elev->current_dir = DOWN; 
                    elev->state = STATE_MOVE_DOWN; 
                }
                else elev->state = STATE_IDLE;
            }
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // PWM 시작 (4채널 모두)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // A호기 모터
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // A호기 서보
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // B호기 모터
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // B호기 서보
  
  // ADC Calibration
  HAL_ADCEx_Calibration_Start(&hadc1);

  LCD_Init();
  LCD_SetCursor(0, 0);
  LCD_Print("Dual Elevator");
  HAL_Delay(1000);

  // 엘리베이터 초기화
  Init_Elevator_A();
  Init_Elevator_B();
  
  // 초기 PWM 설정
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_STOP);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_0_DEG);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MOTOR_STOP);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, SERVO_0_DEG);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. ADC 채널 하나씩 읽기 (각 채널을 순차적으로)
    elevator_A.fsrValue = Read_ADC_Channel(ADC_CHANNEL_0);      // ADC1_IN0 (PA0)
    elevator_B.fsrValue = Read_ADC_Channel(ADC_CHANNEL_1);      // ADC1_IN1 (PA1)
    elevator_A.doorPressure = Read_ADC_Channel(ADC_CHANNEL_4);  // ADC1_IN4 (PA4)
    elevator_B.doorPressure = Read_ADC_Channel(ADC_CHANNEL_5);  // ADC1_IN5 (PA5)

    // 2. LCD 갱신 (2대 동시)
    Update_LCD_Status_Dual(&elevator_A, &elevator_B);

    // 3. 키패드 입력 처리
    process_keypad_input();

    // 4. 두 엘리베이터 상태머신 실행
    elevator_state_machine(&elevator_A);
    elevator_state_machine(&elevator_B);

    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  htim1.Init.Period = 19999;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, KEY_C1_Pin|KEY_C2_Pin|KEY_C3_Pin|KEY_C4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_R1_Pin KEY_R2_Pin KEY_R3_Pin KEY_R4_Pin */
  GPIO_InitStruct.Pin = KEY_R1_Pin|KEY_R2_Pin|KEY_R3_Pin|KEY_R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_C1_Pin KEY_C2_Pin KEY_C3_Pin KEY_C4_Pin */
  GPIO_InitStruct.Pin = KEY_C1_Pin|KEY_C2_Pin|KEY_C3_Pin|KEY_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR2_1F_Pin SENSOR2_2F_Pin SENSOR2_3F_Pin SENSOR_1F_Pin
                           SENSOR_2F_Pin SENSOR_3F_Pin */
  GPIO_InitStruct.Pin = SENSOR2_1F_Pin|SENSOR2_2F_Pin|SENSOR2_3F_Pin|SENSOR_1F_Pin
                          |SENSOR_2F_Pin|SENSOR_3F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPEAKER_Pin */
  GPIO_InitStruct.Pin = SPEAKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPEAKER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
