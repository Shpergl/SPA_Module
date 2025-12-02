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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PARKING_BUTTON_PIN 	GPIO_PIN_1
#define PARKING_RADAR_PIN 	GPIO_PIN_2
#define RIGHT_CAMERA_PIN 	GPIO_PIN_3
#define LEFT_CAMERA_PIN 	GPIO_PIN_4
#define FRONT_CAMERA_PIN	GPIO_PIN_5
#define REAR_CAMERA_PIN 	GPIO_PIN_7
/* GPIO B */
#define VIDEO_OUTPUT_PIN 	GPIO_PIN_0

#define RADAR_START_PULSE 			1880	// ms
#define RADAR_HIGH_PULSE 			200		// ms
#define RADAR_LOW_PULSE 			100		// ms
#define RADAR_PULSE_DEVIATION 		20		// ms
#define RADAR_PULSES_NUMBER 		32
#define RADAR_PACKET_LENGTH 		8
#define RADAR_DISTANCE_MULTIPLIER 	10

#define FRONT_SPEED_LIMIT 		5
#define FRONT_DISTANCE_LIMIT	50

#define DISTANCE_LIMIT_1 30		// cm
#define DISTANCE_LIMIT_2 60		// cm
#define DISTANCE_LIMIT_3 100	// cm
#define DISTANCE_LIMIT_4 200	// cm

#define DISTANCE_LIMIT_1_TIMEOUT 100	// mS
#define DISTANCE_LIMIT_2_TIMEOUT 200	// mS
#define DISTANCE_LIMIT_3_TIMEOUT 400	// mS
#define DISTANCE_LIMIT_4_TIMEOUT 800	// mS

#define RIGHT_CAMERA_INDEX	0
#define LEFT_CAMERA_INDEX	1
#define FRONT_CAMERA_INDEX	2
#define REAR_CAMERA_INDEX	3

#define PARKING_BUTTON_LONG_PRESS_TIME	 1000	// mS
#define PARKING_BUTTON_SHORT_PRESS_TIME	 10		// mS

#define CAN_DATA_LENGTH 			8
#define CAN_SEND_START_TIMEOUT 		200		// mS
#define CAN_SPA_STATUS_FRAME_ID 	0x500
#define CAN_SID_BEEP_FRAME_ID 		0x430
#define CAN_TURN_LIGHT_FRAME_ID 	0x4a0
#define CAN_SPEED_RPM_FRAME_ID 		0x460
#define CAN_REVERSE_GEAR_FRAME_ID 	0x280
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
# define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)
# define CHECK_RADAR_OFF (parking_camera_state == 0 && front_parking_radar == 0)
# define CHECK_RADAR_ON (parking_camera_state != 0 || front_parking_radar != 0)
# define CHECK_F_R_VIDEO_OFF (!CHECK_BIT(parking_camera_state, FRONT_CAMERA_INDEX) && !CHECK_BIT(parking_camera_state, REAR_CAMERA_INDEX))
# define CHECK_F_OR_R_VIDEO (CHECK_BIT(parking_camera_state, FRONT_CAMERA_INDEX) || CHECK_BIT(parking_camera_state, REAR_CAMERA_INDEX))
# define SET_INDEX_BIT(var, pos) (var |= 1 << pos)
# define CLEAR_INDEX_BIT(var, pos) (var &= ~(1 << pos))
# define IS_HIGH_PULSE(var) (RADAR_HIGH_PULSE + RADAR_PULSE_DEVIATION >= var && RADAR_HIGH_PULSE - RADAR_PULSE_DEVIATION <= var)
# define IS_LOW_PULSE(var) (RADAR_LOW_PULSE + RADAR_PULSE_DEVIATION >= var && RADAR_LOW_PULSE - RADAR_PULSE_DEVIATION <= var)
# define IS_START_PULSE(var) (RADAR_START_PULSE + RADAR_PULSE_DEVIATION >= var && RADAR_START_PULSE - RADAR_PULSE_DEVIATION <= var)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t             RxData[8];
uint32_t            TxMailbox;

uint32_t 			front_sensors[4];
uint32_t		    rear_sensors[4];
int16_t				front_min_distance = 255;
int16_t				rear_min_distance = 255;

int8_t				start_radar = 0;
int8_t				notification = 0;
int8_t				enable_notification = 1;
int8_t				left_turn = 0;
int8_t				right_turn = 0;
int8_t				send_can_status = 0;
int8_t				parking_camera_state = 0;
int8_t				front_parking_radar = 0;

volatile uint8_t sleep_mode_activated = 0;
volatile uint8_t enter_sleep_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HandleFrontRadarPulses(TIM_HandleTypeDef *htim)
{
	static 	int8_t 		index = RADAR_PULSES_NUMBER;
	static 	int8_t 		start_cpatured = 0;
	static 	uint32_t	captured_pulses = 0;
			uint32_t 	pulse = 0;
			uint16_t 	packet = 0;

	pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

	if (IS_START_PULSE(pulse))
	{
		start_cpatured = 1;
		index = RADAR_PULSES_NUMBER;
		return;
	}
	if (start_cpatured){
		if (IS_HIGH_PULSE(pulse)) captured_pulses |= (1 << index);

		if (index == 0) {
			front_min_distance = 255;
			for (int i=0; i<4; i+=1) {
				packet = (captured_pulses & (255 << (RADAR_PACKET_LENGTH * i))) >> (RADAR_PACKET_LENGTH * i);
				front_sensors[i] = packet;
				if (packet < front_min_distance) front_min_distance = packet;
			}
			captured_pulses = 0;
			start_cpatured = 0;
		}
		index --;
	}
}

void HandleRearRadarPulses(TIM_HandleTypeDef *htim)
{
	static int8_t 		index = RADAR_PULSES_NUMBER;
	static int8_t 		start_cpatured = 0;
	static uint32_t 	captured_pulses = 0;
	uint32_t 			pulse = 0;
	uint16_t 			packet = 0;

	pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

	if (IS_START_PULSE(pulse))
	{
		start_cpatured = 1;
		index = RADAR_PULSES_NUMBER;
		return;
	}
	if (start_cpatured){
		if (IS_HIGH_PULSE(pulse)) captured_pulses |= (1 << index);

		if (index == 0) {
			rear_min_distance = 255;
			for (int i=0; i<4; i+=1) {
				packet = (captured_pulses & (255 << (RADAR_PACKET_LENGTH * i))) >> (RADAR_PACKET_LENGTH * i);
				rear_sensors[i] = packet;
				if (packet < rear_min_distance) rear_min_distance = packet;
			}
			captured_pulses = 0;
			start_cpatured = 0;
		}
		index --;
	}
}

void can_send_sensors(){

	if (notification) {
		static uint8_t TxData[8];
		TxHeader.StdId = CAN_SPA_STATUS_FRAME_ID;
		TxData[0] = rear_sensors[0];
		TxData[1] = rear_sensors[1];
		TxData[2] = rear_sensors[2];
		TxData[3] = rear_sensors[3];
		TxData[4] = front_sensors[0];
		TxData[5] = front_sensors[1];
		TxData[6] = front_sensors[2];
		TxData[7] = front_sensors[3];
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
	}
}


void send_long_beep_distance(){
	static uint8_t TxData[8];
		TxHeader.StdId = CAN_SID_BEEP_FRAME_ID;
		TxData[2] |= 1 << 7;
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
}

void send_beep_distance(){
	static uint8_t TxData[8];
		TxHeader.StdId = CAN_SID_BEEP_FRAME_ID;
		TxData[1] |= 1 << 4;
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}
}

uint16_t get_min_distance()
{
	uint16_t min_distance = 255;
	for (int i=0; i<3; i++)
	{
		if (front_sensors[i] < min_distance) min_distance = front_sensors[i];
		if (rear_sensors[i] < min_distance) min_distance = rear_sensors[i];
	}
	return min_distance;
}

uint16_t get_front_min_distance()
{
	uint16_t min_distance = 255;
	for (int i=0; i<3; i++){
			if (front_sensors[i] < min_distance) min_distance = front_sensors[i];
		}
	return min_distance;
}

void SendParkingSound()
{
	static uint32_t last_time = 0;

	if (notification == 0) return;

	switch(get_min_distance() * RADAR_DISTANCE_MULTIPLIER)
	{
	case 0 ... DISTANCE_LIMIT_1:
	{
		if ((HAL_GetTick() - last_time >= DISTANCE_LIMIT_1_TIMEOUT) || last_time == 0)
		{
			last_time = HAL_GetTick();
			send_beep_distance();
		}
	}
	break;
	case (DISTANCE_LIMIT_1 + 1) ... DISTANCE_LIMIT_2:
	{
		if ((HAL_GetTick() - last_time >= DISTANCE_LIMIT_2_TIMEOUT) || last_time == 0)
		{
			last_time = HAL_GetTick();
			send_beep_distance();
		}
	}
	break;
	case (DISTANCE_LIMIT_2 + 1) ... DISTANCE_LIMIT_3:
	{
		if ((HAL_GetTick() - last_time >= DISTANCE_LIMIT_3_TIMEOUT) || last_time == 0)
		{
			last_time = HAL_GetTick();
			send_beep_distance();
		}
	}
	break;
	case (DISTANCE_LIMIT_3 + 1) ... DISTANCE_LIMIT_4:
	{
		if ((HAL_GetTick() - last_time >= DISTANCE_LIMIT_4_TIMEOUT) || last_time == 0)
		{
			last_time = HAL_GetTick();
			send_beep_distance();
		}
	}
	break;
	}
}

void HandleCameraState(int16_t state)
{
	HAL_GPIO_WritePin(GPIOA, RIGHT_CAMERA_PIN,
			CHECK_BIT(state, RIGHT_CAMERA_INDEX) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LEFT_CAMERA_PIN,
			CHECK_BIT(state, LEFT_CAMERA_INDEX) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, FRONT_CAMERA_PIN,
			CHECK_BIT(state, FRONT_CAMERA_INDEX) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, REAR_CAMERA_PIN,
			CHECK_BIT(state, REAR_CAMERA_INDEX) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, VIDEO_OUTPUT_PIN, state != 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void StartRadar(){
	start_radar = 1;
	HAL_GPIO_WritePin(GPIOA, PARKING_RADAR_PIN, GPIO_PIN_RESET);
}

void StopRadar(){
	start_radar = 0;
	notification = 0;
	send_can_status = 0;
	for (int i=0; i<3; i++){
		front_sensors[i] = 255;
		rear_sensors[i] = 255;
	}
	front_min_distance = 255;
	rear_min_distance = 255;
	HAL_GPIO_WritePin(GPIOA, PARKING_RADAR_PIN, GPIO_PIN_SET);
}

void HandleRadarState(){
	if (CHECK_RADAR_ON) StartRadar();
	else if (CHECK_RADAR_OFF) StopRadar();
}

void StopRightTurn() {
	CLEAR_INDEX_BIT(parking_camera_state, RIGHT_CAMERA_INDEX);
	HandleCameraState(parking_camera_state);
}

void HandleRightTurnCountdown(int32_t stop_time){
	static int32_t last_stop_triggered = 0;

	if (stop_time != 0) last_stop_triggered = stop_time;
	if (right_turn == 1) last_stop_triggered = 0;

	if (last_stop_triggered != 0 && ((HAL_GetTick() - last_stop_triggered > 300) || left_turn == 1)) {
		StopRightTurn();
		last_stop_triggered = 0;
	}
}

void HandleRightTurn(uint8_t left, uint8_t right){

	if (left == 1 && right == 1) return;

	if (right == 1 && CHECK_F_OR_R_VIDEO){
		if (right_turn == 0){
			right_turn = 1;
			SET_INDEX_BIT(parking_camera_state, RIGHT_CAMERA_INDEX);
			HandleCameraState(parking_camera_state);
		}
	}
	else {
		if (right_turn == 1) {
			right_turn = 0;
			if (CHECK_F_R_VIDEO_OFF){
				StopRightTurn();
			}
			else HandleRightTurnCountdown(HAL_GetTick());
		}
	}
}

void StopLeftTurn() {
	CLEAR_INDEX_BIT(parking_camera_state, LEFT_CAMERA_INDEX);
	HandleCameraState(parking_camera_state);
}

void HandleLeftTurnCountdown(int32_t stop_time){
	static int32_t last_stop_triggered = 0;

	if (stop_time != 0) last_stop_triggered = stop_time;
	if (left_turn == 1) last_stop_triggered = 0;

	if (last_stop_triggered != 0 && ((HAL_GetTick() - last_stop_triggered > 300) || right_turn == 1)) {
		left_turn = 0;
		StopLeftTurn();
		last_stop_triggered = 0;
	}
}

void HandleLeftTurn(uint8_t left, uint8_t right){

	if (left == 1 && right == 1) return;

	if (left == 1 && CHECK_F_OR_R_VIDEO){
		if (left_turn == 0){
			left_turn = 1;
			SET_INDEX_BIT(parking_camera_state, LEFT_CAMERA_INDEX);
			HandleCameraState(parking_camera_state);
		}
	}
	else {
		if (left_turn == 1) {
			left_turn = 0;
			if (CHECK_F_R_VIDEO_OFF){
				StopLeftTurn();
			}
			else HandleLeftTurnCountdown(HAL_GetTick());
		}
	}
}

void HandleRearGear(int8_t rear_gear)
{
	if (rear_gear == 1 && !CHECK_BIT(parking_camera_state, REAR_CAMERA_INDEX)) {
		SET_INDEX_BIT(parking_camera_state, REAR_CAMERA_INDEX);
		notification = 1;
		HandleCameraState(parking_camera_state);
	}
	else if (rear_gear == 0 && CHECK_BIT(parking_camera_state, REAR_CAMERA_INDEX)) {
		CLEAR_INDEX_BIT(parking_camera_state, REAR_CAMERA_INDEX);
		HandleCameraState(parking_camera_state);
	}
}

void HandleFrontSpeed(uint32_t speed)
{
	if (speed == 0){
		 front_parking_radar = 0;
		 return;
	}

	if (speed <= FRONT_SPEED_LIMIT){
		if (CHECK_BIT(parking_camera_state, FRONT_CAMERA_INDEX)) return;

		front_parking_radar = 1;
		if (enable_notification){
			if (get_front_min_distance() * RADAR_DISTANCE_MULTIPLIER <= FRONT_DISTANCE_LIMIT){
				notification = 1;
			} else {
				notification = 0;
			}
		}
	} else {
		front_parking_radar = 0;
		notification = 0;
		if (CHECK_BIT(parking_camera_state, FRONT_CAMERA_INDEX)){
			CLEAR_INDEX_BIT(parking_camera_state, FRONT_CAMERA_INDEX);
			HandleCameraState(parking_camera_state);
			front_parking_radar = 0;
		}
	}
}

void HandleParkingButton(void) {
	static int32_t last_btn_pressed = 0;
	static int8_t button_processed = 0;

	if (HAL_GPIO_ReadPin(GPIOA, PARKING_BUTTON_PIN) == 0){
		if (button_processed) return;
		if (last_btn_pressed == 0) last_btn_pressed = HAL_GetTick();
		if (HAL_GetTick() - last_btn_pressed >= PARKING_BUTTON_LONG_PRESS_TIME){
			 if (front_parking_radar == 1 && notification == 1){
				 enable_notification = 0;
				 notification = 0;
			 }
			 else if (front_parking_radar == 1 && notification == 0) {
				 enable_notification = 1;
				 notification = 1;
			 }
			 last_btn_pressed = 0;
			 button_processed = 1;
		}
	}
	else {
		if (button_processed) button_processed = 0;
		if (last_btn_pressed == 0) return;
		if (HAL_GetTick() - last_btn_pressed >= PARKING_BUTTON_SHORT_PRESS_TIME &&
				HAL_GetTick() - last_btn_pressed < PARKING_BUTTON_LONG_PRESS_TIME){
			if (CHECK_BIT(parking_camera_state, FRONT_CAMERA_INDEX)){
				CLEAR_INDEX_BIT(parking_camera_state, FRONT_CAMERA_INDEX);
				HandleCameraState(parking_camera_state);
				notification = 0;
			}
			else {
				SET_INDEX_BIT(parking_camera_state, FRONT_CAMERA_INDEX);
				HandleCameraState(parking_camera_state);
				notification = 1;
			}
		}
		last_btn_pressed = 0;
	}
}
uint8_t current_radar_idx = -1;
uint8_t front_pulse_captured = 0, rear_pulse_captured = 0;

uint8_t front_start_captured = 0;
void HandleFrontPulses(TIM_HandleTypeDef *htim){
	static uint32_t IC_val1_front = 0, IC_val2_front= 0;
	uint32_t IC_diff_front = 0;

	if (front_pulse_captured) return;

	if (!front_start_captured){
		IC_val1_front = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		front_start_captured = 1;
	}
	else {
		IC_val2_front = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		IC_diff_front = IC_val2_front - IC_val1_front;

		front_sensors[current_radar_idx] = (IC_diff_front * 173)/ 100000;
		front_start_captured = 0;
		front_pulse_captured = 1;
	}

}

uint8_t rear_start_captured = 0;
void HandleRearPulses(TIM_HandleTypeDef *htim){
	static uint32_t IC_val1_rear = 0, IC_val2_rear= 0;
	uint32_t IC_diff_rear = 0;

	if (rear_pulse_captured) return;

	if (!rear_start_captured){
		IC_val1_rear = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		rear_start_captured = 1;
	}
	else {
		IC_val2_rear = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		IC_diff_rear = IC_val2_rear - IC_val1_rear;

		rear_sensors[current_radar_idx] = (IC_diff_rear * 173)/ 100000;
		rear_start_captured = 0;
		rear_pulse_captured = 1;
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (!CHECK_RADAR_ON) return;

	if(htim -> Instance == TIM2 && htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		HandleFrontPulses(htim);
	}
	if(htim -> Instance == TIM2 && htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		HandleRearPulses(htim);
	}
}

uint32_t front_max_distance_counter[4], rear_max_distance_counter[4];

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	static	uint32_t	 start_time = 0;
	if (!CHECK_RADAR_ON) return;

	if (htim->Instance == TIM3){
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			current_radar_idx ++;
			if (current_radar_idx > 3) current_radar_idx = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, CHECK_BIT(current_radar_idx, 0)); //A
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, CHECK_BIT(current_radar_idx, 1));	//B

			if (!front_pulse_captured){
				if (front_sensors[current_radar_idx] != 255) {
					front_max_distance_counter[current_radar_idx] ++;
				}
				if (front_max_distance_counter[current_radar_idx] == 10) {
					front_sensors[current_radar_idx] = 255;
					front_max_distance_counter[current_radar_idx] = 0;
				}
			}
			front_start_captured = 0;
			front_pulse_captured = 0;


			TIM1->CCR3 = 450-1;
			TIM1->CCR1 = 65535;
			htim1.Instance->CR1 |= TIM_CR1_CEN; // Enable Counter

		}

		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (!rear_pulse_captured){
				if (rear_sensors[current_radar_idx] != 255){
					rear_max_distance_counter[current_radar_idx] ++;
				}
				if (rear_max_distance_counter[current_radar_idx] == 10) {
					rear_sensors[current_radar_idx] = 255;
					rear_max_distance_counter[current_radar_idx] = 0;
				}
			}
			rear_start_captured = 0;
			rear_pulse_captured = 0;
			TIM1->CCR3 = 65535;
			TIM1->CCR1 = 450-1;
			htim1.Instance->CR1 |= TIM_CR1_CEN; // Enable Counter
		}
	}

	if (htim->Instance == TIM4){
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
//			SendParkingSound(); // 50 mS
		}
		if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{

			SendParkingSound();
			if (send_can_status) can_send_sensors(); // 100 mS

			/* Wait timeout to proper enter parking mode on Head Unit */
			else {
				if (start_time == 0) start_time = HAL_GetTick();

				if (HAL_GetTick() - start_time > CAN_SEND_START_TIMEOUT) {
					send_can_status = 1;
					start_time = 0;
				}
			}
		}
	}
}

void EnterSleepMode(){
	enter_sleep_flag = 0;
	sleep_mode_activated = 1;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); // CAN off
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// if rising edge
	if(SleepMode_Activated == 1)
	{
		// CPU Has Exited From Sleep Mode, Resume The SysTick!
		HAL_ResumeTick();
		SleepMode_Activated = 0;
	}

	// iuf falling edge
	else
	{
		// Re-Enter Sleep Mode!
		EnterSleepFlag = 1;
	}

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}

	if (RxHeader.IDE != CAN_ID_STD || RxHeader.DLC != CAN_DATA_LENGTH) return;

	switch(RxHeader.StdId)
	{
	case CAN_TURN_LIGHT_FRAME_ID:
	{
		if (CHECK_BIT(RxData[0], 7)){
			HandleRightTurn(CHECK_BIT(RxData[2], 5), CHECK_BIT(RxData[2], 6));
			HandleLeftTurn(CHECK_BIT(RxData[2], 5), CHECK_BIT(RxData[2], 6));
		}
	}
	break;
	case CAN_SPEED_RPM_FRAME_ID:
	{
		HandleFrontSpeed((((uint16_t)RxData[3] << 8) | (uint8_t)RxData[4]) / 10);
	}
	break;
	case CAN_REVERSE_GEAR_FRAME_ID:
	{
		if (RxData[1] == 0x02) // Rear gear selected
		{
			HandleRearGear(1);
		}
		if (RxData[1] == 0xff) // Rear gear cleared
		{
			HandleRearGear(0);
		}
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  EnterSleepMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(enter_sleep_flag) EnterSleepMode();
	  HandleRadarState();
	  HandleParkingButton();
	  HandleLeftTurnCountdown(0);
	  HandleRightTurnCountdown(0);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 54;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* Configure the CAN Filter */
   sFilterConfig.FilterBank = 0;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
   sFilterConfig.FilterIdHigh = 0x460<<5;
   sFilterConfig.FilterIdLow = 0x280<<5;
   sFilterConfig.FilterMaskIdHigh = 0;
   sFilterConfig.FilterMaskIdLow = 0;
   sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;

   if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
   }

   sFilterConfig.FilterBank = 1;
   sFilterConfig.FilterIdHigh = 0x4a0<<5;
   sFilterConfig.FilterIdLow = 0x430<<5;
   sFilterConfig.FilterMaskIdHigh = 0;
   sFilterConfig.FilterMaskIdLow = 0;

   if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
   }


  /* Start the CAN peripheral */
   if (HAL_CAN_Start(&hcan) != HAL_OK)
   {
     /* Start Error */
     Error_Handler();
   }

   /* Activate CAN RX notification */
   if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
   {
	/* Notification Error */
	Error_Handler();
   }
   /* Configure Transmission process */
   TxHeader.IDE = CAN_ID_STD;
   TxHeader.RTR = CAN_RTR_DATA;
   TxHeader.DLC = CAN_DATA_LENGTH;
  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 2-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 900-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 16-1;
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
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
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
  sConfigOC.Pulse = 450-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000-1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 10000-1;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 9000-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 5000-1;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10000-1;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);
  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Parking_Radar_Pin|Right_Camera_Pin|Left_Camera_Pin|Front_Camera_Pin
                          |Rear_Camera_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAN_On_Pin|B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Video_Output_GPIO_Port, Video_Output_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Parking_Button_Pin */
  GPIO_InitStruct.Pin = Parking_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Parking_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Parking_Radar_Pin Right_Camera_Pin Left_Camera_Pin Front_Camera_Pin
                           Rear_Camera_Pin */
  GPIO_InitStruct.Pin = Parking_Radar_Pin|Right_Camera_Pin|Left_Camera_Pin|Front_Camera_Pin
                          |Rear_Camera_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_On_Pin */
  GPIO_InitStruct.Pin = CAN_On_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_On_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Video_Output_Pin */
  GPIO_InitStruct.Pin = Video_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Video_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Power_On_Pin */
  GPIO_InitStruct.Pin = Power_On_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Power_On_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A_Pin */
  GPIO_InitStruct.Pin = A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_Pin */
  GPIO_InitStruct.Pin = B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(B_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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

#ifdef  USE_FULL_ASSERT
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
