/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "string.h"
#include "EEPROM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

uint16_t digitalStates[80];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DEV_ADDR 0xa0

//epromun islekliyin yoxlamq ucun
//int dataw3[2] = {1, 2};
//int datar3[2] = {0, 0};
int dataw3 = 31;
int datar3;
float alarmLevelWrite[38] = {5.92, 5.92, 1.62, 1.62, 1.08, 5.24, 5.24, 1.03, 1.03, 1.08, 1.8, 1.8, 2.7, 2.7, 3.42, 1.35, 2.07, 1.08, 1.08, 0.9, 1.8, 1.8, 3.96, 3.72, 6.12, 6.12, 4.37, 4.37, 6.12, 6.12, 6.12, 5.31, 5.31, 5.27, 5.27, 2.61, 2.61, 4};
float alarmLevelRead[38];

float alarmLevel[38];

int i2_t = 0;

int delayTime = 5;

int sendCountCheck[60];
int catacaqSay=30;


uint16_t message[62];
uint16_t analog[62];

enum alarm_state {
	resetAlarm,
	notResetAlarm
};

int tamHisse = 0, kesirHisse = 0;
int say=0;

////////////////////mux u saydir/////////////////////////////////////////////////////////////////////////////////
void mux(int sel) {

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, (0b0001 & (sel >> 0)));
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (0b0001 & (sel >> 1)));
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, (0b0001 & (sel >> 2)));
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (0b0001 & (sel >> 3)));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////// muxdaki adc leri oxu///////////////////////////////////////////////////////
/*
 * muxdaki adc ve dig leri oxu
 */
void check_channels(int sel) {
	int sampleRate = 100;

	HAL_ADC_Start(&hadc3);
	HAL_ADC_PollForConversion(&hadc3, 1000);
	float sum = 0, average = 0;
	for (int t = 0; t < sampleRate; t++) {
		sum = sum + HAL_ADC_GetValue(&hadc3);
	}
	average = sum / sampleRate;
	message[sel] = average;
	sum = 0;
	average = 0;
	HAL_ADC_Stop(&hadc3);
//
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	for (int t = 0; t < sampleRate; t++) {
		sum = sum + HAL_ADC_GetValue(&hadc2);
	}
	average = sum / sampleRate;
	message[sel + 16] = average;
	sum = 0;
	average = 0;
	HAL_ADC_Stop(&hadc2);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	for (int t = 0; t < sampleRate; t++) {
		sum = sum + HAL_ADC_GetValue(&hadc1);
	}
	average = sum / sampleRate;
	message[sel + 32] = average;
	sum = 0;
	average = 0;
	HAL_ADC_Stop(&hadc1);
}

//////////////////////////id deyerleri////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t id[30] = { 0x300, 0x301, 0x302, 0x303, 0x304, 0x305, 0x306, 0x307, 0x308, 0x309, 0x30A, 0x30B, 0x30C, 0x30D, 0x30E, 0x30F, 0x310, 0x311, 0x312, 0x313, 0x314, 0x315, 0x350 , 0x603, 0x657, 0x655, 0x655, 0x655, 0x655, 0x655};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CAN_TxHeaderTypeDef TxHeader[40];

CAN_RxHeaderTypeDef RxHeader;

int datacheck = 0;
uint8_t pk1 = 0;

uint8_t TxData[40][8];

uint8_t RxData[8];

uint32_t TxMailbox;

/*******	config  ***********/
//uint16_t contactState[63] = 		{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};			//normal open is 0 , normal close 1
uint16_t contactState[63];
uint16_t contactStateTot[5] = {0,0,0,0,0};
uint16_t contactStateRead[5] = {0,0,0,0,0};
uint16_t contactStateTest[71];
uint16_t delaySeconds[64] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };	//signal cixdiqdan sonra neqeder gozleyecek
uint16_t delaySecondsTot[40];
uint16_t delaySecondsTotRead[40];
uint16_t digitalInputId[63] = { 3065, 3066, 3067, 3068, 3069, 3070, 3071, 3072, 3073, 3074, 3075, 3076, 3077, 3078, 3079, 3080, 3081, 3082, 3083, 3084, 3085, 3086, 3087, 3088, 3089, 3090, 3091, 3092, 3093, 3094, 3095, 3096, 3097, 3098, 3099, 3100, 3101, 3102, 3103, 3104, 3105, 3106, 3107, 3108, 3109, 3110, 3111, 3112, 3053, 3054, 3055, 3056, 3057, 3058, 3059, 3060, 3061, 3062, 3063, 3064, 3001, 3002, 3003 };				//signal id leri
uint8_t fadeOut[63] = 		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t fadeOutBaxmaq[63] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t fadeOutTot[12] = 			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t fadeOutTotRead[12] = 		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t fadeOutTotReadTest[12] = 	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //

int fadeOutReg = 0;

uint16_t delaySecondsCount[63] = 		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //delay saniye sayici

uint16_t delaySecondsCountForOff[63] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };		//alarmi sondurmek icin delay saniye sayici

uint16_t alarmOn[63] = 					{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t alarmOnAnalog[40] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t alarmCount[63] = 				{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // digital alarm sayicisi
uint16_t waitingForDelay[63] = 			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // delay ucun gozleme registeri

uint16_t analogInputID[38] = { 3005, 3006, 3007, 3008, 3009, 3010, 3011, 3012, 3013, 3014, 3015, 3016, 3017, 3018, 3019, 3020, 3021, 3022, 3023, 3024, 3025, 3026, 3027, 3028, 3029, 3030, 3031, 3032, 3033, 3034, 3035, 3036, 3037, 3038, 3039, 3040, 3041, 3042 }; // analog ID

uint8_t secondByte[38] = 	 		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // analog fade outlari tutmaq ucun

uint16_t analogAlarmCount[38] = 	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // analog alarm sayicilarin tutmaq ucun
uint16_t analogAlarmCountDown[38] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // analog alarm sayicilarin tutmaq ucun

uint8_t analogFadeOut[38] = 		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // analog fade outlari tutmaq ucun
uint8_t analogFadeOutBaxmaq[38] = 	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // analog fade outlari tutmaq ucun

uint16_t analogFadeOutTot[12] = 		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t analogFadeOutTotReadTest[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t analogFadeOutTotRead[12] = 	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t stationAlarm = 0;

uint8_t recivedReset = 0;
uint8_t alarmLevelRecivedFlag=0;	//bunu qoyuramki alarmLimitLevel yollanildigin qeyd edim ve bunun qebul etdiyimle bagli mesaji geri gonderende buna esasen edim.
uint8_t prencereAcilmaFlag=0;
uint16_t digitalSum[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t intPart[50];
uint8_t fractionPart[50];

uint16_t secondWord[50];


float realVal[50];
float voltValIncorrect[50];
float voltVal[50];

int recivedID = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {

	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.StdId == 0x203) {
		pk1 = RxData[0];
		datacheck = 1;

	}

	if (RxHeader.StdId == 0x500) {
		recivedReset = 1;
		//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	if (RxHeader.StdId == 0x501) {
		pk1 = RxData[0];
	}

	// burda gelen alarmLimitleri fade outlar digitiallar ucun count bilgisi birde contact state qebul et yazdir
	if (RxHeader.StdId == 0x600) {
		recivedID = (int) (RxData[0]) + ((int) (RxData[1]) << 8);
		float value = 0;
		TxData[23][0] = RxData[0];
		TxData[23][1] = RxData[1];
		for (int k = 0; k < 63; k++) {
			if (digitalInputId[k] == recivedID) {
				fadeOutReg = 1;
				alarmLevelRecivedFlag = 1;	//qebul etdiyimizi qey edirik. geri xeber etdiyimizde sifirla.
				fadeOut[k] = RxData[2] & 0x01;
				contactState[k] = (RxData[2] >> 1) & 0x01;
				delaySeconds[k] = (int)RxData[3];
			}
		}

		for(int k=0; k<38; k++)
		{
			if(analogInputID[k] == recivedID){ //deyekki id bunun icindedi
				fadeOutReg = 1;
				alarmLevelRecivedFlag = 1;	//qebul etdiyimizi qey edirik. geri xeber etdiyimizde sifirla.
				value = (int)RxData[3] + ((int)RxData[4] << 8) + ((float)RxData[5])/100;
				analogFadeOut[k] = RxData[2];
				alarmLevel[k]=value;
			}
		}
	}

	//0x650 gelende hl hazirdaki deyerleri qoy 0x656 nin icine yolla pc yeki orda pencerede goruntuleye bilsin.
	if (RxHeader.StdId == 0x650) {
		recivedID = (int) (RxData[0]) + ((int) (RxData[1]) << 8);
		for (int k = 0; k < 63; k++) {
			if (digitalInputId[k] == recivedID) {
				prencereAcilmaFlag = 1;
				TxData[24][0] = RxData[0];
				TxData[24][1] = RxData[1];
				TxData[24][2] = fadeOut[k] + (contactState[k] << 1);
				TxData[24][3] = 0;	///NIYEYSE MENASIZ OLARAQ YERIN DEYISMISEM
				TxData[24][4] = delaySeconds[k];
				TxData[24][5] = 0;
				TxData[24][6] = 0;
				TxData[24][7] = 0;
			}
		}

		for(int k=0; k<38; k++)
		{
			if(analogInputID[k] == recivedID){ //deyekki id bunun icindedi
				prencereAcilmaFlag = 1;
				TxData[24][0] = RxData[0];
				TxData[24][1] = RxData[1];
				TxData[24][2] = analogFadeOut[k];
				TxData[24][3] = 0;
				TxData[24][4] = (int) alarmLevel[k];
				TxData[24][5] = (int) alarmLevel[k] >> 8;
				TxData[24][6] = (int) ((alarmLevel[k] - (int) alarmLevel[k]) * 100);
				TxData[24][7] = 0;
			}
		}
	}
}

//float alarmLevel[] = {5.92, 5.92, 1.62, 1.62, 1.08, 5.24, 5.24, 1.03, 1.03, 1.08, 1.8, 1.8, 2.7, 2.7, 3.42, 1.35, 2.07, 1.08, 1.08, 0.9, 1.8, 1.8, 3.96, 3.72, 6.12, 6.12, 4.37, 4.37, 6.12, 6.12, 6.12, 5.31, 5.31, 5.27, 5.27, 2.61, 2.61, 4};

 struct analogConfig
 {
 float minVolt;
 float maxVolt;
 float minRealVal;
 float maxRealVal;
 float warningLevel;
 int moreThen;


 }analogConfigs[38]={
 {0.64,3.2,0,6.573,40,1},		//3005
 {0.64,3.2,0,6.573,60,1},		//3006
 {0.64,3.2,0,1.8,60,1},			//3007
 {0.64,3.2,0,1.8,60,1},			//3008
 {0.64,3.2,0,1.2,60,1},			//3009
 {0.64,3.2,0,5.833,40,1},		//3010
 {0.64,3.2,0,5.833,60,1},		//3011
 {0.64,3.2,0,1.142,60,1},		//3012
 {0.64,3.2,0,1.142,60,1},		//3013
 {0.64,3.2,0,1.2,60,1},			//3014
 {0.64,3.2,0,2,40,1},			//3015
 {0.64,3.2,0,2,60,1},			//3016
 {0.64,3.2,0,3,60,1},			//3017
 {0.64,3.2,0,3,60,1},			//3018
 {0.64,3.2,0,3.8,60,1},			//3019
 {0.64,3.2,0,1.5,40,1},			//3020
 {0.64,3.2,0,2.3,60,1},			//3021
 {0.64,3.2,0,1.2,60,1},			//3022
 {0.64,3.2,0,1.2,60,1},			//3023
 {0.64,3.2,0,1,60,1},			//3024
 {0.64,3.2,0,2,40,1},			//3025
 {0.64,3.2,0,2,40,1},			//3026
 {0.64,3.2,0,4.4,60,1},			//3027
 {0.64,3.2,0,4.133,60,1},		//3028
 {0.64,3.2,0,6.8,60,1},			//3029
 {0.64,3.2,0,6.8,60,1},			//3030
 {0.64,3.2,0,4.852,60,1},		//3031
 {0.64,3.2,0,4.852,60,1},		//3032
 {0.64,3.2,0,6.8,60,1},			//3033
 {0.64,3.2,0,6.8,60,1},			//3034
 {0.64,3.2,0,6.8,60,1},			//3035
 {0.64,3.2,0,5.9,60,1},			//3036
 {0.64,3.2,0,5.9,60,1},			//3037
 {0.64,3.2,0,5.857,60,1},		//3038
 {0.64,3.2,0,5.857,60,1},		//3039
 {0.64,3.2,0,2.9,60,1},			//3040
 {0.64,3.2,0,2.9,60,1},			//3041
 {0.64,3.2,0,40,60,0}			//3042
 };


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
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

	HAL_CAN_Start(&hcan1);

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

/*
	 for(int t=100;t > 400 ; t++){
	 EEPROM_PageErase(t);
	 }
*/

	for (int j = 0; j < 30; j++) {
		TxHeader[j].DLC = 8;
		TxHeader[j].IDE = CAN_ID_STD;
		TxHeader[j].RTR = CAN_RTR_DATA;
		TxHeader[j].StdId = id[j];
	}

	//Start Timer

	HAL_TIM_Base_Start_IT(&htim6);

	void sendData(int inputId)						//
	{
		TxData[21][0] = inputId;
		TxData[21][1] = inputId >> 8;
		TxData[21][2] = 0;
		TxData[21][3] = 0;
		TxData[21][4] = 0;
		TxData[21][5] = 0;
		TxData[21][6] = 0;
		TxData[21][7] = 0;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader[21], TxData[21], &TxMailbox);
		HAL_Delay(delayTime);
	}


	fadeOutTotRead[0] = EEPROM_Read_NUM(1, 0);
	fadeOutTotRead[1] = EEPROM_Read_NUM(2, 0);
	fadeOutTotRead[2] = EEPROM_Read_NUM(3, 0);
	fadeOutTotRead[3] = EEPROM_Read_NUM(4, 0);

	analogFadeOutTotRead[0] = EEPROM_Read_NUM(5, 0);
	analogFadeOutTotRead[1] = EEPROM_Read_NUM(6, 0);
	analogFadeOutTotRead[2] = EEPROM_Read_NUM(7, 0);

	for (int k = 0; k < 16; k++) {
		fadeOut[k] = (fadeOutTotRead[0] >> k) & 1;
		fadeOut[k + 16] = (fadeOutTotRead[1] >> k) & 1;
		fadeOut[k + 32] = (fadeOutTotRead[2] >> k) & 1;
		if (k < 15) {
			fadeOut[k + 48] = (fadeOutTotRead[3] >> k) & 1;
		}

		analogFadeOut[k] = (analogFadeOutTotRead[0] >> k) & 1;
		analogFadeOut[k + 16] = (analogFadeOutTotRead[1] >> k) & 1;

		if (k < 6) {
			analogFadeOut[k + 32] = (analogFadeOutTotRead[2] >> k) & 1;
		}
	}

	//analog alarmLevel leri burda eepromdan yukle
	for(int k=0;k<38;k++)
	{
		alarmLevel[k] = EEPROM_Read_NUM(400+k, 20);
	}

	contactStateRead[0] = EEPROM_Read_NUM(10, 0);
	contactStateRead[1] = EEPROM_Read_NUM(11, 0);
	contactStateRead[2] = EEPROM_Read_NUM(12, 0);
	contactStateRead[3] = EEPROM_Read_NUM(13, 0);
	contactStateRead[4] = EEPROM_Read_NUM(14, 0);


	//contact state leri eepromdan yukle
	for(int k=0;k<16;k++){
		contactState[k] = (contactStateRead[0] >> k) & 1;
		contactState[k+16] = (contactStateRead[1] >> k) & 1;
		contactState[k+32] = (contactStateRead[2] >> k) & 1;
		if(k<15){
			contactState[k+48] = (contactStateRead[3] >> k) & 1;
		}
	}


	// delay secondsu eepromdan oxuma
	for(int t=0;t<8;t++){
		for(int k=0;k<4;k++){
			delaySecondsTotRead[t*4+k] = EEPROM_Read_NUM(20+t, 16*k);

			delaySeconds[8*t+k*2] = (delaySecondsTotRead[t*4+k]) & 0xff;
			delaySeconds[8*t+k*2+1] = (delaySecondsTotRead[t*4+k] >> 8) & 0xff;
		}
	}


	//bu testi wile icine qoyma cunki eeprom un yazma omru var bu omur 1milyon defedi. bir cel olse butun eprom xarab olacaq.
	EEPROM_Write_NUM(0, 0, dataw3);
	datar3 = EEPROM_Read_NUM(0, 0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//EEPROM_Write_NUM(0, 4, dataw3[1]);
		//datar3[1] = EEPROM_Read_NUM(0, 4);

		if (prencereAcilmaFlag == 1){
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[24], TxData[24], &TxMailbox);
			HAL_Delay(1);
			prencereAcilmaFlag = 0;
		}

		// bu hissede eger alarm level deyisibse yollayirq
		if (alarmLevelRecivedFlag == 1)
		{
			for(int k=0;k<38;k++){
				EEPROM_Write_NUM(400+k, 20, alarmLevel[k]);
				alarmLevelRead[k] = EEPROM_Read_NUM(400+k, 20);
			}

			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[23], TxData[23], &TxMailbox);
			HAL_Delay(1);
			alarmLevelRecivedFlag = 0;
		}

		if (fadeOutReg == 1) {
			fadeOutTot[0] = 0;
			fadeOutTot[1] = 0;
			fadeOutTot[2] = 0;
			fadeOutTot[3] = 0;
			analogFadeOutTot[0] = 0;
			analogFadeOutTot[1] = 0;
			analogFadeOutTot[2] = 0;

			for (int t = 0; t < 16; t++) {
				fadeOutTot[0] |= fadeOut[t] << t;
				fadeOutTot[1] |= fadeOut[t + 16] << t;
				fadeOutTot[2] |= fadeOut[t + 32] << t;
				if (t < 15) {
					fadeOutTot[3] |= fadeOut[t + 48] << t;
				}

				analogFadeOutTot[0] |= analogFadeOut[t] << t;
				analogFadeOutTot[1] |= analogFadeOut[t + 16] << t;

				if (t < 6) {
					analogFadeOutTot[2] |= analogFadeOut[t + 32] << t;
				}
			}

			EEPROM_Write_NUM(1, 0, fadeOutTot[0]);
			EEPROM_Write_NUM(2, 0, fadeOutTot[1]);
			EEPROM_Write_NUM(3, 0, fadeOutTot[2]);
			EEPROM_Write_NUM(4, 0, fadeOutTot[3]);

			EEPROM_Write_NUM(5, 0, analogFadeOutTot[0]);
			EEPROM_Write_NUM(6, 0, analogFadeOutTot[1]);
			EEPROM_Write_NUM(7, 0, analogFadeOutTot[2]);

			fadeOutTotReadTest[0] = EEPROM_Read_NUM(1, 0);
			fadeOutTotReadTest[1] = EEPROM_Read_NUM(2, 0);
			fadeOutTotReadTest[2] = EEPROM_Read_NUM(3, 0);
			fadeOutTotReadTest[3] = EEPROM_Read_NUM(4, 0);

			analogFadeOutTotReadTest[0] = EEPROM_Read_NUM(5, 0);
			analogFadeOutTotReadTest[1] = EEPROM_Read_NUM(6, 0);
			analogFadeOutTotReadTest[2] = EEPROM_Read_NUM(7, 0);

			contactStateTot[0] = 0;
			contactStateTot[1] = 0;
			contactStateTot[2] = 0;
			contactStateTot[3] = 0;
			contactStateTot[4] = 0;

			//digital signallarin contact state ini eeproma yazdirmaq
			for(int t=0;t<16;t++){
				contactStateTot[0] |= contactState[t]<<t;
				contactStateTot[1] |= contactState[t+16]<<t;
				contactStateTot[2] |= contactState[t+32]<<t;
				if(t<15){
					contactStateTot[3] |= contactState[t+48]<<t;
				}
			}

			EEPROM_Write_NUM (10, 0, contactStateTot[0]);
			EEPROM_Write_NUM (11, 0, contactStateTot[1]);
			EEPROM_Write_NUM (12, 0, contactStateTot[2]);
			EEPROM_Write_NUM (13, 0, contactStateTot[3]);
			EEPROM_Write_NUM (14, 0, contactStateTot[4]);

			contactStateTest[0] = EEPROM_Read_NUM(10, 0);
			contactStateTest[1] = EEPROM_Read_NUM(11, 0);
			contactStateTest[2] = EEPROM_Read_NUM(12, 0);
			contactStateTest[3] = EEPROM_Read_NUM(13, 0);
			contactStateTest[4] = EEPROM_Read_NUM(14, 0);


			/////////delay secondsu eeproma yazma

			for(int t=0;t<8;t++){
				for(int k=0;k<4;k++){
					delaySecondsTot[t*4+k] |= delaySeconds[8*t+k*2]<<0;
					delaySecondsTot[t*4+k] |= delaySeconds[8*t+k*2+1]<<8;

					EEPROM_Write_NUM(20+t, 16*k, delaySecondsTot[t*4+k]);
					delaySecondsTot[t*4+k] = 0;
				}
			}
			fadeOutReg = 0;
		}



		for (int k = 0; k < 16; k++) {
			fadeOutBaxmaq[k] = (fadeOutTotReadTest[0] >> k) & 1;
			fadeOutBaxmaq[k + 16] = (fadeOutTotReadTest[1] >> k) & 1;
			fadeOutBaxmaq[k + 32] = (fadeOutTotReadTest[2] >> k) & 1;
			if (k < 15) {
				fadeOutBaxmaq[k + 48] = (fadeOutTotRead[3] >> k) & 1;
			}

			analogFadeOutBaxmaq[k] = (analogFadeOutTotReadTest[0] >> k) & 1;
			analogFadeOutBaxmaq[k + 16] = (analogFadeOutTotReadTest[1] >> k) & 1;

			if (k < 6) {
				analogFadeOutBaxmaq[k + 32] = (analogFadeOutTotReadTest[2] >> k) & 1;
			}
		}


		////////////////////////////////////////////////mux u saydir adc ve dig deyerlri yolla/////////////////////////////////////////////////////
		for (int t = 0; t < 16; ++t) {
			mux(15 - t);
			HAL_Delay(1);
			check_channels(t);
		}

		for (int t = 0; t < 43; t++) {
			analog[t] = message[t]; ////lazimsiz bos mesajlari atmaq
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////tek oxunan digitallar//////////////////////////////////////////////////////////////////////
		/*
		 * digital deyerleri oxu ve surusturerek 1 wordluk sum a yaz
		 */

		///////////////////////////////////16 digital/////////////////////////////////  ///digitallari arraya duzmek
		digitalStates[51] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7); //dig 1.1	//dig 53
		digitalStates[50] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5); //dig 1.2	//dig 54
		digitalStates[49] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); //dig 1.3	//dig 55
		digitalStates[48] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0); //dig 1.4	//dig 56
		digitalStates[52] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12); //dig 1.5	//dig 57
		digitalStates[53] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_15); //dig 1.6	//dig 58
		digitalStates[54] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2); //dig 1.7	//dig 59
		digitalStates[55] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11); //dig 1.8	//dig 60
		digitalStates[56] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15); //dig 1.9	//dig 61
		digitalStates[57] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); //dig 1.10	//dig 62
		digitalStates[58] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13); //dig 1.11	//dig 63
		digitalStates[59] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14); //dig 1.12	//dig 64
		digitalStates[60] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0); //dig 1.13	//dig 1
		digitalStates[61] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1); //dig 1.14	//dig 2
		digitalStates[62] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2); //dig 1.15	//dig 3
		//digitalStates[63]	= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);	//dig 1.16	//dig 4

		///////////////////////////////////16 digital/////////////////////////////////  ///digitallari arraya duzmek
		digitalStates[0] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3);	//dig 2.1	//dig 65
		digitalStates[1] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2);	//dig 2.2	//dig 66
		digitalStates[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);//dig 2.3	//dig 67
		digitalStates[3] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4);	//dig 2.4	//dig 68
		digitalStates[4] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_13);//dig 2.5	//dig 69
		digitalStates[5] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12);//dig 2.6	//dig 70
		digitalStates[6] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);	//dig 2.7	//dig 71
		digitalStates[7] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_14);//dig 2.8	//dig 72
		digitalStates[8] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2);	//dig 2.9	//dig 73
		digitalStates[9] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);//dig 2.10	//dig 74
		digitalStates[10] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);//dig 2.11	//dig 75
		digitalStates[11] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);//dig 2.12	//dig 76
		digitalStates[12] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);//dig 2.13	//dig 77
		digitalStates[13] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);//dig 2.14	//dig 78
		digitalStates[14] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);//dig 2.15	//dig 79
		digitalStates[15] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);//dig 2.16	//dig 80

		//////////////////////////////////16 digital//////////////////////////////////  ///digitallari arraya duzmek
		digitalStates[16] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);//dig 3.1	//dig 81
		digitalStates[17] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);//dig 3.2	//dig 82
		digitalStates[18] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);//dig 3.3	//dig 83
		digitalStates[19] = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);//dig 3.4	//dig 84
		digitalStates[20] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);//dig 3.5	//dig 85
		digitalStates[21] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);//dig 3.6	//dig 86
		digitalStates[22] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);//dig 3.7	//dig 87
		digitalStates[23] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);//dig 3.8	//dig 88
		digitalStates[24] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9);//dig 3.9	//dig 89
		digitalStates[25] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10);//dig 3.10	//dig 90
		digitalStates[26] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11);//dig 3.11	//dig 91
		digitalStates[27] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_15);//dig 3.12	//dig 92
		digitalStates[28] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);//dig 3.13	//dig 93
		digitalStates[29] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);//dig 3.14	//dig 94
		digitalStates[30] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);//dig 3.15	//dig 95
		digitalStates[31] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);//dig 3.16	//dig 96

		//////////////////////////////////16 digital//////////////////////////////////  ///digitallari arraya duzmek
		digitalStates[32] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);//dig 4.1	//dig 97
		digitalStates[33] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);//dig 4.2	//dig 98
		digitalStates[34] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);//dig 4.3	//dig 99
		digitalStates[35] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5);//dig 4.4	//dig 100
		digitalStates[36] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7); //dig 4.5	//dig 101
		digitalStates[37] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); //dig 4.6	//dig 102
		digitalStates[38] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8); //dig 4.7	//dig 103
		digitalStates[39] = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7); //dig 4.8	//dig 104
		digitalStates[40] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9); //dig 4.9	//dig 105
		digitalStates[41] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); //dig 4.10	//dig 106
		digitalStates[42] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9); //dig 4.11	//dig 107
		digitalStates[43] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8); //dig 4.12	//dig 108
		digitalStates[44] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7); //dig 4.13	//dig 109
		digitalStates[45] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6); //dig 4.14	//dig 110
		digitalStates[46] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5); //dig 4.15	//dig 111
		digitalStates[47] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4); //dig 4.16	//dig 112

		for (int k = 0; k < 63; k++) {
			if ((k != 10) && (k != 11) && (k != 12) && (k != 13) && (k != 24) && (k != 25) && (k != 26) && (k != 27) && (k != 28) && (k != 41) && (k != 42) && (k != 44)) {
				if (digitalStates[k] == contactState[k]){ //eger digital girisimiz biizm mueyyen elediyimiz veziyetdedise yeni loru dile yanibsa
					if (alarmOn[k] == 0){    			//eger alarim cixmayibsa
						if (waitingForDelay[k] == 1){ //qoyulan vaxdin tamamlanmagin gozdeyirik
							if ((delaySeconds[k] <= delaySecondsCount[k]) && (fadeOut[k] == 0)){ // qoyulan vaxda catdisa
								alarmOn[k] = 1;                  //alarmi yandir
								delaySecondsCountForOff[k] = 40; //alarmi sonudrmek ucun olan sayicini 5 ele
								sendData(digitalInputId[k]);				//
								stationAlarm = notResetAlarm;//alarimi yandir signal cixdi deye
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);	//alarim isigin yandir
								waitingForDelay[k] = 0;	//delay ucun gozdeme sayicisin sifirla
							}
						} else {
							alarmCount[k]++;//n defe alarm cixidigin yoxlayan sayici
							if (alarmCount[k] > 2){			//2 defe cixdisa gir
								if ((delaySeconds[k] == 0) && (fadeOut[k] == 0)){//saniye sayan 0 disa gir
									alarmOn[k] = 1;				//alari yandir
									delaySecondsCountForOff[k] = 40; //alarmi sonudrmek ucun olan sayicini 5 ele
									sendData(digitalInputId[k]);
									stationAlarm = notResetAlarm;//alarimi yandir signal cixdi deye
									HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);//alarim isigin yandir
								} else {
									waitingForDelay[k] = 1;	//sayci ucun gozdeme regisitiri
								}
								alarmCount[k] = 5;//alarm count 4 den boyukduse 5 den cox boyumesin
							}
						}
						//delay;
					}
				} else { 	//sifirla eger signal gelmeyibse hec
					if (delaySecondsCountForOff[k] == 0) {
						alarmOn[k] = 0;
					}
					alarmCount[k] = 0;
					waitingForDelay[k] = 0;
					delaySecondsCount[k] = 0;
				}

				tamHisse = k / 8; // 1 id 4 word yollayir onagore her word ucun boluruk
				kesirHisse = k % 8;	//sonraki wordun necencisi olduguna bundan baxiriq

				if (alarmOn[k] != 0) {
					digitalSum[tamHisse] |= (1 << kesirHisse * 2); //burdada necncei wordun olduguna sonra hemin wordun necencisi olduguna baxiriq
				}
				digitalSum[tamHisse] |= (fadeOut[k] << (kesirHisse * 2 + 1));
			}
		}
		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == 1 || recivedReset == 1){ //reset basdiq veya conpuyuterden reset geldi
			stationAlarm = resetAlarm;						//alarmi reset et
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);//ve sondur alarmi
			recivedReset = 0;				//compyuterden gelen reseti sifirla
		}

		TxData[22][0] = stationAlarm;
		TxData[22][1] = stationAlarm;
		TxData[22][2] = stationAlarm;
		TxData[22][3] = stationAlarm;

		if(sendCountCheck[40] >= catacaqSay){
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[22], TxData[22], &TxMailbox);
			HAL_Delay(delayTime);
			sendCountCheck[40] = 0;
		}
		else{
			sendCountCheck[40]++;
		}

		TxData[19][0] = digitalSum[0];
		TxData[19][1] = digitalSum[0] >> 8;
		TxData[19][2] = digitalSum[1];
		TxData[19][3] = digitalSum[1] >> 8;
		TxData[19][4] = digitalSum[2];
		TxData[19][5] = digitalSum[2] >> 8;
		TxData[19][6] = digitalSum[3];
		TxData[19][7] = digitalSum[3] >> 8;

		TxData[20][0] = digitalSum[4];
		TxData[20][1] = digitalSum[4] >> 8;
		TxData[20][2] = digitalSum[5];
		TxData[20][3] = digitalSum[5] >> 8;
		TxData[20][4] = digitalSum[6];
		TxData[20][5] = digitalSum[6] >> 8;
		TxData[20][6] = digitalSum[7];
		TxData[20][7] = digitalSum[7] >> 8;

		if(sendCountCheck[41] >= catacaqSay){
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[19], TxData[19], &TxMailbox);
			HAL_Delay(delayTime);
			sendCountCheck[41] = 0;
		}
		else{
			sendCountCheck[41]++;
		}

		if(sendCountCheck[42] >= catacaqSay){
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader[20], TxData[20], &TxMailbox);
			HAL_Delay(delayTime);
			sendCountCheck[42] = 0;
		}
		else{
			sendCountCheck[42]++;
		}

		digitalSum[0] = 0;
		digitalSum[1] = 0;
		digitalSum[2] = 0;
		digitalSum[3] = 0;
		digitalSum[4] = 0;
		digitalSum[5] = 0;
		digitalSum[6] = 0;
		digitalSum[7] = 0;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		for (int i = 0; i < 19; i++) {
			for (int t = 0; t < 2; t++) {
				i2_t = i*2+t;

				voltVal[i2_t] = (((float) analog[i2_t]) * 3.3)/ 4096.0;

				realVal[i2_t] = ((voltVal[i2_t] - analogConfigs[i2_t].minVolt)/ (analogConfigs[i2_t].maxVolt - analogConfigs[i2_t].minVolt)) * (analogConfigs[i2_t].maxRealVal - analogConfigs[i2_t].minRealVal); //olculen vahide gore hesablanan deyer yeni tempdise tempratur qarsiligi voltajin
				intPart[i2_t] = (uint16_t) realVal[i2_t];
				fractionPart[i2_t] = (uint8_t) ((realVal[i2_t] - intPart[i2_t]) * 100);

				if (analogConfigs[i2_t].moreThen == 0) {
					if (realVal[i2_t] < alarmLevel[i2_t]) {
						analogAlarmCountDown[i2_t] = 0;
						if (alarmOnAnalog[i2_t] == 0) {
							analogAlarmCount[i2_t]++; // analog alarimin say
							if ((analogAlarmCount[i2_t] >= 10) && (analogFadeOut[i2_t] == 0)){ // 4 defe alarm verse analog alarimin yandir
								alarmOnAnalog[i2_t] = 1;
								sendData(analogInputID[i2_t]);
								secondByte[i2_t] |= 1; // eger alarim oldusa 1 ci biti 1 ele
								stationAlarm = notResetAlarm;	//alarm cixdi
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //alarm isigin yandir
								analogAlarmCount[i2_t] = 10; //analog sayicisi 4 e catdisa 4 de saxla
							}
						}
					} else {
						analogAlarmCountDown[i2_t]++;
						if (analogAlarmCountDown[i2_t] >= 10) {
							analogAlarmCount[i2_t] = 0; //alarim deyilse sayicini sifirla
							alarmOnAnalog[i2_t] = 0;
							analogAlarmCountDown[i2_t] = 10;
							secondByte[i2_t] &= ~1;
						}
					}
				} else if (analogConfigs[i2_t].moreThen == 1) {
					if (realVal[i2_t] > alarmLevel[i2_t]) {
						analogAlarmCountDown[i2_t] = 0;
						if (alarmOnAnalog[i2_t] == 0) {
							analogAlarmCount[i2_t]++; // analog alarimin say
							if ((analogAlarmCount[i2_t] >= 10) && (analogFadeOut[i2_t] == 0)){ // 4 defe alarm verse analog alarimin yandir
								secondByte[i2_t] |= 1; // eger alarim oldusa 1 ci biti 1 ele
								alarmOnAnalog[i2_t] = 1;
								sendData(analogInputID[i2_t]);
								stationAlarm = notResetAlarm;	//alarm cixdi
								HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); //alarm isigin yandir
								analogAlarmCount[i2_t] = 10; //analog sayicisi 4 e catdisa 4 de saxla
							}
						}
					} else {
						analogAlarmCountDown[i2_t]++;
						if (analogAlarmCountDown[i2_t] >= 10) {
							alarmOnAnalog[i2_t] = 0;
							analogAlarmCountDown[i2_t] = 10;
							analogAlarmCount[i2_t] = 0; //alarim deyilse sayicini sifirla
							secondByte[i2_t] &= ~1;
						}
					}
				}

				if (analogFadeOut[i2_t] == 1) {
					secondByte[i2_t] |= 2;
				} else {
					secondByte[i2_t] &= ~2;
				}

				secondByte[i2_t] |= (int)(63/(analogConfigs[i2_t].maxRealVal)*alarmLevel[i2_t]) << 2;

				secondWord[i2_t] = (uint16_t) secondByte[i2_t] + ((uint16_t) fractionPart[i2_t]) * 256;
				secondByte[i2_t] &= 0x03;

				TxData[i][t * 4] = intPart[i2_t];
				TxData[i][t * 4 + 1] = intPart[i2_t] >> 8;
				TxData[i][t * 4 + 2] = secondWord[i2_t];
				TxData[i][t * 4 + 3] = secondWord[i2_t] >> 8;
			}

			if(sendCountCheck[i] >= catacaqSay){
				HAL_CAN_AddTxMessage(&hcan1, &TxHeader[i], TxData[i], &TxMailbox);
				HAL_Delay(delayTime);
				sendCountCheck[i] = 0;
			}
			else{
				sendCountCheck[i]++;
			}
		}

		/*
		 TxData[16][0] = digitalSum;
		 TxData[16][1] = digitalSum;
		 TxData[16][2] = digitalSum;
		 TxData[16][3] = digitalSum;

		 //bu neyi yollayir bilmedim arasdir
		 HAL_CAN_AddTxMessage(&hcan1, &TxHeader[12], TxData[12], &TxMailbox);
		 HAL_Delay(1);
		 */
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		//  HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 64;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 10;
	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canfilterconfig.FilterIdHigh = 0; //0x446<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0; //0x446<<5;
	canfilterconfig.FilterMaskIdLow = 0;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM30_GPIO_Port, ALARM30_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SEL3_Pin|SEL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SEL1_Pin|SEL0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D2_9_Pin D2_10_Pin D1_11_Pin D2_12_Pin
                           D2_7_Pin D2_11_Pin D1_12_Pin D1_9_Pin
                           D2_3_Pin D2_16_Pin */
  GPIO_InitStruct.Pin = D2_9_Pin|D2_10_Pin|D1_11_Pin|D2_12_Pin
                          |D2_7_Pin|D2_11_Pin|D1_12_Pin|D1_9_Pin
                          |D2_3_Pin|D2_16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_6_Pin PC0 PC1 PC2
                           D4_1_Pin D4_2_Pin D4_8_Pin D4_7_Pin
                           D3_10_Pin D3_11_Pin D3_12_Pin */
  GPIO_InitStruct.Pin = D2_6_Pin|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |D4_1_Pin|D4_2_Pin|D4_8_Pin|D4_7_Pin
                          |D3_10_Pin|D3_11_Pin|D3_12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_5_Pin PF6 D1_8_Pin D1_5_Pin
                           D1_6_Pin */
  GPIO_InitStruct.Pin = D2_5_Pin|GPIO_PIN_6|D1_8_Pin|D1_5_Pin
                          |D1_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_3_Pin D1_2_Pin D1_1_Pin D3_9_Pin
                           D4_5_Pin D4_6_Pin D3_13_Pin D3_14_Pin
                           D2_8_Pin */
  GPIO_InitStruct.Pin = D1_3_Pin|D1_2_Pin|D1_1_Pin|D3_9_Pin
                          |D4_5_Pin|D4_6_Pin|D3_13_Pin|D3_14_Pin
                          |D2_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM30_Pin */
  GPIO_InitStruct.Pin = ALARM30_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM30_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_4_Pin D1_7_Pin D1_10_Pin D1_15_Pin
                           D3_1_Pin D3_2_Pin D3_3_Pin D3_4_Pin
                           D2_13_Pin D2_14_Pin D2_15_Pin */
  GPIO_InitStruct.Pin = D1_4_Pin|D1_7_Pin|D1_10_Pin|D1_15_Pin
                          |D3_1_Pin|D3_2_Pin|D3_3_Pin|D3_4_Pin
                          |D2_13_Pin|D2_14_Pin|D2_15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL3_Pin SEL2_Pin */
  GPIO_InitStruct.Pin = SEL3_Pin|SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL1_Pin SEL0_Pin */
  GPIO_InitStruct.Pin = SEL1_Pin|SEL0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_14_Pin D1_13_Pin D1_16_Pin D3_16_Pin
                           D3_15_Pin D4_3_Pin D4_4_Pin D3_5_Pin
                           D3_6_Pin D3_7_Pin D2_2_Pin D2_1_Pin
                           D2_4_Pin D3_8_Pin */
  GPIO_InitStruct.Pin = D1_14_Pin|D1_13_Pin|D1_16_Pin|D3_16_Pin
                          |D3_15_Pin|D4_3_Pin|D4_4_Pin|D3_5_Pin
                          |D3_6_Pin|D3_7_Pin|D2_2_Pin|D2_1_Pin
                          |D2_4_Pin|D3_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_12_Pin D4_11_Pin D4_10_Pin D4_9_Pin */
  GPIO_InitStruct.Pin = D4_12_Pin|D4_11_Pin|D4_10_Pin|D4_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		//HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);
		for (int h = 0; h < 63; h++) {
			if (waitingForDelay[h] == 1) {
				delaySecondsCount[h]++;
				if (delaySecondsCount[h] >= 255) {
					delaySecondsCount[h] = 255;
				}
			}
			if (delaySecondsCountForOff[h] > 0) {
				delaySecondsCountForOff[h] -= 1;
			}
		}
	}
}


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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
