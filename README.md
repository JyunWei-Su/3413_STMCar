# 110-1 (3413) 微處理機實驗 期末專題
[TOC]
## 架構
### source code on github: [3413_STMCar](https://github.com/JyunWei-Su/3413_STMCar)
* 運作架構圖
![](https://i.imgur.com/hZxdDXw.png)
* 馬達接線圖
![](https://i.imgur.com/4e9Bwbo.jpg)
### 腳位接線
* STM32F401 discovery

    |    介面    |    STM32F401     |         模組腳位         |        模組        |
    |:----------:|:----------------:|:------------------------:|:------------------:|
    | ADC (IN-4) |  PA4(ADC1_IN4)   |           Vout           |      可變電阻      |
    |            |                  |                          |                    |
    |   SPI-1    |      免接線      | PE3要設為<br>GPIO_OUTPUT | L3GD20(板載陀螺儀) |
    |   I2C-1    |      免接線      |           ---            | LSM303DLHC(加速度) |
    |            |                  |                          |                    |
    |   I2C-3    |  PC9(I2C3_SDA)   |           SDA            |      Lcd 16x2      |
    |   I2C-3    |  PC9(I2C3_SCL)   |           SCL            |      Lcd 16x2      |
    |            |                  |                          |                    |
    |   UART2    |  PA2(USART2_TX)  |            RX            |  USB-TTL(115200)   |
    |   UART2    |  PA3(USART2_RX)  |            TX            |  USB-TTL(115200)   |
    |   UART6    |  PC6(USART6_TX)  |            RX            |  藍芽HC-06(9600)   |
    |   UART6    |  PC7(USART6_RX)  |            TX            |  藍芽HC-06(9600)   |
    |            |                  |                          |                    |
    |    PWM     |  PC8(TIM3_CH3)   |           Vin            |       Buzzer       |
    |            |                  |                          |                    |
    |    PWM     |  PC8(TIM3_CH2)   |           B-1A           |       L9110s       |
    |    GPIO    | PC3(GPIO_OUTPUT) |           B-1B           |       L9110s       |
    |    PWM     |  PC4(TIM3_CH1)   |           A-1A           |       L9110s       |
    |    GPIO    | PD7(GPIO_OUTPUT) |           A-1B           |       L9110s       |
    |            |                  |                          |                    |
    |    GPIO    | PA13(GPIO_INPUT) |           ECHO           |   HC-SR04超音波    |
    |    GPIO    | PA10(GPIO_INPUT) |           TRIG           |   HC-SR04超音波    |
### 功能說明

## 開發摘要(一) STM32H747 DISCO
![](https://www.st.com/bin/ecommerce/api/image.PF265379.en.feature-description-include-personalized-no-cpn-large.jpg)
### 使用TouchGFX設計UI介面
1. 使用`Progress Indicator Example`範例作為基礎做修改
![](https://i.imgur.com/Zdpx0gU.png)
2. 介面中加入所需的按鈕、進度條、文字、圖形
    a. 左/右上角為文字進度條(text progress bar)，用來呈現傳送的速度與從小車接收到的回傳速度
    b. 中間的部分為按鈕，用來控制小車前進後退轉彎
    c. 左/右下角為線形進度條(line progress bar)，用來呈現小車所回傳的加速度計數值、陀螺儀數值
![](https://i.imgur.com/Y2dPYFz.png)
3. 設定每一個按鈕的 Interaction
    a. 觸發條件為`按鈕被點擊`
    b. 觸發執行c++ code
    c. 使用c++ code傳送控制data到小車(詳細見 [資料通訊](#資料通訊) 節)
![](https://i.imgur.com/ZIQl39I.png)
4. 輸出專案供下一步使用(電腦上需要安裝CubeMX)
![](https://i.imgur.com/aTmVoCV.png)

### 使用STM32CubeIDE接續開發
1. Import稍早輸出的專案
2. 在`MainViewBase.cpp`中增加等等會用到的變數(用extern是因為他與稍後的
![](https://i.imgur.com/jREK5Qm.png)
```c=
extern UART_HandleTypeDef huart1;
extern int speed_L_T, speed_R_T;
unsigned char uTx;
```
3. 同一個檔案下設定每個按鈕要傳送的資料
![](https://i.imgur.com/faz5A7U.png)
```c=
void MainViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &buttonWithLabel_0)
    {
        //Interaction0
        //When buttonWithLabel_0 clicked execute C++ code
        //Execute C++ code
    	//unsigned char uTx = '0';
        //HAL_UART_Transmit(&huart1, &uTx, sizeof(uTx), 10);
    	speed_L_T = 2;
    	speed_R_T = 2;
        uTx = '0';
        HAL_UART_Transmit_IT(&huart1, &uTx, sizeof(uTx));
        HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_12);
    }
//...略
}
```
4. 在`MainView.cpp`中增加等等會用到的變數(用extern是因為他與稍後的
![](https://i.imgur.com/GZx1P3e.png)
```c=
extern int speed_L_T, speed_L_R, speed_R_T, speed_R_R;
extern int G_X, G_Y, G_Z, A_X, A_Y, A_Z;
```
5. 同樣在`MainView.cpp`中 完成`updateProgress`函式，使其可以動態更新參數至螢幕上
![](https://i.imgur.com/zdtTUbV.png)
```c=
void MainView::updateProgress(uint16_t tick)
{
    textProgress_L_T.setValue(speed_L_T);
    textProgress_R_T.setValue(speed_R_T);
    textProgress_L_R.setValue(speed_L_R);
    textProgress_R_R.setValue(speed_R_R);
    lineProgress_G_X.setValue(G_X);
    lineProgress_G_Y.setValue(G_Y);
    lineProgress_G_Z.setValue(G_Z);
    lineProgress_A_X.setValue(A_X);
    lineProgress_A_Y.setValue(A_Y);
    lineProgress_A_Z.setValue(A_Z);
}

void MainView::handleTickEvent()
{
    updateProgress(tickCounter);
}
```
6. 在`main.c`中定義全域變數
![](https://i.imgur.com/Sg9Sr53.png)
```c=
/* USER CODE BEGIN 0 */
int speed_L_T, speed_L_R, speed_R_T, speed_R_R;
int G_X = 16384, G_Y = 16384, G_Z = 16384, A_X = 16384, A_Y = 16384, A_Z = 16384;
unsigned char uRx_byte;
unsigned char uRx_data[40];
int uRx_index;
/* USER CODE END 0 */
```

7. 同樣在main.c中的USER CODE BEGIN 4 中設定UART的接收函數、功能
![](https://i.imgur.com/0c2iHQT.png)
```c=
    /* USER CODE BEGIN 4 */
int getSubInt(int start, int size)
{
	unsigned char temp[size + 1];
	int i;
	for(i = 0; i < size; i++) temp[i] = uRx_data[start + i];
	temp[size] = '\0';
	return atoi(temp);
}

void update_parameter()
{
  if(uRx_data[0] == '=' && uRx_data[33] == '@')
  {
    speed_L_R = getSubInt(1, 1);
    speed_R_R = getSubInt(2, 1);
    G_X = getSubInt( 3, 5);
    G_Y = getSubInt( 8, 5);
    G_Z = getSubInt(13, 5);
    A_X = getSubInt(18, 5);
    A_Y = getSubInt(23, 5);
    A_Z = getSubInt(28, 5);
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    //HAL_UART_Transmit_IT(&huart1, &uRx_data, 1);
	if (uRx_index >= 35) uRx_index = 0;
    HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_13);
    if(uRx_byte == '=') uRx_index = 0;
    uRx_data[uRx_index] = uRx_byte;
    if(uRx_byte == '@') update_parameter();
    uRx_index++;
    HAL_UART_Receive_IT(&huart1, &uRx_byte, 1);
  }
}
/* USER CODE END 4 */
```

    


## 開發摘要(二) STM32F401 DISCOVERY
![](https://www.st.com/bin/ecommerce/api/image.PF252419.en.feature-description-include-personalized-no-cpn-medium.jpg)
### 腳位設定/參數/Clock
1. 腳位設定
![](https://i.imgur.com/6VqiNHO.png)
![](https://i.imgur.com/WqZv6K6.png)
2. RCC 參數
![](https://i.imgur.com/dXQT0Ak.png)
3. GPIO 參數
![](https://i.imgur.com/0ZnEPge.png)
4. ADＣ　參數
![](https://i.imgur.com/BRccLu8.png)
5. TIM2 參數(==NVIC要記得打勾==/未顯示於下方圖片中)
![](https://i.imgur.com/0XlbaYq.png)
6. TIM3 參數(==NVIC要記得打勾==/未顯示於下方圖片中)
![](https://i.imgur.com/Oue3fkg.png)
7. TIM4 參數(==NVIC要記得打勾==/未顯示於下方圖片中)
![](https://i.imgur.com/4zj9TnU.png)
8. I2C1/3 參數
![](https://i.imgur.com/kACmLUQ.png)
9. SPI1 參數
![](https://i.imgur.com/c4TRSUp.png)
10. UART2/6 參數(==NVIC要記得打勾==/未顯示於下方圖片中)
    * UART2 用於USB-TTL(鮑率==115200==)、UART6 用於HC-SR06藍芽模組(鮑率==9600==)
![](https://i.imgur.com/shflexL.png)
11. Clock 設定到72MHz
![](https://i.imgur.com/My5cuGi.png)
### 程式Coding
==以下Coding皆位於main.c==
1. USER CODE BEGIN Includes
    * 關於板載加速度計、陀螺儀的用法，請參考[STM32F4_ACC_GYRO](https://hackmd.io/@JyunWei-Su/STM32F4_ACC_GYRO)文章
```c=
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f401_discovery_gyroscope.h"
#include "stm32f401_discovery_accelerometer.h"
#include "lcd16x2_i2c.h"
//need to addd flag -u _printf_float
/* USER CODE END Includes */
```
2. Define
```c=
/* USER CODE BEGIN PTD */
#define usTIM	TIM4
/* USER CODE END PTD */
```
3. Function PT
```c=
/* USER CODE BEGIN PFP */
void setMotor(int Speed_L, int Speed_R);
int map(int src, int a_1, int a_2, int b_1, int b_2);
void getADCValue();
void getDistance();
void getGYROandACCR();
void usDelay(uint32_t uSec);
void checkSaveDistance();
void setBuzzer();
/* USER CODE END PFP */
```
4. 全域變數(USER CODE BEGIN 0)
```c=
/* USER CODE BEGIN 0 */
// ================================================================================================= begin 0
// for uart
unsigned char uRx_byte;
unsigned char uTx_data[40];
//unsigned char uTx_Data[100];

// for gyroscope and accelerometer
float pfData[3];
int16_t pDataXYZ[3];

int pwmVal = 0;
int count_tim2 = 0;
int Speed_L = 0, Speed_R = 0; // -2, -1, 0, 1, 2

// for sr04 and adc
uint16_t adc_value = 0;
uint16_t limit_range_cm = 10; // 10 cm ~ 50 cm
const float speedOfSound = 0.0343/2;
float distance;
/* USER CODE END 0 */
```
4. 初始化設定(USER CODE BEGIN 2)
```c=
/* USER CODE BEGIN 2 */
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();

  HAL_UART_Receive_IT(&huart2, &uRx_byte, 1);
  HAL_UART_Receive_IT(&huart6, &uRx_byte, 1);

  // for motor pwm
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  // for buzzer pwm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // for 16x2 lcd
  lcd16x2_i2c_init(&hi2c3);
  lcd16x2_i2c_printf("PRINT_TEST");

  setMotor(0, 0);
  /* USER CODE END 2 */
```
5. 迴圈(USER CODE BEGIN 3)
```c=
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //getDistance();
	  checkSaveDistance();
	  setMotor(Speed_L, Speed_R);

	  //HAL_Delay(0.1);
  }
  /* USER CODE END 3 */
```
6. 功能函數(USER CODE BEGIN 4)
```c=
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		HAL_UART_Transmit_IT(&huart2, &uRx_byte, 1);
		HAL_UART_Receive_IT(&huart2, &uRx_byte, 1);
		//HAL_UART_Transmit_IT(&huart6, &uRx_byte, 1);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // red
	}
	if(huart->Instance == USART6) // BT
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15); // blue
		// HAL_UART_Transmit_IT(&huart2, &uRx_byte, 1);
		HAL_UART_Transmit_IT(&huart2, &uRx_byte, 1); // trans to debug
		switch(uRx_byte)
		{
		  case '0': Speed_L =  0; Speed_R =  0; break;
		  case '1': Speed_L =  1; Speed_R =  1; break;
		  case '2': Speed_L =  2; Speed_R =  2; break;
		  case '3': Speed_L = -1; Speed_R = -1; break;
		  case '4': Speed_L = -2; Speed_R = -2; break;
		  case '5': Speed_L =  1; Speed_R =  0; break;
		  case '6': Speed_L =  2; Speed_R =  0; break;
		  case '7': Speed_L =  2; Speed_R =  1; break;
		  case '8': Speed_L = -2; Speed_R = -1; break;
		  case '9': Speed_L =  0; Speed_R =  1; break;
		  case 'a': Speed_L =  0; Speed_R =  2; break;
		  case 'b': Speed_L =  1; Speed_R =  2; break;
		  case 'c': Speed_L = -1; Speed_R = -2; break;
		  default : Speed_L =  0; Speed_R =  0; break;
		}
		HAL_UART_Receive_IT(&huart6, &uRx_byte, 1);
		//checkSaveDistance();
		//setMotor(Speed_L, Speed_R);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  count_tim2++;
  // Check which version of the timer triggered this callback and toggle LED

  if (htim == &htim2 ) // 25 round per 1 sec
  {
	if(count_tim2 % 5 == 0)
	{
		getADCValue();
		getGYROandACCR();
	}
	else if(count_tim2 % 5 == 1)
	{
	  printLcd();
	}
	else if(count_tim2 % 5 == 2)
	{
	  transmitData();
	}
	else if(count_tim2 % 5 == 3)
	{
	  getDistance();
	  //checkSaveDistance();
	}
	else // %5 == 4
	{
		setBuzzer();
	}
    //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // red

  }
}

// ============== transmit data
void transmitData()
{
	int temp_G[3], temp_A[3];
	int i;
	for(i = 0; i < 3; i++) temp_G[i] = map((int)(pfData[i] * 1), -30000, 30000, 0, 32675);
	for(i = 0; i < 3; i++) temp_A[i] = map((int)(pDataXYZ[i] ), -20000, 20000, 0, 32675);
	sprintf(uTx_data, "=%d%d%05d%05d%05d%05d%05d%05d@\0", Speed_L + 2, Speed_R + 2, temp_G[0], temp_G[1], temp_G[2], temp_A[0], temp_A[1], temp_A[2]);
	HAL_UART_Transmit_IT(&huart2, &uTx_data, sizeof(uTx_data)); // for debug
	HAL_UART_Transmit_IT(&huart6, &uTx_data, sizeof(uTx_data)); // gor BT
}

// ============== print_lcd
void printLcd()
{
	//unsigned char line1[20];
	// speed add 2 to trans from [-2, 2] to [0, 4]
	//sprintf(line1, "%c%d SET:%2dcm R%c%d", );
	lcd16x2_i2c_setCursor(0, 0);
	lcd16x2_i2c_printf("%c%dL SET:%2dcm R%c%d",
			            (Speed_L >= 0 ? '+' : '-'), (Speed_L >= 0 ? Speed_L : -Speed_L),
			            limit_range_cm,
			            (Speed_R >= 0 ? '+' : '-'), (Speed_R >= 0 ? Speed_R : -Speed_R)
			          );
	lcd16x2_i2c_setCursor(1, 0);
	lcd16x2_i2c_printf(" GET:%5dcm", (int)distance);
	//lcd16x2_i2c_printf("bzr_value:%5d ", pwmVal);
}

// ============== adc vaule
void getADCValue()
{
  HAL_ADC_Start(&hadc1);
  // Poll ADC1 Perihperal & TimeOut = 1mSec
  HAL_ADC_PollForConversion(&hadc1, 1);
  // Read The ADC Conversion Result
  adc_value = HAL_ADC_GetValue(&hadc1);
  //unsigned char test[50];
  //sprintf((char *)test, "ADC:%d\n\r", adc_value); //GYRO
  //HAL_UART_Transmit_IT(&huart2, (unsigned char *)&test, strlen((const char *)test));
  limit_range_cm = map(adc_value, 0, 4096, 10, 50);
}

void checkSaveDistance()
{
	if((int)distance <= (int)limit_range_cm)
	{
		if(Speed_R > 0) Speed_R = 0;
		if(Speed_L > 0) Speed_L = 0;
	}
}

// ============== set motor
void setMotor(int Speed_L, int Speed_R) // setting motor
{
  //      | left (B) | right(A)
  // -----+----------+-----------
  // GPIO |   PB3    |   PD7
  // PWM  | PB5 (ch2)| PB4 (ch1)
  // PWM Timer: TIM3
  // speed from -2 to +2


  // set left motor  ==> B
  if(Speed_L > 0)
  {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, map(Speed_L, 0, 2, 50, 180));
  }
  else if(Speed_L < 0)
  {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, map(Speed_L, -2, 0, 20, 150));
  }
  else // (Speed_L == 0)
  {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
  }

  // set right motor  ==> A
  if(Speed_R > 0)
  {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, map(Speed_R, 0, 2, 50, 180));
  }
  else if(Speed_R < 0)
  {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, map(Speed_R, -2, 0, 20, 150));
  }
  else // (Speed_R == 0)
  {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
  }
}

void setBuzzer()
{
	// beep range form limit distance to limit distance + 30
	if((int)distance > limit_range_cm + 30)
	{
		pwmVal = 0;
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
	}
	else if ((int)distance < limit_range_cm)
	{
		pwmVal = 200;
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 200);
	}
	else
	{
		pwmVal = map((int)distance, (int)limit_range_cm + 30, (int)limit_range_cm, 0, 100);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal);
	}
	//lcd16x2_i2c_printf("bzr_value:%6d", pwmVal);


}

// ============= map function
int map(int src, int a_1, int a_2, int b_1, int b_2)
{
	if(a_1 == a_2) return 0;
	float m = 1.0 * (b_2 - b_1) / (a_2 - a_1);
	return (int)(m * (src- a_1) + b_1 + 0.5); // 4 out 5 in
}

// ============== get distance
void getDistance()
{

	uint32_t numTicks = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	usDelay(3);

    // START Ultrasonic measure routine
	//1. Output 10 usec TRIG
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	//2. Wait for ECHO pin rising edge
	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_RESET) usDelay(1);
		else break;
	}

	//3. Start measuring ECHO pulse width in usec
	numTicks = 0;
	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_SET)
		{
			numTicks++;
			usDelay(2); //2.8usec
		}
		else break;
	}

	//4. Estimate distance in cm
	distance = numTicks * 2.8 * speedOfSound;


	//lcd16x2_i2c_setCursor(1, 0);
	//lcd16x2_i2c_printf("GET:");
	//lcd16x2_i2c_printf(" GET:%5dcm", (int)distance);

}

// ============ delay us
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

// ============== get gyro and accer
void getGYROandACCR()
{
	BSP_GYRO_GetXYZ(pfData);
	BSP_ACCELERO_GetXYZ(pDataXYZ);
}
/* USER CODE END 4 */
```



## 資料通訊
### H747 按鈕編號
![](https://i.imgur.com/ng731a3.jpg)

### H747-T (F401-R) 格式
* 1-bit 沒有起始/結尾符
* 0~c 對應到左右馬達不同速度(如下表)
### F401-T (H747-R) 格式
* `=LRaaaaabbbbbcccccdddddeeeeefffff@`
    * = 為起始符
    * LR 對應到左右馬達不同速度(如下表回傳編碼)
    * `aaaaa`, `bbbbb`, `ccccc` 為陀螺儀GX, GY, GZ值，經過線性轉換到(0~2^15)
    * `ddddd`, `eeeee`, `fffff` 為加速度AX, AY, AZ值，經過線性轉換到(0~2^15)
    * 以上兩項數值不足五位皆補0
    * @ 為終止符
#### 按鈕(速度)編碼

* 回傳編碼為實際運作編碼+2

| 按鈕(傳輸)編碼 | 實際運作L | 實際運作R | 回傳編碼(L,R) |
|:--------------:|:---------:|:---------:|:-------------:|
|       0        |     0     |     0     |      2 2      |
|       1        |     1     |     1     |      3 3      |
|       2        |     2     |     2     |      4 4      |
|       3        |    -1     |    -1     |      1 1      |
|       4        |    -2     |    -2     |      0 0      |
|       5        |     1     |     0     |      3 2      |
|       6        |     2     |     0     |      4 2      |
|       7        |     2     |     1     |      4 3      |
|       8        |    -2     |    -1     |      0 1      |
|       9        |     0     |     1     |      2 3      |
|       a        |     0     |     2     |      2 4      |
|       b        |     1     |     2     |      3 4      |
|       c        |    -1     |    -2     |      1 0      |


## 心得
本次微處理機實驗課程囊括許多微控制器上的基礎知識/技術/功能，包含：GPIO、中斷(Interrupt)、Timer、PWM、ADC/DAC、UART、I2C、SPI等等，從助教課堂的基礎範例，後延伸進皆練習，讓同學們可以快速掌握知識及能力，搭配期末的整合專案，將所有所學功能整合到小車上，同時在GUI端使用TouchGFX開發，大大減少開發所需的時間成本，可以輕鬆入手軟硬體整合的技能。 

## 實際圖片/操作影片


## 評分

1. (55%) 是否成功使用藍牙控制小車前後左右 (請勾選) :
    - [x] 是 
    - [ ] 否 
2. (25%) 使用到下列哪些功能 (請勾選，一個功能5%) :
    - [x] 可變電阻(ADC)
    - [ ] ADXL345(SPI)
    - [x] 超音波感測器(GPIO) 
    - [x] LCD面板(I2C)
    - [x] 蜂鳴器(PWM)
    - [x] 其他：板載陀螺儀L3GD20(SPI)
    - [x] 其他：板載加速度計LSM303DLHC(I2C)
3. (10%) 請簡單描述小車功能以及GUI的設計 (有創意者分數較佳)。 
    * 可以透過不同按鈕控制車速變化
    * 整合車子回傳的資料動態呈現於GUI介面
    * 可以透過ADC調整防撞臨界距離
    * 蜂鳴器隨臨界距離調整
    * GUI背景為GIF動畫
5. (5%) 程式撰寫。
    * 所有功能使用中斷實現
    * 功能使用function方式實現，後續若需除錯、擴充較為方便
    * `解決HC-SR04模組發生異常時，系統卡死的問題`
6. (5%) 未來展望 (可改進之處，創意發想)。
    * GUI開發流程
    * 利用加速度、陀螺儀資料進一步做分析，得出車體姿態，回傳至GUI呈現
    * 透過加速度計的資料做動力補償
