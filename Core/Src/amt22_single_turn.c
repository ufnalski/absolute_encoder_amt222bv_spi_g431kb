/*
 AMT22.c - STM32 library for ATM22 series absolute encoders by CUI Devices.
 Created by Simone Di Blasi, December 2020.
 Non-blocking mode added by Bartlomiej Ufnalski, July 2024.
 */

#include "amt22_single_turn.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"

// https://electronics.stackexchange.com/questions/161967/stm32-timer-interrupt-works-immediately
// #define FIX_TIMER_INTERRUPT_BUG(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))
// TIM_SR_UIF = TIM_FLAG_UPDATE
// #define FIX_TIMER_INTERRUPT_BUG(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_FLAG_UPDATE))
TIM_HandleTypeDef *some_tim_ptr;
#define FIX_TIMER_INTERRUPT_BUG(handle_ptr) (some_tim_ptr = handle_ptr) // do nothing - the bug seems to be fixed

volatile uint8_t t_clk_to_be_waited;
volatile uint8_t t_b_to_be_waited;
volatile uint8_t t_r_to_be_waited;
volatile uint8_t t_cs_to_be_waited;
volatile uint8_t ready_to_be_read = 1;
volatile uint8_t ready_for_byte_0;
volatile uint8_t ready_for_byte_1;

uint8_t sendByte0 = AMT22_NOP;
uint8_t sendByte1 = AMT22_NOP;

volatile uint8_t cui_rx_byte_0;
volatile uint8_t cui_rx_byte_1;

void setCSLine(GPIO_TypeDef *encoderPort, uint16_t encoderPin,
		GPIO_PinState csLine)
{
	HAL_GPIO_WritePin(encoderPort, encoderPin, csLine);
}

uint8_t spiWriteRead(SPI_HandleTypeDef *hspi, uint8_t sendByte,
		GPIO_TypeDef *encoderPort, uint16_t encoderPin, uint8_t releaseLine,
		TIM_HandleTypeDef *timer)
{
	//to hold received data
	uint8_t data;

	//set cs low, cs may already be low but there's no issue calling it again except for extra time
	setCSLine(encoderPort, encoderPin, GPIO_PIN_RESET);

	//There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
	delay(timer, TIM_COUNT_2_5_US); // 2.5 us
	//send the command and receive response of the slave

	HAL_SPI_TransmitReceive(hspi, &sendByte, &data, 1, 10);

	//There is also a minimum time after clocking that CS should remain asserted before we release it
	delay(timer, TIM_COUNT_3_US); // 3 us
	setCSLine(encoderPort, encoderPin, releaseLine); //if releaseLine is high set it high else it stays low

	return data;
}

uint8_t spiWriteRead0(SPI_HandleTypeDef *hspi, uint8_t sendByte,
		GPIO_TypeDef *encoderPort, uint16_t encoderPin,
		TIM_HandleTypeDef *timer)
{
	//to hold received data
	uint8_t data;

	//set cs low, cs may already be low but there's no issue calling it again except for extra time
	setCSLine(encoderPort, encoderPin, GPIO_PIN_RESET);

	//There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
	delay(timer, TIM_COUNT_2_5_US);  // 2.5 us
	//send the command and receive response of the slave
	HAL_SPI_TransmitReceive(hspi, &sendByte, &data, 1, 10);

	HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,
			GPIO_PIN_SET);

	return data;
}

uint8_t spiWriteRead1(SPI_HandleTypeDef *hspi, uint8_t sendByte,
		GPIO_TypeDef *encoderPort, uint16_t encoderPin,
		TIM_HandleTypeDef *timer)
{
	//to hold received data
	uint8_t data;

	//There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
	delay(timer, TIM_COUNT_2_5_US);  // 2.5 us

	HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,
			GPIO_PIN_RESET);

	//send the command and receive response of the slave
	HAL_SPI_TransmitReceive(hspi, &sendByte, &data, 1, 10);

	//There is also a minimum time after clocking that CS should remain asserted before we release it
	delay(timer, TIM_COUNT_3_US);  // 3 us
	setCSLine(encoderPort, encoderPin, 1); //if releaseLine is high set it high else it stays low

	return data;
}

uint16_t getPositionSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,
		uint16_t encoderPin, uint8_t resolution, TIM_HandleTypeDef *timer)
{
	uint16_t currentPosition;       //16-bit response from encoder
	uint8_t binaryArray[16]; //after receiving the position we will populate this array and use it for calculating the checksum

	//get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
	currentPosition = spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0,
			timer) << 8;

	//OR the low byte with the currentPosition variable. release line after second byte
	currentPosition |= spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 1,
			timer);

	//run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
	for (int i = 0; i < 16; i++)
		binaryArray[i] = (0x01) & (currentPosition >> (i));

	//using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
	if ((binaryArray[15]
			== !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9]
					^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3]
					^ binaryArray[1]))
			&& (binaryArray[14]
					== !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8]
							^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2]
							^ binaryArray[0])))
	{
		//we got back a good position, so just mask away the checkbits
		currentPosition &= 0x3FFF;
	}
	else
	{
		currentPosition = 0xFFFF; //bad position
	}

	//If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
	if ((resolution == AMT22_RES_12) && (currentPosition != 0xFFFF))
		currentPosition = currentPosition >> 2;
	return currentPosition;
}

uint16_t getPositionSPI_shortest_blocking_time(SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *encoderPort, uint16_t encoderPin, uint8_t resolution,
		TIM_HandleTypeDef *timer)
{
	uint16_t currentPosition;       //16-bit response from encoder
	uint8_t binaryArray[16]; //after receiving the position we will populate this array and use it for calculating the checksum

	//get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
	currentPosition = spiWriteRead0(hspi, AMT22_NOP, encoderPort, encoderPin,
			timer) << 8;

	//OR the low byte with the currentPosition variable. release line after second byte
	currentPosition |= spiWriteRead1(hspi, AMT22_NOP, encoderPort, encoderPin,
			timer);

	//run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
	for (int i = 0; i < 16; i++)
		binaryArray[i] = (0x01) & (currentPosition >> (i));

	//using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
	if ((binaryArray[15]
			== !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9]
					^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3]
					^ binaryArray[1]))
			&& (binaryArray[14]
					== !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8]
							^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2]
							^ binaryArray[0])))
	{
		//we got back a good position, so just mask away the checkbits
		currentPosition &= 0x3FFF;
	}
	else
	{
		currentPosition = 0xFFFF; //bad position
	}

	//If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
	if ((resolution == AMT22_RES_12) && (currentPosition != 0xFFFF))
		currentPosition = currentPosition >> 2;
	return currentPosition;
}

void setZeroSPI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,
		uint16_t encoderPin, TIM_HandleTypeDef *timer)
{
	spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer);

	spiWriteRead(hspi, AMT22_ZERO, encoderPort, encoderPin, 1, timer);

	delay(timer, 2000); // 250 us
}

void resetAMT22(SPI_HandleTypeDef *hspi, GPIO_TypeDef *encoderPort,
		uint16_t encoderPin, TIM_HandleTypeDef *timer)
{
	spiWriteRead(hspi, AMT22_NOP, encoderPort, encoderPin, 0, timer);

	spiWriteRead(hspi, AMT22_RESET, encoderPort, encoderPin, 1, timer);

	delay(timer, 2000);  // 250 us
}

void delay(TIM_HandleTypeDef *timer, uint32_t delayTime)
{
	uint32_t startTime = __HAL_TIM_GET_COUNTER(timer); //reference point to count passed time
	uint32_t passedTime = 0;

	while (passedTime < delayTime)
	{
		passedTime = __HAL_TIM_GET_COUNTER(timer) - startTime;
		if (passedTime < 0)
		{
			passedTime += timer->Init.Period;
		}
	}
}

// For non-blocking mode
void getEncoderRawDataSPI_non_blocking(void)
{
	setCSLine(AMT222BV_CS_GPIO_Port, AMT222BV_CS_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,
			GPIO_PIN_SET);

	__HAL_TIM_SET_AUTORELOAD(&htim8, TIM_COUNT_2_5_US);
	__HAL_TIM_SET_COUNTER(&htim8, 0);

	t_clk_to_be_waited = 1;
	FIX_TIMER_INTERRUPT_BUG(&htim8);

	HAL_TIM_Base_Start_IT(&htim8);
}

uint16_t getPosition(uint8_t resolution)
{
	uint16_t cuiPosition;
	uint8_t cuiArray[16];

	cuiPosition = cui_rx_byte_0 << 8;
	cuiPosition |= cui_rx_byte_1;

	//run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
	for (int i = 0; i < 16; i++)
		cuiArray[i] = (0x01) & (cuiPosition >> (i));

	//using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
	if ((cuiArray[15]
			== !(cuiArray[13] ^ cuiArray[11] ^ cuiArray[9] ^ cuiArray[7]
					^ cuiArray[5] ^ cuiArray[3] ^ cuiArray[1]))
			&& (cuiArray[14]
					== !(cuiArray[12] ^ cuiArray[10] ^ cuiArray[8] ^ cuiArray[6]
							^ cuiArray[4] ^ cuiArray[2] ^ cuiArray[0])))
	{
		//we got back a good position, so just mask away the checkbits
		cuiPosition &= 0x3FFF;
	}
	else
	{
		cuiPosition = 0xFFFF; //bad position
	}

	if ((resolution == AMT22_RES_12) && (cuiPosition != 0xFFFF))
		cuiPosition = cuiPosition >> 2;
	return cuiPosition;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM8)
	{
		if (t_clk_to_be_waited == 1)
		{
			t_clk_to_be_waited = 0;
			ready_for_byte_0 = 1;
			HAL_TIM_Base_Stop_IT(&htim8);

			HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,
					GPIO_PIN_RESET);

			HAL_SPI_TransmitReceive_IT(&hspi1, &sendByte0,
					(uint8_t*) (&cui_rx_byte_0), 1);

		}
		else if (t_b_to_be_waited == 1)
		{
			t_b_to_be_waited = 0;
			ready_for_byte_1 = 1;
			HAL_TIM_Base_Stop_IT(&htim8);

			HAL_SPI_TransmitReceive_IT(&hspi1, &sendByte1,
					(uint8_t*) (&cui_rx_byte_1), 1);

		}
		else if (t_r_to_be_waited == 1)
		{
			t_r_to_be_waited = 0;
			HAL_TIM_Base_Stop_IT(&htim8);
			setCSLine(AMT222BV_CS_GPIO_Port, AMT222BV_CS_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_AUTORELOAD(&htim8, TIM_COUNT_40_US);
			__HAL_TIM_SET_COUNTER(&htim8, 0);

			t_cs_to_be_waited = 1;
			FIX_TIMER_INTERRUPT_BUG(&htim8);
			HAL_TIM_Base_Start_IT(&htim8);
		}
		else if (t_cs_to_be_waited == 1)
		{
			t_cs_to_be_waited = 0;
			ready_to_be_read = 1;
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		if (ready_for_byte_0 == 1)
		{
			ready_for_byte_0 = 0;
			__HAL_TIM_SET_AUTORELOAD(&htim8, TIM_COUNT_2_5_US);
			__HAL_TIM_SET_COUNTER(&htim8, 0);

			t_b_to_be_waited = 1;
			FIX_TIMER_INTERRUPT_BUG(&htim8);
			HAL_TIM_Base_Start_IT(&htim8);
		}
		else if (ready_for_byte_1 == 1)
		{
			ready_for_byte_1 = 0;
			__HAL_TIM_SET_AUTORELOAD(&htim8, TIM_COUNT_3_US);
			__HAL_TIM_SET_COUNTER(&htim8, 0);

			t_r_to_be_waited = 1;
			FIX_TIMER_INTERRUPT_BUG(&htim8);
			HAL_TIM_Base_Start_IT(&htim8);
		}
	}
}
