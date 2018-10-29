/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"
#include "stm32f1xx_nucleo.h"
#include <stdio.h>

#define portW1 GPIOA
#define wire1 GPIO_Pin_7

void delay(uint16_t time)
{
	TIM_SetCounter(TIM2,0); //zerowanie licznika
	while (TIM_GetCounter(TIM2)<=time); //odczyt CNT i porównanie
}

void delayMicroSec(__IO uint32_t microSec)
{
	microSec *= (SystemCoreClock / 1000000) / 8;
	while (microSec--) ;
}

uint16_t RESET_PULSE(void)
{
	uint16_t PRESENCE=0;
	GPIO_ResetBits(portW1, wire1); //ustawienie stanu niskiego
	delay(480); //odczekanie 480us
	GPIO_SetBits(portW1, wire1); //odpuszczenie do stanu wysokiego
	delay(70); //odczekanie ok. 70us na reakcje czujnika

	//sprawdzenie odpowiedzi
	if (GPIO_ReadInputDataBit(portW1, wire1)==Bit_RESET)
		PRESENCE++;
	delay(410);

	if (GPIO_ReadInputDataBit(portW1, wire1)==Bit_SET)
		PRESENCE++;

	if (PRESENCE == 2)
		return 1;
	else
		return 0;
}


void SendBit(uint16_t bit)
{
	if (bit==0)
	{
		GPIO_ResetBits(portW1, wire1);
		delay(65);
		GPIO_SetBits(portW1, wire1);
		delay(10);
	}
	else
	{
		GPIO_ResetBits(portW1, wire1);
		delay(10);
		GPIO_SetBits(portW1, wire1);
		delay(65);
	}
}

uint16_t ReadBit(void)
{
	uint16_t bit=0;
	GPIO_ResetBits(portW1, wire1);
	delay(5);
	GPIO_SetBits(portW1, wire1);
	delay(5);

	//warunek sprawdzający czy zostało wysłane 1 czy 0
	if (GPIO_ReadInputDataBit(portW1, wire1)==Bit_SET)
		bit=1;
	else
		bit=0;

	delay(55);  	//przerwa czasowa między bitami
	return bit;
}


void SendByte(uint16_t value)
{
	uint16_t i,tmp;
	for (i = 0; i < 8; i++)
	{
		tmp = value >> i; //przesunięcie bitowe
		tmp &= 0x01; //wyodrębnienie bitu do wysłania
		SendBit(tmp); //wysłanie bitu
	}
}

uint8_t ReadByte(void)
{
	uint8_t i,value=0;
	for (i = 0; i < 8; i++)
	{
		if(ReadBit()) //odczyt linii danych
			value |= 0x01 << i; //tworzenie ośmiobitowej liczby
	}
	return value;
}

float ds18b20GetTemp(uint16_t data)
{
    float t;
    /* delete all not needed data */
    t = (float)((data & 0x07FF)>>4);
    /* add fraction part */
    t += (float)(data & 0x000F) / 16.0f;
    return t;
}

uint8_t ds18b20ReadSign(uint16_t data)
{
    if ( data & (1<<11))    {   return 1;   }
    else                    {   return 0;   }
}

float ReadTemp(void)
{
	uint16_t i, presence=0;
	uint8_t data[8];
	float temp=0;

	presence=RESET_PULSE();

	if (presence==1)
	{
		SendByte(0xCC); //Skip ROM
		SendByte(0x44); //Convert T

		for (i=0; i < 100; i++) //odczekanie 750ms na odczyt i konwersję
		delay(7500); //temperatury
	}

	presence = RESET_PULSE();

	if (presence==1)
	{
		SendByte(0xCC);	//Skip ROM
		SendByte(0xBE);	//Read Scratchpad

		for (i=0;i<8;i++)
			data[i] = ReadByte(); 		//odczyt 2 bajtów Scratchpad

	    uint16_t readDataRaw = ((uint16_t)data[1]<<8) | data[0];

	    temp = ds18b20GetTemp(readDataRaw);


		/*memory[2] = (240U & memory[1]) >>  7;
		memory[1] = (15U & memory[2] ) <<  8;

		if (memory[2] == 0)		//jeśli dodatnia temperatura
			temp = (memory[0] + memory[1])/16;
		if (memory[2] == 1)		//jeśli ujemna temperatura
			temp = (memory[0] + memory[1]-4095)/16;*/
	}

	presence = RESET_PULSE();
	return temp;
}

void send_char(char c)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

void send_string(const char* s)
{
    while (*s)
        send_char(*s++);
}

int main(void)
{

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE); //włączenie zegara

	TIM_TimeBaseInitTypeDef TIM_InitStruct; //struktura inicjalizacyjna

	TIM_InitStruct.TIM_ClockDivision = 0; //dzielnik 0
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up; // licznik w gore
	TIM_InitStruct.TIM_Period = 65535; // okres licznika
	TIM_InitStruct.TIM_Prescaler = 72; // preskaler 72
	TIM_TimeBaseInit(TIM2, &TIM_InitStruct); // inicjalizacja TIM2
	TIM_Cmd(TIM2, ENABLE);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef gpio;  // obiekt gpio będący konfiguracją portów GPIO

	GPIO_StructInit(&gpio);  // domyślna konfiguracja
	gpio.GPIO_Pin = GPIO_Pin_7;  // konfigurujemy pin 5
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;  // jako wyjście
	GPIO_Init(GPIOA, &gpio);  // inicjalizacja modułu GPIOA


	//Sterowanie grzalka
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &gpio);

	//USART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	USART_InitTypeDef uart;

	USART_StructInit(&uart);
	uart.USART_BaudRate = 115200;
	USART_Init(USART2, &uart);

	USART_Cmd(USART2, ENABLE);

	//silnik

	  TIM_TimeBaseInitTypeDef tim;
	  NVIC_InitTypeDef nvic;
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Prescaler = 64 - 1;
    tim.TIM_Period = 20000;
    TIM_TimeBaseInit(TIM4, &tim);

    TIM_OCInitTypeDef channel;

    TIM_OCStructInit(&channel);
    channel.TIM_OCMode = TIM_OCMode_PWM1;
    channel.TIM_OutputState = TIM_OutputState_Enable;
    channel.TIM_Pulse = 1200;
    TIM_OC1Init(TIM4, &channel);
    channel.TIM_Pulse = 1100;
    TIM_OC2Init(TIM4, &channel);

 //   TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  // TIM_Cmd(TIM2, ENABLE);

  //  nvic.NVIC_IRQChannel = TIM2_IRQn;
 //   nvic.NVIC_IRQChannelPreemptionPriority = 0;
 //   nvic.NVIC_IRQChannelSubPriority = 0;
 //   nvic.NVIC_IRQChannelCmd = ENABLE;
 //   NVIC_Init(&nvic);
    TIM_Cmd(TIM4, ENABLE);



    while (1) {
    	float t = ReadTemp();
    	if (t < 64) {
    		GPIO_SetBits(GPIOC, 1);
    	} else {
    		GPIO_ResetBits(GPIOC, 1);
    	}
    	char buffer[10];
    	sprintf(buffer, "%f\n\r", t);
    	send_string(buffer);

    }
}
