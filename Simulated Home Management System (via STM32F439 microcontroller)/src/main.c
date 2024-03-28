// GROUP PROJECT CODE

/******************************************************
*			STM32F439 Main (C Startup File)  			          *
*			Developed for the STM32								          *
*			Author: Camden Matthews && Olivia Robertson			*
*			Source File	Produced by: Dr Glenn Matthews			*
******************************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"
#include "gpioControl.h"

void configGPIO(void);
void lightToggle(short toggleEn);
void fanToggle(void);
void tim6Setup(void);
void tim7Setup(void);
void tim9Setup(void);
void tim10Setup(void);
void timer_1sec(void);
void timer_30sec(void);
void configUART(void);
int16_t readADC(void);
void configADC(void);
int buttonCheck_Fan(void);
int buttonCheck_Light(void);
void txUsart(uint8_t, uint8_t);

volatile int lastPassFan = 1;
volatile int currentPassFan = 0;
volatile int checkPassFan = -1;
volatile int currentPassLight = 0;
volatile int lastPassLight = 1;
volatile int checkPassLight = -1;

//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{
	volatile int16_t inputs = 0;
	volatile uint8_t humidity = 0;
	volatile uint8_t lightstatus = 0; 
	volatile uint8_t fanstatus = 0; 
	volatile uint8_t transmit = 0; 
	volatile float humidityCalc = 0; 
	volatile int16_t humidmid = 0;
	volatile int16_t header = 0; 
	volatile int16_t uartVal = 0;
	int fanmask = 0;		
	int lightmask = 0;
	int light = 0; 
	int sensormask = 0;
	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	configGPIO();					//initialise peripherals for use
	configUART();
	configADC();
	tim6Setup();					//used to transmit usart out every second
	tim7Setup();					//used for 30 second fan timer
	tim9Setup();					//button check fan, to check if switch for fan has been pressed validly
	tim10Setup();					//button check light, to check if switch for fan has been pressed validly
	
  while (1)
  {
		//MAIN LOOP CODE
		//================================================================================================
		fanmask = 0;
		lightmask = 0;
		//read in fan switch [PA8] and light switch [PA9] - inputs
		inputs = GPIOA->IDR;
		//0x308 is the mask for those bits including light intensity switch [PA3] - input
		inputs &= 0x308;
		//0x308 IF ALL OFF,		0x00 IF ALL ON 
		//read in humidity [PF10] - input potentiometer
		//convert to percentage
		humidmid = readADC();
		humidityCalc = (humidmid/4095) * 100;
		humidityCalc = humidityCalc + 0.5;
		//rounded humidity percentage
		humidity = (short)humidityCalc;  //casted as short so able to TX via usart later on 
		//------------------------------------------
		//now, UART TAKES PRIORITY
		//if uart RX flag turns on (meaning uart has been received by system), then above inputs ignored and usart instructions take place as below
		if (USART3->SR & USART_SR_RXNE)
		{	
			//check first header byte '!' or 0x21
			header = USART3->DR;
			if (header == 0x21)
			{
				while((USART3->SR & USART_SR_RXNE) == 0);
				uartVal = USART3->DR; 
				//check light control instructions
				light = uartVal;
				light &= (0x8);		//masked
				//light intensity sensor value is ignored for usart
				if (light == 0x8)
				{
					//light control off
					//turn led off so write logic 1 to it 
					GPIOB->ODR |= GPIO_ODR_OD0;
				}
				else 
				{
					//turn on light control SO logic 0
					GPIOB->ODR &= ~(GPIO_ODR_OD0);
				}
				//check fan control instructions
				//fan turn on or off IF humidity <75%, else command ignored
				if(humidity <= 75)
				{
					int fan = uartVal; 
					fan &= (0x4);		//masked 
					if(fan == 0x4)
					{
						//fan control off, led off
						GPIOA->ODR |= (GPIO_ODR_OD10);	
					}
					else 
					{
						//fan control on, led on, LOGIC 0 
						GPIOA->ODR &= ~(GPIO_ODR_OD10);
					}
				}
				
			}
			//else invalid RX data , so ignore

		}
		else		//if no uart rx, continue with regular user input check  =================================================================================
		{
			//FAN CONTROL===================================================================================================================================
			fanmask = inputs;					//mask for light control and intensity sensor
			fanmask &= 0x100;
			if(humidity > 75)
			{
				if((TIM7->SR & TIM_SR_UIF) == 1){
					GPIOA->ODR &= ~(GPIO_ODR_OD10);	//turns fan on
				}
				if(checkPassFan == -1){
					checkPassFan = buttonCheck_Fan();
				}
				else if((checkPassFan == 1) && (fanmask == 0x100)){
					//turn fan off for 30 sec (the fan cannont be turned back on during this time, but if the button is pressed again it will extend the timer back to 30 seconds)
					GPIOA->ODR |= (GPIO_ODR_OD10);
					timer_30sec();
					checkPassFan = -1;
				}
			}
			else{
				if(checkPassFan == -1){
					checkPassFan = buttonCheck_Fan();
				}
				else if((checkPassFan == 1) && (fanmask == 0x100)){
					//turn fan off for 30 sec (the fan cannont be turned back on during this time, but if the button is pressed again it will extend the timer back to 30 seconds)
					GPIOA->ODR ^= (GPIO_ODR_OD10);
					checkPassFan = -1;
				}
			}
		
			//SUBROUTINE FOR LIGHT SWITCH SW3 AND SENSOR SW1-------------------------------------------------------
			lightmask = inputs;					//mask for light control and intensity sensor
			sensormask = inputs;
			lightmask &= 0x200;
			sensormask &= 0x8;
				
			if(checkPassLight == -1){
				checkPassLight = buttonCheck_Light();
			}
			else if((checkPassLight == 1) && (sensormask == 0x8)){
					//turn fan off for 30 sec (the fan cannont be turned back on during this time, but if the button is pressed again it will extend the timer back to 30 seconds)
				GPIOB->ODR ^= GPIO_ODR_OD0;
			}
			else if((checkPassLight == 1) && (sensormask == 0x0)){
				GPIOB->ODR |= GPIO_ODR_OD0;
			}
		}
		if ((TIM6->SR & TIM_SR_UIF) == 1)
		{
		//logic 1 if device on 
		//logic 0 if device off
		//PA10 IS FAN - b
		//PB0 IS LIGHT - a
		//============================================================
		//lastly send out parameters via UART to PC every SECOND . or 1HZ
		lightstatus = (GPIOB->ODR & GPIO_ODR_OD0) << 0x1;
		fanstatus = (GPIOA->ODR & GPIO_ODR_OD10) >> 0x8; 
		transmit = lightstatus | fanstatus;
		transmit ^= 0x6;
		txUsart(humidity, transmit);
		timer_1sec();	
		}
		
	} //end while loop
} //end main loop
void configGPIO(){
	//RCC enable GPIOF, A, B & TIM6, 7, 9, 10 & USART3
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN);
	RCC->APB1ENR |= (RCC_APB1ENR_USART3EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN);
	RCC->APB2ENR |= (RCC_APB2ENR_ADC3EN  | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN);
	__ASM("nop");		__ASM("nop");
	RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOFRST | RCC_AHB1RSTR_GPIOERST | RCC_AHB1RSTR_GPIOBRST);
	RCC->APB1RSTR |= (RCC_APB1RSTR_USART3RST | RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST);
	RCC->APB2RSTR |= (RCC_APB2RSTR_ADCRST | RCC_APB2RSTR_TIM10RST | RCC_APB2RSTR_TIM9RST);
	//PF10 Analog IN
	GPIOF->MODER &= ~(GPIO_MODER_MODER10_Msk);
	GPIOF->MODER |= (0x03 << GPIO_MODER_MODE10_Pos);//analog mode
	//PA3, PA8, PA9 Digital IN
	GPIOA->MODER &= ~(GPIO_MODER_MODE3_Msk);//input mode
	//PA10, PB0 Digital OUT
	GPIOA->MODER &= ~(GPIO_MODER_MODE10_Msk);
	GPIOA->MODER |= (0x01 << GPIO_MODER_MODE10_Pos);//general purpose output mode
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE0_Msk);
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODE0_Pos);
	//PB11, PB10 uart alt func
	GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk);
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos);	 
	GPIOB->MODER &= ~(GPIO_MODER_MODE10_Msk);
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODE10_Pos);
	
	//set up AF
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk); 
	GPIOB->AFR[1] |= ((0x07 << GPIO_AFRH_AFSEL11_Pos)|(0x07 << GPIO_AFRH_AFSEL10_Pos));
}
void fanToggle(){
	GPIOA->ODR ^= GPIO_ODR_OD10;
}
//1 sec timer
void tim6Setup(){
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM6->PSC |= 2600;													//this is the prescaler change this value to aproptiate
	
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM6->ARR |= 31200;												//this is the count for the timer (auto reload register)
	TIM6->CR1 |= TIM_CR1_OPM;									//set the timer to run in single shot mode
	TIM6->CR1 |= TIM_CR1_CEN;									//start timer
	TIM6->CNT |= 31199;												// set count high so the timer will expire right away setting the uif bit(this is nessasary for the way we have done the buttons)
}
//30 sec timer
void tim7Setup(){
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM7->PSC |= 39000;													//this is the prescaler change this value to aproptiate
	
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM7->ARR |= 62400;												//this is the count for the timer (auto reload register)
	TIM7->CR1 |= TIM_CR1_OPM;									//set the timer to run in single shot mode
	TIM7->CR1 |= TIM_CR1_CEN;									//start timer
}
void tim9Setup(){
	TIM9->CR1 &= ~TIM_CR1_CEN;
	TIM9->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM9->PSC |= 2600;													//this is the prescaler change this value to aproptiate
	
	TIM9->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM9->ARR |= 31200;												//this is the count for the timer (auto reload register)
	TIM9->CR1 |= TIM_CR1_OPM;									//set the timer to run in single shot mode
	TIM9->CR1 |= TIM_CR1_CEN;									//start timer
}
void tim10Setup(){
	TIM10->CR1 &= ~TIM_CR1_CEN;
	TIM10->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM10->PSC |= 2600;													//this is the prescaler change this value to aproptiate
	
	TIM10->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM10->ARR |= 31200;												//this is the count for the timer (auto reload register)
	TIM10->CR1 |= TIM_CR1_OPM;									//set the timer to run in single shot mode
	TIM10->CR1 |= TIM_CR1_CEN;									//start timer
}
void timer_1sec_Light(){
	TIM10->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM10->ARR |= 31200;												//this is the count for the timer (auto reload register)
	TIM10->SR &= ~(TIM_SR_UIF);
	TIM10->CR1 |= TIM_CR1_CEN;
}
void timer_1sec_Fan(){
	TIM9->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM9->ARR |= 31200;												//this is the count for the timer (auto reload register)
	TIM9->SR &= ~(TIM_SR_UIF);
	TIM9->CR1 |= TIM_CR1_CEN;
}
void timer_1sec(){
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM6->ARR |= 31200;												//this is the count for the timer (auto reload register)
	TIM6->SR &= ~(TIM_SR_UIF);
	TIM6->CR1 |= TIM_CR1_CEN;
	//while((TIM6->SR & TIM_SR_UIF) == 0);
	//TIM6->CNT++;
}
void timer_30sec(){
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);					//clear the auto reload register (count)
	TIM7->ARR |= 62400;												//this is the count for the timer (auto reload register)
	TIM7->SR &= ~(TIM_SR_UIF);
	TIM7->CR1 |= TIM_CR1_CEN;
	//while((TIM6->SR & TIM_SR_UIF) == 0);
	//TIM6->CNT++;
}
void configUART(){
	//115500bps  8 data bits 1 stop
	//22.727272
	
	//OVER 16 = Enable| Data bits=8 | no parity
	USART3->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_M | USART_CR1_PCE);
	//STOP bits = 1 | no clock
	USART3->CR2 &= ~(USART_CR2_STOP_Msk | USART_CR2_CLKEN);
	//BRR 22.727272
	USART3->BRR &= 0xFFFF0000;
	USART3->BRR |= (0x16 << USART_BRR_DIV_Mantissa_Pos) | (0xC << USART_BRR_DIV_Fraction_Pos);
	//disable hardwareflow
	USART3->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	//enable usart transmit and receive
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
}

int16_t readADC()
{
	int16_t rawADC;
	//START ADC CONVERSION 
	ADC3->CR2 |= ADC_CR2_SWSTART;

	//wait 
	while((ADC3->SR & ADC_SR_EOC) == 0);
	
	//now get value 
	rawADC = (ADC3->DR & 0x00000FFF);	//MASKED OUT RESERVED BITS 
	
	return rawADC;
}
void configADC()
{
	//peripheral already enabled
	//12-bit resolution for 4096 AND STOP SCAN MODE 
	ADC3->CR1 &= ~((ADC_CR1_SCAN) | (0X03 << ADC_CR1_RES_Pos));

	//single channel, ADC3_IN8 according to data sheet
	ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
	ADC3->SQR3 |= 0x8;
	ADC3->SQR1 &= ~(ADC_SQR1_L_Msk);			//indicating that only 1 channel to be converted 
	
	//sample time - 56cycles 
	ADC3->SMPR2 &= ~(ADC_SMPR2_SMP8_Msk);
	ADC3->SMPR2 |= 0x03 << (ADC_SMPR2_SMP8_Pos);
	
	//right alignment
	ADC3->CR2 &= ~(ADC_CR2_ALIGN | ADC_CR2_CONT);
	
	//enabling adc
	ADC3->CR2 |= ADC_CR2_ADON;
}

void txUsart(uint8_t humidity, uint8_t transmit)
{
	// light a - bit 1 logic 0 or logic 1
	//fan b - bit 0 logic 0 or logic 1
	int16_t temp = 0;
	// wait for transceiver to be finished 
	while((USART3->SR & USART_SR_TXE) == 0);
	
	//write data to usart reg
	USART3->DR = 0x21;			//this is the header byte
	
	//WAIT for transmission to be complete 
	while((USART3->SR & USART_SR_TC) == 0x00);
	temp = USART3->DR;
	//===========================================
	// wait for transceiver to be finished 
	while((USART3->SR & USART_SR_TXE) == 0);

	//write data to usart reg
	USART3->DR = humidity;			//this is the humidity value, 8bit
	
	//WAIT for transmission to be complete 
	while((USART3->SR & USART_SR_TC) == 0x00);
	temp = USART3->DR;

	//===========================================
	// wait for transceiver to be finished 
	while((USART3->SR & USART_SR_TXE) == 0);
	
	//write data to usart reg
	USART3->DR = transmit;			//this is the fan and light status
	
	//WAIT for transmission to be complete 
	while((USART3->SR & USART_SR_TC) == 0x00);
	temp = USART3->DR;

}


int buttonCheck_Fan(){
	currentPassFan = (GPIOA->ODR & 0x100);
	if((lastPassFan == 0) && (currentPassFan == 0)){
		if((TIM9->SR & TIM_SR_UIF) == 1){
			checkPassFan = 1;
		}
	}
	else if((lastPassFan == 1) && (currentPassFan == 0)){
		checkPassFan = -1;
		timer_1sec_Fan();
	}
	else{
		checkPassFan = -1;
	} 
	lastPassFan = currentPassFan;
	return checkPassFan;
}

int buttonCheck_Light(){
	currentPassLight = (GPIOA->ODR & 0x200);
	if((lastPassLight == 0) && (currentPassLight == 0)){
		if((TIM10->SR & TIM_SR_UIF) == 1){
			checkPassLight = 1;
		}
	}
	else if((lastPassLight == 1) && (currentPassLight == 0)){
		checkPassLight = -1;
		timer_1sec_Light();
	}
	else{
		checkPassLight = -1;
	} 
	lastPassLight = currentPassLight;
	return checkPassLight;
}

