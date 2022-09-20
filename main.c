// Circuit with 100k thermistor (R1) and
// 1k resistor (R2) voltage divider

#include "init.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include "math.h"

//global handle
ADC_HandleTypeDef hadc1;

//lcd frame buffer
#define LCD_FRAME_BUFFER	0xC0000000
//touch screen
TS_StateTypeDef  TS_State = {0};

double temp;
uint32_t adcValue;
float voltage;
uint8_t screen_on = 1;
uint8_t get_temp = 0;

char msg[20];

#define adcVals 100 //keep up to x temp readings in window history
#define adcReadings 100 //avg x readings to get a temp

float nums[adcVals]; //storage of temp readings
float set_temp = 205; //default temp to be set to
float set_motor = 0;

//timer variables
TIM_HandleTypeDef timer7;
TIM_HandleTypeDef timer3_pwm;

//initialize the timer, which will serve to control the stepper motor states
void Init_Timer() {
	//Enable the timer 7 for getting new temps every x sec
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
	// Enable TIM7 clock
	__HAL_RCC_TIM7_CLK_ENABLE();

	//set up the settings
	timer7.Instance = TIM7;
	timer7.Init.Prescaler = 10800; //scales down to 10khz
	timer7.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer7.Init.Period = 5000; //overflow every 0.5s
	timer7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timer7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_Base_Init(&timer7);
	HAL_TIM_Base_Start_IT(&timer7);

	//enable the timer 3 for PWM
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	// Enable TIM3 clock
	__HAL_RCC_TIM3_CLK_ENABLE();

	//set up the pwm settings
	timer3_pwm.Instance = TIM3;
	timer3_pwm.Init.Prescaler = 10800; //scales down to 100khz
	timer3_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer3_pwm.Init.Period = pow(2,8)-1; // 100k/pow(2,8) = pwm signal freq is 40Hz
	timer3_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timer3_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

	HAL_TIM_Base_Init(&timer3_pwm);
	HAL_TIM_Base_Start_IT(&timer3_pwm);
}

//graph starts/ends:
uint16_t xstart=20;
uint16_t xend=540;
uint16_t ystart=40;
uint16_t yend=435;

void draw_graph()
{
	//clear previous graph
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_FillRect(xstart, ystart, xend-xstart, yend-ystart);

	//graph border
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_DrawRect(xstart, ystart, xend-xstart, yend-ystart);

	//find data min and max
	float localMin=500;
	float localMax=0;
	for(uint8_t i=0;i<adcVals;i++)
	{
		if(nums[i]==0){continue;}
		if(nums[i]>localMax)
		{
			localMax=nums[i];
		}
		if(nums[i]<localMin)
		{
			localMin=nums[i];
		}
	}

	localMax += 5;
	localMin -= 5;

	//display min and max vals
	gcvt(localMin, 5, msg);
	BSP_LCD_DisplayStringAt(xstart, yend, msg, LEFT_MODE);

	gcvt(localMax, 5, msg);
	BSP_LCD_DisplayStringAt(xstart, ystart, msg, LEFT_MODE);

	//draw lines that linearly connect the i to i+1
	//value every [xstart:xend:(xend-xstart)/adcVals]
	//y is placed between the min and max of the set

	float localRange = localMax-localMin;

	for(uint8_t i=0;i<adcVals-1;i++)
	{
		if(nums[i]==0){continue;}
		BSP_LCD_DrawLine(
				(xend-xstart)/adcVals * i + xstart,
				yend - ((nums[i] -localMin) / (localRange)) * (yend-ystart),
				(xend-xstart)/adcVals * (i+1) + xstart,
				yend - ((nums[i+1] -localMin) / (localRange)) * (yend-ystart)
				);
	}
}



//Used for configuring the ADC settings
void configureADC()
{
	// Enable the ADC Clock.
	__HAL_RCC_ADC1_CLK_ENABLE();

	//use ADC1 for this instance
	hadc1.Instance = ADC1;
	//decrease clock
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	//use full 12 bit resolution
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	//select  single conversion as a end of conversion event
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	//right justified output
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;

	// Initialize the ADC
	HAL_ADC_Init(&hadc1);


	ADC_ChannelConfTypeDef* sConfig1;
	//select analog channel 6 (for PA6)
	sConfig1->Channel = ADC_CHANNEL_6;
	//set the rank to 1
	sConfig1->Rank = ADC_REGULAR_RANK_1;
	//sample over the course of 480 clock cycles
	sConfig1->SamplingTime = ADC_SAMPLETIME_480CYCLES;

	// Configure the ADC channel
	HAL_ADC_ConfigChannel(&hadc1, sConfig1);
}


void configurePWM()
{
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_Init(&timer3_pwm);
	HAL_TIM_PWM_ConfigChannel(&timer3_pwm, &sConfigOC, TIM_CHANNEL_2); //heater
	HAL_TIM_PWM_ConfigChannel(&timer3_pwm, &sConfigOC, TIM_CHANNEL_1); //re use configuration for motor
}


void mainScreen()
{
	BSP_LCD_SelectLayer(0);
	BSP_LCD_SetTransparency(0, 255);
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetColorKeying(0, 000000);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    // temperature:
    // + rectangle
    BSP_LCD_FillRect(570, 150, 100, 100);
    // - rectangle
    BSP_LCD_FillRect(680, 150, 100, 100);

    // motor control
    // + rectangle
    BSP_LCD_FillRect(570, 300, 100, 100);
    // - rectangle
    BSP_LCD_FillRect(680, 300, 100, 100);


    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DisplayStringAt(610, 190, "+", LEFT_MODE);
    BSP_LCD_DisplayStringAt(720, 190, "-", LEFT_MODE);

    BSP_LCD_DisplayStringAt(610, 340, "+", LEFT_MODE);
    BSP_LCD_DisplayStringAt(720, 340, "-", LEFT_MODE);
}


void updateScreen()
{
	//replace current temp
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);

	//current temp:
	BSP_LCD_DisplayStringAt(570, 30, "Temperature: ", LEFT_MODE);
	gcvt(temp, 5, msg);
	BSP_LCD_DisplayStringAt(570, 60, msg, LEFT_MODE);

	//set temp to:
	BSP_LCD_DisplayStringAt(570, 90, "Set to: ", LEFT_MODE);
	gcvt(set_temp, 5, msg);
	BSP_LCD_DisplayStringAt(570, 120, msg, LEFT_MODE);

	//set motor to:
	BSP_LCD_DisplayStringAt(570, 270, "Motor: ", LEFT_MODE);
	gcvt(set_motor, 3, msg);
	BSP_LCD_DisplayStringAt(680, 270, msg, LEFT_MODE);

	draw_graph();
}


void Init_GPIO(void)
{
	//pushbutton setup (PA0)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef pushbutton;
	pushbutton.Pin = GPIO_PIN_0;
	pushbutton.Mode = GPIO_MODE_IT_RISING;
	pushbutton.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &pushbutton);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);

	// Heater - PC7 - TIM3_CH2 - D0
	// Motor -  PC6 - TIM3_CH1 - D1
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef heater;
	heater.Pin = GPIO_PIN_7 | GPIO_PIN_6;
	heater.Mode = GPIO_MODE_AF_PP; //set to alternate function to use the TIM3 PWM
	heater.Pull = GPIO_NOPULL;
	heater.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	heater.Alternate = GPIO_AF2_TIM3; //important, set alternate function to timer3 to link PWM functionality
	HAL_GPIO_Init(GPIOC, &heater);
}

int main(void)
{
	Sys_Init();
	printf("\033[2J\033[;H"); // Erase screen & move cursor to home position
	fflush(stdout); // Need to flush stdout after using printf that doesn't end in \n

	configureADC();

	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
	BSP_LCD_SetBrightness(80);

	//initialize timer and heater/motor/pushbutton
	Init_Timer();
	Init_GPIO();

    uint32_t x_size = BSP_LCD_GetXSize();
    uint32_t y_size = BSP_LCD_GetYSize();
    printf("Screen: X size: %d | Y size: %d\r\n", x_size, y_size);

    //touchscreen stuff:
    uint16_t x, y;
	uint8_t ts_status;
	uint8_t BSP_TS_ITConfig();

	//configure PWM to control the heater(ch2) and the motor (ch1)
	configurePWM();

	HAL_TIM_PWM_Start_IT(&timer3_pwm, TIM_CHANNEL_1); // Motor
	HAL_TIM_PWM_Start_IT(&timer3_pwm, TIM_CHANNEL_2); // Heater

	/* Touchscreen initialization */
	ts_status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	while(ts_status != TS_OK){;} //wait for setup

	//print static details to main screen such as buttons
	mainScreen();

	//set default PWM to zero to start
	//heater:
	__HAL_TIM_SET_COMPARE(&timer3_pwm, TIM_CHANNEL_2, 0);
	//motor:
	__HAL_TIM_SET_COMPARE(&timer3_pwm, TIM_CHANNEL_1, 0);


	float PID_p = 40;
	float PID_d = 75;
	float PID_c = 55;

	char input = ' ';

	while(1)
	{
		HAL_UART_Receive(&USB_UART, &input, 1, 10);
		if(input=='p')
		{
			printf("enter p now \r\n");
			char input1 = getchar();
			char input2 = getchar();
			PID_p = (input1-48)*10 + (input2-48);
			input=0;
		}
		if(input=='d')
		{
			printf("enter d now \r\n");
			char input1 = getchar();
			char input2 = getchar();
			PID_d = (input1-48)*10 + (input2-48);
			input=0;
		}
		if(input=='c')
		{
			printf("enter c now \r\n");
			char input1 = getchar();
			char input2 = getchar();
			PID_c = (input1-48)*10 + (input2-48);
			input=0;
		}
		if(screen_on)
		{
			ts_status = BSP_TS_GetState(&TS_State);
			if(ts_status==TS_OK && TS_State.touchDetected==1)
			{ //touch detected - find what state we are in and check position
				x = TS_State.touchX[0];
				y = TS_State.touchY[0];
				//temp control:
				printf("Touched at %d %d with %d touches\r\n",x,y, TS_State.touchDetected);
				if(x>570 && x<570+100 && y>150 && y<150+100 && set_temp<300)
				{
					set_temp+=1;
				}
				if(x>680 && x<680+100 && y>150 && y<150+100 && set_temp>0)
				{
					set_temp-=1;
				}
				//motor control:
				if(x>570 && x<570+100 && y>340 && y<340+100 && set_motor<9)
				{
					set_motor+=1;
				}
				if(x>680 && x<680+100 && y>340 && y<340+100 && set_motor>0)
				{
					set_motor-=1;
				}
				updateScreen();
				//set motor PWM speed by the given number:
				__HAL_TIM_SET_COMPARE(&timer3_pwm, TIM_CHANNEL_1, (uint32_t)(set_motor * 25.5));
				HAL_Delay(100);
			}
		}
		if(get_temp)
		{
			temp=0;
			adcValue=0;
			for(int i=0;i<adcReadings;i++)
			{
				HAL_ADC_Start(&hadc1);
				if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
				{
					adcValue += HAL_ADC_GetValue(&hadc1);
				}
			}
			adcValue/=adcReadings;

			//voltage of the ther
			double VrT = (adcValue * 3.3) / (pow(2,12)-1);

			//resistance of ther = VrT / IrT
			double rT = VrT/((3.3-VrT)/500);

			temp = 0.016688106508573*pow(logf(rT),4) +
					-0.738770776626323*pow(logf(rT),3) +
					14.221589761264440*pow(logf(rT),2) +
					-157.9745671986452*logf(rT) +
					792.9224875639983;

			//push back previous temperature vals for graph
			for(uint8_t i=0;i<adcVals-1;i++)
			{
				nums[i]=nums[i+1];
			}


			if(screen_on)
			{
				updateScreen();
			}

			printf("%f\r\n", temp);
			printf("PID: dp%f\t\td:%f\tc:%f\r\n", PID_p,PID_d,PID_c);

//			/*
			float error = (set_temp - temp);
			//change in temp from a previous sample
			float error_derivative = (nums[adcVals-2] - temp);

			float adc_res = error*PID_p + 10.0*error_derivative*PID_d + PID_c;

			if(adc_res<0)
				adc_res=0;
			if(adc_res>255)
				adc_res=255;

			__HAL_TIM_SET_COMPARE(&timer3_pwm, TIM_CHANNEL_2, (uint32_t)adc_res);

			nums[adcVals-1] = temp; //store this temp as newest val in history
			get_temp=0; //reset flag
		}
	}
}


//used for initializing the ADC GPIO pins
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		//utilize pin PA6 for ADC1 conversion - Arduino analog pin A0
		__HAL_RCC_GPIOA_CLK_ENABLE(); // enable clock to GPIOA
		GPIO_InitTypeDef ADCpin;
		ADCpin.Pin = GPIO_PIN_6; // Select pin PA6
		ADCpin.Mode = GPIO_MODE_ANALOG; // Select Analog Mode
		ADCpin.Pull = GPIO_NOPULL; // Disable internal resistors
		HAL_GPIO_Init(GPIOA, &ADCpin);
	}
}


//callback function for the IRQ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//flag to get temp readings
	if(htim == &timer7)
	{
		get_temp=1;
	}
}

//HAL timer7
void TIM7_IRQHandler()
{
	HAL_TIM_IRQHandler(&timer7); //Call HAL handler
}

//HAL timer3
void TIM3_IRQHandler()
{
	HAL_TIM_IRQHandler(&timer3_pwm); //Call HAL handler
}


//HAL - GPIO/EXTI Handler for pushbutton - screen on/off
void EXTI0_IRQHandler()
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//screen turn off/on through pushbutton
	if(screen_on)
	{
		BSP_LCD_SetBrightness(0);
	}
	else
	{
		BSP_LCD_SetBrightness(80);
	}
	screen_on=!screen_on;
}
