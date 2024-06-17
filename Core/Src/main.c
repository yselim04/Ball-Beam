#include "stm32f103xb.h"                 			 // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

//macros for trigger pin
#define Trig_high		GPIOA->ODR |= (1<<0)
#define Trig_low		GPIOA->ODR &= ~(1<<0)
#define Trig2_high		GPIOA->ODR |= (1<<2)
#define Trig2_low		GPIOA->ODR &= ~(1<<2)

uint32_t duration;
int distance;
uint32_t duration2;
int distance2;
double error ;
int setpoint, setpoint_prec;  // In meters : 30cm --> 0.3m
int y_prec , y;
double error_prec ;
double P, I, D;
int U ;
double I_prec=0, U_prec=0, D_prec=0;
char Sat = 0 ;
char Begin = 0 ;
//double Kp = 7.0;
//double Ki = 2.1;
//double Kd = 5.5;
double Kp = 3.916018;
double Ki = 1.491119;
double Kd = 3.4564955;
#define Umin 850
#define Umax 1250
#define Uzero 1100
#define T 0.125 // sampling time

//prototypes of the used functions
void delaymS(uint32_t ms);
void delayuS(uint32_t us);
uint32_t read_echo(uint32_t timeout);
uint32_t read_echo2(uint32_t timeout);
void sensorTask(void *pvParameters);
void motorTask(void *pvParameters);

int main(void)
{
    // System and peripheral initialization
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; 						//enable GPIOA Clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN ;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    GPIOA->CRL &= ~GPIO_CRL_MODE0;
    GPIOA->CRH = 0x00002220 ;
    GPIOA->CRL = 0x0A004242 ;

    //configure Timer2 to generate microseconds delay
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 1;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;

    TIM3->PSC = 15 ;
    TIM3->ARR = 9999;
    TIM3->CCER = 1 ;
    TIM3->CCMR1 |= (0b1101<<3) ;
    TIM3->CR1 |= (1<<7) ;
    TIM3->EGR |= 1 ;
    TIM3->CR1 |= 1 ;
    TIM3->CCR1 = 790 ;
    //vTaskDelay(pdMS_TO_TICKS(53));


    // Create tasks
    xTaskCreate(sensorTask, "SensorTask", 128, NULL, 1, NULL);
    xTaskCreate(motorTask, "MotorTask", 128, NULL, 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();


    while(1)
    {
        // Should never reach here
    }
}

void sensorTask(void *pvParameters)
{
    (void) pvParameters;

    while(1)
    {
        Trig_low; 												// turn off trig
        delayuS(10); 											// wait 10 us
        Trig_high;  											// turn on trig
        delayuS(10); 											// wait 10 us
        Trig_low; 												// turn off trig
        duration = read_echo(1000000); 							// measure the time of echo pin
        distance = duration* 0.342 ;							// distance = duration / 2 * SOUND_SPEED
        if(distance<40){
            setpoint =distance;
            setpoint = 0.56*setpoint + 0.44*setpoint_prec  ;

        }
        vTaskDelay(pdMS_TO_TICKS(23));
        Trig2_low;
        delayuS(10);
        Trig2_high;
        delayuS(10);
        Trig2_low;
        duration2 = read_echo2(1000000);
        distance2 = duration2* 0.342 ;
        if(distance2<40){
            	y =distance2;
            	y = 0.56*y + 0.44*y_prec  ;

                }
        vTaskDelay(pdMS_TO_TICKS(23));
        error = (setpoint - y);
        P = Kp*error;
        if(Sat == 0)
        	I = T*Ki*error + I_prec ;
        D = (Kd/T)*(error-error_prec)  ;
        D = 0.73*D + 0.27*D_prec;
        U = P + D + I + Uzero ;
        if( U < Umin){
            U = Umin;
            Sat = 1 ;
        }
        else{
            if( U > Umax){
                U = Umax ;
                Sat = 1 ;
            }
            else
            	Sat = 0 ;
        }
        if (y <= setpoint+2 && y >= setpoint-2) {
                    GPIOA->ODR = (0b010<<9);  // Set specific pin pattern if y equals setpoint
                    vTaskDelay(pdMS_TO_TICKS(4));
                } else if (y > setpoint) {
                    GPIOA->ODR = (0b100<<9);  // Set another pin pattern if y is greater than setpoint
                } else {
                    GPIOA->ODR = (0b001<<9);  // Set another pin pattern if y is less than setpoint
        }
        I_prec = I;
        error_prec = error;
        setpoint_prec = setpoint ;
        y_prec = y ;
        D = D_prec ;
    }
}

void motorTask(void *pvParameters){
	while(1){
	    TIM3->CCR1 = U ;
	    vTaskDelay(pdMS_TO_TICKS(30));
	}

}

void delaymS(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void delayuS(uint32_t us)
{
    for (int i = 0; i < us; i++)
    {
        TIM2->CNT = 0;
        while (!(TIM2->SR & TIM_SR_UIF)) {}
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

uint32_t read_echo(uint32_t timeout)
{
	duration = 0 ;
    while (!((GPIOA->IDR) & GPIO_IDR_IDR1))
    {
        duration++;
        delayuS(1);
        if (duration > timeout)
        {
            return 0;
        }
    }
	duration = 0 ;
    while ((GPIOA->IDR & GPIO_IDR_IDR1))
    {
        duration++;
        delayuS(1);
        if (duration > timeout)
        {
            return 0;
        }
    }
    return duration;
}
uint32_t read_echo2(uint32_t timeout)
{
	duration2 = 0 ;
    while (!((GPIOA->IDR) & GPIO_IDR_IDR3))
    {
        duration2++;
        delayuS(1);
        if (duration2 > timeout)
        {
            return 0;
        }
    }
	duration2 = 0 ;
    while ((GPIOA->IDR & GPIO_IDR_IDR3))
    {
        duration2++;
        delayuS(1);
        if (duration2 > timeout)
        {
            return 0;
        }
    }
    return duration2;
}
