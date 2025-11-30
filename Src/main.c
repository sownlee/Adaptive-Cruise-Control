/*
 * main.c - Adaptive Cruise Control with HC-SR04 & Dual DC Motor PWM Control
 * Updated based on CSDN blog: STM32 control 2 DC motors with TIM1 PWM + TIM3 Encoder
 * Integrated with previous HC-SR04 code for ACC logic
 * Author: Adapted from sownlee repo + blog
 * Date: Dec 2025
 */

#include "stm32f4xx.h"  // STM32F4xx StdPeriph
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>  // Cho printf (retarget UART)

// From previous HC-SR04 code
typedef struct {
    uint32_t overflow;
    uint32_t cnt;
    float    f_distance;  // Front
    float    l_distance;  // Left
    float    r_distance;  // Right
} Dist_TypeDef;

Dist_TypeDef dist = {0};
static uint16_t cnt_f = 0, cnt_l = 0, cnt_r = 0;

// Motor globals (from blog: 2 motors, speed 0-1000)
int Motor1_Speed = 0, Motor2_Speed = 0;
int Motor1_Direction = 1, Motor2_Direction = 1;  // 1=forward, -1=reverse

// Delay functions (simple, calibrate @84MHz)
void Delay_us(uint32_t us) {
    uint32_t i;
    for (i = 0; i < us * 21; i++);  // Adjust for 84MHz
}

void Delay_ms(uint32_t ms) {
    uint32_t i;
    for (i = 0; i < ms * 21000; i++);  // Adjust for 84MHz
}

// HC-SR04 Trigger (from previous code)
void Get_Distance(uint8_t choice) {
    // ... (copy full from previous main.c - PA4/5/6 Trig)
    // Omitted for brevity; assume it's here
}

// TIM2_IRQHandler for HC-SR04 (from previous)
void TIM2_IRQHandler(void) {
    // ... (copy full from previous - overflow + CH2/3/4)
    // Omitted; assume integrated
}

// UART Init (from previous, for debug)
void UART_Init(void) {
    // ... (copy full from previous - USART1 @115200)
}

// printf retarget
int fputc(int ch, FILE *f) {
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

// PWM Motor Init (from blog: TIM1 CH1/CH2 for 2 motors)
void motorPWM_init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // TimeBase: 1kHz PWM @84MHz (PSC=83, ARR=999)
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 83;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // CH1 (Motor1)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // Initial duty 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // CH2 (Motor2)
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

// PWM Pins Init (from blog: PE9/PE11 for CH1/CH2)
void motorPWMPin_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;  // PE9=CH1, PE11=CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
}

// Direction Pins Init (from blog: PA8/PA9 for Motor1 IN1/IN2; assume PA10/PA11 for Motor2)
void motorDirection_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// Set Motor Speed (from blog: duty = (speed/1000)*ARR)
void Motor_Set_Speed(int motor, int speed) {  // motor: 1 or 2, speed: -1000 to 1000 (negative=reverse)
    int abs_speed = speed > 0 ? speed : -speed;
    uint16_t duty = (abs_speed * 1000) / 1000;  // ARR=999 ~1000

    if (motor == 1) {
        Motor1_Speed = speed;
        TIM_SetCompare1(TIM1, duty);  // CH1

        if (speed >= 0) {  // Forward
            GPIO_SetBits(GPIOA, GPIO_Pin_8);   // IN1=1
            GPIO_ResetBits(GPIOA, GPIO_Pin_9); // IN2=0
        } else {  // Reverse
            GPIO_ResetBits(GPIOA, GPIO_Pin_8); // IN1=0
            GPIO_SetBits(GPIOA, GPIO_Pin_9);   // IN2=1
        }
    } else if (motor == 2) {
        Motor2_Speed = speed;
        TIM_SetCompare2(TIM1, duty);  // CH2

        if (speed >= 0) {
            GPIO_SetBits(GPIOA, GPIO_Pin_10);  // IN3=1
            GPIO_ResetBits(GPIOA, GPIO_Pin_11); // IN4=0
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_10); // IN3=0
            GPIO_SetBits(GPIOA, GPIO_Pin_11);   // IN4=1
        }
    }
}

// Encoder Init for Motor1 (TIM3 CH1/CH2, from blog)
void encoderA_init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    // GPIO for encoder (PA6/PA7 CH1/CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

    // TimeBase
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // Encoder mode
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

// Simple RPM calculation (from blog logic, call in loop)
int Get_Motor1_RPM(void) {
    int pulse = (int)TIM_GetCounter(TIM3);  // Assume PPR=12, cycles=1s
    return (pulse * 60) / (12 * 1);  // RPM = (pulses/min) / PPR
    TIM_SetCounter(TIM3, 0);  // Reset
}

int main(void) {
    // System clock (84MHz)
    SystemInit();

    // UART for debug
    UART_Init();

    // HC-SR04 init (from previous)
    // ... (TIM2 + GPIO for Trig/Echo - omitted for brevity)

    // Motor PWM init (from blog)
    motorPWM_init();
    motorPWMPin_init();
    motorDirection_init();

    // Encoder init (for feedback)
    encoderA_init();

    printf("ACC System Ready! Motors initialized.\r\n");

    while (1) {
        // ACC Loop: Read HC-SR04
        Get_Distance(1);  // Front
        Delay_ms(30);
        Get_Distance(0);  // Left
        Delay_ms(30);
        Get_Distance(2);  // Right
        Delay_ms(40);  // Total ~100ms cycle

        // Simple ACC Logic: If front < 1000mm, reduce speed; else maintain
        int target_speed = 500;  // Base 50% duty
        if (dist.f_distance < 1000.0f) {  // Obstacle close
            target_speed = (int)(target_speed * (dist.f_distance / 1000.0f));  // Proportional
            if (target_speed < 100) target_speed = 0;  // Stop
        }

        // Set both motors (parallel drive)
        Motor_Set_Speed(1, target_speed);  // Motor1 forward
        Motor_Set_Speed(2, target_speed);  // Motor2 forward

        // Feedback: Print RPM
        int rpm = Get_Motor1_RPM();
        printf("Front Dist: %.1f mm | Target Speed: %d | RPM: %d\r\n", dist.f_distance, target_speed, rpm);

        Delay_ms(100);  // ACC update rate
    }
}
