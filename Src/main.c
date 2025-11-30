/*
 * main.c - Adaptive Cruise Control with HC-SR04 Ultrasonic Sensor
 * Updated based on CSDN blog: STM32 control 3-channel HC-SR04 using TIM2 Input Capture
 * Author: Adapted from sownlee repo + blog
 * Date: Dec 2025
 */

#include "stm32f4xx.h"  // Hoặc stm32f1xx.h tùy board
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"  // Để in distance ra UART
#include <stdio.h>            // Cho printf (cần retarget nếu dùng SWO/UART)

// Global struct từ blog (cho 3 kênh: front, left, right)
typedef struct {
    uint32_t overflow;
    uint32_t cnt;
    float    f_distance;  // Front (giữa)
    float    l_distance;  // Left
    float    r_distance;  // Right
} Dist_TypeDef;

Dist_TypeDef dist = {0};

// Static counters cho rising/falling edge detection (từ blog)
static uint16_t cnt_f = 0, cnt_l = 0, cnt_r = 0;

// Hàm delay µs (giả định bạn có, hoặc dùng SysTick)
void Delay_us(uint32_t us) {
    // Simple loop delay, calibrate theo clock 84MHz
    uint32_t i;
    for (i = 0; i < us * 21; i++);  // Adjust 21 theo clock
}

// Hàm trigger Trig theo choice (0: left, 1: front, 2: right) - từ blog
void Get_Distance(uint8_t choice) {
    switch (choice) {
        case 0:  // Left (PA4)
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
            Delay_us(15);
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            break;
        case 1:  // Front (PA5)
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            GPIO_SetBits(GPIOA, GPIO_Pin_5);
            Delay_us(15);
            GPIO_ResetBits(GPIOA, GPIO_Pin_5);
            break;
        case 2:  // Right (PA6)
            GPIO_ResetBits(GPIOA, GPIO_Pin_6);
            GPIO_SetBits(GPIOA, GPIO_Pin_6);
            Delay_us(15);
            GPIO_ResetBits(GPIOA, GPIO_Pin_6);
            break;
    }
}

// TIM2 Interrupt Handler - từ blog (xử lý 3 kênh CH2/3/4)
void TIM2_IRQHandler(void) {
    // Overflow handling
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        dist.overflow++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }

    // Front channel (CH2 - PB3)
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {
        cnt_f++;
        if (cnt_f % 2 == 1) {  // Rising edge: reset counter
            TIM_SetCounter(TIM2, 0);
        } else {  // Falling edge: calculate distance
            dist.cnt = TIM_GetCounter(TIM2);
            dist.f_distance = (dist.overflow * 0xFFFFFFFF + dist.cnt) * 0.17f;  // mm
            printf("Front: %.2f mm\r\n", dist.f_distance);
            dist.overflow = 0;
        }
        if (cnt_f > 65535) cnt_f = 0;
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    }

    // Left channel (CH3 - PA3)
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {
        cnt_l++;
        if (cnt_l % 2 == 1) {
            TIM_SetCounter(TIM2, 0);
        } else {
            dist.cnt = TIM_GetCounter(TIM2);
            dist.l_distance = (dist.overflow * 0xFFFFFFFF + dist.cnt) * 0.17f;
            printf("Left: %.2f mm\r\n", dist.l_distance);
            dist.overflow = 0;
        }
        if (cnt_l > 65535) cnt_l = 0;
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    }

    // Right channel (CH4 - PA2)
    if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) {
        cnt_r++;
        if (cnt_r % 2 == 1) {
            TIM_SetCounter(TIM2, 0);
        } else {
            dist.cnt = TIM_GetCounter(TIM2);
            dist.r_distance = (dist.overflow * 0xFFFFFFFF + dist.cnt) * 0.17f;
            printf("Right: %.2f mm\r\n", dist.r_distance);
            dist.overflow = 0;
        }
        if (cnt_r > 65535) cnt_r = 0;
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    }
}

// UART Init đơn giản để in distance (giả định 115200 baud)
void UART_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_AHB1Periph_GPIOA, ENABLE);

    // PA9 TX, PA10 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // NVIC cho UART (nếu cần RX interrupt, optional)
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

// Retarget printf to UART (thêm vào nếu dùng printf)
int fputc(int ch, FILE *f) {
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

int main(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // System clock init (giả định 84MHz)
    SystemInit();

    // UART init để debug
    UART_Init();

    // RCC clock enable - từ blog
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Trig pins (PA4/PA5/PA6) - Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Echo pins AF input - từ blog
    // PA2 (CH4), PA3 (CH3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    // PB3 (CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

    // TIM2 TimeBase - từ blog (1MHz, 32-bit max)
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;  // 84MHz / 84 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Input Capture cho 3 channels (Both Edge)
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;  // PB3
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;  // PA3
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;  // PA2
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    // NVIC cho TIM2 - từ blog
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    printf("HC-SR04 3-channel ready! Triggering...\r\n");

    while (1) {
        // Loop chính: Trigger 3 kênh liên tục (mỗi 100ms)
        Get_Distance(1);  // Front
        Delay_ms(30);     // Delay giữa trigger (blog khuyên >20ms)

        Get_Distance(0);  // Left
        Delay_ms(30);

        Get_Distance(2);  // Right
        Delay_ms(30);

        // Delay tổng 100ms giữa cycles
        Delay_ms(70);

        // Ở đây bạn có thể thêm logic ACC: e.g., if (dist.f_distance < 500) thì giảm tốc độ
        // Ví dụ: printf("All distances: F=%.1f L=%.1f R=%.1f\r\n", dist.f_distance, dist.l_distance, dist.r_distance);
    }
}

// Thêm hàm Delay_ms nếu chưa có
void Delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 8000; j++);  // Calibrate theo 84MHz
}
