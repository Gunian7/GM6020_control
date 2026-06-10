/**
 * @file blinky.c
 * @brief 最简单的点灯程序 - 测试 JLink 烧录环境
 * 
 * RoboMaster 开发板 C 型 RGB LED 引脚：
 *   红灯: PH12, 绿灯: PH11, 蓝灯: PH10
 *   高电平点亮
 * 
 * 直接用寄存器操作，不依赖 HAL 和 FreeRTOS，编译快，烧录小。
 */
#include "main.h"

/* LED 引脚 */
#define LED_R_PIN   GPIO_PIN_12
#define LED_G_PIN   GPIO_PIN_11
#define LED_B_PIN   GPIO_PIN_10
#define LED_PORT    GPIOH

/* 简单的延时 */
static void delay_loop(volatile uint32_t count)
{
    while (count--) {
        __NOP();
    }
}
uint8_t cnt;
int main(void)
{
    /* 复位 RCC 寄存器 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    
    /* 配置 PH10, PH11, PH12 为推挽输出 */
    LED_PORT->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    LED_PORT->MODER |=  (GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0);
    LED_PORT->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12);
    LED_PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR10 | GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);
    LED_PORT->PUPDR   &= ~(GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    
    /* 初始：红灯亮 */
    LED_PORT->BSRR = LED_R_PIN;
    
    for (;;)
    {
        /* 红灯亮 0.5秒 */
        LED_PORT->BSRR = LED_R_PIN;
        LED_PORT->BSRR = (uint32_t)LED_G_PIN << 16U;
        LED_PORT->BSRR = (uint32_t)LED_B_PIN << 16U;
        cnt++;
        delay_loop(800000);
        
        /* 绿灯亮 0.5秒 */
        LED_PORT->BSRR = (uint32_t)LED_R_PIN << 16U;
        LED_PORT->BSRR = LED_G_PIN;
        delay_loop(800000);
        
        /* 蓝灯亮 0.5秒 */
        LED_PORT->BSRR = (uint32_t)LED_G_PIN << 16U;
        LED_PORT->BSRR = LED_B_PIN;
        delay_loop(800000);
    }
}

void SystemClock_Config(void) { /* 用内部 HSI，不需要配外部晶振 */ }
void Error_Handler(void) { while(1); }
