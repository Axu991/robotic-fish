/**
 * @file: led_driver.c
 * @brief: LED 驱动实现（控制 LED 亮灭、闪烁、状态显示）
 * @author: huangxu <huangxu2024@shanghaitech.com>
 * @date: 2026-07-30
 * @version: 1.0.0
 */

#include "led_driver.h" 

static gpio_num_t s_led_pin = GPIO_NUM_17;
static bool s_led_state = false;

void led_init(gpio_num_t pin)
{
    s_led_pin = pin;
    gpio_reset_pin(s_led_pin); // 重置 GPIO 引脚
    gpio_set_direction(s_led_pin, GPIO_MODE_OUTPUT);  // 设置 GPIO 引脚为输出模式
    gpio_set_level(s_led_pin, 1);  // 初始化 LED 为熄灭状态
    s_led_state = false;
}

void led_set(bool on)
{
    gpio_set_level(s_led_pin, on ? 0 : 1); // LED 点亮时 GPIO 输出低电平，熄灭时输出高电平
    s_led_state = on;
}

void led_toggle(void)
{
    s_led_state = !s_led_state;
    gpio_set_level(s_led_pin, s_led_state ? 0 : 1); // 低电平点亮
}

bool led_get_state(void)
{
    return s_led_state;
}
