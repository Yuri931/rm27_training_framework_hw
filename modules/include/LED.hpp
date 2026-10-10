#ifndef MODULES_LED_HPP
#define MODULES_LED_HPP
#include"main.h"
#include<stdint.h>
enum class LED_Color:uint8_t
{
    Red,Green,Blue
};

void LED_Set(LED_Color color,bool on);
void LED_Toggle(LED_Color color);
#endif