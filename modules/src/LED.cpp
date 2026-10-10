#include "LED.hpp"

namespace
{
    struct LedPin
    {
        GPIO_TypeDef *port;
        uint16_t pin;
    
    };
    constexpr LedPin RED_LED_PIN {GPIOH, GPIO_PIN_10};
    constexpr LedPin GREEN_LED_PIN {GPIOH, GPIO_PIN_11};
    constexpr LedPin BLUE_LED_PIN {GPIOH, GPIO_PIN_12};
    constexpr GPIO_PinState LED_ON_LEVEL = GPIO_PIN_RESET;

    LedPin GetLedPin(LED_Color color)
    {
        switch(color)
        {
            case LED_Color::Red:
                return RED_LED_PIN;
            case LED_Color::Green:
                return GREEN_LED_PIN;
            case LED_Color::Blue:
                return BLUE_LED_PIN;
            default:
                return {nullptr,0};
        }
    }

}
 
void LED_Set(LED_Color color, bool on)
{
    const LedPin led = GetLedPin(color);

    if (led.port == nullptr)
    {
        return;
    }

    const GPIO_PinState level =
        on ? LED_ON_LEVEL
           : (LED_ON_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(led.port, led.pin, level);
}

void LED_Toggle(LED_Color color)
{
    const LedPin led = GetLedPin(color);

    if (led.port == nullptr)
    {
        return;
    }

    HAL_GPIO_TogglePin(led.port, led.pin);
}