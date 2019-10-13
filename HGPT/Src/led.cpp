#include "led.hpp"

void LED::on(){
	HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, GPIO_PIN_SET);
}
void LED::off(){
	HAL_GPIO_WritePin(GPIOx_, GPIO_Pin_, GPIO_PIN_RESET);
}
