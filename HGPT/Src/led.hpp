#include "main.h"

class LED
{
private:
	GPIO_TypeDef* GPIOx_;
	uint16_t GPIO_Pin_;
public:
	LED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
		GPIOx_ = GPIOx;
		GPIO_Pin_ = GPIO_Pin;
	};
	void on();
	void off();
	void toggle();
};
