/*
 * mux.hpp
 *
 *  Created on: 2019/10/19
 *      Author: Ryo
 */

#ifndef MUX_HPP_
#define MUX_HPP_
#include "main.h"


class mux {
private:
	GPIO_TypeDef* GPIOx_MUX_RST_; // Reset pin
	uint16_t GPIO_Pin_MUX_RST_; // Reset pin
	GPIO_TypeDef* GPIOx_MUX1_;
	uint16_t GPIO_Pin_MUX1_;
	GPIO_TypeDef* GPIOx_MUX2_;
	uint16_t GPIO_Pin_MUX2_;
	GPIO_TypeDef* GPIOx_MUX3_;
	uint16_t GPIO_Pin_MUX3_;

public:
	mux(GPIO_TypeDef* GPIOx_MUX_RST, uint16_t GPIO_Pin_MUX_RST,
		GPIO_TypeDef* GPIOx_MUX1, uint16_t GPIO_Pin_MUX1,
		GPIO_TypeDef* GPIOx_MUX2, uint16_t GPIO_Pin_MUX2,
		GPIO_TypeDef* GPIOx_MUX3, uint16_t GPIO_Pin_MUX3){

		// Store pin location
		GPIOx_MUX_RST_ = GPIOx_MUX_RST;
		GPIO_Pin_MUX_RST_ = GPIO_Pin_MUX_RST;
		GPIOx_MUX1_ = GPIOx_MUX1;
		GPIO_Pin_MUX1_ = GPIO_Pin_MUX1;
		GPIOx_MUX2_ = GPIOx_MUX2;
		GPIO_Pin_MUX2_ = GPIO_Pin_MUX2;
		GPIOx_MUX3_ = GPIOx_MUX3;
		GPIO_Pin_MUX3_ = GPIO_Pin_MUX3;
	};
	virtual ~mux();
	void init();
	void select_channel(uint8_t ch,I2C_HandleTypeDef *hi2c);
private:
	const uint8_t myaddress=0b11100000;	//mux's slave address 1110 fixed 000 hardware selectable last bit 1=read 0=write
};


#endif /* MUX_HPP_ */
