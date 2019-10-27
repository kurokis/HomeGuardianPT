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
public:
	mux();
	virtual ~mux();
	void select_cahhnel(int ch,I2C_HandleTypeDef *hi2c);
private:
	const uint8_t myaddress=0b11100001;	//mux's slave address 1110 fixed 000 hardware selectable last bit 1=read 0=write
};


#endif /* MUX_HPP_ */
