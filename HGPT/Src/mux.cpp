/*
 * mux.cpp
 *
 *  Created on: 2019/10/19
 *      Author: Ryo
 */

#include "mux.hpp"
mux::mux() {
	// TODO Auto-generated constructor stub

}

mux::~mux() {
	// TODO Auto-generated destructor stub
}

void mux::select_channel(int ch,I2C_HandleTypeDef *hi2c)
{
	uint8_t msg=8+ch;
	HAL_I2C_Master_Transmit(hi2c,myaddress,&msg,1,100);

}
