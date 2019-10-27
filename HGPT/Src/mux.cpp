/*
 * mux.cpp
 *
 *  Created on: 2019/10/19
 *      Author: Ryo
 */

#include "mux.hpp"
/*mux::mux() {
	// TODO Auto-generated constructor stub

}
*/

mux::~mux() {
	// TODO Auto-generated destructor stub
}

void mux::init(){
	// Set multiplexer address to 0x70(=b1110000)
	HAL_GPIO_WritePin(GPIOx_MUX1_, GPIO_Pin_MUX1_, GPIO_PIN_RESET); // 0
	HAL_GPIO_WritePin(GPIOx_MUX2_, GPIO_Pin_MUX2_, GPIO_PIN_RESET); // 0
	HAL_GPIO_WritePin(GPIOx_MUX3_, GPIO_Pin_MUX3_, GPIO_PIN_RESET); // 0

	// Enable multiplexer by setting reset pin to HIGH
	HAL_GPIO_WritePin(GPIOx_MUX_RST_, GPIO_Pin_MUX_RST_, GPIO_PIN_SET);
}
void mux::select_channel(uint8_t ch,I2C_HandleTypeDef *hi2c)
{
	// Change channel
	uint8_t c;
	c = ch & 0x07; // チャネル(bit0-2)を取り出す
	c = c | 0x08 ; // enableビットを設定する
	HAL_I2C_Master_Transmit(hi2c,myaddress,&c,1,100);

	//uint8_t msg=8+ch;
	//HAL_I2C_Master_Transmit(hi2c,myaddress,&msg,1,100);
}
