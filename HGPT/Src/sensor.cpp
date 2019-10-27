#include "sensor.hpp"
#include "xprintf.h"

#define VL53L0X_Address    0x52
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define	SYSTEM_INTERMEASUREMENT_PERIOD  0x04

void Sensor::requestSingleMeasurement(){
	write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
	isAvailable_=false;
}

uint16_t Sensor::readSingleMeasurement(){
	//uint8_t val;
	//val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
	if (isAvailable())
	{
		// Store new data
		read_block_data_at(0x14, 12);
		dist_ = convuint16(buf[11], buf[10]);

		// Request new data
		requestSingleMeasurement();
	}
	return dist_;
}

bool Sensor::isAvailable(){
	uint8_t val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
	if (val & 0x01){
		isAvailable_ = true;
	}
	return isAvailable_;
}

uint16_t Sensor::VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}

void Sensor::write_byte_data_at(uint8_t reg, uint8_t data) {
	mux_->select_channel(ch_,hi2c_);
	uint8_t val[1];
    val[0]=data;
    HAL_I2C_Mem_Write(hi2c_, VL53L0X_Address, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 1000);
}
void Sensor::write_32Bit_data_at(uint8_t reg, uint32_t data) {
	mux_->select_channel(ch_,hi2c_);
	uint8_t val[4];
	val[0]=((data>>24)&0xFF);
	val[1]=((data>>16)&0xFF);
	val[2]=((data>>8)&0xFF);
	val[3]=data&0xFF;
	HAL_I2C_Mem_Write(hi2c_, VL53L0X_Address, reg, I2C_MEMADD_SIZE_8BIT, val, 4, 1000);
}

uint8_t Sensor::read_byte_data_at(uint8_t reg) {
	mux_->select_channel(ch_,hi2c_);
	uint8_t value;
	HAL_I2C_Mem_Read(hi2c_, VL53L0X_Address, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
	return value;
}

void Sensor::read_block_data_at(uint8_t reg, int sz) {
	mux_->select_channel(ch_,hi2c_);
    HAL_I2C_Mem_Read(hi2c_, VL53L0X_Address, reg, I2C_MEMADD_SIZE_8BIT, buf, sz, 1000);
}

uint16_t Sensor::convuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void Sensor::Vl53L0X_Test(void){

    uint8_t val1 ;
//CHK@Param
      val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
      xprintf("Revision ID: %d, ",val1);

      val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
      xprintf("Device ID:  %d \n",val1);

      val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
     xprintf("PRE_RANGE_CONFIG_VCSEL_PERIOD: %d \n",val1);
      xprintf(" decode:   %d \n",VL53L0X_decode_vcsel_period(val1));

      val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
//      xprintf("FINAL_RANGE_CONFIG_VCSEL_PERIOD: %d \n",val1);
//      xprintf(" decode:   %d \n",VL53L0X_decode_vcsel_period(val1));

//Init Start
      write_32Bit_data_at(SYSTEM_INTERMEASUREMENT_PERIOD,20);
      write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

      uint8_t val = 0;
      int cnt = 0;
      while (cnt < 100) { // 1 second waiting time max
          HAL_Delay(10);
        val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
        if (val & 0x01) break;
        cnt++;
      }
    //  if (val & 0x01) xprintf("Ready!! \n"); else xprintf("Not Ready!!");

      read_block_data_at(0x14, 12);
      //uint16_t acnt = convuint16(buf[7], buf[6]);
      //uint16_t scnt = convuint16(buf[9], buf[8]);
      uint16_t dist = convuint16(buf[11], buf[10]);
      //uint8_t DeviceRangeStatusInternal = ((buf[0] & 0x78) >> 3);
     // xprintf("ambient count: %d, signal count: %d, distance: %d, status: %d  \n",
      //        acnt,scnt,dist,DeviceRangeStatusInternal);
   xprintf("RES: %d@mm  \n",dist);

}

int Sensor::VL53L0X_read()
{
    write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
	while(1)
	{		HAL_Delay(1);

		uint8_t val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    	if (val & 0x01) break;
	}
    read_block_data_at(0x14, 12);

	return       convuint16(buf[11], buf[10]);

}

uint8_t Sensor::VL53L0X_Address_Test(void){
	mux_->select_channel(ch_,hi2c_);
     uint8_t tmp[2];
     HAL_I2C_Mem_Read(hi2c_, VL53L0X_Address, 0xC1, I2C_MEMADD_SIZE_8BIT, tmp, 1, 100);

     if(tmp[0]==0xAA){
        xprintf("VL53L0X is Found! \n");
         return true;
     }
     else{
      xprintf("VL53L0X is NOT Found! ERROR! \n");
         return false;
     }

}
