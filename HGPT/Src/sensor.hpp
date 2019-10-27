#pragma once
#include "main.h"
#include "mux.hpp"

class Sensor{
private:
	I2C_HandleTypeDef *hi2c_;
	mux *mux_;
	uint8_t ch_;
	uint8_t buf[16];
	bool isAvailable_;
	uint16_t dist_; // distance in mm


	uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg);
	void write_byte_data_at(uint8_t reg, uint8_t data);
	void write_32Bit_data_at(uint8_t reg, uint32_t data);
	uint8_t read_byte_data_at(uint8_t reg);
	void read_block_data_at(uint8_t reg, int sz);
	uint16_t convuint16(int lsb, int msb);
	void Vl53L0X_Test(void);
	int VL53L0X_read();
	uint8_t VL53L0X_Address_Test(void);
public:
	Sensor(I2C_HandleTypeDef *hi2c, mux *mux, uint8_t ch){
		hi2c_ = hi2c;
		mux_ = mux;
		ch_ = ch;
		isAvailable_ = false;
		dist_ = 0;
	};
	void requestSingleMeasurement();
	uint16_t readSingleMeasurement(); // get latest distance in mm
	bool isAvailable();
};
