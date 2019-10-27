#include "main.h"

class Encoder
{
private:
	TIM_HandleTypeDef *htim_;
	uint32_t Channel_;
	TIM_TypeDef *TIM_;
	int32_t count_;
	uint32_t reset_value_;
public:
     Encoder(TIM_HandleTypeDef *htim, uint32_t Channel, TIM_TypeDef *TIM){
    	 htim_ = htim;
    	 Channel_ = Channel;
    	 TIM_ = TIM;
    	 count_ = 0;
    	 reset_value_ = 32768;
     };
     void start(); // Call this in USER CODE 2
     int32_t count();
     float deltaMm(); // Difference since last call in millimeters
};
