#include "encoder.hpp"

void Encoder::start(){
	// Start encoder
	HAL_TIM_Encoder_Start( htim_, Channel_ );

	// Reset encoder count
	TIM_->CNT = reset_value_;
}
int32_t Encoder::count(){
	// Get encoder count from timer
	uint32_t encoder_count = TIM_->CNT;

	// Add change from reset value
	if(encoder_count >= reset_value_){
		count_ += (int32_t)(encoder_count - reset_value_);
	}else{
		count_ -= (int32_t)(reset_value_ - encoder_count);
	}

	// Reset encoder count
	TIM_->CNT = reset_value_;

	return count_;
}
float Encoder::deltaMm(){
	static int32_t prev_encoder_count = 0;
	int32_t encoder_count = count();
	float delta_mm = (float)(encoder_count - prev_encoder_count)/512*(2*3.1415)*13.5;

	// update previous encoder count
	prev_encoder_count = encoder_count;

	return delta_mm;
}
