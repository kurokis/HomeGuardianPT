#include "motor.hpp"

//---------------- Public functions ----------------
void Motor::start(){
	// Start PWM
	HAL_TIM_Base_Start_IT(htim_);
	HAL_TIM_PWM_Start(htim_, Channel_);
}
void Motor::setPWMDuty(float duty){
	if(duty>=0){
		setModeCCW();
		setPWM((uint32_t)(duty*max_pulse_));
	}else{
		setModeCW();
		setPWM((uint32_t)(-duty*max_pulse_));
	}
}
void Motor::standby(){
	sConfigOC_.Pulse = 0;
	setModeSTBY();
}
void Motor::brake(){
	sConfigOC_.Pulse = 0;
	setModeSTOP();
}

//---------------- Private functions ----------------
void Motor::setModeSTBY(){
	HAL_GPIO_WritePin(GPIOx_STBY_, GPIO_Pin_STBY_, GPIO_PIN_RESET);
}
void Motor::setModeSTOP(){
	HAL_GPIO_WritePin(GPIOx_IN1_, GPIO_Pin_IN1_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOx_IN2_, GPIO_Pin_IN2_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOx_STBY_, GPIO_Pin_STBY_, GPIO_PIN_SET);
}
void Motor::setModeCW(){
	HAL_GPIO_WritePin(GPIOx_IN1_, GPIO_Pin_IN1_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOx_IN2_, GPIO_Pin_IN2_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOx_STBY_, GPIO_Pin_STBY_, GPIO_PIN_SET);
}
void Motor::setModeCCW(){
	HAL_GPIO_WritePin(GPIOx_IN1_, GPIO_Pin_IN1_, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOx_IN2_, GPIO_Pin_IN2_, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOx_STBY_, GPIO_Pin_STBY_, GPIO_PIN_SET);
}
void Motor::setPWM(uint32_t pulse){
	// Set pulse
	if(pulse < max_pulse_){
		sConfigOC_.Pulse = pulse;
	}else{
		sConfigOC_.Pulse = max_pulse_;
	}

	// Update PWM parameters
	if (HAL_TIM_PWM_ConfigChannel(htim_, &sConfigOC_, Channel_) != HAL_OK)
	{
	   Error_Handler();
	}

	// Start PWM
	if (HAL_TIM_PWM_Start(htim_, Channel_) != HAL_OK)
	{
	   Error_Handler();
	}
}
