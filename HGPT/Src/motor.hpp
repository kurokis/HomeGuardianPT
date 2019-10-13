
#include "main.h"

class Motor
{
  private:
	  //////// Timer
	  TIM_HandleTypeDef *htim_;
	  uint32_t Channel_;
	  TIM_OC_InitTypeDef sConfigOC_;
	  uint32_t max_pulse_; // Pulse saturation threshold

	  //////// Driver pins
	  GPIO_TypeDef* GPIOx_IN1_; // IN1
	  uint16_t GPIO_Pin_IN1_; // IN1
	  GPIO_TypeDef* GPIOx_IN2_; // IN2
	  uint16_t GPIO_Pin_IN2_; // IN2
	  GPIO_TypeDef* GPIOx_STBY_; // STBY
	  uint16_t GPIO_Pin_STBY_; // STBY

	  void setModeSTBY(); // Set driver to standby mode
	  void setModeSTOP(); // Set driver to stop mode
	  void setModeCW(); // Set rotation to clockwise
	  void setModeCCW(); // Set rotation to counter-clockwise

	  void setPWM(uint32_t pulse); // Set PWM pulse width with saturation

  public:
      Motor(TIM_HandleTypeDef *htim, uint32_t Channel,
    		  GPIO_TypeDef* GPIOx_IN1, uint16_t GPIO_Pin_IN1,
			  GPIO_TypeDef* GPIOx_IN2, uint16_t GPIO_Pin_IN2,
			  GPIO_TypeDef* GPIOx_STBY, uint16_t GPIO_Pin_STBY){

    	  //////// Timer
    	  htim_ = htim;
    	  Channel_ = Channel;

    	  //////// PWM settings
    	  sConfigOC_.OCMode = TIM_OCMODE_PWM1;
		  sConfigOC_.Pulse = 0;
		  sConfigOC_.OCPolarity = TIM_OCPOLARITY_HIGH;
		  sConfigOC_.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		  sConfigOC_.OCFastMode = TIM_OCFAST_DISABLE;
		  sConfigOC_.OCIdleState = TIM_OCIDLESTATE_RESET;
		  sConfigOC_.OCNIdleState = TIM_OCNIDLESTATE_RESET;

		  max_pulse_ = 999;

		  //////// Driver pins
		  GPIOx_IN1_ = GPIOx_IN1;
		  GPIO_Pin_IN1_ = GPIO_Pin_IN1;
		  GPIOx_IN2_ = GPIOx_IN2;
		  GPIO_Pin_IN2_ = GPIO_Pin_IN2;
		  GPIOx_STBY_ = GPIOx_STBY;
		  GPIO_Pin_STBY_ = GPIO_Pin_STBY;

		  setModeSTBY();
      };
      void start(); // Call this in USER CODE 2
      void setPWMDuty(float duty); // duty: -1 to 1, positive means counter-clockwise
      void standby();
      void brake();
};
