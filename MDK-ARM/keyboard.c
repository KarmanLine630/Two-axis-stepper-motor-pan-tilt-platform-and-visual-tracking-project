#include "keyboard.h"

int main_manu()
{
	OLED_Clear();
	OLED_ShowString(1,1,"MODE CHOOSE");
	OLED_ShowString(2,1,"MODE1");
	OLED_ShowString(3,1,"MODE2");
	OLED_ShowString(4,1,"MODE3");
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET)
	{
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_SET){}
		return 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET)
	{
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET){}
		return 2;
	}
	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET)
	{
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET){}
		return 3;
	}
	else {return 0;}
}