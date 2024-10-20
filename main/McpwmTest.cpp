#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>
#include "McpwmStepperControl.h"
#include <cstdio>

#ifdef CONFIG_IDF_TARGET_ESP32S3
McpwmStepperControl LiftAxis(GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_8, GPIO_NUM_6, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_46, 0);
McpwmStepperControl BoomAxis(GPIO_NUM_15, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_47, GPIO_NUM_35, GPIO_NUM_36, 1);
#else
McpwmStepperControl LiftAxis(GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_23, 0);
McpwmStepperControl BoomAxis(GPIO_NUM_12, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_22, GPIO_NUM_35, GPIO_NUM_34, 1);
#endif
const int32_t MinSpeed=10;
const int32_t MaxSpeed=40;
const int32_t Delta=1;

extern "C" void app_main(void)
{
	bool quit = false;
	bool up = true;
	int32_t speed = 25;

	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED));

	LiftAxis.begin();
	LiftAxis.setCurrentSpeed(speed);
	LiftAxis.start();
	/*
	BoomAxis.begin();
	BoomAxis.setCurrentSpeed(speed);
	BoomAxis.start();
	*/
    while (!quit) {
   	  vTaskDelay(pdMS_TO_TICKS(500));
	  int c = getchar();
	  if (c != EOF) {
		putchar(c);
		if (c == '+') {
			speed++;
		} else if (c =='-') {
			speed--;
		}
		LiftAxis.setCurrentSpeed(speed);
		std::cout << speed << std::endl;
	  }
   	  /*
   	  if (speed<-MaxSpeed) {
   		  std::cout << "+" << std::flush_emit;
   		  up = true;
   		  speed = -MaxSpeed+1;
   	  } else if (speed < -MinSpeed) {
   		  if (up) {
   			  speed += Delta;
   		  } else {
   			  speed -= Delta;
   		  }
   	  } else if (speed < MinSpeed){
   		  up = true;		// test shortcut
   		  if (up) {
   			  speed = MinSpeed+1;
   		  } else {
   			  speed = -MinSpeed-1;		// a little asymmetrical on the "down" way
   		  }
   	  } else if (speed < MaxSpeed) {
   		  if (up) {
   			  speed += Delta;
   		  } else {
   			  speed -= Delta;
   		  }
   	  } else {
   		  std::cout << "-" << std::flush_emit;
   		  up = false;
   		  speed = MaxSpeed-1;
   	  }
   	  LiftAxis.setCurrentSpeed(speed);
   	  */
//   	  BoomAxis.setCurrentSpeed(speed);
    }
}

