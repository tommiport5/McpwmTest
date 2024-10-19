/*
 * McpwStepperControl.h
 *
 *  Created on: 18.05.2023
 *      Author: dad
 */

#ifndef COMPONENTS_MCPWSTEPPERCONTROL_H_
#define COMPONENTS_MCPWSTEPPERCONTROL_H_

#include "Mcpwm_prelude.h"

#include "McpwmTimer.h"
#include "McpwmOperator.h"
#include "McpwmComparator.h"
#include "McpwmGenerator.h"

#include "Events.h"

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include <atomic>

#include "freertos/timers.h"
#include "freertos/semphr.h"

typedef void (*RefPointFunc)(void *);

const int MsSampleTime = 40;
constexpr int SamplesPerSecond = 1000 / MsSampleTime;

void initDiagPin(gpio_num_t dp);
void setDiagPin(int val);
void toggleDiagPin();

class McpwmStepperControl {
	friend class Drive;
	friend class MachineConfigurator;
	//static const int TimerResolution = 327868;	// defined in McpwmTimer.h

	int32_t MAX_SPEED = 100;	// ticks per second
	int32_t MIN_SPEED = 15;		// lower speeds need longer period
	//float MAX_ACCEL = MAX_SPEED / 2;	// (ticks / second) / second : accelerate to MAX_SPEED in 2 second
	// max period in ticks
	int MAX_PERIOD = 0xfffc;
	static const uint32_t STACKSIZE = 4096;

	McpwmStepperControl(const McpwmStepperControl &) = delete;
	McpwmStepperControl &operator=(const McpwmStepperControl &) = delete;

public:
	McpwmStepperControl(gpio_num_t In1, gpio_num_t In2, gpio_num_t EnA,
			gpio_num_t In3, gpio_num_t In4, gpio_num_t EnB, gpio_num_t CapPin, gpio_num_t RefPin, int GroupID);
	virtual ~McpwmStepperControl();

	void setSpeeds(int32_t min, int32_t max);

	void begin();
	void enable();
	void release();
	void stop(bool release=false);		// stop immediately

	void toggleDiagPin();
	void showPeriods();

	int32_t getPosition() {return _position.load();}
	void setPosition(int32_t pos) {_position = pos;}

	void onReferencePoint(RefPointFunc rpf, void *par);
	void defaultRefPointFunc();
	bool isReferencePoint();

// deal with period and reverse flag
	void start();
	void setPeriod(uint32_t prd);
	void reverse(bool new_reverse);

	void oneShot(bool rev);

// deal with speed (in mm/min, assuming a 1.8° step and a 2 mm/turn thread
// taking into account the reverse flag
// assuming a resolution of 4 µsecs for the period
public:
	int32_t getCurrentSpeed();
	void setCurrentSpeed(int32_t);
	float getPeriodForSpeed(float);

// hardware interface
private:
	std::atomic_int32_t _position;
	bool _reverse;
	//gpio_isr_handle_t hIsr;
	gpio_num_t _In1, _In2, _EnA, _In3, _In4, _EnB, _CapPin, _RefPin;
	gpio_num_t _dp;
	int _GroupID;
	bool _direction;
	int _error_relay;
	int _debounce;
	McpwmTimer Timer;
	McpwmOperator Operator[3];
	McpwmComparator Comparator[3];
	McpwmGenerator Generator[6];

	void buildForward();
	void buildReverse();
	void buildGen01();
	void installCapISR();
	void installRefISR();
	static void captureISR(void *);
	static void defaultRefISR(void *);

	esp_err_t debugForce(int l, bool h) {return Generator[0].force(l,h);}
};

#endif /* COMPONENTS_MCPWSTEPPERCONTROL_H_ */
