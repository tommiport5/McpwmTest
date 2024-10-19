/*
 * McpwStepperControl.cpp
 *
 *  Created on: 18.05.2023
 *      Author: dad
 */

#include "esp_err.h"
#include "esp_log.h"

#include "McpwmStepperControl.h"
#include <cstdio>
#include <cmath>

extern bool verbose;
static const char *TAG = "McpwmStepperControl";

McpwmStepperControl::~McpwmStepperControl() {
	assert(false);
	gpio_isr_handler_remove(_CapPin);
}

McpwmStepperControl::McpwmStepperControl(gpio_num_t In1, gpio_num_t In2,
		gpio_num_t EnA, gpio_num_t In3, gpio_num_t In4, gpio_num_t EnB, gpio_num_t CapPin, gpio_num_t RefPin, int GroupID)
			: _reverse(false), _In1(In1), _In2(In2), _EnA(EnA), _In3(In3), _In4(In4), _EnB(EnB), _CapPin(CapPin), _RefPin(RefPin),
			 _GroupID(GroupID), _direction(false), _error_relay(0), _debounce(0),
			 Timer(GroupID),
			 Operator{McpwmOperator(GroupID), McpwmOperator(GroupID), McpwmOperator(GroupID)},
			 Comparator{McpwmComparator(Operator[0]), McpwmComparator(Operator[1]), McpwmComparator(Operator[2])},
			 Generator{McpwmGenerator(Operator[0], In1, 0), McpwmGenerator(Operator[0], In2, 1),
				 McpwmGenerator(Operator[1], In3, 0), McpwmGenerator(Operator[1], In4, 1),
				 McpwmGenerator(Operator[2], EnA, 0), McpwmGenerator(Operator[2], EnB, 0) }
{
	setSpeeds(10, 150);	// set default values
	_dp = GPIO_NUM_NC;
	Timer.setComparators(&Comparator);
}
void McpwmStepperControl::setSpeeds(int32_t min, int32_t max) {
	//if (min < SamplesPerSecond) ESP_LOGW(TAG, "Min speed %ld (< %d) confuses machine task loop", min, SamplesPerSecond);
	MAX_SPEED = max;
	MIN_SPEED = min;
}

void McpwmStepperControl::buildForward() {
// Generator 0
	// State 0
//	ESP_ERROR_CHECK(Generator[0].setActionOnTimerEvent(
//			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
//			));
	ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(Generator[0].getHandle(),
				MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[2].getHandle(), MCPWM_GEN_ACTION_LOW)));
	ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(Generator[0].getHandle(),
				MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, Comparator[1].getHandle(), MCPWM_GEN_ACTION_HIGH)));
//	ESP_ERROR_CHECK(Generator[0].setActionsOnCompareEvents(
//				MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[0].getHandle(), MCPWM_GEN_ACTION_HIGH),
//				MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[1].getHandle(), MCPWM_GEN_ACTION_LOW),
//				MCPWM_GEN_COMPARE_EVENT_ACTION_END()
//			));
// Generator 1
	// State 0
	ESP_ERROR_CHECK(Generator[1].setActionsOnTimerEvents(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
	// State 3
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_TIMER_EVENT_FULL, MCPWM_GEN_ACTION_LOW),
			MCPWM_GEN_TIMER_EVENT_ACTION_END()
			));
//	// State 2
//	ESP_ERROR_CHECK(Generator[0].setActionsOnCompareEvent(
//			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[0].getHandle(), MCPWM_GEN_ACTION_LOW),
//	// State 6
//			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, Comparator[1].getHandle(), MCPWM_GEN_ACTION_HIGH),
//			MCPWM_GEN_COMPARE_EVENT_ACTION_END()
//			));
// Generator 2
	// State 0
/*
	ESP_ERROR_CHECK(Generator[2].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
			));
	// State 4
	ESP_ERROR_CHECK(Generator[2].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_TIMER_EVENT_FULL, MCPWM_GEN_ACTION_LOW)
			));
// Generator 3
	// State 0
	ESP_ERROR_CHECK(Generator[3].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
			));
	// State 4
	ESP_ERROR_CHECK(Generator[2].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_TIMER_EVENT_FULL, MCPWM_GEN_ACTION_LOW)
			));
// Generator 4
	// State 0
	ESP_ERROR_CHECK(Generator[4].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
			));
	// State 1
	ESP_ERROR_CHECK(Generator[4].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[0].getHandle(), MCPWM_GEN_ACTION_LOW)
			));
	// State 2
	ESP_ERROR_CHECK(Generator[4].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[1].getHandle(), MCPWM_GEN_ACTION_HIGH)
			));
	// State 5
	ESP_ERROR_CHECK(Generator[4].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, Comparator[2].getHandle(), MCPWM_GEN_ACTION_LOW)
			));
	//State 6
	ESP_ERROR_CHECK(Generator[4].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, Comparator[1].getHandle(), MCPWM_GEN_ACTION_HIGH)
			));
// Generator 5
	// State 0
	ESP_ERROR_CHECK(Generator[5].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
			));
	// State 3
	ESP_ERROR_CHECK(Generator[5].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[2].getHandle(), MCPWM_GEN_ACTION_LOW)
			));
	// State 4
	ESP_ERROR_CHECK(Generator[5].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_TIMER_EVENT_FULL, MCPWM_GEN_ACTION_HIGH)
			));
	// State 7
	ESP_ERROR_CHECK(Generator[5].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, Comparator[0].getHandle(), MCPWM_GEN_ACTION_LOW)
			));
			*/
}

void McpwmStepperControl::buildReverse() {
}

void McpwmStepperControl::buildGen01() {
	return;
	bool eff_reverse = _direction != _reverse;
	ESP_ERROR_CHECK(Generator[0].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, eff_reverse ? MCPWM_GEN_ACTION_LOW : MCPWM_GEN_ACTION_HIGH)
			));
	ESP_ERROR_CHECK(Generator[0].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[0].getHandle(), eff_reverse ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW)
			));
	ESP_ERROR_CHECK(Generator[1].setActionOnTimerEvent(
			MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, eff_reverse ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW)
			));
	ESP_ERROR_CHECK(Generator[1].setActionOnCompareEvent(
			MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, Comparator[0].getHandle(), eff_reverse ? MCPWM_GEN_ACTION_LOW : MCPWM_GEN_ACTION_HIGH)
			));
}

void McpwmStepperControl::begin() {
	Operator[0].connectTimer(Timer);
	Operator[1].connectTimer(Timer);
	Operator[2].connectTimer(Timer);
	// normal operating mode
	buildForward();
	Timer.begin();
	//installCapISR();
	//installRefISR();
}

void McpwmStepperControl::enable() {
//	ESP_ERROR_CHECK(gpio_set_level(_EnA, 1));
//	ESP_ERROR_CHECK(gpio_set_level(_EnB, 1));
}

void McpwmStepperControl::release() {
//	ESP_ERROR_CHECK(gpio_set_level(_EnA, 0));
//	ESP_ERROR_CHECK(gpio_set_level(_EnB, 0));
}

void McpwmStepperControl::start() {
	if (Timer.isRunning()) return;
//	ESP_ERROR_CHECK(gpio_set_level(_EnA, 1));
//	ESP_ERROR_CHECK(gpio_set_level(_EnB, 1));
	Timer.start();
}

void McpwmStepperControl::oneShot(bool new_reverse) {
	if (_reverse != new_reverse) {
		_reverse = new_reverse;
		buildGen01();
	}
	Timer.oneShot();
}

void McpwmStepperControl::stop(bool release) {
	_debounce = 0;
	if (Timer.isRunning()) {
		Timer.stop(true);
	} else {
		ESP_LOGI(TAG,"Timer %d already stopped", _GroupID);
	}
	if (release) {
		ESP_ERROR_CHECK(gpio_set_level(_EnA, 0));
		ESP_ERROR_CHECK(gpio_set_level(_EnB, 0));
	}
}

static gpio_num_t _dp = GPIO_NUM_NC;
void initDiagPin(gpio_num_t dp) {
	_dp = dp;
	gpio_config_t en = {
			.pin_bit_mask = 1ULL << dp,
			.mode = GPIO_MODE_INPUT_OUTPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&en));
}

void setDiagPin(int val) {
	if (_dp != GPIO_NUM_NC) ESP_ERROR_CHECK(gpio_set_level(_dp, val));
}

void toggleDiagPin() {
	if (_dp != GPIO_NUM_NC) 	{
		int ds = gpio_get_level(_dp);
		ESP_ERROR_CHECK(gpio_set_level(_dp, ds == 1 ? 0 : 1));
	}
}

void McpwmStepperControl::setPeriod(uint32_t spd) {
	Timer.setPeriod(spd);
}

void McpwmStepperControl::reverse(bool new_reverse) {
	if (_reverse == new_reverse) return;
	//printf("reversing\n");
	_reverse = new_reverse;
	if (_direction && !_reverse) _position++;	// we have counted one pulse too few?
	else if (!_direction && _reverse) _position--;
	buildGen01();
}

void McpwmStepperControl::installCapISR() {
	ESP_ERROR_CHECK(gpio_isr_handler_add(_CapPin, captureISR, (void *)this));
	gpio_config_t gc = {
			.pin_bit_mask = 1ull << _CapPin,
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_NEGEDGE
	};
	ESP_ERROR_CHECK(gpio_config(&gc));
	ESP_ERROR_CHECK(gpio_intr_enable(_CapPin));
}


void McpwmStepperControl::installRefISR() {
	ESP_ERROR_CHECK(gpio_isr_handler_add(_RefPin, defaultRefISR, (void *)this));
	gpio_config_t gc = {
			.pin_bit_mask = 1ull << _RefPin,
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_NEGEDGE
	};
	ESP_ERROR_CHECK(gpio_config(&gc));
	ESP_ERROR_CHECK(gpio_intr_enable(_RefPin));
}

void McpwmStepperControl::captureISR(void* pThis) {
	McpwmStepperControl *This = (McpwmStepperControl *)pThis;
	// This->toggleDiagPin();
	This->_position.fetch_add(This->_reverse ? -1 : 1);
}


#ifdef _HAVE_REFPOINT
/**
 * defaultRefISR
 * stops the machine immediately
 */
void McpwmStepperControl::defaultRefISR(void* pThis) {
	McpwmStepperControl *This = (McpwmStepperControl *)pThis;
	if (gpio_get_level(This->_RefPin) == 1) {
		This->_debounce = 0;
		return;
	}
	if (++This->_debounce >= 3) {
		//This->Timer.stop(false);
		This->_error_relay = UnexpectedRefPoint;
		esp_event_isr_post_to(UserLoop,EMERGENCY,unexpected_refpoint,&This->_GroupID,sizeof(This->_GroupID), NULL);
	}
}
#endif _HAVE_REFPOINT

/**
 * calculate the period from the equation
 *  period = 1/(a + b*speed)
 *  where a and b are defined theoretically by the following pairs:
 *  	period [McpwmTimer ticks]	speed [stepper motor ticks /second]
 *  	3278						25.005
 *  	328							249.899
 *
 */
static constexpr float loper = 3333.*4.; 	// [duration of 1 pulse @ lospd in units of 12 µsec (see McpwmTimer)]
											// this is the longest possible pulse, because lospd is the minimum speed
static constexpr float lospd = 25;			// [distance ticks per sec]

int32_t McpwmStepperControl::getCurrentSpeed() {
	if (!Timer.isRunning()) return 0;
	float pf = (float) Timer.getPeriod();	// [duration of 1 pulse @ current speed in units of 12 µsec]
	int32_t spd = lroundf(lospd*loper / pf);// loper / pf says how many times the loper pulse is longer than the current pulse.
											// multiplied by lospd gives the current speed in [distance ticks / sec]
	if (_reverse) spd = -spd;
	//if (verbose) printf("CurrentSpeed %ld\n", spd);
	return spd;
}

/**
 * set current speed
 * assumes that reversing has been handled before
 */
void McpwmStepperControl::setCurrentSpeed(int32_t spd) {
	if (spd == 0) {
		stop();
		return;
	}
	if ((_reverse && (spd > 0)) || (!_reverse && (spd < 0))) {
		ESP_LOGW(TAG, "_reverse flag %s and speed %ld do not match", _reverse ? "true" : "false", spd);
//		return;
		reverse(spd < 0);
	}
	int32_t as = abs(spd);
	if (as > MAX_SPEED) as = MAX_SPEED;
	//if (as < MIN_SPEED) as = MIN_SPEED;
	int32_t prd = std::lroundf(loper*lospd / float(as));	// as/lospd says how many times spd is faster than lospd
															// loper must be divided by this value to get the period for spd
	//if (verbose) printf("prd to %ld from %ld\n", prd, spd);
	if (prd > MAX_PERIOD) prd = MAX_PERIOD;	// limit to values Timer can take
	Timer.setPeriod((uint32_t)prd);
}

/**
 * getPeriodForSpeed
 * returns the period in units of 12 µsec
 */
float McpwmStepperControl::getPeriodForSpeed(float spd) {
	int32_t as = abs(spd);
	if (as > MAX_SPEED) as = MAX_SPEED;
	//if (as < MIN_SPEED) as = MIN_SPEED;
	return loper*lospd / float(as);
}


void McpwmStepperControl::onReferencePoint(RefPointFunc rpf, void *par) {
	gpio_isr_handler_remove(_RefPin);
	gpio_isr_handler_add(_RefPin, rpf, par);
}

void McpwmStepperControl::defaultRefPointFunc() {
	_debounce = 0;
	gpio_isr_handler_remove(_RefPin);
	gpio_isr_handler_add(_RefPin, defaultRefISR, (void *) this);
}

bool McpwmStepperControl::isReferencePoint() {
	return gpio_get_level(_RefPin) == 0;
}

void McpwmStepperControl::showPeriods() {
	setCurrentSpeed(100);
	printf("period is %ld for speed 100\n", Timer.getPeriod());
	setCurrentSpeed(30);
	printf("period is %ld for speed 30\n", Timer.getPeriod());
}
