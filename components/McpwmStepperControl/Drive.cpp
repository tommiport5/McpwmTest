/*
 * Drive.cpp
 *
 *  Created on: 07.07.2023
 *      Author: dad
 */

#include "Drive.h"

#include "esp_err.h"
#include "esp_log.h"
#include "ErrorHandler.h"

#include <cmath>
#include <cstdio>
#include <compare>

static bool verbose = false;

Drive::~Drive() {
}

Drive::Drive(McpwmStepperControl &MSC, CompletionSema & CS)
: _SC(MSC), _CS(CS), _RefPointTop(false), _awaitingCompletion(false)
{
	snprintf(TAG,32,"Drive%d",_SC._GroupID);
	_last = _state = stopped;
	debounce.clear();
}

void Drive::setSpeeds() {
	min_cont_dist = 10;
	default_speed = _SC.MAX_SPEED / 2;
	ref_point_speed = _SC.MAX_SPEED / 3;
	_aps = (float)_SC.MAX_SPEED / (2*SamplesPerSecond); 		// increase speed every loop by _accel
}


void Drive::begin() {
	hLoopSema = xSemaphoreCreateMutex();
	if (hLoopSema == NULL) ESP_LOGE(TAG, "Could not create loop sema for group %d", _SC._GroupID);
	setSpeeds();
	_SC.begin();
}

bool Drive::forward(int32_t speed) {
	bool ret = true;
	if (speed == 0) speed = default_speed;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	if (speed < _SC.MIN_SPEED) speed = _SC.MIN_SPEED;
	if (speed > _SC.MAX_SPEED) speed = _SC.MAX_SPEED;
	int32_t CurSpeed = _SC.getCurrentSpeed();
	switch (_state) {
	case stopped:
		_target_speed =  speed;
		_state = speed_target;
		_SC.reverse(false);
		_SC.setCurrentSpeed(_SC.MIN_SPEED);
		_SC.start();
		break;
	case free_running:
	case speed_target:
		if (CurSpeed < 0) {
			_target_speed =  speed;
			_state = reverse_speed;
		} else {
			_target_speed = (int32_t) speed;
			_state = speed_target;
		}
		break;
	case reverse_speed:
		_target_speed = speed;
		break;
	case positioning:
	case in_pos:
	case wait_exact:
	case at_ref:
	case leaving_ref:
			ret = false;
		break;
	}
	xSemaphoreGive(hLoopSema);
	return ret;
}

bool Drive::backward(int32_t speed) {
	bool ret = true;
	if (speed == 0) speed = default_speed;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	if (speed < _SC.MIN_SPEED) speed = _SC.MIN_SPEED;
	if (speed > _SC.MAX_SPEED) speed = _SC.MAX_SPEED;
	int32_t CurSpeed = _SC.getCurrentSpeed();
	switch (_state) {
	case stopped:
		_target_speed =  -speed;
		_state = speed_target;
		_SC.reverse(true);
		_SC.setCurrentSpeed(-_SC.MIN_SPEED);
		_SC.start();
		break;
	case free_running:
	case speed_target:
		if (CurSpeed > 0) {
			_target_speed =  -speed;
			_state = reverse_speed;
		} else {
			_target_speed =  -speed;
			_state = speed_target;
		}
		break;
	case reverse_speed:
		_target_speed = (int32_t) -speed;
		break;
	case positioning:
	case in_pos:
	case wait_exact:
	case at_ref:
	case leaving_ref:
		ret = false;
		break;
	}
	xSemaphoreGive(hLoopSema);
	return ret;
}

void Drive::stop(bool release) {
	_SC.stop(release);
	if (_awaitingCompletion) {
		_CS.cancelPart();
		_awaitingCompletion = false;
	}
	_state = stopped;
}


void Drive::halt(bool blocking) {
	if (_state == stopped) return;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	_target_speed = 0;
	_state = speed_target;
	xSemaphoreGive(hLoopSema);
	if (blocking) {
		while (_state != stopped) {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

/**
 * checkRef drive in direction of the reference point,
 * stop when the switch is hit and remember the current position
 */
bool Drive::checkRef() {
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	if (_state != stopped) {
		ESP_LOGE(TAG, "Can only check reference point when stopped");
		xSemaphoreGive(hLoopSema);
		return false;
	}
	if (_SC.isReferencePoint()) {
		ESP_LOGE(TAG, "already on reference point");
		xSemaphoreGive(hLoopSema);
		return false;
	}
	_SC.stop(false);		// just make sure
	_CS.countPart();
	_awaitingCompletion = true;
	_state = speed_target;
	if (_RefPointTop) {
		_target_speed = ref_point_speed;
		_SC.reverse(false);
		_SC.setCurrentSpeed(_SC.MIN_SPEED);
	} else {
		_target_speed = -ref_point_speed;
		_SC.reverse(true);
		_SC.setCurrentSpeed(-_SC.MIN_SPEED);
	}
	_SC.onReferencePoint(checkISR, this);
	_SC.start();
	xSemaphoreGive(hLoopSema);
	return true;
}

bool Drive::searchRef(int32_t new_value) {
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	if (_state != stopped) {
		ESP_LOGE(TAG, "Can only search reference point when stopped");
		xSemaphoreGive(hLoopSema);
		return false;
	}
	if (_SC.isReferencePoint()) {
		ESP_LOGE(TAG, "already on reference point");
		xSemaphoreGive(hLoopSema);
		return false;
	}
	_SC.stop(false);		// just make sure
	_CS.countPart();
	_awaitingCompletion = true;
	_ref_position = new_value;		// not yet exactly true ;-)
	_state = speed_target;
	if (_RefPointTop) {
		_target_speed = ref_point_speed;
		_SC.reverse(false);
		_SC.setCurrentSpeed(_SC.MIN_SPEED);
	} else {
		_target_speed = -ref_point_speed;
		_SC.reverse(true);
		_SC.setCurrentSpeed(-_SC.MIN_SPEED);
	}
	_SC.onReferencePoint(searchISR, this);
	_SC.start();
	xSemaphoreGive(hLoopSema);
	return true;
}

void Drive::release(){
	_SC.release();
}

bool Drive::position(int32_t pos, const uint32_t *nom_speed, bool exact) {
	int32_t nominal_speed = nom_speed == NULL ? default_speed : *nom_speed;
	if(verbose) printf("positionLead to %ld, speed %ld\n", pos, nominal_speed);
	return positionLead(pos, nominal_speed);
}

/**
 * positionLead
 * plans and starts a positioning process from current position to pos, starting and ending at speed 0
 * and using nom_speed if possible. Otherwise accelerating to the reachable speed (see below)
 * and then decelerating again to 0. The number of steps for the accelerating/decelarating and the constant
 * phase is stored in the steps array.
 */
bool Drive::positionLead(int32_t pos, const float nom_speed) {
	bool rev = false;
	int32_t const_dist = 0;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	_origin_position = _SC.getPosition();
	int32_t to_go = pos - _origin_position;
	if (to_go == 0) {
		_state = stopped;
		xSemaphoreGive(hLoopSema);
		return false;
	}
	 if (to_go < 0) {
		 rev = true;
		 to_go = -to_go;
	 }
	_state = positioning;
	_awaitingCompletion = true;
	_CS.countPart();
	_target_position = pos;
	_idat.speed_step = _aps;
	_idat.substate = 0;
	_idat.stepcount = 0;
	_target_speed = reachableSpeed(to_go);
	if (abs(nom_speed) < _SC.MIN_SPEED) {	// period gets too short
		// we want to go slower than MIN_SPEED, use one_shots, no matter how long it takes
		_target_speed = nom_speed;	// gives a useful period
		_idat.substate = 3;
		_idat.ticks_per_sample = _target_speed/float(SamplesPerSecond*1.4);	// avoid attempting new step too soon
		_idat.ticks_since_last = 0.;
		_tps_limit = 1.0;
		_idat.num_steps[0] = to_go;
		_SC.reverse(rev);
		if (rev) _target_speed = -_target_speed;
		_SC.setCurrentSpeed(_target_speed);
		//printf("approaching with %f ticks_per_sample to %ld\n", _idat.ticks_per_sample, _target_position);
	} else if (fabsf(nom_speed) < _target_speed) {
		// nom_speed slower than reachable speed, use it
		_target_speed = nom_speed;
		_idat.num_steps[0] = lroundf((_target_speed -_SC.MIN_SPEED) / _idat.speed_step);
		const_dist = to_go - lroundf(calcDistanceFromSpeed(_target_speed));
		//printf("const_dist %ld (with period %f)\n", const_dist, _SC.getPeriodForSpeed(_target_speed));
		_idat.num_steps[1] = lroundf(float(const_dist * _SC.getPeriodForSpeed(_target_speed)*3) / (MsSampleTime*1000))-2;
	} else {
		// there is a useable speed for continuous, but it is slower than nom_speed
		//printf("reducing speed because nom speed %f >  reachable speed %f\n", nom_speed, _target_speed);
		const_dist = to_go - lroundf(calcDistanceFromSpeed(_target_speed));
		//printf("const_dist %ld (with period %f)\n", const_dist, _SC.getPeriodForSpeed(_target_speed));
		_idat.num_steps[0] = lroundf((_target_speed-_SC.MIN_SPEED) / _idat.speed_step)-2;
		_idat.num_steps[1] = 0;
	}
	if (rev && _idat.substate != 3) {
		_SC.reverse(true);
		_target_speed = -_target_speed;
		_idat.speed_step = - _idat.speed_step;
		_idat.next_speed = -_SC.MIN_SPEED;
		_SC.setCurrentSpeed(-_SC.MIN_SPEED);
		_SC.start();
	} else if (_idat.substate != 3){
		_SC.reverse(false);
		_idat.next_speed = _SC.MIN_SPEED;
		_SC.setCurrentSpeed(_SC.MIN_SPEED);
		_SC.start();
	}
	count = 0;
	once = true;
	xSemaphoreGive(hLoopSema);
	if (verbose) printf("positioning lead %d from %ld to %ld (speedstep %f, targ.speed %f, numsteps [%d,%d], const_dist %ld)\n", _SC._GroupID,
			_origin_position, pos, _idat.speed_step, _target_speed, _idat.num_steps[0], _idat.num_steps[1], const_dist);
	return true;
}

bool Drive::positionFollow(int32_t pos, const float nom_speed, const struct ipol *lead_dat) {
	bool rev = false;
	int32_t const_dist = 0;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	_origin_position = _SC.getPosition();
	int32_t to_go = pos - _origin_position;
	if (to_go == 0) {
		_state = stopped;
		xSemaphoreGive(hLoopSema);
		return false;
	}
	 if (to_go < 0) {
		 rev = true;
		 to_go = -to_go;
	 }
	_state = positioning;
	_awaitingCompletion = true;
	_CS.countPart();
	_target_position = pos;
	_idat.speed_step = _aps;
	_idat.substate = 0;
	_idat.stepcount = 0;
	_target_speed = reachableSpeed(to_go);
	if (abs(nom_speed) < _SC.MIN_SPEED) {
		// no special treatment for follow axis
		_target_speed = nom_speed*1.3;	// gives a useful period
		_idat.substate = 3;
		_idat.ticks_per_sample = _target_speed/float(SamplesPerSecond*1.5);	// avoid attempting new step too soon
		_idat.ticks_since_last = 0.;
		_tps_limit = 1.0;
		_idat.num_steps[0] = to_go;
		_idat.num_steps[1] = 0;
		_SC.reverse(rev);
		if (rev) _target_speed = -_target_speed;
		_SC.setCurrentSpeed(_target_speed);
	} else if (fabsf(nom_speed) < _target_speed) {
		// nom_speed slower than reachable speed, use it
		_target_speed = nom_speed;
		_idat.num_steps[0] = lroundf((_target_speed -_SC.MIN_SPEED) / _idat.speed_step);
		const_dist = to_go - lroundf(calcDistanceFromSpeed(_target_speed));
		//printf("const_dist %ld (with period %f)\n", const_dist, _SC.getPeriodForSpeed(_target_speed));
		_idat.num_steps[1] = lroundf(float(const_dist * _SC.getPeriodForSpeed(_target_speed)*3) / (MsSampleTime*1000))-2;
	} else {
		// there is a useable speed for continuous, but it is slower than nom_speed
		//printf("reducing speed because nom speed %f >  reachable speed %f\n", nom_speed, _target_speed);
		const_dist = to_go - lroundf(calcDistanceFromSpeed(_target_speed));
		//printf("const_dist %ld (with period %f)\n", const_dist, _SC.getPeriodForSpeed(_target_speed));
		_idat.num_steps[0] = lroundf((_target_speed-_SC.MIN_SPEED) / _idat.speed_step)-2;
		_idat.num_steps[1] = 0;
	}
	if (rev && _idat.substate != 3) {
		_SC.reverse(true);
		_target_speed = -_target_speed;
		_idat.speed_step = - _idat.speed_step;
		_idat.next_speed = -_SC.MIN_SPEED;
		_SC.setCurrentSpeed(-_SC.MIN_SPEED);
		_SC.start();
	} else if (_idat.substate != 3){
		_SC.reverse(false);
		_idat.next_speed = _SC.MIN_SPEED;
		_SC.setCurrentSpeed(_SC.MIN_SPEED);
		_SC.start();
	}
	count = 0;
	once = true;
	xSemaphoreGive(hLoopSema);
	if (verbose) printf("positioning follow %d from %ld to %ld (speedstep %f, targ.speed %f, numsteps [%d,%d], const_dist %ld)\n", _SC._GroupID,
			_origin_position, pos, _idat.speed_step, _target_speed, _idat.num_steps[0], _idat.num_steps[1], const_dist);
	return true;
}



void Drive::move(float ts) {
	int32_t rts = lroundf(ts);
	bool neg = rts < 0;
	if ((rts != 0) && (neg != _SC._reverse)) {
		_SC.reverse(ts < 0);
	}
	if (fabs(ts) < _SC.MIN_SPEED)
		rts = neg ? -_SC.MIN_SPEED : _SC.MIN_SPEED;
//	printf("%ld,", rts);
//	if (++count % 20 ==0) printf("\n");
	_SC.setCurrentSpeed(rts);
}


bool Drive::final_approach(int32_t CurPos) {
	if (CurPos == _target_position) {
		//printf("stopping _SC at %ld\n", CurPos);
		_SC.stop();
		return true;
	}
	if (CurPos < _target_position) {
		//putchar('<');
		_SC.reverse(false);
		_SC.setCurrentSpeed(_SC.MIN_SPEED);
	} else {
		//putchar('>');
		_SC.reverse(true);
		_SC.setCurrentSpeed(-_SC.MIN_SPEED);
	}
	return false;
}

void Drive::loop() {
	int32_t CurPos;
	int32_t NewSpeed;
	int32_t CurSpeed;
	static int count;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	_SC._debounce = 0;
	CurPos = _SC.getPosition();
//	if ((_SC._GroupID == 1) && (_state != _last)) {
//		printf("state from %d to %d\n", _last, _state);
//		_last = _state;
//	}
	switch (_state) {
	case wait_exact:
		if (final_approach(CurPos)) {
			_state = stopped;
		}
		break;
	case in_pos:
		//printf("in_pos at %ld\n", CurPos);
		fflush(stdout);
		_CS.completePart();
		_awaitingCompletion = false;
		//printf("completed part\n");
		if (final_approach(CurPos)) _state = stopped;
		else _state = wait_exact;
		break;
	case stopped:
		// nothing to do
		break;
	case positioning:
		switch (_idat.substate) {
		case 0:
			if (++_idat.stepcount > _idat.num_steps[0]) {
				_idat.substate = 1;
				_idat.stepcount = 0;
				//printf("\nconst start: pos %ld\n", CurPos);
				move(_target_speed);
			} else {
				_idat.next_speed += _idat.speed_step;
				move(_idat.next_speed);
			}
			break;
		case 1:
			if (++_idat.stepcount > _idat.num_steps[1]) {
				//printf("const end pos %ld, count %d\n", CurPos, _idat.stepcount);
				_idat.substate = 2;
				_idat.stepcount = 0;
			}
			break;
		case 2:
			if (++_idat.stepcount > _idat.num_steps[0]) {	// tolerance window
				if (once) {
					//printf("\nfinished decel at %ld\n", CurPos);
					once = false;
				}
				if (final_approach(CurPos)) {
					//printf("inpos at count %d\n", _idat.stepcount);
					_state = in_pos;
				}
			} else {
				_idat.next_speed -= _idat.speed_step;
				move(_idat.next_speed);
			}
			break;
		case 3:
			if (CurPos == _target_position) {
				_CS.completePart();
				_awaitingCompletion = false;
				_state = stopped;
				break;
			}
			_idat.ticks_since_last += _idat.ticks_per_sample;
			if (_idat.ticks_since_last >= _tps_limit) {
				_SC.oneShot(CurPos > _target_position);
				_tps_limit += 1.0;
			}
			break;
		}
		break;
	case free_running:
		break;
	case reverse_speed:
		// first go to 0, then reverse and then to _target_speed
		CurSpeed = _SC.getCurrentSpeed();
		NewSpeed = CurSpeed + _SC._reverse ? -_aps : +_aps;
		if ((CurSpeed >= -_SC.MIN_SPEED) && (CurSpeed <= +_SC.MIN_SPEED)) {
			_SC.reverse(_target_speed < 0);
			_state = speed_target;
			_SC.start();
		} else {
			//ESP_LOGI(TAG, "reverse_speed to %ld", NewSpeed);
			_SC.setCurrentSpeed(NewSpeed);
		}
		break;
	case speed_target:
		CurSpeed = _SC.getCurrentSpeed();
		if (_SC._reverse) {
			NewSpeed = CurSpeed - _aps ;
			if (NewSpeed < _target_speed)
				NewSpeed = lroundf(_target_speed);
			//printf("speed_target down from %ld to %ld\n", CurSpeed, NewSpeed);
		} else {
			NewSpeed = CurSpeed + _aps ;
			if (NewSpeed > _target_speed)
				NewSpeed = lroundf(_target_speed);
			//printf("speed_target up from %ld to %ld\n", CurSpeed, NewSpeed);
		}
		_SC.setCurrentSpeed(NewSpeed);
		if (NewSpeed == lroundf(_target_speed))
			_state = _target_speed == 0 ? stopped : free_running;
		break;
	case at_ref:
		_SC.stop();
		debounce.test_and_set();	// make isr unsensitive
		//printf("Reference point found\n");
		if (_RefPointTop) {
			_SC.reverse(true);
			_SC.setCurrentSpeed(-_SC.MIN_SPEED);
		} else {
			_SC.reverse(false);
			_SC.setCurrentSpeed(_SC.MIN_SPEED);
		}
		_SC.start();
		count = 0;
		_state = leaving_ref;
		break;
	case leaving_ref:
		if (_SC.isReferencePoint()) {
			count = 0;
		} else  if (++count > 5) {
			debounce.clear();
			_awaitingCompletion = false;
			_CS.completePart();
			_SC.installRefISR();
			stop();
		}
		break;
	}
	xSemaphoreGive(hLoopSema);
}

int32_t Drive::getLastTarget() {
	int32_t ret;
	xSemaphoreTake(hLoopSema,portMAX_DELAY);
	switch (_state) {
	case wait_exact:
	case in_pos:
	case positioning:
		ret = _target_position;
		break;
	default:
		ret = _SC.getPosition();
		break;
	}
	xSemaphoreGive(hLoopSema);
	return ret;
}


/**
 * loop
 * The core processing of the drive.
 * Becomes active every MsSampleTime msecs and process the partials that have been accumulated.
 */
void Drive::loopTask(void* pThis) {
	Drive *This = (Drive*) pThis;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while (true) {
		This->loop();
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MsSampleTime));
	}
}

/**
 * calcDistanceFromSpeed
 * calculates the distance to reach the speed required and back to min again
*/
float Drive::calcDistanceFromSpeed(int32_t speed) {
	if (speed < _SC.MIN_SPEED) {
		ESP_LOGE(TAG, "Cannot calculate distance from speed %ld ( < MIN_SPEED)", speed);
		return 0;
	}
	return (float)(speed*speed - _SC.MIN_SPEED*_SC.MIN_SPEED) / (_aps*SamplesPerSecond);
}
/**
 * calcDecLimit
 * calculates the length of the deceleration phase
 */
float Drive::calcDecLimit(int32_t speed) {
	if (speed < _SC.MIN_SPEED) {
		ESP_LOGE(TAG, "Cannot calculate limit from speed %ld ( < MIN_SPEED)", speed);
		return 0;
	}
	return float((speed - _SC.MIN_SPEED) * (speed + _SC.MIN_SPEED - _aps)) / (_aps*SamplesPerSecond);
}
/**
 * reachableSpeed
 * returns the absolute maximum speed for a "triangle" transition within dist.
 * Triangle transition means accelerate continuously (without speed limit) and the decelerate continuously.
 * I.e. the distance given is the distance for an up plus a down transition.
 */
float Drive::reachableSpeed(int32_t dist) {
	return sqrt(_aps*abs(dist)*SamplesPerSecond + _SC.MIN_SPEED*_SC.MIN_SPEED);
}

void Drive::searchISR(void *pThis) {
	Drive *This = (Drive*) pThis;
	bool handled = This->debounce.test_and_set();
	if (handled) return;	//interrupt in interrupt, let the first ISR handle this
	This->_SC.setPosition(This->_ref_position);
	This->_SC.Timer.stop(false);
	This->_state = at_ref;
	This->debounce.clear();;
}

void Drive::checkISR(void *pThis) {
	Drive *This = (Drive*) pThis;
	bool handled = This->debounce.test_and_set();
	if (handled) return;	// interrupt in interrupt, let the first ISR handle this
	This->_ref_position = This->_SC.getPosition();
	This->_SC.Timer.stop(false);
	This->_state = at_ref;
	This->debounce.clear();;
}

void Drive::printSema() {
	printf("Drive 0x%lx, Sema %s: 0x%lx\n", (long unsigned int)this, TAG, (long unsigned int) &hLoopSema);
	fflush(stdout);
}

