/*
 * Drive.h
 *
 *  Created on: 07.07.2023
 *      Author: dad
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <deque>
#include <atomic>
#include <array>

#include "McpwmStepperControl.h"
#include "CompletionSema.h"

class Drive {
	friend class MachineConfigurator;
	static const uint32_t STACKSIZE = 4096;
	//std::array<float, 3> _pid;
	int32_t default_speed;
	int32_t ref_point_speed;
	float min_cont_dist;
	float _tps_limit;

	struct ipol {
		float next_speed, speed_step;
		float ticks_since_last, ticks_per_sample;
		int num_steps[2];
		uint8_t substate;
		int stepcount;
	} _idat;

	Drive(const Drive &) = delete;
	Drive &operator=(const Drive &) = delete;
public:
	Drive(McpwmStepperControl & MSC, CompletionSema & CS);
	virtual ~Drive();

	void begin();
	bool forward(int32_t speed=0); // etc. ..
	bool backward(int32_t speed=0);
	/**
	 * stop
	 * immediately, e.g at reference point
	 */
	void stop(bool release=false);
	/**
	 * halt
	 * ramp down and stop
	 */
	void halt(bool blocking=true);
	void release();
	/**
	 * position to pos
	 * speed is the maximum speed (limited by the overall max)
	 * exact means exact stop at the end
	 */
	bool position(int32_t pos, const uint32_t *nom_speed=NULL, bool exact=true);
	bool positionLead(int32_t pos, const float nom_speed);
	const struct ipol *getMydat(){return &_idat;};
	bool positionFollow(int32_t pos, const float nom_speed, const struct ipol *lead_dat);
	bool searchRef(int32_t new_value);
	bool checkRef();
	int32_t getRefPos() {return _ref_position;};
	int32_t getPosition() {return _SC.getPosition();};
	void setSpeeds(int32_t min, int32_t max) {_SC.setSpeeds(min, max); setSpeeds();};
	uint32_t getMinSpeed() {return _SC.MIN_SPEED;};
	uint32_t getMaxSpeed() {return _SC.MAX_SPEED;};
	int getId() {return _SC._GroupID;};
	void showPeriods() {_SC.showPeriods();};

	void loop();

	enum state {stopped, free_running, reverse_speed, speed_target, positioning, wait_exact, in_pos, at_ref, leaving_ref};
	bool isStopped() {return _state == stopped;};
	state getState() {return _state;}
	// get the last target position (which might not have been reached (yet)
	int32_t getLastTarget();

	int32_t getCurrentSpeed() {return _SC.getCurrentSpeed();};
	bool isReferencePoint() {return _SC.isReferencePoint();};
	bool completed() {return _CS.completed();};
	void printSema();

private:
	char TAG[32];
	McpwmStepperControl &_SC;
	CompletionSema &_CS;
	bool _RefPointTop;

	void setSpeeds();

	/**
	 * move
	 * with a new speed, without decel or accel, but reversing if necessary
	 */
	void move(float ts);
	bool final_approach(int32_t CurPos);

	float calcDistanceFromSpeed(int32_t speed);
	float calcDecLimit(int32_t speed);
	float reachableSpeed(int32_t dist);

	static void loopTask(void*);

	state _state, _last;

	float _aps;		// acceleration per sample
	float _target_speed;

	int32_t _target_position;
	int32_t _origin_position;
	int32_t _ref_position;
	float _step;
	bool _exact;
	bool _awaitingCompletion;
	int _printHelper;

	int count;
	bool once;
	// FreeRTOS Semaphore
	SemaphoreHandle_t hLoopSema;

	std::atomic_flag debounce;
	static void searchISR(void *pThis);
	static void checkISR(void *pThis);

};

#endif /* DRIVE_H_ */
