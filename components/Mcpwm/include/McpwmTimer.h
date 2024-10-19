/**
 * McpwmTimer class
 * a mcpwm timer with all the necessary handling
 */
#ifndef COMPONENTS_MCPWMTIMER_H_
#define COMPONENTS_MCPWMTIMER_H_

#include "hal/gpio_types.h"
#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "esp_timer.h"
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <chrono>
using namespace std::chrono_literals;

class McpwmComparator;

typedef McpwmComparator (*Comps_t)[3];

// ! MCPWM_LL_MAX_COUNT_VALUE = 0xffff

class McpwmTimer {
	static const uint32_t Resolution = 327868;	// 0.327868 MHz: 1 Tick = 3.005 µs, shortest period  4*3.0 = 12 µs ~ 83.3 kHz (seems to be the longest possible)
	static const uint32_t default_period = 0xfffc;	//  longest 65532 ticks, (must be dividible by 4), 1 step_longest_period ~ 1.6 s
public:
    McpwmTimer(int GroupId);
    virtual ~McpwmTimer();
	void begin();
	void start();
	void oneShot();
	void stop(bool blocking = true);	// blocking
	void setPeriod(uint32_t prd);
	mcpwm_timer_handle_t getHandle() const {return hTimer;};
	uint32_t getPeriod() const;
	bool isRunning() const {return _state.load() & (updating | running);};

	void setComparators(Comps_t comps) {
		_pComps = comps;
	}

	int getMaxPeriod() {return _max_period;};

private:
	static bool IRAM_ATTR timerStop(mcpwm_timer_t *, const mcpwm_timer_event_data_t *, void *);
	static void oneShotISR(void *);
    int _GroupId;

    enum State {stopped=1, stopping=2, updating=4, running=8, oneshot=16};

    std::atomic_uchar _state;

	std::atomic_uint32_t _period;
	std::atomic_uint32_t _nxt_period;
	int _max_period;

	mcpwm_timer_handle_t hTimer;
	mcpwm_dev_t &_mcpwm_dev;
	esp_timer_handle_t heTimer;

	TaskHandle_t _TaskToNotify;

	Comps_t _pComps;
	void setCompValues(uint32_t prd);
};

#endif
