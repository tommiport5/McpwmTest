/**
 * McpwmTimer
 * implementation
 */
#include "esp_err.h"
#include "esp_log.h"
#include "McpwmTimer.h"
#include "McpwmComparator.h"
#include "mcpwm_timer.h"
#include "esp_timer.h"
#include <stdexcept>

static const char *TAG = "McpwmTimer";
static const uint32_t MIN_PERIOD = 8;
// static const uint32_t MAX_PERIOD = 0xfffc;

void toggleDiagPin();
void setDiagPin(int);

static constexpr mcpwm_dev_t &lookup_dev(const int gid) {
	if (gid == 0) return MCPWM0;
	else if (gid == 1) return MCPWM1;
	else {
		ESP_LOGE(TAG, "Group Id may only be 0 or 1");
		throw std::range_error("Group Id may only be 0 or 1");
	}
	/* unreachable */
	return MCPWM0;
}

McpwmTimer::McpwmTimer(int GroupId) :
	_GroupId(GroupId), _state(stopped), _period(default_period), _max_period(0xfffc),
	_mcpwm_dev(lookup_dev(GroupId))
{
	_TaskToNotify = NULL;
	mcpwm_timer_config_t timer_config = {
			.group_id = _GroupId,
			.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
			.resolution_hz = Resolution,
			.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
			.period_ticks = default_period,
			.flags = {
						.update_period_on_empty = 1,
						.update_period_on_sync = 0
			},
	};
	ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &hTimer));
	mcpwm_timer_event_callbacks_t timer_callbacks = {
			.on_full =  NULL,
			.on_empty = NULL,
			.on_stop = &timerStop
	};
	ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(hTimer, &timer_callbacks, (void *)this));
}

McpwmTimer::~McpwmTimer() {
	assert(false);
	mcpwm_del_timer(hTimer);
}

void McpwmTimer::begin() {
	ESP_ERROR_CHECK(mcpwm_timer_enable(hTimer));
	setPeriod(_period);	// propagate down
	esp_timer_create_args_t etimer_args = {
			.callback = &oneShotISR,
			.arg = (void*) this,
			.dispatch_method = ESP_TIMER_ISR,
			.name = "oneShotTimer",
			.skip_unhandled_events = true
	};
	ESP_ERROR_CHECK(esp_timer_create(&etimer_args, &heTimer));
}

/*
 * from mcpwm_timer.c:
 * // in symmetric (up/down) mode, peak_ticks = period_ticks / 2
 */
void McpwmTimer::setPeriod(uint32_t prd) {
	unsigned char st = _state.load();
	printf("setting period to %ld (state 0x%hhx)\n", prd, st);
	if (prd < MIN_PERIOD){
		ESP_LOGE(TAG, "new period %lu out of range, ignored", prd);
		ESP_ERROR_CHECK(ESP_FAIL);	// track it the hard way
		return;
	}
	if (prd > _max_period) {
		ESP_LOGW(TAG, "period %ld > %d, stopping\n", prd, _max_period);
		stop(false);
		return;
	}
	_nxt_period = prd & ~7;
	//TODO: Check for no change
	if ((st & (stopping | stopped)) == 0) {
		_state = updating;
		printf("postponing setting of period %ld\n", _nxt_period.load());
		ESP_ERROR_CHECK(mcpwm_timer_start_stop(hTimer, MCPWM_TIMER_STOP_FULL));
		_TaskToNotify = xTaskGetCurrentTaskHandle();
		if (!xTaskNotifyWait(0,0, NULL, pdMS_TO_TICKS(100'000))) {
			//throw std::runtime_error("Notification timed out");
			ESP_LOGE(TAG, "Failed to notify timer stop in group %d", _GroupId);
		}
		printf("speed is set\n");
	} else {
		// printf("(1)setting period %ld\n", _nxt_period.load());
		ESP_ERROR_CHECK(mcpwm_timer_set_period(hTimer,_nxt_period.load()));
		_period = _nxt_period.load();
		printf("setting comp values %ld, %ld, %ld\nto handles 0x%x, 0x%x, 0x%x\n",
				_period.load() / 8, _period.load() / 4, _period.load() / 4 + _period.load() / 8,
				(unsigned int)(*_pComps)[0].getHandle(), (unsigned int)(*_pComps)[1].getHandle(), (unsigned int)(*_pComps)[2].getHandle());
		setCompValues(_period);
		// _state should be running
	}
}

uint32_t McpwmTimer::getPeriod() const {
	if ((_state.load() & updating) == updating) return _nxt_period.load();
	return _period.load();
}

bool IRAM_ATTR McpwmTimer::timerStop(mcpwm_timer_t* pTimer, const mcpwm_timer_event_data_t*,
		void* pThis) {
	McpwmTimer *This = (McpwmTimer *)pThis;
	unsigned char st = This->_state.load();
	if ((st & updating) == updating) {
		uint32_t npd = This->_nxt_period.load();
		if (npd >= This->_period.load()) {
			// set new period first, to avoid that comp values are too small
			//This->_mcpwm_dev.timer[pTimer->timer_id].timer_cfg0.timer_period = npd;
			ESP_ERROR_CHECK(mcpwm_timer_set_period(This->hTimer, npd));
		}
		This->setCompValues(npd);
		if (npd < This->_period.load()) {
			// set new period now, to avoid that new period is too small
			//This->_mcpwm_dev.timer[pTimer->timer_id].timer_cfg0.timer_period = npd;
			ESP_ERROR_CHECK(mcpwm_timer_set_period(This->hTimer, npd));
		}
		This->_period = npd;
		This->_state &= ~updating;
	}
	if (st & stopping) {
		//setDiagPin(0);
		This->_state = stopped;
	} else {
		mcpwm_timer_start_stop(This->hTimer, MCPWM_TIMER_START_NO_STOP);
		This->_state = running;
	}
	if (This->_TaskToNotify != NULL) {
		xTaskNotifyFromISR(This->_TaskToNotify, 0, eNoAction, NULL);	// no boost magic for the caller
		This->_TaskToNotify = NULL;
	}
	return false;
}

void McpwmTimer::start() {
	unsigned char st = _state.load();
	if (st & (updating|running)) {
		ESP_LOGW(TAG,"Timer starting from state 0x%x", (int)st);
		return;
	}
	printf("starting timer from state 0x%hhx\n", st);
	_state = running;
	ESP_ERROR_CHECK(mcpwm_timer_start_stop(hTimer, MCPWM_TIMER_START_NO_STOP));
}

void McpwmTimer::oneShotISR(void* pThis) {
	McpwmTimer *This = (McpwmTimer *)pThis;
	unsigned char st = This->_state.load();
	if (st == oneshot) {
		//setDiagPin(1);
		ESP_ERROR_CHECK(mcpwm_timer_start_stop(This->hTimer, MCPWM_TIMER_STOP_FULL));
		This->_state = stopping;
	}
}

void McpwmTimer::oneShot() {
	unsigned char st = _state.load();
	if (st != stopped) {
		ESP_LOGE(TAG,"Timer oneShot from state 0x%x", (int)st);
		return;
	}
	_state = oneshot;
	_TaskToNotify = NULL;
	ESP_ERROR_CHECK(mcpwm_timer_start_stop(hTimer, MCPWM_TIMER_START_NO_STOP));
	ESP_ERROR_CHECK(esp_timer_start_once(heTimer, 4000LL));	// we are using a speed of MIN_SPEED, i.e. a period of 13333 µsec
															// 4000 µ secs should bring us safely out of the EMPTY state
}

void McpwmTimer::stop(bool blocking/*=true*/) {
	unsigned char st = _state.load();
	//printf("timer stop from state 0x%x\n", (int)st);
	if (st == stopped) return;
	if (st == stopping) {
		if (blocking && _TaskToNotify != NULL) {
			ESP_LOGE(TAG, "Cannot notify task of stop: already one waiting");
		}
	} else {
		// stopping takes precedence
		_state = stopping;
		ESP_ERROR_CHECK(mcpwm_timer_start_stop(hTimer, MCPWM_TIMER_STOP_FULL));
	}
	if (blocking) {
		if (_TaskToNotify != NULL) {
			ESP_LOGE(TAG, "Cannot notify task of stop: already one waiting");
			return;
		}
		_TaskToNotify = xTaskGetCurrentTaskHandle();
		if (!xTaskNotifyWait(0,0, NULL, pdMS_TO_TICKS(100'000))) {
			//throw std::runtime_error("Notification timed out");
			ESP_LOGE(TAG, "Failed to notify timer stop in group %d", _GroupId);
		}
	}
	return;
}

void McpwmTimer::setCompValues(uint32_t prd) {
	ESP_ERROR_CHECK((*_pComps)[0].setValue(prd/8));
	ESP_ERROR_CHECK((*_pComps)[1].setValue(prd/4));
	ESP_ERROR_CHECK((*_pComps)[2].setValue(prd/8 + prd/4));
}
