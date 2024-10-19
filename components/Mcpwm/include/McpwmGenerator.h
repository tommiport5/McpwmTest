/*
 * McpwmGenerator.h
 *
 *  Created on: 26.05.2023
 *      Author: dad
 */
#ifndef INCLUDE_MCPWMGENERATOR_H_
#define INCLUDE_MCPWMGENERATOR_H_

extern "C" {
#include "hal/gpio_types.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
}
#include "McpwmOperator.h"

class McpwmGenerator {
public:
	McpwmGenerator(McpwmOperator &op, gpio_num_t ionum, uint32_t invert=0);
	virtual ~McpwmGenerator();
	mcpwm_gen_handle_t getHandle() { return hGenerator;};

	esp_err_t setActionOnTimerEvent(mcpwm_gen_timer_event_action_t ev_act);
	template <class... Args>
		esp_err_t setActionsOnTimerEvents(Args ...ev_act) {
		return mcpwm_generator_set_actions_on_timer_event(hGenerator, std::forward<Args>(ev_act)...);
	}
	esp_err_t setActionOnCompareEvent(mcpwm_gen_compare_event_action_t ev_act);
//	esp_err_t setActionsOnCompareEvent(mcpwm_gen_compare_event_action_t ev_act, ...);
	template <class... Args>
	esp_err_t setActionsOnCompareEvents(Args ...ev_act) {
		return mcpwm_generator_set_actions_on_compare_event(hGenerator, std::forward<Args>(ev_act)...);
	}
	/**
	 * rebuild
	 * rebuilds this generator, optionally with a different invert value.
	 * Note that the actions must be set new afterwards
	 */
	esp_err_t rebuild(uint32_t invert);

	esp_err_t force(int level, bool hold);

private:
	McpwmOperator _op;
	gpio_num_t _ionum;
	uint32_t _invert;
	mcpwm_gen_handle_t hGenerator;
};

#endif /* INCLUDE_MCPWMGENERATOR_H_ */
