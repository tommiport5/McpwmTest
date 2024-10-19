/*
 * McpwmGenerator.cpp
 *
 *  Created on: 26.05.2023
 *      Author: dad
 */

#include "include/McpwmGenerator.h"
#include <utility>

McpwmGenerator::McpwmGenerator(McpwmOperator &op, gpio_num_t ionum, uint32_t invert)
	: _op(op), _ionum(ionum), _invert(invert) {
	mcpwm_generator_config_t generator_config = {
			.gen_gpio_num = ionum,
			.flags = {
				.invert_pwm = invert,
				.io_loop_back = 0
			}
	};
	ESP_ERROR_CHECK(mcpwm_new_generator(op.getHandle(), &generator_config, &hGenerator));
}

esp_err_t McpwmGenerator::rebuild(uint32_t invert) {
	mcpwm_generator_config_t generator_config = {
			.gen_gpio_num = _ionum,
			.flags = {
				.invert_pwm = invert,
				.io_loop_back = 0
			}
	};
	esp_err_t err = mcpwm_del_generator(hGenerator);
	if (err != ESP_OK) return err;
	_invert = invert;
	return mcpwm_new_generator(_op.getHandle(), &generator_config, &hGenerator);
}


McpwmGenerator::~McpwmGenerator() {
	assert(false);
	mcpwm_del_generator(hGenerator);
}

esp_err_t McpwmGenerator::setActionOnTimerEvent(
		mcpwm_gen_timer_event_action_t ev_act) {
	return mcpwm_generator_set_action_on_timer_event(hGenerator, ev_act);

}

esp_err_t McpwmGenerator::setActionOnCompareEvent(
		mcpwm_gen_compare_event_action_t ev_act) {
	return mcpwm_generator_set_action_on_compare_event(hGenerator, ev_act);
}

/*
esp_err_t McpwmGenerator::setActionsOnCompareEvent(mcpwm_gen_compare_event_action_t ev_act, ...) {
	esp_err_t err;
	va_list args;
	va_start(args, ev_act);
	while (ev_act.comparator != NULL) {
		err = mcpwm_generator_set_action_on_compare_event(hGenerator, ev_act);
		if (err != ESP_OK) {
			va_end(args);
			return err;
		}
		ev_act = va_arg(args, mcpwm_gen_compare_event_action_t);
	}
	va_end(args);
	return ESP_OK;
}
*/
esp_err_t McpwmGenerator::force(int level, bool hold) {
	return mcpwm_generator_set_force_level(hGenerator, level, hold);
}
