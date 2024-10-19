/*
 * McpwnOperator.cpp
 *
 *  Created on: 26.05.2023
 *      Author: dad
 */

#include "include/McpwmOperator.h"

McpwmOperator::McpwmOperator(int GroupId) : _GroupId(GroupId)
{
	mcpwm_operator_config_t operator_config = {
			.group_id = _GroupId,
			.flags {
				.update_gen_action_on_tez = 1,
				.update_gen_action_on_tep = 1,
				.update_gen_action_on_sync = 0,
				.update_dead_time_on_tez = 0,
				.update_dead_time_on_tep = 0,
				.update_dead_time_on_sync = 0
			}
	};
	ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &hOperator));

}

McpwmOperator::~McpwmOperator() {
	assert(false);
	mcpwm_del_operator(hOperator);
}

esp_err_t McpwmOperator::connectTimer(McpwmTimer &timer) {
	return mcpwm_operator_connect_timer(hOperator, timer.getHandle());
}
