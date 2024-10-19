/*
 * McpwmComparator.cpp
 *
 *  Created on: 28.05.2023
 *      Author: dad
 */

#include "McpwmComparator.h"
#include "esp_log.h"
#include <exception>

static const char *TAG = "McpwmComparator";

McpwmComparator::McpwmComparator(McpwmOperator &op) {
	mcpwm_comparator_config_t comparator_config = {
			.flags {
				.update_cmp_on_tez = 1,
				.update_cmp_on_tep = 1,
				.update_cmp_on_sync = 0
			}
	};
	ESP_ERROR_CHECK(mcpwm_new_comparator(op.getHandle(), &comparator_config, &hComparator));
}

McpwmComparator::~McpwmComparator() {
	assert(false);
	mcpwm_del_comparator(hComparator);
}

esp_err_t McpwmComparator::setValue(uint32_t val) {
	//ESP_LOGI(TAG, "setting compare value to %lu for comp 0x%x", val, (int)hComparator);
	return mcpwm_comparator_set_compare_value(hComparator, val);
}
