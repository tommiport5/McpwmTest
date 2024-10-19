/*
 * McpwmComparator.h
 *
 *  Created on: 28.05.2023
 *      Author: dad
 */
#ifndef INCLUDE_MCPWMCOMPARATOR_H_
#define INCLUDE_MCPWMCOMPARATOR_H_

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "McpwmOperator.h"

class McpwmComparator {
public:
	McpwmComparator(McpwmOperator &op);
	virtual ~McpwmComparator();
	mcpwm_cmpr_handle_t getHandle() {return hComparator;};

	esp_err_t setValue(uint32_t val);

private:
	mcpwm_cmpr_handle_t hComparator;
};

#endif /* INCLUDE_MCPWMCOMPARATOR_H_ */
