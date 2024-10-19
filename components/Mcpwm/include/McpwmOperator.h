/*
 * McpwnOperator.h
 *
 *  Created on: 26.05.2023
 *      Author: dad
 */
#ifndef MCPWNOPERATOR_H_
#define MCPWNOPERATOR_H_

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "McpwmTimer.h"

class McpwmOperator {
public:
	McpwmOperator(int GroupId);
	virtual ~McpwmOperator();
	mcpwm_oper_handle_t getHandle() {return hOperator;};

	esp_err_t connectTimer(McpwmTimer &timer);

private:
	int _GroupId;
	mcpwm_oper_handle_t hOperator;
};

#endif /* MCPWNOPERATOR_H_ */
