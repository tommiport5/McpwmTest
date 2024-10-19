/*
 * Events.h
 *
 *  Created on: 10.10.2023
 *      Author: dad
 */

#ifndef INCLUDE_EVENTS_H_
#define INCLUDE_EVENTS_H_

#include "esp_event.h"
#include "esp_event_base.h"

extern esp_event_loop_handle_t UserLoop;

// for records
enum in_pos_detail {position=0, pump};
enum record_detail {entered, finished};
enum emergency_detail {unexpected_refpoint, load_lost};

ESP_EVENT_DECLARE_BASE(INPUT_CHAR);
ESP_EVENT_DECLARE_BASE(RECORD);
ESP_EVENT_DECLARE_BASE(INPOS);
ESP_EVENT_DECLARE_BASE(TIMER);
ESP_EVENT_DECLARE_BASE(EMERGENCY);

#endif /* INCLUDE_EVENTS_H_ */
