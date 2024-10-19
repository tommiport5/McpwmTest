/*
 * CompletionSema.cpp
 *
 *  Created on: 16.08.2023
 *      Author: dad
 */

#include "CompletionSema.h"

#include "esp_err.h"
#include "esp_log.h"
#include "Events.h"

static const char *TAG = "CompletionSema";

static bool verbose=false;

CompletionSema::CompletionSema() : _NumParts(0), _suppressEvent(false) {
	hSema = NULL;
	hSema = xSemaphoreCreateCounting(10,0); // typically the max count is never reached
	if (hSema == NULL) {
		ESP_LOGE(TAG, "could not create semaphore");
	}
	hMutex = NULL;
	hMutex = xSemaphoreCreateMutex();
	if (hMutex == NULL) {
		ESP_LOGE(TAG, "could not create mutex");
	}
}

CompletionSema::~CompletionSema() {
	if (hSema) vSemaphoreDelete(hSema);
	if (hSema) vSemaphoreDelete(hSema);
}

void CompletionSema::countPart() {
	xSemaphoreTake(hMutex, portMAX_DELAY);
	_NumParts++;
	if (verbose) printf("Counting part %d\n", _NumParts);
	xSemaphoreGive(hMutex);
}

void CompletionSema::cancelPart() {
	xSemaphoreTake(hMutex, portMAX_DELAY);
	if (_NumParts > 0) {
		if (verbose) printf("canceled part %d\n", _NumParts);
		_suppressEvent = true;
		xSemaphoreGive(hSema);
	}
	xSemaphoreGive(hMutex);
}

void CompletionSema::completePart() {
	xSemaphoreTake(hMutex, portMAX_DELAY);
	if (verbose) printf("completePart %d\n", _NumParts);
	xSemaphoreGive(hSema);
	xSemaphoreGive(hMutex);
}

void CompletionSema::cleanup() {
	xSemaphoreTake(hMutex, portMAX_DELAY);
	while (_NumParts > 0) {
		xSemaphoreGive(hMutex);
		--_NumParts;
	}
	xSemaphoreGive(hMutex);
}

void CompletionSema::waitForCompletion() {
	xSemaphoreTake(hMutex, portMAX_DELAY);
	if (verbose) printf("waitForCompletion %d\n", _NumParts);
	do {
		xSemaphoreGive(hMutex);
		xSemaphoreTake(hSema, portMAX_DELAY);
		xSemaphoreTake(hMutex, portMAX_DELAY);
	} while (--_NumParts);
	if (!_suppressEvent) {
		ESP_ERROR_CHECK(esp_event_post_to(UserLoop, INPOS, position, NULL, 0, portMAX_DELAY));
		if (verbose) printf("sent completion\n");
	} else {
		_suppressEvent = false;
	}
	xSemaphoreGive(hMutex);
}

void CompletionSema::printStat() {
	printf("NumParts %d, Count %d\n", _NumParts, uxSemaphoreGetCount(hSema));
}

