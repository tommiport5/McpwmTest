/*
 * CompletionSema.h
 *
 *  Created on: 16.08.2023
 *      Author: dad
 */

#ifndef COMPLETIONSEMA_H_
#define COMPLETIONSEMA_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class CompletionSema {
public:
	CompletionSema();
	virtual ~CompletionSema();

	/**
	 * countPart
	 * increments the counter that defines
	 * how many completeParts are necessary for the next Completion
	 */
	void countPart();
	/**
	 * completePart
	 * when all Parts that were announced with countPart are completed,
	 * waitForCompletion is released
	 */
	void completePart();
	/**
	 * cancelPart
	 * Use this, if an error occurs that will prevent completion
	 */
	void cancelPart();
	void waitForCompletion();
	/**
	 * completed
	 * true, if currently no completion is in progress.
	 * Can be used for a handshake type of interface.
	 */
	bool completed() {return _NumParts == 0;}
	/**
	 * complete all pending parts
	 */
	void cleanup();
	void printStat();
private:
	int _NumParts;
	SemaphoreHandle_t hSema;
	SemaphoreHandle_t hMutex;
	bool _suppressEvent;
};

#endif /* COMPLETIONSEMA_H_ */
