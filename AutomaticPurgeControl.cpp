#include "Arduino.h"
#include "AutomaticPurgeControl.h"


// Purge Values
#define PURGE_INTERVAL 2300
int PURGE_DURATION 0.2
// NEED TO FIND OUT LOOP TIME ThROUGH TESTING
#define PURGE_LOOP_TIME 0.5
#define PURGE_RESET_TIME 3

float purgeLastCallTime = 0;
float ampSecSincePurge = 0;
int purgeOpenCounter = 0;

AutomaticPurgeControl::AutomaticPurgeControl(void) {
}

bool AutomaticPurgeControl::openPurgeValve(bool purgeValveOpen, bool fcRelayOpen, float fcCurrent) {
	boolean newOpenValveState;
	float currentTime = millis();

	// If function hasn't been called in RESET_TIME, reset local variables
	if (currentTime - purgeLastCallTime >= PURGE_RESET_TIME) {
		ampSecSincePurge = 0;
		purgeOpenCounter = 0;
	}
	purgeLastCallTime = millis();

	// If purge valve is open, reset the counter. Else, add to ampSecSincePurge
	if (purgeValveOpen) {
		ampSecSincePurge = 0
	} else {
		ampSecSincePurge = ampSecSincePurge + (fcCurrent * PURGE_LOOP_TIME);
	}

	// If PURGE_INTERVAL exceeded, open valve and set the counter for some number of loops
	if (ampSecSincePurge > PURGE_INTERVAL) {
		newOpenValveState = true;
		purgeOpenCounter = PURGE_DURATION/PURGE_LOOP_TIME;
	} else {
		// If counter is > 0, keep the valve open and simply decrease the counter
		if (purgeOpenCounter > 0) {
			newOpenValveState = true;
			purgeOpenCounter = purgeOpenCounter - 1;
		} else {
			newOpenValveState = false;
		}
	}
	return newOpenValveState;
}
