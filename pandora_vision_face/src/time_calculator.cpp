/*#############################################################
 * TimeCalculator.h
 *
 * File Description :
 *	Class for timing code blocks
 *	
 * Contents :	startTimer()
 * 				endTimer()
 * 				timeElapsed()
 *
 * Author : Michael Skolarikis
 *
 * Date :	24-10-2011
 *
 * Date :
 *
 * Change History :
 *
 *#############################################################
 */

#include <iostream>

#include "pandora_vision_face/time_calculator.h" 


using namespace std;


TimeCalculator::TimeCalculator(){}

TimeCalculator::~TimeCalculator(){}

long TimeCalculator::timeElapsed (struct timeval &t1, struct timeval &t2) {

	long sec, usec;

	sec = t2.tv_sec - t1.tv_sec;
	
	usec = t2.tv_usec - t1.tv_usec;
	
	if (usec < 0) {
		--sec;
		usec = usec + USEC_PER_SEC;
	}
	
	return sec*USEC_PER_SEC + usec;
}

void TimeCalculator::startTimer() {
	
	struct timezone tz;
	
	gettimeofday (&startTime, &tz);
	
}

long TimeCalculator::endTimer() {
	
	struct timezone tz;
	
	gettimeofday (&endTime, &tz);
	
	return timeElapsed(startTime, endTime);
	
}
