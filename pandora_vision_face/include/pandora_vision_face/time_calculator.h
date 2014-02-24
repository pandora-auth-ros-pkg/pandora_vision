/*#############################################################
 * TimeCalculator.h
 *
 * File Description :
 *	TimeCalculator Header file
 *	
 * Contents :
 *
 * Author : mike
 *
 * Date :
 *
 * Change History :
 *
 *#############################################################
 */

#ifndef TIMECALC_H
#define TIMECALC_H

#include <sys/time.h>

#define USEC_PER_SEC 1000000L

class TimeCalculator{
	
	private:
		
		struct timeval startTime;
		struct timeval endTime;
		
		long timeElapsed (struct timeval &t1, struct timeval &t2);
		
	public:
	
		TimeCalculator();
		~TimeCalculator();
	
		void startTimer();
		long endTimer();
		 
};

#endif
