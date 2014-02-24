/*#############################################################
 * TrackerChain.h
 *
 * File Description :
 *	TrackerChain Header file
 *	
 * Contents :
 *
 * Author : Michael Skolarikis
 *
 * Date :
 *
 * Change History : 11.05.11
 *
 *#############################################################
 */

#ifndef TRACKERCHAIN_H
#define TRACKERCHAIN_H

#include <stdint.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include "thing.h"
#include "ros/ros.h"
class TrackerChain : public std::vector<Thing*>
{
	
	public:										
		
		TrackerChain();					// constructor
		~TrackerChain();				// destructor
		
		void push(Thing* thingPtr);		// push a Thing* in the chain
				
		uintptr_t m_id; 				// id of chain
		
		bool active;					// true if the chain's last blob, is at most MIN_INACTIVITY frames old
		bool inactive; 					// true if the chain, is at least, is at least MAX_INACTIVITY frames old 
		int inactivity;					// shows how many frames have been elapsed, since the last blob has been inserted
		double probability;				// probability of the chain's last blob
		
		CvScalar color;					// color, with which the chain's blobs will be filled
};

#endif
