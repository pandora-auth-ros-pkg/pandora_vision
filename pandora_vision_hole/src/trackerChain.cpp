/*#############################################################
 * Tracker.cpp
 *
 * File Description :
 *	Class for holding blobs for tracking
 *
 * Contents :	push()
 *
 * Author : Michael Skolarikis
 *
 * Date :	24-10-2011
 *
 * Change History :
 *
 *#############################################################
 */

#include "pandora_vision_hole/trackerChain.h"

using namespace std;

/**
 * Constructor
 */
TrackerChain::TrackerChain()
{
	m_id = (uintptr_t)this;
	
	/// find a random color, with which the blobs
	/// of this chain, will be visualized
	srand(m_id);
	int red = 0;
	int green = 0;
	int blue = 0;
	
	while (red + blue + green < 600)
	{
		red = rand() % 256;
		blue = rand() % 256;
		green = rand() % 256;
	}
		
	color = cvScalar( blue , green , red );
	
	/// set probability to 0
	probability = 0;
	inactive = false;
	
	ROS_INFO("New TrackerChain created! ID : %d", m_id );	
}

/**
 * Destructor
 */
TrackerChain::~TrackerChain()
{
	ROS_INFO("TrackerChain destroyed!");	
}

/**
 * Pushes a thing on the trackerchain
 * @param thingPtr
 */
void TrackerChain::push(Thing* thingPtr)
{
	thingPtr->m_chainId = m_id;
	this->push_back( thingPtr );
}
