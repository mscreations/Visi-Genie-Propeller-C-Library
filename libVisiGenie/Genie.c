#include <propeller.h>
#include <cog.h>

#include "fdserial.h"
#include "mstimer.h"
#include "simpletext.h"
#include "Genie.h"

/////////////////////////// GenieArduino 27/09/2013 /////////////////////////
//
//      Library to utilise the 4D Systems Genie interface to displays
//      that have been created using the Visi-Genie creator platform.
//      This is intended to be used with the Arduino platform.
//
//		Updated by
//		4D Systems Engineering, September 2013, www.4dsystems.com.au
//		Written by 
//		Rob Gray (GRAYnomad), June 2013, www.robgray.com
//      Based on code by
//		Gordon Henderson, February 2013, <projects@drogon.net>
//
//      Copyright (c) 2012-2013 4D Systems PTY Ltd, Sydney, Australia
/*********************************************************************
 * This file is part of genieArduino:
 *    genieArduino is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    genieArduino is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with genieArduino.
 *    If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************/

//#include "genieArduino.h"

////////////////////////////////////////////////////////////
// Functions not available to the user code
//
/*void		_geniePutchar_Serial	(int c, int baud);
void		_geniePutchar_Serial1 	(int c, int baud);
void		_geniePutchar_Serial2 	(int c, int baud);
void		_geniePutchar_Serial3 	(int c, int baud);
void		_geniePutchar_SerialUSB (int c, int baud);
int	_genieGetchar_Serial	(void);
int	_genieGetchar_Serial1	(void);
int	_genieGetchar_Serial2	(void);
int	_genieGetchar_Serial3	(void);
int	_genieGetchar_SerialUSB	(void);*/
void		_genieFlushEventQueue	(void);
void		_handleError			(void);
void		_geniePutchar			(int c);
int		_genieGetchar			(void);
void		_genieSetLinkState		(int newstate);
int	_genieGetLinkState		(void);	
bool		_genieEnqueueEvent		(int * data);

fdserial *term;

//////////////////////////////////////////////////////////////
// A structure to hold up to MAX_GENIE_EVENTS events receive
// from the display
//
static genieEventQueueStruct _genieEventQueue;

//////////////////////////////////////////////////////////////
// Simple 5-deep stack for the link state, this allows 
// genieDoEvents() to save the current state, receive a frame,
// then restore the state
//
static int _genieLinkStates[5] = {GENIE_LINK_IDLE};
//
// Stack pointer
//
static int *_genieLinkState = &_genieLinkStates[0];

//////////////////////////////////////////////////////////////
// Number of mS the genieGetChar() function will wait before
// giving up on the display
static int _genieTimeout = TIMEOUT_PERIOD;

//////////////////////////////////////////////////////////////
// Number of times we have had a timeout
static int _genieTimeouts = 0;

//////////////////////////////////////////////////////////////
// Global error variable
static int _genieError = ERROR_NONE;


static int	rxframe_count = 0;

//////////////////////////////////////////////////////////////
// Number of fatal errors encountered
static int _genieFatalErrors = 0;

//////////////////////////////////////////////////////////////
// Pointer to the user's event handler function
//
static genieUserEventHandlerPtr _genieUserHandler = NULL;

////////////////////// genieGetEventData ////////////////////////
//
// Returns the LSB and MSB of the event's data combined into
// a single uint16
//
// The data is transmitted from the display in big-endian format 
// and stored the same so the user can't just access it as an int 
// directly from the structure. 
//
int genieGetEventData (genieFrame * e) {
	return  (e->reportObject.data_msb << 8) + e->reportObject.data_lsb;
}

//////////////////////// genieEventIs ///////////////////////////
//
// Compares the cmd, object and index fields of the event's 
// structure.
//
// Returns:		TRUE if all the fields match the caller's parms
//				FALSE if any of them don't
//
bool genieEventIs(genieFrame * e, int cmd, int object, int index) {

	return (e->reportObject.cmd == cmd &&
		e->reportObject.object == object &&
		e->reportObject.index == index);

}

////////////////////// _genieWaitForIdle ////////////////////////
//
// Wait for the link to become idle or for the timeout period, 
// whichever comes first.
//
void _genieWaitForIdle (void) {
	int do_event_result;
	long timeout = mstime_get() + _genieTimeout;

	for ( ; mstime_get() < timeout;) {
		do_event_result = genieDoEvents();

		// if there was a character received from the 
		// display restart the timeout because doEvents
		// is in the process of receiving something
		if (do_event_result == GENIE_EVENT_RXCHAR) {
			timeout = mstime_get() + _genieTimeout;
		}
		
		if (_genieGetLinkState() == GENIE_LINK_IDLE) {
			return;
		}
	}
	_genieError = ERROR_TIMEOUT;
	_handleError();
	return;
}

////////////////////// _geniePushLinkState //////////////////////
//
// Push a link state onto a FILO stack
//
void _geniePushLinkState (int newstate) {

	_genieLinkState++;
	_genieSetLinkState(newstate);

}

////////////////////// _geniePopLinkState //////////////////////
//
// Pop a link state from a FILO stack
//
void _geniePopLinkState (void) {
	if (_genieLinkState > &_genieLinkStates[0]) {
		*_genieLinkState = 0xFF;
		_genieLinkState--;
	}
}

///////////////////////// genieDoEvents /////////////////////////
//
// This is the heart of the Genie comms state machine.
//
int genieDoEvents (void) {
	int c;
	static int	rx_data[6];
	static int	checksum = 0;

	c = _genieGetchar();

	////////////////////////////////////////////
	//
	// If there are no characters to process and we have 
	// queued events call the user's handler function.
	//
	if (_genieError == ERROR_NOCHAR) {
		if (_genieEventQueue.n_events > 0 && _genieUserHandler!= NULL) (_genieUserHandler)();
		return GENIE_EVENT_NONE;
	}
	
	///////////////////////////////////////////
	//
	// Main state machine
	//
	switch (_genieGetLinkState()) {
		case GENIE_LINK_IDLE:
			switch (c) {
				case GENIE_REPORT_EVENT:
				// event frame out of the blue, set the link state
				// and fall through to the frame-accumulate code
				// at the end of this function
				_geniePushLinkState(GENIE_LINK_RXEVENT);
				break;
					
				default:
				// error, bad character, no other character 
				// is acceptable in this state
				return GENIE_EVENT_RXCHAR;
				
			}
			break;
				
		case GENIE_LINK_WFAN:
			switch (c) {

				case GENIE_ACK:
					_geniePopLinkState();
					return GENIE_EVENT_RXCHAR;

				case GENIE_NAK:
					_geniePopLinkState();
					_genieError = ERROR_NAK;
					_handleError();
					return GENIE_EVENT_RXCHAR;
			
				case GENIE_REPORT_EVENT:
					// event frame out of the blue while waiting for an ACK
					// save/set the link state and fall through to the 
					// frame-accumulate code at the end of this function
					_geniePushLinkState(GENIE_LINK_RXEVENT);
					break;

				case GENIE_REPORT_OBJ:
				default:
					// error, bad character
					return GENIE_EVENT_RXCHAR;	
			}
			break;

		case GENIE_LINK_WF_RXREPORT: // waiting for the first byte of a report
			switch (c) {
			
				case GENIE_REPORT_EVENT:
				// event frame out of the blue while waiting for the first
				// byte of a report frame
				// save/set the link state and fall through to the
				// frame-accumulate code at the end of this function
				_geniePushLinkState(GENIE_LINK_RXEVENT);
				break;

				case GENIE_REPORT_OBJ:
				// first byte of a report frame
				// replace the GENIE_LINK_WF_RXREPORT link state 
				// with GENIE_LINK_RXREPORT to indicate that we
				// are now receiving a report frame
				_geniePopLinkState();
				_geniePushLinkState(GENIE_LINK_RXREPORT);
				break;

				case GENIE_ACK:
				case GENIE_NAK:
				default:
				// error, bad character
				return GENIE_EVENT_RXCHAR;
//				break;
			}

		case GENIE_LINK_RXREPORT:		// already receiving report
		case GENIE_LINK_RXEVENT:		// already receiving event
		default:
			break;
		
	}

	///////////////////////////////////////////////////////
	// We get here if we are in the process of receiving 
	// a report or event frame. Accumulate GENIE_FRAME_SIZE 
	// bytes into a local buffer then queue them as a frame
	// into the event queue
	//
	if (_genieGetLinkState() == GENIE_LINK_RXREPORT || \
		_genieGetLinkState() == GENIE_LINK_RXEVENT) {
			
		checksum = (rxframe_count == 0) ? c : checksum ^ c;

		rx_data[rxframe_count] = c;

		if (rxframe_count == GENIE_FRAME_SIZE -1) {
			// all bytes received, if the CS is good 
			// queue the frame and restore the link state
			if (checksum == 0) {
				_genieEnqueueEvent(rx_data);
				rxframe_count = 0;
				// revert the link state to whatever it was before
				// we started accumulating this frame
				_geniePopLinkState();
				return GENIE_EVENT_RXCHAR;
			} else {
				_genieError = ERROR_BAD_CS;
				_handleError();
			}	
		}
		rxframe_count++;
		return GENIE_EVENT_RXCHAR;
	}
}

/////////////////// _genieFatalError ///////////////////////
//
void _genieFatalError(void) {

	if (_genieFatalErrors++ > MAX_GENIE_FATALS) {
//		*_genieLinkState = GENIE_LINK_SHDN;
//		_genieError = ERROR_NODISPLAY;
	}
}

///////////////// _genieFlushSerialInput ///////////////////
//
// Removes and discards all characters from the currently 
// used serial port's Rx buffer.
//
void _genieFlushSerialInput(void) {
	do {
		_genieGetchar();
	} while (_genieError != ERROR_NOCHAR);
}

/////////////////////// genieResync //////////////////////////
//
// This function does nothing for RESYNC_PERIOD to allow the display 
// time to stop talking, then it flushes everything so the link 
// can start again.
//
// Untested, will need work I'm sure.
//
void genieResync (void) {
	
	for (long timeout = mstime_get() + RESYNC_PERIOD ; mstime_get() < timeout;) {};

	_genieFlushSerialInput();
	_genieFlushEventQueue();
	_genieTimeouts = 0;
	_genieGetLinkState() == GENIE_LINK_IDLE;

}

///////////////////////// _handleError /////////////////////////
//
// So far really just a debugging aid, but can be enhanced to
// help recover from errors.
//
void _handleError (void) {
//	Serial2.write (_genieError + (1<<5));
//	if (_genieError == GENIE_NAK) genieResync();
}

////////////////////// _genieFlushEventQueue ////////////////////
//
// Reset all the event queue variables and start from scratch.
//
void _genieFlushEventQueue(void) {
	_genieEventQueue.rd_index = 0;
	_genieEventQueue.wr_index = 0;
	_genieEventQueue.n_events = 0;
}

////////////////////// genieDequeueEvent ///////////////////
//
// Copy the bytes from a queued input event to a buffer supplied 
// by the caller.
//
// Parms:	genieFrame * buff, a pointer to the user's buffer
//
// Returns:	TRUE if there was an event to copy
//			FALSE if not
//
bool genieDequeueEvent(genieFrame * buff) {

	if (_genieEventQueue.n_events > 0) {

    for (int i = 0; i < GENIE_FRAME_SIZE; i++)
    {
      (*buff).bytes[i] = _genieEventQueue.frames[_genieEventQueue.rd_index].bytes[i];
    }
   
		_genieEventQueue.rd_index++;
		_genieEventQueue.rd_index &= MAX_GENIE_EVENTS -1;
		_genieEventQueue.n_events--;
		return TRUE;
	} 
	return FALSE;
}

////////////////////// _genieEnqueueEvent ///////////////////
//
// Copy the bytes from a buffer supplied by the caller 
// to the input queue 
//
// Parms:	int * data, a pointer to the user's data
//
// Returns:	TRUE if there was an empty location in the queue
//				to copy the data into
//			FALSE if not
// Sets:	ERROR_REPLY_OVR if there was no room in the queue
//
bool _genieEnqueueEvent (int * data) {

	if (_genieEventQueue.n_events < MAX_GENIE_EVENTS-2) {
//		memcpy (&_genieEventQueue.frames[_genieEventQueue.wr_index], data, 
	//			GENIE_FRAME_SIZE);
    for(int j = 0; j < 6; j++)
    {
      _genieEventQueue.frames[_genieEventQueue.wr_index].bytes[j] = data[j];
    }
		_genieEventQueue.wr_index++;
		_genieEventQueue.wr_index &= MAX_GENIE_EVENTS -1;
		_genieEventQueue.n_events++;
		return TRUE;
	} else {
		_genieError = ERROR_REPLY_OVR;
		_handleError();
		return FALSE;
	}
}

//////////////////////// genieReadObject ///////////////////////
//
// Send a read object command to the Genie display. Note that this 
// function does not wait for the reply, that will be read in due 
// course by genieDoEvents() and subsequently by the user's event 
// handler.
//
bool genieReadObject (int object, int index) {

	int checksum;

	_genieFlushEventQueue();	// Discard any pending reply frames

	_genieWaitForIdle();

	_genieError = ERROR_NONE;

	_geniePutchar(GENIE_READ_OBJ); checksum   = GENIE_READ_OBJ ;
	_geniePutchar(object);         checksum  ^= object ;
	_geniePutchar(index);          checksum  ^= index ;
	_geniePutchar(checksum);

	_geniePushLinkState(GENIE_LINK_WF_RXREPORT);

	return TRUE;
}


///////////////////// _genieSetLinkState ////////////////////////
//
// Set the logical state of the link to the display.
//
// Parms:	int newstate, a value to be written to the 
//				link's _genieLinkState variable. Valid values are
//		GENIE_LINK_IDLE			0
//		GENIE_LINK_WFAN			1 // waiting for Ack or Nak
//		GENIE_LINK_WF_RXREPORT	2 // waiting for a report frame
//		GENIE_LINK_RXREPORT		3 // receiving a report frame
//		GENIE_LINK_RXEVENT		4 // receiving an event frame
//		GENIE_LINK_SHDN			5
//
void _genieSetLinkState (int newstate) {
	
	*_genieLinkState = newstate;

	if (newstate == GENIE_LINK_RXREPORT || \
		newstate == GENIE_LINK_RXEVENT)
		rxframe_count = 0;	
}

/////////////////////// _genieGetLinkState //////////////////////
//
// Get the current logical state of the link to the display.
//
int _genieGetLinkState (void) {
	return *_genieLinkState;
}

///////////////////////// genieWriteObject //////////////////////
//
// Write data to an object on the display
//
int genieWriteObject (int object, int index, int data)
{
	int msb, lsb ;
	int checksum ;

	_genieWaitForIdle();

	lsb = data & 0xFF;
	msb = (data >> 8) & 0xFF;

	_genieError = ERROR_NONE;

	_geniePutchar(GENIE_WRITE_OBJ) ; checksum  = GENIE_WRITE_OBJ ;
	_geniePutchar(object) ;          checksum ^= object ;
	_geniePutchar(index) ;           checksum ^= index ;
	_geniePutchar(msb) ;             checksum ^= msb;
	_geniePutchar(lsb) ;             checksum ^= lsb;
	_geniePutchar(checksum) ;

	_geniePushLinkState(GENIE_LINK_WFAN);
	
}

/////////////////////// genieWriteContrast //////////////////////
// 
// Alter the display contrast (backlight)
//
// Parms:	int value: The required contrast setting, only
//		values from 0 to 15 are valid. 0 or 1 for most displays
//      and 0 to 15 for the uLCD-43
//
void genieWriteContrast (int value) {
	unsigned int checksum ;

	_genieWaitForIdle();

	_geniePutchar(GENIE_WRITE_CONTRAST) ; checksum  = GENIE_WRITE_CONTRAST ;
	_geniePutchar(value) ;                checksum ^= value ;
	_geniePutchar(checksum) ;

	_geniePushLinkState(GENIE_LINK_WFAN);

}

//////////////////////// _genieWriteStrX ///////////////////////
//
// Non-user function used by genieWriteStr() and genieWriteStrU()
//
static int _genieWriteStrX (int code, int index, char *string)
{
	char *p ;
	unsigned int checksum ;
	int len = strlen (string) ;

	if (len > 255)
	return -1 ;

	_genieWaitForIdle();

	_geniePutchar(code) ;               checksum  = code ;
	_geniePutchar(index) ;              checksum ^= index ;
	_geniePutchar((unsigned char)len) ; checksum ^= len ;
	for (p = string ; *p ; ++p)	{
		_geniePutchar (*p) ;
		checksum ^= *p ;
	}
	_geniePutchar(checksum) ;

	_geniePushLinkState(GENIE_LINK_WFAN);

	return 0 ;
}

/////////////////////// genieWriteStr ////////////////////////
//
// Write a string to the display (ASCII)
//
int genieWriteStr (int index, char *string) {
 
  return _genieWriteStrX (GENIE_WRITE_STR, index, string);

}

/////////////////////// genieWriteStrU ////////////////////////
//
// Write a string to the display (Unicode)
//
int genieWriteStrU (int index, char *string) {

  return _genieWriteStrX (GENIE_WRITE_STRU, index, string);

}

/////////////////// genieAttachEventHandler //////////////////////
//
// "Attaches" a pointer to the users event handler by writing 
// the pointer into the variable used by doEVents()
//
void genieAttachEventHandler (genieUserEventHandlerPtr handler) {
	_genieUserHandler = handler;
}

//////////////////////// _genieGetchar //////////////////////////
//
// Get a character from the selected Genie serial port
//
// Returns:	ERROR_NOHANDLER if an Rx handler has not 
//				been defined
//			ERROR_NOCHAR if no bytes have beeb received
//			The char if there was one to get
// Sets:	_genieError with any errors encountered
//
int _genieGetchar() 
{
	_genieError = ERROR_NONE;

  if (fdserial_rxReady(term) == 0) {
		_genieError = ERROR_NOCHAR;
		return ERROR_NOCHAR;  
	}	
  return (int) fdserial_rxChar(term) & 0xFF;
}


/////////////////////// _geniePutchar ///////////////////////////
//
// Output the supplied character to the Genie display over 
// the selected serial port
//
void _geniePutchar(int c) 
{
  fdserial_txChar(term, c);
}

//////////////////////////////////// genieSetup /////////////////////////////////////////

int genieBegin(int rxpin, int txpin, int rstpin, int baud) 
{
	term = fdserial_open(rxpin, txpin, 0, baud);

  mstime_start();
	
  set_direction(rstpin, 0);
  low(rstpin);
  pause(500);
  high(rstpin);
	return true;
}