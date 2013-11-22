#include <propeller.h>
#include <cog.h>

#include "fdserial.h"
#include "simpletext.h"

// Genie commands & replys:

#define GENIE_ACK               0x06
#define GENIE_NAK               0x15

#define TIMEOUT_PERIOD          500
#define RESYNC_PERIOD           100

#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7

// Objects
//  the manual says:
//    Note: Object IDs may change with future releases; it is not
//    advisable to code their values as constants.

#define GENIE_OBJ_DIPSW         0
#define GENIE_OBJ_KNOB          1
#define GENIE_OBJ_ROCKERSW      2
#define GENIE_OBJ_ROTARYSW      3
#define GENIE_OBJ_SLIDER        4
#define GENIE_OBJ_TRACKBAR      5
#define GENIE_OBJ_WINBUTTON     6
#define GENIE_OBJ_ANGULAR_METER 7
#define GENIE_OBJ_COOL_GAUGE    8
#define GENIE_OBJ_CUSTOM_DIGITS 9
#define GENIE_OBJ_FORM          10
#define GENIE_OBJ_GAUGE         11
#define GENIE_OBJ_IMAGE         12
#define GENIE_OBJ_KEYBOARD      13
#define GENIE_OBJ_LED           14
#define GENIE_OBJ_LED_DIGITS    15
#define GENIE_OBJ_METER         16
#define GENIE_OBJ_STRINGS       17
#define GENIE_OBJ_THERMOMETER   18
#define GENIE_OBJ_USER_LED      19
#define GENIE_OBJ_VIDEO         20
#define GENIE_OBJ_STATIC_TEXT   21
#define GENIE_OBJ_SOUND         22
#define GENIE_OBJ_TIMER         23
#define GENIE_OBJ_SPECTRUM      24
#define GENIE_OBJ_SCOPE         25
#define GENIE_OBJ_TANK          26
#define GENIE_OBJ_USERIMAGES    27
#define GENIE_OBJ_PINOUTPUT     28
#define GENIE_OBJ_PININPUT      29
#define GENIE_OBJ_4DBUTTON      30
#define GENIE_OBJ_ANIBUTTON     31
#define GENIE_OBJ_COLORPICKER   32
#define GENIE_OBJ_USERBUTTON    33

// Structure to store replys returned from a display

#define GENIE_FRAME_SIZE        6

struct genieFrameReportObj
{
  int   cmd;
  int   object;
  int   index;
  int   data_msb;
  int   data_lsb;
  int   checksum;
};

/////////////////////////////////////////////////////////////////////
// The Genie frame definition
//
// The union allows the data to be referenced as an array of int
// or a structure of type genieFrameReportObj, eg
//
//  genieFrame f;
//  f.bytes[4];
//  f.reportObject.data_lsb
//
//  both methods get the same byte
//
union genieFrame
{
  int                 bytes[GENIE_FRAME_SIZE];
  genieFrameReportObj reportObject;
};

#define MAX_GENIE_EVENTS        16  // MUST be a power of 2
#define MAX_GENIE_FATALS        10

struct genieEventQueueStruct
{
  genieFrame  frames[MAX_GENIE_EVENTS];
  int         rd_index;
  int         wr_index;
  int         n_events;
};

typedef void  (*geniePutCharFuncPtr)      (int c, int baud);
typedef int   (*genieGetCharFuncPtr)      (void);
typedef void  (*genieUserEventHandlerPtr) (void);

/////////////////////////////////////////////////////////////////////
// User API functions
// These function prototypes are the user API to the library
//
extern int    genieBegin                (int rxpin, int txpin, int rstpin, int baud);
extern bool   genieReadObject           (int object, int index);
extern int    genieWriteObject          (int object, int index, int data);
extern void   genieWriteContrast        (int value);
extern int    genieWriteStr             (int index, char *string);
extern int    genieWriteStrU            (int index, char *string);
extern bool   genieEventIs              (genieFrame * e, int cmd, int object, int index);
extern int    genieGetEventData         (genieFrame * e); 
extern int    genieDoEvents             (void);
extern void   genieAttachEventHandler   (genieUserEventHandlerPtr userHandler);
extern bool   genieDequeueEvent         (genieFrame * buff);

#ifndef TRUE
#define TRUE  (1==1)
#define FALSE (!TRUE)
#endif

#define ERROR_NONE              0
#define ERROR_TIMEOUT           -1  // 255  0xFF
#define ERROR_NOHANDLER         -2  // 254  0xFE
#define ERROR_NOCHAR            -3  // 253  0xFD
#define ERROR_NAK               -4  // 252  0xFC
#define ERROR_REPLY_OVR         -5  // 251  0xFB
#define ERROR_RESYNC            -6  // 250  0xFA
#define ERROR_NODISPLAY         -7  // 249  0xF9
#define ERROR_BAD_CS            -8  // 248  0xF8

#define GENIE_LINK_IDLE         0
#define GENIE_LINK_WFAN         1 // waiting for Ack or Nak
#define GENIE_LINK_WF_RXREPORT  2 // waiting for a report frame
#define GENIE_LINK_RXREPORT     3 // receiving a report frame
#define GENIE_LINK_RXEVENT      4 // receiving an event frame
#define GENIE_LINK_SHDN         5

#define GENIE_EVENT_NONE        0
#define GENIE_EVENT_RXCHAR      1