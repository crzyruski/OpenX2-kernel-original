/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _PXA930_ACIPC_H_
#define _PXA930_ACIPC_H_

#define API_ALIGNMENT

/* user level ioctl commands for accessing APIs */
#define ACIPC_SET_EVENT         0
#define ACIPC_GET_EVENT         1
#define ACIPC_SEND_DATA      	2
#define ACIPC_READ_DATA         3
#define ACIPC_BIND_EVENT        4
#define ACIPC_UNBIND_EVENT      5
#define ACIPC_GET_BIND_EVENT_ARG	6

#define ACIPC_NUMBER_OF_EVENTS (10)
#define ACIPC_NUMBER_OF_INTERRUPTS (3)
#define ACIPC_INT0_EVENTS (0xff)
#define ACIPC_INT1_EVENTS (ACIPC_DDR_260_READY_REQ)
#define ACIPC_INT2_EVENTS (ACIPC_DDR_READY_REQ)

/* clients callback type*/
/*ICAT EXPORTED FUNCTION_TYPEDEF*/
typedef u32 (*acipc_rec_event_callback)(u32 events_status);
typedef u32 acipc_data;


enum DDR_mode {
	DDR_NOREQ = 0,
	DDR_208MHZ = 0x1,
	DDR_260MHZ = 0x2,
};

struct DDR_status {
	int mode;
	int needed_modes;
};

/* this enum define the event type*/
/*ICAT EXPORTED ENUM*/
typedef enum
{
	ACIPC_DDR_RELQ_REQ	= 0x00000001,
	ACIPC_DDR_RELQ_ACK	= 0x00000001,
	ACIPC_DDR_260_RELQ_REQ	= 0x00000002,
	ACIPC_DDR_260_RELQ_ACK	= 0x00000002,
	ACIPC_MSL_SLEEP_ALLOW	= 0x00000004,	
	ACIPC_MSL_WAKEUP_ACK	= 0x00000008,
	ACIPC_MSL_WAKEUP_REQ	= 0x00000010,
	ACIPC_DATA_IND          = 0x00000020,
	ACIPC_SPARE_2		= 0x00000040,
	ACIPC_SPARE_1		= 0x00000080,
	ACIPC_DDR_260_READY_REQ	= 0x00000100,
	ACIPC_DDR_260_READY_ACK	= 0x00000100,
	ACIPC_DDR_READY_REQ	= 0x00000200,
	ACIPC_DDR_READY_ACK	= 0x00000200,	
}acipc_events;

typedef enum
{
	ACIPC_RC_OK=0,
	ACIPC_HISTORICAL_EVENT_OCCUR,
	ACIPC_EVENT_ALREADY_BIND,	
	ACIPC_RC_FAILURE,
	ACIPC_RC_API_FAILURE,
	ACIPC_RC_WRONG_PARAM
}acipc_return_code;

/* used by clients when binding a callback to an event*/
/*ICAT EXPORTED ENUM*/
typedef enum
{
	ACIPC_CB_NORMAL=0,     /* callback will be called only if the DDR available        */	
	ACIPC_CB_ALWAYS_NO_DDR /* callback will be called always ,even if the DDR is not available*/	
}acipc_callback_mode;

typedef struct
{
	acipc_events IIR_bit;	
	acipc_callback_mode mode;
	u32 mask; /* add to support multiple events binding. see ACIPCEventBind for more details*/	
	acipc_rec_event_callback cb;
}acipc_database_cell;

/*CAT EXPORTED STRUCT*/
typedef struct  
{
	acipc_database_cell event_db[ACIPC_NUMBER_OF_EVENTS];
	acipc_callback_mode driver_mode;
	u32 int0_events_cnt;
	u32 historical_event_status;       /* hold status of events that occur before the clients bind their callback*/
}acipc_database;

/*#define ACIPC_DEBUG*/
#ifdef ACIPC_DEBUG
	#define IPCTRACE(format, args...) printk(KERN_INFO format, ## args)
	#define	IPC_ENTER()	printk("IPC: ENTER %s\n",__FUNCTION__)
	#define	IPC_LEAVE()	printk("IPC: LEAVE %s\n",__FUNCTION__)
#else
	#define IPCTRACE(s...)	do{}while(0)
	#define	IPC_ENTER()	do{}while(0)
	#define	IPC_LEAVE()	do{}while(0)
#endif

struct acipc_ioctl_arg{
	u32 arg;
	acipc_events set_event;
	acipc_callback_mode cb_mode;
};

#ifdef API_ALIGNMENT 
typedef u32 (*ACIPC_RecEventCB)(u32 eventsStatus);
typedef acipc_data ACIPC_Data;
typedef acipc_events ACIPC_EventsE;
typedef acipc_return_code ACIPC_ReturnCodeE;
typedef acipc_callback_mode ACIPC_CBModeE;
#endif

#endif  /* _PXA930_ACIPC_H_ */

