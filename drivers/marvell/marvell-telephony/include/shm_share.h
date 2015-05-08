/*

 *(C) Copyright 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All Rights Reserved
 */

#ifndef _SHM_SHARE_H_
#define _SHM_SHARE_H_

#define CiStubSvc_Id 1
#define NVMSvc_Id 2
#define CiDataStubSvc_Id 3
#define DiagSvc_Id 4
#define AudioStubSvc_Id 5
#define CiCsdStubSvc_Id 6
#define CiImsStubSvc_Id 9
#define CISTUB_PORT 1
#define NVMSRV_PORT 2
#define CIDATASTUB_PORT 3	/* ttc */
#define DIAG_PORT 4
#define AUDIOSTUB_PORT 5
#define CICSDSTUB_PORT 6
#define CIIMSSTUB_PORT 9
#ifdef CONFIG_SSIPC_SUPPORT
#define RAW_AT_PORT 7
#endif
#define TEST_PORT 8
#define TestPortSvc_Id 8
#ifdef CONFIG_SSIPC_SUPPORT
#define RAW_AT_DUN_PORT 10
#define RAW_AT_PROD_PORT 11
#define RAW_AT_SIMAL_PORT 12
#endif

#define MsocketLinkdownProcId   0xFFFE	/* For linkdown indication */
#define MsocketLinkupProcId             0xFFFD	/* For linkup indication */
#define MsocketSuspendNotify   0xFFFC                  //For system will enter in suspend indication

/* The linkup messages will be sent to the ports by this order. */
#define BroadCastSequence	{0, NVMSRV_PORT, DIAG_PORT, CISTUB_PORT, \
		CIDATASTUB_PORT, CICSDSTUB_PORT, TEST_PORT, AUDIOSTUB_PORT}
typedef struct _ShmApiMsg {
	int svcId;
	int procId;
	int msglen;
} ShmApiMsg;

#define SHM_HEADER_SIZE sizeof(ShmApiMsg)

#endif
