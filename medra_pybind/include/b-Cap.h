/** @file b-Cap.h
 *
 *  @brief b-CAP client library header
 *
 *  @version	1.3
 *	@date		2013/10/15
 *	@author		DENSO WAVE (y)
 *
 */

/*
   [NOTES]
   This is a sample source code. Copy and modify this code in accordance with a device and a device version. Especially please note timeout and timeout-retry settings.
 */

#ifdef __cplusplus
extern "C" {
#endif


#ifndef B_CAP_H
#define B_CAP_H

/*#define	BCAP_CONNECTION_TCP		// TCP/IP (DEFAULT if BCAP_CONNECTION_XXX is not defined.) */
/*#define	BCAP_CONNECTION_UDP		// UDP/IP */
/*#define	BCAP_CONNECTION_COM		// COM(Serial device) */
#ifdef _MSC_VER
#define SERIAL_BAUDRATE     115200
#else
#define SERIAL_BAUDRATE     0x0000f /*B38400*/
#endif

#include "stdint.h"
typedef int32_t BCAP_HRESULT;

/**
 * @def BCAP_S_OK
 * @brief Succeeded.
 */
#define BCAP_S_OK ((BCAP_HRESULT)0L)

/**
 * @def BCAP_E_NOTIMPL
 * @brief Not implemented function is called.
 */
#define BCAP_E_NOTIMPL ((BCAP_HRESULT)0x80004001L)

/**
 * @def BCAP_E_ABORT
 * @brief Function aborted.
 */
#define BCAP_E_ABORT ((BCAP_HRESULT)0x80004004L)

/**
 * @def BCAP_E_FAIL
 * @brief Function failed.
 */
#define BCAP_E_FAIL ((BCAP_HRESULT)0x80004005L)

/**
 * @def BCAP_E_UNEXPECTED
 * @brief Fatal Error occurred.
 */
#define BCAP_E_UNEXPECTED ((BCAP_HRESULT)0x8000FFFFL)

/**
 * @def BCAP_E_INVALIDRCVPACKET
 * @brief Invalid packet is received.
 */
#define BCAP_E_INVALIDRCVPACKET ((BCAP_HRESULT)0x80010001L)

/**
 * @def BCAP_E_INVALIDSNDPACKET
 * @brief Invalid packet is sent.
 */
#define BCAP_E_INVALIDSNDPACKET ((BCAP_HRESULT)0x80010002L)

/**
 * @def BCAP_E_INVALIDARGTYPE
 * @brief Invalid argument type.
 */
#define BCAP_E_INVALIDARGTYPE ((BCAP_HRESULT)0x80010003L)

/**
 * @def BCAP_E_ROBOTISBUSY
 * @brief Robot is busy (Wait for a while).
 */
#define BCAP_E_ROBOTISBUSY ((BCAP_HRESULT)0x80010004L)

/**
 * @def BCAP_E_INVALIDCOMMAND
 * @brief Invalid command string is received.
 */
#define BCAP_E_INVALIDCOMMAND ((BCAP_HRESULT)0x80010005L)

/**
 * @def BCAP_E_PACKETSIZEOVER
 * @brief Received packet size over ( > 16Mbytes).
 */
#define BCAP_E_PACKETSIZEOVER ((BCAP_HRESULT)0x80010011L)

/**
 * @def BCAP_E_ARGSIZEOVER
 * @brief An argument siez over of the received packet. ( > 16Mbytes).
 */
#define BCAP_E_ARGSIZEOVER ((BCAP_HRESULT)0x80010012L)

/**
 * @def BCAP_E_ACCESSDENIED
 * @brief Access denied.
 */
#define BCAP_E_ACCESSDENIED ((BCAP_HRESULT)0x80070005L)

/**
 * @def BCAP_E_HANDLE
 * @brief Invalid handle.
 */
#define BCAP_E_HANDLE ((BCAP_HRESULT)0x80070006L)

/**
 * @def BCAP_E_OUTOFMEMORY
 * @brief Out of memory.
 */
#define BCAP_E_OUTOFMEMORY ((BCAP_HRESULT)0x8007000EL)

/**
 * @def BCAP_E_INVALIDARG
 * @brief Invalid argument.
 */
#define BCAP_E_INVALIDARG ((BCAP_HRESULT)0x80070057L)

/**
 * @def BCAP_E_COMMAND_BUFFER_EMPTY
 * @brief Slave mode command buffer is empty.
 */
#define BCAP_E_COMMAND_BUFFER_EMPTY ((BCAP_HRESULT)0x84201482L)

/**
 * @def BCAP_E_COMMAND_POSITION_GEN_STOPPED
 * @brief Command position generation is stopped.
 */
#define BCAP_E_COMMAND_POSITION_GEN_STOPPED ((BCAP_HRESULT)0x84201486L)

/**
 * @def BCAP_E_SLVMODE_OFF
 * @brief Slave move command was executed while slave mode was off.
 */
#define BCAP_E_SLVMODE_OFF ((BCAP_HRESULT)0x83500121L)

/* b-CAP Type id */
#define VT_EMPTY                0           /* (0Byte) */
#define VT_NULL                 1           /* (0Byte) */
#define VT_ERROR                10          /* (2Byte) */
#define VT_UI1                  17          /* (1Byte) */
#define VT_I2                   2           /* (2Byte) */
#define VT_UI2                  18          /* (2Byte) */
#define VT_I4                   3           /* (4Byte) */
#define VT_UI4                  19          /* (4Byte) */
#define VT_R4                   4           /* (4Byte) */
#define VT_R8                   5           /* (8Byte) */
#define VT_CY                   6           /* (8Byte) */
#define VT_DATE                 7           /* (8Byte) */
#define VT_BOOL                 11          /* (2Byte) */
#define VT_BSTR                 8           /* (ascii string length *2 + 4 Byte) */
                                            /* Double bytes per character */
#define VT_VARIANT              12          /* Variant */
#define VT_ARRAY                0x2000      /* Array */


/* b-CAP Utility macros */
#ifndef SUCCEEDED
#define SUCCEEDED(Status) ((BCAP_HRESULT)(Status) >= 0)
#endif

#ifndef FAILED
#define FAILED(Status) ((BCAP_HRESULT)(Status) < 0)
#endif

#include <stdint.h>

/* b-CAP standard types */
#ifndef u_char
typedef uint8_t u_char;
#endif
#ifndef u_short
typedef uint16_t u_short;
#endif


/*  b-CAP Variant Parameter */
typedef struct  {
    u_short Type;                           /* b-CAP Type id */
    uint32_t Arrays;                        /* Array count, must be >= 1 */
    union {
        u_char CharValue;
        u_short ShortValue;
        uint32_t LongValue;
        float FloatValue;
        double DoubleValue;

        u_char String[40 + 1];
        float FloatArray[16];
        double DoubleArray[16];

        void    *Data;                      /* When the Type is VT_Array, Value is stored in *Data */
        /* The client program must allocate memorys and set the pointer to *Data.  */
    } Value;
} BCAP_VARIANT;

/* b-CAP Functions */
BCAP_HRESULT    bCap_Open(const char *pIPStr, int iPort, int *piSockFd);
BCAP_HRESULT    bCap_Close(int iSockFd);

BCAP_HRESULT    bCap_ServiceStart(int iSockFd);
BCAP_HRESULT    bCap_ServiceStop(int iSockFd);

/* b-CAP Controller Functions */
BCAP_HRESULT    bCap_ControllerConnect(int iSockFd, const char *pStrCtrlname, const char *pStrProvName, const char *pStrPcName, const char *pStrOption, uint32_t *plhController);
BCAP_HRESULT    bCap_ControllerDisconnect(int iSockFd, uint32_t lhController);

BCAP_HRESULT    bCap_ControllerGetRobot(int iSockFd, uint32_t lhController, const char *pStrRobotName, const char *pStrOption, uint32_t *lhRobot);
BCAP_HRESULT    bCap_ControllerGetVariable(int iSockFd, uint32_t lhController, const char *pVarName, const char *pstrOption, uint32_t *plhVar);
BCAP_HRESULT    bCap_ControllerGetTask(int iSockFd, uint32_t lhController, char *pTskName, char *pstrOption, uint32_t *plhVar);
BCAP_HRESULT    bCap_ControllerExecute(int iSockFd, uint32_t lhController, const char *pStrCommand, const char *pStrOption, void *plResult);
BCAP_HRESULT    bCap_ControllerExecute2(int iSockFd, uint32_t lhController, const char *pStrCommand, const BCAP_VARIANT *pVntOption, BCAP_VARIANT *pvntResult);

/* b-CAP Robot Functions */
BCAP_HRESULT    bCap_RobotRelease(int iSockFd, uint32_t lhRobot);
BCAP_HRESULT    bCap_RobotGetVariable(int iSockFd, uint32_t lhRobot, const char *pVarName, const char *pStrOption, uint32_t *plhVar);
BCAP_HRESULT    bCap_RobotExecute(int iSockFd, uint32_t lhRobot, const char *pStrCommand, const char *pStrOption, void *plResult);
BCAP_HRESULT    bCap_RobotChange(int iSockFd, uint32_t lhRobot, char *pStrCommand);
BCAP_HRESULT    bCap_RobotMove(int iSockFd, uint32_t lhRobot, long lComp, const char *pStrPose, const char *pStrOption);
BCAP_HRESULT    bCap_RobotExecuteSlaveMove(int iSockFd, uint32_t lhRobot, char *pStrCommand, float *pfOption, void *pResult);
BCAP_HRESULT    bCap_RobotExecute2(int iSockFd, uint32_t lhRobot, const char *pStrCommand, const BCAP_VARIANT *pVntOption, BCAP_VARIANT *pvntResult);

/* b-CAP Task Functions */
BCAP_HRESULT    bCap_TaskRelease(int iSockFd, uint32_t lhTask);
BCAP_HRESULT    bCap_TaskGetVariable(int iSockFd, uint32_t lhTask, const char *pVarName, const char *pstrOption, uint32_t *plhVar);
BCAP_HRESULT    bCap_TaskStart(int iSockFd, uint32_t lhTask, long lMode, char *pStrOption);
BCAP_HRESULT    bCap_TaskStop(int iSockFd, uint32_t lhTask, long lMode, char *pStrOption);

/* b-CAP Variable Functions */
BCAP_HRESULT    bCap_VariableRelease(int iSockFd, uint32_t lhVar);
BCAP_HRESULT    bCap_VariableGetValue(int iSockFd, uint32_t lhVar, void *pVntValue);
BCAP_HRESULT    bCap_VariablePutValue(int iSockFd, uint32_t lhVar, u_short iType, uint32_t lArrays, void  *pVntValue);

#endif

#ifdef __cplusplus
}
#endif
