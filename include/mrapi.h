//  mrapi.h
//
//  ~~~~~~~~~~~~
//
//  Modular-Robot API
//
//  ~~~~~~~~~~~~
//
//  ------------------------------------------------------------------
//  Author : Lou Wei
//	Last change: 19.01.2018 Beijing
//
//  Language: ANSI-C
//  ------------------------------------------------------------------
//
//  Copyright (C) 2015-2018  Aubo Robotics, Beijing
//  more Info at http://www.aubo-robotics.cn 
//
#ifndef _MRAPI_H_
#define _MRAPI_H_
#include <stdint.h>

#ifndef _WINDOWS
#define __stdcall
#endif

////////////////////////////////////////////////////////////
// Value definitions
////////////////////////////////////////////////////////////

#define MASTER(x) x

// Represent the joint types supported by this API
// 
#define MODEL_TYPE_M14        0x010		// 
#define MODEL_TYPE_M17        0x020		//
#define MODEL_TYPE_M17V2      0x021		//
#define MODEL_TYPE_M20        0x030		//
#define MODEL_TYPE_M20V2      0x031		//
#define MODEL_TYPE_M20V3      0x032		//
#define MODEL_TYPE_LIFT       0x040		//

// Represent the work modes supported by Joint
// 
#define MODE_OPEN             0       // Open mode
#define MODE_CURRENT          1       // Current/Torque servo mode
#define MODE_SPEED            2       // Speed servo mode
#define MODE_POSITION         3       // Position servo mode
#define MODE_CYCLESYNC        4       //

// Represent the MRAPI error and status codes
//
#define MR_ERROR_OK                0x00000U  // No error
#define MR_ERROR_TIMEOUT           0x00001U  // Read/Write timeout error
#define MR_ERROR_BUSY              0x00002U  // Read/Write in progress
#define MR_ERROR_ACK0              0x00004U  // Write acknowlogy 0
#define MR_ERROR_ACK1              0x00008U  // Write acknowlogy 1
#define MR_ERROR_QXMTFULL          0x00010U  // Transmit queue is full
#define MR_ERROR_QXMTEMPTY         0x00020U  // Transmit queue is empty
#define MR_ERROR_ILLDATA           0x00040U  // Invalid data, function, or action

// Represent the Error bit mask
// 
#define ERROR_MASK_OVER_CURRENT   0x0001    // Current overflow
#define ERROR_MASK_OVER_VOLTAGE   0x0002    // Voltage overflow
#define ERROR_MASK_UNDER_VOLTAGE  0x0004    // Voltage underflow
#define ERROR_MASK_OVER_TEMP      0x0008    // Temperature too high
#define ERROR_MASK_BATTERY        0x0010    // Battery voltage low
#define ERROR_MASK_ENCODER        0x0020    // Encoder Error
#define ERROR_MASK_POTEN          0x0040    // Poten Error
#define ERROR_MASK_CURRENT_INIT   0x0080    // Current calibration Error
#define ERROR_MASK_FUSE           0x0100    // Fuse off

// Represent the Scope bit mask
// 
#define SCP_MASK_TAGCUR				0x0001		// Bit of monitoring target current
#define SCP_MASK_MEACUR				0x0002		// Bit of monitoring actual current
#define SCP_MASK_TAGSPD				0x0004		// Bit of monitoring target speed
#define SCP_MASK_MEASPD				0x0008		// Bit of monitoring actual speed
#define SCP_MASK_TAGPOS				0x0010		// Bit of monitoring target position
#define SCP_MASK_MEAPOS				0x0020		// Bit of monitoring actual position

#define isJointType(t) (t==MODEL_TYPE_M14)||(t==MODEL_TYPE_M17)||(t==MODEL_TYPE_M17V2)||(t==MODEL_TYPE_M20)||(t==MODEL_TYPE_M20V2) \
						||(t==MODEL_TYPE_M20V3)||(t==MODEL_TYPE_LIFT)
#define isJointMode(t) (t==MODE_OPEN)||(t==MODE_CURRENT)||(t==MODE_SPEED)||(t==MODE_POSITION)||(t==MODE_CYCLESYNC)

////////////////////////////////////////////////////////////
// Structure definitions
////////////////////////////////////////////////////////////

/// <summary>	Defines an alias representing handle of the joint. </summary>
///
/// <remarks>	Represents a Joint hardware channel handle </remarks>

typedef void* JOINT_HANDLE; 
typedef int32_t(*jQueShortHandler_t)(JOINT_HANDLE pJoint, uint16_t len);
typedef int32_t(*jCallback_t)(uint16_t id, uint16_t index, void* args);

#ifdef __cplusplus
extern "C" {
#define _DEF_ARG =0
#else
#define _DEF_ARG
#endif

////////////////////////////////////////////////////////////
// Modular-Robot API function declarations
////////////////////////////////////////////////////////////


/// <summary>	Starts a can device as a master. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="masterId">	Identifier for the master. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall startMaster(
	char* busname,
	uint8_t masterId);

/// <summary>	Stops a master. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="masterId">	Identifier for the master. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall stopMaster(
	uint8_t masterId);

/// <summary>	Join master. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="masterId">	Identifier for the master. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall joinMaster(
	uint8_t masterId);


/// <summary>	Sets control loop frequency. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="hz">	The Hz. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall setControlLoopFreq(
	int32_t hz);

/// <summary>	Joint up. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="id">	  	The identifier. </param>
/// <param name="canSend">	[in,out] If non-null, the can send. </param>
/// <returns>	A JOINT_HANDLE. </returns>
JOINT_HANDLE __stdcall jointUp(
	uint16_t jointId, 
	uint8_t masterId); //construct Joint and put it in joint stack


/// <summary>	Joint down. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">	The joint. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t      __stdcall jointDown(
	JOINT_HANDLE pJoint);          //destruct joint and remove it from joint stack


/// <summary>	Joint select. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="id">	The identifier. </param>
/// <returns>	A JOINT_HANDLE. </returns>
JOINT_HANDLE __stdcall jointSelect(
	uint16_t id);  //find joint by it's ID


/// <summary>	Joint start servo. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint"> 	The joint. </param>
/// <param name="handler">	The handler. </param>
void __stdcall jointStartServo(
	JOINT_HANDLE pJoint, 
	jQueShortHandler_t handler);


/// <summary>	Joint stop servo. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">	The joint. </param>
void __stdcall jointStopServo(
	JOINT_HANDLE pJoint);


/// <summary>	Joint push. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="h">	  	The JOINT_HANDLE to process. </param>
/// <param name="pos">	  	[in,out] If non-null, the position. </param>
/// <param name="speed">  	[in,out] If non-null, the speed. </param>
/// <param name="current">	[in,out] If non-null, the current. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointPush(
	JOINT_HANDLE h, 
	int32_t* pos, 
	int32_t* speed, 
	int32_t* current);


/// <summary>	Joint poll. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="h">	  	The JOINT_HANDLE to process. </param>
/// <param name="pos">	  	[in,out] If non-null, the position. </param>
/// <param name="speed">  	[in,out] If non-null, the speed. </param>
/// <param name="current">	[in,out] If non-null, the current. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointPoll(
	JOINT_HANDLE h, 
	int32_t* pos, 
	int32_t* speed, 
	int32_t* current);


/// <summary>	Joint poll scope. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="h">	  	The JOINT_HANDLE to process. </param>
/// <param name="pos">	  	[in,out] If non-null, the position. </param>
/// <param name="speed">  	[in,out] If non-null, the speed. </param>
/// <param name="current">	[in,out] If non-null, the current. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointPollScope(
	JOINT_HANDLE h, 
	int32_t* pos, 
	int32_t* speed, 
	int32_t* current);


/// <summary>	Joint get identifier. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetId(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get type. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetType(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get error. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetError(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get voltage. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetVoltage(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get temporary. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetTemp(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get baudrate. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetBaudrate(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get current. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetCurrent(
	JOINT_HANDLE pJoint, 
	uint32_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get speed. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetSpeed(
	JOINT_HANDLE pJoint, 
	uint32_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get position. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetPosition(
	JOINT_HANDLE pJoint, 
	uint32_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get mode. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetMode(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get maximum speed. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetMaxSpeed(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get maximum acceleration. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetMaxAcceleration(
	JOINT_HANDLE pJoint, 
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint ge position limit. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="data">	   	[in,out] If non-null, the data. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGePositionLimit(
	JOINT_HANDLE pJoint,
	uint16_t* data, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get curr p. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="pValue">  	[in,out] If non-null, the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetCurrP(
	JOINT_HANDLE pJoint,
	uint16_t* pValue, int32_t timeout,
	jCallback_t callBack);

/// <summary>	Joint get curr i. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="iValue">  	[in,out] If non-null, zero-based index of the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetCurrI(
	JOINT_HANDLE pJoint, 
	uint16_t* iValue, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get speed p. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="pValue">  	[in,out] If non-null, the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetSpeedP(
	JOINT_HANDLE pJoint,
	uint16_t* pValue, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get speed i. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="iValue">  	[in,out] If non-null, zero-based index of the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetSpeedI(
	JOINT_HANDLE pJoint, 
	uint16_t* iValue, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get position p. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="pValue">  	[in,out] If non-null, the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetPositionP(
	JOINT_HANDLE pJoint,
	uint16_t* pValue,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint get position ds. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="dsValue"> 	[in,out] If non-null, the ds value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointGetPositionDs(
	JOINT_HANDLE pJoint, 
	uint16_t* dsValue, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint set identifier. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="id">	   	The identifier. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointSetID(
	JOINT_HANDLE pJoint,
	uint16_t id, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>	Joint set baudrate. </summary>
/// <remarks>	Louwei, 2018/1/19. </remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="baud">	   	The baud. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>	A MRAPI error code. </returns>
int32_t __stdcall jointSetBaudrate(
	JOINT_HANDLE pJoint, 
	uint16_t baud, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set enable.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="isEnable">	The is enable. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetEnable(
	JOINT_HANDLE pJoint, 
	uint16_t isEnable,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set power on status.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="isEnable">	The is enable. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetPowerOnStatus(
	JOINT_HANDLE pJoint,
	uint16_t isEnable,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set save 2 flash.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetSave2Flash(
	JOINT_HANDLE pJoint, 
	int32_t timeout,
	jCallback_t callBack);

/// <summary>Joint set zero.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetZero(
	JOINT_HANDLE pJoint,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set clear error.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetClearError(
	JOINT_HANDLE pJoint, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set mode.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="mode">	   	The mode. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetMode(
	JOINT_HANDLE pJoint, 
	uint16_t mode, 
	int32_t timeout,
	jCallback_t callBack);

/// <summary>Joint set speed.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="speed">   	The speed. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetSpeed(
	JOINT_HANDLE pJoint, 
	int32_t speed,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set position.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="position">	The position. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetPosition(
	JOINT_HANDLE pJoint,
	int32_t position, 
	int32_t timeout,
	jCallback_t callBack);

/// <summary>Joint set maximum speed./</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="maxspeed">	The maxspeed. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetMaxSpeed(
	JOINT_HANDLE pJoint, 
	int32_t maxspeed, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set maximum acceleration.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="maxacc">  	The maxacc. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetMaxAcceleration(
	JOINT_HANDLE pJoint,
	int32_t maxacc, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set position limit.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">	   	The joint. </param>
/// <param name="position_min">	The position minimum. </param>
/// <param name="position_max">	The position maximum. </param>
/// <param name="timeout">	   	The timeout. </param>
/// <param name="callBack">	   	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetPositionLimit(
	JOINT_HANDLE pJoint,
	int32_t position_min,
	int32_t position_max,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set curr p.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="pValue">  	The value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetCurrP(
	JOINT_HANDLE pJoint, 
	uint16_t pValue, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set curr i.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="iValue">  	Zero-based index of the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetCurrI(
	JOINT_HANDLE pJoint, 
	uint16_t iValue, 
	int32_t timeout,
	jCallback_t callBack);

/// <summary>Joint set speed p.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="pValue">  	The value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetSpeedP(
	JOINT_HANDLE pJoint,
	uint16_t pValue,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set speed i.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="iValue">  	Zero-based index of the value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetSpeedI(
	JOINT_HANDLE pJoint, 
	uint16_t iValue, 
	int32_t timeout,
	jCallback_t callBack);

/// <summary>Joint set position p.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="pValue">  	The value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetPositionP(
	JOINT_HANDLE pJoint, 
	uint16_t pValue, 
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set position ds.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="dsValue"> 	The ds value. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetPositionDs(
	JOINT_HANDLE pJoint,
	uint16_t dsValue,
	int32_t timeout, 
	jCallback_t callBack);

/// <summary>Joint set scp mask.</summary>
/// <remarks>Louwei, 2018/1/19.</remarks>
/// <param name="pJoint">  	The joint. </param>
/// <param name="mask">	   	The mask. </param>
/// <param name="timeout"> 	The timeout. </param>
/// <param name="callBack">	The call back. </param>
/// <returns>A MRAPI error code.</returns>
int32_t __stdcall jointSetScpMask(
	JOINT_HANDLE pJoint, 
	uint16_t mask, 
	int32_t timeout, 
	jCallback_t callBack);

#ifdef __cplusplus
}
#endif
#endif

