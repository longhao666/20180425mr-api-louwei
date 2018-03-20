#ifndef __GRIPPER_H__
#define __GRIPPER_H__
#include "mrapi.h"
#include "module.h"

#define	SYS_HW_VERSION				0x00	// 固件版本
#define	SYS_ID						0x01	// 手抓ID
#define	SYS_MODEL_TYPE				0x02	// 驱动器型号：0x10-M14，0x20-M17,0x21-M17E,0x30-M20，0x80-Gripper
#define	SYS_FW_VERSION				0x03	// 固件版本
#define	SYS_ERROR					0x04	// 错误代码
#define	SYS_VOLTAGE					0x05	// 系统电压（0.01V）,暂不可用
#define	SYS_TEMP					0x06	// 系统温度（0.1℃）,暂不可用
#define	SYS_SERVO_LED				0x07	// 舵机LED:高8位-舵机ID，低8位-亮灭
#define	SYS_BAUDRATE_485			0x08	// RS485总线波特率(串口波特率)
#define	SYS_BAUDRATE_CAN			0x09	// CAN总线波特率
#define	SYS_ENABLE_TORQUE			0x0A	// 使能舵机力矩输出
#define	SYS_ENABLE_ON_POWER			0x0B	// 上电时使能舵机力矩输出
#define	SYS_SAVE_TO_FLASH			0x0C	// 保存数据到FLASH标志（自动清零）

#define	SYS_SET_ZERO_POS			0x0E	// 设置零点
#define	SYS_CLEAR_ERROR				0x0F	// 清除错误标志

#define	SYS_GET_STATE_MASK			0x10	// 更新舵机的信息，相应位置写1，自动清除
#define	SYS_POSITION_LEFT			0x11	// 左舵机当前位置
#define	SYS_POSITION_RIGHT			0x12	// 右舵机当前位置
#define	SYS_SPEED_LEFT				0x13	// 左舵机当前速度
#define	SYS_SPEED_RIGHT				0x14	// 右舵机当前速度
#define	SYS_LOAD_LEFT				0x15	// 左舵机当前负载
#define	SYS_LOAD_RIGHT				0x16	// 右舵机当前负载
#define	SYS_VOLTAGE_LEFT			0x17	// 左舵机当前电压（0.1V）
#define	SYS_VOLTAGE_RIGHT			0x18	// 右舵机当前电压（0.1V）
#define	SYS_TEMP_LEFT				0x19	// 左舵机当前温度（1℃）
#define	SYS_TEMP_RIGHT				0x1A	// 右舵机当前温度（1℃）

#define SYS_CURRENT					0x1B
#define SYS_ACC_X					0x1C
#define SYS_ACC_Y					0x1D
#define SYS_ACC_Z					0x1E

#define	TAG_MODE					0x20	// 工作模式：1-开合，2-位置
#define	TAG_OPEN_ANGLE				0x21	// 开合大小
#define	TAG_OPEN_STA				0x22	// 手指开合状态
#define	TAG_ZERO_LEFT				0x23	// 左舵机零点
#define	TAG_ZERO_RIGHT				0x24	// 右舵机零点
#define TAG_SPEED_LEFT				0x25	// 左舵机最大速度
#define	TAG_SPEED_RIGHT				0x26    // 右舵机最大速度
#define TAG_TORQUE_LEFT				0x27    // 左舵机最大力矩
#define TAG_TORQUE_RIGHT			0x28	// 右舵机最大力矩
#define	TAG_POSITION_LEFT			0x29	// 左舵机目标位置
#define	TAG_POSITION_RIGHT			0x2A	// 右舵机目标位置

#define	MAX_TORQUE					0x30	// 最大限制转矩(舵机ROM)
#define	LIMIT_TORQUE				0x31	// 最大限制转矩(舵机RAM)
#define	MOVING_SPEED				0x32	// 转动速度
#define	COMPLIANCE_MARGIN			0x33	// 位置误差
#define	COMPLIANCE_SLOPE			0x34	// 加速度
#define	PUNCH						0x35	//
#define	RETURN_DELAY				0x36	// Status返回延迟
#define	CW_ANGLE_LEFT				0x37	// 左舵机顺时针角度限制
#define	CCW_ANGLE_LEFT				0x38	// 左舵机逆时针角度限制
#define	CW_ANGLE_RIGHT				0x39	// 右舵机顺时针角度限制
#define	CCW_ANGLE_RIGHT				0x3A	// 右舵机逆时针角度限制
#define LIMIT_TEMP					0x3B	// 舵机温度报警阈值


typedef struct td_gripper
{
	Module* basicModule;
	uint16_t* gripperId;
	uint16_t* gripperType;
	uint8_t isOnline;
}Gripper;

#endif