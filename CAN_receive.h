/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      CAN中断接收函数接收电机数据，CAN发送函数发送电流控制电机。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "freertos.h"

#define CHASSIS_CAN hcan2
#define CAPID_CAN hcan2

#define GIMBAL_CAN 	hcan2
#define POWER_CAN   hcan2
#define SHOOT_CAN 	hcan1
#define CAP_CAN		hcan2
#define UI_CAN		hcan1
#define TUI_CAN		hcan1


#define CAN_FEEDBACK_FREAM_ID_A       0x11
#define CAN_FEEDBACK_FREAM_ID_B       0x12
#define CAN_CTRL_FREAM_ID             0x21       // CAN帧ID
/* CAN send and receive ID */
typedef enum
{
	//Can2
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	

	//Can1
	// CAN_SHOOT_ALL_ID = 0x200,
	// CAN_3508_LEFT_ID = 0x201,
	// CAN_3508_RIGHT_ID = 0x202,
	// CAN_3508_BLEFT_ID = 0x203,
	// CAN_3508_BRIGHT_ID = 0x204,
	
	CAN_GIMBAL_ALL_ID = 0x1FF,
	CAN_YAW_MOTOR_ID = 0x207,//1
	CAN_PIT_MOTOR_ID = 0x206,

	CAN_TRIGGER_MOTOR_ID = 0x207,
	
	

	CAN_CAP_ID=0x211, // 电容板


} can_msg_id_e;

// 全向舵轮 GM6020 接收 ID（CAN2，0x205-0x208）
#define CAN_RUDDER_FL_ID  0x205   // Forward_L
#define CAN_RUDDER_FR_ID  0x206   // Forward_R
#define CAN_RUDDER_BL_ID  0x207   // Back_L
#define CAN_RUDDER_BR_ID  0x208   // Back_R

typedef struct
{
    float err;                    // 错误状态
    float status;                 // 运行状态
    float chassis_power;          // 底盘功率
    float cap_volt;               // 电容电压
    float power_target;           // 目标功率
    float cap_precentage;         // 电容剩余能量百分比
} cap_measure_t; // meng电容板

//typedef struct
//{
//    int16_t chassis_power;					// 底盘功率
//    int16_t cap_volt;								// 电容电压
//    int16_t cap_curr;								// 电容电流
//	  int16_t cap_energy;							// 电容剩余能量百分比
////	  int16_t sumChassisEnergy;				// 底盘总能量
//} cap_measure_t; // 溪流电容板

// 电源来源
typedef enum
{
    BATTERY = 1,
    CAPACITY,
    OUT_OFF,
}power_source_enum; 
typedef struct 
{
    uint16_t    input_voltage;      // 输入电压
    uint16_t    current;            // 输入电流
    uint16_t    cap_voltage;        // 电容电压
    uint8_t     p_set;              // 设定功率
    uint8_t     crc_checksum;
}can_feedback_a_typedef;  // CAN反馈数据A
typedef struct 
{
    uint16_t    output_voltage;     // 输出电压
    uint8_t     power_source:7;       // 电源来源
    uint8_t     out_auto_en:1;     // 输出使能是否自动
    uint8_t     nc1;                
    uint8_t     nc2;                
    uint8_t     nc3;                
    uint8_t     nc4;                // 保留
    uint8_t     crc_checksum;
}can_feedback_b_typedef;  // CAN反馈数据B

typedef struct 
{
    uint8_t     p_set;                  // 设定功率
    uint8_t     power_source:7;           // 控制电源来源-1:前级电源  2:电容  3:关闭
    uint8_t     out_auto_en:1;     // 输出使能是否自动
    uint16_t    freq_feedback:15;       // 反馈频率，默认100
    uint16_t    wireless_en:1;          // 无线使能
    uint8_t     nc1;                
    uint8_t     nc2;                
    uint8_t     nc3;                    // 保留
    uint8_t     crc_checksum;
}can_control_typedef;  // CAN控制数据

// RM 电机数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


typedef struct
{
    fp32 voltage;
    fp32 cuttent;
    fp32 power;
} pm_measure_t;

extern TickType_t REPLACE_COMM_A_TIME, REPLACE_COMM_B_TIME, REPLACE_COMM_C_TIME;

/**
  * @brief          发送云台电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      rev1: (0x207) 备用电机控制电流
  * @param[in]      rev2: (0x208) 备用电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

extern void CAN_cmd_pitch(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

extern can_feedback_a_typedef get_capA;
extern can_feedback_b_typedef get_capB;
/**
  * @brief          发送ID为0x700的CAN帧，重置3508电机ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          发送底盘电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          发送发射机构电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      s0: (0x201) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      s1: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      s2: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      trigger: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t left, int16_t right, int16_t bleft, int16_t bright);
extern void CAN_cmd_trigger(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief          电容板CAN发送功率控制
 * @author         LYH
 * @param[in]      target_power: (15-350w)
                   flag:0:禁止;1:使能
 * @retval         发送状态
 */
extern void CAN_cmd_cap(int16_t target_power, uint16_t flag);

////溪流电容板CAN发送
//void CAN_cmd_cap(int16_t temPower);

extern void CAN_cmd_ui(int16_t key1, int16_t key2, int16_t key3, int16_t key4);
extern void	CAN_cmd_ui2(int16_t Key1, int16_t Key2, int16_t Key3, int16_t Key4);


/**
 * @brief          获取电容板数据指针
 * @param[in]      none
 * @retval         电容数据指针
 */
extern const cap_measure_t *get_cap_data_point(void);

/**
 * @brief          获取yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          获取pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          返回拨弹轮 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机索引, 范围 [0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回摩擦轮 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_2006_measure_point(void);
		
/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_3508_left_measure_point(void);

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_3508_right_measure_point(void);

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_3508_bleft_measure_point(void);

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_3508_bright_measure_point(void);

// ============================================================
// 全向舵轮 GM6020 电机数据与控制接口
// ============================================================
// 舵轮电机数据数组：[0]=Forward_L, [1]=Forward_R, [2]=Back_L, [3]=Back_R
extern motor_measure_t rudder_motor[4];

/**
  * @brief          发送舵轮方向电机控制电流 (0x205,0x206,0x207,0x208) via 0x1FF on CAN2
  * @param[in]      forward_L: (0x205) GM6020电流, 范围 [-16384,16384]
  * @param[in]      forward_R: (0x206) GM6020电流, 范围 [-16384,16384]
  * @param[in]      back_L:    (0x207) GM6020电流, 范围 [-16384,16384]
  * @param[in]      back_R:    (0x208) GM6020电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_rudder(int16_t forward_L, int16_t forward_R, int16_t back_L, int16_t back_R);

/**
  * @brief          获取舵轮电机数据指针
  * @param[in]      i: 索引 0=FL, 1=FR, 2=BL, 3=BR
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_rudder_motor_measure_point(uint8_t i);

#endif