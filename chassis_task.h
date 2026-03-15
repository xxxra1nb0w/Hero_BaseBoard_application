/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "freertos.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "stdlib.h"
#include "can_comm.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//rocker value (max 660) change to vertial speed (m/s) 
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN -0.005f
//in following yaw angle mode, rocker value add to angle 
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//#define CHASSIS_ACCEL_X_NUM 0.1666666667f
//#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_X_NUM 0.0111111111f
#define CHASSIS_ACCEL_Y_NUM 0.2222222222f

//rocker value deadline
//ҡ������
#define CHASSIS_RC_DEADLINE 50

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER_F 0.283f//0.2f
#define MOTOR_DISTANCE_TO_CENTER_B 0.320f//0.2f

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//press the key, chassis will swing
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508 rmp change to chassis speed,
//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//�������̵������ٶ�
#define MAX_WHEEL_SPEED 5.0f
//chassis forward or back max speed
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 5.0f
//chassis left or right max speed
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 5.0f

#define CHASSIS_WZ_SET_SCALE 0.0f

// ============================================================
// 全向舵轮相关常量
// ============================================================
// GM6020 编码器弧度换算系数 (2*PI/8191)
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.00076708403f
#endif
// 角度转弧度宏
#ifndef DEG2R
#define DEG2R(x) ((x) * 3.14159265f / 180.0f)
#endif
// GM6020 转速转线速度系数 (m/s per rpm)
#define GM6020_RPM_TO_VECTOR  0.001746201886833f
// rpm 转 rad/s
#define RpmToOmega(rpm)       ((rpm) * 3.14159265f / 30.0f)

// 舵轮零点编码器值（需根据实际硬件标定，当前为参考值）
#define Forward_L_ecd  6834
#define Forward_R_ecd  2078
#define Back_L_ecd     7511
#define Back_R_ecd     3873

// 舵轮角度环 PID 参数（外环：编码器误差 → 速度设定）
#define RUDDER_ANGLE_PID_KP       20.0f
#define RUDDER_ANGLE_PID_KI       0.0f
#define RUDDER_ANGLE_PID_KD       0.0f
#define RUDDER_ANGLE_PID_MAX_OUT  300.0f
#define RUDDER_ANGLE_PID_MAX_IOUT 100.0f

// 舵轮速度环 PID 参数（内环：速度误差 → 电流）
#define RUDDER_SPEED_PID_KP       4.0f
#define RUDDER_SPEED_PID_KI       0.0f
#define RUDDER_SPEED_PID_KD       0.0f
#define RUDDER_SPEED_PID_MAX_OUT  16384.0f
#define RUDDER_SPEED_PID_MAX_IOUT 2000.0f

//when chassis is not set to move, swing max angle
//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 2700.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 10.0f//20.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 25.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  30.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f
//50 600

#define M3505_MOTOR_POWER_PID_KP 1.0f
#define M3505_MOTOR_POWER_PID_KI 0.0f//0.2f//0.5
#define M3505_MOTOR_POWER_PID_KD 0.0f
#define M3505_MOTOR_POWER_PID_MAX_OUT 10.0f//30
#define M3505_MOTOR_POWER_PID_MAX_IOUT 10.0f

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,  // chassis will follow yaw gimbal motor relative angle.���̻������̨��ԽǶ�
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW, // chassis will have yaw angle(chassis_yaw) close-looped control.�����е��̽Ƕȿ��Ʊջ�
    CHASSIS_VECTOR_NO_FOLLOW_YAW,      // chassis will have rotation speed control. ��������ת�ٶȿ���
    CHASSIS_VECTOR_RAW,                // control-current will be sent to CAN bus derectly.
    CHASSIS_VECTOR_BPIN,               // ����
    CHASSIS_VECTOR_TWIST,              // ����ҡ��ģʽ
	CHASSIS_VECTOR_TURN_ROUND,		   // һ����ͷģʽ

} chassis_mode_e;

/**
 * @brief  全向舵轮电机控制结构体（GM6020 方向电机）
 *         使用 Hero_Chasiss 原有 pid_type_def 实现级联 PID，
 *         替代 Base_Board 中的 Simulink Rudder_control。
 */
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;  // GM6020 电机数据指针
    pid_type_def angle_pid;    // 外环：编码器误差 → 速度设定
    pid_type_def speed_pid;    // 内环：速度误差  → 电流输出
    fp32 motor_speed;          // 当前转速 (rpm)
    int16_t given_current;     // 最终输出电流
    int16_t ecd_add;           // 运动学解算出的目标编码器增量
    int16_t last_ecd_add;      // 上次增量（备用）
    int16_t ecd_set;           // 目标编码器绝对值
    int16_t ecd_error;         // 编码器误差（带最短路径处理）
    fp32 wheel_speed;          // 该轮目标线速度
    fp32 rudder_angle;         // 当前舵轮目标角度 (rad)
    fp32 last_rudder_angle;    // 上一周期舵轮角度 (rad)
    int16_t ecd_zero_set;      // 零点编码器值（正前方）
    fp32 Judge_Speed_Direction;// 方向系数：+1 正转，-1 反转（角差>90°时翻向）
    fp32 Judge_Speed_cosk;     // cos³ 衰减系数（减小角差大时的轮速抖动）
} Rudder_Motor_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
    fp32 speed[4];
    int16_t max_current[4];
    int16_t current[4];
    fp32 totalCurrent;
    fp32 totalspeed;
    int16_t power_limit;
    int16_t k;                          // 功率控制系数

    uint16_t power_charge;              // 超级电容充电功率
    fp32 forecast_motor_power[4];       // 预测单电机功率
    fp32 forecast_total_power;          // 预测总功率
    fp32 POWER_MAX;
    fp32 MAX_current[4];

    // 舵轮功率控制（GM6020）
    struct {
        fp32 power_scale[4];      // 功率缩放系数
        fp32 sumPowerCmd;         // 预测功率之和
        fp32 alloctablePower;     // 可分配功率
        fp32 alloctableSumPower;  // 已分配功率总和
    } rudder_;
    fp32 SPEED_MIN;                     // 最小速度阈值（防止除零）
} power_ctrl_t;

/* ˫��ͨ�Žṹ�� */
// ����
typedef struct __attribute__((__packed__))
{
    uint16_t tx_current_heat;
    uint8_t tx_robo_level;
    uint16_t tx_initial_speed_x100;
    uint8_t tx_flag;
} comm_tx_a_t;
// ����
typedef struct __attribute__((__packed__))
{
    fp32 rx_vx_set;
    fp32 rx_vy_set;
} comm_rx_A_t;
typedef struct __attribute__((__packed__))
{
    fp32 rx_vz_set;
	uint8_t rx_flag_b;
    uint8_t rev;
    uint8_t rx_chassis_mode;
    uint8_t rx_Flag;
} comm_rx_B_t;
typedef struct __attribute__((__packed__))
{
    fp32 rx_PITCH;
    int16_t rx_first_speed;
	int16_t rx_back_speed;
} comm_rx_C_t;
typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����.
  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid
	const INS_t* chassis_INS_point;             // the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�
  
  bool_t twist_init_flag;//摇摆初始化
  int16_t change_twist_flag;

  // ============================================================
  // 全向舵轮电机（GM6020 方向电机）
  // ============================================================
  Rudder_Motor_t Forward_L;  // 左前舵轮（GM6020 ID 0x205）
  Rudder_Motor_t Forward_R;  // 右前舵轮（GM6020 ID 0x206）
  Rudder_Motor_t Back_L;     // 左后舵轮（GM6020 ID 0x207）
  Rudder_Motor_t Back_R;     // 右后舵轮（GM6020 ID 0x208）

  fp32    rudder_given_current[4]; // 舵轮输出电流 [FL,FR,BL,BR]
  fp32    rudder_speed[4];         // 舵轮线速度 (m/s)
  fp32    rudder_omega[4];         // 舵轮角速度 (rad/s)
  fp32    rudder_torque_current[4];// 舵轮力矩电流
  fp32    Encoder_add[4];          // 编码器目标增量缓存


  const fp32 *gimbal_INT_angle_point;
  const fp32 *gimbal_INT_gyro_point;
  //	  Power_Control  power_control;
  //	  Power_Control  rudder_power_control;

  pid_type_def buffer_pid;      // ���ʻ�PID
  power_ctrl_t power_control;
  const cap_measure_t *cap_data;
  int8_t BIG_cap;
  TickType_t BIG_cap_time;

  // ��ȡ����ϵͳ����
  ext_robot_state_t *robot_state; // ����ϵͳ������״̬
  ext_power_heat_data_t *power_heat_data;
  ext_shoot_data_t *shoot_data;

  // ˫��ͨ��
  // ����
  comm_tx_a_t comm_tx_a;    // ˫��ͨ������
  uint8_t comm_a_output[8]; // can���͵�����֡
  // ����
  comm_rx_A_t comm_rx_A; // ˫��ͨ������֡
  comm_rx_B_t comm_rx_B; // ˫��ͨ������֡
  comm_rx_C_t comm_rx_C; // ˫��ͨ������֡
  uint8_t rx_chassis_mode; // ������յ��ĵ���ģʽ
  fp32 rx_PITCH_angle; // ������յ���pitch�Ƕ�

  TickType_t CHASSIS_xTickCount; // ��ȡfreertosϵͳʱ�ӵδ���
} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);
void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_motor);
void RUDDER_POWER_CONTROL(chassis_move_t *chassis_motor);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
