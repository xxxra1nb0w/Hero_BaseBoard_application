/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "bsp_usart.h"
#include "Mathh.h"
#include "referee.h"

int anglesr;
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
 * @brief          初始化"chassis_move"结构体，初始化PID，获取遥控器指针，
 *                 初始化3508电机结构体，初始化云台电机结构体，初始化底盘电机角度结构体
 * @param[out]     chassis_move_init:"chassis_move"结构体指针
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

static void chassis_get_comm_data(chassis_move_t *receive_comm_data);

// ============================================================
// 全向舵轮相关静态函数声明
// ============================================================
// 舵轮运动学分解：vx/vy/wz → 各轮速度 + 目标角度
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t *chassic_rudder_preliminary_solution);
// 舵轮角度控制主循环（驱动4个GM6020）
static void rudder_control_loop(chassis_move_t *rudder_move_control_loop);
// 单个舵轮相对角度计算与方向处理
static void Rudder_motor_relative_angle_control(Rudder_Motor_t *chassis_motor);
// 单个舵轮 PID 计算（外环角度 + 内环速度，使用pid_type_def）
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t *rudder_motor);
// /**
//  * @brief          设置底盘模式，如需修改请直接更改'chassis_behaviour_mode_set'
//  * @param[out]     chassis_move_mode:"chassis_move"结构体指针
//  * @retval         none
//  */
// static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          模式切换过渡处理，当模式改变时，需要将一些变量清零，
 *                 例如滤波器值、PID积分值等，确保平滑切换
 * @param[out]     chassis_move_transit:"chassis_move"结构体指针
 * @retval         none
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
 * @brief          更新底盘反馈数据，读取电机速度、编码器角度，
 *                 计算机体速度、角度，以及IMU姿态
 * @param[out]     chassis_move_update:"chassis_move"结构体指针
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

// /**
//  * @brief          设置底盘控制量（速度、角度等）
//  * @param[out]     chassis_move_control:"chassis_move"结构体指针
//  * @retval         none
//  */
// static void chassis_set_control(chassis_move_t *chassis_move_control);

/**
 * @brief          主控制循环：运动学解算、PID计算、输出限制
 * @param[out]     chassis_move_control_loop:"chassis_move"结构体指针
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

// 双通道通信数据打包
static void comm_data_pack(chassis_move_t *send_data_pack);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
// 底盘控制结构体
chassis_move_t chassis_move;
extern int8_t QA, BPIN, FOLLOW;
int8_t DBUS_error_flag = 0;
extern fp32 RX_PITCH, RX_first_speed, RX_back_speed;
int8_t KEY_shift, KEY_z, KEY_c, KEY_q; // 双通道按键标志
int16_t cnts = 0;
extern gimbal_control_t gimbal_control;
extern gimbal_behaviour_e gimbal_behaviour;
extern vision_rxfifo_t *vision_rx;
int8_t turn_flags = 0;
extern int MODE;
/**
 * @brief          底盘任务入口，周期 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 参数
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // 延迟一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    // 初始化底盘
    chassis_init(&chassis_move);
    // 等待底盘电机和遥控器就绪
    /*while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }*/

    while (1)
    {
		chassis_move.CHASSIS_xTickCount = xTaskGetTickCount();
		chassis_get_comm_data(&chassis_move);
		// // 设置底盘模式
		// chassis_set_mode(&chassis_move);
		// // 模式切换过渡 清除滤波器等
		// chassis_mode_change_control_transit(&chassis_move);
		// 更新反馈数据
		chassis_feedback_update(&chassis_move);
		// // 设置控制量
		// chassis_set_control(&chassis_move);
		// 底盘控制PID计算（含全向舵轮运动学分解 + 轮速PID）
		chassis_control_loop(&chassis_move);
		// M3508 轮速功率控制（原有逻辑保留）
		CHASSIC_MOTOR_POWER_CONTROL(&chassis_move);
		// GM6020 舵轮电流汇总 + 安全限幅
		RUDDER_POWER_CONTROL(&chassis_move);
		// 双通道打包
		comm_data_pack(&chassis_move);
		// 双通道发送
//		CAN_comm_up(chassis_move.comm_a_output);

		// 检查双通道通信时间，确保数据有效
		if (chassis_move.CHASSIS_xTickCount - REPLACE_COMM_A_TIME < 2000 && chassis_move.CHASSIS_xTickCount - REPLACE_COMM_B_TIME < 2000)
		{
			// 遥控器掉线时停止所有电机
			if (DBUS_error_flag)
			{
//				CAN_cmd_chassis(0, 0, 0, 0);
//				CAN_cmd_rudder(0, 0, 0, 0);
//				CAN_cmd_chassis(0x2000, 0x2000, 0x2000, 0x2000);
//				CAN_cmd_rudder(0x2000, 0x2000, 0x2000, 0x2000);
			}
			else
			{
				// 发送 M3508 轮速电流（CAN2, 0x200 → 0x201-0x204）
//				CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
//								chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
//				// 发送 GM6020 舵轮方向电流（CAN2, 0x1FF → 0x205-0x208）
//				CAN_cmd_rudder((int16_t)chassis_move.rudder_given_current[0],
//							   (int16_t)chassis_move.rudder_given_current[1],
//							   (int16_t)chassis_move.rudder_given_current[2],
//							   (int16_t)chassis_move.rudder_given_current[3]);
//							CAN_cmd_rudder(0x2000, 0x2000, 0x2000, 0x2000);

			}
		}
		else
		{
			CAN_cmd_rudder(0x2000, 0x2000, 0x2000, 0x2000);
			CAN_cmd_chassis(0x2000, 0x2000, 0x2000, 0x2000);
//			CAN_cmd_chassis(0, 0, 0, 0);
//			CAN_cmd_rudder(0, 0, 0, 0);
		}
	// 系统延时
	vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          初始化"chassis_move"结构体，初始化PID，获取遥控器指针，
 *                 初始化3508电机结构体，初始化云台电机结构体，初始化底盘电机角度结构体
 * @param[out]     chassis_move_init:"chassis_move"结构体指针
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    memset(chassis_move_init, 0, sizeof(chassis_move_t));

    // 初始化电机速度PID值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    // 初始化底盘角度PID值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    // 功率缓冲PID
    const static fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD}; // 功率环PID参数

    // 底盘模式为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    // get gyro sensor euler angle point
    // 获取惯性导航姿态角指针
    chassis_move_init->chassis_INS_point = get_INS_point();
    // 获取电容数据指针
    chassis_move_init->cap_data = get_cap_data_point();
    // 获取机器人状态指针
    chassis_move_init->robot_state = get_robot_status_point();
    chassis_move_init->power_heat_data = get_power_heat_data_point();
    chassis_move_init->shoot_data = get_shoot_data_point();

    uint8_t i;
    // 获取底盘电机测量值指针，初始化PID
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    // 初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    // 功率环PID
    PID_init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);

    // ============================================================
    // 全向舵轮初始化
    // ============================================================
    // PID 参数数组
    const static fp32 rudder_angle_pid[3] = {RUDDER_ANGLE_PID_KP, RUDDER_ANGLE_PID_KI, RUDDER_ANGLE_PID_KD};
    const static fp32 rudder_speed_pid[3] = {RUDDER_SPEED_PID_KP, RUDDER_SPEED_PID_KI, RUDDER_SPEED_PID_KD};

    // 绑定 GM6020 数据指针
    chassis_move_init->Forward_L.gimbal_motor_measure = get_rudder_motor_measure_point(0);
    chassis_move_init->Forward_R.gimbal_motor_measure = get_rudder_motor_measure_point(1);
    chassis_move_init->Back_L.gimbal_motor_measure    = get_rudder_motor_measure_point(2);
    chassis_move_init->Back_R.gimbal_motor_measure    = get_rudder_motor_measure_point(3);

    // 设置零点编码器值（标定值，需根据实际安装调整）
    chassis_move_init->Forward_L.ecd_zero_set = Forward_L_ecd;
    chassis_move_init->Forward_R.ecd_zero_set = Forward_R_ecd;
    chassis_move_init->Back_L.ecd_zero_set    = Back_L_ecd;
    chassis_move_init->Back_R.ecd_zero_set    = Back_R_ecd;

    // 初始化各舵轮级联 PID（外环角度 + 内环速度）
    PID_init(&chassis_move_init->Forward_L.angle_pid, PID_POSITION, rudder_angle_pid, RUDDER_ANGLE_PID_MAX_OUT, RUDDER_ANGLE_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Forward_L.speed_pid, PID_POSITION, rudder_speed_pid, RUDDER_SPEED_PID_MAX_OUT, RUDDER_SPEED_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Forward_R.angle_pid, PID_POSITION, rudder_angle_pid, RUDDER_ANGLE_PID_MAX_OUT, RUDDER_ANGLE_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Forward_R.speed_pid, PID_POSITION, rudder_speed_pid, RUDDER_SPEED_PID_MAX_OUT, RUDDER_SPEED_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Back_L.angle_pid, PID_POSITION, rudder_angle_pid, RUDDER_ANGLE_PID_MAX_OUT, RUDDER_ANGLE_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Back_L.speed_pid, PID_POSITION, rudder_speed_pid, RUDDER_SPEED_PID_MAX_OUT, RUDDER_SPEED_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Back_R.angle_pid, PID_POSITION, rudder_angle_pid, RUDDER_ANGLE_PID_MAX_OUT, RUDDER_ANGLE_PID_MAX_IOUT);
    PID_init(&chassis_move_init->Back_R.speed_pid, PID_POSITION, rudder_speed_pid, RUDDER_SPEED_PID_MAX_OUT, RUDDER_SPEED_PID_MAX_IOUT);

    // 初始化方向系数为正向
    chassis_move_init->Forward_L.Judge_Speed_Direction = 1.0f;
    chassis_move_init->Forward_R.Judge_Speed_Direction = 1.0f;
    chassis_move_init->Back_L.Judge_Speed_Direction    = 1.0f;
    chassis_move_init->Back_R.Judge_Speed_Direction    = 1.0f;

    // 限制最大最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//初始化 扭转相关
	chassis_move_init->twist_init_flag = 0;
	chassis_move_init->change_twist_flag = 0;

    // 更新一次反馈数据
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          解包通信数据
 * @param[out]     unpack_comm_data:"chassis_move"结构体指针
 * @retval         none
 */
extern int8_t R, QA, BPIN, AUTO_ATTACK, STUCK, VISION, FOLLOW, TURN_REMOTE_FIRE;
void chassis_get_comm_data(chassis_move_t *receive_comm_data)
{
    /* A通道 */
    receive_comm_data->vx_set = receive_comm_data->comm_rx_A.rx_vx_set;
    receive_comm_data->vy_set = receive_comm_data->comm_rx_A.rx_vy_set;

    /* B通道 */
    // vz_set
    receive_comm_data->wz_set = receive_comm_data->comm_rx_B.rx_vz_set;

	if (receive_comm_data->comm_rx_B.rx_flag_b & COMM_FLAG_TURN_FLAG)
		turn_flags = 1;
	else
		turn_flags = 0;
	if (receive_comm_data->comm_rx_B.rx_flag_b & COMM_FLAG_KEY_C)
		KEY_c = 1;
	else
		KEY_c = 0;
	if (receive_comm_data->comm_rx_B.rx_flag_b & COMM_FLAG_GIMBAL_MODE)
		TURN_REMOTE_FIRE = 1;
	else
		TURN_REMOTE_FIRE = 0;
	
    // 标志位

    // 底盘模式(低4位取出来,最多16种模式) 前4位为标志位
    receive_comm_data->rx_chassis_mode = receive_comm_data->comm_rx_B.rx_chassis_mode & 0x0F;
    if (receive_comm_data->rx_chassis_mode == 0)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_RAW;
    else if (receive_comm_data->rx_chassis_mode == 1)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    else if (receive_comm_data->rx_chassis_mode == 2)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    else if (receive_comm_data->rx_chassis_mode == 3)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    else if (receive_comm_data->rx_chassis_mode == 4)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_BPIN;
    else if (receive_comm_data->rx_chassis_mode == 5)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_TWIST;
	else if (receive_comm_data->rx_chassis_mode == 6)
        receive_comm_data->chassis_mode = CHASSIS_VECTOR_TURN_ROUND;
	else
		receive_comm_data->chassis_mode = CHASSIS_VECTOR_RAW;
        // 判断是否跟随模式
        if (receive_comm_data->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
            FOLLOW = 1;
        else
            FOLLOW = 0;
        //按键Z
        if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_Z)
            KEY_z = 1;
        else
            KEY_z = 0;
		//按键Q
        if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_Q)
            KEY_q = 1;
        else
            KEY_q = 0;
//		if (receive_comm_data->comm_rx_B.rx_chassis_mode & COMM_FLAG_TURN_FLAG)
//            turn_flags = 1;
//        else
//            turn_flags = 0;

    // 其他Flag
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_SHIFT)//按下SHIFT
        KEY_shift = 1;
    else
        KEY_shift = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_R)
        R = 1;
    else
        R = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_QA)
        QA = 1;
    else
        QA = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_BPIN)
        BPIN = 1;
    else
        BPIN = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_AUTO_ATTACK)
        AUTO_ATTACK = 1;
    else
        AUTO_ATTACK = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_STUCK)
        STUCK = 1;
    else
        STUCK = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_VISION)
        VISION = 1;
    else
        VISION = 0;
    if (receive_comm_data->comm_rx_B.rx_Flag & COMM_FLAG_DBUS_TOE)
        DBUS_error_flag = 1;
    else
        DBUS_error_flag = 0;

    /* C通道 */
	static int16_t rx_first_speed_tmp[10] = {0}; // 取值滤波,暂时没写
	static int16_t rx_back_speed_tmp[10] = {0}; 
	static int16_t j,k = 0;
	
    RX_PITCH = receive_comm_data->comm_rx_C.rx_PITCH;
    RX_first_speed = receive_comm_data->comm_rx_C.rx_first_speed;
    RX_back_speed = receive_comm_data->comm_rx_C.rx_back_speed;
	if (RX_first_speed > 3000 && RX_first_speed < 6000)
	{
		rx_first_speed_tmp[j] = RX_first_speed;
		j++;
		if (j >= 10)
		{
			j = 0;
		}
	}
	if (RX_back_speed > 3000 && RX_back_speed < 6000)
	{
		rx_back_speed_tmp[k] = RX_back_speed;
		k++;
		if (k >= 10)
		{
			k = 0;
		}
	}
}


/**
 * @brief          更新底盘反馈数据，读取电机速度、编码器角度，
 *                 计算机体速度、角度，以及IMU姿态
 * @param[out]     chassis_move_update:"chassis_move"结构体指针
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    // UI显示底盘角度
    anglesr = abs((int)(chassis_move.chassis_yaw_motor->relative_angle * 100));
    if (anglesr > 157 && anglesr < 314)
    {
        anglesr = 314 - anglesr;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        // 电机速度、加速度、PID微分项
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE; //
    }

    // 计算机体速度 vx, vy, wz (基于电机速度解算，坐标系为机体坐标系)
    // chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    // chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    // chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //    //计算机体角度，需要结合云台电机角度
    //    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    //    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    //    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
	chassis_move_update->chassis_yaw   = chassis_move_update->chassis_INS_point->Yaw;
	chassis_move_update->chassis_pitch = chassis_move_update->chassis_INS_point->Pitch;
    chassis_move_update->chassis_roll  = chassis_move_update->chassis_INS_point->Roll;

    // ============================================================
    // 更新全向舵轮电机数据
    // ============================================================
    chassis_move_update->Forward_L.motor_speed = chassis_move_update->Forward_L.gimbal_motor_measure->speed_rpm;
    chassis_move_update->Forward_R.motor_speed = chassis_move_update->Forward_R.gimbal_motor_measure->speed_rpm;
    chassis_move_update->Back_L.motor_speed    = chassis_move_update->Back_L.gimbal_motor_measure->speed_rpm;
    chassis_move_update->Back_R.motor_speed    = chassis_move_update->Back_R.gimbal_motor_measure->speed_rpm;

    // 舵轮线速度 (m/s)
    chassis_move_update->rudder_speed[0] = chassis_move_update->Forward_L.gimbal_motor_measure->speed_rpm * GM6020_RPM_TO_VECTOR;
    chassis_move_update->rudder_speed[1] = chassis_move_update->Forward_R.gimbal_motor_measure->speed_rpm * GM6020_RPM_TO_VECTOR;
    chassis_move_update->rudder_speed[2] = chassis_move_update->Back_L.gimbal_motor_measure->speed_rpm    * GM6020_RPM_TO_VECTOR;
    chassis_move_update->rudder_speed[3] = chassis_move_update->Back_R.gimbal_motor_measure->speed_rpm    * GM6020_RPM_TO_VECTOR;

    // 舵轮角速度 (rad/s)
    chassis_move_update->rudder_omega[0] = RpmToOmega(chassis_move_update->Forward_L.gimbal_motor_measure->speed_rpm);
    chassis_move_update->rudder_omega[1] = RpmToOmega(chassis_move_update->Forward_R.gimbal_motor_measure->speed_rpm);
    chassis_move_update->rudder_omega[2] = RpmToOmega(chassis_move_update->Back_L.gimbal_motor_measure->speed_rpm);
    chassis_move_update->rudder_omega[3] = RpmToOmega(chassis_move_update->Back_R.gimbal_motor_measure->speed_rpm);

    // 编码器增量缓存
    chassis_move_update->Encoder_add[0] = chassis_move_update->Forward_L.ecd_add;
    chassis_move_update->Encoder_add[1] = chassis_move_update->Forward_R.ecd_add;
    chassis_move_update->Encoder_add[2] = chassis_move_update->Back_L.ecd_add;
    chassis_move_update->Encoder_add[3] = chassis_move_update->Back_R.ecd_add;
}
/**
 * @brief          将遥控器通道值转换为底盘速度向量
 *
 * @param[out]     vx_set: x方向速度指针
 * @param[out]     vy_set: y方向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 结构体指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    // 遥控器死区处理，超出死区的值才有效
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    //    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN+vision_rx->vx;
    //    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN+vision_rx->vy;
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

    // 键盘控制
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    // 一阶低通滤波，使速度变化平滑
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    // 停止状态下，直接置零速度
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z)
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
    }
}

/**
 * @brief          全向舵轮运动学分解：将 vx/vy/wz 分解为各轮速度目标和舵轮目标角度
 *                 假设四轮均匀分布，方位角各为 45°（正方形底盘近似）。
 *                 结果写入 chassis_move.Forward_L/R, Back_L/R 的 wheel_speed/rudder_angle/ecd_add。
 * @param[in/out]  chassic_rudder_preliminary_solution: chassis_move 指针
 * @retval         none
 */
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t *chassic_rudder_preliminary_solution)
{
    fp32 vx_set, vy_set, vw_set = 0.0f;
    vx_set = chassic_rudder_preliminary_solution->vx_set;
    vy_set = chassic_rudder_preliminary_solution->vy_set;
    vw_set = chassic_rudder_preliminary_solution->wz_set;

    // 保存上一周期舵轮角度
    chassic_rudder_preliminary_solution->Forward_L.last_rudder_angle = chassic_rudder_preliminary_solution->Forward_L.rudder_angle;
    chassic_rudder_preliminary_solution->Forward_R.last_rudder_angle = chassic_rudder_preliminary_solution->Forward_R.rudder_angle;
    chassic_rudder_preliminary_solution->Back_L.last_rudder_angle    = chassic_rudder_preliminary_solution->Back_L.rudder_angle;
    chassic_rudder_preliminary_solution->Back_R.last_rudder_angle    = chassic_rudder_preliminary_solution->Back_R.rudder_angle;

    // 计算各轮合速度大小（Forward_R 取负保证正方向一致）
    chassic_rudder_preliminary_solution->Forward_L.wheel_speed =  sqrt(pow((vy_set + vw_set * arm_cos_f32(DEG2R(45))), 2) + pow((vx_set + vw_set * arm_sin_f32(DEG2R(45))), 2));
    chassic_rudder_preliminary_solution->Back_L.wheel_speed    =  sqrt(pow((vy_set - vw_set * arm_cos_f32(DEG2R(45))), 2) + pow((vx_set + vw_set * arm_sin_f32(DEG2R(45))), 2));
    chassic_rudder_preliminary_solution->Back_R.wheel_speed    =  sqrt(pow((vy_set - vw_set * arm_cos_f32(DEG2R(45))), 2) + pow((vx_set - vw_set * arm_sin_f32(DEG2R(45))), 2));
    chassic_rudder_preliminary_solution->Forward_R.wheel_speed = -sqrt(pow((vy_set + vw_set * arm_cos_f32(DEG2R(45))), 2) + pow((vx_set - vw_set * arm_sin_f32(DEG2R(45))), 2));

    // 计算各轮舵轮目标角度 (rad)，由速度方向决定
    chassic_rudder_preliminary_solution->Forward_L.rudder_angle = atan2((vy_set + vw_set * arm_cos_f32(DEG2R(45))), (vx_set + vw_set * arm_sin_f32(DEG2R(45))));
    chassic_rudder_preliminary_solution->Back_L.rudder_angle    = atan2((vy_set - vw_set * arm_cos_f32(DEG2R(45))), (vx_set + vw_set * arm_sin_f32(DEG2R(45))));
    chassic_rudder_preliminary_solution->Back_R.rudder_angle    = atan2((vy_set - vw_set * arm_cos_f32(DEG2R(45))), (vx_set - vw_set * arm_sin_f32(DEG2R(45))));
    chassic_rudder_preliminary_solution->Forward_R.rudder_angle = atan2((vy_set + vw_set * arm_cos_f32(DEG2R(45))), (vx_set - vw_set * arm_sin_f32(DEG2R(45))));

    // 将弧度角度转换为编码器增量
    chassic_rudder_preliminary_solution->Forward_L.ecd_add = (int16_t)(chassic_rudder_preliminary_solution->Forward_L.rudder_angle / Motor_Ecd_to_Rad);
    chassic_rudder_preliminary_solution->Forward_R.ecd_add = (int16_t)(chassic_rudder_preliminary_solution->Forward_R.rudder_angle / Motor_Ecd_to_Rad);
    chassic_rudder_preliminary_solution->Back_L.ecd_add    = (int16_t)(chassic_rudder_preliminary_solution->Back_L.rudder_angle    / Motor_Ecd_to_Rad);
    chassic_rudder_preliminary_solution->Back_R.ecd_add    = (int16_t)(chassic_rudder_preliminary_solution->Back_R.rudder_angle    / Motor_Ecd_to_Rad);

    // 保存增量（备用）
    chassic_rudder_preliminary_solution->Forward_L.last_ecd_add = chassic_rudder_preliminary_solution->Forward_L.ecd_add;
    chassic_rudder_preliminary_solution->Forward_R.last_ecd_add = chassic_rudder_preliminary_solution->Forward_R.ecd_add;
    chassic_rudder_preliminary_solution->Back_L.last_ecd_add    = chassic_rudder_preliminary_solution->Back_L.ecd_add;
    chassic_rudder_preliminary_solution->Back_R.last_ecd_add    = chassic_rudder_preliminary_solution->Back_R.ecd_add;
}

/**
 * @brief          舵轮角度控制主循环（驱动4个GM6020）
 * @param[in/out]  rudder_move_control_loop: chassis_move 指针
 * @retval         none
 */
static void rudder_control_loop(chassis_move_t *rudder_move_control_loop)
{
    Rudder_motor_relative_angle_control(&rudder_move_control_loop->Forward_L);
    Rudder_motor_relative_angle_control(&rudder_move_control_loop->Forward_R);
    Rudder_motor_relative_angle_control(&rudder_move_control_loop->Back_L);
    Rudder_motor_relative_angle_control(&rudder_move_control_loop->Back_R);
}

/**
 * @brief          单个舵轮相对角度计算与方向处理
 *                 - 计算最短路程的 ecd_error
 *                 - 超90°时反转轮速方向并减小角度
 *                 - 计算 cos3 衰减系数减小调向时轮速抖动
 *                 - 调用 RUDDER_MOTOR_PID_CONTROL 输出电流
 * @param[in/out]  chassis_motor: 单个舵轮结构体指针
 * @retval         none
 */
static void Rudder_motor_relative_angle_control(Rudder_Motor_t *chassis_motor)
{
    float angle;

    // 计算目标编码器绝对值（零点 + 运动学增量，处理越界）
    if (chassis_motor->ecd_add > 0)
    {
        if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add > 8191)
            chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add - 8191;
        else
            chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add;
    }
    else if (chassis_motor->ecd_add < 0)
    {
        if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add < 0)
            chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add + 8191;
        else
            chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add;
    }
    else if (fabs(chassis_move.vx_set) <= 0.05f && fabs(chassis_move.vy_set) <= 0.05f
             && chassis_move.chassis_mode != CHASSIS_VECTOR_BPIN)
    {
        // 停止时保持当前角度，防止低速抖动
        chassis_motor->ecd_set = chassis_motor->gimbal_motor_measure->ecd;
    }
    else
    {
        chassis_motor->ecd_set = chassis_motor->ecd_zero_set;
    }

    // 计算编码器误差
    chassis_motor->ecd_error = chassis_motor->ecd_set - chassis_motor->gimbal_motor_measure->ecd;

    // 处理跨越零点的最短路程
    if (chassis_motor->ecd_error > 4096)
        chassis_motor->ecd_error -= 8191;
    else if (chassis_motor->ecd_error < -4096)
        chassis_motor->ecd_error += 8191;

    // 超过 90° 时翻转轮速方向，并将角度折叠至 [-90°, 90°]
    if (chassis_motor->ecd_error > 2048)
    {
        chassis_motor->ecd_error -= 4096;
        chassis_motor->Judge_Speed_Direction = -1.0f;
    }
    else if (chassis_motor->ecd_error < -2048)
    {
        chassis_motor->ecd_error += 4096;
        chassis_motor->Judge_Speed_Direction = -1.0f;
    }
    else
    {
        chassis_motor->Judge_Speed_Direction = 1.0f;
    }

    // 计算 cos3 衰减系数，角差越大轮速越低（防止调向时冲力过大）
    angle = (float)(chassis_motor->ecd_error) * Motor_Ecd_to_Rad;
    if (fabs(angle) > 90.0f)
        angle = 90.0f; // 不会触发（最大误差约1.57 rad < 90），保留保护
    else if (fabs(angle) < 0.5f && chassis_move.chassis_mode != CHASSIS_VECTOR_BPIN)
        angle = 0.0f; // 死区：极小误差时直接置零，防止低速颤振
    chassis_motor->Judge_Speed_cosk = arm_cos_f32(angle) * arm_cos_f32(angle) * arm_cos_f32(angle);

    RUDDER_MOTOR_PID_CONTROL(chassis_motor);
}

/**
 * @brief          舵轮方向电机 PID 控制（级联：角度外环 + 速度内环）
 *                 使用 Hero_Chasiss 原有 pid_type_def / PID_calc，
 *                 功能等效于 Base_Board 的 Matlab_PID_Calc 级联 PID。
 * @param[in/out]  rudder_motor: 单个舵轮结构体指针
 * @retval         none
 */
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t *rudder_motor)
{
    // 外环：编码器误差 → 速度设定 (PID_calc: error = set - ref = ecd_error - 0)
    fp32 speed_set = PID_calc(&rudder_motor->angle_pid, 0.0f, (fp32)rudder_motor->ecd_error);
    // 内环：速度误差 → 电流输出
    rudder_motor->given_current = (int16_t)PID_calc(&rudder_motor->speed_pid, rudder_motor->motor_speed, speed_set);
}

/**
 * @brief          舵轮功率控制（简化版：直接电流限幅，无复杂功率预测）
 *                 GM6020 在此工程中无 power_forecast/Power_reso_GM6020，
 *                 仅做 [-16384, 16384] 安全限幅并汇总 rudder_given_current。
 * @param[in/out]  chassis_motor: chassis_move 指针
 * @retval         none
 */
void RUDDER_POWER_CONTROL(chassis_move_t *chassis_motor)
{
    // 汇总各舵轮输出电流
    chassis_motor->rudder_given_current[0] = chassis_motor->Forward_L.given_current;
    chassis_motor->rudder_given_current[1] = chassis_motor->Forward_R.given_current;
    chassis_motor->rudder_given_current[2] = chassis_motor->Back_L.given_current;
    chassis_motor->rudder_given_current[3] = chassis_motor->Back_R.given_current;

    // 安全限幅
    for (uint8_t i = 0; i < 4; i++)
    {
        if      (chassis_motor->rudder_given_current[i] >  16384.0f) chassis_motor->rudder_given_current[i] =  16384.0f;
        else if (chassis_motor->rudder_given_current[i] < -16384.0f) chassis_motor->rudder_given_current[i] = -16384.0f;
    }
}

/**
 * @brief          控制循环：全向舵轮运动学分解 + 方向电机角度控制 + 轮速 PID
 *                 替代原麦轮版本，保留功率控制、限幅等所有原始逻辑。
 * @param[out]     chassis_move_control_loop: "chassis_move" 结构体指针
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;

    // ① 全向舵轮运动学分解（计算各轮 wheel_speed / rudder_angle / ecd_add）
    chassic_rudder_preliminary_A_S_solution(chassis_move_control_loop);

    // ② 舵轮方向控制（GM6020 角度 PID → given_current）
    rudder_control_loop(chassis_move_control_loop);

    // RAW 模式：直接发送0，舵轮仍会运动到目标角度
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        for (i = 0; i < 4; i++)
            chassis_move_control_loop->motor_chassis[i].give_current = 0;
        return;
    }

    // ③ 根据舵轮角差衰减系数，设置 M3508 轮速目标
    //    Judge_Speed_Direction：超90°翻向；Judge_Speed_cosk：cos3衰减
    chassis_move_control_loop->motor_chassis[0].speed_set =
        chassis_move_control_loop->Forward_L.wheel_speed *
        chassis_move_control_loop->Forward_L.Judge_Speed_Direction *
        chassis_move_control_loop->Forward_L.Judge_Speed_cosk;

    chassis_move_control_loop->motor_chassis[1].speed_set =
        chassis_move_control_loop->Forward_R.wheel_speed *
        chassis_move_control_loop->Forward_R.Judge_Speed_Direction *
        chassis_move_control_loop->Forward_R.Judge_Speed_cosk;

    chassis_move_control_loop->motor_chassis[2].speed_set =
        chassis_move_control_loop->Back_L.wheel_speed *
        chassis_move_control_loop->Back_L.Judge_Speed_Direction *
        chassis_move_control_loop->Back_L.Judge_Speed_cosk;

    chassis_move_control_loop->motor_chassis[3].speed_set =
        chassis_move_control_loop->Back_R.wheel_speed *
        chassis_move_control_loop->Back_R.Judge_Speed_Direction *
        chassis_move_control_loop->Back_R.Judge_Speed_cosk;

    // ④ 轮速限幅（保持各轮速比例）
    for (i = 0; i < 4; i++)
    {
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
            max_vector = temp;
    }
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
    }

    // ⑤ M3508 轮速 PID
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i],
                 chassis_move_control_loop->motor_chassis[i].speed,
                 chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    // ⑥ 赋值电流
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current =
            (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}



void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_motor)
{
    static uint8_t can_send_tmp = 0;

    can_send_tmp++;

    uint16_t max_power_limit = 30;
    fp32 input_power = 0; // 输入功率(经过缓冲后的可用功率)
    fp32 scaled_motor_power[4];
    fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55  转矩系数
    fp32 k2 = 1.23e-07;                      // 电流系数
    fp32 k1 = 1.453e-07;                     // 电流系数

    fp32 constant = 4.081f;                                // 3508电机空转功率
    chassis_motor->power_control.POWER_MAX = 0;            // 可用的最大功率
    chassis_motor->power_control.forecast_total_power = 0; // 预测总功率

    // PID_Calc(&chassis_motor->buffer_pid, chassis_motor->power_heat_data->buffer_energy, 30); //使用缓冲能量维持一个范围内,这个PID没有要移植的地方，先保留一个PID
    PID_calc(&chassis_motor->buffer_pid, chassis_motor->power_heat_data->buffer_energy, 30);

    // 获取裁判系统限制功率
    if (chassis_motor->robot_state->chassis_power_limit > 120)
    {
        max_power_limit = 120;
	}
	else
    {
        max_power_limit = chassis_motor->robot_state->chassis_power_limit;
    }

    input_power = max_power_limit - chassis_motor->buffer_pid.out; // 通过缓冲PID调整输入功率

    chassis_motor->power_control.power_charge = input_power * 100; // 转换为电容充电功率

    if (chassis_motor->power_control.power_charge > 150 * 100)
        chassis_motor->power_control.power_charge = 150 * 100; // 限制最大充电功率，防止电容过充

    //	if(CAN_CMD_cap_send_t>2)
    //	{
    //		CAN_cmd_cap(chassis_motor->power_control.power_charge,1); // 发送电容充电功率
    //		CAN_CMD_cap_send_t = 0;
    //	}

    if (chassis_motor->cap_data->cap_volt > 6.0f)
    {
		if (KEY_shift == 1) // 按下Shift
		{
			if (chassis_motor->cap_data->cap_volt > 13)
			{
				chassis_motor->power_control.POWER_MAX = input_power + 25;
			}
			else
			{
				chassis_motor->power_control.POWER_MAX = input_power + 5;
			}
//				if (chassis_motor->robot_state->chassis_power_limit <= 40) // 如果功率较低
//				{
//					if (chassis_motor->cap_data->cap_volt > 12)
//					{
//						chassis_motor->power_control.POWER_MAX = 70;
//					}
//					else
//					{
//						chassis_motor->power_control.POWER_MAX = 60;
//					}
//				}
		}
		else // 未按下Shift
		{
			chassis_motor->power_control.POWER_MAX = input_power;
//				if (chassis_motor->robot_state->chassis_power_limit <= 40) // 功率较低时不补充
//				{
//					if (chassis_motor->cap_data->cap_volt > 18)
//						chassis_motor->power_control.POWER_MAX = 50;
//					else
//						chassis_motor->power_control.POWER_MAX = 45;
//				}
		}
    }
    else
    {
        chassis_motor->power_control.POWER_MAX = input_power - 30;
//		if (chassis_motor->robot_state->chassis_power_limit <= 40) // 功率较低
//		{
//			chassis_motor->power_control.POWER_MAX = chassis_motor->robot_state->chassis_power_limit - 5;
//		}
    }

    if (chassis_motor->cap_data->err != 0)
	{
        CAN_cmd_cap(chassis_motor->power_control.power_charge, 0);
		chassis_motor->power_control.POWER_MAX = input_power;
	}
    else
        CAN_cmd_cap(chassis_motor->power_control.power_charge, 1);

	
	
	// 功率模式
    for (uint8_t i = 0; i < 4; i++) // 遍历四个3508电机，预测功率
    {
        chassis_motor->power_control.forecast_motor_power[i] =
            chassis_motor->motor_chassis[i].give_current * toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm        // 转距功率
            + k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm // 摩擦功率
            + k2 * chassis_motor->motor_chassis[i].give_current * chassis_motor->motor_chassis[i].give_current + constant;

        if (chassis_motor->power_control.forecast_motor_power[i] < 0)
            continue; // 小于0忽略

        chassis_motor->power_control.forecast_total_power += chassis_motor->power_control.forecast_motor_power[i]; // 计算总功率 = 预测功率
    }
 
    if (chassis_motor->power_control.forecast_total_power > chassis_motor->power_control.POWER_MAX) // 如果超功率，进行功率限制
    {
        fp32 power_scale = chassis_motor->power_control.POWER_MAX / chassis_motor->power_control.forecast_total_power;
        for (uint8_t i = 0; i < 4; i++)
        {
            scaled_motor_power[i] = chassis_motor->power_control.forecast_motor_power[i] * power_scale; // 缩放后的功率

            if (scaled_motor_power[i] < 0)
                continue;

            fp32 b = toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm;
            fp32 c = k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant;

            if (chassis_motor->motor_chassis[i].give_current > 0) // 正转
            {
                chassis_motor->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);
                if (chassis_motor->power_control.MAX_current[i] > 16000)
                {
                    chassis_motor->motor_chassis[i].give_current = 16000;
                }
                else
                    chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
            }
            else
            {
                chassis_motor->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
                if (chassis_motor->power_control.MAX_current[i] < -16000)
                {
                    chassis_motor->motor_chassis[i].give_current = -16000;
                }
                else
                    chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
            }
        }
    }
}

/**
 * @brief          打包双通道发送帧
 * @param[out]     send_data_pack:"chassis_move"结构体指针
 * @retval         none
 */
void comm_data_pack(chassis_move_t *send_data_pack)
{
    /* A通道 */
    uint8_t send_flag;
    // tx_Flag
    if (send_data_pack->robot_state->power_management_gimbal_output == 1) // 云台输出
        send_flag |= (1 << 0);
    else
        send_flag &= ~(1 << 0);
    if (send_data_pack->robot_state->power_management_shooter_output == 1) // 发射器输出
        send_flag |= (1 << 1);
    else
        send_flag &= ~(1 << 1);
	if (send_data_pack->robot_state->robot_id > 100) // 机器人ID 0红 1蓝
        send_flag |= (1 << 2); // 蓝
    else
        send_flag &= ~(1 << 2); // 红
	

    // 填充待发送的结构体
    send_data_pack->comm_tx_a.tx_current_heat = send_data_pack->power_heat_data->shooter_42mm_barrel_heat;
    send_data_pack->comm_tx_a.tx_robo_level = send_data_pack->robot_state->robot_level;
    send_data_pack->comm_tx_a.tx_initial_speed_x100 = (uint16_t)(send_data_pack->shoot_data->initial_speed*100.0f);
    send_data_pack->comm_tx_a.tx_flag = send_flag;//1
    PACK_STRUCT_TO_CAN_BUFFER(send_data_pack->comm_tx_a, send_data_pack->comm_a_output);
}