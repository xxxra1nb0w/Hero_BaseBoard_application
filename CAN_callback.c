/**   ****************************(C) 版权所有 2024 none****************************
 * @file       CAN_Callback.c
 * @brief      CAN通信回调函数
 *
 * @details    实现功能：封装can通信回调函数
 *
 * @note       将回调函数从每个task文件移出并封装，防止多个task用一个can导致的回调函数冲突
 *
 * @history    版本        日期            作者           修改内容
 *             V1.0.0     2024-12-18      BaiShuhao      可利用这个做代码修改的记录
 *
 * @verbatim
 * ==============================================================================
 *  通信协议详情：
 *  - 设备地址: 0x01
 *  - 功能码: 0x03 
 *  - 数据格式: 可转换为ASCII编码的十六进制重量值 6个字节
 * ==============================================================================
 * @endverbatim
 ****************************(C) 版权所有 none ********************************
 */
 
 #include "CAN_callback.h"
 
    /**
  * @brief CAN1报文回调函数
  *
  * @param Rx_Buffer CAN接收的信息结构体
  */
 void CAN_Motor_Call_Back_CAN1(Struct_CAN_Rx_Buffer *Rx_Buffer)
 {
 	switch (Rx_Buffer->Header.StdId)
     {
 		case CAN_3508_LEFT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_left, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_3508_RIGHT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_right, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_3508_BLEFT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_bleft, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_3508_BRIGHT_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_bright, Rx_Buffer->Data);
 		break;
 	}
 	case CAN_TRIGGER_MOTOR_RID:
 	{
 		Motor_C620_CAN_RxCpltCallback(&shoot_control.Motor_trigger, Rx_Buffer->Data);
 		break;
 	}
 	}
 }
 
  /**
  * @brief CAN2报文回调函数
  *
  * @param Rx_Buffer CAN接收的信息结构体
  */
 void CAN_Motor_Call_Back_CAN2(Struct_CAN_Rx_Buffer *Rx_Buffer)
 {
 	    switch (Rx_Buffer->Header.StdId)
     {
 	case CAN_3508_M1_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[0].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	case CAN_3508_M2_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[1].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	case CAN_3508_M3_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[2].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	case CAN_3508_M4_RID:
 	{
         Motor_C620_CAN_RxCpltCallback(&chassis_move.Chassis_3508[3].Motor_C620, Rx_Buffer->Data);
         break;
     }
 	//
 		case 0X205: // yaw
 	{
         Motor_GM6020_CAN_RxCpltCallback(&gimbal_control.gimbal_yaw_motor.GM6020_measure, Rx_Buffer->Data);
         break;
    }
 	case 0X206: // PITCH
 	{
         Motor_C620_CAN_RxCpltCallback(&gimbal_control.gimbal_pitch_motor.C620_measure, Rx_Buffer->Data);
         break;
    }
    }
 }
