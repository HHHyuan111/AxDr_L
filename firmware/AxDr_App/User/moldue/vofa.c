#include "common.h"
#include "modlue.h"
#include "usbd_cdc_if.h"

#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:	void
* @retval:     void
* @details:    发送调试数据
***********************************************************************
**/
float adc_value[3];
extern uint16_t adc1_buff[2];
extern uint16_t adc2_buff[4];
_RAM_FUNC void vofa_start(void)
{
	
//	vofa_send_data(0, g_foc.ref.theta_e);
//	vofa_send_data(1, g_foc.fb.theta_e);
	vofa_send_data(1, g_foc.adc.raw.ia);
	vofa_send_data(2, g_foc.adc.raw.ib);
	vofa_send_data(3, g_foc.adc.raw.ic);
	
//	vofa_send_data(1, g_foc.fb.spd_r);
//	vofa_send_data(2, g_foc.fb.iq);
//	vofa_send_data(2, g_foc.fb.ib);
//	vofa_send_data(3, g_foc.fb.ic);

	// calibr
//	vofa_send_data(0, g_foc.calibr.pos);
//	vofa_send_data(1, g_foc.calibr.pr_set);
//	vofa_send_data(2, g_foc.calibr.pos_err);
//	vofa_send_data(3, g_foc.map.enc_lut);
//	vofa_send_data(3, g_foc.calibr.pn);
//	vofa_send_data(3, g_foc.calibr.e_off);
//	vofa_send_data(3, g_foc.calibr.r_off);
//	vofa_send_data(3, g_foc.motor.phase_order);
	
	// idpm
//	vofa_send_data(0, g_foc.idpm.Rs);
//	vofa_send_data(2, g_foc.idpm.Ld);
//	vofa_send_data(3, g_foc.idpm.Lq);
//	vofa_send_data(5, g_foc.idpm.flux);
//	vofa_send_data(6, g_foc.idpm.Js);
//	vofa_send_data(7, g_foc.fb.iq);
//	vofa_send_data(8, g_foc.fb.spd_r);
//	vofa_send_data(9,  g_foc.id_pi.kp);
//	vofa_send_data(10, g_foc.id_pi.ki);
//	vofa_send_data(11, g_foc.iq_pi.kp);
//	vofa_send_data(12, g_foc.iq_pi.ki);
//	vofa_send_data(13, g_foc.spd_pi.kp);
//	vofa_send_data(14, g_foc.spd_pi.ki);

//	vofa_send_data(0, g_foc.fb.id);
//	vofa_send_data(1, g_foc.ref.id);
//	vofa_send_data(2, g_foc.fb.iq);
//	vofa_send_data(3, g_foc.ref.iq);
//	vofa_send_data(4, g_foc.fb.spd_r);

	
//	vofa_send_data(4, g_foc.fb.spd_r);
//	vofa_send_data(4, g_foc.ref.spd_m);
//	vofa_send_data(4, g_foc.traj.spd_step);
//	vofa_send_data(4, g_foc.fb.pos_m_1t);
//	
//	vofa_send_data(4, g_foc.fb.pos_m);
//	vofa_send_data(4, g_foc.ref.pos_m);
//	vofa_send_data(4, g_foc.ref.pos_r);
//	
//	vofa_send_data(4, g_foc.fb.rev);
	
	vofa_sendframetail();
}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
//	HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 0xFFFF);
	CDC_Transmit_FS((uint8_t *)buf, len);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
_RAM_FUNC void vofa_send_data(uint8_t num, float data) 
{
    (void)num;

//	send_buf[cnt++] = byte0(data);
//	send_buf[cnt++] = byte1(data);
//	send_buf[cnt++] = byte2(data);
//	send_buf[cnt++] = byte3(data);
	
	data_u f;
	
	f.f_val = data;
	
	send_buf[cnt++] = f.u8_val[0];
	send_buf[cnt++] = f.u8_val[1];
	send_buf[cnt++] = f.u8_val[2];
	send_buf[cnt++] = f.u8_val[3];
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void vofa_sendframetail(void) 
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;
	
	/* 将数据和帧尾打包发送 */
	vofa_transmit((uint8_t *)send_buf, cnt);
	cnt = 0;// 每次发送完帧尾都需要清零
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL 
* @retval     void
* @details:   demo示例
***********************************************************************
**/
void vofa_demo(void) 
{
	static float scnt = 0.0f;

	scnt += 0.01f;

	if(scnt >= 360.0f)
		scnt = 0.0f;

	float v1 = scnt;
	float v2 = sin((double)scnt / 180 * 3.14159) * 180 + 180;
	float v3 = sin((double)(scnt + 120) / 180 * 3.14159) * 180 + 180;
	float v4 = sin((double)(scnt + 240) / 180 * 3.14159) * 180 + 180;

	// Call the function to store the data in the buffer
	vofa_send_data(0, v1);
	vofa_send_data(1, v2);
	vofa_send_data(2, v3);
	vofa_send_data(3, v4);

	// Call the function to send the frame tail
	vofa_sendframetail();
}









