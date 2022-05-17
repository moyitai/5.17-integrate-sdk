
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
//#include "twi_master.h"
#include "hx3605.h"
#include "hx3605_hrs_agc.h"
//#include "hx3605_hrs_alg.h"
//#include "SEGGER_RTT.h"

#ifdef HRS_ALG_LIB

extern const uint8_t  hx3605_hrs_agc_idac;
extern const uint8_t  green_led_max_init;
extern uint8_t low_vbat_flag;

//HRS_INFRARED_THRES
extern const int32_t  hrs_ir_unwear_thres;
extern const int32_t  hrs_ir_wear_thres;
extern uint8_t read_fifo_first_flg;

static uint8_t s_ppg_state = 0;
static uint8_t s_cal_state = 0;
//static int32_t s_buf[64] = {0};
static int32_t agc_buf[64] = {0};

static uint8_t cal_delay = CAL_DELAY_COUNT;
static HRS_CAL_SET_T  calReg;
//
static hx3605_hrs_wear_msg_code_t hrs_wear_status = MSG_HRS_NO_WEAR;
static hx3605_hrs_wear_msg_code_t hrs_wear_status_pre = MSG_HRS_NO_WEAR;

static uint8_t no_touch_cnt = 0;

void Init_hrs_PPG_Calibration_Routine(HRS_CAL_SET_T *calR,uint8_t led)
{
	printf("-----%s\n",__func__);
    calR->flag = CAL_FLG_LED_DR|CAL_FLG_LED_DAC|CAL_FLG_AMB_DAC|CAL_FLG_RF;

    calR->LEDDAC = 0;   /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    calR->AMBDAC = 0;   /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    calR->RF = 5;       /* 0-63 */
    calR->LED = HRS_CAL_INIT_LED;
    calR->state = hrsCalStart;
    calR->int_cnt = 0;
    calR->cur255_cnt =0;
    if(low_vbat_flag==1)
    {
       calR->led_max_cur = 63;
    }
    else
    {
       calR->led_max_cur = green_led_max_init;
    }
}

void Restart_hrs_PPG_Calibration_Routine(HRS_CAL_SET_T *calR)
{
	printf("-----%s\n",__func__);

    calR->flag = CAL_FLG_LED_DAC;
    calR->state = hrsCalLed;
    calR->int_cnt = 0;
}

hx3605_hrs_wear_msg_code_t hx3605_hrs_wear_mode_check(WORK_MODE_T mode,int32_t infrared_data)
{
	//printf("-----%s\n",__func__);

    if(infrared_data > hrs_ir_wear_thres)
    {
        if(no_touch_cnt < NO_TOUCH_CHECK_NUM)
        {
            no_touch_cnt++;
        }
        if(no_touch_cnt >= NO_TOUCH_CHECK_NUM)
        {
            hrs_wear_status = MSG_HRS_WEAR;
        }
    }
    else if(infrared_data < hrs_ir_unwear_thres)
    {
        if(no_touch_cnt>0)
        {
            no_touch_cnt--;
        }
        if(no_touch_cnt == 0)
        {
            hrs_wear_status = MSG_HRS_NO_WEAR;
        }
    }

   //printf("hrs wearstates: hrs_wear_status_pre=%d,hrs_wear_status=%d\r\n",\
   //         hrs_wear_status_pre,hrs_wear_status);
    if(mode == WEAR_MODE)
    {
    	printf("mode == WEAR_MODE");
        return hrs_wear_status;
    }
    if(hrs_wear_status_pre != hrs_wear_status)
    {
        hrs_wear_status_pre = hrs_wear_status;
        if(hrs_wear_status_pre == MSG_HRS_NO_WEAR)
        {
            hx3605_hrs_low_power();
        }
        else if(hrs_wear_status_pre == MSG_HRS_WEAR)
        {
            hx3605_alg_open_deep();
            //hx3605_hrs_set_mode(PPG_INIT);
            //hx3605_hrs_set_mode(CAL_INIT);
        }
    }
    return hrs_wear_status;
}
void PPG_hrs_Calibration_Routine(HRS_CAL_SET_T *calR, int32_t led, int32_t amb)
{
//printf("-----%s=calR->state = %d\n",__func__,calR->state);
    switch(calR->state)
    {
				case hrsCalStart:
					  calR->state = hrsCalStart1;
            break;

        case hrsCalStart1:
			if(amb>300000)
			{
				calR->AMBDAC = (amb-300000)/10000;
			}
			else
			{
				calR->AMBDAC = 0;
			}
            if(led>amb+128)
            {
              calR->led_step = (led-amb)/HRS_CAL_INIT_LED;
              calR->LED = 10000*(hx3605_hrs_agc_idac+calR->AMBDAC)/calR->led_step;
              if(calR->LED>calR->led_max_cur)
              {
                calR->LED = calR->led_max_cur;
              }
              calR->LEDDAC = 0;
              calR->RF = 32;
              calR->flag = CAL_FLG_LED_DR|CAL_FLG_LED_DAC|CAL_FLG_AMB_DAC|CAL_FLG_RF;
              calR->state = hrsCalLed;
            }
            else
            {
              calR->state = hrsCalFinish;
            }
            break;

        case hrsCalLed:
            if(led>320000 && calR->LEDDAC<=120)
            {
              calR->LEDDAC = calR->LEDDAC + 1;
              calR->state = hrsCalLed;
            }
            else if(led<200000 && calR->LEDDAC>=1)
            {
              calR->LEDDAC = calR->LEDDAC - 1;
              calR->state = hrsCalLed;
            }
            else
            {
              //calR->state = hrsCalLed2;
							calR->state = hrsCalFinish;
            }
            calR->flag = CAL_FLG_LED_DAC;
            break;

//       case hrsCalLed2:
//            if(led+calR->LEDDAC*130000 < (amb+calR->AMBDAC*130000)+((130000*hx3605_hrs_agc_idac*3)>>2) && calR->LED < calR->led_max_cur)
//            {
//              calR->LED = calR->led_max_cur;
//              calR->state = hrsCalLed;
//              calR->flag = CAL_FLG_LED_DR;
//            }
//            else
//            {
//              calR->state = hrsCalFinish;
//              calR->flag = 0;
//            }
//            break;

        case hrsCalRfEnd:
            calR->state = hrsCalFinish;
            calR->flag = 0;
            break;

        default:
            break;

    }
    //AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",\
            calR->LED, calR->LEDDAC, calR->AMBDAC,calR->led_step,calR->RF);
}

HRS_CAL_SET_T PPG_hrs_agc(void)
{
	//printf("-----%s--%d--\n",__func__,s_cal_state);


    int32_t led_val, amb_val;

    //AGC_LOG("agc  in\r\n");
    calReg.work = false;
    if (!s_cal_state)
    {
      //printf("agc  out\r\n");
        return  calReg;
    }
    printf("agc  in\r\n");
		#ifdef INT_MODE
    calReg.int_cnt ++;
    if(calReg.int_cnt < 8)
    {
         return calReg;
    }
    calReg.int_cnt = 0;
		hx3605_gpioint_cfg(false);
		#endif
    calReg.work = true;

    read_data_packet(agc_buf);
    led_val = agc_buf[0];
    amb_val = agc_buf[1];

    //printf("cal dat ch1=%d,ch2=%d,led_val=%d,amb_val=%d \r\n",
    //agc_buf[0], agc_buf[1], led_val, amb_val);

    PPG_hrs_Calibration_Routine(&calReg, led_val, amb_val);

    if (calReg.state == hrsCalFinish)
		{
        hx3605_hrs_set_mode(CAL_OFF);
				#if defined(TIMMER_MODE)
				#else
				hx3605_gpioint_cfg(true);
				#endif
    }
		else
		{
        hx3605_hrs_updata_reg();
				#if defined(INT_MODE)
				hx3605_gpioint_cfg(true);
				#endif
    }
    return  calReg;
}

void hx3605_hrs_cal_init(void) // 20200615 ericy afe cali online
{
	printf("-----%s\n",__func__);
	uint16_t sample_rate = 200;                      /*config the data rate of chip alps2_fm ,uint is Hz*/
	uint32_t prf_clk_num = 32000/sample_rate;        /*period in clk num, num = Fclk/fs */
	uint8_t ps1_cp_avg_num_sel= 0;
	uint8_t ps0_cp_avg_num_sel= 0;
	uint8_t thres_int =0;    //thres int enable
	uint8_t data_rdy_int =8;    //[3]:ps1_data2 [2]:ps1_data1 [1]:ps0_data2 [0]:ps0_data1

	hx3605_write_reg(0X11, (uint8_t)prf_clk_num);    // prf bit<7:0>6   default 0x00
	hx3605_write_reg(0X12, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>  default 0x03
	hx3605_write_reg(0X16, (ps1_cp_avg_num_sel<<4)|(ps0_cp_avg_num_sel));  //default 0x00

	hx3605_write_reg(0x2d,0x00);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0X26, (thres_int<<4|data_rdy_int));   //default 0x0f

#if defined(INT_MODE)
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf ,04=enable almost full
#endif

	hx3605_write_reg(0X68, 0X01);  //soft reset
	hx3605_delay(5);
	hx3605_write_reg(0X68, 0X00);  //release
	hx3605_delay(5);             //Delay for reset time
}

void hx3605_hrs_cal_off(uint8_t enable_50_hz) // 20200615 ericy afe cali offline
{
	printf("-----%s\n",__func__);
	uint16_t sample_rate = 25;                      /*config the data rate of chip alps2_fm ,uint is Hz*/
	uint32_t prf_clk_num = 32000/sample_rate;        /*period in clk num, num = Fclk/fs */
	uint8_t ps1_cp_avg_num_sel= 0;
	uint8_t ps0_cp_avg_num_sel= 1;
	uint8_t thres_int =0;    //thres int enable
	uint8_t data_rdy_int =0;    //[3]:ps1_data2 [2]:ps1_data1 [1]:ps0_data2 [0]:ps0_data1
	uint8_t databuf[3] = {0};

	hx3605_write_reg(0X11, (uint8_t)prf_clk_num);    // prf bit<7:0>6   default 0x00
	hx3605_write_reg(0X12, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>  default 0x03
	hx3605_write_reg(0X16, (ps1_cp_avg_num_sel<<4)|(ps0_cp_avg_num_sel));  //default 0x00

	hx3605_write_reg(0x2d,0x83);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0X26, (thres_int<<4|data_rdy_int));   //default 0x0f

#if defined(INT_MODE)
	hx3605_write_reg(0x27,0x20);
#endif

	hx3605_write_reg(0X68, 0X01);  //soft reset
	hx3605_delay(5);
	hx3605_write_reg(0X68, 0X00);  //release
	hx3605_delay(5);
	read_fifo_first_flg = 1;
}

void read_hrs_data_packet(int32_t *buf)
{
	printf("-----%s\n",__func__);

	uint8_t dataBuf[6];

	hx3605_brust_read_reg(0x03, dataBuf, 3);
	hx3605_brust_read_reg(0x0c, dataBuf+3, 3);

	for (uint8_t i=0; i<2; i++)
	{
		buf[i] = (int32_t)(dataBuf[3*i]|(dataBuf[3*i+1]<<8)|(dataBuf[3*i+2]<<16));
	}
}

void read_hrs_ir_packet(int32_t *buf) // 20200615 ericy read reg_data phase1 and phase3
{
	printf("-----%s\n",__func__);

	uint8_t dataBuf[6];

	hx3605_brust_read_reg(0x06, dataBuf, 6);     //phase3(ir) and phase4(als for ir)

	for (uint8_t i=0; i<2; i++)
	{
	    buf[i] = (int32_t)(dataBuf[3*i]|(dataBuf[3*i+1]<<8)|(dataBuf[3*i+2]<<16));
	}
}

uint8_t hx3605_hrs_read_data_packet(int32_t *s_buf) // 20200615 ericy read reg_data phase1 and phase3
{
	printf("%s",__func__);
	uint8_t  databuf1[6] = {0};
	uint8_t  databuf2[6] = {0};
	uint32_t P1 = 0,P2 = 0 ,P3 = 0 ,P4 =0 ;
	if (!s_ppg_state || s_cal_state)
	{
		return NULL;
	}
	hx3605_brust_read_reg(0x03, databuf1, 6);
	hx3605_brust_read_reg(0x09, databuf2, 6);

	P1 = ((databuf1[0])|(databuf1[1]<<8)|(databuf1[2]<<16));
	P3 = ((databuf1[3])|(databuf1[4]<<8)|(databuf1[5]<<16));
	P4 = ((databuf2[0])|(databuf2[1]<<8)|(databuf2[2]<<16));
	P2 = ((databuf2[3])|(databuf2[4]<<8)|(databuf2[5]<<16));

	s_buf[0] = P1;
	s_buf[1] = P2;
	s_buf[2] = P3;
	s_buf[3] = P4;
	s_buf[4] = calReg.LED;

	uint8_t recal = 0;
	if (s_buf[0]<500000 || s_buf[0]>1500000)
	{
		recal = true;
	if(hrs_wear_status==MSG_HRS_NO_WEAR)
	{
	    recal = false;
	}
	}
	uint32_t ir_data = 0;
	if(s_buf[2]>s_buf[3])
	{
		ir_data = s_buf[2]-s_buf[3];
	}
	else
	{
		ir_data = 0;
	}
	hx3605_hrs_wear_mode_check(HRS_MODE,ir_data);
	if (recal)
	{
	cal_delay--;
	if (cal_delay <= 0)
	{
	    cal_delay = CAL_DELAY_COUNT;
	    hx3605_hrs_set_mode(RECAL_INIT);
	}
	}
	else
	{
		cal_delay = CAL_DELAY_COUNT;
	}
	return 1;
}

void hx3605_hrs_low_power(void)
{
	printf("-----%s\n",__func__);
	uint16_t sample_rate = 10;                      /*config the data rate of chip alps2_fm ,uint is Hz*/
	uint32_t prf_clk_num = 32000/sample_rate;        /*period in clk num, num = Fclk/fs */
	uint8_t ps1_cp_avg_num_sel= 0;
	uint8_t ps0_cp_avg_num_sel= 0;
	uint8_t thres_int =0;    //thres int enable
	uint8_t data_rdy_int =0;    //[3]:ps1_data2 [2]:ps1_data1 [1]:ps0_data2 [0]:ps0_data1
	uint8_t led_on_time  = 5;
	uint8_t ldrsel_ps1 =2;      //ps1 LDR SELECT 01:ldr0-IR 02:ldr1-RLED  04:ldr2-GLED
	uint8_t ldrsel_ps0 =0;      //ps0 LDR SELECT 01:ldr0-IR 02:ldr1-RLED  04:ldr2-GLED

	uint8_t dccancel_ps0_data1 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps0_data2 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps1_data1 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps1_data2 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t ir_pden_ps0 =0;     //IR_PDEN_PS0
	uint8_t ext_pden_ps0 =0;    //EXT_PDEN_PS0
	uint8_t pddrive_ps0 =0;     //PDDRIVE_PS0  0-63

	hx3605_write_reg(0X11, (uint8_t)prf_clk_num);    // prf bit<7:0>6   default 0x00
	hx3605_write_reg(0X12, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>  default 0x03
	hx3605_write_reg(0X16, (ps1_cp_avg_num_sel<<4)|(ps0_cp_avg_num_sel));  //default 0x00
	hx3605_write_reg(0X20, (ir_pden_ps0<<7|ir_pden_ps0<<6|pddrive_ps0));  //default 0x00
	hx3605_write_reg(0X18, (uint8_t)dccancel_ps0_data1);     //default 0x00
	hx3605_write_reg(0X19, (uint8_t)(dccancel_ps0_data1>>8));   //default 0x00
	hx3605_write_reg(0X1a, (uint8_t)dccancel_ps0_data2);   //default 0x00
	hx3605_write_reg(0X1b, (uint8_t)(dccancel_ps0_data2>>8));  //default 0x00
	hx3605_write_reg(0X1c, (uint8_t)dccancel_ps1_data1);   //default 0x00
	hx3605_write_reg(0X1d, (uint8_t)(dccancel_ps1_data1>>8));   //default 0x00
	hx3605_write_reg(0X1e, (uint8_t)dccancel_ps1_data2 );   //default 0x00
	hx3605_write_reg(0X1f, (uint8_t)(dccancel_ps1_data2 >>8));   //default 0x00
	hx3605_write_reg(0X22, (ldrsel_ps1<<4|ldrsel_ps0));      //default 0x00
	hx3605_write_reg(0X15, led_on_time);

#ifdef INT_MODE
	hx3605_write_reg(0X12,0x14);   // fifo almostfull cfg ,max=0x40;
	hx3605_write_reg(0x13,0x31); //FIFO bypass mode enable
	hx3605_write_reg(0x23,0x00); //phase3 convertion ready disable
#else
	hx3605_write_reg(0x2d,0x83);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0X26, (thres_int<<4|data_rdy_int));   //default 0x0f
#endif

	hx3605_write_reg(0X68, 0X01);  //soft reset
	hx3605_delay(5);
	hx3605_write_reg(0X68, 0X00);  //release
	hx3605_delay(5);

	calReg.LED = 0;

	AGC_LOG(" chip go to low power mode  \r\n" );

//yorke close 40m timer
}

void hx3605_hrs_updata_reg(void)
{
	printf("-----%s\n",__func__);

    if (calReg.flag & CAL_FLG_LED_DR)
    {
				hx3605_write_reg(0X20, calReg.LED);
    }

    if (calReg.flag & CAL_FLG_LED_DAC)
    {
			  hx3605_write_reg(0X18, (uint8_t)calReg.LEDDAC);
				hx3605_write_reg(0X19, (uint8_t)(calReg.LEDDAC>>8));
    }

    if (calReg.flag & CAL_FLG_AMB_DAC)
    {
				hx3605_write_reg(0X1a, (uint8_t)calReg.AMBDAC);
				hx3605_write_reg(0X1b, (uint8_t)(calReg.AMBDAC>>8));
    }

    if (calReg.flag & CAL_FLG_RF)
    {
			hx3605_write_reg(0X15, calReg.RF);
    }
}

void hx3605_hrs_set_mode(uint8_t mode_cmd)
{
	printf("-----%s==%d\n",__func__,mode_cmd);

    switch (mode_cmd)
    {
        case PPG_INIT:
            hx3605_hrs_ppg_init();
						#if defined(TIMMER_MODE)
							hx3605_320ms_timer_cfg(true);
							#if defined(GSEN_40MS_TIMMER)
							hx3605_40ms_timer_cfg(true);
							#endif
						#else
							hx3605_gpioint_cfg(true);
						#endif
            s_ppg_state = 1;
            printf("ppg init mode s_ppg_state = %d\r\n",s_ppg_state);
            break;

        case PPG_OFF:
            hx3605_ppg_off();
            s_ppg_state = 0;
            printf("ppg off mode\r\n");
            break;

        case PPG_LED_OFF:
            hx3605_hrs_low_power();
            s_ppg_state = 0;
            printf("ppg led off mode\r\n");
            break;

        case CAL_INIT:
            Init_hrs_PPG_Calibration_Routine(&calReg, 64);
            hx3605_hrs_cal_init();
            hx3605_hrs_updata_reg();
            #if defined(TIMMER_MODE)
							#if defined(GSEN_40MS_TIMMER)
							#else
							hx3605_40ms_timer_cfg(true);
							#endif
            #endif
            s_cal_state = 1;
            printf("cal init mode\r\n");
            break;

        case RECAL_INIT:
            Restart_hrs_PPG_Calibration_Routine(&calReg);
            hx3605_hrs_cal_init();
            hx3605_hrs_updata_reg();
            #if defined(TIMMER_MODE)
							#if defined(GSEN_40MS_TIMMER)
							#else
							hx3605_40ms_timer_cfg(true);
							#endif
            #endif
            s_cal_state = 1;
            printf("Recal init mode\r\n");
            break;

        case CAL_OFF:
            #if defined(TIMMER_MODE)
							#if defined(GSEN_40MS_TIMMER)
							#else
							hx3605_40ms_timer_cfg(false);
							#endif
            #endif
            hx3605_hrs_cal_off(0);
            s_cal_state = 0;
            printf("cal off mode\r\n");
            break;

        default:
            break;
    }
}

SENSOR_ERROR_T hx3605_hrs_enable(void)
{

	printf("hx3605_hrs_enable\n");
    if (!hx3605_chip_check())
    {
        printf("hx3690l check id failed!\r\n");
        return SENSOR_OP_FAILED;
    }

    printf("hx3690l check id success!\r\n");

    if (s_ppg_state)
    {
        printf("ppg already on!\r\n");
        return SENSOR_OP_FAILED;
    }
    if(!hx3605_alg_open())
    {
        printf("hrs alg open fail,or dynamic ram not enough!\r\n");
    }

    hrs_wear_status = MSG_HRS_NO_WEAR;
    hrs_wear_status_pre = MSG_HRS_NO_WEAR;
	work_mode_flag =HRS_MODE;
    hx3605_hrs_set_mode(PPG_INIT);

    printf("hx3690l enable!\r\n");

    return SENSOR_OK;
}

void hx3605_hrs_disable(void)
{
		#if defined(TIMMER_MODE)
    hx3605_320ms_timer_cfg(false);
    hx3605_40ms_timer_cfg(false);
		#elif defined(INT_MODE)
    hx3605_gpioint_cfg(false);
		#endif
		hx3605_hrs_set_mode(PPG_OFF);
		s_ppg_state = 0;
		s_cal_state = 0;
    hx3605_alg_close();
    AGC_LOG("hx3690l disable!\r\n");
}

void hx3605_hrs_data_reset(void)
{

		s_ppg_state = 0;
		s_cal_state = 0;
		hx3605_alg_close();
}

hx3605_hrs_wear_msg_code_t hx3605_hrs_get_wear_status(void)
{
    return  hrs_wear_status;
}

HRS_CAL_SET_T get_hrs_agc_status(void)
{
    HRS_CAL_SET_T cal;

    cal.flag =  calReg.flag;
    cal.int_cnt =  calReg.int_cnt;
    cal.LED=  calReg.LED;     // phasex led driver config
    cal.LEDDAC=  calReg.LEDDAC;  // phasex led offset idac cfg
    cal.AMBDAC=  calReg.AMBDAC;  // phasex offset idac cfg
    cal.RF=  calReg.RF;      // phasex tia feed back resister cfg
    cal.led_step=  calReg.led_step;
    cal.state=  calReg.state;

    return cal;
}

void hx3605_hrs_read_fifo_data(uint8_t read_fifo_size,int32_t *buf)
{
    uint8_t data_flg = 127;
    int32_t data;
    uint8_t databuf[3];
    uint8_t ii=0;
     //uint8_t jj=0;
    for(ii=0; ii<read_fifo_size; ii++)
    {
        hx3605_write_reg(0x17, 0x00); // write any walue to 0x17 will update a new data
        hx3605_delay_us(100);
        hx3605_brust_read_reg(0x15, databuf, 3);
        data_flg = databuf[2]>>5;
        data = (int32_t)(databuf[0]|(databuf[1]<<8)|((databuf[2]&0x1f)<<16));

        if(ii==0)
				{
						if(data_flg ==3)
						{
							ii=3;
							buf[0] = 0;
							buf[1] = 0;
							buf[2] = 0;
						}
						if(data_flg ==2)
						{
							ii=2;
							buf[0] = 0;
							buf[1] = 0;
						}
						if(data_flg ==1)
						{
							ii=1;
							buf[0] = 0;
						}
				}

        if(data_flg == 0)
        {
            buf[ii]= data;
        }
        else if(data_flg == 1)
        {
            buf[ii]= data;
        }
        else if(data_flg == 2)
        {
            buf[ii]= data;
        }
        else if(data_flg == 3)
        {
            buf[ii]= data;
        }
    }
}

uint8_t hx3605_hrs_read(hrs_sensor_data_t * s_dat)
{
	//printf("-----%s\n",__func__);

    int32_t PPG_src_data;
    int32_t Ir_src_data;
    bool recal = false;
    uint8_t size = 0;
    uint8_t size_byte = 0;
    int32_t *PPG_buf =  &(s_dat->ppg_data[0]);
    int32_t *ir_buf =  &(s_dat->ir_data[0]);
    int32_t *s_buf =  &(s_dat->s_buf[0]);
    s_dat->agc_green =  calReg.LED;
		int32_t ps_data[4]={0};
		int32_t ps_green_data = 0;
		uint8_t data_count = 0;

    if (!s_ppg_state || s_cal_state)
    {
    printf("=========retuen NULL \n");
        return NULL;
    }
	//printf("hx3605_read_fifo_data============\n");
        data_count = hx3605_read_fifo_data(s_buf,2,1);
//		printf("ppg data size: %d %d\r\n", data_count,hrs_phase_num);
		//printf("af_hx3605_read_fifo_data:::%d %d %d\r\n", hx3605_read_reg(0X2a),hx3605_read_reg(0X2b),hx3605_read_reg(0X2c));
        s_dat->count =  data_count;
        for (uint8_t i=0; i<data_count; i++)
        {
            PPG_src_data = s_buf[i*2];
            Ir_src_data = s_buf[i*2+1];

            PPG_buf[i] = 2*PPG_src_data;
            ir_buf[i] = Ir_src_data;
			//printf("ppg data size: %d\r\n", Ir_src_data);
            hx3605_hrs_wear_mode_check(HRS_MODE,Ir_src_data);
			//printf("hx3605_hrs_wear_mode_check:::%d %d %d\r\n", hx3605_read_reg(0X2a),hx3605_read_reg(0X2b),hx3605_read_reg(0X2c));
			//printf("=1+i==data_count==PPG_buf[i]=====ir_buf[i]================%d/%d %d %d\r\n" ,1+i,data_count,PPG_buf[i],ir_buf[i]);
        }
			read_data_packet(ps_data);
			ps_green_data = ps_data[0];
			if (ps_green_data<150000 || ps_green_data>400000)
			{
					recal = true;
			//		printf("recal=============\n");
					if(hrs_wear_status==MSG_HRS_NO_WEAR)
					{
						recal = false;
					}
			}
			if (recal)
			//if (0)
			{
			cal_delay--;

			if (cal_delay <= 0)
			{
			cal_delay = CAL_DELAY_COUNT;
			hx3605_hrs_set_mode(RECAL_INIT);
			//printf("========>>>>LED_IDAC = %d\r\n",calReg.led_idac);
			}
			}
			else
			{
			cal_delay = CAL_DELAY_COUNT;
			}
			//    }
    return 1;
}
#endif




