#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
//#include "twi_master.h"
#include "hx3605.h"
#include "hx3605_check_touch.h"
//#include "SEGGER_RTT.h"

hx3605_hrs_wear_msg_code_t hx3605_wear_status = MSG_HRS_NO_WEAR;
hx3605_hrs_wear_msg_code_t hx3605_wear_status_pre = MSG_HRS_NO_WEAR;
extern const int32_t check_mode_unwear_thre;
extern const int32_t check_mode_wear_thre;
uint8_t notouch_cnt = 0;
uint8_t touch_cnt = 0;

#ifdef CHECK_TOUCH_LIB
void hx3605_check_touch_init(void) //20200615 ericy ppg fs=25hz, phase3 conversion ready interupt en
{
    uint16_t sample_rate = 5; /*config the data rate of chip alps2_fm ,uint is Hz*/

    uint32_t prf_clk_num = 32000/sample_rate;   /*period in clk num, num = Fclk/fs */

    uint8_t cic_mode_en =0;
    uint8_t cic_b2_en = 0;
    uint8_t samp_delay_leden_num = 0; /* 00=8,01=16,10=32,11=64*/
    uint8_t samp_copy_avg = 0;        /* 0=1, 1=2, 2=4 ,3=8, 4=16*/
    uint8_t data_avg_num = 0;         /* 0 = 1 ; 1 = 2; 2 =4 ; 3 =8 ; 4 =16 ;*/
    uint8_t phase3_4_internal = 0;    /* phase3 and phase4 prf internal cfg */

    uint8_t phase1_enable = 1;     /*phase1_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase2_enable = 1;     /*phase2_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase3_enable = 1;     /*phase3_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase4_enable = 1;     /*phase4_enable  , 1 mean enable ; 0 mean disable */

    uint8_t phase1_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase2_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase3_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase4_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    //green
    uint8_t phase1_inner_avg = 0;   /* phase1 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase1_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase1_ldr_sel = IR_LED_SLE;     /*ball led 1 = ldr1(red); 2 = ldr2(ir); 4 = ldr3(green); 8 = ldr4 ;
                                    * 3in1 led 1 = ldr1(red); 2 = ldr2(green); 4 = ldr3(ir); 8 = ldr4 ;
                                    * 205U led 1 = ldr1(green); 2 = ldr2(red); 4 = ldr3(ir); 8 = ldr4 ;
                                    * GT01 led 1 = ldr1(green); 2 = ldr2(IR); 4 = ldr3(red); 8 = ldr4 ;
                                    */
    uint8_t phase1_pd_sel = 1;      /* 1 = pd1; 2 = pd2; */
    uint8_t phase1_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase1_ldr_cur = 64;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase1_led_en = 1;      /* phase1 led enable*/
    //als(green)
    uint8_t phase2_inner_avg = 0;   /* phase2 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase2_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase2_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase2_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase2_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase2_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase2_led_en = 0;      /* phase2 led enable*/
    //ir
    uint8_t phase3_inner_avg = 0;   /* phase3 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase3_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase3_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase3_pd_sel = 0;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase3_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase3_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase3_led_en = 0;      /* phase3 led enable*/
    //als(ir)
    uint8_t phase4_inner_avg = 0;   /* phase4 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase4_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase4_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase4_pd_sel = 0;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase4_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase4_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase4_led_en = 0;      /* phase4 led enable*/

    uint8_t init_wait_delay = 5 ; /* 0 = 31clk ; 1 = 64clk ; 2 = 127clk ; 3 = 255clk(d) ;
                                     4 = 511clk; 5 = 1023clk; 6 = 2047; 7 = 2048clk */

    uint8_t afe_reset = 3;        /* 0 = 15clk ; 1 = 31clk ; 2 = 63clk ; 3 = 127clk(d) ;
                                     4 = 255clk; 5 = 511clk; 6 = 1024; 7 = 2048clk */

    uint8_t led_on_time = 2;      /* 0 = 32clk=8us ; 1 = 64clk=16us; 2=128clk=32us ; 3 = 256clk=64us ;
                                     4 = 512clk=128us ; 5 = 1024clk=256us; 6= 2048clk=512us; 7 = 4096clk=1024us */
    hx3605_write_reg(0x02, 0x30);
    hx3605_delay(5);
    hx3605_write_reg(0X6a, 0X00);	//rest int
    hx3605_delay(5);

    hx3605_write_reg(0X1a, (uint8_t)prf_clk_num);    // prf bit<7:0>
    hx3605_write_reg(0X1b, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>
    hx3605_write_reg(0X1c, (uint8_t)(prf_clk_num>>16)); // prf bit<23:16>

    hx3605_write_reg(0X1d, phase3_4_internal);
    hx3605_write_reg(0X1e, ((afe_reset<<3)| 0x00) );
    hx3605_write_reg(0X1f, (led_on_time<<4| phase1_led_en<<3 | phase3_led_en<<2 | phase4_led_en<<1 | phase2_led_en) );

    hx3605_write_reg(0X26, (init_wait_delay<<4 | 0x0f));
    hx3605_write_reg(0X27, (phase1_inner_avg | (phase2_inner_avg<<4)));
    hx3605_write_reg(0X28, (phase3_inner_avg | (phase4_inner_avg<<4)));
    hx3605_write_reg(0X29, cic_mode_en<<7 | cic_b2_en<<6 | samp_delay_leden_num<<4 | samp_copy_avg);

    hx3605_write_reg(0X2c, phase1_tia_res);
    hx3605_write_reg(0X2d, phase3_tia_res);
    hx3605_write_reg(0X2e, phase4_tia_res);
    hx3605_write_reg(0X2f, phase2_tia_res);

    hx3605_write_reg(0X30, phase1_ldr_cur);
    hx3605_write_reg(0X31, phase3_ldr_cur);
    hx3605_write_reg(0X32, phase4_ldr_cur);
    hx3605_write_reg(0X33, phase2_ldr_cur);

    hx3605_write_reg(0X34, (phase1_pd_sel<<4 |  phase1_ldr_sel));
    hx3605_write_reg(0X35, (phase3_pd_sel<<4 |  phase3_ldr_sel));
    hx3605_write_reg(0X36, (phase4_pd_sel<<4 |  phase4_ldr_sel));
    hx3605_write_reg(0X37, (phase2_pd_sel<<4 |  phase2_ldr_sel));

    hx3605_write_reg(0X38, phase1_offset_idac);
    hx3605_write_reg(0X39, phase3_offset_idac);
    hx3605_write_reg(0X3a, phase4_offset_idac);
    hx3605_write_reg(0X3b, phase2_offset_idac);
    hx3605_write_reg(0X3c, data_avg_num<<4 | data_avg_num );
    hx3605_write_reg(0X3d, data_avg_num<<4 | data_avg_num );

		hx3605_write_reg(0X18,(phase1_enable<<3)|(phase1_adc_osr)|(phase3_enable<<7)|(phase3_adc_osr<<4) );
    hx3605_write_reg(0X19,(phase4_enable<<3)|(phase4_adc_osr)|(phase2_enable<<7)|(phase2_adc_osr<<4) );

// analog circuit cfg
    hx3605_write_reg(0X60, 0x0a);	//1a= adc self test
    hx3605_write_reg(0X66, 0x92);	//0x92= r2r idac en; 0x91= mos idac en; 0x93= two idac en;
    hx3605_write_reg(0X67, 0xbf);	//32k osc cfg relate
    hx3605_write_reg(0X69, 0xa0);	//bit<0>: rc_comb_en bits<1>=rc_rbp_en bits<7>= vcom_clamp_en bits<6:4>= LED_vdesl
    hx3605_write_reg(0X6a, 0X02);	//02= u_low_pow, INT cmos output

/////////FIFO and adc conversion ready config////////
    hx3605_write_reg(0X12, 0x20);   // fifo almostfull cfg ,max=0x40;
    hx3605_write_reg(0X13, 0x11);
    hx3605_write_reg(0X20, 0x03);   // int width
    hx3605_write_reg(0X23, 0x00);   // phase int sel  80=p1 / 10=p2 / 40=p3 / 20 =p4

		#if defined(INT_MODE)
    hx3605_write_reg(0X24, 0x20);   // fifo int output sel
		#else
    hx3605_write_reg(0X24, 0x00);   // fifo int output sel
		#endif

    hx3605_write_reg(0X51, 0x02);
    hx3605_delay(5);
    hx3605_write_reg(0X13, 0x10);
    hx3605_delay(5);
    hx3605_write_reg(0X13, 0x11);
    hx3605_write_reg(0X51, 0x00);
		hx3605_delay(5);
}

bool hx3605_check_touch_open(void)
{
    return true;
}

SENSOR_ERROR_T hx3605_check_touch_enable(void)
{
    if (!hx3605_chip_check())
    {
        AGC_LOG("hx3690l check id failed!\r\n");
        return SENSOR_OP_FAILED;
    }

    AGC_LOG("hx3690l check id success!\r\n");


    if(!hx3605_check_touch_open())
    {
        AGC_LOG("spo2 alg open fail,or dynamic ram not enough!\r\n");
        return SENSOR_OP_FAILED;
    }
    hx3605_check_touch_init();
    hx3605_320ms_timer_cfg(true);

    return SENSOR_OK;
}

uint8_t hx3605_check_touch_read_fifo(int32_t *ir_buf)
{
    uint8_t data_flg = 127;
    int32_t data;
    uint8_t databuf[3];
    uint8_t ii=0;
    uint8_t data_size=0;
    uint8_t count=0;
    int32_t buf[64];

    data_size = hx3605_read_fifo_size();
    count = data_size>>1;
    data_size = count*2;

    for(ii=0; ii<data_size; ii++)
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
    for (ii=0; ii<count; ii++)
    {
      //DEBUG_PRINTF(0,"%d %d %d\r\n" ,buf[ii*2],buf[ii*2+1],hx3605_read_reg(0x2c));
        if(buf[ii*2]>buf[ii*2+1])
        {
            ir_buf[ii] = buf[ii*2] - buf[ii*2+1];
        }
        else
        {
            ir_buf[ii] = 0;
        }
    }
    return count;
}

hx3605_hrs_wear_msg_code_t hx3605_check_touch_send_data(int32_t *ir_data, uint8_t count)
{
  uint8_t ii;
  hx3605_hrs_wear_msg_code_t wear_status;
  if(count==0)
  {
    return wear_status;
  }
  for(ii=0; ii<count; ii++)
  {
    wear_status = check_touch_alg(ir_data[ii]);
  }
  return wear_status;
}

hx3605_hrs_wear_msg_code_t check_touch_alg(int32_t ir_data)
{
   if(ir_data>check_mode_wear_thre)
   {
     notouch_cnt = 0;
     if(touch_cnt>=24)
     {
        hx3605_wear_status = MSG_HRS_WEAR;
        touch_cnt=0;
     }
     else
     {
        touch_cnt++;
     }
   }
   else if(ir_data<check_mode_unwear_thre)
   {
     touch_cnt = 0;
     if(notouch_cnt>=24)
     {
        hx3605_wear_status = MSG_HRS_NO_WEAR;
        notouch_cnt=0;
     }
     else
     {
        notouch_cnt++;
     }
   }
   if(hx3605_wear_status != hx3605_wear_status_pre)
   {
     if(hx3605_wear_status==MSG_HRS_WEAR)
     {
         hx3605_write_reg(0X31, 0x40);
         hx3605_write_reg(0X1f, 0X2f);
     }
     else
     {
         hx3605_write_reg(0X31, 0x00);
         hx3605_write_reg(0X1f, 0X29);
     }
     hx3605_wear_status_pre = hx3605_wear_status;
   }
   return hx3605_wear_status;
}
#endif
