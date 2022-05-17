
//#include <stdio.h>
//#include <stdbool.h>
//#include <stdint.h>
//#include <stdlib.h>
//#include "SEGGER_RTT.h"
//#include "app_timer.h"
//#include "nrf_delay.h"
//#include "nrf_gpio.h"
//#include "nrf_drv_gpiote.h"
//#include "twi_master.h"
//#include "drv_oled.h"
//#include "opr_oled.h"
//#include "oled_iic.h"
//#include "word.h"
//#include "iic.h"
#include <math.h>
#include "hrSensor_manage.h"
#include "printf.h"
#include "hx3605_factory_test.h"
#include "hx3605.h"
#include "app_config.h"
#include "os/os_api.h"
#include "asm/clock.h"

#ifdef SPO2_VECTOR
#include "spo2_vec.h"
uint32_t spo2_send_cnt = 0;
uint32_t spo2_send_cnt1 = 0;
int32_t red_buf_vec[8];
int32_t ir_buf_vec[8];
int32_t green_buf_vec[8];
#endif
#if   TCFG_HRS3605_EN
#ifdef HR_VECTOR
#include "hr_vec.h"
uint32_t spo2_send_cnt = 0;
uint32_t PPG_buf_vec[8];
#endif

#ifdef HRV_TESTVEC
#include "hrv_testvec.h"
#endif

#ifdef GSENSER_DATA
//#include "lis3dh_drv.h"
#endif


const uint8_t  hx3605_accurate_first_shot = 0;
const uint8_t  hx3605_up_factor1 = 3;
const uint8_t  hx3605_up_shift1 = 2;
const uint8_t  hx3605_up_factor2 = 31;
const uint8_t  hx3605_up_shift2 = 5;

// configure for JUDGE_HUMAN_ENABLE
const uint8_t  hx3605_human_delta1_thrs     = 40;   //The smaller value easy enter human status,recommand vaule 40 to 60
const uint8_t  hx3605_human_delta1_thrs2    = 20;   //The smaller value easy enter human status,recommand vaule 20 to 30
const uint8_t  hx3605_human_thrs            = 4;    //The smaller value easy enter human status,recommand vaule 2 to 6 or to 10
const uint16_t  hx3605_human_static_thrs    = 4000; //The bigger value easy enter human status,recommand value 2000 to 9000,

//SPO2 agc
const uint8_t  hx3605_spo2_agc_red_idac = RED_AGC_OFFSET;  //110 6,7,8...
const uint8_t  hx3605_spo2_agc_ir_idac = IR_AGC_OFFSET;  //110 6,7,8...
const uint8_t  hx3605_spo2_agc_green_idac = GREEN_AGC_OFFSET;  //110 6,7,8...
//hrs agc
const uint8_t  hx3605_hrs_agc_idac = HR_GREEN_AGC_OFFSET;  //default=14 6,7,8...
//hrv agc
const uint8_t  hx3605_hrv_agc_idac = HR_GREEN_AGC_OFFSET;

const uint8_t  green_led_max_init = 63;
const uint8_t  red_led_max_init = 63;
const uint8_t  ir_led_max_init = 63;
uint8_t low_vbat_flag =0;
uint8_t read_fifo_first_flg = 0;

//HRS_INFRARED_THRES
const int32_t  hrs_ir_unwear_thres =100000;//-2000;//
const int32_t  hrs_ir_wear_thres = 20000;//-3000;//;
//HRV_INFRARED_THRES
const int32_t  hrv_ir_unwear_thres = 8000;
const int32_t  hrv_ir_wear_thres = 15000;
//SPO2_INFRARED_THRES
const int32_t  spo2_ir_unwear_thres = 10000;
const int32_t  spo2_ir_wear_thres = 3000;
//CHECK_WEAR_MODE_THRES
const int32_t  check_mode_unwear_thre = 20000;
const int32_t  check_mode_wear_thre = 40000;

//static volatile oled_display_t oled_dis = {0};
uint32_t vec_data_cnt=0;

uint8_t spo2_rand_cnt = 0;
int8_t spo2_rand_value = 0;
int8_t spo2_rand_value_pre = 0;

static uint8_t demo_pow_swi = 1;
extern hx3605_sports_mode_t sports_mode_menu;

#if defined(MALLOC_MEMORY)
uint8_t alg_ram[2*1024] = {0};
#else
uint8_t alg_ram[11*1024] = {0};
#endif

#ifdef GSENSER_DATA
int16_t gsen_fifo_x[16];  //ppg time 330ms..330/40 = 8.25
int16_t gsen_fifo_y[16];
int16_t gsen_fifo_z[16];
#else
int16_t gen_dummy[16] = {0};
#endif

WORK_MODE_T work_mode_flag = HRS_MODE;
//////// spo2 para and switches
const  uint8_t   COUNT_BLOCK_NUM = 50;            //delay the block of some single good signal after a series of bad signal
const  uint8_t   SPO2_LOW_XCORR_THRE = 30;        //(64*xcorr)'s square below this threshold, means error signal
const  uint8_t   SPO2_CALI = 1;                       //spo2_result cali mode
const  uint8_t   XCORR_MODE = 1;                  //xcorr mode switch
const  uint8_t   QUICK_RESULT = 1;                //come out the spo2 result quickly ;0 is normal,1 is quick
const  uint16_t  MEAN_NUM = 256;                  //the length of smooth-average ;the value of MEAN_NUM can be given only 256 and 512
const  uint8_t   G_SENSOR = 1;                      //if =1, open the gsensor mode
const  uint8_t   SPO2_GSEN_POW_THRE = 150;         //gsen pow judge move, range:0-200;
const  uint32_t  SPO2_BASE_LINE_INIT = 138000;    //spo2 baseline init, = 103000 + ratio(a variable quantity,depends on different cases)*SPO2_SLOPE
const  int32_t   SOP2_DEGLITCH_THRE = 100000;     //remove signal glitch over this threshold
const  int32_t   SPO2_REMOVE_JUMP_THRE = 50000;  //remove signal jump over this threshold
const  uint32_t  SPO2_SLOPE = 50000;              //increase this slope, spo2 reduce more
const  uint16_t  SPO2_LOW_CLIP_END_TIME = 1500;   //low clip mode before this data_cnt, normal clip mode after this
const  uint16_t  SPO2_LOW_CLIP_DN  = 150;         //spo2 reduce 0.15/s at most in low clip mode
const  uint16_t  SPO2_NORMAL_CLIP_DN  = 500;      //spo2 reduce 0.5/s at most in normal clip mode
const  uint8_t   SPO2_LOW_SNR_THRE = 40;          //snr below this threshold, means error signal
const  uint16_t  IR_AC_TOUCH_THRE = 200;          //AC_min*0.3
const  uint16_t  IR_FFT_POW_THRE = 500;           //fft_pow_min
const  uint16_t  HRV_LENGTH = 128;
const  uint16_t  STATIC_THRE = 150;
#ifdef MALLOC_MEMORY
void *hx_malloc(size_t size)
{
    return (void*)malloc(size);
}

void hx_free(void * ptr)
{
    free(ptr);
}
#endif
void nrf_delay_us(uint32_t us)
{
	delay_us(us);
}
void nrf_delay_ms(uint32_t ms)
{
	delay(ms);
}

void hx3605_delay_us(uint32_t us)
{
    nrf_delay_us(us);
}

void hx3605_delay(uint32_t ms)
{
    nrf_delay_ms(ms);
}
struct _hx3605_dev_platform_data {
    uint8_t iic_hdl;
    uint8_t iic_delay;        //这个延时并非影响iic的时钟频率，而是2Byte数据之间的延时
};
static struct _hx3605_dev_platform_data hx3605_iic_info_test = {
    .iic_hdl = 1,
    .iic_delay = 0,
};
static struct _hx3605_dev_platform_data *hx3605_iic_info;

uint8_t hx3605_is_idle = 0;
spinlock_t sensor_3605_iic;


#if 0//TCFG_HR_SENOR_USER_IIC_TYPE
#define iic_init(iic)                       hw_iic_init(iic)
#define iic_uninit(iic)                     hw_iic_uninit(iic)
#define iic_start(iic)                      hw_iic_start(iic)
#define iic_stop(iic)                       hw_iic_stop(iic)
#define iic_tx_byte(iic, byte)              hw_iic_tx_byte(iic, byte)
#define iic_rx_byte(iic, ack)               hw_iic_rx_byte(iic, ack)
#define iic_read_buf(iic, buf, len)         hw_iic_read_buf(iic, buf, len)
#define iic_write_buf(iic, buf, len)        hw_iic_write_buf(iic, buf, len)
#define iic_suspend(iic)                    hw_iic_suspend(iic)
#define iic_resume(iic)                     hw_iic_resume(iic)
#else
#define iic_init(iic)                       soft_iic_init(iic)
#define iic_uninit(iic)                     soft_iic_uninit(iic)
#define iic_start(iic)                      soft_iic_start(iic)
#define iic_stop(iic)                       soft_iic_stop(iic)
#define iic_tx_byte(iic, byte)              soft_iic_tx_byte(iic, byte)
#define iic_rx_byte(iic, ack)               soft_iic_rx_byte(iic, ack)
#define iic_read_buf(iic, buf, len)         soft_iic_read_buf(iic, buf, len)
#define iic_write_buf(iic, buf, len)        soft_iic_write_buf(iic, buf, len)
#define iic_suspend(iic)                    soft_iic_suspend(iic)
#define iic_resume(iic)                     soft_iic_resume(iic)
#endif

enum I2C_ADDR_HRS3605{I2C_ADDR_HRS3605_W = 0x88, I2C_ADDR_HRS3605_R = 0x89,};

static uint8_t hx3605_write_data(uint8_t hx3605_reg_addr, uint8_t data)
{
    /* os_mutex_pend(&SENSOR_IIC_MUTEX,0); */
    //printf("-----%s\n",__func__);
    spin_lock(&sensor_3605_iic);
    iic_start(hx3605_iic_info->iic_hdl);
	// printf("aaaaa\n");
    if (0 == iic_tx_byte(hx3605_iic_info->iic_hdl, I2C_ADDR_HRS3605_W)) {
        iic_stop(hx3605_iic_info->iic_hdl);
        spin_unlock(&sensor_3605_iic);
        log_e("hx3603 write fail1!\n");
        return false;
    }
	 //printf("bbbbb\n");
    delay_2ms(hx3605_iic_info->iic_delay);
    if (0 == iic_tx_byte(hx3605_iic_info->iic_hdl, hx3605_reg_addr)) {
        iic_stop(hx3605_iic_info->iic_hdl);
        spin_unlock(&sensor_3605_iic);
        log_e("hx3603 write fail2!\n");
        return false;
    }
	// printf("ccccc\n");
    delay_2ms(hx3605_iic_info->iic_delay);
    if (0 == iic_tx_byte(hx3605_iic_info->iic_hdl, data)) {
        iic_stop(hx3605_iic_info->iic_hdl);
        spin_unlock(&sensor_3605_iic);
        log_e("hx3603 write fail3!\n");
        return false;
    }
	// printf("dddd\n");
    iic_stop(hx3605_iic_info->iic_hdl);
    delay_2ms(hx3605_iic_info->iic_delay);
    /* os_mutex_post(&SENSOR_IIC_MUTEX); */
    spin_unlock(&sensor_3605_iic);
    return true;
}



bool hx3605_write_reg(uint8_t addr, uint8_t data)
{
	//printf("hx3605_write_reg addr %0x data %d",addr,data);
	hx3605_write_data(addr,data);
	return 0;
}
static uint8_t hx3605_read_data(uint8_t hx3605_reg_addr, uint8_t *read_data, uint8_t len)//通
{
     //printf("-----%s\n",__func__);
    /* os_mutex_pend(&SENSOR_IIC_MUTEX,0); */
    spin_lock(&sensor_3605_iic);
    iic_start(hx3605_iic_info->iic_hdl);
	 //printf("222aaaaa\n");
    if (0 == iic_tx_byte(hx3605_iic_info->iic_hdl, I2C_ADDR_HRS3605_W)) {
        iic_stop(hx3605_iic_info->iic_hdl);
        spin_unlock(&sensor_3605_iic);
        log_e("hx3603 read fail1!\n");
        return false;
    }
	// printf("333aaaaa\n");
    delay_2ms(hx3605_iic_info->iic_delay);
    if (0 == iic_tx_byte(hx3605_iic_info->iic_hdl, hx3605_reg_addr)) {
        iic_stop(hx3605_iic_info->iic_hdl);
        spin_unlock(&sensor_3605_iic);
        log_e("hx3603 read fail2!\n");
        return false;
    }
	// printf("444aaaaa\n");
    delay_2ms(hx3605_iic_info->iic_delay);
    iic_start(hx3605_iic_info->iic_hdl);
    if (0 == iic_tx_byte(hx3605_iic_info->iic_hdl, I2C_ADDR_HRS3605_R)) {
        iic_stop(hx3605_iic_info->iic_hdl);
        spin_unlock(&sensor_3605_iic);
        log_e("hx3603 read fail3!\n");
        return false;
    }
	//printf("5555aaaaa\n");

    for (uint8_t i = 0; i < len - 1; i++) {
        read_data[i] = iic_rx_byte(hx3605_iic_info->iic_hdl, 1);
        delay_2ms(hx3605_iic_info->iic_delay);
    }
    read_data[len - 1] = iic_rx_byte(hx3605_iic_info->iic_hdl, 0);
	//printf("read_data = %d", read_data[len - 1]);//读到的值
    delay_2ms(hx3605_iic_info->iic_delay);
    /* os_mutex_post(&SENSOR_IIC_MUTEX); */
    spin_unlock(&sensor_3605_iic);
    return true;
}
uint8_t hx3605_read_reg(uint8_t addr)
{
	//printf("-----%s\n",__func__);

     uint8_t data_buf = 0;
    if (hx3605_read_data(addr, &data_buf, 1)) {
        return data_buf;
    } else {
        return 0;
    }

}

bool hx3605_brust_read_reg(uint8_t addr , uint8_t *buf, uint8_t length)
{
	//printf("-----%s\n",__func__);
	hx3605_read_data(addr, buf, length);
    return true;
}


/*
bool hx3605_write_reg(uint8_t addr, uint8_t data)
{

  	uint8_t data_buf[2];
    data_buf[0] = addr;
    data_buf[1] = data;
    twi_pin_switch(1);
    twi_master_transfer(0x88, data_buf, 2, true);    //write
    return 0;

}

uint8_t hx3605_read_reg(uint8_t addr)
{

		uint8_t data_buf;
    twi_pin_switch(1);
    twi_master_transfer(0x88, &addr, 1, false);      //write
    twi_master_transfer(0x89, &data_buf, 1, true);//read
    return data_buf;
}

bool hx3605_brust_read_reg(uint8_t addr , uint8_t *buf, uint8_t length)
{
    twi_pin_switch(1);
    twi_master_transfer(0x88, &addr, 1, false);      //write
    twi_master_transfer(0x89, buf, length, true); //read
    return true;
}
*/
uint16_t hx3605_read_fifo_data(int32_t *buf, uint8_t phase_num, uint8_t sig)
{
	//printf("-----%s\n",__func__);

    uint8_t data_flg_start = 0;
		uint8_t data_flg_end = 0;
    int32_t data;
    uint8_t databuf[3];
    uint32_t ii=0;
		uint16_t data_len = 0;
		uint16_t fifo_data_length = 0;
		uint16_t fifo_read_length = 0;
		uint16_t fifo_read_bytes = 0;
		uint16_t fifo_out_length = 0;
		uint16_t fifo_out_count = 0;
		uint8_t fifo_data_buf[96] = {0};

		data_len = hx3605_read_reg(0x30);
		fifo_data_length = data_len;
		//DEBUG_PRINTF("%d\r\n",data_len);
		if(fifo_data_length<2*phase_num)
		{
			return 0;
		}
		fifo_read_length = ((fifo_data_length-phase_num)/phase_num)*phase_num;
		fifo_read_bytes = fifo_read_length*3;
		if(read_fifo_first_flg == 1)
		{
			hx3605_brust_read_reg(0x31, databuf, 3);
			read_fifo_first_flg = 0;
		}
		hx3605_brust_read_reg(0x31, fifo_data_buf, fifo_read_bytes);
		//printf("hx3605_read_fifo_data==========test\n");
		//for(ii=0; ii<fifo_read_bytes; ii++)
		//{
		//	printf("ii+1=%d/fifo_read_bytes=%d, fifo_data_buf=%d\r\n", ii+1,fifo_read_bytes,fifo_data_buf[ii]);
		//}
		for(ii=0; ii<fifo_read_length; ii++)
		{
			if(sig==0)
			{
				buf[ii] = (int32_t)(fifo_data_buf[ii*3]|(fifo_data_buf[ii*3+1]<<8)|((fifo_data_buf[ii*3+2]&0x1f)<<16));
			}
			else
			{
				if((fifo_data_buf[ii*3+2]&0x10)!=0)
				{
					buf[ii] = (int32_t)(fifo_data_buf[ii*3]|(fifo_data_buf[ii*3+1]<<8)|((fifo_data_buf[ii*3+2]&0x0f)<<16))-1048576;
				}
				else
				{
					buf[ii] = (int32_t)(fifo_data_buf[ii*3]|(fifo_data_buf[ii*3+1]<<8)|((fifo_data_buf[ii*3+2]&0x1f)<<16));
				}
			}
			//SEGGER_RTT_printf(0,"%d/%d, %d %d\r\n", ii+1,fifo_read_length,buf[ii],(fifo_data_buf[ii*3+2]>>5)&0x03);
		}
		data_flg_start = (fifo_data_buf[2]>>5)&0x03;
		data_flg_end = (fifo_data_buf[fifo_read_bytes-1]>>5)&0x03;
		fifo_out_length = fifo_read_length;
		if(data_flg_start>0)
		{
			fifo_out_length = fifo_read_length-phase_num+data_flg_start;
			for(ii=0; ii<fifo_out_length; ii++)
			{
				buf[ii] = buf[ii+phase_num-data_flg_start];
			}
			for(ii=fifo_out_length; ii<fifo_read_length; ii++)
			{
				buf[ii] = 0;
			}
		}
		if(data_flg_end<phase_num-1)
		{
			for(ii=fifo_out_length; ii<fifo_out_length+phase_num-data_flg_end-1; ii++)
			hx3605_brust_read_reg(0x31, databuf, 3);
			buf[ii] = (int32_t)(databuf[0]|(databuf[1]<<8)|((databuf[2]&0x1f)<<16));
		}
//		for(ii=0; ii<fifo_out_length; ii++)
//		{
//			SEGGER_RTT_printf(0,"%d/%d, %d\r\n", ii+1,fifo_out_length,buf[ii]);
//		}
		fifo_out_length = fifo_out_length+phase_num-data_flg_end-1;
		fifo_out_count = fifo_out_length/phase_num;
		return fifo_out_count;
}

void read_data_packet(int32_t *ps_data) // 20200615 ericy read reg_data phase1 and phase3
{
	//printf("-----%s\n",__func__);

    uint8_t  databuf1[6] = {0};
    uint8_t  databuf2[6] = {0};
		uint8_t  databuf3[6] = {0};
		uint8_t  databuf4[6] = {0};
    uint32_t P1 = 0, P2 = 0, P4 = 0, P3 = 0;
    hx3605_brust_read_reg(0x02, databuf1, 6);
		hx3605_brust_read_reg(0x08, databuf2, 6);

		ps_data[0] = ((databuf1[0]) | (databuf1[1] << 8) | (databuf1[2] << 16));
		ps_data[1] = ((databuf1[3]) | (databuf1[4] << 8) | (databuf1[5] << 16));
		ps_data[2] = ((databuf2[0]) | (databuf2[1] << 8) | (databuf2[2] << 16));
		ps_data[3] = ((databuf2[3]) | (databuf2[4] << 8) | (databuf2[5] << 16));
		//SEGGER_RTT_printf(0, " %d %d %d %d \r\n",  ps_data[0], ps_data[1], ps_data[2], ps_data[3]);
    return ;
}

uint8_t chip_id = 0;
bool hx3605_chip_check(void)
{
	printf("-----%s\n",__func__);

    uint8_t i = 0;
	//uint8_t databuf[2];
    for(i=0;i<10;i++)
    {
        hx3605_write_reg(0x01, 0x00);
        hx3605_delay(5);
        chip_id = hx3605_read_reg(0x00);
		//hx3605_brust_read_reg(0x00, databuf, 2);
		//printf("databuf[0] = %d,databuf[1] = %d",databuf[0],databuf[1]);
		printf("chip_id = %d",chip_id);
        if (chip_id == 0x25)
        {
            return true;
        }
    }
    return false;
}

uint8_t hx3605_read_fifo_size(void) // 20200615 ericy read fifo data number
{
	//printf("-----%s\n",__func__);

    uint8_t fifo_num_temp = 0;
    fifo_num_temp = hx3605_read_reg(0x14)&0x7f;

    return fifo_num_temp;
}

void hx3605_vin_check(uint16_t led_vin)
{
	printf("-----%s\n",__func__);

    low_vbat_flag = 0;
    if(led_vin < 3700)
    {
      low_vbat_flag = 1;
    }
}

void hx3605_ppg_off(void) // 20200615 ericy chip sleep enable
{
	printf("-----%s\n",__func__);

    hx3605_write_reg(0x01, 0x01);
}

void hx3605_ppg_on(void)
{
	printf("-----%s\n",__func__);

    hx3605_write_reg(0x01, 0x00);
		hx3605_delay(5);
}

void hx3605_data_reset(void)
{
	printf("-----%s\n",__func__);

		#if defined(TIMMER_MODE)
    	hx3605_320ms_timer_cfg(false);
		#if defined(GSEN_40MS_TIMMER)
    	hx3605_40ms_timer_cfg(false);
		#endif
		#elif defined(INT_MODE)
   		hx3605_gpioint_cfg(false);
		#endif
		#if defined(HRS_ALG_LIB)
		hx3605_hrs_data_reset();
		#endif
		#if defined(SPO2_ALG_LIB)
		hx3605_spo2_data_reset();
		#endif
		#if defined(HRV_ALG_LIB)
		hx3605_hrv_data_reset();
		#endif
    printf("hx3690l data reset!\r\n");
}

void hx3605_demo_power_swi(void)
{
	printf("-----%s\n",__func__);

		if(demo_pow_swi==1)
		{
			hx3605_320ms_timer_cfg(false);
			hx3605_40ms_timer_cfg(false);
			hx3605_ppg_off();
			OLED_Display_Off();
			demo_pow_swi = 0;
		}
		else
		{
			//OLED_Display_On();
			hx3605_ppg_on();
			//OLED_DisplayString(0,0,16,16,"DEMO  OPEN");
			demo_pow_swi = 1;
		}
}

#ifdef HRS_ALG_LIB
void hx3605_hrs_ppg_init(void) //20200615 ericy ppg fs=25hz, phase3 conversion ready interupt en
{
	//printf("-----%s\n",__func__);

	uint16_t sample_rate = 25;                  /*config the data rate of chip alps2_fm ,uint is Hz*/
	uint32_t prf_clk_num = 32000/sample_rate;   /*period in clk num, num = Fclk/fs */
	uint8_t ps0_enable=1;       /*ps0_enable  , 1 mean enable ; 0 mean disable */
	uint8_t ps1_enable=1;       /*ps1_enable  , 1 mean enable ; 0 mean disable */
	uint8_t ps0_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 */
	uint8_t ps1_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 */

	uint8_t ps1_cp_avg_num_sel= 0;
	uint8_t ps0_cp_avg_num_sel= 1;
	uint8_t ps1_avg_num_sel_i2c=0;
	uint8_t ps0_avg_num_sel_i2c=0;
	/***********led open enable***********/
	uint8_t dccancel_ps0_data1 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps0_data2 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps1_data1 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps1_data2 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA

	uint8_t ir_pden_ps0 =0;     //IR_PDEN_PS0
	uint8_t ext_pden_ps0 =0;    //EXT_PDEN_PS0
	uint8_t pddrive_ps0 =0;     //PDDRIVE_PS0  0-63

	uint8_t ir_pden_ps1 =0;      //IR_PDEN_PS1
	uint8_t ext_pden_ps1 =0;     //EXT_PDEN_PS1
	uint8_t pddrive_ps1 =2;     //PDDRIVE_PS1  0-63
	uint8_t ps1_interval_i2c =0;    // config 0/5

	uint8_t led_en_begin =1;       // 0 = 2 ; 1 = 4 ; 2 = 8 ; 3 = 16 ;
	uint8_t afe_reset = 1;        //* 0 = 32clk ; 1 = 64clk ; 2 = 128clk ; 3 = 256clk(d) ;
	uint8_t en2rst_delay =2;
	uint8_t init_wait_delay = 2; /* 0 = 31clk ; 1 = 64clk ; 2 = 127clk ; 3 = 255clk(d) ;
	                             4 = 511clk; 5 = 1023clk; 6 = 2047; 7 = 2048clk */
	uint16_t thres_level =0;
	uint8_t thres_int =1;    //thres int enable
	uint8_t data_rdy_int =8;    //[3]:ps1_data2 [2]:ps1_data1 [1]:ps0_data2 [0]:ps0_data1

	uint8_t ldrsel_ps1 =2;      //ps1 LDR SELECT 01:ldr0-IR 02:ldr1-RLED  04:ldr2-GLED
	uint8_t ldrsel_ps0 =1;      //ps0 LDR SELECT 01:ldr0-IR 02:ldr1-RLED  04:ldr2-GLED
	/***********cap *********/
	uint8_t intcapsel_ps1 =1;   //01=4fp 02=8pf 03=12pf 04=16pf 05=20pf 06=24pf 07=28pf 08=32pf 09=36pf 10=40pf 11=44pf 12=48pf 13=52pf 14=56pf 15=60pf
	uint8_t intcapsel_ps0 =1;   //01=4fp 02=8pf 03=12pf 04=16pf 05=20pf 06=24pf 07=28pf 08=32pf 09=36pf 10=40pf 11=44pf 12=48pf 13=52pf 14=56pf 15=60pf

	uint8_t led_on_time  = 32;      /* 0 = 32clk2=8us ; 8 = 64clk=16us; 16=128clk=32us ; 32 = 256clk=64us ;
	                             64 = 512clk=128us ; 128= 1024clk=256us; 256= 2048clk=512us*/
	uint8_t force_adc_clk_sel=0;
	uint8_t force_adc_clk_cfg =0;
	uint8_t force_PEN =0;     //phase enable
	uint8_t force_PEN_cfg =0;
	uint8_t force_LED_EN =0;
	uint8_t force_LED_EN_cfg =0;
	uint8_t force_CKAFEINT_sel =0;
	uint8_t force_CKAFEINT_cfg =0;

	uint8_t PDBIASEN =0;
	uint8_t PDILOADEN =0;
	uint8_t databuf[3] = {0};

	hx3605_write_reg(0X10, (ps1_enable<<5| ps0_enable<<4|ps0_osr<<2|ps1_osr));	//default 0x00
	hx3605_write_reg(0X11, (uint8_t)prf_clk_num);    // prf bit<7:0>6   default 0x00
	hx3605_write_reg(0X12, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>  default 0x03
	hx3605_write_reg(0X13, (ps1_interval_i2c));   //default 0x00
	hx3605_write_reg(0X14, (led_en_begin<<6|afe_reset<<4|en2rst_delay<<2|init_wait_delay));  //default 0x00
	hx3605_write_reg(0X15, led_on_time); // led_en_num*8     //default 0x00
	hx3605_write_reg(0X16, (ps1_cp_avg_num_sel<<4)|(ps0_cp_avg_num_sel));  //default 0x00
	hx3605_write_reg(0X17, (ps1_avg_num_sel_i2c<<2)|(ps0_avg_num_sel_i2c));   //default 0x00

	hx3605_write_reg(0X18, (uint8_t)dccancel_ps0_data1);     //default 0x00
	hx3605_write_reg(0X19, (uint8_t)(dccancel_ps0_data1>>8));   //default 0x00
	hx3605_write_reg(0X1a, (uint8_t)dccancel_ps0_data2);   //default 0x00
	hx3605_write_reg(0X1b, (uint8_t)(dccancel_ps0_data2>>8));  //default 0x00
	hx3605_write_reg(0X1c, (uint8_t)dccancel_ps1_data1);   //default 0x00
	hx3605_write_reg(0X1d, (uint8_t)(dccancel_ps1_data1>>8));   //default 0x00
	hx3605_write_reg(0X1e, (uint8_t)dccancel_ps1_data2 );   //default 0x00
	hx3605_write_reg(0X1f, (uint8_t)(dccancel_ps1_data2 >>8));   //default 0x00

	hx3605_write_reg(0X20, (ir_pden_ps0<<7|ir_pden_ps0<<6|pddrive_ps0));  //default 0x00
	hx3605_write_reg(0X21, (ir_pden_ps1<<7|ir_pden_ps1<<6|pddrive_ps1));  //default 0x00
	hx3605_write_reg(0X22, (ldrsel_ps1<<4|ldrsel_ps0));      //default 0x00
	hx3605_write_reg(0X23, (intcapsel_ps1<<4|intcapsel_ps0));   //default 0x00
	hx3605_write_reg(0X26, (thres_int<<4|data_rdy_int));   //default 0x0f

	//    hx3605_write_reg(0X34, 0X10);  //w_almost_full ENABLE   /default0x04

	hx3605_write_reg(0X69, (force_adc_clk_sel<<7|force_adc_clk_cfg<<6|force_PEN<<5|force_PEN_cfg<<4|force_LED_EN<<3|
	                     force_LED_EN_cfg<<2|force_CKAFEINT_sel<<1|force_CKAFEINT_cfg));   //default 0x00
	hx3605_write_reg(0Xc2, 0x0b);
	hx3605_write_reg(0Xc3, (PDBIASEN<<5|PDILOADEN<<4));   //default 0x27

#if defined(TIMMER_MODE)
	hx3605_write_reg(0x2d,0x83);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0x2e,0x20);			 //watermark
	hx3605_write_reg(0x2c,0x1f);     // int_width_i2c
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf int,04=enable almost full int
#elif defined(FIFO_ALMOST_FULL_READ_ENABLE)
	hx3605_write_reg(0x2d,0x00);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0x2c,0x1f);     // int_width_i2c
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf int,04=enable almost full int
#else ///////////INT Mode
	hx3605_write_reg(0x2d,0x00);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0x2c,0x1f);     // int_width_i2c
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf ,04=enable almost full
#endif

	hx3605_write_reg(0X68, 0X01);  //soft reset
	nrf_delay_ms(5);
	hx3605_write_reg(0X68, 0X00);  //release
	nrf_delay_ms(5);
	read_fifo_first_flg = 1;
	printf("===read_fifo_first_flg = %d",read_fifo_first_flg);
}
#endif

#ifdef SPO2_ALG_LIB
void hx3605_spo2_ppg_init(void) //20200615 ericy ppg fs=25hz, phase3 conversion ready interupt en
{
	//printf("-----%s\n",__func__);

	uint16_t sample_rate = 25;                  /*config the data rate of chip alps2_fm ,uint is Hz*/
	uint32_t prf_clk_num = 32000/sample_rate;   /*period in clk num, num = Fclk/fs */
	uint8_t ps0_enable=1;       /*ps0_enable  , 1 mean enable ; 0 mean disable */
	uint8_t ps1_enable=1;       /*ps1_enable  , 1 mean enable ; 0 mean disable */
	uint8_t ps0_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 */
	uint8_t ps1_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 */

	uint8_t ps1_cp_avg_num_sel= 2;
	uint8_t ps0_cp_avg_num_sel= 2;
	uint8_t ps1_avg_num_sel_i2c=0;
	uint8_t ps0_avg_num_sel_i2c=0;
	/***********led open enable***********/
	uint8_t dccancel_ps0_data1 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps0_data2 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps1_data1 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA
	uint8_t dccancel_ps1_data2 =0;    //offset idac   BIT[9:0]  0 31.25nA, 1023 32uA, step 31.25nA

	uint8_t ir_pden_ps0 =0;     //IR_PDEN_PS0
	uint8_t ext_pden_ps0 =0;    //EXT_PDEN_PS0
	uint8_t pddrive_ps0 =0;     //PDDRIVE_PS0  0-63

	uint8_t ir_pden_ps1 =0;      //IR_PDEN_PS1
	uint8_t ext_pden_ps1 =0;     //EXT_PDEN_PS1
	uint8_t pddrive_ps1 =2;     //PDDRIVE_PS1  0-63
	uint8_t ps1_interval_i2c =0;    // config 0/5

	uint8_t led_en_begin =1;       // 0 = 2 ; 1 = 4 ; 2 = 8 ; 3 = 16 ;
	uint8_t afe_reset = 1;        //* 0 = 32clk ; 1 = 64clk ; 2 = 128clk ; 3 = 256clk(d) ;
	uint8_t en2rst_delay =2;
	uint8_t init_wait_delay = 2; /* 0 = 31clk ; 1 = 64clk ; 2 = 127clk ; 3 = 255clk(d) ;
	                 				4 = 511clk; 5 = 1023clk; 6 = 2047; 7 = 2048clk */
	uint16_t thres_level =0;
	uint8_t thres_int =1;    //thres int enable
	uint8_t data_rdy_int =8;    //[3]:ps1_data2 [2]:ps1_data1 [1]:ps0_data2 [0]:ps0_data1

	uint8_t ldrsel_ps1 =2;      //ps1 LDR SELECT 01:ldr0-IR 02:ldr1-RLED  04:ldr2-GLED
	uint8_t ldrsel_ps0 =4;      //ps0 LDR SELECT 01:ldr0-IR 02:ldr1-RLED  04:ldr2-GLED
	/***********cap *********/
	uint8_t intcapsel_ps1 =5;   //01=4fp 02=8pf 03=12pf 04=16pf 05=20pf 06=24pf 07=28pf 08=32pf 09=36pf 10=40pf 11=44pf 12=48pf 13=52pf 14=56pf 15=60pf
	uint8_t intcapsel_ps0 =5;   //01=4fp 02=8pf 03=12pf 04=16pf 05=20pf 06=24pf 07=28pf 08=32pf 09=36pf 10=40pf 11=44pf 12=48pf 13=52pf 14=56pf 15=60pf

	uint8_t led_on_time  = 5;      /* 0 = 32clk2=8us ; 8 = 64clk=16us; 16=128clk=32us ; 32 = 256clk=64us ;
	                 64 = 512clk=128us ; 128= 1024clk=256us; 256= 2048clk=512us*/
	uint8_t force_adc_clk_sel=0;
	uint8_t force_adc_clk_cfg =0;
	uint8_t force_PEN =0;     //phase enable
	uint8_t force_PEN_cfg =0;
	uint8_t force_LED_EN =0;
	uint8_t force_LED_EN_cfg =0;
	uint8_t force_CKAFEINT_sel =0;
	uint8_t force_CKAFEINT_cfg =0;

	uint8_t PDBIASEN =0;
	uint8_t PDILOADEN =0;
	uint8_t databuf[3] = {0};

    hx3605_write_reg(0X10, (ps1_enable<<5| ps0_enable<<4|ps0_osr<<2|ps1_osr));	//default 0x00
    hx3605_write_reg(0X11, (uint8_t)prf_clk_num);    // prf bit<7:0>6   default 0x00
    hx3605_write_reg(0X12, (uint8_t)(prf_clk_num>>8)); // prf bit<15:8>  default 0x03
    hx3605_write_reg(0X13, (ps1_interval_i2c));   //default 0x00
    hx3605_write_reg(0X14, (led_en_begin<<6|afe_reset<<4|en2rst_delay<<2|init_wait_delay));  //default 0x00
    hx3605_write_reg(0X15, led_on_time); // led_en_num*8     //default 0x00
    hx3605_write_reg(0X16, (ps1_cp_avg_num_sel<<4)|(ps0_cp_avg_num_sel));  //default 0x00
    hx3605_write_reg(0X17, (ps1_avg_num_sel_i2c<<2)|(ps0_avg_num_sel_i2c));   //default 0x00

    hx3605_write_reg(0X18, (uint8_t)dccancel_ps0_data1);     //default 0x00
    hx3605_write_reg(0X19, (uint8_t)(dccancel_ps0_data1>>8));   //default 0x00
    hx3605_write_reg(0X1a, (uint8_t)dccancel_ps0_data2);   //default 0x00
    hx3605_write_reg(0X1b, (uint8_t)(dccancel_ps0_data2>>8));  //default 0x00
    hx3605_write_reg(0X1c, (uint8_t)dccancel_ps1_data1);   //default 0x00
    hx3605_write_reg(0X1d, (uint8_t)(dccancel_ps1_data1>>8));   //default 0x00
    hx3605_write_reg(0X1e, (uint8_t)dccancel_ps1_data2 );   //default 0x00
    hx3605_write_reg(0X1f, (uint8_t)(dccancel_ps1_data2 >>8));   //default 0x00

    hx3605_write_reg(0X20, (ir_pden_ps0<<7|ir_pden_ps0<<6|pddrive_ps0));  //default 0x00
    hx3605_write_reg(0X21, (ir_pden_ps1<<7|ir_pden_ps1<<6|pddrive_ps1));  //default 0x00
    hx3605_write_reg(0X22, (ldrsel_ps1<<4|ldrsel_ps0));      //default 0x00
    hx3605_write_reg(0X23, (intcapsel_ps1<<4|intcapsel_ps0));   //default 0x00
    hx3605_write_reg(0X26, (thres_int<<4|data_rdy_int));   //default 0x0f

    //    hx3605_write_reg(0X34, 0X10);  //w_almost_full ENABLE   /default0x04

    hx3605_write_reg(0X69, (force_adc_clk_sel<<7|force_adc_clk_cfg<<6|force_PEN<<5|force_PEN_cfg<<4|force_LED_EN<<3|
                             force_LED_EN_cfg<<2|force_CKAFEINT_sel<<1|force_CKAFEINT_cfg));   //default 0x00
    hx3605_write_reg(0Xc2, 0x0b);
    hx3605_write_reg(0Xc3, (PDBIASEN<<5|PDILOADEN<<4));   //default 0x27

#if defined(TIMMER_MODE)
	hx3605_write_reg(0x2d,0x83);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0x2e,0x20);			 //watermark
	hx3605_write_reg(0x2c,0x1f);     // int_width_i2c
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf int,04=enable almost full int
#elif defined(FIFO_ALMOST_FULL_READ_ENABLE)
	hx3605_write_reg(0x2d,0x00);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0x2c,0x1f);     // int_width_i2c
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf int,04=enable almost full int
#else ///////////INT Mode
	hx3605_write_reg(0x2d,0x00);     //bits<3:0> fifo data sel, 0000 = p1;0001= p2;0010=p3;0011=p4;bits<7> fifo enble
	hx3605_write_reg(0x2c,0x1f);     // int_width_i2c
	hx3605_write_reg(0X27, 0x00);   // int sel,01=prf ,04=enable almost full
#endif

    hx3605_write_reg(0X68, 0X01);  //soft reset
    nrf_delay_ms(5);
    hx3605_write_reg(0X68, 0X00);  //release
	nrf_delay_ms(5);
		read_fifo_first_flg = 1;
}
#endif

#ifdef HRV_ALG_LIB
void hx3605_hrv_ppg_init(void) //20200615 ericy ppg fs=25hz, phase3 conversion ready interupt en
{
	//printf("-----%s\n",__func__);

    uint16_t sample_rate = 125; /*config the data rate of chip alps2_fm ,uint is Hz*/

    uint32_t prf_clk_num = 32000/sample_rate;   /*period in clk num, num = Fclk/fs */

    uint8_t cic_mode_en =0;
    uint8_t cic_b2_en = 0;
    uint8_t samp_delay_leden_num = 0; /* 00=8,01=16,10=32,11=64*/
    uint8_t samp_copy_avg = 0;        /* 0=1, 1=2, 2=4 ,3=8, 4=16*/
    uint8_t data_avg_num = 0;         /* 0 = 1 ; 1 = 2; 2 =4 ; 3 =8 ; 4 =16 ;*/
    uint8_t phase3_4_internal = 0;    /* phase3 and phase4 prf internal cfg */

    uint8_t phase1_enable = 1;     /*phase1_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase2_enable = 0;     /*phase2_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase3_enable = 1;     /*phase3_enable  , 1 mean enable ; 0 mean disable */
    uint8_t phase4_enable = 1;     /*phase4_enable  , 1 mean enable ; 0 mean disable */

    uint8_t phase1_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase2_adc_osr = 0;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase3_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    uint8_t phase4_adc_osr = 3;    /* 0 = 128 ; 1 = 256 ; 2 = 512 ; 3 = 1024 ; 4 = 2048*/
    //green
    uint8_t phase1_inner_avg = 0;   /* phase1 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase1_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase1_ldr_sel = GREEN_LED_SLE;     /*ball led 1 = ldr1(red); 2 = ldr2(ir); 4 = ldr3(green); 8 = ldr4 ;
                                    * 3in1 led 1 = ldr1(red); 2 = ldr2(green); 4 = ldr3(ir); 8 = ldr4 ;
                                    * 205U led 1 = ldr1(green); 2 = ldr2(red); 4 = ldr3(ir); 8 = ldr4 ;
                                    * GT01 led 1 = ldr1(green); 2 = ldr2(IR); 4 = ldr3(red); 8 = ldr4 ;
                                    */
    uint8_t phase1_pd_sel = 1;      /* 1 = pd1; 2 = pd2; */
    uint8_t phase1_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase1_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase1_led_en = 1;      /* phase1 led enable*/
    //als(green)
    uint8_t phase2_inner_avg = 0;   /* phase2 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase2_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase2_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase2_pd_sel = 0;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase2_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase2_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase2_led_en = 0;      /* phase2 led enable*/
    //ir
    uint8_t phase3_inner_avg = 0;   /* phase3 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase3_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase3_ldr_sel = IR_LED_SLE;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase3_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase3_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase3_ldr_cur = 64;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase3_led_en = 1;      /* phase3 led enable*/
    //als(ir)
    uint8_t phase4_inner_avg = 0;   /* phase4 adc avg num 0=1, 1=2, 2=4, 3=8 , 4=16*/
    uint8_t phase4_tia_res = 0;     /* 0= 10K; 1= 20k; 2= 50k; 3= 100k; 4= 150k; 5= 200k; 6= 500k; 7= 1M*/
    uint8_t phase4_ldr_sel = 0;     /* 1 = ldr1; 2 = ldr2; 4 = ldr3; 8 = ldr4 ; */
    uint8_t phase4_pd_sel = 1;      /* 1 = pd1; 2 = pd2; 4 = pd3; */
    uint8_t phase4_offset_idac = 0; /* 0~127 = 0 ~ 32ua , step = 0.25ua */
    uint8_t phase4_ldr_cur = 0;     /* 0~255 = 0 ~ 200ma, step = 0.8ma */
    uint8_t phase4_led_en = 0;      /* phase4 led enable*/

    uint8_t init_wait_delay = 5 ; /* 0 = 31clk ; 1 = 64clk ; 2 = 127clk ; 3 = 255clk(d) ;
                                     4 = 511clk; 5 = 1023clk; 6 = 2047; 7 = 2048clk */

    uint8_t afe_reset = 4;        /* 0 = 15clk ; 1 = 31clk ; 2 = 63clk ; 3 = 127clk(d) ;
                                     4 = 255clk; 5 = 511clk; 6 = 1024; 7 = 2048clk */

    uint8_t led_on_time = 3;      /* 0 = 32clk=8us ; 1 = 64clk=16us; 2=128clk=32us ; 3 = 256clk=64us ;
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

// analog circuit cfg
    hx3605_write_reg(0X60, 0x0a);	//1a= adc self test
    hx3605_write_reg(0X66, 0x92);	//0x92= r2r idac en; 0x91= mos idac en; 0x93= two idac en;
    hx3605_write_reg(0X67, 0xbf);	//32k osc cfg relate
    hx3605_write_reg(0X69, 0xa0);	//bit<0>: rc_comb_en bits<1>=rc_rbp_en bits<7>= vcom_clamp_en bits<6:4>= LED_vdesl
    hx3605_write_reg(0X6a, 0X02);	//02= u_low_pow, INT cmos output

/////////FIFO and adc conversion ready config////////
    hx3605_write_reg(0X13, 0x11);   /* bits<7:4> fifo data sel, 0000 = p1-p2;0001= p1,p2;0010=p3,p4;
                                       0011=p1,p2,p3,p4;0100=p3-(p2+p4)/2;0101=p1-p2,p3-p4;0110=p2-(p1+p3)/2;
                                       bits<3:2> fifo int clear mode, 00 = selfclear;01=reserve;10=manual clear;
									   bits<1:0> fifo mode sel, 00=bypass,01=fifo,10=stream,11=reserve;*/
    hx3605_write_reg(0X20, 0x03);   // int width
    hx3605_write_reg(0X23, 0x00);   // phase int sel  80=p1 / 10=p2 / 40=p3 / 20 =p4
#if defined(INT_MODE)
	hx3605_write_reg(0X12, 0x28);   // fifo almostfull cfg ,max=0x40;
	hx3605_write_reg(0X24, 0x20);   // fifo int output sel
#else
	hx3605_write_reg(0X12, 0x40);   // fifo almostfull cfg ,max=0x40;
	hx3605_write_reg(0X24, 0x00);   // fifo int output sel
#endif

    hx3605_write_reg(0X18,(phase1_enable<<3)|(phase1_adc_osr)|(phase3_enable<<7)|(phase3_adc_osr<<4) );
    hx3605_write_reg(0X19,(phase4_enable<<3)|(phase4_adc_osr)|(phase2_enable<<7)|(phase2_adc_osr<<4) );

    hx3605_write_reg(0X51, 0x02);
    hx3605_delay(5);
    hx3605_write_reg(0X13, 0x10);
    hx3605_delay(5);
    hx3605_write_reg(0X13, 0x11);
    hx3605_write_reg(0X51, 0x00);
	hx3605_delay(5);
}
#endif

uint32_t  hx3605_timers_start(void)
{
	//printf("-----%s\n",__func__);
	if(!hx3605_get_data_timer_320)
	{
		hx3605_get_data_timer_320 = sys_timer_add(NULL, heart_rate_meas_timeout_handler, 315);
	}
	return 0;
}
uint32_t  hx3605_timers_stop(void)
{
	//printf("-----%s\n",__func__);

	if(hx3605_get_data_timer_320)
	{
		sys_timer_del(hx3605_get_data_timer_320);
		hx3605_get_data_timer_320 = 0;
		os_time_dly(2);
	}
	return 0;
}

uint32_t  gsen_read_timers_start(void)
{
	//printf("-----%s\n",__func__);

	if(!hx3605_get_data_timer_40)
	{
		hx3605_get_data_timer_40 = sys_timer_add(NULL,hx3605_ppg_Int_handle, 40);
	}
	return 0;
}
uint32_t  gsen_read_timers_stop(void)
{
	//printf("-----%s\n",__func__);

	if(hx3605_get_data_timer_40)
	{
		sys_timer_del(hx3605_get_data_timer_40);
		hx3605_get_data_timer_40 = 0;
		os_time_dly(2);
	}
	return 0;
}

void hx3605_320ms_timer_cfg(bool en)
{
	printf("-----%s\n  en = %d",__func__,en);

    if(en)
    {
        hx3605_timers_start();
    }
    else
    {
        hx3605_timers_stop();
    }

}
void hx3605_40ms_timer_cfg(bool en)
{
	printf("-----%s\n  en = %d",__func__,en);

    if(en)
    {
        gsen_read_timers_start();
    }
    else
    {
        gsen_read_timers_stop();
    }
}
/*void hx3605_gpioint_cfg(bool en)
{
    if(en)
    {
         hx3605_gpioint_enable();
    }
    else
    {
         hx3605_gpioint_disable();
    }
}*/

bool hx3605_init(WORK_MODE_T mode)
{
    printf(" hx3605_init\n");
	hx3605_iic_info = &hx3605_iic_info_test;
   // work_mode_flag =HRS_MODE;//mode;//SPO2_MODE;//HRS_MODE,SPO2_MODE
    hx3605_data_reset();
	//timer3_init();
    hx3605_vin_check(3800);

	/*printf("work_mode_flag = %d",work_mode_flag);
		switch (work_mode_flag)
		{
			case HRS_MODE:
#ifdef HRS_ALG_LIB
		if(hx3605_hrs_enable()== SENSOR_OP_FAILED)
		{
		return false;
		}
#endif
#ifdef SPO2_ALG_LIB
		if(hx3605_spo2_enable()== SENSOR_OP_FAILED)
		{
		return false;
		}
#endif
				break;

			case LIVING_MODE:
				#ifdef CHECK_LIVING_LIB
				if(hx3605_hrs_enable()== SENSOR_OP_FAILED)
        {
            return false;
        }
				#endif
				break;

			case SPO2_MODE:
#ifdef SPO2_ALG_LIB
		if(hx3605_spo2_enable()== SENSOR_OP_FAILED)
		{
		return false;
		}
#endif
				break;

			case HRV_MODE:
				#ifdef HRV_ALG_LIB
				if(hx3605_hrv_enable()== SENSOR_OP_FAILED)
        {
            return false;
        }
				#endif
				break;

			case WEAR_MODE:
				#ifdef CHECK_TOUCH_LIB
				if(hx3605_check_touch_enable()== SENSOR_OP_FAILED)
        {
            return false;
        }
				#endif
				break;

			case FT_LEAK_LIGHT_MODE:
				hx3605_factroy_test(LEAK_LIGHT_TEST);
				break;

			case FT_GRAY_CARD_MODE:
				hx3605_factroy_test(GRAY_CARD_TEST);
				break;

			case FT_INT_TEST_MODE:
				hx3605_factroy_test(FT_INT_TEST);
//			  hx3605_gpioint_cfg(true);
				break;

			case LAB_TEST_MODE:
				#ifdef LAB_TEST
				if(hx3605_lab_test_enable()== SENSOR_OP_FAILED)
        {
            return false;
        }
				#endif
				break;

			default:
				break;
		}*/

    return true;
}

void hx3605_agc_Int_handle(void)
{
	//printf("-----%s\n=====%d",__func__,work_mode_flag);

    switch (work_mode_flag)
		{

				case HRS_MODE:
				{
					#ifdef HRS_ALG_LIB
					HRS_CAL_SET_T cal;
					cal = PPG_hrs_agc();
					if(cal.work)
					{
							AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d\r\n",
							cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
					}
					#endif
					break;
				}
				case LIVING_MODE:
				{
					#ifdef HRS_ALG_LIB
					HRS_CAL_SET_T cal;
					cal = PPG_hrs_agc();
					if(cal.work)
					{
							AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",
							cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
					}
					#endif
					break;
				}
				case HRV_MODE:
				{
					#ifdef HRV_ALG_LIB
					HRV_CAL_SET_T cal;
					cal = PPG_hrv_agc();
					if(cal.work)
					{
							AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",
							cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
					}
					#endif
					break;
				}
				case SPO2_MODE:
				{
					#ifdef SPO2_ALG_LIB
					SPO2_CAL_SET_T cal;
					cal = PPG_spo2_agc();
					if(cal.work)
					{
							AGC_LOG("AGC. Rled_drv=%d,Irled_drv=%d,RledDac=%d,IrledDac=%d,ambDac=%d,Rledstep=%d,Irledstep=%d,Rrf=%d,Irrf=%d,\r\n",\
							cal.R_LED, cal.IR_LED,cal.R_LEDDAC,cal.IR_LEDDAC,cal.AMBDAC,\
							cal.R_led_step,cal.IR_led_step,cal.R_RF,cal.IR_RF);
					}
					#endif
					break;
				}
				default:
					break;
		}
}

void gsen_read_timeout_handler(void * p_context)
{
	printf("-----%s\n",__func__);

		#ifdef TIMMER_MODE
    switch (work_mode_flag)
		{
				case HRS_MODE:
				{
					#ifdef HRS_ALG_LIB
					HRS_CAL_SET_T cal;
					cal = PPG_hrs_agc();
					if(cal.work)
					{
							AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,%x\r\n",
							cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF,hx3605_read_reg(0x20));
					}
					#endif
					break;
				}
				case LIVING_MODE:
				{
					#ifdef HRS_ALG_LIB
					HRS_CAL_SET_T cal;
					cal = PPG_hrs_agc();
					if(cal.work)
					{
							AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",
							cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
					}
					#endif
					break;
				}
				case HRV_MODE:
				{
					#ifdef HRV_ALG_LIB
					HRV_CAL_SET_T cal;
					cal = PPG_hrv_agc();
					if(cal.work)
					{
							AGC_LOG("AGC: led_drv=%d,ledDac=%d,ambDac=%d,ledstep=%d,rf=%d,\r\n",
							cal.LED, cal.LEDDAC, cal.AMBDAC,cal.led_step,cal.RF);
					}
					#endif
					break;
				}
				case SPO2_MODE:
				{
					#ifdef SPO2_ALG_LIB
					SPO2_CAL_SET_T cal;
					cal = PPG_spo2_agc();
					if(cal.work)
					{
							AGC_LOG("AGC. Rled_drv=%d,Irled_drv=%d,RledDac=%d,IrledDac=%d,ambDac=%d,Rledstep=%d,Irledstep=%d,Rrf=%d,Irrf=%d,\r\n",\
							cal.R_LED, cal.IR_LED,cal.R_LEDDAC,cal.IR_LEDDAC,cal.AMBDAC,\
							cal.R_led_step,cal.IR_led_step,cal.R_RF,cal.IR_RF);
					}
					#endif
					break;
				}
				case LAB_TEST_MODE:
				{
					#ifdef LAB_TEST
					hx3605_lab_test_Int_handle();
					#endif
				}
				default:
					break;
		}
		#endif

//		int32_t psdata[4]={0};
//		read_data_packet(psdata);

		#ifdef GSEN_40MS_TIMMER
		hx3605_gesensor_Int_handle();
		#endif
}

void heart_rate_meas_timeout_handler(void * p_context)
{
//printf("heart_rate_meas_timeout_handler===work_mode_flag = %d",work_mode_flag);

		switch(work_mode_flag)
		{
			case HRS_MODE://获取心率值
				#ifdef HRS_ALG_LIB
        hx3605_hrs_ppg_Int_handle();
        #endif
				break;

			case HRV_MODE:
				#ifdef HRV_ALG_LIB
        hx3605_hrv_ppg_Int_handle();
        #endif
				break;

			case SPO2_MODE://获取血氧值
				#ifdef SPO2_ALG_LIB
        hx3605_spo2_ppg_Int_handle();
        #endif
				break;

			case WEAR_MODE:
				#ifdef CHECK_TOUCH_LIB
				hx3605_wear_ppg_Int_handle();
				#endif
				break;

			case LIVING_MODE:
				#ifdef CHECK_LIVING_LIB
				hx3605_living_Int_handle();
				#endif
				break;

			default:
				break;
		}
}

void hx3605_ppg_Int_handle(void)
{
	//printf("-----%s\n",__func__);

	hx3605_agc_Int_handle();
	//printf("-----%s====================\n",__func__);
		#if defined(INT_MODE)
		switch(work_mode_flag)
		{
			case HRS_MODE:
				#ifdef HRS_ALG_LIB
        hx3605_hrs_ppg_Int_handle();
        #endif
				break;

			case SPO2_MODE:
				#ifdef SPO2_ALG_LIB
        hx3605_spo2_ppg_Int_handle();
        #endif
				break;

			case HRV_MODE:
				#ifdef HRV_ALG_LIB
        hx3605_hrv_ppg_Int_handle();
        #endif
				break;

			case WEAR_MODE:
				#ifdef CHECK_TOUCH_LIB
				hx3605_wear_ppg_Int_handle();
				#endif
				break;

			case LAB_TEST_MODE:
				#ifdef LAB_TEST
				hx3605_lab_test_Int_handle();
				#endif

			default:
				break;
		}
		#endif
}


#ifdef SPO2_DATA_CALI
static int32_t red_data_fifo[4] = {0};
static int32_t ir_data_fifo[4] = {0};
static int32_t red_dc_temp = 0;
static int32_t ir_dc_temp = 0;
static int32_t red_data_pre = 0;
static int32_t ir_data_pre = 0;
static int32_t red_jump_delta = 0;
static int32_t ir_jump_delta = 0;
static int32_t cali_data_cnt = 0;

int32_t hx3605_red_data_cali(int32_t red_new_raw_data)
{
	printf("-----%s\n",__func__);

    uint8_t ii;
    int32_t red_data_final;
    int32_t red_data_cali;
    hx3605_spo2_wear_msg_code_t touch_status;
    touch_status = hx3605_spo2_get_wear_status();
    if(touch_status == MSG_SPO2_NO_WEAR)
    {
      cali_data_cnt = 0;
      red_data_cali = red_new_raw_data;
      for(ii=0;ii<4;ii++)
      {
        red_data_fifo[ii] = 0;
      }
      red_dc_temp = 0;
    }
    else
    {
      for(ii=3;ii>0;ii--)
      {
          red_data_fifo[ii] = red_data_fifo[ii-1];
      }
      red_data_fifo[0] = red_new_raw_data;
      if(cali_data_cnt>=25)
      {
        if ((((red_data_fifo[1] - red_data_fifo[2]) > SOP2_DEGLITCH_THRE) && ((red_data_fifo[1] - red_new_raw_data) > SOP2_DEGLITCH_THRE)) || \
           (((red_data_fifo[1] - red_data_fifo[2]) < -SOP2_DEGLITCH_THRE) && ((red_data_fifo[1] - red_new_raw_data) < -SOP2_DEGLITCH_THRE)))
        {
            red_new_raw_data = red_data_fifo[2];
            red_data_fifo[1] = red_data_fifo[2];
        }
        else
        {
            red_new_raw_data = red_data_fifo[1];
        }
        if ((abs((red_new_raw_data - red_jump_delta) - red_data_pre) > SPO2_REMOVE_JUMP_THRE))
        {
            red_jump_delta = red_new_raw_data - red_data_pre;
        }
        red_data_cali = red_new_raw_data - red_jump_delta;
        red_data_pre = red_data_cali;
      }
      else
      {
        red_data_cali = red_data_fifo[1];
        red_data_pre = red_data_fifo[1];
      }
    }
    if(cali_data_cnt<=30)
    {
      red_dc_temp = red_data_cali;
      red_data_final = red_new_raw_data;
    }
    else
    {
      red_dc_temp = (red_dc_temp*31 + red_data_cali)>>5;
      red_data_final = red_data_cali - red_dc_temp + 2608*50*hx3605_spo2_agc_red_idac;
    }
    return red_data_final;
}
int32_t hx3605_ir_data_cali(int32_t ir_new_raw_data)
{
	printf("-----%s\n",__func__);

    uint8_t ii;
    int32_t ir_data_final;
    int32_t ir_data_cali;
    hx3605_spo2_wear_msg_code_t touch_status;
    touch_status = hx3605_spo2_get_wear_status();
    if(touch_status == MSG_SPO2_NO_WEAR)
    {
      cali_data_cnt = 0;
      ir_data_cali = ir_new_raw_data;
      for(ii=0;ii<4;ii++)
      {
        ir_data_fifo[ii] = 0;
      }
      ir_dc_temp = 0;
    }
    else
    {
      for(ii=3;ii>0;ii--)
      {
          ir_data_fifo[ii] = ir_data_fifo[ii-1];
      }
      ir_data_fifo[0] = ir_new_raw_data;
      cali_data_cnt++;
      if(cali_data_cnt>=25)
      {
        if ((((ir_data_fifo[1] - ir_data_fifo[2]) > SOP2_DEGLITCH_THRE) && ((ir_data_fifo[1] - ir_new_raw_data) > SOP2_DEGLITCH_THRE)) || \
           (((ir_data_fifo[1] - ir_data_fifo[2]) < -SOP2_DEGLITCH_THRE) && ((ir_data_fifo[1] - ir_new_raw_data) < -SOP2_DEGLITCH_THRE)))
        {
            ir_new_raw_data = ir_data_fifo[2];
            ir_data_fifo[1] = ir_data_fifo[2];
        }
        else
        {
            ir_new_raw_data = ir_data_fifo[1];
        }
        if ((abs((ir_new_raw_data - ir_jump_delta) - ir_data_pre) > SPO2_REMOVE_JUMP_THRE))
        {
            ir_jump_delta = ir_new_raw_data - ir_data_pre;
        }
        ir_data_cali = ir_new_raw_data - ir_jump_delta;
        ir_data_pre = ir_data_cali;
      }
      else
      {
        ir_data_cali = ir_data_fifo[1];
        ir_data_pre = ir_data_fifo[1];
      }
    }
    if(cali_data_cnt<=30)
    {
      ir_dc_temp = ir_data_cali;
      ir_data_final = ir_new_raw_data;
    }
    else
    {
      ir_dc_temp = (ir_dc_temp*31 + ir_data_cali)>>5;
      ir_data_final = ir_data_cali - ir_dc_temp + 2608*50*hx3605_spo2_agc_ir_idac;
    }
    return ir_data_final;
}
#endif
/*
void hx3605_gesensor_Int_handle(void)
{
	#ifdef GSENSER_DATA
    uint8_t ii = 0;
    AxesRaw_t gsen_buf;
    if(work_mode_flag == WEAR_MODE)
    {
        return;
    }

    LIS3DH_GetAccAxesRaw(&gsen_buf);

    for(ii=0;ii<15;ii++)
    {
        gsen_fifo_x[ii] = gsen_fifo_x[ii+1];
        gsen_fifo_y[ii] = gsen_fifo_y[ii+1];
        gsen_fifo_z[ii] = gsen_fifo_z[ii+1];
    }
    gsen_fifo_x[15] = gsen_buf.AXIS_X>>1;
    gsen_fifo_y[15] = gsen_buf.AXIS_Y>>1;
    gsen_fifo_z[15] = gsen_buf.AXIS_Z>>1;
printf("%d %d %d\r\n",gsen_fifo_x[15],gsen_fifo_y[15],gsen_fifo_z[15]);
		#endif
}*/

static uint8_t get_op_hrs_results;
static uint8_t get_op_hrs_wear_results;
uint8_t get_hrs_wear_results(void)
{
	return get_op_hrs_wear_results;
}
uint8_t get_hrs_results(void)
{
	return get_op_hrs_results ;
}

#ifdef HRS_ALG_LIB
hrs_sensor_data_t hrs_s_dat;
hx3605_results_t alg_results_pre;
void hx3605_hrs_ppg_Int_handle(void)
{
	//printf("-----%s\n",__func__);

    uint8_t        ii=0;
    hx3605_results_t alg_results = {MSG_HRS_ALG_NOT_OPEN,MSG_HRS_NO_WEAR,0,0,0,false};
	//uint8_t hrs_results = 0;

    int32_t *PPG_buf = &(hrs_s_dat.ppg_data[0]);
    uint32_t *als = &(hrs_s_dat.als);
    int32_t *ir_buf = &(hrs_s_dat.ir_data[0]);
    uint8_t *count = &(hrs_s_dat.count);
    int32_t *s_buf = &(hrs_s_dat.s_buf[0]);
		int16_t gsen_fifo_x_send[16]={0};
		int16_t gsen_fifo_y_send[16]={0};
		int16_t gsen_fifo_z_send[16]={0};
		#ifdef BP_CUSTDOWN_ALG_LIB
    hx3605_bp_results_t    bp_alg_results ;
		#endif
    #ifdef HR_VECTOR
      for(ii=0;ii<8;ii++)
      {
        PPG_buf_vec[ii] = hrm_input_data[spo2_send_cnt+ii];
        gsen_fifo_x[ii] = gsen_input_data_x[spo2_send_cnt+ii];
        gsen_fifo_y[ii] = gsen_input_data_y[spo2_send_cnt+ii];
        gsen_fifo_z[ii] = gsen_input_data_z[spo2_send_cnt+ii];
      }
      spo2_send_cnt = spo2_send_cnt+8;
      *count = 8;
      hx3605_alg_send_data(PPG_buf_vec,*count, 0, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);
    #else
      if(hx3605_hrs_read(&hrs_s_dat) == NULL)
      {
          return;
      }
      for(ii=0;ii<*count;ii++)//计步
      {
					gsen_fifo_x_send[ii] = gsen_fifo_x[16-*count+ii];
					gsen_fifo_y_send[ii] = gsen_fifo_y[16-*count+ii];
					gsen_fifo_z_send[ii] = gsen_fifo_z[16-*count+ii];
          //DEBUG_PRINTF("%d/%d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
          //PPG_buf[ii],ir_buf[ii],s_buf[ii*4],s_buf[ii*4+1],s_buf[ii*4+2], \
          //s_buf[ii*4+3],hrs_s_dat.agc_green);
         //printf("%d %d %d %d %d %d\r\n", PPG_buf[ii],ir_buf[ii],gsen_fifo_x[ii],gsen_fifo_y[ii],gsen_fifo_z[ii],hrs_s_dat.agc_green);
      }
      hx3605_alg_send_data(PPG_buf,*count, *als, gsen_fifo_x_send, gsen_fifo_y_send, gsen_fifo_z_send);
    #endif



    //display part
    alg_results = hx3605_alg_get_results();
	get_op_hrs_results = alg_results.hr_result;
	get_op_hrs_wear_results = alg_results.hrs_wear_status;

	//printf("hrs_alg_status = %d\n",alg_results.hrs_alg_status);
	printf("hrs_wear_status = %d hr_result = %d\n",alg_results.hrs_wear_status,alg_results.hr_result);
	//printf("data_cnt = %d\n",alg_results.data_cnt);
	//printf("hr_result = %d\n",alg_results.hr_result);
	//printf("cal_result = %d\n",alg_results.cal_result);
	//printf("hr_result_qual = %d\n",alg_results.hr_result_qual);
	//printf("object_flg = %d\n",alg_results.object_flg);

    //DEBUG_PRINTF(0,"====>>>>quality = %d\r\n", alg_results.hr_result_qual);
/*    oled_dis.refresh_time++;
		if(alg_results.hr_result==0)
		{
			oled_dis.dis_mode = DIS_HR;
		}
		if(alg_results.hr_result>0 && alg_results_pre.hr_result==0)
		{
			if(alg_results.hr_result_qual<5)
			{
				oled_dis.dis_mode = DIS_WAIT;
			}
		}
    if(oled_dis.refresh_time >= 5) //320ms*5 = 1600ms
    {
        oled_dis.refresh_flag = 1;
        oled_dis.refresh_time = 0;
        //oled_dis.dis_mode = DIS_HR;
        oled_dis.dis_data1 = alg_results.hr_result;
				oled_dis.dis_data2 = alg_results.cal_result;
    }*/
		alg_results_pre = alg_results;
 //   SEGGER_RTT_printf(0,"oledata: %d,oledstatus: %d\r\n", alg_results.hr_result,alg_results.hrs_wear_status);

    #ifdef HRS_BLE_APP
    {
        rawdata_vector_t rawdata;

        HRS_CAL_SET_T cal= get_hrs_agc_status();
        for(ii=0;ii<*count;ii++)
        {
            rawdata.vector_flag = HRS_VECTOR_FLAG;
            rawdata.data_cnt = alg_results.data_cnt-*count+ii;
            rawdata.hr_result = alg_results.hr_result;
            rawdata.red_raw_data = PPG_buf[ii];
            rawdata.ir_raw_data = ir_buf[ii];
            rawdata.gsensor_x = gsen_fifo_x[ii];
            rawdata.gsensor_y = gsen_fifo_y[ii];
            rawdata.gsensor_z = gsen_fifo_z[ii];
            rawdata.red_cur = cal.LED;
            rawdata.ir_cur = alg_results.hrs_alg_status;

            ble_rawdata_vector_push(rawdata);
        }
    }
    #endif
}


void hx3605_living_Int_handle(void)
{
	printf("-----%s\n",__func__);

	  uint8_t        ii=0;
    hx3605_living_results_t living_alg_results = {MSG_HRS_NO_WEAR,0,0,0};

    int32_t *PPG_buf = &(hrs_s_dat.ppg_data[0]);
    uint32_t *als = &(hrs_s_dat.als);
    uint8_t *count = &(hrs_s_dat.count);

		if(hx3605_hrs_read(&hrs_s_dat) == NULL)
		{

				return;
		}
		for(ii=0;ii<*count;ii++)
		{
				//DEBUG_PRINTF(0,"%d/%d %d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
				PPG_buf[ii],PPG_buf[ii],ir_buf[ii],s_buf[ii*4],s_buf[ii*4+1],s_buf[ii*4+2], \
				s_buf[ii*4+3],hrs_s_dat.agc_green);
			 //SEGGER_RTT_printf(0,"%d/%d %d %d %d %d %d %d\r\n",1+ii,*count,PPG_buf[ii],ir_buf[ii],gsen_fifo_x[ii],gsen_fifo_y[ii],gsen_fifo_z[ii],hrs_s_dat.agc_green);
		}
		hx3605_living_send_data(PPG_buf, *count, *als, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);
		living_alg_results = hx3605_living_get_results();
		//DEBUG_PRINTF(0,"%d %d %d %d\r\n" ,living_alg_results.data_cnt,living_alg_results.motion_status,living_alg_results.signal_quality,living_alg_results.wear_status);
}
#endif


#ifdef SPO2_ALG_LIB
static uint8_t get_op_spo2_results;
static uint8_t get_op_spo2_wear_results;
uint8_t get_spo2_wear_results(void)
{
	return get_op_spo2_wear_results;
}


uint8_t get_spo_result(void)
{
	return get_op_spo2_results ;
}

spo2_sensor_data_t spo2_s_dat;
void hx3605_spo2_ppg_Int_handle(void)
{
	//printf("-----%s\n",__func__);

	uint8_t        ii=0;
	hx3605_spo2_results_t alg_results = {MSG_SPO2_ALG_NOT_OPEN,MSG_SPO2_NO_WEAR,0,0,0,0,0};
	SPO2_CAL_SET_T cal=get_spo2_agc_status();
	int32_t *red_buf = &(spo2_s_dat.red_data[0]);
	int32_t *ir_buf = &(spo2_s_dat.ir_data[0]);
	int32_t *green_buf = &(spo2_s_dat.green_data[0]);
	uint8_t *count = &(spo2_s_dat.count);
	int32_t *s_buf = &(spo2_s_dat.s_buf[0]);
#ifdef SPO2_DATA_CALI
	int32_t red_data_cali, ir_data_cali;
#endif

#ifdef SPO2_VECTOR
	for(ii=0;ii<8;ii++)
	{
	red_buf_vec[ii] = vec_red_data[spo2_send_cnt+ii];
	ir_buf_vec[ii] = vec_ir_data[spo2_send_cnt+ii];
			green_buf_vec[ii] = vec_green_data[spo2_send_cnt+ii];
	}
	spo2_send_cnt = spo2_send_cnt+8;
	*count = 8;
	for(ii=0;ii<10;ii++)
	{
	gsen_fifo_x[ii] = vec_red_data[spo2_send_cnt1+ii];;
	gsen_fifo_y[ii] = vec_ir_data[spo2_send_cnt1+ii];;
	gsen_fifo_z[ii] = vec_green_data[spo2_send_cnt1+ii];;
	}
		spo2_send_cnt1 = spo2_send_cnt1+10;
	hx3605_spo2_alg_send_data(red_buf_vec, ir_buf_vec, green_buf_vec, *count, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);
#else
		if(hx3605_spo2_read(&spo2_s_dat) == NULL)
		{
				return;
		}
	//			for(ii=0;ii<*count;ii++)
	//			{
	//					DEBUG_PRINTF("%d/%d %d %d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
	//					red_buf[ii],ir_buf[ii],green_buf[ii], cal.G_LEDDAC,\
	//					cal.R_LEDDAC,cal.IR_LEDDAC,spo2_s_dat.agc_green,spo2_s_dat.agc_red,spo2_s_dat.agc_ir);
	//			}
	//	for(ii=0;ii<*count;ii++)
	//	{
	//			printf("%d/%d %d %d %d %d %d %d %d %d %d %d %d\r\n" ,1+ii,*count,\
	//			red_buf[ii],ir_buf[ii], cal.G_LEDDAC,\
	//			cal.R_LEDDAC,cal.IR_LEDDAC,spo2_s_dat.agc_green,spo2_s_dat.agc_red,spo2_s_dat.agc_ir,gsen_fifo_x[ii],gsen_fifo_y[ii],gsen_fifo_z[ii]);
	//	}
		hx3605_spo2_alg_send_data(red_buf, ir_buf, *count, gsen_fifo_x, gsen_fifo_y, gsen_fifo_z);
#endif

	alg_results = hx3605_spo2_alg_get_results();
	get_op_spo2_results = alg_results.spo2_result;
	get_op_spo2_wear_results = alg_results.spo2_wear_status;

	//printf("spo2_result:%d alg_status:%d wear_status
//:%d data_cnt:%d \r\n",\
	//	alg_results.spo2_result,alg_results.spo2_alg_status,alg_results.spo2_wear_status,alg_results.data_cnt);

	//printf("hr_result:%d block_cnt:%d ir_quality:%d reliable_score_result:%d \r\n",alg_results.hr_result,alg_results.block_cnt,alg_results.ir_quality,alg_results.reliable_score_result);

	//printf("spo2_alg_status = %d\n",alg_results.spo2_alg_status);
	printf("wear_status = %d spo2_result = %d\n",alg_results.spo2_wear_status,alg_results.spo2_result);
	//printf("data_cnt = %d\n",alg_results.data_cnt);
	//printf("hr_result = %d\n",alg_results.hr_result);
	//printf("block_cnt = %d\n",alg_results.block_cnt);
	//printf("ir_quality = %d\n",alg_results.ir_quality);
	//printf("reliable_score_result = %d\n",alg_results.reliable_score_result);
	/*    oled_dis.refresh_time++;
	if(alg_results.data_cnt<256)
	{
		oled_dis.dis_mode = DIS_SPO2;
	}
	if(alg_results.data_cnt>=256 && alg_results.data_cnt<280)
	{
		if(alg_results.ir_quality<5)
		{
			oled_dis.dis_mode = DIS_WAIT;
		}
	}
	if(oled_dis.refresh_time >= 7) //330ms*3 = 990ms ~ 1s
	{
	oled_dis.refresh_flag = 1;
	oled_dis.refresh_time = 0;
	//oled_dis.dis_mode = DIS_SPO2;
			oled_dis.dis_data1 = alg_results.hr_result;
	oled_dis.dis_data2 = alg_results.spo2_result;
	}*/

	//SEGGER_RTT_printf(0,"oledata: %d,oledstatus: %d\r\n", alg_results.hr_result,alg_results.alg_status);

#ifdef HRS_BLE_APP
	{
	rawdata_vector_t rawdata;
	SPO2_CAL_SET_T cal= get_spo2_agc_status();
	for(ii=0;ii<*count;ii++)
	{
	  #ifdef SPO2_DATA_CALI
	    ir_data_cali = hx3605_ir_data_cali(ir_buf[ii]);
	    red_data_cali = hx3605_red_data_cali(red_buf[ii]);
	    rawdata.red_raw_data = red_data_cali;
	    rawdata.ir_raw_data = ir_data_cali;
	  #else
	    rawdata.red_raw_data = red_buf[ii];
	    rawdata.ir_raw_data = ir_buf[ii];
	  #endif
	    rawdata.vector_flag = SPO2_VECTOR_FLAG;
	    rawdata.data_cnt = alg_results.data_cnt-*count+ii;
					//rawdata.data_cnt = cal.G_LEDDAC;
	    rawdata.hr_result = alg_results.spo2_result;
	    rawdata.gsensor_x = gsen_fifo_x[ii];
	    rawdata.gsensor_y = gsen_fifo_y[ii];
	    rawdata.gsensor_z = gsen_fifo_z[ii];
	//						rawdata.gsensor_x = (int16_t)(green_buf[ii]>>5);
	//            rawdata.gsensor_y = cal.R_LEDDAC;
	//            rawdata.gsensor_z = cal.IR_LEDDAC;
	    rawdata.red_cur = cal.R_LED;
	    rawdata.ir_cur = cal.IR_LED;
	    //SEGGER_RTT_printf(0,"G_LEDDAC: %d,R_LEDDAC: %d\r\n", cal.G_LEDDAC,cal.R_LEDDAC);
	    ble_rawdata_vector_push(rawdata);
	}
	}
#endif
}
#endif

#ifdef HRV_ALG_LIB
hrv_sensor_data_t hrv_s_dat;
void hx3605_hrv_ppg_Int_handle(void)
{
	printf("-----%s\n",__func__);

    uint8_t        ii=0;
    //hx3605_hrv_results_t alg_hrv_results = {MSG_HRV_ALG_NOT_OPEN,MSG_HRV_NO_WEAR,0,0,0,false};
    hx3605_hrv_results_t alg_hrv_results= {MSG_HRV_ALG_NOT_OPEN,MSG_HRV_NO_WEAR,0,0,0};
    int32_t *PPG_buf = &(hrv_s_dat.ppg_data[0]);
    uint8_t *count = &(hrv_s_dat.count);
	  hx3605_hrv_wear_msg_code_t wear_mode;

    #ifdef HRV_TESTVEC
		int32_t hrm_raw_data;
    hrm_raw_data = vec_data[vec_data_cnt];
    vec_data_cnt++;
    alg_hrv_results = hx3605_hrv_alg_send_data(hrm_raw_data, 0, 0);
		//DEBUG_PRINTF(0,"hrm_raw_data: %d\r\n", hrm_raw_data);
    #else
		if(hx3605_hrv_read(&hrv_s_dat) == NULL)
		{
				return;
		}
		for(ii=0;ii<*count;ii++)
		{
				//DEBUG_PRINTF("%d/%d %d\r\n" ,1+ii,*count,PPG_buf[ii]);
			  //SEGGER_RTT_printf("%d %d %d %d %d %d\r\n", PPG_buf[ii],ir_buf[ii],gsen_fifo_x[ii],gsen_fifo_y[ii],gsen_fifo_z[ii],hrs_s_dat.agc_green);
		}
		wear_mode = hx3605_hrv_get_wear_status();
		if(wear_mode==MSG_HRS_NO_WEAR)
		{
					alg_hrv_results.hrv_result = 0;
		}
		else
		{
          alg_hrv_results = hx3605_hrv_alg_send_bufdata(PPG_buf, *count, 0);
		}
		#endif

    //display part
    /*
    oled_dis.refresh_time++;
    if(oled_dis.refresh_time >= 10)
    {
        oled_dis.refresh_flag = 1;
        oled_dis.refresh_time = 0;
        oled_dis.dis_mode = DIS_HRV;
        oled_dis.dis_data1 = alg_hrv_results.hrv_result; //alg_hrv_results.hrv_peak;
				oled_dis.dis_data2 = alg_hrv_results.hrv_result;
    }
*/
    #ifdef HRS_BLE_APP
    {
        rawdata_vector_t rawdata;

        HRS_CAL_SET_T cal= get_hrs_agc_status();
        for(ii=0;ii<*count;ii++)
        {
            rawdata.vector_flag = HRS_VECTOR_FLAG;
            rawdata.data_cnt = 0;
            rawdata.hr_result = alg_hrv_results.hrv_result;
            rawdata.red_raw_data = PPG_buf[ii];
            rawdata.ir_raw_data = 0;
            rawdata.gsensor_x = gsen_fifo_x[ii];
            rawdata.gsensor_y = gsen_fifo_y[ii];
            rawdata.gsensor_z = gsen_fifo_z[ii];
            rawdata.red_cur = cal.LED;
            rawdata.ir_cur = 0;
            ble_rawdata_vector_push(rawdata);
        }
    }
    #endif

}
#endif

void hx3605_wear_ppg_Int_handle(void)
{
	printf("-----%s\n",__func__);

		#ifdef CHECK_TOUCH_LIB
    int32_t ir_data[16];
    uint8_t count;
    hx3605_hrs_wear_msg_code_t hx3605_wear_status;
    count = hx3605_check_touch_read_fifo(ir_data);
    hx3605_wear_status = hx3605_check_touch_send_data(ir_data,count);
		#endif
}

void hx3605_lab_test_Int_handle(void)
{
		#ifdef LAB_TEST
		uint32_t data_buf[4];
		hx3605_lab_test_read_packet(data_buf);
		//DEBUG_PRINTF("%d %d %d %d\r\n", data_buf[0], data_buf[1], data_buf[2], data_buf[3]);
		#endif
}
/*
void display_refresh(void)
{
  #ifdef BIG_SCREEN
    char dis_buf1[]="0000";
		char dis_buf2[]="0000";
    if(oled_dis.refresh_flag)
    {
        oled_dis.refresh_flag = 0;

        switch(oled_dis.dis_mode)
        {
            case DIS_WAIT:
               OLED_DisplayString(72,4,16,16,"---"); // init error
               OLED_DisplayString(72,6,16,16,"---"); // init error
               //opr_display_wait(0,oled_dis.dis_data);
               break;
            case DIS_BP:
               //opr_display_bp(oled_dis.dis_sbp,oled_dis.dis_dbp);
               break;
            case DIS_HR:
                sprintf(dis_buf1, "%02d", oled_dis.dis_data1);
                OLED_DisplayString(0,4,16,16,"Hrs ");
								OLED_DisplayString(72,4,16,16,"    "); // init error
                OLED_DisplayString(72,4,16,16,dis_buf1); // init error
						    sprintf(dis_buf2, "%02d", oled_dis.dis_data2);
                OLED_DisplayString(0,6,16,16,"Cal "); // init error
								OLED_DisplayString(72,6,16,16,"    "); // init error
                OLED_DisplayString(72,6,16,16,dis_buf2); // init error
               break;
            case DIS_SPO2:
                sprintf(dis_buf1, "%02d", oled_dis.dis_data1);
                OLED_DisplayString(0,4,16,16,"Hrs ");
								OLED_DisplayString(72,4,16,16,"    ");
                OLED_DisplayString(72,4,16,16,dis_buf1); // init error
						    sprintf(dis_buf2, "%02d", oled_dis.dis_data2);
                OLED_DisplayString(0,6,16,16,"Spo2"); // init error
								OLED_DisplayString(72,6,16,16,"    "); // init error
                OLED_DisplayString(72,6,16,16,dis_buf2); // init error
               break;
            case DIS_HRV:
                sprintf(dis_buf1, "%02d", oled_dis.dis_data1);
                OLED_DisplayString(0,4,16,16,"Hrv ");
								OLED_DisplayString(72,4,16,16,"    ");
                OLED_DisplayString(72,4,16,16,dis_buf1); // init error
						    sprintf(dis_buf2, "%02d", oled_dis.dis_data1);
                OLED_DisplayString(0,6,16,16,"Str "); // init error
								OLED_DisplayString(72,6,16,16,"    "); // init error
                OLED_DisplayString(72,6,16,16,dis_buf2); // init error
               break;
            default:
                sprintf(dis_buf1, "%02d", oled_dis.dis_data1);
                OLED_DisplayString(0,4,16,16,"Hrs ");
								OLED_DisplayString(72,4,16,16,"    ");
                OLED_DisplayString(72,4,16,16,dis_buf1); // init error
						    sprintf(dis_buf2, "%02d", oled_dis.dis_data2);
                OLED_DisplayString(0,6,16,16,"Spo2"); // init error
								OLED_DisplayString(72,6,16,16,"    "); // init error
                OLED_DisplayString(72,6,16,16,dis_buf2); // init error
               break;
        }
    }

  #else
//    if(oled_dis.refresh_flag)
//    {
//        oled_dis.refresh_flag = 0;

//        switch(oled_dis.dis_mode)
//        {
//            case DIS_WAIT:
//
//               opr_display_wait(0,oled_dis.dis_data);
//               break;
//            case DIS_BP:
//               opr_display_bp(oled_dis.dis_sbp,oled_dis.dis_dbp);
//               break;
//            case DIS_HR:
//               opr_display_hr(1,oled_dis.dis_data);
//               break;
//            case DIS_HRV:
//               opr_display_hr(1,oled_dis.dis_data);
//               break;
//            case DIS_SPO2:
//               opr_display_hr(0,oled_dis.dis_data);
//               break;
//
//            default:
//               opr_display_hr(1,oled_dis.dis_data);
//               break;
//        }
//    }
  #endif
}*/

#ifdef HRS_BLE_APP

#define  MAX_VECTOR     50
volatile rawdata_vector_t rawdata_vector[MAX_VECTOR];
uint32_t vec_push_p = 0;
uint32_t vec_pull_p = 0;
uint8_t ble_send_start = 0;
void ble_rawdata_vector_push(rawdata_vector_t rawdata);

#ifdef HRS_BLE_APP_1S
extern  void ble_hrs_heart_rate_send(uint16_t hrs);
#endif

void ble_rawdata_clear(void)
{
	printf("-----%s\n",__func__);

    int nIndex = 0;
    if (ble_send_start == 1)
    {
        vec_push_p = 0;
        vec_pull_p = 0;
        ble_send_start = 0;
        for(nIndex = 0; nIndex< MAX_VECTOR-1; nIndex++)
        {
            rawdata_vector[nIndex].vector_flag = HRS_VECTOR_FLAG;
        }
    }
    DEBUG_PRINTF(0, "ble_rawdata_clear\n");

}

void ble_rawdata_vector_push(rawdata_vector_t rawdata)
{
	printf("-----%s\n",__func__);

    int nIndex = 0;

    if (ble_send_start == 0)
    {
        vec_push_p = 0;
    }
    else
    {
        vec_push_p++;
        if (vec_push_p > MAX_VECTOR-1)
        {
            vec_push_p = 0;
        }
    }
    nIndex = vec_push_p;

    rawdata_vector[nIndex].vector_flag = rawdata.vector_flag;
    rawdata_vector[nIndex].data_cnt = rawdata.data_cnt;
    rawdata_vector[nIndex].hr_result = rawdata.hr_result;
    rawdata_vector[nIndex].red_raw_data = rawdata.red_raw_data;
    rawdata_vector[nIndex].ir_raw_data = rawdata.ir_raw_data;
    rawdata_vector[nIndex].gsensor_x = rawdata.gsensor_x;
    rawdata_vector[nIndex].gsensor_y = rawdata.gsensor_y;
    rawdata_vector[nIndex].gsensor_z = rawdata.gsensor_z;
    rawdata_vector[nIndex].red_cur = rawdata.red_cur;
    rawdata_vector[nIndex].ir_cur = rawdata.ir_cur;
}


uint32_t ble_rawdata_send_handler( )
{
    uint32_t error_code = NRF_ERROR_INVALID_STATE;
             //DEBUG_PRINTF(0,"cnt = %d, red=%d, ir=%d,X = %d, y= %d, z=%d\n", rawdata_vector[vec_pull_p].data_cnt, \
                        rawdata_vector[vec_pull_p].red_raw_data,rawdata_vector[vec_pull_p].ir_raw_data, \
                        rawdata_vector[vec_pull_p].gsensor_x, rawdata_vector[vec_pull_p].gsensor_y, \
                        rawdata_vector[vec_pull_p].gsensor_z);
    do
    {
        if (rawdata_vector[vec_pull_p].vector_flag >0)
        {

            error_code =  ble_hrs_heart_rate_send_ext(rawdata_vector[vec_pull_p].vector_flag,\
                        rawdata_vector[vec_pull_p].data_cnt,rawdata_vector[vec_pull_p].hr_result,\
                        rawdata_vector[vec_pull_p].red_raw_data,\
                        rawdata_vector[vec_pull_p].ir_raw_data,\
                        rawdata_vector[vec_pull_p].gsensor_x,\
                        rawdata_vector[vec_pull_p].gsensor_y,\
                        rawdata_vector[vec_pull_p].gsensor_z,\
                        rawdata_vector[vec_pull_p].red_cur ,rawdata_vector[vec_pull_p].ir_cur);



            if ( NRF_SUCCESS == error_code)
            {
                ble_send_start = 1;
                rawdata_vector[vec_pull_p].vector_flag = 0;
                vec_pull_p++;
                if (vec_pull_p > MAX_VECTOR-1)
                {
                    vec_pull_p = 0;
                }
            }
        }
    }
    while((NRF_SUCCESS == error_code)&&(rawdata_vector[vec_pull_p].vector_flag >0));

    return error_code;

}
#endif

static uint8_t	hrs_enable_swich = 0;
static uint8_t	spo_enable_swich = 0;
uint8_t get_hrs_enable_status()
{
return hrs_enable_swich;
}
uint8_t get_spo_enable_status()
{
return spo_enable_swich;
}

int Hrs3605_io_ctl(u8 cmd, void *arg)
{
	//printf("%s cmd = %d",__func__,cmd);
    switch (cmd) {
    case HR_SENSOR_ENABLE://0
#ifdef HRS_ALG_LIB
	if(!!hx3605_hrs_enable())
	{

	    printf("faild");
		break;
	}
	hrs_enable_swich = 1;
#endif
	break;
    case HR_SENSOR_DISABLE://1
#ifdef HRS_ALG_LIB
        hx3605_hrs_disable();
	hrs_enable_swich = 0;

            break;

#endif
        break;
    case SPO2_SENSOR_ENABLE://2
#ifdef SPO2_ALG_LIB
		if(hx3605_spo2_enable())
		{
            printf("faild");
            break;
		}
		spo_enable_swich = 1;
#endif
		break;
    case SPO2_SENSOR_DISABLE://3
#ifdef SPO2_ALG_LIB
        hx3605_spo2_disable();
		spo_enable_swich = 0;

            break;
#endif
    case HR_SENSOR_READ_DATA://4
		//printf("HR_SENSOR_READ_DATA=get_op_hrs_results = %d\n",get_op_hrs_results);
		return 1;
		break;
    case SPO2_SENSOR_READ_DATA://5
        //返回血氧传感器数据
		//printf("SPO2_SENSOR_READ_DATA=get_op_spo2_results = %d\n",get_op_spo2_results);
		return 1;
		break;
    case HR_SEARCH_SENSOR://6
		if (!hx3605_chip_check())
		{
			printf("hx3690l check id failed!\r\n");
		break;
		}
		char res = 1;
		memcpy(arg, &res, 1);

		break;
    default:
        break;
    }
    return 0;
}

REGISTER_HR_SENSOR(hrSensor) = {
    .logo = "hrs3605",
    .heart_rate_sensor_init  = hx3605_init,
    .heart_rate_sensor_check = NULL,
    .heart_rate_sensor_ctl   = Hrs3605_io_ctl,
};

static uint8_t hx3605_idle_query(void)
{
    return hx3605_is_idle;
}

REGISTER_LP_TARGET(hx3605_lp_target) = {
    .name = "hx3605",
    .is_idle = hx3605_idle_query,
};
#endif//TCFG_HRS3605_EN

