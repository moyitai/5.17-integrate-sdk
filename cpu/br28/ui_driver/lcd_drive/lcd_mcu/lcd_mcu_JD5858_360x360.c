/*
** 包含board的头文件，确定baord里面开关的屏驱宏
*/
#include "app_config.h"


/*
** 驱动代码的宏开关
*/
#if TCFG_LCD_MCU_JD5858_ENABLE


#define LCD_DRIVE_CONFIG                    MCU_8BITS_RGB565

/*
** 包含imd头文件，屏驱相关的变量和结构体都定义在imd.h
*/
#include "asm/imd.h"
#include "asm/imb.h"
#include "includes.h"
#include "ui/ui_api.h"


#define SCR_X 0
#define SCR_Y 0
#define SCR_W 360
#define SCR_H 360
#define LCD_W 360
#define LCD_H 360
#define LCD_BLOCK_W 360
#define LCD_BLOCK_H 60
#define BUF_NUM 2

#define TELINE 0x0000

//========== JD5858+INX 1.3" initial setting ==========//
static const u8 lcd_mcu_jd5858_cmdlist[] ALIGNED(4) = {
    //PASSWORD
    _BEGIN_, 0xDF, 0x58, 0x58, 0xB0, _END_,

    //---------------- PAGE0 --------------
    _BEGIN_, 0xDE, 0x00, _END_,

    //VCOM_SET
    _BEGIN_, 0xB2, 0x01, 0x10, _END_, //VCOM

    //Gamma_Set
    _BEGIN_, 0xB7, 0x10, 0x4A, 0x00, 0x10, 0x4A, 0x00, _END_,

    //DCDC_SEL
    _BEGIN_, 0xBB, 0x01, 0x1D, 0x43, 0x43, 0x21, 0x21, _END_,

    //GATE_POWER
    _BEGIN_, 0xCF, 0x20, 0x50, _END_,

    //SET_R_GAMMA
    _BEGIN_, 0xC8, 0x7F, 0x52, 0x3B, 0x2A, 0x22, 0x12, 0x17, 0x04, 0x21, 0x26, 0x29, 0x4B, 0x3A, 0x45, 0x3A, 0x35, 0x2C, 0x1E, 0x01, 0x7F, 0x52, 0x3B, 0x2A, 0x22, 0x12, 0x17, 0x04, 0x21, 0x26, 0x29, 0x4B, 0x3A, 0x45, 0x3A, 0x35, 0x2C, 0x1E, 0x01, _END_,

    //-----------------------------
    // SET page4 TCON & GIP
    //------------------------------
    _BEGIN_, 0xDE, 0x04, _END_,

    //SETSTBA
    _BEGIN_, 0xB2, 0x14, 0x14, _END_,

    //SETRGBCYC1
    _BEGIN_, 0xB8, 0x74, 0x44, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x09, 0x82, 0x10, 0x8A, 0x03, 0x11, 0x0B, 0x84, 0x21, 0x8C, 0x05, 0x22, 0x0D, 0x86, 0x32, 0x8E, 0x07, 0x00, 0x00, 0x00, _END_,

    //SETRGBCYC2
    _BEGIN_, 0xB9, 0x40, 0x22, 0x08, 0x3A, 0x22, 0x4B, 0x7D, 0x22, 0x8D, 0xBF, 0x32, 0xD0, 0x02, 0x33, 0x12, 0x44, 0x00, 0x0A, 0x00, 0x0A, 0x0A, 0x00, 0x0A, 0x0A, 0x00, 0x0A, 0x0A, _END_,

    //SETRGBCYC3
    _BEGIN_, 0xBA, 0x00, 0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x01, 0x01, 0x00, 0x0A, 0x01, 0x00, 0x01, 0x30, 0x0A, 0x40, 0x30, 0x01, 0x3E, 0x00, 0x00, 0x00, 0x00, _END_,


    //SET_TCON
    _BEGIN_, 0xBC, 0x1A, 0x00, 0xB4, 0x03, 0x00, 0xD0, 0x08, 0x00, 0x07, 0x2C, 0x00, 0xD0, 0x08, 0x00, 0x07, 0x2C, 0x82, 0x00, 0x03, 0x00, 0xD0, 0x08, 0x00, 0x07, 0x2C, _END_,

    //-------------------GIP----------------------
    //SET_GIP_EQ
    _BEGIN_, 0xC4, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _END_,

    //SET_GIP_L
    _BEGIN_, 0xC5, 0x00, 0x1F, 0x1F, 0x1F, 0x1E, 0xDF, 0x1F, 0xC7, 0xC5, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, _END_,


    //SET_GIP_R
    _BEGIN_, 0xC6, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0xC4, 0xC6, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, _END_,


    //SET_GIP_L_GS
    _BEGIN_, 0xC7, 0x00, 0x1F, 0x1F, 0x1F, 0xDE, 0x1F, 0x00, 0xC4, 0xC6, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, _END_,


    //SET_GIP_R_GS
    _BEGIN_, 0xC8, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xC7, 0xC5, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, _END_,

    //SETGIP1
    _BEGIN_, 0xC9, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _END_,

    //SETGIP2
    _BEGIN_, 0xCB, 0x01, 0x10, 0x00, 0x00, 0x07, 0x01, 0x00, 0x0A, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x21, 0x23, 0x30, 0x00, 0x08, 0x04, 0x00, 0x00, 0x05, 0x10, 0x01, 0x04, 0x06, 0x10, 0x10, _END_,

    //SET_GIP_ONOFF
    _BEGIN_, 0xD1, 0x00, 0x00, 0x03, 0x60, 0x30, 0x03, 0x18, 0x30, 0x07, 0x3A, 0x30, 0x03, 0x18, 0x30, 0x03, 0x18, 0x30, 0x03, 0x18, _END_,

    //SET_GIP_ONOFF_WB
    _BEGIN_, 0xD2, 0x00, 0x30, 0x07, 0x3A, 0x32, 0xBC, 0x20, 0x32, 0xBC, 0x20, 0x32, 0xBC, 0x20, 0x32, 0xBC, 0x20, 0x30, 0x10, 0x20, 0x30, 0x10, 0x20, 0x30, 0x10, 0x20, 0x30, 0x10, 0x20, _END_,

    //SETGIP8_CKH1 CKH_ON/OFF_CKH0-CKH7_odd
    _BEGIN_, 0xD4, 0x00, 0x00, 0x03, 0x14, 0x00, 0x03, 0x20, 0x00, 0x09, 0x82, 0x10, 0x8A, 0x03, 0x11, 0x0B, 0x84, 0x21, 0x8C, 0x05, 0x22, 0x0D, 0x86, 0x32, 0x8E, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _END_,

    //---------------- PAGE0 --------------
    _BEGIN_, 0xDE, 0x00, _END_,

    // RAM_CTRL
    _BEGIN_, 0xD7, 0x20,  0x00,  0x00, _END_,

    //---------------- PAGE1 --------------
    _BEGIN_, 0xDE, 0x01, _END_,

    ////MCMD_CTRL
    _BEGIN_, 0xCA, 0x00, _END_,

    //---------------- PAGE2 --------------
    _BEGIN_, 0xDE, 0x02, _END_,

    //OSC DIV
    _BEGIN_, 0xC5, 0x03, _END_,

    //---------------- PAGE0 --------------
    _BEGIN_, 0xDE, 0x00, _END_,

    //Color Pixel Format
#if OUT_FORMAT(LCD_DRIVE_CONFIG) == FORMAT_RGB565
    _BEGIN_, 0x3A, 0x05, _END_,
#elif OUT_FORMAT(LCD_DRIVE_CONFIG) == FORMAT_RGB666
    _BEGIN_, 0x3A, 0x06, _END_,
#elif OUT_FORMAT(LCD_DRIVE_CONFIG) == FORMAT_RGB666
    _BEGIN_, 0x3A, 0x07, _END_,
#endif

    //TE ON
    _BEGIN_, 0x35, 0x00, _END_,

    _BEGIN_, 0x44, (TELINE >> 8) & 0xff, TELINE & 0xff, _END_,

    //MADCTL
    _BEGIN_, 0x36, 0x00, _END_,

    //SLP OUT
    _BEGIN_, 0x11, _END_,
    _BEGIN_, REGFLAG_DELAY, 120, _END_,

    //DISP ON
    _BEGIN_, 0x29, _END_,
    _BEGIN_, REGFLAG_DELAY, 50, _END_,

    /* _BEGIN_, 0x23, _END_,  */
    // _BEGIN_, 0x55, 2, _END_,
};


struct imd_param lcd_mcu_jd5858_param = {
    .scr_x    = SCR_X,
    .scr_y	  = SCR_Y,
    .scr_w	  = SCR_W,
    .scr_h	  = SCR_H,
    .in_width  = SCR_W,
    .in_height = SCR_H,

    .lcd_width  = LCD_W,
    .lcd_height = LCD_H,

    .in_format = OUTPUT_FORMAT_RGB565,
    .lcd_type = LCD_TYPE_MCU,

    .buffer_num = BUF_NUM,
    .buffer_size = LCD_BLOCK_W * LCD_BLOCK_H * 2,
    .fps = 60,

    .pap = {
        .out_format = OUT_FORMAT(LCD_DRIVE_CONFIG),
        .right_shift_2bit = 0,
        .dat_l2h_en = 0,
    },

    .debug_mode_en = false,
    .debug_mode_color = 0xff0000,
};

/*
** lcd背光控制
** 考虑到手表应用lcd背光控制需要更灵活自由，可能需要pwm调光，随时亮灭等
** 因此内部不操作lcd背光，全部由外部自行控制
*/
static int lcd_mcu_jd5858_backlight_ctrl(u8 onoff)
{
    return 0;
}


/*
** 设置lcd进入睡眠
*/
static void lcd_mcu_jd5858_entersleep(void)
{
    //TODO
    lcd_write_cmd(0x28, NULL, 0);
    lcd_write_cmd(0x10, NULL, 0);
    delay_2ms(120 / 2);	// delay 120ms
}

/*
** 设置lcd退出睡眠
*/
static void lcd_mcu_jd5858_exitsleep(void)
{
    //TODO
    lcd_write_cmd(0x11, NULL, 0);
    delay_2ms(5);	// delay 120ms
    lcd_write_cmd(0x29, NULL, 0);
}




REGISTER_LCD_DEVICE() = {
    .logo = "jd5858",
    .row_addr_align 	= 1,
    .column_addr_align	= 1,

    .lcd_cmd = (void *) &lcd_mcu_jd5858_cmdlist,
    .cmd_cnt = sizeof(lcd_mcu_jd5858_cmdlist) / sizeof(lcd_mcu_jd5858_cmdlist[0]),
    .param   = (void *) &lcd_mcu_jd5858_param,

    .reset 			= NULL,	// 没有特殊的复位操作，用内部普通复位函数即可
    .backlight_ctrl = NULL,//lcd_mcu_jd5858_backlight_ctrl,
    .entersleep 	= lcd_mcu_jd5858_entersleep,
    .exitsleep 		= lcd_mcu_jd5858_exitsleep,
};



#endif

