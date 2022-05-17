/******************************************************************************/
#ifndef _DEF_BD_HEALTH_H_
#define _DEF_BD_HEALTH_H_

/*******�Զ��庯��**********/
uint8_t get_hrs_results(void);
uint8_t get_spo_result(void);
uint8_t get_spo2_wear_results(void);
uint8_t get_hrs_wear_results(void);
uint8_t get_hrs_enable_status(void);
uint8_t get_spo_enable_status(void);

//////***************define const value**********/////////
//////*************define data type****************////////
/*
typedef   signed char     int8_t;
typedef   signed short    int16_t;
typedef   signed int     int32_t;
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int     uint32_t;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
#ifndef bool
#define bool unsigned char

#endif
*/
///////////-----------------����˵��------------------////////
///////--------��Ҫ�û����ж����ⲿ����˵��----------/////////
extern uint16_t bp_data_up,bp_data_down,bp_difdata;
///////******ʱ���ӳٺ���˵��--START********/////////
////------��������:
///---ʱ���ӳٺ�������Ҫ�ͻ��ṩ
///---�β�˵����delay_timeΪ��Ҫ�ӳٵ�ʱ����ֵ(��λms)
extern void Delay_ms(u16 delay_time);
//////*********ʱ���ӳٺ���--END*************///////

///////******PWM-LED������������--START********/////////
////------��������:
///---����LED��PWM�������ߵ�ƽ��Ч���͵�ƽ��Ч
///---�β�˵����duty_dataΪPWM�����ź���Ч��ƽ��ռ�ձ�(��λ%)
extern void LED_PWM_On(uint8_t duty_data);
//////*********PWM-LED��������--END*************///////

///////******PWM-LED�����رպ���--START********/////////
////------��������:
///---�ر�LED��PWM����
extern void LED_PWM_Off(void);
//////*********PWM-LED��������--END*************///////

//////******I2CͨѶд����˵��--START*********///////
////------�������ܣ�
///---���ļ��в��ֺ�����Ҫ���ÿͻ�оƬ��I2C�ӿ�(�ͻ�оƬ��I2CΪ����)�򴫸���(�ӻ�)�������
//��Ҫ�ͻ������Լ�оƬӲ������д��I2Cд������
///---�β�˵����slave_addressΪ���贫�����ӻ���ַ��cmd_regΪ��Ҫ��д���ݵļĴ�����ַ�������������ʶ
//dataΪҪ��д������
///---����ֵ˵����1---дOK��0---дʧ��
extern uint8_t BD_I2C_Write(uint8_t slave_address,uint8_t cmd_reg,uint8_t data);
//////************I2Cд����--END*************///////

//////******I2CͨѶ������˵��--START*********///////
////------�������ܣ�
///---���ļ��в��ֺ�����Ҫ���ÿͻ�оƬ��I2C�ӿ�(�ͻ�оƬI2CΪ����)�Ӵ�����(�ӻ�)��ȡ���ݣ�
//��Ҫ�ͻ������Լ�оƬӲ������д��I2C��������
///---�β�˵����slave_addressΪ���贫�����ӻ���ַ��cmd_regΪ��Ҫ�����ݵļĴ�����ַ��
//state_commuָ��ָ��洢ͨѶ���״̬�ı�����ַ(1--ͨѶ��ȡ�ɹ���0--ͨѶ��ȡʧ��)
///---����ֵ˵��������ȡ�Ĵ����е���ֵ
extern uint8_t BD_I2C_Read(uint8_t slave_address,uint8_t cmd_reg,uint8_t *state_commu);
//////************I2C������--END*************///////
//////----------�û��Զ����ⲿ����˵��--END-----------//////////

//////////---------���ͻ�ʹ�õ��㷨�⺯��˵��--START-----------////////////

//////*******�㷨��ʼ������--START******/////////
////------�������ܣ�
///---�㷨�������ʼ��
///---����λ�ã��������ͻ�ϵͳ�ϵ�����������֮ǰ�����������ӿں���֮ǰ��
//ÿ�ζϵ�������ϵ��Ҫ����һ�δ˺���
void BD_heartbp_init(void);
//////***********����--END*********///////////

///////************��������ʼ������*************//////////
///---����ֵ��1--��ʼ��OK��0--��ʼ��ʧ��
//---λ�ã�ÿ�δ������ϵ�������ϵ���Ҫִ��һ�δ˺������˺���������ڳ�BD_heartbp_init(void)����֮�����������֮ǰ��
uint8_t BD_Sensor_Init(void);
//////***********����--END*********///////////


///////*********��������������**********//////////
///---����ֵ��1--����OK��0--����ʧ��
uint8_t BD_Sensor_Start(void);
//////***********����--END*********///////////

///////**********�رմ���������**********//////////
///---����ֵ��1--�ر�OK��0--�ر�ʧ��
uint8_t BD_Sensor_Close(void);
//////***********����--END*********///////////

///////**********��ʱ(50����)��ȡ����������**********//////////
///---�β�˵����in_buf����ָ�룬�洢�Ӵ�������ȡ����������
///---����ֵ˵����
//    0--��ʾһ֡������δ��ȡ��ϣ�
//    1--��ʾһ֡���ݶ�ȡ��ϣ�
//    2--��ʾͨѶ�쳣
uint8_t BD_Sensor_Data_Read(uint16_t *in_buf);
//////***********��ʱ��ȡ����--END*********///////////


//////***********���ݴ�������START********///////////
////------�ββ���˵����
//--cmd_ack:����������(8-���ʣ�Ѫѹ��Ѫ��)
//--rxdata_buf������������洢ָ���ַ
//--bd_heartbp_buf: ���������洢ָ���ַ

////-----����ֵ˵����
//--7���㷨������
//--8: �㷨������ϣ�[0]-����,[1]-��ѹ,[2]-��ѹ,[3]-Ѫ��,[4]-����״̬��[5]~[14]Ϊ���Բ���
//--14���������ݻ�����Ч��������
///---����λ�ã����������ѭ����������������ж��ӳ�������(�㷨ִ��һ�ε�ʱ��ϳ�����˲���������ж�����)�������յ�һ�����ݺ��ִ�д˺���
uint8_t BD_health_pro(uint8_t cmd_ack,uint16_t*rxdata_buf,uint32_t *bd_heartbp_buf);
//////*********���ݴ�������END**************////////////
/////-----------------�⺯��˵��END-----------/////////
void BD_16XX_datapro(void);
void BD_16XX_dataread(void);
void BD_16XX_close(void);
uint8_t BD_16XX_init_start(void);
void bd_50ms_timer_cfg(bool en);
void bd_timers_stop(void);
void bd_timers_start(void);
//1.50ms Ӳ����ʱ��
//�����ϴ�ֵ �ֻ�
//

#endif
//\************************END OF FILE***********************\//