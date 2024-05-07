
#include "UART.h"

bit Timer1Overflow;	//������1�����־λ
unsigned char disbuff[4]={0,0,0,0};//���ڷֱ��ž����ֵ�ף����ף�����
unsigned int LeftDistance = 0, RightDistance = 0, FrontDistance = 0; //��̨�����뻺��
unsigned int DistBuf[5] = {0};//distance data buffer
unsigned int   TickTime = 0;//ϵͳ�δ��ʱ
unsigned char  idata g_UartData;//���ֽڴ�������
unsigned char	pwm_val_left,pwm_val_right;	//�м�������û������޸ġ�
unsigned char 	pwm_left,pwm_right;			//����PWM����ߵ�ƽ��ʱ��ı������û�����PWM�ı�����
unsigned char 	g_LeftSpeed = 120,g_RightSpeed = 120;//�޸�������ܿ��Ƴ��ٷֱ��Ӧ���ҳ��٣�ȡֵ��Χ60-200
enum e_SetMode SysMode = E_BleRemote;	//ϵͳ����ģʽ
#define		PWM_DUTY		255			//����PWM�����ڣ���ֵΪ��ʱ��0������ڣ����綨ʱ�����ʱ��Ϊ100us����PWM����Ϊ20ms��
#define		PWM_HIGH_MIN	100			//����PWM�������Сռ�ձȡ��û������޸ġ�
#define		PWM_HIGH_MAX	PWM_DUTY	//����PWM��������ռ�ձȡ��û������޸ġ�
#define 	IRAVOID 1  //���忪��������Ϲ���
#define 	IRTRACK 1  //���忪������ѭ������

void Timer0_Init(void); //��ʱ��0��ʼ��
void Timer1_Init(void);//��ʱ��1��ʼ��
void UartInit(void);		//���ڳ�ʼ��9600bps@11.0592MHz
void Ext1_Init(void);  //�ⲿ�ж�1��ʼ�������ڼ��ģʽ�л�����key״̬
void QXMBOT_LoadPWM(void);//װ��PWM���ֵ 
void Delay_Ms(unsigned int ms);//���뼶��ʱ����
void forward(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11����С��ǰ�� 
void left_run(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11����С����ת  
void right_run(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11����С����ת
void back_run(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11����С������
void SmartCarLeftTurn(unsigned char LeftSpeed, unsigned char RightSpeed);//QX_A11����С��ԭ����ת
void SmartCarRightTurn(unsigned char LeftSpeed, unsigned char RightSpeed);//QX_A11����С��ԭ����ת
void stop(void);//QX_A11����С��ͣ��
void Tracking(void);//����ѭ��
void Tracking2(void);//����ѭ��2
void Track_Avoid(unsigned int val);//����ѭ��+���������϶���һ
void IRAvoid(void);//�������
void IRFollow(void);//�������

void Delay18450us(void);	//@11.0592MHz
void Delay1550us(void);		//@11.0592MHz
void Delay19400us(void);	//@11.0592MHz
void Delay600us(void);		//@11.0592MHz
void Delay17500us(void);	//@11.0592MHz
void Delay2500us(void);		//@11.0592MHz

void RICHBOT_bubble(unsigned int *a,unsigned char n);//ð������
void RICHBOT_ServoFront(void);//�����ǰ
void RICHBOT_ServoLeft(void);//�����ת
void RICHBOT_ServoRight(void);//�����ת
void RICHBOT_CSB_Avoid(unsigned int val);//������+�������
void  RICHBOT_PTZ_Avoid(unsigned int val);//�����̨����
unsigned int RICHBOT_GetDistance(void);//��ȡ����������
void RICHBOT_TrigUltrasonic(void);// ����������
unsigned int RICHBOT_RefreshDistance(void);//���������

void BLE_Remote(void);//��������ң��
u8 getIRAvoidSta(void);//��ȡ���ϴ�����״̬
u8 getTrackSta(void);//��ȡѭ��������״̬
char *PackSysData(void);//���ϵͳ����

/*********************************************
QX_A11����С������ң��ִ�к���
��ڲ�������
���ڲ���: ��
˵����ͨ�����յ���������������ִ����Ӧ����
��λ���������ݸ�ʽΪ4λ
��һλ��ʼ�ַ�'$'
�ڶ�λ�����ַ�'C'��ʾ����λ����Ϊ�������ߵ����ݣ�L��ʾ����λΪ������������
����λʮ����������
����λ�����ַ�'#'
**********************************************/
void BLE_Remote()
{
	switch(g_UartData)
	{
		case 1: stop(); break;
		case 2: forward(g_LeftSpeed, g_RightSpeed); break;
		case 3: back_run(g_LeftSpeed, g_RightSpeed); break;
		case 4: SmartCarLeftTurn(g_LeftSpeed, g_RightSpeed);	break;
		case 5: SmartCarRightTurn(g_LeftSpeed, g_RightSpeed);	break;
		case 6: left_run(g_LeftSpeed, g_RightSpeed); break;
		case 7: right_run(g_LeftSpeed, g_RightSpeed); break;
		case 8: Buzzer = 0;	break;
		case 9: Buzzer = 1;	break;

		case 101: Tracking2(); 	break;	/*����ѭ��ģʽ*/
		case 102: IRAvoid();	break;	/*�������ģʽ*/
		case 103: IRFollow();	break;	/*�������ģʽ*/
		case 104: RICHBOT_CSB_Avoid(300);	break;	/*����+����������,�������������루���ף�*/
		case 105: RICHBOT_PTZ_Avoid(300);	break;	/*����+�����̨����,�������������루���ף�*/
		case 106: Track_Avoid(300);	break;	/*����ѭ��+���������ϣ��Զ��ܿ����ص�ѭ�����ϣ��������������루���ף�*/
		default:  stop();	 break;
	}
}

/*������*/     
void main(void)
{
	EA_on;								//�����ж�
	Timer0_Init();//��ʱ0��ʼ��
	Timer1_Init();//��ʱ0��ʼ��
	UartInit();						//���ڳ�ʼ��
	RICHBOT_ServoFront();//�����ǰ
	while(1)
	{
		if(TickTime > 10000)//���һ��ʱ���ϴ�ϵͳ����
		{
			PrintString1(PackSysData());	//UART1����ϵͳ����
			TickTime = 0;
		}
		BLE_Remote();                       //���ô��ڽ�������
	}	
}


/*********************************************
QX_A11����С��ǰ��
��ڲ�����LeftSpeed��RightSpeed
���ڲ���: ��
˵����LeftSpeed��RightSpeed�ֱ��������ҳ���ת��
**********************************************/
void forward(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//�����ٶ�
	left_motor_go; //����ǰ��
	right_motor_go; //�ҵ��ǰ��
}
/*С����ת*/
/*********************************************
QX_A11����С����ת
��ڲ�����LeftSpeed��RightSpeed
���ڲ���: ��
˵����LeftSpeed��RightSpeed�ֱ��������ҳ���ת��
**********************************************/
void left_run(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//�����ٶ�
	left_motor_stops; //����ͣ
	right_motor_go; //�ҵ��ǰ��	
}

/*********************************************
QX_A11����С����ת
��ڲ�����LeftSpeed��RightSpeed
���ڲ���: ��
˵����LeftSpeed��RightSpeed�ֱ��������ҳ���ת��
**********************************************/
void right_run(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//�����ٶ�
	right_motor_stops;//�ҵ��ͣ
	left_motor_go;    //����ǰ��
}
/*********************************************
QX_A11����С������
��ڲ�����LeftSpeed��RightSpeed
���ڲ���: ��
˵����LeftSpeed��RightSpeed�ֱ��������ҳ���ת��
**********************************************/
void back_run(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//�����ٶ�
	right_motor_back;//�ҵ������
	left_motor_back; //��������
}
/*********************************************
QX_A11����С��ԭ����ת
��ڲ�����LeftSpeed��RightSpeed
���ڲ���: ��
˵����LeftSpeed��RightSpeed�ֱ��������ҳ���ת��
**********************************************/
void SmartCarLeftTurn(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//�����ٶ�
	left_motor_back; //��������
	right_motor_go; //�ҵ��ǰ��	
}
/*********************************************
QX_A11����С��ԭ����ת
��ڲ�����LeftSpeed��RightSpeed
���ڲ���: ��
˵����LeftSpeed��RightSpeed�ֱ��������ҳ���ת��
**********************************************/
void SmartCarRightTurn(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//�����ٶ�
	right_motor_back;//�ҵ������
	left_motor_go;    //����ǰ��	
}
/*********************************************
QX_A11����С��ͣ��
��ڲ�������
���ڲ���: ��
˵����QX_A11����С��ͣ��
**********************************************/
void stop(void)
{
	left_motor_stops;
	right_motor_stops;
}
/*********************************************
QX_A11����С��PWM���
��ڲ�������
���ڲ���: ��
˵����װ��PWM���,�������ȫ�ֱ���pwm_left,pwm_right�ֱ�������������ߵ�ƽʱ��
**********************************************/
void QXMBOT_LoadPWM(void)
{
	if(pwm_left > PWM_HIGH_MAX)		pwm_left = PWM_HIGH_MAX;	//��������д��������ռ�ձ����ݣ���ǿ��Ϊ���ռ�ձȡ�
	if(pwm_left < PWM_HIGH_MIN)		pwm_left = PWM_HIGH_MIN;	//��������д��С����Сռ�ձ����ݣ���ǿ��Ϊ��Сռ�ձȡ�
	if(pwm_right > PWM_HIGH_MAX)	pwm_right = PWM_HIGH_MAX;	//��������д��������ռ�ձ����ݣ���ǿ��Ϊ���ռ�ձȡ�
	if(pwm_right < PWM_HIGH_MIN)	pwm_right = PWM_HIGH_MIN;	//��������д��С����Сռ�ձ����ݣ���ǿ��Ϊ��Сռ�ձȡ�
	if(pwm_val_left<=pwm_left)		LM_EN_Pin = 1;  //װ����PWM����ߵ�ƽʱ��
	else LM_EN_Pin = 0; 						    //װ����PWM����͵�ƽʱ��
	if(pwm_val_left>=PWM_DUTY)		pwm_val_left = 0;	//�����Ա�ֵ���ڵ������ռ�ձ����ݣ���Ϊ��

	if(pwm_val_right<=pwm_right)	RM_EN_Pin = 1; //װ����PWM����ߵ�ƽʱ��
	else RM_EN_Pin = 0; 							//װ����PWM����͵�ƽʱ��
	if(pwm_val_right>=PWM_DUTY)		pwm_val_right = 0;	//����ҶԱ�ֵ���ڵ������ռ�ձ����ݣ���Ϊ��
}
/*********************************************
QX_A11����С��ϵͳ���ݴ������
��ڲ�������
���ڲ���: SysDataBuffer
˵��������Ҫ���͵�����ϵͳ���ݴ��
**********************************************/
char *PackSysData(void)
{
	SysDataBUF_Define idata SysData;

	memset(SysData.SysDataBuffer, 0, sizeof(SysData.SysDataBuffer));	//��ʼ�����棬��������������

	
	strcat(SysData.SysDataBuffer, "$RICHBOT"); //���Ӱ�ͷ�ַ���
	#ifdef ULTRASOUND
	SysData.distance = QXMBOT_GetDistance();//��ȡ������������,��λ������
	strcat(SysData.SysDataBuffer, ",CSB"); //���Ӱ�ͷ�ַ���
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//��ʼ�����棬��������������
	sprintf(SysData.strBUF, "%03d", SysData.distance);		   //ת��Ϊ�ַ���
	strcat(SysData.SysDataBuffer, SysData.strBUF); //���Ӿ����ַ���
	#endif

	#ifdef VOLTAGE_SAMP
	strcat(SysData.SysDataBuffer, ",DC8.0");//���ӵ�ص�ѹ�ַ���
	#endif
	#ifdef IRAVOID
	strcat(SysData.SysDataBuffer, ",IRA");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//��ʼ�����棬��������������
	SysData.IRavoidSta = getIRAvoidSta();
	sprintf(SysData.strBUF, "%02d", SysData.IRavoidSta);		   //ת��Ϊ�ַ���
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	#endif
	#ifdef IRTRACK
	strcat(SysData.SysDataBuffer, ",IRT");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//��ʼ�����棬��������������
	SysData.IRtrackSta = getTrackSta();
	sprintf(SysData.strBUF, "%02d", SysData.IRtrackSta);		   //ת��Ϊ�ַ���
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	#endif
	#ifdef MOTOR_SPEED
	strcat(SysData.SysDataBuffer, ",LS");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//��ʼ�����棬��������������
	sprintf(SysData.strBUF, "%d", SysData.LeftMotorSpeed);		   //ת��Ϊ�ַ���
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	strcat(SysData.SysDataBuffer, ",RS");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//��ʼ�����棬��������������
	sprintf(SysData.strBUF, "%d", SysData.RightMotorSpeed);		   //ת��Ϊ�ַ���
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	#endif
	strcat(SysData.SysDataBuffer, ",Mode");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//��ʼ�����棬��������������
	sprintf(SysData.strBUF, "%d", (u16)SysMode);		   //�ϴ�ϵͳģʽ
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	
	strcat(SysData.SysDataBuffer, "#");//���Ӱ�β�ַ���
	
	return(SysData.SysDataBuffer);
}
/*����ѭ������*/
void Tracking()
{
	//��⵽���ߣ�����������ߵ�ƽ
	char data1;
	data1 = getTrackSta();//��ȡ������״̬
	if(data1 == 11)//�ں����ϣ�ǰ��
	{
		forward(120,120);//ǰ��
	}
	else
	{
	 	if(data1 == 10)//С��ƫ�ң���ת
		{
			SmartCarLeftTurn(80,160);//��ת
		}
		if(data1 == 1)//С��ƫ����ת
		{
			SmartCarRightTurn(160,80);//��ת
		}
		if(data1 == 0)//���ƫ���ƫ�ң���������
		{
			stop();	
		}
	}
}
/*����ѭ������2*/
void Tracking2()
{
	//��⵽���ߣ�����������ߵ�ƽ
	char data1;
	data1 = getTrackSta();//��ȡ������״̬
	if(data1 == 11)//�ں����ϣ�ǰ��
	{
		forward(150,150);//ǰ��
	}
	else
	{
	 	if(data1 == 1)//С��ƫ����ת
		{
			right_run(150,150);//��ת
		}
		if(data1 == 10)//С��ƫ�ң���ת
		{
			left_run(150,150);//��ת
		}
	}
}

/*�������*/
void IRFollow(void)
{
	char data1;
	data1 = getIRAvoidSta();//��ȡ������״̬
	
	if(data1 == 10)//�Ҽ�⵽���壬��ת����¼Ϊ״̬1
	{
		right_run(g_LeftSpeed,g_RightSpeed);//��ת	
	}
	if(data1 == 1)//���⵽���壬��ת����¼Ϊ״̬2
	{	
		left_run(g_LeftSpeed,g_RightSpeed);//��ת	
	}
	if(data1 == 0)//���Ҷ���⵽���壬ǰ��
	{
		forward(g_LeftSpeed,g_RightSpeed);//ǰ��				
	}
	else if(data1 == 11)//û��⵽����
	{
		stop();
	}
}
/*�������*/
void IRAvoid(void)
{
	char data1;
	data1 = getIRAvoidSta();//��ȡ������״̬
	if(data1 == 11)//û��⵽�ϰ��ǰ��
	{
		forward(g_LeftSpeed,g_RightSpeed);//ǰ��
	}
	else
	{
		Buzzer = 0;
		stop();//ͣ��
		Delay_Ms(20);//ͣ��ʱ��
		Buzzer = 1;
		if(data1 == 10)//�Ҽ�⵽�ϰ����ת����¼Ϊ״̬1
		{
			SmartCarLeftTurn(g_LeftSpeed,g_RightSpeed);//��ת		
		}
		if(data1 == 1)//���⵽�ϰ����ת����¼Ϊ״̬2
		{
			SmartCarRightTurn(g_LeftSpeed,g_RightSpeed);//��ת
		}
		if(data1 == 0)//���Ҷ���⵽�ϰ������
		{
			stop();//ͣ��
			Delay_Ms(100);//ִ��ͣ����ʱ��
			back_run(g_LeftSpeed,g_RightSpeed);//����
			Delay_Ms(200);//ִ�к��˵�ʱ��
			right_run(g_LeftSpeed,g_RightSpeed);//��ת��ͷ
			Delay_Ms(280);//ִ����ת��ʱ��				
		}
	}
}
/*�������2*/
void IR_Avoid2(unsigned char leftSpeed,rightSpeed)
{
	//Ϊ0 û��ʶ�𵽺��� Ϊ1ʶ�𵽺���
	char data1;
	data1 = getIRAvoidSta();//��ȡ������״̬
	if(data1 == 10)//�Ҽ�⵽�ϰ����ת����¼Ϊ״̬1
	{
			pwm_left = leftSpeed,pwm_right =  rightSpeed;//�����ٶ�
			left_motor_stops; //��������
			right_motor_go; //�ҵ��ǰ��	
	}
	if(data1 == 1)//���⵽�ϰ����ת����¼Ϊ״̬2
	{
			pwm_left = leftSpeed,pwm_right =  rightSpeed;//�����ٶ�
			left_motor_go; //��������
			right_motor_stops; //�ҵ��ǰ��	
	}
	if(data1 == 11)//û��⵽�ϰ��ǰ��
	{
		forward(leftSpeed,rightSpeed);
	}
}
/*********************************************
QX_A11����С����ȡ������ϴ�����״̬
��ڲ�������
���ڲ���: dat1���ر��ϴ�����״̬
˵����
**********************************************/
u8 getIRAvoidSta(void)
{
	//���⴫����IO��⵽�ϰ�������͵�ƽ
	char dat1 = L_AvoidSensorPin, dat2 = R_AvoidwSensorPin;
	dat1 = dat1*10+dat2;
	return(dat1);
}
/*********************************************
QX_A11����С����ȡ����ѭ��������״̬
��ڲ�������
���ڲ���: dat1����ѭ��������״̬
˵����
**********************************************/
u8 getTrackSta(void)
{
	//���⴫����IO��⵽��������͵��
	char dat1 = L_TrackSensorPin, dat2 = R_TrackSensorPin;
	dat1 = dat1*10+dat2;
	return(dat1);
}
/*====================================
������void Delay_Ms(INT16U ms)
������ms��������ʱ�β�
������12T 51��Ƭ������Ӧ��ʱ�Ӻ��뼶��ʱ����
====================================*/
void Delay_Ms(unsigned int ms)
{
     unsigned int i;
	 do{
	      i = MAIN_Fosc / 96000; 
		  while(--i);   //96T per loop
     }while(--ms);
}
/*���������ʱ����С����ǰ��*/
void Delay1550us()		//@11.0592MHz
{
	unsigned char i, j;

	i = 3;
	j = 196;
	do
	{
		while (--j);
	} while (--i);
}

void Delay18450us()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	i = 34;
	j = 16;
	do
	{
		while (--j);
	} while (--i);
}
/*���������ʱ��С���ҷ�*/
void Delay600us()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	i = 2;
	j = 15;
	do
	{
		while (--j);
	} while (--i);
}
void Delay19400us()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	i = 35;
	j = 197;
	do
	{
		while (--j);
	} while (--i);
}
/*���������ʱ����С����*/
void Delay17500us()		//@11.0592MHz
{
	unsigned char i, j;

	i = 32;
	j = 93;
	do
	{
		while (--j);
	} while (--i);
}
void Delay2500us()		//@11.0592MHz
{
	unsigned char i, j;

	i = 5;
	j = 120;
	do
	{
		while (--j);
	} while (--i);
}

//ð������
void RICHBOT_bubble(unsigned int *a,unsigned char n) /*�������������������׵�ַ�������С*/
{
	unsigned int i,j,temp;	
	for(i = 0;i < n-1; i++)	
	{	
		for(j = i + 1; j < n; j++) /*ע��ѭ����������*/
		{
			if(a[i] > a[j])
			{
				temp = a[i];		
				a[i] = a[j];		
				a[j] = temp;			
			}
		}
	}

}
/*����������*/
void RICHBOT_TrigUltrasonic()
{
	TrigPin = 0; //������ģ��Trig	���ƶ�
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	TrigPin = 1; //������ģ��Trig	���ƶ�
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	TrigPin = 0; //������ģ��Trig	���ƶ�
}
/*====================================
������	��RICHBOT_GetDistance
����	����
����ֵ	����ȡ���뵥λ����
����	�����������
ͨ�������źŵ��յ������źŵ�ʱ����Ծ���
��Ƭ������11.0592Mhz
ע��������Ϊ60ms����
====================================*/
unsigned int RICHBOT_GetDistance()
{
	unsigned int Distance = 0;	//����������
	unsigned int  Time=0;		//���ڴ�Ŷ�ʱ��ʱ��ֵ
	RICHBOT_TrigUltrasonic();	//����������
	while(!EchoPin);  	//�жϻ����ź��Ƿ�Ϊ�͵�ƽ
	Timer1On;			//������ʱ��1
	while(EchoPin);		//�ȴ��յ������ź�
	Timer1Off;			//�رն�ʱ��1
	Time=TH1*256+TL1;	//��ȡʱ��
	TH1=0;
	TL1=0;				//��ն�ʱ��
    Distance = (float)(Time*1.085)*0.17;//�������MM
	return(Distance);//���ؾ���				
}
/*====================================
������	��RICHBOT_RefreshDistance
����	����
����ֵ	������ð�������ľ���
����	������5�β�࣬ȥ�����ֵ����Сֵ��ȡ�м�3��ƽ��ֵ
���뵥λ������
====================================*/
unsigned int RICHBOT_RefreshDistance()
{
	unsigned char num;
	unsigned int Dist;
	for(num=0; num<5; num++)
	{
		DistBuf[num] = RICHBOT_GetDistance();
		Delay_Ms(60);//������ڲ�����60����	
	}
	RICHBOT_bubble(DistBuf, 5);//
	Dist = (DistBuf[1]+DistBuf[2]+DistBuf[3])/3; //ȥ��������Сȡ�м�ƽ��ֵ
	return(Dist);
}
/*=================================================
*�������ƣ�RICHBOT_ServoFront
*�������ܣ���̨��ǰת��
*���ã�
*���룺
=================================================*/
void RICHBOT_ServoFront()
{
	char i;
	EA_off;	//�ر��жϷ����Ӱ����ת��
	for(i=0;i<10;i++)
	{	
		ServoPin = 1;
		Delay1550us();
		ServoPin = 0;
		Delay18450us();
	}
	EA_on;	//���ж�
	Delay_Ms(100);
}
/*=================================================
*�������ƣ�RICHBOT_ServoLeft
*�������ܣ���̨����ת��
*���ã�
*���룺
=================================================*/
void RICHBOT_ServoLeft()
{
	char i;
	EA_off;	//�ر��жϷ����Ӱ����ת��
	for(i=0;i<10;i++)
	{	
		ServoPin = 1;
		Delay2500us();
		ServoPin = 0;
		Delay17500us();
	}
	EA_on;	//���ж�
	Delay_Ms(100);
}
/*=================================================
*�������ƣ�RICHBOT_ServoFront
*�������ܣ���̨����ת��
*���ã�
*���룺
=================================================*/
void RICHBOT_ServoRight()
{
	char i;
	EA_off;	//�ر��жϷ����Ӱ����ת��
	for(i=0;i<10;i++)
	{	
		ServoPin = 1;
		Delay600us();
		ServoPin = 0;
		Delay19400us();
	}
	EA_on;	//���ж�
	Delay_Ms(100);
}
/*====================================
������	��RICHBOT_CSB_Avoid
����	��val���ñ��ϴ�������
����ֵ	����
����	������С��������+�������
���뵥λ������
====================================*/
void  RICHBOT_CSB_Avoid(unsigned int val)
{
	unsigned int Dis;//�����ݴ����
	Dis = RICHBOT_GetDistance();//��ȡ������������,��λ������
	if((Dis > 30) && (Dis < val))
	{
		stop();//ͣ��
		Delay_Ms(100);//ִ��ͣ����ʱ��
		back_run(g_LeftSpeed,g_RightSpeed);//����
		Delay_Ms(200);//ִ�к��˵�ʱ��
		right_run(g_LeftSpeed,g_RightSpeed);//��ת��ͷ
		Delay_Ms(280);//ִ����ת��ʱ��	
	}
	else
	{
		for(Dis=0; Dis<1800; Dis++)//������ڲ�����60ms �������Dis������ѭ��
		{
			IR_Avoid2(150,150);//�������
		}	
	}			
}
/*====================================
������	��RICHBOT_PTZ_Avoid
����	��val���ñ��ϴ�������
����ֵ	����
����	������С�������̨����
���뵥λ������
====================================*/
void  RICHBOT_PTZ_Avoid(unsigned int val)
{
	unsigned int Dis;//�����ݴ����
	Dis = RICHBOT_GetDistance();//��ȡ������������,��λ������
	if((Dis > 30) && (Dis < val))
	{
		LED1=0,LED2=0,Buzzer=0;//LED1,2����������	
		stop();	//ͣ��
		Delay_Ms(100);
		LED1=1,LED2=1,Buzzer=1;//LED1,2Ϩ�𣬾���

		/*�����ת���*/
		RICHBOT_ServoLeft();
		LeftDistance = RICHBOT_RefreshDistance();

		/*�����ת���*/
		RICHBOT_ServoRight();
		RightDistance = RICHBOT_RefreshDistance();

		/*�����ǰ�����*/
		RICHBOT_ServoFront();
		FrontDistance = RICHBOT_RefreshDistance();
		if(LeftDistance > RightDistance)
		{
			LED1=1,LED2=0;//LED1��,2����
			stop();	//ͣ��
			Delay_Ms(100);
			left_run(200, 200);//��ת
			Delay_Ms(60);
			LED1=1,LED2=1;//LED1��,2����		
		}else if(RightDistance > LeftDistance)
		{
			LED1=0,LED2=1;//LED1��,2����
			stop();	//ͣ��
			Delay_Ms(100);
			right_run(200, 200);//��ת
			Delay_Ms(60);
			LED1=1,LED2=1;//LED1��,2����	
		}		
	}
	else
	{
		for(Dis=0; Dis<1800; Dis++)//������ڲ�����60ms �������Dis������ѭ��
		{
			IR_Avoid2(150,150);//�������
		}	
	}			
}
/*����ѭ��+���������϶���һ���Զ��ܿ��ϰ��ﲢ�ص�ѭ�����ϣ�*/
void Track_Avoid(unsigned int val)
{
	unsigned int Dis;//�����ݴ����
	Dis = RICHBOT_GetDistance();//��ȡ������������,��λ������
	if((Dis > 30) && (Dis < val))
	{
		Buzzer = 0;//ʹ�ܷ�����
		stop();//ͣ��
		Delay_Ms(100);//ͣ��ʱ��
		SmartCarRightTurn(180,180);//ԭ����ת
		Delay_Ms(80);//���ҽǶȣ���ֵԽ��ת��Ƕ�Խ��
		forward(150,150);//ǰ��
		Delay_Ms(10);//ǰ������
		do{
			forward(100,255);//��ת
			}while(L_TrackSensorPin == 0 || R_TrackSensorPin == 0); //�ص����������˳����������ԭ��ת��Ѱ�Һ���
		Buzzer = 1;//�رշ�����
		stop();//ͣ��
		Delay_Ms(10);//ͣ��ʱ��
		SmartCarRightTurn(180,180);//ԭ����ת
		Delay_Ms(30);
	}else {
		for(Dis=0; Dis<1800; Dis++)  //������ÿ�β����������65ms
		Tracking2();	//ѭ��
	}
	
}

/********************* Timer0��ʼ��************************/
void Timer0_Init(void)
{
	TMOD |= 0x02;//��ʱ��0��8λ�Զ���װģ��
	TH0 = 164;
	TL0 = 164;//11.0592M����12T���ʱ��Լ����100΢��
	TR0 = 1;//������ʱ��0
	ET0 = 1;//����ʱ��0�ж�	
}

/*���ڳ�ʼ��*/
void UartInit(void)		//9600bps@11.0592MHz
{
	SCON |= 0x50; 	// SCON: ģʽ1, 8-bit UART, ʹ�ܽ���
	T2CON |= 0x34; //���ö�ʱ��2Ϊ���ڲ����ʷ�������������ʱ��2
	TL2 = RCAP2L = (65536-(MAIN_Fosc/32/BAUD)); //���ò�����
	TH2 = RCAP2H = (65536-(MAIN_Fosc/32/BAUD)) >> 8;
	ES= 1; 			//�򿪴����ж�
	PrintString1("A11 Ready!\r\n");	//UART1����һ���ַ���
}
/********************* Timer0�жϺ���************************/
void timer0_ISR (void) interrupt 1
{
	 pwm_val_left++;
	 pwm_val_right++;
	 TickTime++;
	 QXMBOT_LoadPWM();//װ��PWM���
}
/*��ʱ��1��ʼ��*/
void Timer1_Init(void)		
{
	TMOD |= 0x10;	//��ʱ��1����ģʽ1,16λ��ʱģʽ��T1�ò�ECH0���峤��
	TH1 = 0;		   
    TL1 = 0;
	ET1 = 1;             //����T1�ж�
}
/* Timer1 interrupt routine */
void tm1_ISR() interrupt 3 using 1
{
	Timer1Overflow = 1;	//������1�����־λ
	EchoPin = 0;		//���������ն�	
}	
