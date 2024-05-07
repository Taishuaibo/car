
#include "UART.h"

bit Timer1Overflow;	//计数器1溢出标志位
unsigned char disbuff[4]={0,0,0,0};//用于分别存放距离的值米，厘米，毫米
unsigned int LeftDistance = 0, RightDistance = 0, FrontDistance = 0; //云台测距距离缓存
unsigned int DistBuf[5] = {0};//distance data buffer
unsigned int   TickTime = 0;//系统滴答计时
unsigned char  idata g_UartData;//单字节串口数据
unsigned char	pwm_val_left,pwm_val_right;	//中间变量，用户请勿修改。
unsigned char 	pwm_left,pwm_right;			//定义PWM输出高电平的时间的变量。用户操作PWM的变量。
unsigned char 	g_LeftSpeed = 120,g_RightSpeed = 120;//修改这里就能控制车速分别对应左右车速，取值范围60-200
enum e_SetMode SysMode = E_BleRemote;	//系统运行模式
#define		PWM_DUTY		255			//定义PWM的周期，数值为定时器0溢出周期，假如定时器溢出时间为100us，则PWM周期为20ms。
#define		PWM_HIGH_MIN	100			//限制PWM输出的最小占空比。用户请勿修改。
#define		PWM_HIGH_MAX	PWM_DUTY	//限制PWM输出的最大占空比。用户请勿修改。
#define 	IRAVOID 1  //定义开启红外避障功能
#define 	IRTRACK 1  //定义开启红外循迹功能

void Timer0_Init(void); //定时器0初始化
void Timer1_Init(void);//定时器1初始化
void UartInit(void);		//串口初始化9600bps@11.0592MHz
void Ext1_Init(void);  //外部中断1初始化，用于检测模式切换按键key状态
void QXMBOT_LoadPWM(void);//装入PWM输出值 
void Delay_Ms(unsigned int ms);//毫秒级延时函数
void forward(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11智能小车前进 
void left_run(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11智能小车左转  
void right_run(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11智能小车右转
void back_run(unsigned char LeftSpeed,unsigned char RightSpeed);//QX_A11智能小车后退
void SmartCarLeftTurn(unsigned char LeftSpeed, unsigned char RightSpeed);//QX_A11智能小车原地左转
void SmartCarRightTurn(unsigned char LeftSpeed, unsigned char RightSpeed);//QX_A11智能小车原地右转
void stop(void);//QX_A11智能小车停车
void Tracking(void);//黑线循迹
void Tracking2(void);//黑线循迹2
void Track_Avoid(unsigned int val);//红外循迹+超声波避障二合一
void IRAvoid(void);//红外避障
void IRFollow(void);//物体跟随

void Delay18450us(void);	//@11.0592MHz
void Delay1550us(void);		//@11.0592MHz
void Delay19400us(void);	//@11.0592MHz
void Delay600us(void);		//@11.0592MHz
void Delay17500us(void);	//@11.0592MHz
void Delay2500us(void);		//@11.0592MHz

void RICHBOT_bubble(unsigned int *a,unsigned char n);//冒泡排序
void RICHBOT_ServoFront(void);//舵机向前
void RICHBOT_ServoLeft(void);//舵机左转
void RICHBOT_ServoRight(void);//舵机右转
void RICHBOT_CSB_Avoid(unsigned int val);//超声波+红外避障
void  RICHBOT_PTZ_Avoid(unsigned int val);//舵机云台避障
unsigned int RICHBOT_GetDistance(void);//获取超声波距离
void RICHBOT_TrigUltrasonic(void);// 触发超声波
unsigned int RICHBOT_RefreshDistance(void);//超声波测距

void BLE_Remote(void);//蓝牙无线遥控
u8 getIRAvoidSta(void);//获取避障传感器状态
u8 getTrackSta(void);//获取循迹传感器状态
char *PackSysData(void);//打包系统数据

/*********************************************
QX_A11智能小车蓝牙遥控执行函数
入口参数：无
出口参数: 无
说明：通过接收到串口数据命令来执行相应功能
上位机发送数据格式为4位
第一位起始字符'$'
第二位功能字符'C'表示第三位数据为控制行走的数据，L表示第三位为左电机调速数据
第三位十六进制数据
第四位结束字符'#'
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

		case 101: Tracking2(); 	break;	/*红外循迹模式*/
		case 102: IRAvoid();	break;	/*红外避障模式*/
		case 103: IRFollow();	break;	/*物体跟随模式*/
		case 104: RICHBOT_CSB_Avoid(300);	break;	/*红外+超声波避障,参数：触发距离（毫米）*/
		case 105: RICHBOT_PTZ_Avoid(300);	break;	/*红外+舵机云台避障,参数：触发距离（毫米）*/
		case 106: Track_Avoid(300);	break;	/*黑线循迹+超声波避障（自动避开并回到循迹线上）参数：触发距离（毫米）*/
		default:  stop();	 break;
	}
}

/*主函数*/     
void main(void)
{
	EA_on;								//开总中断
	Timer0_Init();//定时0初始化
	Timer1_Init();//定时0初始化
	UartInit();						//串口初始化
	RICHBOT_ServoFront();//舵机向前
	while(1)
	{
		if(TickTime > 10000)//间隔一端时间上传系统数据
		{
			PrintString1(PackSysData());	//UART1发送系统数据
			TickTime = 0;
		}
		BLE_Remote();                       //调用串口解析函数
	}	
}


/*********************************************
QX_A11智能小车前进
入口参数：LeftSpeed，RightSpeed
出口参数: 无
说明：LeftSpeed，RightSpeed分别设置左右车轮转速
**********************************************/
void forward(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//设置速度
	left_motor_go; //左电机前进
	right_motor_go; //右电机前进
}
/*小车左转*/
/*********************************************
QX_A11智能小车左转
入口参数：LeftSpeed，RightSpeed
出口参数: 无
说明：LeftSpeed，RightSpeed分别设置左右车轮转速
**********************************************/
void left_run(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//设置速度
	left_motor_stops; //左电机停
	right_motor_go; //右电机前进	
}

/*********************************************
QX_A11智能小车右转
入口参数：LeftSpeed，RightSpeed
出口参数: 无
说明：LeftSpeed，RightSpeed分别设置左右车轮转速
**********************************************/
void right_run(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//设置速度
	right_motor_stops;//右电机停
	left_motor_go;    //左电机前进
}
/*********************************************
QX_A11智能小车后退
入口参数：LeftSpeed，RightSpeed
出口参数: 无
说明：LeftSpeed，RightSpeed分别设置左右车轮转速
**********************************************/
void back_run(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//设置速度
	right_motor_back;//右电机后退
	left_motor_back; //左电机后退
}
/*********************************************
QX_A11智能小车原地左转
入口参数：LeftSpeed，RightSpeed
出口参数: 无
说明：LeftSpeed，RightSpeed分别设置左右车轮转速
**********************************************/
void SmartCarLeftTurn(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//设置速度
	left_motor_back; //左电机后退
	right_motor_go; //右电机前进	
}
/*********************************************
QX_A11智能小车原地右转
入口参数：LeftSpeed，RightSpeed
出口参数: 无
说明：LeftSpeed，RightSpeed分别设置左右车轮转速
**********************************************/
void SmartCarRightTurn(unsigned char LeftSpeed,unsigned char RightSpeed)
{
	pwm_left = LeftSpeed,pwm_right =  RightSpeed;//设置速度
	right_motor_back;//右电机后退
	left_motor_go;    //左电机前进	
}
/*********************************************
QX_A11智能小车停车
入口参数：无
出口参数: 无
说明：QX_A11智能小车停车
**********************************************/
void stop(void)
{
	left_motor_stops;
	right_motor_stops;
}
/*********************************************
QX_A11智能小车PWM输出
入口参数：无
出口参数: 无
说明：装载PWM输出,如果设置全局变量pwm_left,pwm_right分别配置左右输出高电平时间
**********************************************/
void QXMBOT_LoadPWM(void)
{
	if(pwm_left > PWM_HIGH_MAX)		pwm_left = PWM_HIGH_MAX;	//如果左输出写入大于最大占空比数据，则强制为最大占空比。
	if(pwm_left < PWM_HIGH_MIN)		pwm_left = PWM_HIGH_MIN;	//如果左输出写入小于最小占空比数据，则强制为最小占空比。
	if(pwm_right > PWM_HIGH_MAX)	pwm_right = PWM_HIGH_MAX;	//如果右输出写入大于最大占空比数据，则强制为最大占空比。
	if(pwm_right < PWM_HIGH_MIN)	pwm_right = PWM_HIGH_MIN;	//如果右输出写入小于最小占空比数据，则强制为最小占空比。
	if(pwm_val_left<=pwm_left)		LM_EN_Pin = 1;  //装载左PWM输出高电平时间
	else LM_EN_Pin = 0; 						    //装载左PWM输出低电平时间
	if(pwm_val_left>=PWM_DUTY)		pwm_val_left = 0;	//如果左对比值大于等于最大占空比数据，则为零

	if(pwm_val_right<=pwm_right)	RM_EN_Pin = 1; //装载右PWM输出高电平时间
	else RM_EN_Pin = 0; 							//装载右PWM输出低电平时间
	if(pwm_val_right>=PWM_DUTY)		pwm_val_right = 0;	//如果右对比值大于等于最大占空比数据，则为零
}
/*********************************************
QX_A11智能小车系统数据打包函数
入口参数：无
出口参数: SysDataBuffer
说明：把需要发送的所有系统数据打包
**********************************************/
char *PackSysData(void)
{
	SysDataBUF_Define idata SysData;

	memset(SysData.SysDataBuffer, 0, sizeof(SysData.SysDataBuffer));	//初始化缓存，把所有数据清零

	
	strcat(SysData.SysDataBuffer, "$RICHBOT"); //连接包头字符串
	#ifdef ULTRASOUND
	SysData.distance = QXMBOT_GetDistance();//获取超声波测距距离,单位：毫米
	strcat(SysData.SysDataBuffer, ",CSB"); //连接包头字符串
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//初始化缓存，把所有数据清零
	sprintf(SysData.strBUF, "%03d", SysData.distance);		   //转换为字符串
	strcat(SysData.SysDataBuffer, SysData.strBUF); //连接距离字符串
	#endif

	#ifdef VOLTAGE_SAMP
	strcat(SysData.SysDataBuffer, ",DC8.0");//连接电池电压字符串
	#endif
	#ifdef IRAVOID
	strcat(SysData.SysDataBuffer, ",IRA");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//初始化缓存，把所有数据清零
	SysData.IRavoidSta = getIRAvoidSta();
	sprintf(SysData.strBUF, "%02d", SysData.IRavoidSta);		   //转换为字符串
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	#endif
	#ifdef IRTRACK
	strcat(SysData.SysDataBuffer, ",IRT");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//初始化缓存，把所有数据清零
	SysData.IRtrackSta = getTrackSta();
	sprintf(SysData.strBUF, "%02d", SysData.IRtrackSta);		   //转换为字符串
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	#endif
	#ifdef MOTOR_SPEED
	strcat(SysData.SysDataBuffer, ",LS");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//初始化缓存，把所有数据清零
	sprintf(SysData.strBUF, "%d", SysData.LeftMotorSpeed);		   //转换为字符串
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	strcat(SysData.SysDataBuffer, ",RS");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//初始化缓存，把所有数据清零
	sprintf(SysData.strBUF, "%d", SysData.RightMotorSpeed);		   //转换为字符串
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	#endif
	strcat(SysData.SysDataBuffer, ",Mode");
	memset(SysData.strBUF, 0, sizeof(SysData.strBUF));	//初始化缓存，把所有数据清零
	sprintf(SysData.strBUF, "%d", (u16)SysMode);		   //上传系统模式
	strcat(SysData.SysDataBuffer, SysData.strBUF);
	
	strcat(SysData.SysDataBuffer, "#");//连接包尾字符串
	
	return(SysData.SysDataBuffer);
}
/*黑线循迹函数*/
void Tracking()
{
	//检测到黑线，传感器输出高电平
	char data1;
	data1 = getTrackSta();//获取传感器状态
	if(data1 == 11)//在黑线上，前进
	{
		forward(120,120);//前进
	}
	else
	{
	 	if(data1 == 10)//小幅偏右，左转
		{
			SmartCarLeftTurn(80,160);//左转
		}
		if(data1 == 1)//小幅偏左，右转
		{
			SmartCarRightTurn(160,80);//右转
		}
		if(data1 == 0)//大幅偏左或偏右，已脱离轨道
		{
			stop();	
		}
	}
}
/*黑线循迹函数2*/
void Tracking2()
{
	//检测到黑线，传感器输出高电平
	char data1;
	data1 = getTrackSta();//获取传感器状态
	if(data1 == 11)//在黑线上，前进
	{
		forward(150,150);//前进
	}
	else
	{
	 	if(data1 == 1)//小幅偏左，右转
		{
			right_run(150,150);//右转
		}
		if(data1 == 10)//小幅偏右，左转
		{
			left_run(150,150);//左转
		}
	}
}

/*红外跟随*/
void IRFollow(void)
{
	char data1;
	data1 = getIRAvoidSta();//获取传感器状态
	
	if(data1 == 10)//右检测到物体，左转，记录为状态1
	{
		right_run(g_LeftSpeed,g_RightSpeed);//右转	
	}
	if(data1 == 1)//左检测到物体，右转，记录为状态2
	{	
		left_run(g_LeftSpeed,g_RightSpeed);//左转	
	}
	if(data1 == 0)//左右都检测到物体，前进
	{
		forward(g_LeftSpeed,g_RightSpeed);//前进				
	}
	else if(data1 == 11)//没检测到物体
	{
		stop();
	}
}
/*红外避障*/
void IRAvoid(void)
{
	char data1;
	data1 = getIRAvoidSta();//获取传感器状态
	if(data1 == 11)//没检测到障碍物，前进
	{
		forward(g_LeftSpeed,g_RightSpeed);//前进
	}
	else
	{
		Buzzer = 0;
		stop();//停车
		Delay_Ms(20);//停车时间
		Buzzer = 1;
		if(data1 == 10)//右检测到障碍物，左转，记录为状态1
		{
			SmartCarLeftTurn(g_LeftSpeed,g_RightSpeed);//左转		
		}
		if(data1 == 1)//左检测到障碍物，右转，记录为状态2
		{
			SmartCarRightTurn(g_LeftSpeed,g_RightSpeed);//右转
		}
		if(data1 == 0)//左右都检测到障碍物，后退
		{
			stop();//停车
			Delay_Ms(100);//执行停车的时间
			back_run(g_LeftSpeed,g_RightSpeed);//后退
			Delay_Ms(200);//执行后退的时间
			right_run(g_LeftSpeed,g_RightSpeed);//右转掉头
			Delay_Ms(280);//执行右转的时间				
		}
	}
}
/*红外避障2*/
void IR_Avoid2(unsigned char leftSpeed,rightSpeed)
{
	//为0 没有识别到黑线 为1识别到黑线
	char data1;
	data1 = getIRAvoidSta();//获取传感器状态
	if(data1 == 10)//右检测到障碍物，左转，记录为状态1
	{
			pwm_left = leftSpeed,pwm_right =  rightSpeed;//设置速度
			left_motor_stops; //左电机后退
			right_motor_go; //右电机前进	
	}
	if(data1 == 1)//左检测到障碍物，右转，记录为状态2
	{
			pwm_left = leftSpeed,pwm_right =  rightSpeed;//设置速度
			left_motor_go; //左电机后退
			right_motor_stops; //右电机前进	
	}
	if(data1 == 11)//没检测到障碍物，前进
	{
		forward(leftSpeed,rightSpeed);
	}
}
/*********************************************
QX_A11智能小车获取红外避障传感器状态
入口参数：无
出口参数: dat1返回避障传感器状态
说明：
**********************************************/
u8 getIRAvoidSta(void)
{
	//红外传感器IO检测到障碍物输出低电平
	char dat1 = L_AvoidSensorPin, dat2 = R_AvoidwSensorPin;
	dat1 = dat1*10+dat2;
	return(dat1);
}
/*********************************************
QX_A11智能小车获取红外循迹传感器状态
入口参数：无
出口参数: dat1返回循迹传感器状态
说明：
**********************************************/
u8 getTrackSta(void)
{
	//红外传感器IO检测到黑线输出低电高
	char dat1 = L_TrackSensorPin, dat2 = R_TrackSensorPin;
	dat1 = dat1*10+dat2;
	return(dat1);
}
/*====================================
函数：void Delay_Ms(INT16U ms)
参数：ms，毫秒延时形参
描述：12T 51单片机自适应主时钟毫秒级延时函数
====================================*/
void Delay_Ms(unsigned int ms)
{
     unsigned int i;
	 do{
	      i = MAIN_Fosc / 96000; 
		  while(--i);   //96T per loop
     }while(--ms);
}
/*舵机方波延时朝向小车正前方*/
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
/*舵机方波延时向小车右方*/
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
/*舵机方波延时朝向小车左方*/
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

//冒泡排序
void RICHBOT_bubble(unsigned int *a,unsigned char n) /*定义两个参数：数组首地址与数组大小*/
{
	unsigned int i,j,temp;	
	for(i = 0;i < n-1; i++)	
	{	
		for(j = i + 1; j < n; j++) /*注意循环的上下限*/
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
/*超声波触发*/
void RICHBOT_TrigUltrasonic()
{
	TrigPin = 0; //超声波模块Trig	控制端
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	TrigPin = 1; //超声波模块Trig	控制端
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	_nop_();_nop_();_nop_();_nop_();_nop_();
	TrigPin = 0; //超声波模块Trig	控制端
}
/*====================================
函数名	：RICHBOT_GetDistance
参数	：无
返回值	：获取距离单位毫米
描述	：超声波测距
通过发射信号到收到回响信号的时间测试距离
单片机晶振11.0592Mhz
注意测距周期为60ms以上
====================================*/
unsigned int RICHBOT_GetDistance()
{
	unsigned int Distance = 0;	//超声波距离
	unsigned int  Time=0;		//用于存放定时器时间值
	RICHBOT_TrigUltrasonic();	//超声波触发
	while(!EchoPin);  	//判断回响信号是否为低电平
	Timer1On;			//启动定时器1
	while(EchoPin);		//等待收到回响信号
	Timer1Off;			//关闭定时器1
	Time=TH1*256+TL1;	//读取时间
	TH1=0;
	TL1=0;				//清空定时器
    Distance = (float)(Time*1.085)*0.17;//算出来是MM
	return(Distance);//返回距离				
}
/*====================================
函数名	：RICHBOT_RefreshDistance
参数	：无
返回值	：经过冒泡排序后的距离
描述	：经过5次测距，去除最大值和最小值，取中间3次平均值
距离单位：毫米
====================================*/
unsigned int RICHBOT_RefreshDistance()
{
	unsigned char num;
	unsigned int Dist;
	for(num=0; num<5; num++)
	{
		DistBuf[num] = RICHBOT_GetDistance();
		Delay_Ms(60);//测距周期不低于60毫秒	
	}
	RICHBOT_bubble(DistBuf, 5);//
	Dist = (DistBuf[1]+DistBuf[2]+DistBuf[3])/3; //去掉最大和最小取中间平均值
	return(Dist);
}
/*=================================================
*函数名称：RICHBOT_ServoFront
*函数功能：云台向前转动
*调用：
*输入：
=================================================*/
void RICHBOT_ServoFront()
{
	char i;
	EA_off;	//关闭中断否则会影响舵机转向
	for(i=0;i<10;i++)
	{	
		ServoPin = 1;
		Delay1550us();
		ServoPin = 0;
		Delay18450us();
	}
	EA_on;	//开中断
	Delay_Ms(100);
}
/*=================================================
*函数名称：RICHBOT_ServoLeft
*函数功能：云台向左转动
*调用：
*输入：
=================================================*/
void RICHBOT_ServoLeft()
{
	char i;
	EA_off;	//关闭中断否则会影响舵机转向
	for(i=0;i<10;i++)
	{	
		ServoPin = 1;
		Delay2500us();
		ServoPin = 0;
		Delay17500us();
	}
	EA_on;	//开中断
	Delay_Ms(100);
}
/*=================================================
*函数名称：RICHBOT_ServoFront
*函数功能：云台向右转动
*调用：
*输入：
=================================================*/
void RICHBOT_ServoRight()
{
	char i;
	EA_off;	//关闭中断否则会影响舵机转向
	for(i=0;i<10;i++)
	{	
		ServoPin = 1;
		Delay600us();
		ServoPin = 0;
		Delay19400us();
	}
	EA_on;	//开中断
	Delay_Ms(100);
}
/*====================================
函数名	：RICHBOT_CSB_Avoid
参数	：val设置避障触发距离
返回值	：无
描述	：智能小车超声波+红外避障
距离单位：毫米
====================================*/
void  RICHBOT_CSB_Avoid(unsigned int val)
{
	unsigned int Dis;//距离暂存变量
	Dis = RICHBOT_GetDistance();//获取超声波测距距离,单位：毫米
	if((Dis > 30) && (Dis < val))
	{
		stop();//停车
		Delay_Ms(100);//执行停车的时间
		back_run(g_LeftSpeed,g_RightSpeed);//后退
		Delay_Ms(200);//执行后退的时间
		right_run(g_LeftSpeed,g_RightSpeed);//右转掉头
		Delay_Ms(280);//执行右转的时间	
	}
	else
	{
		for(Dis=0; Dis<1800; Dis++)//测距周期不低于60ms 这里借用Dis变量做循环
		{
			IR_Avoid2(150,150);//红外避障
		}	
	}			
}
/*====================================
函数名	：RICHBOT_PTZ_Avoid
参数	：val设置避障触发距离
返回值	：无
描述	：智能小车舵机云台避障
距离单位：毫米
====================================*/
void  RICHBOT_PTZ_Avoid(unsigned int val)
{
	unsigned int Dis;//距离暂存变量
	Dis = RICHBOT_GetDistance();//获取超声波测距距离,单位：毫米
	if((Dis > 30) && (Dis < val))
	{
		LED1=0,LED2=0,Buzzer=0;//LED1,2点亮，鸣笛	
		stop();	//停车
		Delay_Ms(100);
		LED1=1,LED2=1,Buzzer=1;//LED1,2熄灭，静音

		/*舵机左转测距*/
		RICHBOT_ServoLeft();
		LeftDistance = RICHBOT_RefreshDistance();

		/*舵机右转测距*/
		RICHBOT_ServoRight();
		RightDistance = RICHBOT_RefreshDistance();

		/*舵机正前方测距*/
		RICHBOT_ServoFront();
		FrontDistance = RICHBOT_RefreshDistance();
		if(LeftDistance > RightDistance)
		{
			LED1=1,LED2=0;//LED1灭,2点亮
			stop();	//停车
			Delay_Ms(100);
			left_run(200, 200);//左转
			Delay_Ms(60);
			LED1=1,LED2=1;//LED1灭,2点灭		
		}else if(RightDistance > LeftDistance)
		{
			LED1=0,LED2=1;//LED1亮,2点灭
			stop();	//停车
			Delay_Ms(100);
			right_run(200, 200);//右转
			Delay_Ms(60);
			LED1=1,LED2=1;//LED1灭,2点灭	
		}		
	}
	else
	{
		for(Dis=0; Dis<1800; Dis++)//测距周期不低于60ms 这里借用Dis变量做循环
		{
			IR_Avoid2(150,150);//红外避障
		}	
	}			
}
/*红外循迹+超声波避障二合一（自动避开障碍物并回到循迹线上）*/
void Track_Avoid(unsigned int val)
{
	unsigned int Dis;//距离暂存变量
	Dis = RICHBOT_GetDistance();//获取超声波测距距离,单位：毫米
	if((Dis > 30) && (Dis < val))
	{
		Buzzer = 0;//使能蜂鸣器
		stop();//停车
		Delay_Ms(100);//停车时间
		SmartCarRightTurn(180,180);//原地右转
		Delay_Ms(80);//左右角度，数值越大转向角度越大
		forward(150,150);//前进
		Delay_Ms(10);//前进距离
		do{
			forward(100,255);//左转
			}while(L_TrackSensorPin == 0 || R_TrackSensorPin == 0); //回到黑线上则退出，否则继续原地转向寻找黑线
		Buzzer = 1;//关闭蜂鸣器
		stop();//停车
		Delay_Ms(10);//停车时间
		SmartCarRightTurn(180,180);//原地右转
		Delay_Ms(30);
	}else {
		for(Dis=0; Dis<1800; Dis++)  //超声波每次测距间隔不低于65ms
		Tracking2();	//循迹
	}
	
}

/********************* Timer0初始化************************/
void Timer0_Init(void)
{
	TMOD |= 0x02;//定时器0，8位自动重装模块
	TH0 = 164;
	TL0 = 164;//11.0592M晶振，12T溢出时间约等于100微秒
	TR0 = 1;//启动定时器0
	ET0 = 1;//允许定时器0中断	
}

/*串口初始化*/
void UartInit(void)		//9600bps@11.0592MHz
{
	SCON |= 0x50; 	// SCON: 模式1, 8-bit UART, 使能接收
	T2CON |= 0x34; //设置定时器2为串口波特率发生器并启动定时器2
	TL2 = RCAP2L = (65536-(MAIN_Fosc/32/BAUD)); //设置波特率
	TH2 = RCAP2H = (65536-(MAIN_Fosc/32/BAUD)) >> 8;
	ES= 1; 			//打开串口中断
	PrintString1("A11 Ready!\r\n");	//UART1发送一个字符串
}
/********************* Timer0中断函数************************/
void timer0_ISR (void) interrupt 1
{
	 pwm_val_left++;
	 pwm_val_right++;
	 TickTime++;
	 QXMBOT_LoadPWM();//装载PWM输出
}
/*定时器1初始化*/
void Timer1_Init(void)		
{
	TMOD |= 0x10;	//定时器1工作模式1,16位定时模式。T1用测ECH0脉冲长度
	TH1 = 0;		   
    TL1 = 0;
	ET1 = 1;             //允许T1中断
}
/* Timer1 interrupt routine */
void tm1_ISR() interrupt 3 using 1
{
	Timer1Overflow = 1;	//计数器1溢出标志位
	EchoPin = 0;		//超声波接收端	
}	
