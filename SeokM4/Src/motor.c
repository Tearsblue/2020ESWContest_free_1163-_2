#define MOTOR_H_
#include "motor.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"
#include <math.h>
#define MAX_I_TERM		(float)(5.0)
#define MIN_I_TERM		(float)(-5.0)


#define PWM_CONVERT		(double)(0.22222222)// pwm 50kHz

#define MAX_PID_OUT		(8399)
#define MIN_PID_OUT		(-8399)

#define TimeTick 		(float)(0.001)


#define PULSE_TO_DIS_L	(float)(0.8021087*3)//custom0.008184883203933 (2pir=376.99111836)/(encoder 1turn=470)
#define PULSE_TO_DIS_R	(float)(1.4361568)//custom0.008184883203933



// 바퀴지름 *PHI[75.398223686155037723103441198708]/(512*4)/기어비(4.8333333333333333333333333333333)
#define PULSE_TO_VEL_L	(float)((802.1087*1.2)) //dis/0.001
#define PULSE_TO_VEL_R	(float)((1436.1568))


// 바퀴지름 *PHI[75.398223686155037723103441198708]/(512*4)/기어비(4.8333333333333333333333333333333)/0.0005
//바퀴지름 25
//보정 부분
int ccr=0;
void InitMotor_L(  Motor_Val *pmotor)
{


	pmotor -> f_Kp = 0.85 ;	//4.5
	pmotor -> f_Ki =  0.0000;	//	0.00001
	pmotor -> f_Kd = 0.85;	//	4.0

	pmotor -> f_Accel = 2000;
	pmotor -> f_User_Velocity = 0.0;
	pmotor->f_Distace_Sum=0;
	pmotor->f_Tick_Distance=0;


	Angle.f_kp_roll=400;//1000;
	Angle.f_ki_roll=0.1;
	Angle.f_kd_roll=12000;//22030;
	Angle.f_kp_yaw=1;
	Angle.f_ki_yaw=1;
	Angle.f_kd_yaw=0;
	Angle.f_Next_angle_roll=2;

}

void InitMotor_R(  Motor_Val *pmotor)
{


	pmotor -> f_Kp = 0.85 ;	//4.5
	pmotor -> f_Ki =  0.0000;	//	0.00001
	pmotor -> f_Kd = 0.85;	//	4.0

	pmotor -> f_Accel = 8000;
	pmotor -> f_User_Velocity = 0.0;
	pmotor->f_Distace_Sum=0;
	pmotor->f_Tick_Distance=0;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance == TIM7)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	int z;
	HAL_ADC_Stop_DMA(&hadc1);
	//stopcputimer FIRST
	L_Motor.f_posadjrate=R_Motor.f_posadjrate=1;
	R_Motor.U16Qep_Sample = TIM2->CNT;//뒷바퀴
	L_Motor.U16Qep_Sample = TIM1->CNT;//모멘텀

//	RightQepRegs.QEPCTL.bit.SWI = 1;//initialize position counter
//	LeftQepRegs.QEPCTL.bit.SWI = 1;//initialize position counter

	R_Motor.i16QepVal = -1*(( R_Motor.U16Qep_Sample > 1024 )? -( R_Motor.U16Qep_Sample - 2049 ) : -( R_Motor.U16Qep_Sample ));
	L_Motor.i16QepVal = ( L_Motor.U16Qep_Sample > 1024 )? (L_Motor.U16Qep_Sample - 2049 ) : L_Motor.U16Qep_Sample;

	//R_Motor.i16QepVal = -1*( ( R_Motor.U16Qep_Sample > 1024 ) ? -( 2049 - R_Motor.U16Qep_Sample  ) : ( R_Motor.U16Qep_Sample ) );
	//L_Motor.i16QepVal = -1*( (L_Motor.U16Qep_Sample > 1024 ) ? ( 2049 -L_Motor.U16Qep_Sample ) : -(L_Motor.U16Qep_Sample ) );

	//엔코더초기화
	TIM2->CNT=0;   //뒷바퀴
	TIM1->CNT=0; //모멘텀
	//한 틱당 거리를 구한다.
	R_Motor.f_Tick_Distance = (R_Motor.i16QepVal)*PULSE_TO_DIS_R;
	L_Motor.f_Tick_Distance = (L_Motor.i16QepVal)*PULSE_TO_DIS_L;

	//틱당 거리를 합쳐 현재의 거리를 구한다.
	R_Motor.f_Distace_Sum +=  R_Motor.f_Tick_Distance;
	L_Motor.f_Distace_Sum +=  L_Motor.f_Tick_Distance;

	//사용자가 정해준 거리에 합친 거리를 제거해 남은 거리를 구한다.
	R_Motor.f_Remaning_Disatance = R_Motor.f_User_Distacne - R_Motor.f_Distace_Sum;
	L_Motor.f_Remaning_Disatance = L_Motor.f_User_Distacne - L_Motor.f_Distace_Sum;

	//펄스당 속도와 QEP를 곱해 현재의 속도를 구한다.
	R_Motor.f_Current_Velocity = R_Motor.i16QepVal * PULSE_TO_VEL_R;
	L_Motor.f_Current_Velocity = L_Motor.i16QepVal * PULSE_TO_VEL_L;
	//남은 거리 확인
	if((abs( R_Motor.f_Remaning_Disatance ) <= R_Motor.f_StopDistance ) && !( R_Motor.Stop_Flag ) )
	{
		R_Motor.f_User_Velocity = R_Motor.f_Decel_Velocity;

		if( R_Motor.f_Decel_Velocity == 0 )
			R_Motor.Stop_Flag = 1;
		else
			R_Motor.Stop_Flag = 2;
	}
		if((abs( L_Motor.f_Remaning_Disatance ) <= L_Motor.f_StopDistance ) && !( L_Motor.Stop_Flag ) )
		{
			L_Motor.f_User_Velocity = L_Motor.f_Decel_Velocity;

			if( L_Motor.f_Decel_Velocity == 0 )
				L_Motor.Stop_Flag = 1;
			else
				L_Motor.Stop_Flag = 2;
		}

	//가속

	if(R_Motor.f_User_Velocity > R_Motor.f_Next_Velocity)
	{
		R_Motor.f_Next_Velocity += TimeTick * R_Motor.f_Accel;
		if(R_Motor.f_User_Velocity < R_Motor.f_Next_Velocity)
			R_Motor.f_Next_Velocity = R_Motor.f_User_Velocity;
	}
	else if(R_Motor.f_User_Velocity < R_Motor.f_Next_Velocity)
	{
		R_Motor.f_Next_Velocity -= TimeTick * R_Motor.f_Accel;
		if(R_Motor.f_User_Velocity > R_Motor.f_Next_Velocity)
			R_Motor.f_Next_Velocity = R_Motor.f_User_Velocity;
	}

//	if(L_Motor.f_User_Velocity > L_Motor.f_Next_Velocity)
//	{
//		L_Motor.f_Next_Velocity += TimeTick * L_Motor.f_Accel;
//		if(L_Motor.f_User_Velocity < L_Motor.f_Next_Velocity)
//			L_Motor.f_Next_Velocity = L_Motor.f_User_Velocity;
//	}
//	else if(L_Motor.f_User_Velocity < L_Motor.f_Next_Velocity)
//	{
//		L_Motor.f_Next_Velocity -= TimeTick * L_Motor.f_Accel;
//		if(L_Motor.f_User_Velocity > L_Motor.f_Next_Velocity)
//			L_Motor.f_Next_Velocity = L_Motor.f_User_Velocity;
//	}



	R_Motor.f_Current_Velocity_temp[0]=R_Motor.f_Current_Velocity;
	L_Motor.f_Current_Velocity_temp[0]=L_Motor.f_Current_Velocity;
	R_Motor.f_Current_Velocity_av-=R_Motor.f_Current_Velocity_temp[99]/100;
	L_Motor.f_Current_Velocity_av-=L_Motor.f_Current_Velocity_temp[99]/100;
	for(z=99;z>0;z--)
		{
			R_Motor.f_Current_Velocity_temp[z]=R_Motor.f_Current_Velocity_temp[z-1];
			L_Motor.f_Current_Velocity_temp[z]=L_Motor.f_Current_Velocity_temp[z-1];
		}



	R_Motor.f_Current_Velocity_av+=R_Motor.f_Current_Velocity_temp[0]/100;
	L_Motor.f_Current_Velocity_av+=L_Motor.f_Current_Velocity_temp[0]/100;

	//현재 정지 중인지를 확인.
	if( ( R_Motor.Stop_Flag == 1 ) && ( L_Motor.Stop_Flag == 1 ) && ( R_Motor.f_Current_Velocity == 0 ) && ( L_Motor.f_Current_Velocity == 0 ) )
	{
		gStopcount++;
		if( gStopcount > 3 )
		{
			gMovestate = ON;
			gStopcount = 0;
		}
	}
	else
	{
		gStopcount = 0;
		gMovestate = OFF;
	}




	//각도 PID

	Angle.f_Current_angle_roll=f_roll-1;
	Angle.f_Current_angle_yaw=f_yaw;



	Angle.f_Err_angle_sum_roll-= Angle.f_Err_angle_roll[3];
	Angle.f_Err_angle_roll[3]  = Angle.f_Err_angle_roll[2];
	Angle.f_Err_angle_roll[2]  = Angle.f_Err_angle_roll[1];
	Angle.f_Err_angle_roll[1]  = Angle.f_Err_angle_roll[0];
	Angle.f_Err_angle_roll[0]  = Angle.f_Next_angle_roll-Angle.f_Current_angle_roll;
	Angle.f_Err_angle_sum_roll+= Angle.f_Err_angle_roll[0];

	Angle.f_proportionalterm_roll=Angle.f_kp_roll * Angle.f_Err_angle_roll[0];
	Angle.f_derivativeterm_roll=Angle.f_kd_roll * ((Angle.f_Err_angle_roll[0]-Angle.f_Err_angle_roll[3])-(Angle.f_Err_angle_roll[1]-Angle.f_Err_angle_roll[2]));
	Angle.f_integralterm_roll+=Angle.f_ki_roll * Angle.f_Err_angle_roll[0];
	Angle.f_pidoutterm_roll=(Angle.f_proportionalterm_roll+Angle.f_derivativeterm_roll+Angle.f_integralterm_roll);
	//Angle.f_pidoutterm_roll=fabs(sinf(f_pitch*0.017453))*(Angle.f_proportionalterm_roll+Angle.f_derivativeterm_roll+Angle.f_integralterm_roll);

	if(Angle.f_Err_angle_roll[3]*Angle.f_Err_angle_roll[0]<0)
	{
		L_Motor.f_Accel=0;
		Angle.f_proportionalterm_roll=0;
			Angle.f_derivativeterm_roll=0;
			Angle.f_integralterm_roll=0;
			Angle.f_pidoutterm_roll=0;
			Angle.f_Err_angle_sum_roll=0;
				Angle.f_Err_angle_roll[3]  = 0;
				Angle.f_Err_angle_roll[2]  = 0;
				Angle.f_Err_angle_roll[1]  = 0;
				Angle.f_Err_angle_roll[0]  = 0;
				Angle.f_Err_angle_sum_roll=0;
	}

	//if(f_roll>=0)
	//		L_Motor.f_User_Velocity=-2500;
	//	if(f_roll<0)
	//		L_Motor.f_User_Velocity=2500;
	//L_Motor.f_User_Velocity=(int)f_roll*(-1000);

	//L_Motor.f_User_Velocity=Angle.f_pidoutterm_roll;



	//L_Motor.i32Accel=abs((int)Angle.f_pidoutterm_roll);
	L_Motor.f_Accel=Angle.f_pidoutterm_roll;


	L_Motor.f_Next_Velocity -= TimeTick * L_Motor.f_Accel;
			if(L_Motor.f_Next_Velocity>2500)
				L_Motor.f_Next_Velocity=2500;
			if(L_Motor.f_Next_Velocity<-2500)
				L_Motor.f_Next_Velocity=-2500;

			Angle.f_Next_angle_roll=-L_Motor.f_Current_Velocity_av/450;
			if(Angle.f_Next_angle_roll>10)
				Angle.f_Next_angle_roll=10;
			if(Angle.f_Next_angle_roll<-10)
					Angle.f_Next_angle_roll=-10;




	//모터 PID
	R_Motor.f_ErrVelocitySum -= R_Motor.f_ErrVelocity[ 3 ];
	R_Motor.f_ErrVelocity[ 3 ]	= R_Motor.f_ErrVelocity[ 2 ];
	R_Motor.f_ErrVelocity[ 2 ]	= R_Motor.f_ErrVelocity[ 1 ];
	R_Motor.f_ErrVelocity[ 1 ]	= R_Motor.f_ErrVelocity[ 0 ];
	R_Motor.f_ErrVelocity[ 0 ]	= R_Motor.f_Next_Velocity - R_Motor.f_Current_Velocity;//R_Motor.q26posadjrate
	R_Motor.f_ErrVelocitySum += R_Motor.f_ErrVelocity[ 0 ];

	R_Motor.f_proportionalterm = R_Motor.f_Kp * R_Motor.f_ErrVelocity[ 0 ];
	R_Motor.f_derivativeterm = R_Motor.f_Kd * ( ( R_Motor.f_ErrVelocity[ 0 ] - R_Motor.f_ErrVelocity[ 3 ] ) +  ( R_Motor.f_ErrVelocity[ 1 ] - R_Motor.f_ErrVelocity[ 2 ] ) );
	R_Motor.f_integralterm =  R_Motor.f_Ki * R_Motor.f_ErrVelocitySum ;

	if( R_Motor.f_integralterm > MAX_I_TERM )
		R_Motor.f_integralterm = MAX_I_TERM;
	else if( R_Motor.f_integralterm < MIN_I_TERM )
		R_Motor.f_integralterm = MIN_I_TERM;

	R_Motor.f_pidoutterm += R_Motor.f_proportionalterm + R_Motor.f_derivativeterm + R_Motor.f_integralterm;


	L_Motor.f_ErrVelocitySum -= L_Motor.f_ErrVelocity[ 3 ];
	L_Motor.f_ErrVelocity[ 3 ]	= L_Motor.f_ErrVelocity[ 2 ];
	L_Motor.f_ErrVelocity[ 2 ]	= L_Motor.f_ErrVelocity[ 1 ];
	L_Motor.f_ErrVelocity[ 1 ]	= L_Motor.f_ErrVelocity[ 0 ];
	L_Motor.f_ErrVelocity[ 0 ]	= L_Motor.f_Next_Velocity - L_Motor.f_Current_Velocity;//
	L_Motor.f_ErrVelocitySum += L_Motor.f_ErrVelocity[ 0 ];

	L_Motor.f_proportionalterm = L_Motor.f_Kp * L_Motor.f_ErrVelocity[ 0 ];
	L_Motor.f_derivativeterm = L_Motor.f_Kd * ( ( L_Motor.f_ErrVelocity[ 0 ] - L_Motor.f_ErrVelocity[ 3 ] ) +  ( L_Motor.f_ErrVelocity[ 1 ] - L_Motor.f_ErrVelocity[ 2 ] ) );
	L_Motor.f_integralterm =  L_Motor.f_Ki * L_Motor.f_ErrVelocitySum ;

	if( L_Motor.f_integralterm > MAX_I_TERM )
		L_Motor.f_integralterm = MAX_I_TERM;
	else if( L_Motor.f_integralterm < MIN_I_TERM )
		L_Motor.f_integralterm = MIN_I_TERM;

	L_Motor.f_pidoutterm += L_Motor.f_proportionalterm + L_Motor.f_derivativeterm + L_Motor.f_integralterm;





	//R_Motor.f_pidoutterm+=1;
	//g_uint16_pwm_flag=1;
	if(g_uint16_pwm_flag == 1)
	{

		if( R_Motor.f_pidoutterm >= 0 )
		{
			if( R_Motor.f_pidoutterm > MAX_PID_OUT )
				R_Motor.f_pidoutterm = MAX_PID_OUT;

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);


			R_Motor.f_pidoutresult =  R_Motor.f_pidoutterm  ;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(uint32_t)R_Motor.f_pidoutresult);//PWM설정 최대 8400

		}
		else
		{
			if( R_Motor.f_pidoutterm < MIN_PID_OUT )
				R_Motor.f_pidoutterm = MIN_PID_OUT;


			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);



			R_Motor.f_pidoutresult = -1 * (R_Motor.f_pidoutterm);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(uint32_t)R_Motor.f_pidoutresult);//PWM설정 최대 8400



		}

		if( L_Motor.f_pidoutterm >= 0 )
		{
			if(L_Motor.f_pidoutterm > MAX_PID_OUT )
				L_Motor.f_pidoutterm = MAX_PID_OUT;

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);





			L_Motor.f_pidoutresult =  L_Motor.f_pidoutterm;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(uint32_t)L_Motor.f_pidoutresult);//PWM설정 최대 8400




		}
		else
		{
			if( L_Motor.f_pidoutterm < MIN_PID_OUT )
				L_Motor.f_pidoutterm = MIN_PID_OUT;

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);




			L_Motor.f_pidoutresult =  -1 * (L_Motor.f_pidoutterm);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(uint32_t)L_Motor.f_pidoutresult);//PWM설정 최대 8400



		}
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	}

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,g_servo_pulse);//서보모터 설정
	g_u16motortic++;
	R_Motor.U16Tick++;
	L_Motor.U16Tick++;
	gUserTimeCnt++;
	gsamplecount++;

//	if(gsamplecount>=50)
//	{
//		gsamplecount=0;
//		Angle.f_roll_sog-=Angle.f_roll_lifo[99];
//		for(int j=99;j>0;j--)
//			Angle.f_roll_lifo[j]=Angle.f_roll_lifo[j-1];
//		Angle.f_roll_lifo[0]=f_roll/100;
//		//Angle.f_roll_sog+=Angle.f_roll_lifo[0];
//	}

	//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,ccr);
	//ccr=3000;
	//if(ccr>TIM3->ARR) ccr=0;
	//CpuTimer2Regs.TCR.bit.TRB = 1;
	//PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	//StartCpuTimer0();// sensor int start -- sensor shoot
	HAL_ADC_Start_DMA(&hadc1,&adcval[0],3);
}
}
