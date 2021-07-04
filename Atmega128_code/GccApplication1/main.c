/*
 * GccApplication1.c
 *
 * Created: 2019-08-28 오후 12:13:16
 * Author : CDSL
 */ 

#include "mcu_init.h"
#include "dataType.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Model Constant Define
#define Ke		0.0683
#define Kt		0.0683


enum{
	CURRENT_CONTROL = 0x01,
	VELOCITY_CONTROL = 0x02,
	POSITION_CONTROL = 0x04
};


volatile int32_t g_Cnt, g_preCnt;
volatile int32_t dw_Cnt=0;

// Position Variables
volatile double g_Pdes = 0., g_Ppre;
volatile double g_Pcur, g_Pvcur;
volatile double g_Perr, g_Pverr;

// Velocity Variables
volatile double g_Vcur, g_Vpre;
volatile double g_Vdes = 0.2;
volatile double g_Verr;
volatile double g_Verr_sum;
volatile double g_err_sum_saturation;
volatile double g_Vlimit = 1.;

// Current Variables
volatile double g_Ccur;
volatile double g_Cdes;
volatile double g_Cerr;
volatile double g_Cerr_sum;
volatile double g_Climit = 1.;

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int cur_control = 0;
volatile double g_vel_control;
volatile double g_pos_control;
volatile unsigned char g_TimerCnt;
volatile unsigned char g_ControlMode = POSITION_CONTROL | VELOCITY_CONTROL | POSITION_CONTROL;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;
volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

// PID Gain Tuning Variables //
volatile double dt = 0.0005;
// Current Control Gain //
volatile double Kp_c = 0.8269;			// 0.0827;
volatile double Ki_c = 2.2117e+03;		// 2.2117e+03
//volatile double Kp_v = 0.0183;
volatile double Kp_v = 1.83;			// 0.183
volatile double Ki_v = 0.1;				// 4.8689;
volatile double Kp_p = 12.5664;			// 12.5664
volatile double Kd_p = 0.1;				// 0.1



void SetDutyCW(double v){
	
	while(TCNT1  == 0);

	int ocr = v * (200. / 24.) + 200;
	
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;
	//OCR1A = OCR1B = ocr;
	
	OCR1A = OCR3B = ocr + 8;		//1 H
	OCR1B = OCR3A = ocr - 8;		//1 L


}


void InitLS7366(){
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX |FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}



int getADC(char ch){

	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}




ISR(USART0_RX_vect){
	g_buf[g_BufWriteCnt++] =UDR0;
}




//ISR(TIMER3_OVF_vect){
ISR(TIMER0_OVF_vect){ // 0.5ms Overflow timer
			
	TCNT0 = 256 - 125;
			
	// Read LS7366 //
	
	int32_t cnt;
	
	PORTC = 0x01;
	//g_ADC = getADC(0);
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);
	PORTB = 0x01;
			
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();		cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;
	
	PORTC = 0x03;

	// Read Angle // 
	//=> g_Pcur : Radian으로 출력되는 것 확인 -> Degree :: g_Pcur / 2.0 / M_PI * 360.0
	g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI ;
	if((g_TimerCnt % 10) == 0){
		// Angular Velocity 
		g_Vcur = (g_Pcur - g_Pvcur) / (dt * 10.0);
		g_Pvcur = g_Pcur;
	}
	///////////////
	
	// Read Current //
	g_ADC = getADC(0);
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.488) * 10.);
	///////////////


	/////////////////////////////////////////
	//To Do
	/*
	static int flag = 0;
	if((++flag) == 8000){
		
		SetDutyCW(12.);
	}
	else if(flag == 16000){
		flag = 0;
		SetDutyCW(-12.);
	}*/

	// ** CONTROL ** //
	// Position Control
	if((g_TimerCnt % 100) == 0){
/*		g_Pdes = 1.5;*/
		g_Perr = g_Pdes - g_Pcur;
		g_pos_control = g_Perr * Kp_p + (g_Perr-g_Pverr) * Kd_p / (dt * 100);
		g_Pverr = g_Perr; 
		// Norminal Angluar Velocity Saturation // 6410 RPM -> 642 rad/s
		// Load가 없을 때의 Saturation이기 때문에 의미는 없음
		if(g_pos_control > 642.){ 
			g_pos_control = 642.;
		}
		else if(g_pos_control < -642.){
			g_pos_control = -642.;
		}
		// User Angluar Velocity Saturation 
		if(g_pos_control > g_Vlimit){
			g_pos_control = g_Vlimit;
		}
		else if(g_pos_control < -g_Vlimit){
			g_pos_control = -g_Vlimit;
		}
		g_TimerCnt = 0;
	}
	//////////////////
	// Velocity Control
	if((g_TimerCnt % 10) == 0){
		/*g_Vdes = g_pos_control;*/
		g_Vdes = g_pos_control;
		g_Verr = g_Vdes - g_Vcur;
		// Velocity PI Control
		g_vel_control = g_Verr * Kp_v + g_Verr_sum * Ki_v * (dt *10 ); // *10
		g_Verr_sum += g_Verr;


		// Norminal Current Saturation
// 		if(g_vel_control > 27.3){
// 			g_Verr_sum -= (g_vel_control - 27.3) * 1. / Kp_v / 3.;
// 			g_vel_control = 27.3;
// 		}
// 		else if(g_vel_control < -27.3){
// 			g_Verr_sum -= (g_vel_control + 27.3) * 1. / Kp_v / 3.;
// 			g_vel_control = -27.3;
// 		}
		if(g_vel_control > 2.08){
			g_Verr_sum -= (g_vel_control - 2.08) * 1. / Kp_v / 3.;
			g_vel_control = 2.08;
		}
		else if(g_vel_control < -2.08){
			g_Verr_sum -= (g_vel_control + 2.08) * 1. / Kp_v / 3.;
			g_vel_control = -2.08;
		}
		g_err_sum_saturation = g_Vdes;
		if(g_Verr_sum > g_err_sum_saturation) g_Verr_sum = g_err_sum_saturation;
		if(g_Verr_sum < -g_err_sum_saturation) g_Verr_sum = -g_err_sum_saturation;
		// User Current Saturation
		//g_Climit /= Ke;
		if(g_vel_control > g_Climit){
			g_vel_control = g_Climit;
		}
		else if(g_vel_control < -g_Climit){
			g_vel_control = -g_Climit;
		}
	}
	//////////////////
	// Current Control
	if((g_TimerCnt % 1) == 0){
			/*g_Cdes = g_vel_control;*/
			g_Cdes = g_vel_control;			
			g_Cerr = g_Cdes - g_Ccur;
			// Current PI Control // cur
			cur_control = g_Cerr * Kp_c + g_Cerr_sum * Ki_c * dt;
			// 전향 보상
			cur_control += g_Vcur * Ke;
			
			g_Cerr_sum += g_Cerr;
			// Norminal Voltage Saturation
			// I-term anti
			if(cur_control > 24){
				g_Cerr_sum -= (cur_control - 24.) * 1. / Kp_c / 3.;
				cur_control = 24;
			}
			else if(cur_control < -24){
				g_Cerr_sum -= (cur_control + 24.) * 1. / Kp_c / 3.;
				cur_control = -24;
			}
	}
	//////////////////

	g_TimerCnt++;
	/*SetDutyCW(cur_control);*/
	SetDutyCW(cur_control);
	/////////////////////////////////////////
	
	g_SendFlag++;

}



int main(void){
	
	Packet_t packet;
	packet.data.header[0] =	packet.data.header[1] =	packet.data.header[2] =	packet.data.header[3] = 0xFE;
	
	InitIO();
	
	//Uart
	InitUart0();
	
	//SPI
	InitSPI();
	
	//Timer
	InitTimer0();
	InitTimer1();
	InitTimer3();


	TCNT1 = TCNT3 = 0;
	SetDutyCW(0.);
	
	//ADC
	InitADC();
	
	//LS7366
	InitLS7366();
	
	//TCNT3 = 65536 - 125;
	TCNT0 = 256 - 125;
	sei();

	unsigned char check = 0;
	
    while (1) {
		for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){
			switch(g_PacketMode){
				case 0:
				if(g_buf[g_BufReadCnt] == 0xFF){
					checkSize++;
					if(checkSize == 4){
						g_PacketMode =1;
					}
				}
				else{
					checkSize = 0;
				}
				break;
				case 1:
				
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				
				if(checkSize == 8){
					
					if(g_PacketBuffer.data.id == g_ID){
						g_PacketMode = 2;
						
					}
					else{
						g_PacketMode = 0;
						checkSize = 0;
					}
					//OK
					
				}
				break;
				
				case 2:
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				
				if(checkSize == g_PacketBuffer.data.size){
					if(check == g_PacketBuffer.data.check){
						switch(g_PacketBuffer.data.mode){
							case 2:
							g_Pdes =g_PacketBuffer.data.pos /1000.0;
							g_Vlimit=g_PacketBuffer.data.velo /1000.0;
							g_Climit=g_PacketBuffer.data.cur /1000.0;
							PORTA &= 0xfe;
							//g_SendFlag = 20;
							break;
						}
					}
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
					
				}
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)){
					check = 0;
					g_PacketMode =0;
					checkSize = 0;
				}

			}
		}
		if(g_SendFlag > 19){
			PORTA &= 0xfc;
			g_SendFlag = 0;

			packet.data.id = g_ID;
			packet.data.size = sizeof(Packet_data_t);
			packet.data.mode = 3;
			packet.data.check = 0;
			
// 			packet.data.pos=g_Pdes * 1000;
// 			packet.data.velo=g_Vlimit * 1000;
// 			packet.data.cur=g_Climit * 1000;

			packet.data.pos=g_Pcur * 1000;
			packet.data.velo=g_Vcur * 1000;
			packet.data.cur=g_Ccur * 1000;		

			for(int i=8; i<sizeof(Packet_t); i++)
			packet.data.check += packet.buffer[i];

			for(int i=0; i<packet.data.size; i++){
				TransUart0(packet.buffer[i]);
			}
		}
	}
		
}

