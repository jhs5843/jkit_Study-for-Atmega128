#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define FND_NUM0 0x3f
#define FND_NUM1 0x06
#define FND_NUM2 0x5b
#define FND_NUM3 0x4f
#define FND_NUM4 0x66
#define FND_NUM5 0x6d
#define FND_NUM6 0x7d
#define FND_NUM7 0x27
#define FND_NUM8 0x7f
#define FND_NUM9 0x6f

#define FND_SEL1 0x01
#define FND_SEL2 0x02
#define FND_SEL3 0x04
#define FND_SEL4 0x08

#define I2C_SCL PD0
#define I2C_SDA PD1

unsigned int fnd[10]= {FND_NUM0,FND_NUM1,FND_NUM2,FND_NUM3,FND_NUM4,FND_NUM5,FND_NUM6,FND_NUM7,FND_NUM8,FND_NUM9};
unsigned int fnd_sel[4] = {FND_SEL1,FND_SEL2,FND_SEL3,FND_SEL4};
unsigned int count=0, sec=0;

char textBuff[100];
unsigned int extCount=0;

void init_led();
void init_buzzer();
void init_fnd();
void init_switch();
void init_timer();
void init_serial();
void init_adc();
void init_extSwitch();
void init_i2c();

/* 미구현
void init_pwmDimmer();//pwm dimmer
//void init_i2c();//i2c 온도센서
void init_pwmMotor(); //pwm motor
void init_pwmBuzzer(); //pwm buzzer
void init_INTSerial(); // uart인터럽트
*/

void fnd_display(int num);
void SendByte(char data);
void SendLine(char *string);
char ReceiveByte();
uint16_t read_adc(uint8_t channel);
void I2C_start();
void I2C_transmit(uint8_t data);
uint8_t I2C_receive_ACK();
uint8_t I2C_receive_NACK();
void I2C_stop();

void test_led();
void test_buzzer();
void test_fnd();
void test_switch();
void test_serial();
void test_timer();
void test_adc();
void test_extSwitch();


void init_led()
{
	DDRA = 0xff;
	PORTA = 0x00;
}
void test_led()
{
	PORTA = 0x00;
	_delay_ms(1000);
	PORTA = 0xff;
	_delay_ms(1000);
}
void init_buzzer()
{
	DDRB = 0x10;
}
void test_buzzer()
{
	PORTB = 0x10;
	_delay_ms(10);
	PORTB = 0x10;
	_delay_ms(10);
}
void init_fnd(){
	DDRC = 0xff;
	DDRG = 0x00;
}
void test_fnd()
{
	fnd_display(1234);
}
void fnd_display(int num)
{
	int fnd1,fnd2,fnd3,fnd4;
	fnd1 = num%10;
	fnd2 = (num/10)%10;
	fnd3 = (num/100)%10;
	fnd4 = num/1000;

	PORTC = fnd[fnd1];
	PORTG = fnd_sel[0];
	_delay_ms(1);
	PORTC = fnd[fnd2];
	PORTG = fnd_sel[1];
	_delay_ms(1);
	PORTC = fnd[fnd3];
	PORTG = fnd_sel[2];
	_delay_ms(1);
	PORTC = fnd[fnd4];
	PORTG = fnd_sel[3];
	_delay_ms(1);
}
void init_switch()
{
	// pull-up
	DDRE = 0x00;
	PORTE = 0x30;
}
void test_switch()
{
	init_led();
	if((PINE & 0x20)== 0x00)
	PORTA = 0xff;
	if((PINE & 0x10)== 0x00)
	PORTA = 0x00;
}
void init_timer()
{
	TIMSK = 0x01; // R/W 선택 TIMER 0 사용
	TCCR0 = 0x04; // 분주비 64
	TCNT0 =256-5; // 0에서 시작 255가되어 256이 되면 OVF가 되어 인터럽트 구문을 실행한다.
	/*
	Timer set
	1/16000000 = 0.0000000625
	64분주
	0.0000000625* 64 = 0.000004
	0부터 250회 돌면 0.000004 *250 = 0.001
	OVF 발생 count 증가
	1000번 발생하면 1초 가 된다.
	*/
}
void test_timer()
{
	init_fnd();
	init_serial();
	fnd_display(sec);
	sprintf(textBuff,"%d\r\n",sec);
	SendLine(textBuff);
}
void init_serial(long BaudRate)
{
	UBRR0H = 0;
	switch(BaudRate)
	{
		case 115200:
		UBRR0L = 8;
		break;
		
		case 57600:
		UBRR0L = 16;
		break;
		
		case 38400:
		UBRR0L = 25;
		break;
		
		case 19200:
		UBRR0L = 51;
		break;
		
		case 14400:
		UBRR0L = 68;
		break;
		
		case 9600:
		UBRR0L = 103;
		break;
		
		// Default 115200
		default:
		UBRR0L = 8;
		break;
	}
	UCSR0A = 0x00;
	UCSR0B = 0x18;
	UCSR0C = 0x06;  // 8 bit
}
void SendByte(char data)
{
	while((UCSR0A & 0x20) == 0x00);
	UDR0 = data;
}
/*** Function for sending line ***/
void SendLine(char *string)
{
	while(*string != '\0')
	{
		SendByte(*string);
		string++;
	}
}

char ReceiveByte()
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}
void test_serial()
{
	SendByte('A');
	_delay_ms(10);
	SendLine("Hello World");
	
	if((ReceiveByte() == 'a') && (ReceiveByte() =='b'))
	{
		SendByte('o');
		SendByte('k');
	}
}
void init_adc()
{
	    ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)); //16Mhz/128 = 125Khz
	    ADMUX |= (1<<REFS0);       //AVCC(5V)
	    ADCSRA |= (1<<ADEN);      //ADC 인에이블
}
uint16_t read_adc(uint8_t channel)
{
	ADMUX &= 0xF0;
	ADMUX |= channel;
	
	ADCSRA |= (1<<ADSC);      //변환 시작
	while(ADCSRA&(1<<ADSC));//변환 완료되기를 기다림.
	
	return ADCW;  //ADC값 반환
}
void test_adc()
{
	init_adc();
	init_fnd();
	uint16_t adcValue =0;
	adcValue = read_adc(0);
	sprintf(textBuff,"adcValue : %d\r\n",adcValue);
	SendLine(textBuff);
	fnd_display(adcValue);
	_delay_ms(100);
}
void init_extSwitch()
{
	DDRE = 0x00;
	PORTE = 0xf0;
	EICRB =0x0a;
	
	EIMSK = 0x30; // INT4, INT5 사용
}
void test_extSwitch()
{
	init_fnd();
	fnd_display(extCount);
}

void init_i2c()
{
	DDRD |= (1<< I2C_SCL);
	DDRD |= (1<< I2C_SDA);
	TWBR = 32; // clock 주파수 설정 200khz
}
void I2C_start()
{
	TWCR =(1<<TWINT)| (1<<TWSTA)|(1<<TWEN);
	while (!(TWCR&(1<<TWINT)));
}
void I2C_transmit(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!(TWCR &(1<<TWINT)));
}
uint8_t I2C_receive_ACK()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!(TWCR &(1<<TWINT)));
	return TWDR;
}
uint8_t I2C_receive_NACK()
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR&(1<<TWINT)));
	return TWDR;
}
void I2C_stop()
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}
uint8_t bcd_to_decimal(uint8_t bcd)
{
	return (bcd>>4) *10 +(bcd&0x0f);
}
uint8_t decimal_to_bcd(uint8_t decimal)
{
	return (((decimal/10)<<4)|(decimal%10));
}
void write_twi_1byte_nopreset(char data)
{
	TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN);// START 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08) ;		// START 상태 검사, 이후 모두 상태 검사
	TWDR = 0x98 | 0;			// SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);		// SLA+W 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18) ;
	TWDR = 1;				// aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);		// aTS75 Reg 값 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
	TWDR = data;				// DATA 준비
	TWCR = (1 << TWINT) | (1 << TWEN);		// DATA 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);	// STOP 전송
	while ((TWCR & (1 << TWSTO))) ;		// STOP 확인
}
int read_twi_2byte_nopreset()
{
	char high_byte, low_byte;
	TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN);// START 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x08) ;		// START 상태 검사, 이후 ACK 및 상태 검사
	TWDR = 0x98 | 0;			// SLA+W 준비, W=0
	TWCR = (1 << TWINT) | (1 << TWEN);		// SLA+W 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x18) ;
	TWDR = 1;				// aTS75 Reg 값 준비
	TWCR = (1 << TWINT) | (1 << TWEN);		// aTS75 Reg 값 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x28) ;
	TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN);// RESTART 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x10) ;		// RESTART 상태 검사, 이후 ACK, NO_ACK 상태 검사
	TWDR = 0x98 | 1;			// SLA+R 준비, R=1
	TWCR = (1 << TWINT) | (1 << TWEN);		// SLA+R 전송
	while (((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x40) ;
	TWCR = (1 << TWINT) | (1 << TWEN | 1 << TWEA);// 1st DATA 준비
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x50);
	high_byte = TWDR;				// 1st DATA 수신
	TWCR = (1 << TWINT) | (1 << TWEN);// 2nd DATA 준비
	while(((TWCR & (1 << TWINT)) == 0x00) || (TWSR & 0xf8) != 0x58);
	low_byte = TWDR;				// 2nd DATA 수신
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);	// STOP 전송
	while ((TWCR & (1 << TWSTO))) ;		 // STOP 확인
	return((high_byte<<8) | low_byte);		// 수신 DATA 리턴
}

int main(void)
{
	init_led();
	init_buzzer();
	init_fnd();
	init_switch();
	init_timer();
	init_serial(115200);
	init_adc();
	init_extSwitch();
	init_i2c();
	
	//uint8_t address = 
	sei();
	
	uint8_t address = 0x98;
	int temp =0;

	
	while (1)
	{
		//test_led();
		//test_buzzer();
		//test_fnd();
		//test_switch();
		//test_timer();
		//test_serial();
		//test_adc();
		//test_extSwitch();
		
		/*
		temp = read_twi_2byte_nopreset(0);
		sprintf(textBuff,"temp : %d",temp);
		SendLine(textBuff);
		*/
	}
}

ISR(INT4_vect){
	extCount++;
}
ISR(INT5_vect){
	extCount--;
}
ISR(TIMER0_OVF_vect){
	TCNT0 =256-(256-5);
	count++;
	if (count >=1000){
		sec++; count=0;
	}
}
