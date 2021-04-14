
#include "xc.h"
#define _XTAL_FREQ 64000000

#pragma config FEXTOSC = OFF    // ->Oscillator not enabled
#pragma config RSTOSC = HFINTOSC_64MHZ    // ->HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1
#pragma config CLKOUTEN = OFF    // ->CLKOUT function is disabled
#pragma config CSWEN = ON    // ->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor enabled
#pragma config MCLRE = EXTMCLR    // ->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
#pragma config PWRTE = ON    // Power-up Timer Enable bit->Power up timer enabled
#pragma config LPBOREN = OFF    // ->ULPBOR disabled
#pragma config BOREN = SBORDIS    // Brown-out Reset Enable bits->Brown-out Reset enabled , SBOREN bit is ignored
#pragma config BORV = VBOR_2P45    // Brown Out Reset Voltage selection bits->Brown-out Reset Voltage (VBOR) set to 2.45V
#pragma config ZCD = OFF    // ZCD Disable bit->ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config DEBUG = OFF    // Debugger Enable bit->Background debugger disabled
#pragma config XINST = OFF    // Extended Instruction Set Enable bit->Extended Instruction Set and Indexed Addressing Mode disabled
#pragma config WDTCPS = WDTCPS_31    // ->Divider ratio 1:65536; software control of WDTPS
#pragma config WDTE = SWDTEN    // WDT operating mode->firmware control
#pragma config WDTCWS = WDTCWS_7    // WDT Window Select bits->window always open (100%); software control; keyed access not required
#pragma config WDTCCS = LFINTOSC    // WDT input clock selector->WDT reference clock is the 31.0 kHz LFINTOSC
#pragma config WRT0 = OFF    // Write Protection Block 0->Block 0 (000800-003FFFh) not write-protected
#pragma config WRT1 = OFF    // Write Protection Block 1->Block 1 (004000-007FFFh) not write-protected
#pragma config WRT2 = OFF    // Write Protection Block 2->Block 2 (008000-00BFFFh) not write-protected
#pragma config WRT3 = OFF    // Write Protection Block 3->Block 3 (00C000-00FFFFh) not write-protected
#pragma config WRTC = OFF    // Configuration Register Write Protection bit->Configuration registers (300000-30000Bh) not write-protected
#pragma config WRTB = OFF    // Boot Block Write Protection bit->Boot Block (000000-0007FFh) not write-protected
#pragma config WRTD = OFF    // Data EEPROM Write Protection bit->Data EEPROM not write-protected
#pragma config SCANE = ON    // ->Scanner module is available for use, SCANMD bit can control the module
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored
#pragma config CP = OFF    // UserNVM Program Memory Code Protection bit->UserNVM code protection disabled
#pragma config CPD = OFF    // DataNVM Memory Code Protection bit->DataNVM code protection disabled
#pragma config EBTR0 = OFF    // Table Read Protection Block 0->Block 0 (000800-003FFFh) not protected from table reads executed in other blocks
#pragma config EBTR1 = OFF    // Table Read Protection Block 1->Block 1 (004000-007FFFh) not protected from table reads executed in other blocks
#pragma config EBTR2 = OFF    // Table Read Protection Block 2->Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks
#pragma config EBTR3 = OFF    // Table Read Protection Block 3->Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks
#pragma config EBTRB = OFF    // Boot Block Table Read Protection bit->Boot Block (000000-0007FFh) not protected from table reads executed in other blocks

#define led_pin LATAbits.LATA7

#define m1a LATBbits.LATB1
#define m1b LATBbits.LATB0
#define sensor_enable LATBbits.LATB3
//data input a5
//motor outputs B0 and B1
//hall power enable b3
//button c7
//hall read b2

bit pressed, new_press;

unsigned char read_packet(void);
unsigned char bitpos __at(0x5c);
unsigned char test __at(0x5d);


#define UPDATE_RATE 500
unsigned int update_timer;
unsigned int led_timer;
bit update;

unsigned int runtime;
unsigned int milliseconds;

unsigned char system_state=0;
unsigned int state_timer=0;

bit motor_active;
unsigned char motor_state;
unsigned int motor_timer;
unsigned char motor_target;
unsigned int motor_timeout;
enum motor_target{
    open,
    closed,
};

void configure(void);
void delayms(int);
unsigned int get_analog(void);
void output_pwm(unsigned int value);
void ms_tasks(void);
void debounce(void);
void shutdown(void);
void sleep_1s(void);
unsigned int ms_ticks;

unsigned char EEPROM_read(unsigned int address);
void EEPROM_write(unsigned int address, unsigned char value);
void EEPROM_check_write(unsigned int address, unsigned char value);

void interrupt ISR(void)
{
    if(PIR0bits.TMR0IF){
		PIR0bits.TMR0IF=0;
		if(led_timer){
			led_pin=1;
			if(--led_timer==0) led_pin=0;
		}
        ms_ticks++;
        debounce();
	}
	
	if(PIR4bits.TMR1IF){
		PIR4bits.TMR1IF=0;

	}
	
}

void ms_tasks(void)
{
    if(++update_timer>UPDATE_RATE){update=1; update_timer=0;}
    if(++milliseconds>999){
        milliseconds=0;
        runtime++;
    }
    if(state_timer) state_timer--;
    if(motor_timer) motor_timer--;
}

void main(void)
{
	configure();

	runtime=0;
	
	INTCONbits.GIE = 1;

	led_timer=25;

    motor_state=0;
    motor_target=open;
    motor_active=0;
    
    system_state=0;
    state_timer=0;
    
    char match_open=0;
    char match_close=0;
    
    delayms(1);

	while(1){
        
        CLRWDT();
        
        if(ms_ticks){
            ms_ticks--;
            ms_tasks();
        }
        
        if(state_timer==0){
            switch(system_state){
                case 0: //initial state. normally happens after assembly loop locks up because we were just disconnected from aircraft. initiate check for data
                    state_timer=30;
                    IOCAF=0;
                    system_state=1;
                    break;
                case 1: //check results of test for active data line
                    if(IOCAF){
                        system_state=2; 
                    }
                    else{
                        system_state=10; 
                    }
                    break;
                case 2: //check for command packet
                    match_open=1; match_close=1;
                    //led_timer=5;
                    led_pin=1;
                    for(int i=0; i<5; i++){
                        char packet=read_packet();
                        led_pin=0;
                        if(packet!=138)match_open=0; //orange opens
                        if(packet!=222)match_close=0; //yellow closes
                    }
                    if(match_open) motor_target=open;
                    else if(match_close) motor_target=closed;
                    system_state=3;
                    state_timer=5;
                    break;
                case 3: //wait for motor movement. if not moving, sleep 1 second and then read another packet
                    if(!motor_active){
                        sleep_1s();
                        system_state=2;
                    }
                    else state_timer=5; 
                    break;
                case 10:    //sleeping (not connected to aircraft)
                    shutdown();
                    system_state=2;
                    break;
            }
        }
        
//        if(!motor_active){
//            shutdown();
////            if(pressed){
////                if(motor_target==closed) motor_target=open;
////                else motor_target=closed;
////            }
//            match_open=1; match_close=1;
//            led_timer=5;
//            for(int i=0; i<5; i++){
//                char packet=read_packet();
//                if(packet!=138)match_open=0; //orange opens
//                if(packet!=222)match_close=0; //yellow closes
//            }
//            if(match_open) motor_target=open;
//            else if(match_close) motor_target=closed;
//        }
        
        if(motor_timer==0){
            switch(motor_state){
                case 0: //idle, open
                    if(motor_target==closed){
                        motor_state=1;
                        motor_active=1;
                        sensor_enable=1;
                        motor_timer=5;
                    }
                    break;
                case 1: //start closing
                    motor_timer=2; //update rate
                    m1a=0; m1b=1; //close
                    motor_state=2;
                    motor_timeout=200; //400ms
                    break;
                case 2: //closing
                    motor_timer=2;
                    motor_timeout--;
                    if(get_analog()<380){
                        m1a=0; m1b=0;
                        sensor_enable=0;
                        motor_state=3;
                        motor_active=0;
                    }
                    else if(motor_timeout==0){
                        m1a=0; m1b=0;
                        sensor_enable=0;
                        motor_active=0;
                        motor_state=255; //error
                    }
                    break;
                case 3: //idle closed
                    if(motor_target==open){
                        motor_state=4;
                        motor_active=1;
                        sensor_enable=1;
                        motor_timer=5;
                    }
                    break;
                case 4: //start opening
                    motor_timer=2;
                    m1a=1; m1b=0; //open
                    motor_state=5;
                    motor_timeout=200;
                    break;
                case 5: //opening
                    motor_timer=2;
                    motor_timeout--;
                    if(get_analog()>450){
                        m1a=0; m1b=0;
                        sensor_enable=0;
                        motor_state=0;
                        motor_active=0;
                    }
                    else if(motor_timeout==0){
                        m1a=0; m1b=0;
                        sensor_enable=0;
                        motor_active=0;
                        motor_state=255; //error
                    }
                    break;
                case 255: //error
                    motor_state=0;
                    break;
            }
        }

		
        if(update){
			update=0;
			//led_timer=10;
		}
        
        if(new_press){
            new_press=0;
//            while(pressed);
//            shutdown();
//            new_press=0;
//            system_state=0;
        }

	}
}

unsigned char read_packet(void)
{
    INTCONbits.GIE=0;
    unsigned char timer=20;
    unsigned char escape=120; //30ms max wait time
    while(timer){
        if(PORTAbits.RA5) timer=20; //wait for 5ms gap
        __delay_us(250);
        timer--;
        if(--escape==0){
            INTCONbits.GIE=1;
            return 0;
        }
    }

    bitpos=8;
#asm
    banksel PORTA
startloop:
    btfss PORTA,5
    goto startloop ; wait until pulse starts
    nop
    rlncf _test,f,c
    bcf _test,0,c ; wait 6 cycles from pulse start
    btfsc PORTA,5
    bsf _test,0,c
    decfsz _bitpos,f,c
    goto startloop
#endasm
    INTCONbits.GIE=1;
    return test;
}

void sleep_1s(void)
{
    WDTCON0=0b00010101; //1 second, on
    INTCONbits.GIE=0;
    m1a=0; m1b=0; 
    led_pin=0;
    sensor_enable=0;
    ADCON0=0;
    PIE1=0; PIE2=0;
    PIE0=0; 
    IOCAP=0;
    SLEEP(); NOP(); NOP();
    configure();
    CLRWDT();

    INTCONbits.GIE=1;
}

void shutdown(void)
{
    //if(PORTAbits.RA5) return;
    WDTCON0=0;
    INTCONbits.GIE=0;
    m1a=0; m1b=0; 
    led_pin=0;
    sensor_enable=0;
    ADCON0=0;
    PIE1=0; PIE2=0;
    PIE0=0b00010000; //ioc
    //IOCCN=0b10000000; //switch
    //IOCCF=0;
    IOCAP=0b00100000; //data input
    IOCAN=0b00100000;
    IOCAF=0;
    unsigned char c=PORTA;
    SLEEP(); NOP(); NOP();
    configure();
    CLRWDT();
//    for(int i=0; i<20; i++){
//        debounce();
//        __delay_us(10);
//    }
//    new_press=0;
    INTCONbits.GIE=1;
}

void debounce(void)
{
    static unsigned char port_copy;
    static unsigned char db_count;
    
    unsigned char sample = PORTC;
    sample&=0b10000000; //user switch c7
    if(sample==port_copy){
        if(db_count<10) db_count++;
        else{
            if(sample) pressed=0;
            else{
                if(!pressed) new_press=1;
                pressed=1;
            }
        }
    }
    else{
        port_copy=sample;
        db_count=0;
    }
}

void configure(void)
{
	NVMCON1 = 0b10000000; //fucking silicon error http://www.microchip.com/forums/m957860.aspx
	
	WDTCON0=0b00010101; //1 second, onWDTCON0=0b00001101; //64ms, on
    
	
    OSCCON1 = 0x60;   // NOSC HFINTOSC; NDIV 1; 
    OSCCON3 = 0x00;   // CSWHOLD may proceed; SOSCPWR Low power; 
    OSCEN = 0x00;   // MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCFRQ = 0x08; //64MHz  OSCFRQ = 0x05;   // HFFRQ 16_MHz; 
    OSCTUNE = 0x00;  // TUN 0; 
  
    T0CON0 = 0x00;// T0OUTPS 1:1; T0EN disabled; T016BIT 8-bit; 
    T0CON1 = 0x56; // T0CS FOSC/4; T0CKPS 1:64; T0ASYNC not_synchronised;
    TMR0H = 0xFF;// TMR0H 255; 
    TMR0L = 0x00;// TMR0L 0; 
    PIR0bits.TMR0IF = 0; // Clear Interrupt flag before enabling the interrupt
    PIE0bits.TMR0IE = 1;// Enabling TMR0 interrupt.
    T0CON0bits.T0EN = 1;// Start the Timer by writing to TMR0ON bit
    PIR0bits.TMR0IF = 0; // Clear Interrupt flag before enabling the interrupt
    PIE0bits.TMR0IE = 1;// Enabling TMR0 interrupt.
    T0CON0bits.T0EN = 1;// Start the Timer by writing to TMR0ON bit
	
	INTCON=0b01000000; //enable peripheral
	PIE0=0b00100000; //timer0
	PIE1=0; PIE2=0; 
	PIE3=0b00000000; //
	PIE4=0b00000000; //
    

	
	TMR1CLK=1; //fosc/4
	T1CON=0b00000011; //1:1 prescale for 16ms timeout
	
//	T2PR = 255;
//	T2CON = 0b10110000; //on, 1:8 prescale - 1.9kHz PWM	
//	T2CLKCON = 0b00000001; //fosc/4
//	T2HLT = 0;
	
	IOCAN=0;
    IOCAP=0b00100000; //data input
    IOCAF=0;
	IOCBN=0b00000000; //
	IOCBP=0;
	IOCCN=0;
	IOCCP=0; 


    LATA = 0;    
    LATB = 0;    
    LATC = 0;  
	
    TRISA = 0b01101111;
    ANSELA= 0b00000000;
    WPUA =  0b01001111;
	
    TRISB = 0b11110100;
    ANSELB= 0b00000100;
    WPUB =  0b11110000;
	
    TRISC = 0b11111111;
    ANSELC= 0b00000000;
    WPUC =  0b11111111;
	
    ODCONA = 0;
    ODCONB = 0;
    ODCONC = 0;
	SLRCONC=0;
	INLVLC=0;

//	//USB connection
//    SPBRG1 = 12;                                    //Writing SPBRG Register
//    TX1STAbits.SYNC = 0;                                     //Setting Asynchronous Mode, ie UART
//    RC1STAbits.SPEN = 1;                                     //Enables Serial Port
//    RC1STAbits.CREN = 1;                                     //Enables Continuous Reception
//    TX1STAbits.TXEN = 1;                                     //Enables Transmission
	
//    PPSLOCK = 0x55;
//    PPSLOCK = 0xAA;
//    PPSLOCKbits.PPSLOCKED = 0x00; // unlock PPS
	
//    RB6PPS = 0b00001001; //UART1 TX 
//	RX1PPS = 0b00001111; //RB7
	
	//RC2PPS = 0b00000111; //PWM3
	
//    PPSLOCK = 0x55;
//    PPSLOCK = 0xAA;
//    PPSLOCKbits.PPSLOCKED = 0x01; // lock PPS
	
//	//i2c master
//	SSP2STAT=0b10000000;
//	SSP2CON1=0b00101000;
//	SSP2CON2=0;
//	SSP2CON3=0;
//	SSP2ADD=0x09;	//400kHz clock
	
//	PWM3DCH = 0; PWM3DCL = 0;
//	PWM3CON = 0b10000000; 
//	CCPTMRS = 0b01010101; // PWM3/4 from T2, CCP from T2
    
	ADCON0=0b10000100; //adc on, not continuous, clock from fosc div, right justify
	ADCON1=0b00000000; //single sample, no precharging
	ADCON2=0b00000000; //no filtering
	ADCON3=0b00000000; //no threshold interrupts
	ADCLK=63; //fosc/128
	ADREF=0b00000000; //vref from rails
	ADPCH=0b00001010; //ANB2 channel    
	
}

void output_pwm(unsigned int value) 
{
    PWM3DCH = (value & 0x03FC)>>2;
    PWM3DCL = (value & 0x0003)<<6;
}

unsigned int get_analog(void)
{
	ADCON0bits.GO_nDONE=1;
	while(ADCON0bits.GO_nDONE);
	return ADRES;
}
void delayms(int delay)
{
	while(delay){
		__delay_ms(1);	 
		 delay--;
	}
}	





unsigned char EEPROM_read(unsigned int address)
{
//copied from auto-generated MCC code
    NVMADRH = ((address >> 8) & 0x03);
    NVMADRL = (address & 0xFF);
    NVMCON1bits.NVMREG = 0;
    NVMCON1bits.RD = 1;
    NOP();  // NOPs may be required for latency at high frequencies
    NOP();
	NVMCON1 = 0b10000000; //fucking silicon error http://www.microchip.com/forums/m957860.aspx
    return (NVMDAT);
}

void EEPROM_write(unsigned int address, unsigned char value)
{
	//copied from auto-generated MCC code
    char GIEBitValue = INTCONbits.GIE;
    NVMADRH = ((address >> 8) & 0x03);
    NVMADRL = (address & 0xFF);
    NVMDAT = value;
    NVMCON1bits.NVMREG = 0;
    NVMCON1bits.WREN = 1;
    INTCONbits.GIE = 0;     // Disable interrupts
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1bits.WR = 1;
    // Wait for write to complete
    while (NVMCON1bits.WR)
    {
    }
    NVMCON1bits.WREN = 0;
	NVMCON1 = 0b10000000; //fucking silicon error http://www.microchip.com/forums/m957860.aspx
    INTCONbits.GIE = GIEBitValue;   // restore interrupt enable
}	

void EEPROM_check_write(unsigned int address, unsigned char value)
{
	if(EEPROM_read(address)!=value) EEPROM_write(address, value);
}

__EEPROM_DATA(1, 2, 3, 4, 0, 0, 0, 0);
__EEPROM_DATA(0, 0, 0, 0, 0, 0, 0, 0);
__EEPROM_DATA(0, 0, 0, 0, 0, 0, 0, 0);
