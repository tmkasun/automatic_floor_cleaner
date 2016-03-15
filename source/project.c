/* © University Of Moratuwa all rights received 2012 visit http://www.itfac.mrt.ac.lk */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
-------------------------------Messages For Developers---------------------------------------------
Planed Codings={


}

Special Notes -: Pin D0,D1,C0,C3 are assigned to Input Pins In Driver Controller IC L293D
                 D0 and D1 pins are for left motor inputs
                 C0 and C3 are for Right Motor Inputs
                 Algo:
                       C0 High D0 High = Rotate Clockwise
                       C3 High D1 High = Rotate Aniticlockwise
                 All A port pins are used for Radio Transmition
                 we have used all the I/O pins in the pic
                 
                 PIN_C7 is used for Enable Motors and fans
                 PIN_C6 is used for buzzer
                 

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <18F452.H>          
#device adc=8 
#DEVICE HIGH_INTS=TRUE
#fuses NOWDT, HS, NOPUT, NOPROTECT, NOBROWNOUT, NOLVP, NOCPD, NOWRT, NODEBUG 
#use delay(clock=20000000) 


#include <stdlib.h>
#include <stdio.h>
#include <math.h> 
#include <string.h>
#include "f_LCD.c" 
#include "f_kbd.c" 
#include "TONES.C"
#define trig pin_E0     // Change as you wish, can use any pin in the MCU
#define echo pin_E1     // Change as you wish, can use any pin in the MCU 
//pin_C6 is used for music genaration can change bin by editing TONES.c file in ccs c folder

#define isr_cons 1000      //Constant value to be used in Interrupt Service Routing 
#USE FAST_IO(A)
//-------------declare global variable for duty values----------------
//RTCc variables
unsigned int16  tic=0;
unsigned long up_time;//Motors uptime start from each column
float distance_obs;
char controller_mode;//select controller mode Remote or manual
byte obs_status=1;  //select to avoid the obstracles or not 1 Automatic 2 manual
long duty1,duty2,width,length=0;//Width of the room and length of the room
float speed=10.96;//speed of the robot=16.96 cm/s (Orginal) now in mm per second
int direction=0,turn=1;//fwd=0,bkwd=1,LHS=0,RHS=1
int main_dir = 0;
unsigned int16 cunt1=0,cunt2=0;//cunt1 and cunt2 for obstracle avoidance width and length

unsigned int16 int_value=0;//for compair with tic
char ch1=0;//character from the RF module
unsigned int8 ascii,lsb,msb;//least significant bit and mostsignificant bit of the  ascii character of the reciving character
static char last_char;
float T180_temp=(56.87/speed)*1000; //Time in second for turn robot in 180 deg 50.9727 (Orginal) is turning distance in centimeters
unsigned int16 T180 = 3700;//orginal 3005.4658 ms
unsigned int16 T180r = 4100;//orginal 3005.4658 ms

unsigned int16 T90 = 2000;
unsigned int16 T90r = 2100;

//--------------------function declaration----------------
//------------Motor Driving Functions------------
void key_fwd(unsigned long time); //Drive Robot Forward

void key_left(unsigned long time); //Turn Left Hand Side

void key_right(unsigned long time);//Turn Right Hand Side

void key_back(unsigned long time); //Drive Backward

void key_stop();//Stop Robot
//-------------------------------------------------
//-------------------ISR Motor Driving Functions----------
void isr_fwd(unsigned int16 time); //Drive Robot Forward

void isr_left(unsigned int16 time); //Turn Left Hand Side

void isr_right(unsigned int16 time);//Turn Right Hand Side

void isr_back(unsigned int16 time); //Drive Backward

void isr_stop();//Stop Robot
//---------------------------------------------------
float isr();
void int_fwd(unsigned long time);
int eepw();//EEPROM Write Testing
unsigned int16 inew;
long int scanf2();//Read A numeric value Digit From Keyboard
long int scanf_remote();//Read A Digit From Remote
void eepr();//Read EEPOR and Get Data which were previously written  
void startRoller();//Start Dry And Wet Rollers
void stopRoller();//Stop Dry And Wet Rollers
void animation1();//Partially Marquee Animation (Marquee String in LCD line 2)
char keybd();//Read keypad and return a character value from it
void proAnimation();//Progress animation (Animate with '#' in the second line of the LCD panel)
void startclening();//Start Cleaning Process
void main_algo(long int width,long int length);//Main Algorithamic Function
char rf_Rx(unsigned int8 selection);//0=ASCII mode 1=Fast mode ,recive a character from RF module
void remote_c();//Totall remote control
void remote();//Remot Control main menu
void obs_function();//obstracle avoidance cheacker 1 auto 2 manual
#INLINE
float distance();//return the distance in cm whwn the function is called
#INLINE
void servo_lhs();//turn servo LHS
#INLINE
void servo_rhs();//turn servo RHS
#INLINE
void servo_mid();
void obs_avoid();
void service();//Enter to service menu
#define SIZE 25
void music();


//--------------Define Interrupt Routing--------------------
/* Requiered Function calls
enable_interrupts (level);
disable_interrupts (level);
clear_interrupt(level);


*/

//----------------Interrupt service routing for timer1 overflow-------------------
#int_rtcc HIGH
void TIMER0_isr(void)
{
    //Timer 1 Interrupt Service Routing
    tic+=13.10; //increment tic count by every 13 milisecond for timing pourpse

}



//----------------------------Main Function Start From hear-------------------
void main() { 

    //-----------For power On indication------------------------------
    set_tris_e(000);//set all E ports to outputs
    set_tris_a(63);//set all A ports as inputs for RF modul
    //!         output_low(PIN_E0);//Power on LED
    //!         output_high(PIN_E1);// ""     "" // no need to add since power on led works on external
    //----------------Setup timers-----------------

    setup_timer_0(RTCC_INTERNAL|RTCC_DIV_256|RTCC_8_BIT);//for INT_RTCC
    setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);//Timer1 for sonar
    setup_timer_2(T2_DIV_BY_4,128,1);  //set frequency 1.35KHz=CLOCK/(4*T2_DIV_BY_16*128*1)
    //-----------------Local variable Declaration-----------------------
    char k;//selecting character
    unsigned int8 usr_e=0;

    //------------------Set speed and Obstracle Mode value by getting it from EEPROM------------------------------
    usr_e=read_eeprom(0);
    if(read_eeprom(1)==-1)
        speed-=((float)usr_e)/100;
    else if(read_eeprom(1)==+1)
        speed+=((float)usr_e)/100;
    // obs_status=read_eeprom(2);
    //------------------Get The servo to midle position------------------------------
    servo_mid();
    //------------------Set Duty Values------------------------------
    duty1=0x00;
    duty2=0x00;
    //--------------------initialize motors-----------------------------
    output_low(PIN_D0);//Disable Motor 1 Input 1 L293D
    output_low(PIN_D1);//Disable Motor 1 input 2 L293D
    output_low(PIN_C0);//Disable Motor 2 input 4 L293D
    output_low(PIN_C3);//Disable Motor 2 input 3 L293D
    output_low(PIN_C6);
    output_low(PIN_C7);
    stoproller();//Set Rollers Off
    //----------------- Setting up PWM Timer------------------------

    setup_ccp1(CCP_PWM);
    setup_ccp2(CCP_PWM);
    //-------------------Initializing KBD and LCD----------------------------------------------
    lcd_init();//Setting Up LCD
    kbd_init();//Setting Up Keyboard
    //----------------------Testing Codes Goes Here-------------------------------
    //!
    //!         printf(lcd_putc,"\fT180 %Lu",T180);
    //!         delay_ms(9000);

    //printf(lcd_putc,"\fTesting speed\n=%Lu us%u a=%u",speed,read_eeprom(0),read_eeprom(1));
    //printf(lcd_putc,"\fobs_status=%u",obs_status);
    //delay_ms(1000);
    //obs_function();
    //!
    //---------------------------------------------------------------------------
    //-----------starting startup Animation------------------------
    lcd_putc("\f   Welcome To   \n");
    delay_ms(300);
    lcd_putc("      F");
    delay_ms(250);
    lcd_putc("i");
    delay_ms(250);
    lcd_putc("T");
    delay_ms(250);
    lcd_putc("Z");
    delay_ms(250);

    //--------------------------------------------------------------
mode_selection:
    lcd_putc("\fSelect Controller\nMode");
    delay_ms(1000);
    lcd_putc("\fRemote||Manual\n   1  ||   2 :");
    controller_mode=rf_rx(0);
    if(controller_mode=='1')
        remote();
    printf(lcd_putc,"\f1.Cleanning\n2.Service Menu");
    delay_ms(1500);
    lcd_putc("\f3.Remote Menu");

    while (TRUE) {
        //-----------------Read Values From keyboard---------
        if(controller_mode=='2')
            k=keybd();
        else if(controller_mode=='1')
            k=rf_rx(0);
        if(k!=0)
            switch (k){
            case '1':
                startclening();
                main_algo(width,length);
                break;

            case '2':
                service();
                break;

            case '3':
                remote();
                break;

            case '4':
                isr_left(T90);
                break;

            case '5':
                lcd_putc("\fTesting w=44\nlenght=150");
                main_algo(44,350);
                break;

            case '6':
                isr_right(T90);
                break;

            case '7':
                lcd_putc("\fWhile Testing");
                delay_ms(2000);
                servo_rhs();
                lcd_putc("\fWait 2500ms");
                delay_ms(2500);
                while(distance()<30.0){
                    key_fwd(100);
                    cunt2++;
                }
                printf(lcd_putc,"\fcunt2=%Lu",cunt2);
                delay_ms(2000);
                cunt2 = 0;


                servo_lhs();
                lcd_putc("\fWait 2500ms");
                delay_ms(2500);
                while(distance()<30.0){
                    key_fwd(100);
                    cunt2++;
                }
                printf(lcd_putc,"\fcunt2=%Lu",cunt2);
                cunt2 = 0;
                delay_ms(2000);



                servo_mid();
                lcd_putc("\fWait 2500ms");
                delay_ms(2500);
                while(distance()<30.0){
                    key_fwd(100);
                    cunt2++;
                }
                printf(lcd_putc,"\fcunt2=%Lu",cunt2);
                delay_ms(2000);
                break;

            case '8':
                char k1;
                lcd_putc("\f1.T90  2.T180\n3.T90r 4.T180r:");
                k1=keybd();
                switch (k1){
                case '1':
                    printf(lcd_putc,"\fT90 = %Lu\nT90 :",T90);
                    T90 = scanf2();
                    delay_ms(1000);
                    isr_left(T90);
                    break;

                case '2':
                    printf(lcd_putc,"\fT180 = %Lu\nT180 :",T180);
                    T180 = scanf2();
                    delay_ms(1000);
                    isr_left(T180);
                    break;


                case '3':
                    printf(lcd_putc,"\fT90r = %Lu\nT90r :",T90r);
                    T90r = scanf2();
                    delay_ms(1000);
                    isr_right(T90r);
                    break;

                case '4':
                    printf(lcd_putc,"\fT180r = %Lu\nT180r :",T180r);
                    T180r = scanf2();
                    delay_ms(1000);
                    isr_right(T180r);
                    break;
                case '5':
                    int duty_right,duty_left;
                    printf(lcd_putc,"\fenter duty_right:\n");
                    duty_right = scanf2();
                    lcd_putc("\fduty_left:");
                    duty_left = scanf2();
                    delay_ms(1000);
                    output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                    output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
                    set_pwm1_duty(duty_left);//125 good values 116
                    set_pwm2_duty(duty_right);//128 good values 120
                    delay_ms(3000);
                    key_stop();
                    break;
                }
                break;
            case '9':
                output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
                set_pwm1_duty(126);
                set_pwm2_duty(128);
                while(distance()<35.0){
                    delay_ms(100);
                    cunt1++;
                }
                //key_stop();
                lcd_putc("\fFunctionpas");//hv to remove only 4f debuging
                delay_ms(2000);
                int i1,i2;
                for(i1=128,i2 = 126;i1>=0 || i2>=0;i1--,i2--){
                    set_pwm1_duty(i2);
                    set_pwm2_duty(i1);

                }
                output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
                //isr_fwd(2000);
                stopRoller();

                lcd_putc("\fdelay 5000ms");
                delay_ms(5000);

                lcd_putc("\fpwm1 1st \nstop");
                output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
                set_pwm1_duty(126);
                set_pwm2_duty(128);
                delay_ms(2000);

                output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                delay_ms(20);
                output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D

                set_pwm1_duty(0);
                set_pwm2_duty(0);

                break;

            default :
                lcd_putc("\fSorry :( Invalid\n   Selection");
            }

    }


    //--------------------------End Of the main Function----------------------------
}
void key_back(unsigned long time){
    
    //---------------Enable Motors-----------------------
    output_high(PIN_C0);//Enable (Right)Motor 2 input 3 L293D
    output_high(PIN_D1);//Enable (Left)Motor 2 input 3 L293D

    //----------------accelerating motors-----------
    
    set_pwm1_duty(106);
    set_pwm2_duty(110);
    //    delay_ms(10);
    //    printf(lcd_putc,"\f\fAccelerating\n \t Motors");
    

    //    lcd_putc("\fMotors running\n@ full speed");
    delay_ms(time);
    key_stop();
    direction=1;


}


void service(){
    char k;
    lcd_putc("\fWelcome To FiTZ\nService Menu");
    while (TRUE) {
        //-----------------Read Values From keyboard---------
        if(controller_mode=='2')
            k=keybd();
        else if(controller_mode=='1')
            k=rf_rx(0);
        if(k!=0)
            switch (k){
            case '1':
                float distance_value=6;
                while(distance_value>5){
                    distance_value=distance();
                    printf(lcd_putc,"\fDistance=%f ",distance_value);
                    delay_ms(500);
                }
                break;

            case '2':
                key_fwd(2000);
                break;

            case '3':

                printf(lcd_putc,"\fTic Test\ncurnt Tic=%Lu",tic);
                delay_ms(2000);
                unsigned int16 x,delay_time=0;
                for(x=0;x<10;x++){
                    enable_interrupts(INT_RTCC);
                    enable_interrupts(GLOBAL);
                    delay_ms(delay_time);
                    printf(lcd_putc,"\fdly time=%Lu\ntics=%Lu",delay_time,tic);


                    disable_interrupts(INT_RTCC);
                    disable_interrupts(GLOBAL);
                    delay_ms(2000);
                    printf(lcd_putc,"\feps=%Lu",delay_time-tic);
                    delay_ms(2000);
                    tic = 0;
                    delay_time +=100;
                }
loop_x:
                lcd_putc("\fMotors uptime\ntic count test");
                delay_ms(2000);
                delay_time = scanf2();
                enable_interrupts(INT_RTCC);
                enable_interrupts(GLOBAL);
                key_fwd(delay_time);
                disable_interrupts(INT_RTCC);
                disable_interrupts(GLOBAL);
                printf(lcd_putc,"\fdelay time=%Lu\nTic time=%Lu",delay_time,tic);
                //                long int i1,number1=0,vt=52;
                //                printf(lcd_putc,"\fcalibrating..");
                //                for(i1=0;i1<500;i1++){
                //                    set_timer0(0);
                //                    delay_us(vt);
                //                    vt+=52;
                //                    number1=get_timer0();
                //                    printf(lcd_putc,"\fTime=%lu",number1);
                //                    delay_ms(700);
                //                }

                tic = 0;
                goto loop_x;
                disable_interrupts(INT_RTCC);
                disable_interrupts(GLOBAL);
                break;
                
            case '4':
                isr_left(T180);
                break;

            case '5':
                //                output_low(PIN_E2);
                lcd_putc("\fServo testing");
                servo_mid();
                delay_ms(500);
                lcd_putc("\fRHS");
                servo_rhs();

                delay_ms(500);

                lcd_putc("\fMID");
                servo_mid();
                delay_ms(500);

                lcd_putc("\fLHS");
                servo_lhs();
                delay_ms(500);

                lcd_putc("\fRHS");
                servo_rhs();
                delay_ms(500);

                lcd_putc("\fLHS");
                servo_lhs();
                break;

            case '6':
                
                isr_right(T180r);
                break;
                
            case '7':
                char arg;
                float intermid_value=0;
                unsigned int8 usr_e=0;
                lcd_putc("\fEnter Val(0-255)\ndef(16.198):");
                usr_e=(unsigned int8)scanf2();
argument:
                printf(lcd_putc,"\f%s\nkey(*)||key(#)","Add(+)|Sub(-)");
                //arg=keybd();
                if(controller_mode=='2')
                    arg=keybd();
                else if(controller_mode=='1')
                    arg=rf_rx(0);
                if(arg=='*'){
                    intermid_value=16.198+(float)usr_e/100;
                    printf(lcd_putc,"\f New speed=%f",intermid_value);
                    write_eeprom(1,+1);
                    delay_ms(289);
                }
                else if(arg=='#'){
                    intermid_value=16.198-(float)usr_e/100;
                    printf(lcd_putc,"\f New speed=%f",intermid_value);
                    write_eeprom(1,-1);
                    delay_ms(289);
                }
                else {
                    lcd_putc("\fWrong Argument");
                    delay_ms(289);
                    goto argument;
                }
                write_eeprom(0,usr_e);
                lcd_putc("Writing 2 EEPROM");
                delay_ms(1000);

                printf(lcd_putc,"\fNew speed:\n%f cm/s",intermid_value);
                delay_ms(289);
                lcd_putc("\fRestarting...");
                delay_ms(289);
                reset_cpu();

                break;

            case '8':
                key_back(2500);
                break;
            case '9':
                obs_function();
                break;

            case '#':
                lcd_putc("\fExit From \nService Menu");
                delay_ms(500);
                return;
                break;

            case 'A':
                //lcd_putc("\fMusic Testing\n& EEPROM Testing");
                //!       int xx=0;
                //!       while(xx <1000){
                //!       output_high(PIN_A0);
                //!delay_us(1894);
                //!output_low(PIN_A0);
                //!delay_us(1894);
                //!xx++;
                //!}
                //                music();
                lcd_putc("\fEEPROM Write\n Test");
                delay_ms(1000);
                lcd_putc("\fEnter Owners\nname:");
                eepw();
                break;

            case 'B':
                char select,cha;
                int8 mode;

loop_rf:
                lcd_putc("\fEnter Reciving\nmode:");
                select=keybd();
                if(select=='A')
                    mode=0;
                else if(select=='B')
                    mode=1;
                lcd_putc("\fWating For Data");
                cha=rf_rx(mode);
                printf(lcd_putc,"\fchar=%c:%u",cha,cha);
                delay_ms(1000);
                goto loop_rf;
                break;


            case 'C':
                int i=0,number=0;
                printf(lcd_putc,"\fTime");
                number=scanf2();
                printf(lcd_putc,"\fTime=%u\nSpeed=%u",i,number);
                delay_ms(2000);
                printf(lcd_putc,"\fcalibrating..");
                output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
                set_pwm1_duty(106);//125 good values 116
                set_pwm2_duty(110);//128 good values 120
                delay_ms(number);
                output_low(PIN_D0);//disable enable pin1 in L293D
                output_low(PIN_C0);//Disable enable pin2 in L293D
                output_low(PIN_C3);
                output_low(PIN_D1);
                set_pwm1_duty(0);
                set_pwm2_duty(0);


                break;

            case 'D':
                startclening();
                main_algo(width,length);
                break;

            default :
                lcd_putc("\fSorry :( Invalid\n   Selection");


            }


    }


}
void key_fwd(unsigned long time){
    //clear_interrupt(GLOBAL);
    //enable_interrupts(INT_RTCC);
    //enable_interrupts(GLOBAL);

    //int dt=0;dt// means delay time

    output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
    output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
    set_pwm1_duty(126);
    set_pwm2_duty(127);

    lcd_putc("\fFWD");
    delay_ms(time);//keep the accelarated speed until "Time" miliseconds
    key_stop();
    direction=0;
}

//--------------------------Stop Robot------------------------------------------
void key_stop(){
    int i,init=duty1,dt;

    dt=(2000)/init;

    for(i=110;i<=0;i--){
        //------------------Braking motors-----------------
        set_pwm1_duty(i);
        set_pwm2_duty(i);
        delay_ms(dt);
    }

    lcd_putc("\fStop");
    output_low(PIN_D0);//disable enable pin1 in L293D
    output_low(PIN_C0);//Disable enable pin2 in L293D
    output_low(PIN_C3);
    output_low(PIN_D1);
    return;

}

//--------------------------Function for turning LHS----------------------------
void key_left(unsigned long time){
    //clear_interrupt(GLOBAL);
    //enable_interrupts(INT_RTCC);
    //enable_interrupts(GLOBAL);
    // duty1=duty2=0x00;
    output_high(PIN_D0);//Enable enable pin1 in L293D
    //output_high(PIN_C0);//Enable enable pin2 in L293D

    set_pwm2_duty(128);
    set_pwm1_duty(0);


    lcd_putc("\fTurning Left");
    delay_ms(time);

    key_stop();

    turn=0;

}

//--------------------------Function for turning RHS----------------------------

void key_right(unsigned long time){
    //clear_interrupt(GLOBAL);
    //enable_interrupts(INT_RTCC);
    //enable_interrupts(GLOBAL);

    //output_high(PIN_D1);//Enable enable pin1 in L293D
    output_high(PIN_C3);//Enable enable pin2 in L293D

    set_pwm1_duty(118);
    set_pwm2_duty(0);
    delay_ms(10);

    lcd_putc("\fTurning Right");
    delay_ms(time);

    key_stop();
    turn=1;
}

//---------------------EEPROM Write Test----------------------------------------
int eepw(){
    char ch;
    int eeprom_add=0,ascii;
    //------Clear EEPROM memory before writing--------------
    for(eeprom_add=0;eeprom_add<=12;eeprom_add++){
        write_eeprom(eeprom_add,NULL);

    }
    eeprom_add=0;
    //-------------------------------------------
    while(TRUE){
        ch=kbd_getc();
        if(ch=='#')
            lcd_putc("\b \b");
        else if(ch=='*'||eeprom_add==11 ){
            lcd_putc("\fSaving...");
            delay_ms(1000);
            return 0;
        }
        else if(ch!=0){
            lcd_putc(ch);
            ascii=(int)ch;
            write_eeprom(eeprom_add,ascii);
            eeprom_add++;
        }

    }

}
//---------------------EEPROM Read Test-----------------------------------------  

void eepr(){
    char ch;
    int eeprom_add;
    for(eeprom_add=0;eeprom_add<=11;eeprom_add++){
        ch=read_eeprom(eeprom_add);
        lcd_putc(ch);
    }



}

long int scanf2(){
    char ch,zero='0';
    unsigned int32 number=0,in;
    int count=0;
    

    //----------------------------------------------
    while(TRUE){
        ch=kbd_getc();

        if(ch!=0){

            if(ch=='#'&&count>0){
                lcd_putc("\b \b");
                count--;
                number=(number-in)/10;
                continue; }

            if(ch>='0' && ch<='9'){
                count++;
                lcd_putc(ch);
                //----Convert Character Digits to Integer values by deducting ASCII '0'----
                in=ch-zero;
                number=(number*10)+in;


            }

            if(ch=='*')
                return number;
        }

    }
}

//-----Start Rollers by Enabling E1 and E2 Pins---------

void startRoller(){
    //Dry roller is attached to Enable Enable pin 1 -->> 25 (Port c6)

    //output_high(PIN_C6);
    output_high(PIN_C7);
    //  printf(lcd_putc,"\fRollers Started\n");
    //    delay_ms(1000);
    //Dry roller is attached to Enable Enable pin 2 -->> 26 (Port c7)
    //printf(lcd_putc,"Fans Started");
    //  delay_ms(1000);
}

//--------------Stop Rollers by Enabling E1 and E2 Pins------------------------

void stopRoller(){

    //Dry roller is attached to Enable Enable pin 1 -->> 25 (Port c6)
    printf(lcd_putc,"\fRollers Stoped\n");
    //output_low(PIN_C6);
    //Dry roller is attached to Enable Enable pin 2 -->> 26 (Port c7)
    printf(lcd_putc,"Fans Stoped");
    output_low(PIN_C7);


}
//-------------return a character from keyboard in formated way-----------------

char keybd(){
    //-----------A tempory character -------------
    char k;
    while (TRUE) {
        //-----------------Read Values From keyboard---------
        k=kbd_getc();

        if(k!=0)
            return k;
        
    }



}

void animation1(){

    lcd_putc('\f');

    signed int8 i=0;


    while(1){

        lcd_gotoxy(1,2);
        for(i=1;i>(-16);--i){
            printf(lcd_putc,"%s",":) :)");
            if(i==1)
                delay_ms(1000);
            delay_ms(100);
            lcd_putc('\f');
            lcd_gotoxy(i,2);
            if(i==(-16)){
                lcd_gotoxy(1,2);
                printf(lcd_putc,"%s",":) :)");
                delay_ms(1000);
                break;}
        }

    }
}

//----------------------Small animation on screen using '#'--------------------
void proAnimation(){
    int i=0;
    lcd_gotoxy(1,2);
    lcd_putc("0           100%");
    lcd_gotoxy(2,2);
    for(i=2;i<13;++i){
        lcd_putc("#");
        delay_ms(200);

    }
}
void startclening(){

    int selection;
reenter1:
    lcd_putc("\fEnter Width Of The \nRoom:       (cm)");
    lcd_gotoxy(6,2);
    width=scanf2();
    printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",width);
reselect1:
    delay_ms(1000);
    printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
    //selection=keybd();
    if(controller_mode=='2')
        selection=keybd();
    else if(controller_mode=='1')
        selection=rf_rx(0);
    if(selection=='#')
        goto reenter1;
    else if(selection=='*'){
        lcd_putc("\f");
        break;
    }

    else {
        lcd_putc("\fInvalid Input :(");
        goto reselect1;
    }
reenter2:
    lcd_putc("\flength(Min 50cm)\nOf Room:    (cm)");
    lcd_gotoxy(9,2);
    length=scanf2();
    printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",length);
reselect2:
    delay_ms(1000);
    printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
    //selection=keybd();
    if(controller_mode=='2')
        selection=keybd();
    else if(controller_mode=='1')
        selection=rf_rx(0);
    if(selection=='#')
        goto reenter2;
    else if(selection=='*'){
        lcd_putc("\f");
        break;
    }

    else {
        lcd_putc("\fInvalid Input :(");
        goto reselect2;
    }


    lcd_putc("\fCalculating....");
    //   proAnimation();
    printf(lcd_putc,"\fStart Cleaning");
    delay_ms(500);

    return;

}

void main_algo(long int width,long int length){
    //--------------enable Interrupts-----------------------------
    unsigned int16 width_of_the_robot=20,nc,i,modulus;//width of the robot is in centimeters
    unsigned int16 distance_calibrat_left,distance_calibrat_right;
    length-=45.72;//reduce length of the robot(47.72) from the user enter value and accelaration time and the stop time
    up_time=(length/speed)*1000;
    printf(lcd_putc,"\fMotors \nRun time=%Lums",up_time);//this is for debuging hv to remove
    delay_ms(2000);
    modulus=width%width_of_the_robot;//uncleaned part because of the width diference
    printf(lcd_putc,"\fExtra area= %Lu",modulus);//this is for debuging hv to remove
    delay_ms(2000);
    nc=width/width_of_the_robot;//NC = Number of columns
    printf(lcd_putc,"\fNumber of C=%Lu",nc);
    delay_ms(2000);
    clear_interrupt(INT_RTCC);
    for(i=0;i<nc;i++){

        startRoller();
        int_fwd(up_time);
        //        disable_interrupts(GLOBAL);
        //        disable_interrupts(INT_RTCC);
        if (i==0){
            servo_lhs();
            delay_ms(500);
            distance_calibrat_left = distance();
            delay_ms(500);
            servo_rhs();
            distance_calibrat_right = distance();
            if (distance_calibrat_left < distance_calibrat_right)
                main_dir = 0;

            else
             main_dir = 1;

            servo_mid();
        }

        printf(lcd_putc,"\fFinish Cleaning\nColumn %Lu",i+1);
        delay_us(2000);

        lcd_putc("\fFunctionpas");//hv to remove only 4f debuging
        delay_ms(1500);
        printf(lcd_putc,"\fcount tics=%Lu\ndiffrence=%Lu",tic,up_time-tic);
        delay_us(3000);
        tic=0;//set the timer to 0 to re count on next obstracle
        delay_ms(800);

        isr_back(1000);
        delay_ms(800);
        if((main_dir==0) && (i!=(nc-1))){
            stopRoller();
            delay_ms(800);
            isr_right(T180r);
            main_dir = 1;
            delay_ms(800);
        }

        else if((main_dir==1) && (i!=(nc-1))){
            stopRoller();
            delay_ms(800);
            isr_left(T180);

            main_dir = 0;
            delay_ms(800);
        }

    }
    if(modulus!=0){
        if(main_dir==0 && i!=(nc-1)){

            main_dir = 1;
            stopRoller();
            delay_ms(800);
            isr_right(T90);
            startRoller();
            delay_ms(800);
        }

        else if(main_dir==1 && i!=(nc-1)){

            main_dir = 0;
            stopRoller();
            delay_ms(800);
            isr_left(T90);
            
            delay_ms(800);
            startRoller();

        }
        lcd_putc("\fBacking");
        delay_ms(800);
        printf(lcd_putc,"\fBack Time= %f",((22-modulus)/speed)*1000);//this is for debuging hv to remove
        key_back(((22-modulus)/speed)*1000);//turn and go back

        delay_ms(2000);
        if(main_dir==0){

            main_dir = 1;
            stopRoller();
            key_left(T90);
            
            delay_ms(800);
        }

        else if(main_dir==1){

            main_dir = 0;
            stopRoller();
            key_right(T90);
            
            delay_ms(800);

        }
        startRoller();
        int_fwd(up_time);

    }
    stopRoller();
    while(i<5){
        generate_tone(B_NOTE[0],200);
        delay_ms(300);
        generate_tone(C_NOTE[0],200);
        i++;
    }
    i=0;//set sound counter 0 for future use
    printf(lcd_putc,"\fCompleated");
}

float distance()//return the distance in cm when the function is called
{
    float distance,time1=0;          // Defining variables

    output_high(trig);           // ping the sonar
    delay_us(12);                          // sending 10us pulse
    output_low(trig);
    delay_us(150);//While sending ultrasonic sound wave (Actual delay time =200 us)
    while(!input(ECHO));                       // wait for high state of echo pin

    set_timer1(0);                           // setting timer zero

    while((get_timer1()< 5798) && input(ECHO));                       // Wait for high state of echo pin

    time1=get_timer1(); // Getting the time get_timer1 gives the number of trigers during the plus high
    //and 1 trig = 2x10^-6 s =2us

    distance=(time1*2)/77.3;            // Calculating the distance
    delay_ms(52);
    return distance;
}

void servo_lhs()//turn servo LHS
{
    lcd_putc("\fLHS");
    int s;
    for(s=0;s<100;s++){
        output_high(pin_e2);
        delay_us(600);
        output_low(PIN_E2);
        delay_us(15000);
    }
}

void servo_rhs()//turn servo RHS
{

    lcd_putc("\fRHS");
    int s;
    for(s=0;s<100;s++){
        output_high(pin_e2);
        delay_us(2400);
        output_low(PIN_E2);
        delay_us(15000);
    }

}

void servo_mid(){
    lcd_putc("\fMID");
    int s;
    for(s=0;s<100;s++){
        output_high(pin_e2);
        delay_us(1200);
        output_low(PIN_E2);
        delay_us(15000);
    }

}
void obs_avoid(){


}

//----------------------------------------------------------ISR Functions-----------------------------------------------------------------------


void isr_back(unsigned int16 time){
    
    //    disable_interrupts(GLOBAL);
    //    disable_interrupts(INT_RTCC);
    //---------------Enable Motors-----------------------
    output_high(PIN_C0);//Enable (Right)Motor 2 input 3 L293D
    output_high(PIN_D1);//Enable (Left)Motor 2 input 3 L293D

    //----------------accelerating motors-----------
    
    set_pwm1_duty(106);
    set_pwm2_duty(110);
    //delay_ms(10);
    //printf(lcd_putc,"\f\fAccelerating\n \t Motors");
    

    //lcd_putc("\fMotors running\n@ full speed");
    delay_ms(time);
    isr_stop();



}
void isr_stop(){

    //    disable_interrupts(GLOBAL);
    //    disable_interrupts(INT_RTCC);

    int i,init=duty1,dt;

    dt=(3000)/init;

    for(i=init;i<=0;i--){
        //------------------Braking motors-----------------
        set_pwm1_duty(i);
        set_pwm2_duty(i);
        delay_ms(dt);
    }

    lcd_putc("\fStop");
    output_low(PIN_D0);//disable enable pin1 in L293D
    output_low(PIN_C0);//Disable enable pin2 in L293D
    output_low(PIN_C3);
    output_low(PIN_D1);
    return;

}

//--------------------------Function for turning LHS----------------------------
void isr_left(unsigned int16 time){

    //    disable_interrupts(GLOBAL);
    //    disable_interrupts(INT_RTCC);
    // duty1=duty2=0x00;
    output_high(PIN_D0);//Enable enable pin1 in L293D
    //output_high(PIN_C0);//Enable enable pin2 in L293D

    set_pwm2_duty(128);
    set_pwm1_duty(0);


    lcd_putc("\fTurning Left");
    delay_ms(time);

    isr_stop();
    turn=0;



}

//--------------------------Function for turning RHS----------------------------

void isr_right(unsigned int16 time){

    //    disable_interrupts(GLOBAL);
    //    disable_interrupts(INT_RTCC);

    //output_high(PIN_D1);//Enable enable pin1 in L293D
    output_high(PIN_C3);//Enable enable pin2 in L293D

    set_pwm1_duty(118);
    set_pwm2_duty(0);


    lcd_putc("\fTurning Right");
    delay_ms(time);

    isr_stop();
    turn=1;
}


void isr_fwd(unsigned int16 time){
    //    disable_interrupts(GLOBAL);
    //    disable_interrupts(INT_RTCC);

    //int i,init=duty1,dt=0;//dt means delay time

    output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
    output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
    set_pwm1_duty(106);
    set_pwm2_duty(110);

    lcd_putc("\fFWD");
    delay_ms(time);//keep the accelarated speed until "Time*1000" seconds
    isr_stop();

}



//!void music(){
//!    int i;
//!
//!    while(TRUE)
//!    {
//!        for(i=0; i<SIZE; ++i)
//!        {
//!            generate_tone(happy_bday[i].tone,happy_bday[i].length);
//!            delay_ms(100);
//!        }
//!    }
//!
//!
//!
//!}

char rf_Rx(unsigned int8 selection){
    if (selection==0){
        while(TRUE){

            if(input_A()!=0){
                //lcd_putc("\fReciving Data");
                delay_ms(40);
                lsb=input_A();
                delay_ms(400);
                msb=input_A();
                msb<<=4;
                ascii=lsb|msb;
                ch1=(char)ascii;
                if(ch1=='/')
                    ch1++;
                //printf(lcd_putc,"\fch1=%c",ch1);
                delay_ms(500);
                return ch1;
            }
        }
    }
    else if(selection==1){
        char cha;

        while(TRUE){

            if(input_A()!=0)//&&(input_A()!=last_char))
            {
                //lcd_putc("\fReciving Data");
                delay_ms(90);
                cha=input_A();

                switch (cha)
                {
                //Key -'1'-
                case 1:
                    last_char=1;
                    return '1';
                    break;

                    //Key -'2'-
                case 2:
                    last_char=2;
                    return '2';
                    break;

                    //Key -'3'-
                case 3:
                    last_char=3;
                    return '3';
                    break;

                    //Key -'4'-
                case 4:
                    last_char=4;
                    return '4';
                    break;

                    //Key -'5'-
                case 5:
                    last_char=5;
                    return '5';
                    break;

                    //Key -'6'-
                case 6:
                    last_char=6;
                    return '6';
                    break;

                    //Key -'7'-
                case 7:
                    last_char=7;
                    return '7';
                    break;

                    //Key -'8'-
                case 8:
                    last_char=8;
                    return '8';
                    break;

                    //Key -'9'-
                case 9:
                    last_char=9;
                    return '9';
                    break;

                    //Key -'0'-
                case 10:
                    last_char=10;
                    return '0';
                    break;

                    //Key -'A'-
                case 11:
                    last_char=11;
                    return 'A';
                    break;

                    //Key -'B'-
                case 12:
                    last_char=12;
                    return 'B';
                    break;

                    //Key -'C'-
                case 13:
                    last_char=13;
                    return 'C';
                    break;

                    //Key -'*'-
                case 14:
                    last_char=14;
                    return '*';
                    break;

                    //Key -'#'-
                case 15:
                    last_char=15;
                    return '#';
                    break;

                }
            }

        }
    }
}

void remote_c(){
    char k;
    lcd_putc("\fRobot Remote\nController Menu");
    delay_ms(500);
    unsigned int16 cunt3=0;
    while(TRUE){
        k=rf_rx(1);
        if(k=='#'){
            key_stop();
            continue;
        }
        if (input_A()== 11)
            return;
        switch (k){
        case '1':
            lcd_putc("\fBack To Main\nMenu :)");
            return;
            break;

        case '2':
            lcd_putc("\fForward");
            while(rf_rx(1)=='2'){
                output_low(PIN_C0);//Enable (Right)Motor 2 input 3 L293D
                output_low(PIN_D1);//Enable (Left)Motor 2 input 3 L293D

                output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
                output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
                set_pwm1_duty(124);
                set_pwm2_duty(127);
                cunt3++;
                delay_ms(15);
            }
            break;


        case '4':
            lcd_putc("\fLeft");
            output_high(PIN_D0);//Enable enable pin1 in L293D
            set_pwm2_duty(110);
            set_pwm1_duty(0);

            break;


        case '6':
            lcd_putc("\fRight");
            output_high(PIN_C3);//Enable enable pin2 in L293D
            set_pwm1_duty(106);
            set_pwm2_duty(0);

            break;


        case '8':
            lcd_putc("\fBack");
            output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
            output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
            output_high(PIN_C0);//Enable (Right)Motor 2 input 3 L293D
            output_high(PIN_D1);//Enable (Left)Motor 2 input 3 L293D
            set_pwm1_duty(106);
            set_pwm2_duty(110);
            break;

        case '#':
            lcd_putc("\fBreak");
            isr_stop();
            break;

        case 'B':
            startRoller();
            break;

        case 'C':
            stopRoller();
            break;


            //        case 'A':
            //            isr_stop();
            //            return;
            //            break;

        default:
            printf(lcd_putc,"\fWrong Input\nChar=%c:%u",k,k);
            delay_ms(1000);
        }
    }

}

long int scanf_remote(){//Read A Digit From Keyboard
    char ch,zero='0';
    unsigned int32 number=0,in;
    int count=0;
    

    //----------------------------------------------
    while(TRUE){
        ch=rf_rx(0);

        if(ch!=0){

            if(ch=='#'&&count>0){
                lcd_putc("\b \b");
                count--;
                number=(number-in)/10;
                continue; }

            if(ch>='0' && ch<='9'){
                count++;
                lcd_putc(ch);
                //----Convert Character Digits to Integer values by deducting ASCII '0'----
                in=ch-zero;
                number=(number*10)+in;


            }

            if(ch=='*')
                return number;
        }

    }


}

void remote(){
    char k;
    lcd_putc("\fWelcome To FiTZ\nRemote Menu");
    while (TRUE) {
        //-----------------Read Values From keyboard---------
        k=rf_rx(0);

        if(k!=0)
            switch (k){
            case '1':
                lcd_putc("\fRemote Cleaning\nStarted");

                int selection;
re_enter1:
                lcd_putc("\fEnter Width Of The \nRoom:       (cm)");
                lcd_gotoxy(6,2);
                width=scanf_remote();
                printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",width);
re_select1:
                delay_ms(1000);
                printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
                selection=rf_rx(0);
                if(selection=='#')
                    goto re_enter1;
                else if(selection=='*'){
                    lcd_putc("\f");
                    //break;
                }

                else {
                    lcd_putc("\fInvalid Input :(");
                    goto re_select1;
                }
re_enter2:
                lcd_putc("\fEnter length Of \nThe Room:   (cm)");
                lcd_gotoxy(10,2);
                length=scanf_remote();
                printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",length);
re_select2:
                delay_ms(1000);
                printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
                selection=rf_rx(0);
                if(selection=='#')
                    goto re_enter2;
                else if(selection=='*'){
                    lcd_putc("\f");
                    //break;
                }

                else {
                    lcd_putc("\fInvalid Input :(");
                    goto re_select2;
                }


                lcd_putc("\fCalculating....");
                //   proAnimation();
                printf(lcd_putc,"\fStart Cleaning");
                delay_ms(500);

                main_algo(width,length);

                break;

            case '2':
                remote_c();
                break;

            case '3':
                lcd_putc("\fRestart CPU :)");
                reset_cpu();
                break;

            case '4':
                char select,cha;
                int8 mode;

loop_rf:
                lcd_putc("\fEnter Reciving\nmode:");
                select=keybd();
                if(select=='A')
                    mode=0;
                else if(select=='B')
                    mode=1;
                lcd_putc("\fWating For Data");
                cha=rf_rx(mode);
                printf(lcd_putc,"\fchar=%c:%u",cha,cha);
                delay_ms(1000);
                goto loop_rf;
                break;
            }
    }
}

void int_fwd(unsigned long time){
    
    float numberofloops,ext_fwd=0;
    numberofloops = time/67;
    inew=0;
    printf(lcd_putc,"\fmtrs uptime:%Lu\nloops:%f",time,numberofloops);
    delay_ms(1000);
    //numberofloops = numberofloops%15;

    printf(lcd_putc,"\fmtrs uptime:%Lu\nloops:%f",time,numberofloops);
    delay_ms(960);
    distance();
    output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
    output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
    set_pwm1_duty(106);//125 good values 116
    set_pwm2_duty(110);//128 good values 120
    int_value =up_time - (25/speed);
    //clear_interrupt(GLOBAL);
    enable_interrupts(INT_RTCC);
    enable_interrupts(GLOBAL);

    while(inew<numberofloops){

        delay_ms(15);

        inew++;

        printf(lcd_putc,"\fi= %f i=%Lu \ntic=%f",numberofloops,inew,tic*speed);
        ext_fwd=isr();
        if(inew<10)
            continue;
        //isr();
        if(input_A()==15){
            remote_c();
            return;
        }
        if(ext_fwd!=0){
            numberofloops = numberofloops - ext_fwd;
            ext_fwd = 0;
            
        }

    }
    if(distance()>20){
        lcd_putc("\fI'm working :)");
        //delay_ms(2000);
        output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        set_pwm1_duty(106);
        set_pwm2_duty(110);
        while(distance()>10);
        key_stop();
    }

    //key_stop();
    direction=0;
    //clear_interrupt(GLOBAL);
    disable_interrupts(INT_RTCC);
    disable_interrupts(GLOBAL);
    tic = 0;

}






float isr(){
    if(distance() > 20.0)
        return 0;

    if(tic >= int_value)
        return 0;
    disable_interrupts(INT_RTCC);
    disable_interrupts(GLOBAL);
    unsigned int lhs_distance,rhs_distance;
    isr_stop();
obs_check:
    servo_lhs();
    delay_ms(500);
    lhs_distance = distance();
    servo_rhs();
    delay_ms(500);
    rhs_distance = distance();
    if((lhs_distance <20.0) && (rhs_distance< 20.0)){
        isr_back(1000);
        goto obs_check;
    }
    cunt1 =0;
    cunt2 = 0;
    key_fwd(1000);
    delay_ms(900);
    key_back(1000);
    delay_ms(1000);
    if(lhs_distance >= rhs_distance){
        stopRoller();
        isr_left(T90);
        delay_ms(300);
        startRoller();
        servo_rhs();
        delay_ms(500);
        output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        set_pwm1_duty(106);
        set_pwm2_duty(110);
        while(distance()<35.0){
            delay_ms(100);
            cunt1++;
        }
        //key_stop();
        lcd_putc("\fFunctionpas");//hv to remove only 4f debuging
        delay_ms(3500);
        int16 i;
        for(i=128;i>=10;i--){
            set_pwm1_duty(i);
            set_pwm2_duty(i-5);
            delay_ms(10);
        }
        output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        //isr_fwd(2000);
        stopRoller();
        delay_ms(600);
        isr_right(T90r);
        delay_ms(500);
        output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        set_pwm1_duty(106);
        set_pwm2_duty(110);
        startRoller();

        delay_ms(1000);

        while(distance()<35.0){
            delay_ms(100);
            cunt2++;
        }
        //key_stop();
        delay_ms(4000);
        for(i=128;i>=10;i--){
            set_pwm1_duty(i);
            set_pwm2_duty(i-5);
            delay_ms(10);
        }
        output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        isr_stop();
        //isr_fwd(2000);
        stopRoller();
        delay_ms(600);
        isr_right(T90r);
        delay_ms(800);
        startRoller();
        isr_fwd((cunt1*100) + 4000);
        delay_ms(300);
        stopRoller();
        delay_ms(600);
        isr_left(T90);
        delay_ms(600);
        startRoller();

    }


    //-----------RHS obs-------------------

    else if(rhs_distance >= lhs_distance){
        stopRoller();
        isr_right(T90);
        delay_ms(300);
        startRoller();
        servo_lhs();
        delay_ms(500);
        output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        set_pwm1_duty(106);
        set_pwm2_duty(110);
        while(distance()<45.0){
            delay_ms(100);
            cunt1++;
        }
        //key_stop();
        lcd_putc("\fFunctionpas");//hv to remove only 4f debuging
        delay_ms(5800);
        int16 i;
        for(i=128;i>=10;i--){
            set_pwm1_duty(i);
            set_pwm2_duty(i-5);
            delay_ms(10);
        }
        output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        //isr_fwd(2000);
        stopRoller();
        delay_ms(600);
        isr_left(T90r);
        delay_ms(500);
        output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        set_pwm1_duty(106);
        set_pwm2_duty(110);

        delay_ms(1000);
        startRoller();
        while(distance()<48.0){
            delay_ms(100);
            cunt2++;
        }
        //key_stop();
        delay_ms(6000);
        for(i=128;i>=10;i--){
            set_pwm1_duty(i);
            set_pwm2_duty(i-5);
            delay_ms(10);
        }
        output_low(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
        output_low(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
        isr_stop();
        //isr_fwd(2000);
        stopRoller();
        delay_ms(600);
        isr_left(T90r);
        delay_ms(800);
        startRoller();
        isr_fwd((cunt1*100) + 4000);
        delay_ms(300);
        stopRoller();
        delay_ms(600);
        isr_right(T90);
        delay_ms(600);
        startRoller();

    }


    servo_mid();
    //-------------------------------------
    float return_value;
    float temp;
    //    if(cunt1!=0)
    //    return_value = ((cunt1*100)/15)+(333);
    //    else if(cunt2!=0)
    return_value = ((cunt2*100 + 6000)/67);
    temp = ((cunt2*100 + 6000)/13.10);
    tic += temp;
    printf(lcd_putc,"\fre= %f",return_value);
    delay_ms(3000);
    output_high(PIN_C3);//Enable (Right)Motor 2 input 3 L293D
    output_high(PIN_D0);//Enable (Left)Motor 2 input 3 L293D
    set_pwm1_duty(106);
    set_pwm2_duty(110);
    enable_interrupts(INT_RTCC);
    enable_interrupts(GLOBAL);
    return return_value;
    isr_fwd((tic+cunt2)-up_time);
    output_low(PIN_D0);//disable enable pin1 in L293D
    output_low(PIN_C0);//Disable enable pin2 in L293D
    output_low(PIN_C3);
    output_low(PIN_D1);


}

void obs_function(){
    unsigned int8 obstracle_level;
    lcd_putc("\fSet Cleaner Mode\nObstracle level");
    lcd_putc("\f1.Automatic OAS\n2.Manual OAS:");
    obstracle_level=(unsigned int8)scanf2();
    if(obstracle_level==1)
        write_eeprom(2,1);
    else if(obstracle_level==2)
        write_eeprom(2,2);
    else if(obstracle_level == 0){
        lcd_putc("\fNo Changes");
        delay_ms(1500);
        return;
    }
    lcd_putc("\fSaving....\n");
    delay_ms(500);
    lcd_putc("Restarting Robot");
    delay_ms(500);
    reset_cpu();
}
/*
2011 - 2013 © FiTz


  */
