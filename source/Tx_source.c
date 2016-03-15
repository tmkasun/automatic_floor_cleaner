/* © University Of Moratuwa all rights received 2012 visit http://www.itfac.mrt.ac.lk */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
-------------------------------Messages For Developers---------------------------------------------
This c source file for project FiTz Remote controller Transmiter Source code

Notes for developers:

                        Increse the speed of character transmition
                        encode '0' for transmition
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <16F877A.h>
#device adc=8
#use delay(clock=20000000)
#fuses  HS,NOWDT,NOPROTECT,NOLVP
#include <stdlib.h>
#include <stdio.h>
#include "f_LCD3.c" 
#include "f_kbd.c" 
#USE FAST_IO(A)

void remote_c();
void service();
void rf_tx(char cha,unsigned int8 selection);
void rf_tx_control(char cha);
long int scanf2();//Read A Digit From Keyboard
char keybd();//Read keypad and return a character value from it
void startclening();//Start Cleaning Process
char ch;
char ch1=0;//character from the RF module 
unsigned int8 ascii,lsb,msb;//least significant bit and mostsignificant bit of the  ascii character of the reciving character
long int width,length;//Width of the room and length of the room
static char last_char;

void main(){
    char select,controller_mode;
    int8 mode,i;//for mode selection and PWM animation
    //!   setup_adc_ports(NO_ANALOGS);
    //!   setup_psp(PSP_DISABLED);
    //!   setup_spi(SPI_SS_DISABLED);
    //!   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
    //!   setup_timer_1(T1_DISABLED);

    setup_timer_2(T2_DIV_BY_16,128,1);  //for
    setup_ccp1(CCP_PWM);   //LCD backlight
    lcd_init();
    set_tris_a(0);
    kbd_init();

    lcd_putc("\fWelcome To FiTZ\nRemote Controller");
    for(i=0;i<128;i++){
        set_pwm1_duty(i);
        delay_ms(20);
    }//Animation
    delay_ms(100);
mode_selection:
    lcd_putc("\fSelect Controller\nMode");
    delay_ms(1000);
    lcd_putc("\fRemote||Manual\n   1  ||   2 :");
    controller_mode=keybd();
    if(controller_mode=='1'||controller_mode=='2'){
        rf_tx(controller_mode,0);
    }
    else{
        printf(lcd_putc,"\fSorry %c is a\nWrong Selection",controller_mode);
        delay_ms(700);
        goto mode_selection;
    }


    char k;
    lcd_putc("\fWelcome To FiTZ\nRemote Menu");
    delay_ms(500);
    lcd_putc("\f1.Remote Cleaner\n2.Remote Control");
    while (TRUE) {
        //-----------------Read Values From keyboard---------
        k=keybd();
        rf_tx(k,0);
        if(k!=0)
            switch (k){
            case '1':
                lcd_putc("\fRemote Cleaning\nStarted");
                delay_ms(500);

                int selection;
reenter1:
                lcd_putc("\fEnter Width Of The \nRoom:       (cm)");
                lcd_gotoxy(6,2);
                width=scanf2();
                printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",width);
reselect1:
                delay_ms(1000);
                printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
                selection=keybd();
                rf_tx(selection,0);
                if(selection=='#')
                    goto reenter1;
                else if(selection=='*'){
                    lcd_putc("\f");
                    //break;
                }

                else {
                    lcd_putc("\fInvalid Input :(");
                    goto reselect1;
                }
reenter2:
                lcd_putc("\fEnter length Of \nThe Room:   (cm)");
                lcd_gotoxy(10,2);
                length=scanf2();
                printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",length);
reselect2:
                delay_ms(1000);
                printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
                selection=keybd();
                rf_tx(selection,0);
                if(selection=='#')
                    goto reenter2;
                else if(selection=='*'){
                    lcd_putc("\f");
                    //break;
                }

                else {
                    lcd_putc("\fInvalid Input :(");
                    goto reselect2;
                }


                lcd_putc("\fCalculating....");
                //   proAnimation();
                printf(lcd_putc,"\fStart Cleaning");
                delay_ms(500);
                remote_c();
                break;

            case '2':
                remote_c();
                break;

            case '3':
                lcd_putc("\fRestart CPU :)");
                break;

            case '4':
                while(TRUE){
                    lcd_putc("\fEnter Transfer\nmode:");
                    select=keybd();
                    if(select=='A')
                        mode=0;
                    else if(select=='B')
                        mode=1;


                    ch=keybd();
                    printf(lcd_putc,"\fASCII CM:%c: %u",ch,ch);
                    rf_tx(ch,mode);

                }
                break;

            }
    }




}




char keybd(){

    //-----------A tempory character -------------
    char k;
    while(TRUE) {
        //-----------------Read Values From keyboard---------
        k=kbd_getc();

        if(k!=0) {
            lcd_putc(k);
            return k;
        }
    }



}

void remote_c(){
    lcd_putc("\fRemote Control\nMode :)");
    delay_ms(800);
    lcd_putc("\fPress '#' to\ntake the control");
    delay_ms(1000);
    while(TRUE){
        ch=keybd();
        printf(lcd_putc,"\fASCII CM:%c: %u",ch,ch);
        rf_tx(ch,1);
        if(ch=='1'){

            OUTPUT_A(0);
            delay_ms(100);
            rf_tx('1',0);
            return;
        }
    }
    lcd_putc("\fBack To Main\nMenu :)");

}
void rf_tx(char cha,unsigned int8 selection){
    if(selection==0){
        if(cha=='0')
            cha--;
        ascii=(unsigned int8)cha;
        lsb=15&ascii;
        msb=240&ascii;
        msb>>=4;
        OUTPUT_A(lsb);
        delay_ms(200);
        OUTPUT_A(0);
        delay_ms(200);
        OUTPUT_A(msb);
        delay_ms(200);
        OUTPUT_A(0);
    }

    else if(selection==1){
        switch (cha)
        {
        //Key -'1'-
        case '1':
            OUTPUT_A(1);
            break;

            //Key -'2'-
        case '2':
            OUTPUT_A(2);
            break;

            //Key -'3'-
        case '3':
            OUTPUT_A(3);
            break;

            //Key -'4'-
        case '4':
            OUTPUT_A(4);
            break;

            //Key -'5'-
        case '5':
            OUTPUT_A(5);
            break;

            //Key -'6'-
        case '6':
            OUTPUT_A(6);
            break;

            //Key -'7'-
        case '7':
            OUTPUT_A(7);
            break;

            //Key -'8'-
        case '8':
            OUTPUT_A(8);
            break;

            //Key -'9'-
        case '9':
            OUTPUT_A(9);
            break;

            //Key -'0'-
        case '0':
            OUTPUT_A(10);
            break;

            //Key -'A'-
        case 'A':
            OUTPUT_A(11);
            break;

            //Key -'B'-
        case 'B':
            OUTPUT_A(12);
            break;

            //Key -'C'-
        case 'C':
            OUTPUT_A(13);
            break;

            //Key -'*'-
        case '*':
            OUTPUT_A(14);
            break;

            //Key -'#'-
        case '#':
            OUTPUT_A(15);
            break;

        default:
            OUTPUT_A(0);
        }

    }
}
void rf_tx_control(char cha){
    switch (cha)
    {
    //Key -'1'-
    case '1':
        OUTPUT_A(1);
        break;

        //Key -'2'-
    case '2':
        OUTPUT_A(2);
        break;

        //Key -'3'-
    case '3':
        OUTPUT_A(3);
        break;

        //Key -'4'-
    case '4':
        OUTPUT_A(4);
        break;

        //Key -'5'-
    case '5':
        OUTPUT_A(5);
        break;

        //Key -'6'-
    case '6':
        OUTPUT_A(6);
        break;

        //Key -'7'-
    case '7':
        OUTPUT_A(7);
        break;

        //Key -'8'-
    case '8':
        OUTPUT_A(8);
        break;

        //Key -'9'-
    case '9':
        OUTPUT_A(9);
        break;

        //Key -'0'-
    case '0':
        OUTPUT_A(10);
        break;

        //Key -'A'-
    case 'A':
        OUTPUT_A(11);
        break;

        //Key -'B'-
    case 'B':
        OUTPUT_A(12);
        break;

        //Key -'C'-
    case 'C':
        OUTPUT_A(13);
        break;

        //Key -'*'-
    case '*':
        OUTPUT_A(14);
        break;

        //Key -'#'-
    case '#':
        OUTPUT_A(15);
        break;

    default:
        OUTPUT_A(0);
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
            rf_tx(ch,0);
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
    selection=keybd();
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
    lcd_putc("\fEnter length Of \nThe Room:   (cm)");
    lcd_gotoxy(10,2);
    length=scanf2();
    printf(lcd_putc,"\f%s\n   %Lucm","You have Entered",length);
reselect2:
    delay_ms(1000);
    printf(lcd_putc,"\f%s\nkey(*)||key(#)","Continue#ReEnter");
    selection=keybd();
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
    rf_tx('1',0);
    return;

}

