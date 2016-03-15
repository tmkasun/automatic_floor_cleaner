/* © University Of Moratuwa all rights received 2012 visit http://www.itfac.mrt.ac.lk */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
-------------------------------Messages For Developers---------------------------------------------
This c source file for project FiTz
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <18F452.h>
#device adc=8
#DEVICE HIGH_INTS=TRUE
#use delay(clock=20000000)

#fuses  HS,NOWDT,NOPROTECT,NOLVP
#include <stdlib.h>
#include <stdio.h>
#include "f_LCD3.c" 
#include "f_kbd.c" 
#USE FAST_IO(A)
char ch=0;
unsigned int8 ascii,lsb,msb;
char keybd();
char rf_Rx(unsigned int8 selection);//0=ASCII mode 1=Fast mode
char rf_rx_control(void);
//----------------Interrupt service routing for timer1 overflow-------------------
//!#int_rb //HIGH
//!void TIMER0_isr(void){//Timer 1 Interrupt Service Routing
//!}

void main(){
char select;
int8 mode;
 setup_timer_0(RTCC_INTERNAL|RTCC_DIV_64|RTCC_8_BIT);//for INT_TIMER0
 //enable_interrupts(INT_RB);
 //enable_interrupts(GLOBAL);
 set_tris_a(63);
lcd_init();
kbd_init();

lcd_putc("\fwating 4 data");
while(TRUE){
lcd_putc("\fEnter Reciving\nmode:");
select=keybd();
if(select=='A')
mode=0;
else if(select=='B')
mode=1;
ch=rf_rx(mode);
printf(lcd_putc,"\fchar=%c:%u",ch,ch);
delay_ms(1000);
}
   
      }
char rf_Rx(unsigned int8 selection){
if (selection==0){
while(TRUE){

if(input_A()!=0){
lcd_putc("\fReciving Data");
delay_ms(90);
lsb=input_A();
delay_ms(1800);
msb=input_A();
msb<<=4;
ascii=lsb|msb;
ch=(char)ascii;
return ch;
}
   }
      }
else if(selection==1){
char cha;
static char last_char;
while(TRUE){

if((input_A()!=0)&&(input_A()!=last_char)){
            lcd_putc("\fReciving Data");
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

char rf_rx_control(void){
char cha;
static char last_char;
while(TRUE){

if((input_A()!=0)&&(input_A()!=last_char)){
            lcd_putc("\fReciving Data");
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

