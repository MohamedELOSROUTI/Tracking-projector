/*
 * File:   Int_Project.c
 * Author: Martin Spits
 *
 * Created on 24 avril 2019, 21:33  
 */

//TIMER0 => Utilisé comme une interupt : Fréquence d'overflow = fréquence de correction du buck
//TIMER1 => CCP1 : Utiliser comme compteur pour calculer distance avec CCP1 en capture mode
//TIMER2 => CCP2 et 3 : Utiliser pour set le PWM freq

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <htc.h>

#define _XTAL_FREQ 32000000

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF       // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

int V_buck;
int Error;
int Error_prec = 0;
int Error_I = 0;
int Error_tot = 0;
int Angle = 0;
int AngleStart = 0;
char Distance1 = 2;
char Distance2 = 2;
char Distance_prec1 = 2;
char Distance_prec2 = 2;
int Error_distance_tot = 0;
char i = 1 ; 
char Sensor_num = 1;
unsigned Anti_rebond = 0;
//#define Vref  1065; // 1.3 Volt de ref     (2^12 = 4096)
//#define Kp  1;
//#define Ki  1;
//#define Kd  2;
//const int Vref = 1065; // 1.3 Volt de ref     (2^12 = 4096)
//const int Kp = 1;
//const int Ki = 1;
//const int Kd = 2;

void Init_ADC() {
    TRISDbits.TRISD0 = 1; //Set RD0 pin as Input.
    ANSELDbits.ANSD0 = 1; //Set RD0 pin in analog mode.
    WPUDbits.WPUD0 = 0; //Weak pull-up in RD0 is disabled.
    ADCON0 = 0b00000001; // ADRMD 12_bit_mode; GO_nDONE stop; ADON enabled; CHS AN0; 
    ADCON1 = 0b01010000; // ADFM sign_magnitude; ADNREF VSS; ADPREF FVR; ADCS FOSC/4;
    ADCON2 = 0b00001111; // TRIGSEL disabled; 
    PIR1bits.ADIF = 0; // Disabling ADC interrupt flag.
    PIE1bits.ADIE = 1; // Enabling ADC interrupt.
}
//ADC no interrupt

unsigned int ADC_Conversion(unsigned char AnalogChannel) {
    ADCON0bits.CHS = 21; //Select the A/D channel
    ADCON0bits.ADON = 1; //Turn on the ADC module
    __delay_us(7); //Acquisition time delay
    ADCON0bits.GO_nDONE = 1; //Start the conversion
    while (ADCON0bits.GO_nDONE) {
        ;
    }
    ADCON0bits.ADON = 0; //Turn off the ADC module
    unsigned int ValADC = (int) ((int) (ADRESH << 4) +(int) (ADRESL >> 4));
    return ValADC; //Conversion finished, return the result
} //End of ADC_Conversion

//ADC WITH interrupt

void ADC_Conv_isr(unsigned char AnalogChannel) {
    ADCON0bits.CHS = AnalogChannel; //Select the A/D channel
    ADCON0bits.ADON = 1; //Turn on the ADC module
    __delay_us(7); //Acquisition time delay
    ADCON0bits.GO_nDONE = 1; //Start the conversion
}

void US_Pulse() {
    CCP1CON = 0b00000101; //detect risig edge       
    if (Sensor_num == 0) {
        Sensor_num = 1;
        APFCONbits.CCP1SEL = 0;
        LATCbits.LATC3 = 1;
        __delay_us(15);
        LATCbits.LATC3 = 0;
    } else {
        Sensor_num = 0;
        APFCONbits.CCP1SEL = 1;
        LATCbits.LATC4 = 1;
        __delay_us(15);
        LATCbits.LATC4 = 0;
    }
    CCP1CON = 0b00000101; //detect risig edge       
    PIE1bits.CCP1IE = 1;
    PIR1bits.CCP1IF = 0;
}

void Use_Motor(char direction) {
    if (direction == 0) {
        LATEbits.LATE1 = 0;
        LATEbits.LATE2 = 1;
    } else {
        LATEbits.LATE2 = 0;
        LATEbits.LATE1 = 1;
    }
}

void Calcul_Vitesse() {
    if ((Distance2 - 15 < Distance1) && (Distance2 + 15 > Distance1)) {
        CCPR3L = 30;
    } else {

        if (Distance1 < Distance2) {
             // CCPR3L =  165 +((Distance2 - Distance1)>>4);
            CCPR3L = 170;
            Use_Motor(0);
        } else {
            //  CCPR3L =  150+ (Distance1 - Distance2) / 4;
            //CCPR3L =  165 +((Distance1 - Distance2)>>4);
             CCPR3L = 170;
            Use_Motor(1);
        }

    }
    if (Angle > 2700) {
        Use_Motor(0);
    }
    if (Angle < 1300) {
        Use_Motor(1);
    }
    //    if(Error_distance_tot1 > 1000){
    //        Use_Motor(1);
    //    }
    //    if(Error_distance_tot1 < -1000){
    //        Use_Motor(0);
    //    }


}

void Init_Pin() {
    OSCCON = 0b11111000;
    OSCTUNE = 0b00000000;

    //RB INPUT  WITH NO ANALOG PIN (must initialize or it bugs)
    TRISB = 0xFF;
    ANSELB = 0b00100000;

    // RD INPUT    RD0 = Read V_buck
    TRISD = 0xFF;
    ANSELD = 0b00000010;

    //RC OUTPUT (Sauf RC2)
    TRISC = 0b00000100;
    ANSELC = 0b00000000;
    LATC = 0xFF;

    TRISE = 0b00000000;
    ANSELE = 0b00000000;

}

void reset() {
    while ((Angle > 2040) || (Angle < 1960)) {
        ADC_Conv_isr(13);
        INTCONbits.TMR0IF = 0;
        TMR0 = 0;
        CCPR3L = 160;

        if (Angle > 2040) {
            Use_Motor(0);
        }
        if (Angle < 1960) {
            Use_Motor(1);
        }
        __delay_ms(20);
    }
}

void main() {

    //output
    Init_Pin();
    //ADC
    Init_ADC();

    //Interupt timer 0 pour l'ADC
    INTCON = 0b11100001;
    OPTION_REG = 0b10000000; //prescaler of 16
    TMR0 = 0x00;

    // AngleStart = ADC_Conversion(13);
    //TIMER 2 pour PWM freq
    T2CON = 0b00000100;
    PIR1bits.TMR2IF = 0;
    PR2 = 255; //30 khz PWM

    //PWM BUCK: 
    APFCON1bits.CCP2SEL = 0; //0 :RC1   1 :RB3
    CCP2CON = 0b00001111;
    CCPR2L = 80;

    //PWM Motor
    APFCON2bits.CCP3SEL = 0; // 1 : RB5    0 : RE0
    CCP3CON = 0b00001111;
    CCPR3L = 30; // pwm de 0 au départ

    // CAPTURE US
    T1CON = 0b00010001; // Fosc/4   Prescale de 2 pour pouvoir prendre toute la pulse
    APFCONbits.CCP1SEL = 1; // CCP1 on 1: RB0   0 : RC2
    CCP1CON = 0b00000100; // 0101 capture every risng edge  0100 every falling
    PIR1bits.CCP1IF = 0;
    PIE1bits.CCP1IE = 1;

    while (1) {
        if (PORTBbits.RB4 == 0) {
            Anti_rebond = ~Anti_rebond;
            __delay_ms(100);
        }
        if (Anti_rebond == 0) {
            reset();
            CCPR3L = 30;
            ADC_Conv_isr(13);
            INTCONbits.TMR0IF = 0;
            TMR0 = 0;
            CCPR2L = 0;
        } else {
            for (i = 1; i < 6; i++) {
                US_Pulse();
                Calcul_Vitesse();
                __delay_ms(25);
            }

            Distance1 = 230;
            Distance2 = 230;
            //mesure angle 

            ADC_Conv_isr(13);
            INTCONbits.TMR0IF = 0;
            TMR0 = 0;
        }
    }
}

void __interrupt() ISR(void) {
    //capture mode of falling edge of rising edge


    if (PIR1bits.CCP1IF == 1) {
        // !!!! Sens des interrupts important !!!
        //detect falling
        if (CCP1CON == 0b00000100) {
            PIR1bits.CCP1IF = 0;
            T1CONbits.TMR1ON = 0;
            TMR1 = 0;
            //Allume une led avec PWM de 0 a 130
            if (Sensor_num == 0) {
                //mettre distance en char car 8 bits
                // if((Distance_prec1 - 50 < CCPR1H) && (Distance_prec1 + 50 > CCPR1H)){
                Distance1 = CCPR1H;
                //  }
            } else {
                //mettre distance en char car 8 bits
                //     Error_distance_tot1 = Distance2;
                // if ((Distance_prec2 - 50 < CCPR1H) && (Distance_prec2 + 50 > CCPR1H)){
                Distance2 = CCPR1H;
                //}
            }
            // Error_distance_tot1 = Error_distance_tot1 + (Distance1 - Distance2);
            // Distance = Distance>>8;
            //distance max = 3m duty max quand CCPR3L = 130
            PIE1bits.CCP1IE = 0;
        }//plutot mettre else ici pour gagner du temps
            //detect rising
        else {
            TMR1 = 0;
            T1CONbits.TMR1ON = 1;
            CCP1CON = 0b00000100;
            PIR1bits.CCP1IF = 0;
            PIE1bits.CCP1IE = 1;
        }
    }






    if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
        ADC_Conv_isr(21);
        INTCONbits.TMR0IF = 0;
        TMR0 = 0;
    }






    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        if (ADCON0bits.CHS == 21) {
            V_buck = (int) ((int) (ADRESH << 4) +(int) (ADRESL >> 4));
            ADCON0bits.ADON = 0; //Turn off the ADC module
           // Error_tot = 1065 - V_buck; //V ref = 1065
             //Error_I = Error_I + (Error >> 3);
             // Error_tot = Error; // + Error_I>>3 + (Error - Error_prec);
            //Une erreur de 1V => 0.15 V sur le pont = Une erreur de 133
            // Error_tot = Error_tot >> 5; //au lieu de diviser par 50
//                    if ((CCPR2L + Error_tot/50 >= 0)&& (CCPR2L + Error_tot/50 < 256)) {
//                        CCPR2L = CCPR2L + Error_tot/50;
//                   }
            if (V_buck > 1065) {
                CCPR2L--;
            } else {
                CCPR2L++;
            }
        }
        if (ADCON0bits.CHS == 13) {

            Angle = (int) ((int) (ADRESH << 4) +(int) (ADRESL >> 4));
            ADCON0bits.ADON = 0; //Turn off the ADC module
            //Error_prec = Error;
        }
    }
}