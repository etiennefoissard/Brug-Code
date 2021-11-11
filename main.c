/*
 */

#include <avr/io.h>
#include <util/delay.h>
#include "servo.h"


#define GROEN1 PA0  // stoplicht
#define GROEN2 PA2  // stoplicht
#define GROEN3 PL0  // rechts
#define GROEN4 PA5
#define GROEN5 PC7
#define GROEN6 PC6
#define GROEN7 PC3  // links
#define GROEN8 PL1
#define GROEN9 PD7
#define GROEN10 PG2
#define ROOD11 PH4  // stoplicht
#define ROOD12 PA3 // stoplicht
#define ROOD13 PA6  // rechts
#define ROOD14 PB2
#define ROOD15 PC5
#define ROOD16 PB3
#define ROOD17 PC1  // links
#define ROOD18 PC0
#define ROOD19 PG1
#define ROOD20 PG0
#define ROOD21BOOT PH0
#define ROOD22WIND PH1
#define ROOD23STOP PJ0
#define ROOD24VAAR PD3     // aan als vaarverkeer groen licht krijgt
#define ROOD25SLAGBOOM PD2
#define GEEL26 PL2
#define GEEL27 PL3
#define GEEL28 PL4
#define GEEL29 PL5
#define GEEL30OPEN PD0
#define GEEL31DICHT PD1

#define KNOP1 PK6             // noodstop
#define KNOP2 PK7         // brug open
#define KNOP3 PK5         // brug dicht
#define EINDSTOP PF2

#define IRSENSOR1 PF7   // IR sensor achter
#define IRSENSOR2 PF6   // IR sensor voor

#define BUZZER PE5

int main(void)
{
    // 31 LEDS configureren
    DDRA |= (1 << GROEN1); // groen
    DDRA |= (1 << GROEN2); // groen
    DDRL |= (1 << GROEN3); // groen
    DDRA |= (1 << GROEN4); // groen
    DDRC |= (1 << GROEN5); // groen
    DDRC |= (1 << GROEN6); // groen
    DDRC |= (1 << GROEN7); // groen
    DDRL |= (1 << GROEN8); // groen
    DDRD |= (1 << GROEN9); // groen
    DDRG |= (1 << GROEN10); // groen

    DDRH |= (1 << ROOD11); // rood
    DDRA |= (1 << ROOD12); // rood
    DDRA |= (1 << ROOD13); // rood
    DDRB |= (1 << ROOD14); // rood
    DDRC |= (1 << ROOD15); // rood
    DDRB |= (1 << ROOD16); // rood
    DDRC |= (1 << ROOD17); // rood
    DDRC |= (1 << ROOD18); // rood
    DDRG |= (1 << ROOD19); // rood
    DDRG |= (1 << ROOD20); // rood
    DDRH |= (1 << ROOD21BOOT);
    DDRH |= (1 << ROOD22WIND);
    DDRJ |= (1 << ROOD23STOP);
    DDRD |= (1 << ROOD24VAAR);
    DDRD |= (1 << ROOD25SLAGBOOM);

    DDRL |= (1 << GEEL26); // geel
    DDRL |= (1 << GEEL27); // geel
    DDRL |= (1 << GEEL28); // geel
    DDRL |= (1 << GEEL29); // geel
    DDRD |= (1 << GEEL30OPEN);
    DDRD |= (1 << GEEL31DICHT);

    PORTA &= ~(1 << GROEN1); //groen // wegverkeer
    PORTA &= ~(1 << GROEN2);
    PORTL &= ~(1 << GROEN3);         // vaarverkeer
    PORTA &= ~(1 << GROEN4);
    PORTC &= ~(1 << GROEN5);
    PORTC &= ~(1 << GROEN6);
    PORTC &= ~(1 << GROEN7);
    PORTL &= ~(1 << GROEN8);
    PORTD &= ~(1 << GROEN9);
    PORTG &= ~(1 << GROEN10);

    PORTH &= ~(1 << ROOD11); // rood // wegverkeer
    PORTA &= ~(1 << ROOD12);
    PORTA &= ~(1 << ROOD13);         // vaarverkeer
    PORTB &= ~(1 << ROOD14);
    PORTC &= ~(1 << ROOD15);
    PORTB &= ~(1 << ROOD16);
    PORTC &= ~(1 << ROOD17);
    PORTC &= ~(1 << ROOD18);
    PORTG &= ~(1 << ROOD19);
    PORTG &= ~(1 << ROOD20);
    PORTH &= ~(1 << ROOD21BOOT);
    PORTH &= ~(1 << ROOD22WIND);
    PORTJ &= ~(1 << ROOD23STOP);
    PORTD &= ~(1 << ROOD24VAAR);
    PORTD &= ~(1 << ROOD25SLAGBOOM);

    PORTL &= ~(1 << GEEL26); // geel // vaarverkeer
    PORTL &= ~(1 << GEEL27);
    PORTL &= ~(1 << GEEL28);
    PORTL &= ~(1 << GEEL29);
    PORTD &= ~(1 << GEEL30OPEN);    // Bedieningspaneel
    PORTD &= ~(1 << GEEL31DICHT);

    // configureer knop als input
    DDRK &= ~(1 << KNOP1);
    DDRK &= ~(1 << KNOP2);
    DDRK &= ~(1 << KNOP3);
    DDRF &= ~(1 << EINDSTOP);

    // buzzer
    DDRE |= (1 << BUZZER);
    PORTE &= ~(1 << BUZZER);

    // IR sensor
    DDRF &= ~(1 << IRSENSOR1);
    DDRF &= ~(1 << IRSENSOR2);

    // motor
    DDRH |= (1 << PH6);
    DDRF |= (1 << PF5);
    PORTH &= ~(1 << PH6);
    PORTF &= ~(1 << PF5);


    init_servo();

    while (1)
    {
        int noodstop = !(PINK & (1 << KNOP1));
        int brug_open = !(PINK & (1 << KNOP2));
        int brug_dicht = !(PINK & (1 << KNOP3));
        int detected1 = !(PINF & (1 << IRSENSOR1));
        int detected2 = !(PINF & (1 << IRSENSOR2));
        int ingedrukt_eindstop = !(PINF & (1 << EINDSTOP));

        // lichten van wegverkeer staan op groen, lichten van vaarverkeer staan op rood en gele lichten ook aan,
        // wegverkeer
        PORTA |= (1 << GROEN1); //groen
        PORTA |= (1 << GROEN2);

        // vaarverkeer
        PORTA |= (1 << ROOD13); // rood
        PORTB |= (1 << ROOD14);
        PORTC |= (1 << ROOD15);
        PORTB |= (1 << ROOD16);
        PORTC |= (1 << ROOD17);
        PORTC |= (1 << ROOD18);
        PORTG |= (1 << ROOD19);
        PORTG |= (1 << ROOD20);

        if(ingedrukt_eindstop)
        {
            PORTD |= (1 << GEEL31DICHT);
            PORTD &= ~(1 << GEEL30OPEN);
        }
        else
        {
            PORTD &= ~(1 << GEEL31DICHT);
            PORTD |= (1 << GEEL30OPEN);
        }


        if (detected1)         // als boot wordt , anders naar else
            {
                PORTH |= (1 << ROOD21BOOT);
                ADCSRA |= (1 << ADSC);   // ADCSRA &= ~(1 << ADSC);

                while(ADCSRA & 0b01000000);
                {
                    if (ADCH <= 52)
                    {
                        // geluid aan lichten wegverkeer op rood
                        PORTE |= (1 << BUZZER); // buzzer aan
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);
                        PORTE |= (1 << BUZZER);
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);

                        PORTH |= (1 << ROOD11); // rood
                        PORTA |= (1 << ROOD12);
                        PORTA &= ~(1 << GROEN1); //groen
                        PORTA &= ~(1 << GROEN2);

                        //slagbomen gaan dicht
                        PORTD |= (1 << ROOD25SLAGBOOM);
                        servo1_set_percentage(100);
                        servo2_set_percentage(100);
                        _delay_ms(500);

                        // brug gaat open, Geel licht op bedieningspaneel brug open aan, geel licht brug dicht uit
                        PORTF |= (1 << PF5);
                        _delay_ms(650);
                        PORTD &= ~(1 << GEEL31DICHT);           // met eindstop als het kan
                        PORTD |= (1 << GEEL30OPEN);

                        _delay_ms(1500);

                        // lampjes voor vaarverkeer op groen        // het liefst met een eindstop
                        PORTC |= (1 << GROEN5);
                        PORTC |= (1 << GROEN6);     // deze doet ook de rode led er boven aan. Rechts boven
                        PORTD |= (1 << GROEN9);
                        PORTG |= (1 << GROEN10);
                        PORTC &= ~(1 << ROOD15);
                        PORTB &= ~(1 << ROOD16);
                        PORTG &= ~(1 << ROOD19);
                        PORTG &= ~(1 << ROOD20);

                        PORTD |= (1 << ROOD24VAAR);

                        // delay van ... seconden / ms
                        _delay_ms(3000);

                        // lampen vaarverkeer gaan op rood
                        PORTC |= (1 << ROOD15);
                        PORTB |= (1 << ROOD16);
                        PORTG |= (1 << ROOD19);
                        PORTG |= (1 << ROOD20);
                        PORTC &= ~(1 << GROEN5);
                        PORTC &= ~(1 << GROEN6);
                        PORTD &= ~(1 << GROEN9);
                        PORTG &= ~(1 << GROEN10);
                        _delay_ms(200);
                        PORTD &= ~(1 << ROOD24VAAR);

                        _delay_ms(1000);

                        // brug gaat dicht
                        PORTF &= ~(1 << PF5);
                        PORTH |= (1 << PH6);
                        _delay_ms(1600);
                        PORTH &= ~(1 << PH6);

                        if(ingedrukt_eindstop)
                        {
                            PORTD |= (1 << GEEL31DICHT);
                            PORTD &= ~(1 << GEEL30OPEN);
                        }
                        else
                        {
                            PORTD &= ~(1 << GEEL31DICHT);
                            PORTD |= (1 << GEEL30OPEN);
                        }

                        _delay_ms(1000);

                        // als brug dicht is, geluid aan, slagbomen omhoog
                        PORTE |= (1 << BUZZER);
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);
                        PORTE |= (1 << BUZZER);
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);

                        servo1_set_percentage(-100);
                        servo2_set_percentage(-100);
                        _delay_ms(500);


                        // lichten wegverkeer op groen, geluid uit
                        PORTA |= (1 << GROEN1); //groen
                        PORTA |= (1 << GROEN2);
                        PORTH &= ~(1 << ROOD11); // rood
                        PORTA &= ~(1 << ROOD12);
                        PORTD &= ~(1 << ROOD25SLAGBOOM);


                        PORTH &= ~(1 << ROOD21BOOT);
                    }
                    else        //waarde boven 52, dus brug blijft dicht oftwel er gebeurt niks
                    {
                        PORTH |= (1 << ROOD22WIND);         // misschien niet aangesloten
                    }
                }
        }


        if (detected2)         // als boot wordt , anders naar else
            {
                PORTH |= (1 << ROOD21BOOT);
                ADCSRA |= (1 << ADSC);   // ADCSRA &= ~(1 << ADSC);

                while(ADCSRA & 0b01000000);
                {
                    if (ADCH <= 52)
                    {
                        // geluid aan lichten wegverkeer op rood
                        PORTE |= (1 << BUZZER); // buzzer aan
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);
                        PORTE |= (1 << BUZZER);
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);

                        PORTH |= (1 << ROOD11); // rood
                        PORTA |= (1 << ROOD12);
                        PORTA &= ~(1 << GROEN1); //groen
                        PORTA &= ~(1 << GROEN2);

                        //slagbomen gaan dicht
                        PORTD |= (1 << ROOD25SLAGBOOM);
                        servo1_set_percentage(100);
                        servo2_set_percentage(100);
                        _delay_ms(500);

                        // brug gaat open, Geel licht op bedieningspaneel brug open aan, geel licht brug dicht uit
                        PORTF |= (1 << PF5);
                        _delay_ms(650);
                        PORTD &= ~(1 << GEEL31DICHT);           // met eindstop als het kan
                        PORTD |= (1 << GEEL30OPEN);

                        _delay_ms(1500);

                        // lampjes voor vaarverkeer op groen        // het liefst met een eindstop
                        PORTL |= (1 << GROEN3);
                        PORTA |= (1 << GROEN4);     // deze doet ook de rode led er boven aan. Rechts boven
                        PORTC |= (1 << GROEN7);
                        PORTL |= (1 << GROEN8);
                        PORTA &= ~(1 << ROOD13);
                        PORTB &= ~(1 << ROOD14);
                        PORTC &= ~(1 << ROOD17);
                        PORTC &= ~(1 << ROOD18);

                        PORTD |= (1 << ROOD24VAAR);

                        // delay van ... seconden / ms
                        _delay_ms(3000);

                        // lampen vaarverkeer gaan op rood
                        PORTA |= (1 << ROOD13);
                        PORTB |= (1 << ROOD14);
                        PORTC |= (1 << ROOD17);
                        PORTC |= (1 << ROOD18);
                        PORTL &= ~(1 << GROEN3);
                        PORTA &= ~(1 << GROEN4);
                        PORTC &= ~(1 << GROEN7);
                        PORTL &= ~(1 << GROEN8);
                        _delay_ms(200);
                        PORTD &= ~(1 << ROOD24VAAR);

                        _delay_ms(1000);

                        // brug gaat dicht
                        PORTF &= ~(1 << PF5);
                        PORTH |= (1 << PH6);
                        _delay_ms(1600);
                        PORTH &= ~(1 << PH6);

                        if(ingedrukt_eindstop)
                        {
                            PORTD |= (1 << GEEL31DICHT);
                            PORTD &= ~(1 << GEEL30OPEN);
                        }
                        else
                        {
                            PORTD &= ~(1 << GEEL31DICHT);
                            PORTD |= (1 << GEEL30OPEN);
                        }

                        _delay_ms(1000);

                        // als brug dicht is, geluid aan, slagbomen omhoog
                        PORTE |= (1 << BUZZER);
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);
                        PORTE |= (1 << BUZZER);
                        _delay_ms(200);
                        PORTE &= ~(1 << BUZZER);
                        _delay_ms(200);

                        servo1_set_percentage(-100);
                        servo2_set_percentage(-100);
                        _delay_ms(500);


                        // lichten wegverkeer op groen, geluid uit
                        PORTA |= (1 << GROEN1); //groen
                        PORTA |= (1 << GROEN2);
                        PORTH &= ~(1 << ROOD11); // rood
                        PORTA &= ~(1 << ROOD12);
                        PORTD &= ~(1 << ROOD25SLAGBOOM);


                        PORTH &= ~(1 << ROOD21BOOT);
                    }
                    else        //waarde boven 52, dus brug blijft dicht oftwel er gebeurt niks
                    {
                        PORTH |= (1 << ROOD22WIND);         // misschien niet aangesloten
                    }
                }
        }
    }


    return 0;
}


