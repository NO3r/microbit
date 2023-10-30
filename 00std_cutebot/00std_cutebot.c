#include <stdio.h>
#include "nrf52833.h"
#include <nrf.h>



/*
sources:
    https://github.com/elecfreaks/circuitpython_cutebot/blob/main/cutebot.py
    https://github.com/Krakenus/microbit-cutebot-micropython/blob/master/cutebot.py
    https://github.com/bbcmicrobit/micropython/blob/master/source/microbit/microbiti2c.cpp
    https://microbit-micropython.readthedocs.io/en/latest/i2c.html#
    https://makecode.microbit.org/device/pins

I2C:
    config:
        7-bit addressing
    pins:
        freq: 100000
        SCL==micro:bit pin 19 (header)==P0.26 (nRF)
        SDA==micro:bit pin 20 (header)==P1.00 (nRF)
*/

/*
motor control
[
    motor,     // 0x01: left,    0x02: right
    direction, // 0x02: forward, 0x01: backward
    speed,     // between 0 and 100
    0,
]
*/
#define MOTOR_SPEED 20 // [0...100]
uint8_t I2CBUF_MOTOR_LEFT_FWD[]   = {0x01,0x02,MOTOR_SPEED,0};
uint8_t I2CBUF_MOTOR_LEFT_BACK[]  = {0x01,0x01,MOTOR_SPEED,0};
uint8_t I2CBUF_MOTOR_LEFT_STOP[]  = {0x01,0x02,          0,0};
uint8_t I2CBUF_MOTOR_RIGHT_FWD[]  = {0x02,0x02,MOTOR_SPEED,0};
uint8_t I2CBUF_MOTOR_RIGHT_BACK[] = {0x02,0x01,MOTOR_SPEED,0};
uint8_t I2CBUF_MOTOR_RIGHT_STOP[] = {0x02,0x02,          0,0};

/*
LED control
[
    led,       // 0x04: left,    0x08: right
    r,         // 0..255?
    g,         // 0..255?
    b,         // 0..255?
]
*/
#define LED_INTENSITY 0xff // [0x00...0xff]
uint8_t I2CBUF_LED_LEFT_WHITE[]   = {0x04,LED_INTENSITY,LED_INTENSITY,LED_INTENSITY};
uint8_t I2CBUF_LED_LEFT_RED[]     = {0x04,LED_INTENSITY,         0x00,         0x00};
uint8_t I2CBUF_LED_LEFT_GREEN[]   = {0x04,         0x00,LED_INTENSITY,         0x00};
uint8_t I2CBUF_LED_LEFT_BLUE[]    = {0x04,         0x00,         0x00,LED_INTENSITY};
uint8_t I2CBUF_LED_LEFT_OFF[]     = {0x04,         0x00,         0x00,         0x00};
uint8_t I2CBUF_LED_RIGHT_WHITE[]  = {0x08,LED_INTENSITY,LED_INTENSITY,LED_INTENSITY};
uint8_t I2CBUF_LED_RIGHT_RED[]    = {0x08,LED_INTENSITY,         0x00,         0x00};
uint8_t I2CBUF_LED_RIGHT_GREEN[]  = {0x08,         0x00,LED_INTENSITY,         0x00};
uint8_t I2CBUF_LED_RIGHT_BLUE[]   = {0x08,         0x00,         0x00,LED_INTENSITY};
uint8_t I2CBUF_LED_RIGHT_OFF[]    = {0x08,         0x00,         0x00,         0x00};

void i2c_init(void) {
   //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:   0=Input
    // .... .... .... .... .... .... .... ..B. B: INPUT: 1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:  0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE: 6=S0D1
    // .... .... .... ..EE .... .... .... .... E: SENSE: 0=Disabled
    // xxxx xxxx xxxx xx00 xxxx x110 xxxx 0010 
    //    0    0    0    0    0    6    0    2 0x00000602
    NRF_P0->PIN_CNF[26]           = 0x00000602; // SCL (P0.26)
    NRF_P1->PIN_CNF[0]            = 0x00000602; // SDA (P1.00)

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... AAAA A: ENABLE: 5=Enabled
    // xxxx xxxx xxxx xxxx xxxx xxxx xxxx 0101 
    //    0    0    0    0    0    0    0    5 0x00000005
    NRF_TWI0->ENABLE              = 0x00000005;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:    26 (P0.26)
    // .... .... .... .... .... .... ..B. .... B: PORT:    0 (P0.26)
    // C... .... .... .... .... .... .... .... C: CONNECT: 0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx01 1010 
    //    0    0    0    0    0    0    1    a 0x0000001a
    NRF_TWI0->PSEL.SCL            = 0x0000001a;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:    00 (P1.00)
    // .... .... .... .... .... .... ..B. .... B: PORT:    1 (P1.00)
    // C... .... .... .... .... .... .... .... C: CONNECT: 0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx10 0000 
    //    0    0    0    0    0    0    2    0 0x00000020
    NRF_TWI0->PSEL.SDA            = 0x00000020;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // AAAA AAAA AAAA AAAA AAAA AAAA AAAA AAAA A: FREQUENCY: 0x01980000==K100==100 kbps
    NRF_TWI0->FREQUENCY           = 0x01980000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .AAA AAAA A: ADDRESS: 16
    // xxxx xxxx xxxx xxxx xxxx xxxx x001 0000 
    //    0    0    0    0    0    0    1    0 0x00000010
    NRF_TWI0->ADDRESS             = 0x10;
}

void i2c_send(uint8_t* buf, uint8_t buflen) {
    uint8_t i;

    i=0;
    NRF_TWI0->TXD                 = buf[i];
    NRF_TWI0->EVENTS_TXDSENT      = 0;
    NRF_TWI0->TASKS_STARTTX       = 1;
    i++;
    while(i<buflen) {
        while(NRF_TWI0->EVENTS_TXDSENT==0);
        NRF_TWI0->EVENTS_TXDSENT  = 0;
        NRF_TWI0->TXD             = buf[i];
        i++;
    }
    while(NRF_TWI0->EVENTS_TXDSENT==0);
    NRF_TWI0->TASKS_STOP     = 1;
}

static uint8_t pdu[8+1] = { 0 };

int main(void) {
    
    i2c_init();
/*
    // motor left
    i2c_send(I2CBUF_MOTOR_LEFT_FWD,    sizeof(I2CBUF_MOTOR_LEFT_FWD));
    i2c_send(I2CBUF_MOTOR_LEFT_BACK,   sizeof(I2CBUF_MOTOR_LEFT_BACK));
    i2c_send(I2CBUF_MOTOR_LEFT_STOP,   sizeof(I2CBUF_MOTOR_LEFT_STOP));
    // motor right
    i2c_send(I2CBUF_MOTOR_RIGHT_FWD,   sizeof(I2CBUF_MOTOR_RIGHT_FWD));
    i2c_send(I2CBUF_MOTOR_RIGHT_BACK,  sizeof(I2CBUF_MOTOR_RIGHT_BACK));
    i2c_send(I2CBUF_MOTOR_RIGHT_STOP,  sizeof(I2CBUF_MOTOR_RIGHT_STOP));
    // led left
    i2c_send(I2CBUF_LED_LEFT_WHITE,    sizeof(I2CBUF_LED_LEFT_WHITE));
    i2c_send(I2CBUF_LED_LEFT_RED,      sizeof(I2CBUF_LED_LEFT_RED));
    i2c_send(I2CBUF_LED_LEFT_GREEN,    sizeof(I2CBUF_LED_LEFT_GREEN));
    i2c_send(I2CBUF_LED_LEFT_BLUE,     sizeof(I2CBUF_LED_LEFT_BLUE));
    i2c_send(I2CBUF_LED_LEFT_OFF,      sizeof(I2CBUF_LED_LEFT_OFF));
    // led right
    i2c_send(I2CBUF_LED_RIGHT_WHITE,   sizeof(I2CBUF_LED_RIGHT_WHITE));
    i2c_send(I2CBUF_LED_RIGHT_RED,     sizeof(I2CBUF_LED_RIGHT_RED));
    i2c_send(I2CBUF_LED_RIGHT_GREEN,   sizeof(I2CBUF_LED_RIGHT_GREEN));
    i2c_send(I2CBUF_LED_RIGHT_BLUE,    sizeof(I2CBUF_LED_RIGHT_BLUE));
    i2c_send(I2CBUF_LED_RIGHT_OFF,     sizeof(I2CBUF_LED_RIGHT_OFF));
*/
        
    // confiureg HF clock
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    // configure radio
    NRF_RADIO->MODE          = (  RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER       = (  RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0         = (                              8 << RADIO_PCNF0_LFLEN_Pos)          |
                               (                              1 << RADIO_PCNF0_S0LEN_Pos)          |
                               (                              0 << RADIO_PCNF0_S1LEN_Pos)          |
                               (                              2 << RADIO_PCNF0_CILEN_Pos)          |
                               (     RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos)           |
                               (                              3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1         = (                    sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos)         |
                               (                              0 << RADIO_PCNF1_STATLEN_Pos)        |
                               (                              3 << RADIO_PCNF1_BALEN_Pos)          |
                               (      RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos)         |
                               (   RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0         = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS     = 0UL;
    NRF_RADIO->RXADDRESSES   = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS          = 0;
    NRF_RADIO->CRCCNF        = (         RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos)           |
                               (     RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT       = 0xFFFFUL;
    NRF_RADIO->CRCPOLY       = 0x00065b; // CRC poly: x^16 + x^12^x^5 + 1
    NRF_RADIO->FREQUENCY     = 20;
    NRF_RADIO->PACKETPTR     = (uint32_t)pdu;

    // receive
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                        (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    NRF_RADIO->TASKS_RXEN    = 1;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NVIC_EnableIRQ(RADIO_IRQn);
    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);

    while(1) {
        __WFE();
    }


}


void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        if (NRF_RADIO->CRCSTATUS != RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
            puts("Invalid CRC");
        } else {
            printf("Received packet (%dB): %d\n", pdu[1], pdu[2]);
            switch (pdu[2]) {
    
                case 1 :  //Demarrer
                    i2c_send(I2CBUF_MOTOR_RIGHT_FWD,   sizeof(I2CBUF_MOTOR_RIGHT_FWD));
                    i2c_send(I2CBUF_MOTOR_LEFT_FWD,    sizeof(I2CBUF_MOTOR_LEFT_FWD));
                    break;

                case 2 : // Reculer
                    i2c_send(I2CBUF_MOTOR_RIGHT_BACK,   sizeof(I2CBUF_MOTOR_RIGHT_BACK));
                    i2c_send(I2CBUF_MOTOR_LEFT_BACK,    sizeof(I2CBUF_MOTOR_LEFT_BACK));
                    break;
  
                case 3 : //STOP
                    i2c_send(I2CBUF_MOTOR_LEFT_STOP,   sizeof(I2CBUF_MOTOR_LEFT_STOP));
                    i2c_send(I2CBUF_MOTOR_RIGHT_STOP,   sizeof(I2CBUF_MOTOR_RIGHT_STOP));
                    break;


                case 4 : //LED
                    i2c_send(I2CBUF_LED_LEFT_WHITE,    sizeof(I2CBUF_LED_LEFT_WHITE));
                    i2c_send(I2CBUF_LED_RIGHT_RED,     sizeof(I2CBUF_LED_RIGHT_RED));
                    break;

                case 5: //Eteindre LED
                    i2c_send(I2CBUF_LED_LEFT_OFF, sizeof(I2CBUF_LED_LEFT_OFF));
                    i2c_send(I2CBUF_LED_RIGHT_OFF, sizeof(I2CBUF_LED_RIGHT_OFF));
                    break;

                case 6 : //Gauche
                    i2c_send(I2CBUF_MOTOR_LEFT_STOP,   sizeof(I2CBUF_MOTOR_LEFT_STOP));
                    i2c_send(I2CBUF_MOTOR_RIGHT_FWD,   sizeof(I2CBUF_MOTOR_RIGHT_FWD));
                    i2c_send(I2CBUF_LED_RIGHT_RED, sizeof(I2CBUF_LED_RIGHT_RED));
                    i2c_send(I2CBUF_LED_LEFT_BLUE, sizeof(I2CBUF_LED_LEFT_BLUE));
                    break;

                case 7 : //Droite
                    i2c_send(I2CBUF_MOTOR_RIGHT_STOP,   sizeof(I2CBUF_MOTOR_RIGHT_STOP));
                    i2c_send(I2CBUF_MOTOR_LEFT_FWD,   sizeof(I2CBUF_MOTOR_LEFT_FWD));
                    i2c_send(I2CBUF_LED_RIGHT_RED, sizeof(I2CBUF_LED_RIGHT_RED));
                    i2c_send(I2CBUF_LED_LEFT_BLUE, sizeof(I2CBUF_LED_LEFT_BLUE));
                    break;
                  
                //i2c_send(I2CBUF_LED_LEFT_WHITE,    sizeof(I2CBUF_LED_LEFT_WHITE));
                //i2c_send(I2CBUF_LED_LEFT_RED,      sizeof(I2CBUF_LED_LEFT_RED));
                //i2c_send(I2CBUF_LED_LEFT_GREEN,    sizeof(I2CBUF_LED_LEFT_GREEN));
                //i2c_send(I2CBUF_LED_LEFT_BLUE,     sizeof(I2CBUF_LED_LEFT_BLUE));
                //i2c_send(I2CBUF_LED_LEFT_OFF,      sizeof(I2CBUF_LED_LEFT_OFF));
                //i2c_send(I2CBUF_LED_RIGHT_WHITE,   sizeof(I2CBUF_LED_RIGHT_WHITE));
                //i2c_send(I2CBUF_LED_RIGHT_RED,     sizeof(I2CBUF_LED_RIGHT_RED));
                //i2c_send(I2CBUF_LED_RIGHT_GREEN,   sizeof(I2CBUF_LED_RIGHT_GREEN));
                //i2c_send(I2CBUF_LED_RIGHT_BLUE,    sizeof(I2CBUF_LED_RIGHT_BLUE));
                //i2c_send(I2CBUF_LED_RIGHT_OFF,     sizeof(I2CBUF_LED_RIGHT_OFF));

            }
        }
    }
}
