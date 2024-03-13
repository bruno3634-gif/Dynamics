/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>      // Defines NULL
#include <stdbool.h>     // Defines true
#include <stdlib.h>      // Defines EXIT_FAILURE
#include "definitions.h" // SYS function prototypes

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
#define VOMID_RR 1.65
#define VOMIN_RR 0
#define DMIN_RR 25

#define VOMID_RL 1.65
#define VOMIN_RL 0
#define DMIN_RL 25

#define REVERSE_SIGNAL 0

#if REVERSE_SIGNAL
#define steadyGain_RL ((VOMID_RL - VOMIN_RL) / (-DMIN_RL))
#define steadyGain_RR ((VOMID_RR - VOMIN_RR) / (-DMIN_RR))
#else
#define steadyGain_RL ((VOMID_RL - VOMIN_RL) / (DMIN_RL))
#define steadyGain_RR ((VOMID_RR - VOMIN_RR) / (DMIN_RR))
#endif


#define ONCAR 0

#define CALIB_ID 0x40
// ############# RX CAN FRAME ###############################
static uint8_t rx_message[8] = {};

uint32_t messageID = 0;
uint32_t rx_messageID = 0;
uint32_t status = 0;
uint8_t messageLength = 0;
uint8_t rx_messageLength = 0;
uint8_t count = 0;
uint8_t user_input = 0;
CANFD_MSG_RX_ATTRIBUTE msgAttr = CANFD_MSG_RX_DATA_FRAME;

unsigned long vTimer1 = 0;
unsigned long vTimer2 = 0;
static uint8_t message[8];
uint16_t captValue1 = 0;
uint16_t captValue2 = 0;

uint8_t wheelSpeed_RL = 0;
uint8_t wheelSpeed_RR = 0;

uint16_t susPos_RL = 0;
uint16_t susPos_RR = 0;
uint8_t brakeState = 0;
uint8_t dynamicsState = 1;

float freq_RR = 0;
float freq_RL = 0;

#define Dentes_Coroa 4
#define Raio_Roda 100

static uint16_t ADC[64]; // ADC�raw�data

float graus_Segundo_RR = 0;
float graus_Segundo_RL = 0;

float velocidade_Roda_RR = 0;
float velocidade_Roda_RL = 0;

float flatPos_RR_Voltage = 0;
float flatPos_RL_Voltage = 0;
float pos_RL_Voltage = 0;
float pos_RR_Voltage = 0;
float travelDistance_RR = 0;
float travelDistance_RL = 0;

void captVal1();
void captVal2();

void Send_CAN(uint32_t id, uint8_t *message, uint8_t size);
void Read_CAN();

void Read_ADC(ADCHS_CHANNEL_NUM channel);

unsigned long millis();
uint8_t resetState = 0;
uint8_t resetSuspension();

uint8_t resetedFlag = 0;

int main(void) {
    /* Initialize all modules */

    SYS_Initialize(NULL);
    UART1_Initialize();

#if ONCAR
    while (!calibrated) {
        Read_CAN();
        if (rx_messageID == CALIB_ID) {
            flatPos_RR_Voltage = rx_message[0] | (rx_message[1] << 8);
            flatPos_RL_Voltage = rx_message[2] | (rx_message[3] << 8);
        }
    }
#else
    flatPos_RR_Voltage = 1.65;
    flatPos_RL_Voltage = 1.65;
#endif

    TMR3_Start();

    ICAP1_CallbackRegister(&captVal1, 1);
    ICAP1_Enable();
    TMR2_Start();
    ICAP2_CallbackRegister(&captVal2, 1);
    ICAP2_Enable();

    // setup

    while (true) {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();

        /*Get Susp Sensors values*/
        Read_ADC(ADCHS_CH0);
        Read_ADC(ADCHS_CH1);
        pos_RL_Voltage = 3.3 * (float) ADC[0] / (float) 4095;
        pos_RR_Voltage = 3.3 * (float) ADC[1] / (float) 4095;
        /************************/

        if (rx_messageID == 0x30)
            resetedFlag = resetSuspension();

        travelDistance_RR = (pos_RR_Voltage - flatPos_RR_Voltage) / steadyGain_RR;
        travelDistance_RL = (pos_RL_Voltage - flatPos_RL_Voltage) / steadyGain_RL;

        freq_RR = 1000 / (float) (captValue1 * 279 / 65389);
        freq_RL = 1000 / (float) (captValue2 * 279 / 65389);

        graus_Segundo_RR = 360 * freq_RR * Dentes_Coroa;
        graus_Segundo_RL = 360 * freq_RL * Dentes_Coroa;

        velocidade_Roda_RR = graus_Segundo_RR * 3.14 * Raio_Roda / 50;
        velocidade_Roda_RL = graus_Segundo_RL * 3.14 * Raio_Roda / 50;

        if (millis() - vTimer1 >= 5) {

            uint16_t travel_RL_CAN = (travelDistance_RL + 25)*100;
            uint16_t travel_RR_CAN = (travelDistance_RR + 25)*100;
            uint16_t velocidade_Roda_RL_CAN = velocidade_Roda_RL;

            message[0] = velocidade_Roda_RL_CAN;
            message[1] = 120;
            message[2] = travel_RL_CAN;
            message[3] = travel_RL_CAN >> 8;
            message[4] = travel_RR_CAN;
            message[5] = travel_RR_CAN >> 8;
            message[6] = resetedFlag;
            message[7] = dynamicsState;

            Send_CAN(0x80, message, 8);
            vTimer1 = millis();
        }

        if (millis() - vTimer2 >= 500) {

            GPIO_RB1_Toggle();
            printf("ICAP2: %d     ICAP1: %d   RL:%.2f     RR:%.2f\r\n", captValue2, captValue1, (travelDistance_RL + 25)*100, travelDistance_RR);
            vTimer2 = millis();
        }
    }

    /* Execution should not come here during normal operation */

    return (EXIT_FAILURE);
}

uint8_t resetSuspension() {
    flatPos_RL_Voltage = 3.3 * (float) ADC[0] / (float) 4095;
    flatPos_RR_Voltage = 3.3 * (float) ADC[1] / (float) 4095;
    return 1;
}

void captVal1() {

    captValue1 = IC1BUF;
    TMR3 = 0x0;
}

void captVal2() {

    captValue2 = IC2BUF;
    TMR2 = 0x0;
}

unsigned long millis(void) {
    return (unsigned int) (CORETIMER_CounterGet() / (CORE_TIMER_FREQUENCY / 1000));
}

void Read_CAN() {
    status = CAN1_ErrorGet();

    if (status == CANFD_ERROR_NONE) {
        memset(rx_message, 0x00, sizeof (rx_message));
        if (CAN1_MessageReceive(&rx_messageID, &rx_messageLength, rx_message, 0, 2, &msgAttr))
            ;
    }
}

void Send_CAN(uint32_t id, uint8_t *message, uint8_t size) {
    if (CAN1_TxFIFOQueueIsFull(0))
        ;
    else if (CAN1_MessageTransmit(id, size, message, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME))
        ;
}

void Read_ADC(ADCHS_CHANNEL_NUM channel) {
    ADCHS_ChannelConversionStart(channel);

    if (ADCHS_ChannelResultIsReady(channel)) {
        ADC[channel] = ADCHS_ChannelResultGet(channel);
    }
}

/*******************************************************************************
 End of File
 */
