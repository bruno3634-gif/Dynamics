/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h UUUUUUUUU

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for ST_ANGLE pin ***/
#define ST_ANGLE_Get()               ((PORTG >> 6) & 0x1U)
#define ST_ANGLE_PIN                  GPIO_PIN_RG6

/*** Macros for BTN pin ***/
#define BTN_Set()               (LATBSET = (1U<<1))
#define BTN_Clear()             (LATBCLR = (1U<<1))
#define BTN_Toggle()            (LATBINV= (1U<<1))
#define BTN_OutputEnable()      (TRISBCLR = (1U<<1))
#define BTN_InputEnable()       (TRISBSET = (1U<<1))
#define BTN_Get()               ((PORTB >> 1) & 0x1U)
#define BTN_PIN                  GPIO_PIN_RB1
#define BTN_InterruptEnable()   (CNENBSET = (1U<<1))
#define BTN_InterruptDisable()  (CNENBCLR = (1U<<1))

/*** Macros for GPIO_RC11 pin ***/
#define GPIO_RC11_Set()               (LATCSET = (1U<<11))
#define GPIO_RC11_Clear()             (LATCCLR = (1U<<11))
#define GPIO_RC11_Toggle()            (LATCINV= (1U<<11))
#define GPIO_RC11_OutputEnable()      (TRISCCLR = (1U<<11))
#define GPIO_RC11_InputEnable()       (TRISCSET = (1U<<11))
#define GPIO_RC11_Get()               ((PORTC >> 11) & 0x1U)
#define GPIO_RC11_PIN                  GPIO_PIN_RC11

/*** Macros for Brake_signal pin ***/
#define Brake_signal_Set()               (LATBSET = (1U<<7))
#define Brake_signal_Clear()             (LATBCLR = (1U<<7))
#define Brake_signal_Toggle()            (LATBINV= (1U<<7))
#define Brake_signal_OutputEnable()      (TRISBCLR = (1U<<7))
#define Brake_signal_InputEnable()       (TRISBSET = (1U<<7))
#define Brake_signal_Get()               ((PORTB >> 7) & 0x1U)
#define Brake_signal_PIN                  GPIO_PIN_RB7

/*** Macros for Blue_leds pin ***/
#define Blue_leds_Set()               (LATCSET = (1U<<13))
#define Blue_leds_Clear()             (LATCCLR = (1U<<13))
#define Blue_leds_Toggle()            (LATCINV= (1U<<13))
#define Blue_leds_OutputEnable()      (TRISCCLR = (1U<<13))
#define Blue_leds_InputEnable()       (TRISCSET = (1U<<13))
#define Blue_leds_Get()               ((PORTC >> 13) & 0x1U)
#define Blue_leds_PIN                  GPIO_PIN_RC13

/*** Macros for Yellow_Leds pin ***/
#define Yellow_Leds_Set()               (LATBSET = (1U<<8))
#define Yellow_Leds_Clear()             (LATBCLR = (1U<<8))
#define Yellow_Leds_Toggle()            (LATBINV= (1U<<8))
#define Yellow_Leds_OutputEnable()      (TRISBCLR = (1U<<8))
#define Yellow_Leds_InputEnable()       (TRISBSET = (1U<<8))
#define Yellow_Leds_Get()               ((PORTB >> 8) & 0x1U)
#define Yellow_Leds_PIN                  GPIO_PIN_RB8

/*** Macros for MS1 pin ***/
#define MS1_Set()               (LATBSET = (1U<<9))
#define MS1_Clear()             (LATBCLR = (1U<<9))
#define MS1_Toggle()            (LATBINV= (1U<<9))
#define MS1_OutputEnable()      (TRISBCLR = (1U<<9))
#define MS1_InputEnable()       (TRISBSET = (1U<<9))
#define MS1_Get()               ((PORTB >> 9) & 0x1U)
#define MS1_PIN                  GPIO_PIN_RB9

/*** Macros for MS2 pin ***/
#define MS2_Set()               (LATCSET = (1U<<6))
#define MS2_Clear()             (LATCCLR = (1U<<6))
#define MS2_Toggle()            (LATCINV= (1U<<6))
#define MS2_OutputEnable()      (TRISCCLR = (1U<<6))
#define MS2_InputEnable()       (TRISCSET = (1U<<6))
#define MS2_Get()               ((PORTC >> 6) & 0x1U)
#define MS2_PIN                  GPIO_PIN_RC6

/*** Macros for MS3 pin ***/
#define MS3_Set()               (LATCSET = (1U<<7))
#define MS3_Clear()             (LATCCLR = (1U<<7))
#define MS3_Toggle()            (LATCINV= (1U<<7))
#define MS3_OutputEnable()      (TRISCCLR = (1U<<7))
#define MS3_InputEnable()       (TRISCSET = (1U<<7))
#define MS3_Get()               ((PORTC >> 7) & 0x1U)
#define MS3_PIN                  GPIO_PIN_RC7

/*** Macros for MS4 pin ***/
#define MS4_Set()               (LATCSET = (1U<<8))
#define MS4_Clear()             (LATCCLR = (1U<<8))
#define MS4_Toggle()            (LATCINV= (1U<<8))
#define MS4_OutputEnable()      (TRISCCLR = (1U<<8))
#define MS4_InputEnable()       (TRISCSET = (1U<<8))
#define MS4_Get()               ((PORTC >> 8) & 0x1U)
#define MS4_PIN                  GPIO_PIN_RC8

/*** Macros for MS5 pin ***/
#define MS5_Set()               (LATDSET = (1U<<5))
#define MS5_Clear()             (LATDCLR = (1U<<5))
#define MS5_Toggle()            (LATDINV= (1U<<5))
#define MS5_OutputEnable()      (TRISDCLR = (1U<<5))
#define MS5_InputEnable()       (TRISDSET = (1U<<5))
#define MS5_Get()               ((PORTD >> 5) & 0x1U)
#define MS5_PIN                  GPIO_PIN_RD5

/*** Macros for MS6 pin ***/
#define MS6_Set()               (LATDSET = (1U<<6))
#define MS6_Clear()             (LATDCLR = (1U<<6))
#define MS6_Toggle()            (LATDINV= (1U<<6))
#define MS6_OutputEnable()      (TRISDCLR = (1U<<6))
#define MS6_InputEnable()       (TRISDSET = (1U<<6))
#define MS6_Get()               ((PORTD >> 6) & 0x1U)
#define MS6_PIN                  GPIO_PIN_RD6


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/


#define    GPIO_PORT_A  (0)
#define    GPIO_PORT_B  (1)
#define    GPIO_PORT_C  (2)
#define    GPIO_PORT_D  (3)
#define    GPIO_PORT_E  (4)
#define    GPIO_PORT_F  (5)
#define    GPIO_PORT_G  (6)
typedef uint32_t GPIO_PORT;

typedef enum
{
    GPIO_INTERRUPT_ON_MISMATCH,
    GPIO_INTERRUPT_ON_RISING_EDGE,
    GPIO_INTERRUPT_ON_FALLING_EDGE,
    GPIO_INTERRUPT_ON_BOTH_EDGES,
}GPIO_INTERRUPT_STYLE;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/


#define     GPIO_PIN_RA0  (0U)
#define     GPIO_PIN_RA1  (1U)
#define     GPIO_PIN_RA4  (4U)
#define     GPIO_PIN_RA7  (7U)
#define     GPIO_PIN_RA8  (8U)
#define     GPIO_PIN_RA10  (10U)
#define     GPIO_PIN_RA11  (11U)
#define     GPIO_PIN_RA12  (12U)
#define     GPIO_PIN_RB0  (16U)
#define     GPIO_PIN_RB1  (17U)
#define     GPIO_PIN_RB2  (18U)
#define     GPIO_PIN_RB3  (19U)
#define     GPIO_PIN_RB4  (20U)
#define     GPIO_PIN_RB5  (21U)
#define     GPIO_PIN_RB6  (22U)
#define     GPIO_PIN_RB7  (23U)
#define     GPIO_PIN_RB8  (24U)
#define     GPIO_PIN_RB9  (25U)
#define     GPIO_PIN_RB10  (26U)
#define     GPIO_PIN_RB11  (27U)
#define     GPIO_PIN_RB12  (28U)
#define     GPIO_PIN_RB13  (29U)
#define     GPIO_PIN_RB14  (30U)
#define     GPIO_PIN_RB15  (31U)
#define     GPIO_PIN_RC0  (32U)
#define     GPIO_PIN_RC1  (33U)
#define     GPIO_PIN_RC2  (34U)
#define     GPIO_PIN_RC6  (38U)
#define     GPIO_PIN_RC7  (39U)
#define     GPIO_PIN_RC8  (40U)
#define     GPIO_PIN_RC9  (41U)
#define     GPIO_PIN_RC10  (42U)
#define     GPIO_PIN_RC11  (43U)
#define     GPIO_PIN_RC12  (44U)
#define     GPIO_PIN_RC13  (45U)
#define     GPIO_PIN_RC15  (47U)
#define     GPIO_PIN_RD5  (53U)
#define     GPIO_PIN_RD6  (54U)
#define     GPIO_PIN_RD8  (56U)
#define     GPIO_PIN_RE12  (76U)
#define     GPIO_PIN_RE13  (77U)
#define     GPIO_PIN_RE14  (78U)
#define     GPIO_PIN_RE15  (79U)
#define     GPIO_PIN_RF0  (80U)
#define     GPIO_PIN_RF1  (81U)
#define     GPIO_PIN_RG6  (102U)
#define     GPIO_PIN_RG7  (103U)
#define     GPIO_PIN_RG8  (104U)
#define     GPIO_PIN_RG9  (105U)

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
#define    GPIO_PIN_NONE   (-1)

typedef uint32_t GPIO_PIN;

typedef  void (*GPIO_PIN_CALLBACK) ( GPIO_PIN pin, uintptr_t context);

void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptDisable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: Local Data types and Prototypes
// *****************************************************************************
// *****************************************************************************

typedef struct {

    /* target pin */
    GPIO_PIN                 pin;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK        callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
	 uint32_t xvalue = (uint32_t)value;
    GPIO_PortWrite((pin>>4U), (uint32_t)(0x1U) << (pin & 0xFU), (xvalue) << (pin & 0xFU));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return ((((GPIO_PortRead((GPIO_PORT)(pin>>4U))) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (((GPIO_PortLatchRead((GPIO_PORT)(pin>>4U)) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

#define GPIO_PinInterruptEnable(pin)       GPIO_PinIntEnable(pin, GPIO_INTERRUPT_ON_MISMATCH)
#define GPIO_PinInterruptDisable(pin)      GPIO_PinIntDisable(pin)

void GPIO_PinIntEnable(GPIO_PIN pin, GPIO_INTERRUPT_STYLE style);
void GPIO_PinIntDisable(GPIO_PIN pin);

bool GPIO_PinInterruptCallbackRegister(
    GPIO_PIN pin,
    const   GPIO_PIN_CALLBACK callback,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
