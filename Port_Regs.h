 /******************************************************************************
 *
 * Module: PORT
 *
 * File Name: PORT_Regs.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - PORT Driver Registers
 *
 * Author: Muhamed Amr
 ******************************************************************************/

#ifndef PORT_REGS_H
#define PORT_REGS_H

#include "Std_Types.h"


/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/* Tiva-c Ports */
#define PORT_PORTA                  (0U)
#define PORT_PORTB                  (1U)
#define PORT_PORTC                  (2U)
#define PORT_PORTD                  (3U)
#define PORT_PORTE                  (4U)
#define PORT_PORTF                  (5U)

/* Tiva-c Pins */
#define PORT_PIN0                   (0U)
#define PORT_PIN1                   (1U)
#define PORT_PIN2                   (2U)
#define PORT_PIN3                   (3U)
#define PORT_PIN4                   (4U)
#define PORT_PIN5                   (5U)
#define PORT_PIN6                   (6U)
#define PORT_PIN7                   (7U)



#endif /* PORT_REGS_H */
