
 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Muhamed Amr
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and PORT Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
STATIC const Port_ConfigType *Port_ConfigPtr = NULL_PTR  ;

/************************************************************************************
* Service Name: Port_SetupGpioPin
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr )
{

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
             PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /*
         * Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures
         */
        Port_Status=PORT_INITIALIZED;
        Port_ConfigPtr=ConfigPtr; /* initialize Global const Pointer points to the array of Port_PinConfigType   */

        volatile uint32* PortGpio_Ptr=NULL_PTR; /* point to the required Port Registers base address */
        Port_PinType Pin_Index; /* for iteration  */
        for(Pin_Index=0; Pin_Index<PORT_CONFIGURED_PINS; Pin_Index++)
        {
            switch(ConfigPtr->Pins[Pin_Index].Port_Num)
            {
                case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                         break;
                case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                         break;
                case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                         break;
                case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                         break;
                case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                         break;
                case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                         break;
                default: break; /* Report an Error */
        }

            if (((ConfigPtr->Pins[Pin_Index].Port_Num==3&&ConfigPtr->Pins[Pin_Index].Pin_Num==7)||
                 (ConfigPtr->Pins[Pin_Index].Port_Num==5&&ConfigPtr->Pins[Pin_Index].Pin_Num== 0))) /* PD7 or PF0 */
            {
                /* Unlock the GPIOCR register */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_LOCK_REG_OFFSET)=0x4C4F434B;
                /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_COMMIT_REG_OFFSET) ,ConfigPtr->Pins[Pin_Index].Pin_Num);
            }
            else if((ConfigPtr->Pins[Pin_Index].Port_Num==2)&&(ConfigPtr->Pins[Pin_Index].Pin_Num<=3)) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins */
                return;
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }

            if(ConfigPtr->Pins[Pin_Index].Pin_Direction==PORT_PIN_OUT)
            {
                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET) ,ConfigPtr->Pins[Pin_Index].Pin_Num);
                if(ConfigPtr->Pins[Pin_Index].Pin_Initial_Value==STD_HIGH)
                {
                    /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                    SET_BIT(*(volatile uint32*)((volatile uint8 *)PortGpio_Ptr+PORT_DATA_REG_OFFSET) ,ConfigPtr->Pins[Pin_Index].Pin_Num);
                }
                else
                {
                    /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                    CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DATA_REG_OFFSET) ,ConfigPtr->Pins[Pin_Index].Pin_Num);
                }
            }
            else if(ConfigPtr->Pins[Pin_Index].Pin_Direction==PORT_PIN_IN)
            {
                /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                if(ConfigPtr->Pins[Pin_Index].Pin_Resistor==PULL_UP)
                {
                    /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                    SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_PULL_UP_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                }
                else if(ConfigPtr->Pins[Pin_Index].Pin_Resistor==PULL_DOWN)
                {
                    /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                    SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                }
                else
                {
                    /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                    CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_PULL_UP_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                    /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                    CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_PULL_DOWN_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                }
            }
            else
            {
                /* Do Nothing */
            }

            /* Setup the pin mode as GPIO */
            if(ConfigPtr->Pins[Pin_Index].Pin_Mode==PORT_PIN_MODE_DIO)
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Clear the PMCx bits for this pin */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_CTL_REG_OFFSET)&=~(0x0000000F<<((ConfigPtr->Pins[Pin_Index].Pin_Num)*4));
                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
            }
            /* Setup the pin mode as ADC*/
            else if (ConfigPtr->Pins[Pin_Index].Pin_Num==PORT_PIN_MODE_ADC)
            {
                /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Clear the PMCx bits for this pin */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_CTL_REG_OFFSET)&=~(0x0000000F<<((ConfigPtr->Pins[Pin_Index].Pin_Num)*4));
                /* Clear the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
            }
            /* Setup the pin mode for other alternative modes*/
            else
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
                /* Control the PMCx bits for this pin */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_CTL_REG_OFFSET) |=((ConfigPtr->Pins[Pin_Index].Pin_Mode)<<((ConfigPtr->Pins[Pin_Index].Pin_Num)*4));
                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->Pins[Pin_Index].Pin_Num);
            }
        }
    }
}

/************************************************************************************
* Service Name      : Port_SetPinDirection
* Sync/Async        : Synchronous
* Reentrancy        : reentrant
* Parameters (in)   : Pin - hold the index of the desired PIN
*                     Direction - hold the new direction of the desired PIN
* Parameters (inout): None
* Parameters (out)  : None
* Return value      : None
* Description       : Function to Set the port pin direction:
*                     - Check PIN direction status
*                     - Set the new direction for the desired PIN
************************************************************************************/

#if (PORT_SET_PIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
    volatile uint32 * PortGpio_Ptr=NULL_PTR; /* point to the required Port Registers base address */
    boolean Error = FALSE; /* Variable to check error */

#if (PORT_DEV_ERROR_DETECT==STD_ON )
    /* Check if the Driver is initialized before using this function */
    if(Port_Status==PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
        Error=TRUE;
    }
    else
    {
        /* No Action Required */
    }

    /* Check if the used Pin is within the valid range */
    if(Pin>PORT_CONFIGURED_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
        Error=TRUE;
    }
    else
    {
        /* No Action Required */
    }

    /* Check if the used Pin direction status is changeable */
    if(Port_ConfigPtr->Pins[Pin].Pin_DirectionStatus==PORT_PIN_UNCHANGEABLE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
        Error=TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif
    /* In-case there are no errors */
    if(Error==FALSE)
    {

        switch(Port_ConfigPtr->Pins[Pin].Port_Num)
        {
            case  0: PortGpio_Ptr=(volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                     break;
            case  1: PortGpio_Ptr=(volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                     break;
            case  2: PortGpio_Ptr=(volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                     break;
            case  3: PortGpio_Ptr=(volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                     break;
            case  4: PortGpio_Ptr=(volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                     break;
            case  5: PortGpio_Ptr=(volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                     break;
            default: break; /* Report an Error */
        }
        if( (Port_ConfigPtr->Pins[Pin].Port_Num==2)&&(Port_ConfigPtr->Pins[Pin].Pin_Num<=3)) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
            return;
        }
        else
        {
            /* Clear direction of pin */
            CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
            /* Set the new direction of PIN */
            *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET)|=(Direction<<Port_ConfigPtr->Pins[Pin].Pin_Num);
        }
    }
    else
    {
        /* No Action Required */
    }
}
#endif

/************************************************************************************
* Service Name      : Port_RefreshPortDirection
* Sync/Async        : Synchronous
* Reentrancy        : reentrant
* Parameters (in)   : None
* Parameters (inout): None
* Parameters (out)  : None
* Return value      : None
* Description       : Function to Refreshes port direction:
*                     - Check PIN direction status
*                     - Reset direction of All unchangeable PINs
************************************************************************************/

void Port_RefreshPortDirection(void)
{
    volatile uint32 * PortGpio_Ptr=NULL_PTR; /* point to the required Port Registers base address */
    Port_PinType Pin_Index; /* for iteration  */
    boolean Error = FALSE; /* Variable to check error */

#if (PORT_DEV_ERROR_DETECT==STD_ON )
       /* Check if the Driver is initialized before using this function */
       if(Port_Status == PORT_NOT_INITIALIZED)
       {
           Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                           PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
           Error=TRUE;
       }
       else
       {
           /* No Action Required */
       }
#endif

    /* In-case there are no errors */
    if(Error==FALSE)
    {
        for(Pin_Index=0; Pin_Index<PORT_CONFIGURED_PINS; Pin_Index++)
        {
            /* Refresh the direction of all configured ports to the configured direction
             * Except those port pins from refreshing that are configured as ‘pin direction changeable during runtime */
            if(Port_ConfigPtr->Pins[Pin_Index].Pin_DirectionStatus==PORT_PIN_UNCHANGEABLE)
            {
                switch(Port_ConfigPtr->Pins[Pin_Index].Port_Num)
                {
                case  0: PortGpio_Ptr=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                         break;
                case  1: PortGpio_Ptr=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                         break;
                case  2: PortGpio_Ptr=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                         break;
                case  3: PortGpio_Ptr=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                         break;
                case  4: PortGpio_Ptr=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                         break;
                case  5: PortGpio_Ptr=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                         break;
                default: break; /* Report an Error */
                }

                if( (Port_ConfigPtr->Pins[Pin_Index].Port_Num==2)&&(Port_ConfigPtr->Pins[Pin_Index].Pin_Num<=3)) /* PC0 to PC3 */
                {
                    /* Do Nothing ...  this is the JTAG pins */
                    return;
                }
                else
                {
                    /* Refresh direction of pin */
                    *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET)|=((Port_ConfigPtr->Pins[Pin_Index].Pin_Direction)<<(Port_ConfigPtr->Pins[Pin_Index].Pin_Num));
                }
            }
            else
            {
                /* Do Nothing */
            }
        }
    }
    else
    {
        /* No Action Required */
    }
}

/* Function to Return the version information of this module */
#if (PORT_VERSION_INFO_API==STD_ON)
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
{
#if (PORT_DEV_ERROR_DETECT==STD_ON)
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR==versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID=(uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID=(uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version=(uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version=(uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version=(uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif

/************************************************************************************
* Service Name      : Port_SetPinMode
* Sync/Async        : Synchronous
* Reentrancy        : reentrant
* Parameters (in)   : Pin - hold the index of the desired PIN
*                     Mode - hold the new mode of the desired PIN
* Parameters (inout): None
* Parameters (out)  : None
* Return value      : None
* Description       : Function to Set the port PIN mode:
*                     - Check PIN mode status
*                     - Set the new mode for the desired PIN
************************************************************************************/
#if (PORT_SET_PIN_MODE_API==STD_ON)
void Port_SetPinMode( Port_PinType Pin, Port_PinInitialModeType Mode )
{
    volatile uint32 * PortGpio_Ptr=NULL_PTR; /* point to the required Port Registers base address */
    boolean Error = FALSE; /* Variable to check error */

#if (PORT_DEV_ERROR_DETECT==STD_ON )
    /* Check if the Driver is initialized before using this function */
    if(Port_Status==PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
        Error=TRUE;
    }
    else
    {
        /* No Action required */
    }

    /* Check if the used Pin is within the valid range */
    if(Pin>PORT_CONFIGURED_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
        Error=TRUE;
    }
    else
    {
        /* No Action required */
    }

    /* Check if the used Mode is within the valid range */
    if(Mode > PORT_PIN_MODES_COUNT)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        Error=TRUE;
    }
    else
    {
        /* No Action required */
    }

    /* Check if the used Pin status is changeable */
    if(Port_ConfigPtr->Pins[Pin].Pin_ModeStatus==PORT_PIN_UNCHANGEABLE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
        Error=TRUE;
    }
    else
    {
        /* No Action required */
    }
#endif

    /* In-case there are no errors */
    if(Error==FALSE )
    {
        switch(Port_ConfigPtr->Pins[Pin].Port_Num)
        {
        case  0: PortGpio_Ptr=(volatile uint32*)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                 break;
        case  1: PortGpio_Ptr=(volatile uint32*)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                 break;
        case  2: PortGpio_Ptr=(volatile uint32*)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                 break;
        case  3: PortGpio_Ptr=(volatile uint32*)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                 break;
        case  4: PortGpio_Ptr=(volatile uint32*)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                 break;
        case  5: PortGpio_Ptr=(volatile uint32*)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                 break;
        default: break; /* Report an Error */
        }
        if( (Port_ConfigPtr->Pins[Pin].Port_Num==2)&&(Port_ConfigPtr->Pins[Pin].Pin_Num<=3)) /* PC0 to PC3 */
        {
            /* Do Nothing ...  this is the JTAG pins */
            return;
        }
        else
        {
            if(Mode==PORT_PIN_MODE_DIO)
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Clear the PMCx bits for this pin */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_CTL_REG_OFFSET)&=~(0x0000000F<<((Port_ConfigPtr->Pins[Pin].Pin_Num)*4));
                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
            }
            else if(Mode==PORT_PIN_MODE_ADC)
            {
                /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIR_REG_OFFSET) , Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Set the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Clear the PMCx bits for this pin */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_CTL_REG_OFFSET)&=~(0x0000000F<<((Port_ConfigPtr->Pins[Pin].Pin_Num)*4));
                /* Clear the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET) ,Port_ConfigPtr->Pins[Pin].Pin_Num);
            }
            /* Setup the pin mode for other alternative modes*/
            else
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Enable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET) , Port_ConfigPtr->Pins[Pin].Pin_Num);
                /* Clear the PMCx bits for this pin */
                *(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_CTL_REG_OFFSET)&=~(0x0000000F<<((Port_ConfigPtr->Pins[Pin].Pin_Num)*4));
                /* Set the PMCx bits for this pin */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr+PORT_CTL_REG_OFFSET)|=((Port_ConfigPtr->Pins[Pin].Pin_Mode)<<((Port_ConfigPtr->Pins[Pin].Pin_Num)*4));
                /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
                SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_ConfigPtr->Pins[Pin].Pin_Num);
            }
        }
    }
    else
    {
        /* No Action required */
    }
}
#endif
