/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Muhamed Amr
 ******************************************************************************/

 #ifndef PORT_H
 #define PORT_H

 
/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID    (120U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/*
 * Macro for Pin Mode
 */
#define PORT_PIN_MODES_COUNT            (10U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and PORT Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* PORT Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PORT_Cfg.h does not match the expected version"
#endif

/* Software Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PORT_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"
#include "Port_Regs.h"



 /*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

 /* Type definition for Port_Type used by the PORT APIs to hold PORT number */
 typedef uint8  Port_Type;

 /* Type definition for Port_PinType used by the PORT APIs to hold PIN number */
 typedef uint8  Port_PinType;

 /* Type definition for Port_PinStateType used by the PORT APIs to hold PIN Output value */
 typedef enum
 {
     PORT_PIN_LEVEL_LOW,
     PORT_PIN_LEVEL_HIGH
 }Port_InitialValueType;

 /* Type definition for Port_PinType used by the PORT APIs to hold PIN status */
 typedef enum
 {
     PORT_PIN_UNCHANGEABLE,
     PORT_PIN_CHANGEABLE
 }Port_PinStatusType;

 /* Type definition for Port_PinDirectionType used by the PORT APIs to hold PIN direction*/
 typedef enum
 {
                 PORT_PIN_IN,
                 PORT_PIN_OUT
 }Port_PinDirectionType;


/* Type definition for Port_PinDirectionType used by the PORT APIs to hold  internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistorType;

/* Type definition for Port_PinInitialModeType used by the PORT APIs to hold  PIN Mode number */
typedef enum
{
    PORT_PIN_MODE_DIO,
    PORT_PIN_MODE_ALT1,
    PORT_PIN_MODE_ALT2,
    PORT_PIN_MODE_ALT3,
    PORT_PIN_MODE_ALT4,
    PORT_PIN_MODE_ALT5,
    PORT_PIN_MODE_ALT6,
    PORT_PIN_MODE_ALT7,
    PORT_PIN_MODE_ALT8,
    PORT_PIN_MODE_ALT9,
    PORT_PIN_MODE_ADC

}Port_PinInitialModeType;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *  3. the direction of pin       --> INPUT or OUTPUT
 *  4. the mode of pin            --> GPIO or UART or ...
 *  5. the internal resistor      --> Disable, Pull up or Pull down
 *  6. the initial value of pin   --> HIGH or LOW
 *  7. the mode status of pin      --> CHANGEABLE or UNCHANGEABLE
 *  8. the direction status of pin --> CHANGEABLE or UNCHANGEABLE
 */
typedef struct 
{
    Port_Type                 Port_Num; 
    Port_PinType              Pin_Num; 
    Port_PinDirectionType     Pin_Direction;
    Port_PinInitialModeType   Pin_Mode;
    Port_InternalResistorType Pin_Resistor;
    Port_InitialValueType     Pin_Initial_Value;
    Port_PinStatusType        Pin_ModeStatus;
    Port_PinStatusType        Pin_DirectionStatus;
}Port_PinConfigType;

typedef struct
{
    Port_PinConfigType Pins[PORT_CONFIGURED_PINS];
}Port_ConfigType;



/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

 /* Service ID for PORT Init */
 #define PORT_INIT_SID                              (uint8)0x00

 /* Service ID for PORT Set Pin Direction */
 #define PORT_SET_PIN_DIRECTION_SID                 (uint8)0x01

 /* Service ID for PORT Refresh Direction */
 #define PORT_REFRESH_PORT_DIRECTION_SID            (uint8)0x02

 /* Service ID for PORT Get Version Info */
 #define PORT_GET_VERSION_INFO_SID                  (uint8)0x03

 /* Service ID for PORT Set Pin Mode */
 #define PORT_SET_PIN_MODE_SID                      (uint8)0x04



/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

 /* DET code to report Invalid Port Pin ID requested */
 #define PORT_E_PARAM_PIN                         (uint8)0x0A

 /* DET code to report Port Pin not configured as changeable */
 #define PORT_E_DIRECTION_UNCHANGEABLE            (uint8)0x0B

 /* DET code to report API Port_Init service called with wrong parameter */
 #define PORT_E_PARAM_CONFIG                      (uint8)0x0C

 /* DET code to report API Port_SetPinMode service called when mode is unchangeable */
 #define PORT_E_PARAM_INVALID_MODE                (uint8)0x0D

 /* DET code to report API Port_SetPinMode service called when mode is unchangeable */
 #define PORT_E_MODE_UNCHANGEABLE                 (uint8)0x0E

 /* DET code to report API service called without module initialization */
 #define PORT_E_UNINIT                            (uint8)0x0F

 /* DET code to report APIs called with a Null Pointer */
 #define PORT_E_PARAM_POINTER                     (uint8)0x10


 /*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

 /* Function to Initializes the Port Driver module */
 void Port_Init( const Port_ConfigType* ConfigPtr );

 /* Function to Set the port pin direction*/
 void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );

 /* Function to Refreshes port direction */
 void Port_RefreshPortDirection(void);

 /* Function to Return the version information of this module */
 void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );

 /* Funtion to Set the port pin mode */
 void Port_SetPinMode( Port_PinType Pin, Port_PinInitialModeType Mode );

 /*******************************************************************************
  *                       External Variables                                    *
  *******************************************************************************/

 /* Extern PB structures to be used by Port and other modules */
 extern const Port_ConfigType PORT_Configuration;


#endif
