/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: ABDELRAHMAN MOHAMED
 ******************************************************************************/

/*******************************************************************************
*                                   INCLUDES                                  *
*******************************************************************************/

#include "PORT.h"
#include "PORT_REGS.h"
#include "tm4c123gh6pm_registers.h"  /* For SYSCTL_RCGCGPIO_REG */

#if ( (STD_ON)==PORT_DEV_ERROR_DETECT )
	#include "DET.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

/*******************************************************************************
 *                           PORT driver API's Definitions                     *
 *******************************************************************************/
 /* 
 * Static pointer to the Port_ConfigChannel configuration data. 
 * This pointer will hold the address of the configuration data 
 * used to initialize the port. It is initially set to NULL_PTR 
 * to indicate that no configuration has been assigned yet.
 */
 STATIC const  Port_ConfigChannel * Port_config = NULL_PTR;
 
 /* 
 * Static variable to keep track of the initialization status of the port driver. 
 * It is set to PORT_NOT_INITIALIZED to indicate that the port has not yet been initialized. 
 * This status will be updated once the Port_Init function is called and the port is initialized.
 */
 STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
 
 
 /*******************************************************************************
* Service Name: Port_Init
* Service ID: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initialize the Port Driver module
********************************************************************************/
void Port_Init( const Port_ConfigType* ConfigPtr ){


     /* Check if the input paramater is valid */
     #if ( (STD_ON)==PORT_DEV_ERROR_DETECT )
         if(NULL_PTR == ConfigPtr )
         {
             Det_ReportError(  PORT_MODULE_ID ,
                 PORT_INSTANCE_ID ,
                 PORT_INIT_SID ,
             PORT_E_PARAM_CONFIG );

             return;
         }

         else

         {

         }
     #endif

     /*
      * CRITICAL: Enable GPIO clocks for all ports BEFORE configuring pins!
      * Without this, all register writes to GPIO ports will be ignored.
      */

     /* Enable clock for all GPIO ports (A, B, C, D, E, F) */
     SYSCTL_RCGCGPIO_REG |= 0x3F;  /* Bits 0-5 for ports A-F */

     /* Wait until the GPIO peripheral clocks are ready (PRGPIO bits 0-5) */
     while ((SYSCTL_PRGPIO_REG & 0x3F) != 0x3F);

     /*
         * Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures
     */
     Port_Status=PORT_INITIALIZED ;
     Port_config=ConfigPtr->Channels;  /* address of the first Channels structure --> Channels[0] */
     /*
         * The Algorithm:
         *    1. Obtain the base address of the current port
         *    2. Check special pins for unlocking
         *    3. Set pin direction
         *    4. Set the resistor network
         *    5. Set the initial value
         *    6. Set the mode
     */
     volatile uint32 * Port_BaseAdd_Ptr = NULL_PTR; /* point to the required Port Registers base address */
     volatile Port_PinType pin_ID = 0;


     for (pin_ID=0; pin_ID < PORT_PINS_NUM ; pin_ID++)
     {
         /*1. Obtain the base address of the current port*/
         switch(Port_config[pin_ID].port_num)
         {
             case PORT_A : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS       ;break;  /* PORTA Base Address */
             case PORT_B : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS       ;break;  /* PORTB Base Address */
             case PORT_D : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS       ;break;  /* PORTD Base Address */
             case PORT_C : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS       ;break; /* PORTC Base Address */
             case PORT_E : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS       ;break;  /* PORTE Base Address */
             case PORT_F : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS       ;break;  /* PORTF Base Address */
             default     : Port_BaseAdd_Ptr= NULL_PTR ; continue;  /* Invalid port number: skip this pin safely */
         }
         /* 2. Check special pins for unlocking PF0,PD7 and PC[0:3] */
         if ( ((PORT_F  ==  Port_config[pin_ID].port_num) && (PIN_0 == Port_config[pin_ID].pin_num)) ||
             ((PORT_D  ==  Port_config[pin_ID].port_num) && (PIN_7 == Port_config[pin_ID].pin_num))
         )

         {
             /* Unlocking the lock register */
             *(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
             /* Unlocking the commit register */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_COMMIT_REG_OFFSET), Port_config[pin_ID].pin_num);

         }

         else if (   ((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_0 == Port_config[pin_ID].pin_num)) ||
             ((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_1 == Port_config[pin_ID].pin_num)) ||
             ((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_2 == Port_config[pin_ID].pin_num)) ||
         ((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_3 == Port_config[pin_ID].pin_num)))
         {
             continue ;
         }
         else
         {
             /* Normal Pins Do Nothing */
         }
		 
         /* 3. Set pin direction  , 4.set it's initial value  5.and set the resistor*/

         /*************** For Output Pin ***************/
         if (PORT_PIN_OUT == Port_config[pin_ID].pin_direction)
         {
             /* Set DIR Register bit to configure it as Output pin for the selected bit */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIR_REG_OFFSET) , Port_config[pin_ID].pin_num);
             if(Port_config[pin_ID].pin_initial_value == STD_HIGH)
             {
                 /* Set DATA Register bit */
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DATA_REG_OFFSET) , Port_config[pin_ID].pin_num);
             }
             else
             {
                 /* Clear DATA Register bit */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DATA_REG_OFFSET) , Port_config[pin_ID].pin_num);
             }
         }
         /*************** For Input Pin ***************/
         else if(PORT_PIN_IN == Port_config[pin_ID].pin_direction )
         {
             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIR_REG_OFFSET) , Port_config[pin_ID].pin_num);

             switch (Port_config[pin_ID].pin_internal_resistor_type )
             {
                 /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                 case PORT_PIN_INTERNAL_RESISTOR_UP:    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_config[pin_ID].pin_num); break;

                 /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                 case PORT_PIN_INTERNAL_RESISTOR_DOWN :    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_config[pin_ID].pin_num); break;

                 /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                 /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                 case PORT_PIN_INTERNAL_RESISTOR_OFF :  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_config[pin_ID].pin_num);
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_config[pin_ID].pin_num); break;
             }
         }
         else
         {
             /* Do Nothing */
         }

         /*6. set the pin mode */
         if (Port_config[pin_ID].pin_mode > PORT_DIGITAL_IO &&Port_config[pin_ID].pin_mode < PORT_ANALOG  )
         {
             /* Set the alterantive function bit */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_ALT_FUNC_REG_OFFSET), Port_config[pin_ID].pin_num);
             /* Set the peripheral function in the control register */
             *(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET) =
             ((*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET)) & ~(PIN_MODE_MASK << Port_config[pin_ID].pin_num * SHIFT_4))
             | (Port_config[pin_ID].pin_mode << Port_config[pin_ID].pin_num * SHIFT_4);

             /* Enable the digital functionality */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_config[pin_ID].pin_num);



         }
         else if (Port_config[pin_ID].pin_mode == PORT_ANALOG ) /* if it analog mode */
         {
             /* Clear the alternative function bit (AFSEL=0 for analog).
              * P4-2: this used to be MISSING here while Port_SetPinMode's analog
              * path did clear it - the two analog code paths now agree. It was
              * benign in practice because every AIN-capable pin (PB4/PB5,
              * PD0-PD3, PE0-PE5) resets with AFSEL=0 and only PE3 is configured
              * analog; it would have bitten a pin re-muxed from an alternate
              * function to analog. */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_ALT_FUNC_REG_OFFSET), Port_config[pin_ID].pin_num);

             /* disable Digital I/O  */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr +   PORT_DIGITAL_ENABLE_REG_OFFSET ), Port_config[pin_ID].pin_num) ;

             /* Clear the peripheral function in the control register */
             *(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET) =
             ((*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET)) & ~(PIN_MODE_MASK << Port_config[pin_ID].pin_num * SHIFT_4));

             /* enable Analog  */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ), Port_config[pin_ID].pin_num) ;





         }
         else if (Port_config[pin_ID].pin_mode == PORT_DIGITAL_IO)
         {     /* if it digital mode */


             /* disable the alterantive function bit */
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_ALT_FUNC_REG_OFFSET), Port_config[pin_ID].pin_num);

             /* Clear the peripheral function in the control register */
             *(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET) =
             ((*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET)) & ~(PIN_MODE_MASK << Port_config[pin_ID].pin_num * SHIFT_4));


             /* Enable the digital functionality */
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_config[pin_ID].pin_num);





         }
         else
         {
             /* P4-3: the mode in the config table matches NOTHING valid.
              *
              * Valid modes are exactly: PORT_DIGITAL_IO (0), an alternate
              * function 1..15 (used directly as the PCTL nibble), and
              * PORT_ANALOG (16). pin_mode is uint8, so anything 17..255 lands
              * here.
              *
              * This used to fall through to the digital branch above and be
              * configured as a plain GPIO, SILENTLY - so one mistyped mode in
              * PORT_PBCFG.c would leave a peripheral's pin un-muxed with no
              * error anywhere, in the driver whose failures are hardest to
              * diagnose. Port_SetPinMode already validated the same input and
              * reported PORT_E_PARAM_INVALID_MODE; only Port_Init did not.
              *
              * Report, then HALT - matching how main.c treats every other failed
              * init (SysTick/UART/Motor/Encoder, main.c:130-141). This is a
              * compile-time configuration typo caught at bring-up, not a field
              * runtime fault, and a wrong pin mux is worse to continue with than
              * to stop on: stopping makes it obvious on the bench immediately.
              * The halt is unconditional so behaviour does not change when DET
              * is compiled out. */
             #if ( (STD_ON)==PORT_DEV_ERROR_DETECT )
                 Det_ReportError(  PORT_MODULE_ID ,
                     PORT_INSTANCE_ID ,
                     PORT_INIT_SID ,
                 PORT_E_PARAM_INVALID_MODE );
             #endif

             while (1) { }   /* invalid pin mux - do not run with it */
         }

     }


}


#if (STD_ON == PORT_SET_PIN_DIRECTION_API)
/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number , Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function for Setting the port pin direction 
************************************************************************************/  
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{   
    boolean error = FALSE;
    #if (STD_ON == PORT_DEV_ERROR_DETECT)
    /* Check if the module is initialized */
    if( PORT_NOT_INITIALIZED == Port_Status)
    {
    Det_ReportError(  PORT_MODULE_ID ,
    PORT_INSTANCE_ID ,
    PORT_SET_PIN_DIRECTION_SID ,
    PORT_E_UNINIT   );
    
    
    error = TRUE ;
    }
    else 
    {
    
    /* Do Nothing */
    
    }
	
    /* Check if the passed pin number is valid */
    if (PORT_PINS_NUM  <= Pin)
    {
    Det_ReportError(  PORT_MODULE_ID ,
    PORT_INSTANCE_ID ,
    PORT_SET_PIN_DIRECTION_SID ,
    PORT_E_PARAM_PIN  );

    error = TRUE ;
    }
    else
    {
    /* Do Nothing */
    }

    /* Check if the pin direction is configured as changeable */
    if(PIN_DIRECTION_CHANGEABLE_OFF == Port_config[Pin].pin_direction_changeable)
    {
    Det_ReportError(  PORT_MODULE_ID ,
    PORT_INSTANCE_ID ,
    PORT_SET_PIN_DIRECTION_SID ,
    PORT_E_DIRECTION_UNCHANGEABLE  );

    error = TRUE ;
    }
    else
    {

    /* Do Nothing */
    }

#endif
    
    /*
    * The Algorithm:
    *    1. Obtain the base address of the current port
    *    2. Check special pins for unlocking
    *    3. Set pin direction
    
    */
    volatile uint32 * Port_BaseAdd_Ptr = NULL_PTR;
    
    if(error == FALSE ) {
    switch(Port_config[Pin].port_num)
    {
    
    case PORT_A : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS       ;break;  /* PORTA Base Address */
    case PORT_B : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS       ;break;  /* PORTB Base Address */
    case PORT_D : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS       ;break;  /* PORTD Base Address */
    case PORT_C : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS       ;break; /* PORTC Base Address */
    case PORT_E : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS       ;break;  /* PORTE Base Address */
    case PORT_F : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS       ;break;  /* PORTF Base Address */
    default     : Port_BaseAdd_Ptr= NULL_PTR ; error = TRUE ; return;  /* Invalid port number: abort before touching hardware */

    }
	
    /* 2. Check special pins for unlocking PF0,PD7 and PC[0:3] */
    if ( ((PORT_F  ==  Port_config[Pin].port_num) && (PIN_0 == Port_config[Pin].pin_num)) ||
    ((PORT_D  ==  Port_config[Pin].port_num) && (PIN_7 == Port_config[Pin].pin_num))
    )
    
    {
    /* Unlocking the lock register */
    *(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
    /* Unlocking the commit register */
    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_COMMIT_REG_OFFSET), Port_config[Pin].pin_num);
    
    }
    
    else if (((PORT_C  ==  Port_config[Pin].port_num) && (PIN_0 == Port_config[Pin].pin_num)) ||
    ((PORT_C  ==  Port_config[Pin].port_num) && (PIN_1 == Port_config[Pin].pin_num)) ||
    ((PORT_C  ==  Port_config[Pin].port_num) && (PIN_2 == Port_config[Pin].pin_num)) ||
    ((PORT_C  ==  Port_config[Pin].port_num) && (PIN_3 == Port_config[Pin].pin_num)))
    {
        /* Do Nothing ...  this is the JTAG pins */
                        return;
    }
    else
    {
    /* Normal Pins Do Nothing */
    }
    
    /* 3. Set pin direction */
    if (PORT_PIN_IN == Direction )
    {
     /* Set pin as input */
     CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIR_REG_OFFSET), Port_config[Pin].pin_num);
    
    }
    
    else
    {
     /* Set pin as output */
     SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIR_REG_OFFSET), Port_config[Pin].pin_num);
    }

}
}
#endif

/*******************************************************************************
* Service Name: Port_WritePin
* Service ID: 0x05
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin   - Port Pin ID number , Level - STD_HIGH or STD_LOW
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Writes a digital level to a port pin's data register
********************************************************************************/
void Port_WritePin( Port_PinType Pin, boolean Level )
{
    boolean error = FALSE;
    #if (STD_ON == PORT_DEV_ERROR_DETECT)
    /* Check if the module is initialized */
    if( PORT_NOT_INITIALIZED == Port_Status)
    {
    Det_ReportError(  PORT_MODULE_ID ,
    PORT_INSTANCE_ID ,
    PORT_WRITE_PIN_SID ,
    PORT_E_UNINIT   );

    error = TRUE ;
    }
    else
    {
    /* Do Nothing */
    }

    /* Check if the passed pin number is valid */
    if (PORT_PINS_NUM  <= Pin)
    {
    Det_ReportError(  PORT_MODULE_ID ,
    PORT_INSTANCE_ID ,
    PORT_WRITE_PIN_SID ,
    PORT_E_PARAM_PIN  );

    error = TRUE ;
    }
    else
    {
    /* Do Nothing */
    }

    #endif

    volatile uint32 * Port_BaseAdd_Ptr = NULL_PTR;

    if(error == FALSE )
    {
    switch(Port_config[Pin].port_num)
    {

    case PORT_A : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS       ;break;  /* PORTA Base Address */
    case PORT_B : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS       ;break;  /* PORTB Base Address */
    case PORT_D : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS       ;break;  /* PORTD Base Address */
    case PORT_C : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS       ;break;  /* PORTC Base Address */
    case PORT_E : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS       ;break;  /* PORTE Base Address */
    case PORT_F : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS       ;break;  /* PORTF Base Address */
    default     : Port_BaseAdd_Ptr= NULL_PTR ; return;  /* Invalid port number: abort before touching hardware */

    }

    if (STD_HIGH == Level)
    {
        /* Set DATA Register bit */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DATA_REG_OFFSET) , Port_config[Pin].pin_num);
    }
    else
    {
        /* Clear DATA Register bit */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DATA_REG_OFFSET) , Port_config[Pin].pin_num);
    }
    }
}

/*******************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
********************************************************************************/
void Port_RefreshPortDirection( void )
{
	boolean error = FALSE;
	#if (STD_ON == PORT_DEV_ERROR_DETECT)
		
		/* Check if the module is initialized */
		if(PORT_NOT_INITIALIZED == Port_Status)
		{
			Det_ReportError( PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION ,
			PORT_E_UNINIT );
			
			error = TRUE ;
			
		}
		else
		{
			/* Do Nothing */
		}
		
		
	#endif
	/*
		* The Algorithm:
		*    1. Obtain the base address of the current port
		*    2. Check special pins for unlocking
		*    3. refresh pin direction
		
	*/
	
	volatile uint32 * Port_BaseAdd_Ptr = NULL_PTR;
	volatile uint8 pin_ID = 0;
	
	if(error == FALSE){
		for (pin_ID=0; pin_ID< PORT_PINS_NUM ; pin_ID++)
		{
			/* 1. Obtain the base address of the current port   */
			switch(Port_config[pin_ID].port_num)
			{
				
				case PORT_A : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS       ;break;  /* PORTA Base Address */
				case PORT_B : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS       ;break;  /* PORTB Base Address */
				case PORT_D : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS       ;break;  /* PORTD Base Address */
				case PORT_C : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS       ;break; /* PORTC Base Address */
				case PORT_E : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS       ;break;  /* PORTE Base Address */
				case PORT_F : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS       ;break;  /* PORTF Base Address */ 
				default     : Port_BaseAdd_Ptr= NULL_PTR ; continue;  /* Invalid port number: skip this pin safely */
				
			}
			
			/* 2. Check special pins for unlocking PF0,PD7 and PC[0:3] */
			if ( ((PORT_F  ==  Port_config[pin_ID].port_num) && (PIN_0 == Port_config[pin_ID].pin_num)) ||
				((PORT_D  ==  Port_config[pin_ID].port_num) && (PIN_7 == Port_config[pin_ID].pin_num))
			)
			
			{
				/* Unlocking the lock register */
				*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
				/* Unlocking the commit register */
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_COMMIT_REG_OFFSET), Port_config[pin_ID].pin_num);
				
			}
			
			else if (   ((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_0 == Port_config[pin_ID].pin_num)) ||
				((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_1 == Port_config[pin_ID].pin_num)) ||
				((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_2 == Port_config[pin_ID].pin_num)) ||
			((PORT_C  ==  Port_config[pin_ID].port_num) && (PIN_3 == Port_config[pin_ID].pin_num)))
			{
				continue ;
			}
			else
			{
				/* Normal Pins Do Nothing */
			}
			
			/* Check if the current pin direction is changeable */
			if(PIN_DIRECTION_CHANGEABLE_OFF == Port_config[pin_ID].pin_direction_changeable)
			{
				continue;
			}
			else
			{
				if(PORT_PIN_IN == Port_config[pin_ID].pin_direction)
				{
					/* Refresh pin as input */
					CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIR_REG_OFFSET), Port_config[pin_ID].pin_num);
				}
				else
				{
					/* Refresh pin as output */
					SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_DIR_REG_OFFSET), Port_config[pin_ID].pin_num);
				}
				
			}
			
			
			
		}
	}
	
	
	
}
#if ((STD_ON) == PORT_VERSION_INFO_API )
/*******************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version info
* Return value: None
* Description: Returns the version information of this module
********************************************************************************/
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
{
	  boolean error = FALSE;
	#if (STD_ON == PORT_DEV_ERROR_DETECT)
  if(NULL_PTR == versioninfo)
   {
    Det_ReportError( PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID ,
                     PORT_E_POINTER );
					 error = TRUE ;
   }
    else
   {
    /* Do Nothing */
   }
   /* Check if the module is initialized */
	if( PORT_NOT_INITIALIZED == Port_Status)
	{
		  Det_ReportError(  PORT_MODULE_ID ,
                                 PORT_INSTANCE_ID ,
                                 PORT_GET_VERSION_INFO_SID ,
		                 PORT_E_UNINIT   );
		
		
		error = TRUE ;
	}
	else 
	{
		
		/* Do Nothing */
		
	}
#endif
    if(error == FALSE){
	versioninfo->vendorID= (uint16)PORT_VENDOR_ID ;  /* Copy the vendor Id */
	versioninfo->moduleID= (uint16)PORT_MODULE_ID ;  /* Copy the module Id */
    versioninfo->sw_major_version= (uint8)PORT_SW_MAJOR_VERSION ; /* Copy Software Major Version */
    versioninfo->sw_minor_version= (uint8)PORT_SW_MINOR_VERSION ; /* Copy Software Minor Version */
    versioninfo->sw_patch_version=(uint8) PORT_SW_PATCH_VERSION ;  /* Copy Software Patch Version */
	}
	
}
#endif


#if (STD_ON == PORT_SET_MODE_API )
/*******************************************************************************
* Service Name: Port_SetPinMode
* Service ID: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin  - Port Pin ID number
*                  Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
********************************************************************************/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
	boolean error = FALSE;
	#if (STD_ON == PORT_DEV_ERROR_DETECT)
		/* Check if the module is initialized */
		if( PORT_NOT_INITIALIZED == Port_Status)
		{
			Det_ReportError(  PORT_MODULE_ID ,
				PORT_INSTANCE_ID ,
				PORT_SET_PIN_MODE_SID  ,
			PORT_E_UNINIT   );
			
			
			error = TRUE ;
		}
		else 
		{
			
			/* Do Nothing */
			
		}
		if (PORT_PINS_NUM  <= Pin)
		{
			Det_ReportError(  PORT_MODULE_ID ,
				PORT_INSTANCE_ID ,
				PORT_SET_PIN_MODE_SID ,
			PORT_E_PARAM_PIN  );	
			
			error = TRUE ;
		}
		else 
		{
			/* Do Nothing */
		}
		/* Check if the mode is valid*/
		if(MAX_MODE_NUMBER < Mode)
		{
			Det_ReportError( PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
			PORT_E_PARAM_INVALID_MODE );
			
			error = TRUE ;
		}
		else
		{
			/* Do Nothing */
		}
		
		/* Check if the pin mode is configured as changeable */
		if(PIN_MODE_CHANGEABLE_OFF == Port_config[Pin].pin_mode_changeable)
		{
			Det_ReportError(  PORT_MODULE_ID ,
				PORT_INSTANCE_ID ,
				PORT_SET_PIN_MODE_SID ,
			PORT_E_MODE_UNCHANGEABLE   );	
			
			error = TRUE ;
		}
		else 
		{
			
			/* Do Nothing */
		}
	#endif
	/*
		* The Algorithm:
		*    1. Obtain the base address of the current port
		*    2. Check special pins for unlocking
		*    3. Set the mode
	*/
	volatile uint32 * Port_BaseAdd_Ptr = NULL_PTR;
	if(FALSE == error )
	{
		switch(Port_config[Pin].port_num)
		{
			
			case PORT_A : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS       ;break;  /* PORTA Base Address */
			case PORT_B : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS       ;break;  /* PORTB Base Address */
			case PORT_D : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS       ;break;  /* PORTD Base Address */
			case PORT_C : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS       ;break; /* PORTC Base Address */
			case PORT_E : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS       ;break;  /* PORTE Base Address */
			case PORT_F : Port_BaseAdd_Ptr= (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS       ;break;  /* PORTF Base Address */ 
			default     : Port_BaseAdd_Ptr= NULL_PTR ; error = TRUE ; return;  /* Invalid port number: abort before touching hardware */
			
		}
		
		/* 2. Check special pins for unlocking PF0,PD7 and PC[0:3] */
		if ( ((PORT_F  ==  Port_config[Pin].port_num) && (PIN_0 == Port_config[Pin].pin_num)) ||
			((PORT_D  ==  Port_config[Pin].port_num) && (PIN_7 == Port_config[Pin].pin_num))
		)
		
		{
			/* Unlocking the lock register */
			*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
			/* Unlocking the commit register */
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_COMMIT_REG_OFFSET), Port_config[Pin].pin_num);
			
		}
		
		else if (((PORT_C  ==  Port_config[Pin].port_num) && (PIN_0 == Port_config[Pin].pin_num)) ||
			((PORT_C  ==  Port_config[Pin].port_num) && (PIN_1 == Port_config[Pin].pin_num)) ||
			((PORT_C  ==  Port_config[Pin].port_num) && (PIN_2 == Port_config[Pin].pin_num)) ||
		((PORT_C  ==  Port_config[Pin].port_num) && (PIN_3 == Port_config[Pin].pin_num)))
		{
		    /* Do Nothing ...  this is the JTAG pins */
		                    return;
		}
		else
		{
			/* Normal Pins Do Nothing */
		}
		
		
		/*  3.Set the mode */
		if(((PORT_ANALOG  == Mode) && (PORT_B == Port_config[Pin].port_num) &&  (PIN_4 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_B == Port_config[Pin].port_num) &&  (PIN_5 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_D == Port_config[Pin].port_num) &&  (PIN_0 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_D == Port_config[Pin].port_num) &&  (PIN_1 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_D == Port_config[Pin].port_num) &&  (PIN_2 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_D == Port_config[Pin].port_num) &&  (PIN_3 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_E == Port_config[Pin].port_num) &&  (PIN_0 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_E == Port_config[Pin].port_num) &&  (PIN_1 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_E == Port_config[Pin].port_num) &&  (PIN_2 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_E == Port_config[Pin].port_num) &&  (PIN_3 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_E == Port_config[Pin].port_num) &&  (PIN_4 == Port_config[Pin].pin_num)) ||
			((PORT_ANALOG  == Mode) && (PORT_E == Port_config[Pin].port_num) &&  (PIN_5 == Port_config[Pin].pin_num)) 
		) 
		
		{

			/* Clear the alternative function bit (AFSEL=0 for analog). Port_Init's
			 * analog branch does the same since FIX-07/P4-2 - before that it did
			 * NOT, and this comment's claim of a match was wrong. */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_ALT_FUNC_REG_OFFSET), Port_config[Pin].pin_num);

			/* Clear the peripheral function in the control register */
			*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET) =
			((*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET)) & ~(PIN_MODE_MASK << Port_config[Pin].pin_num * SHIFT_4));

			/* disable Digital I/O  */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr +   PORT_DIGITAL_ENABLE_REG_OFFSET ), Port_config[Pin].pin_num) ;

		    /* enable Analog  */
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET ), Port_config[Pin].pin_num) ;
			
			
			
		}  
		
		else
		{
			/* Clear the Analog bit */
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_config[Pin].pin_num);
			
			/* Check whether the mode is DIO or another mode */
			if(PORT_DIGITAL_IO == Mode)
			{
				/* disable the alterantive function bit */
				CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_ALT_FUNC_REG_OFFSET), Port_config[Pin].pin_num);
				
				/* Clear the peripheral function in the control register */
				*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET) = 
				((*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET)) & ~(PIN_MODE_MASK << Port_config[Pin].pin_num * SHIFT_4));
				
				/* enable Digital I/O  */   
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr +   PORT_DIGITAL_ENABLE_REG_OFFSET ), Port_config[Pin].pin_num) ;
			}
			else
			{
				/* Set the alterantive function bit */
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_ALT_FUNC_REG_OFFSET), Port_config[Pin].pin_num);
				/* Set the peripheral function in the control register */
				*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET) = 
				((*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr  + PORT_CTL_REG_OFFSET)) & ~(PIN_MODE_MASK << Port_config[Pin].pin_num * SHIFT_4))
				| (Port_config[Pin].pin_mode << Port_config[Pin].pin_num * SHIFT_4);
				/* Enable the digital functionality */
				SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_BaseAdd_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_config[Pin].pin_num);
				
				
				
				
			}
			
		}
		
		
	}
	
	
}

#endif
