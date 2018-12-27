/** ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : CS.c
**     Project   : Project
**     Processor : MC9S12XS128MAA
**     Component : BitIO
**     Version   : Component 02.075, Driver 03.14, CPU db: 3.00.019
**     Compiler  : CodeWarrior HCS12X C Compiler
**     Date/Time : 2016/1/26, 14:30
**     Abstract  :
**         This component "BitIO" implements an one-bit input/output.
**         It uses one bit/pin of a port.
**         Note: This component is set to work in Output direction only.
**         Methods of this component are mostly implemented as a macros
**         (if supported by target language and compiler).
**     Settings  :
**         Used pin                    :
**             ----------------------------------------------------
**                Number (on package)  |    Name
**             ----------------------------------------------------
**                       8             |  PT3_IOC3
**             ----------------------------------------------------
**
**         Port name                   : T
**
**         Bit number (in port)        : 3
**         Bit mask of the port        : $0008
**
**         Initial direction           : Output (direction cannot be changed)
**         Initial output value        : 0
**         Initial pull option         : up
**
**         Port data register          : PTT       [$0240]
**         Port control register       : DDRT      [$0242]
**
**         Optimization for            : speed
**     Contents  :
**         GetVal - bool CS_GetVal(void);
**         PutVal - void CS_PutVal(bool Val);
**         ClrVal - void CS_ClrVal(void);
**         SetVal - void CS_SetVal(void);
**
**     Copyright : 1997 - 2010 Freescale Semiconductor, Inc. All Rights Reserved.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################*/

/* MODULE CS. */

#include "CS.h"
  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Cpu.h"

#pragma DATA_SEG CS_DATA               /* Select data segment "CS_DATA" */
#pragma CODE_SEG CS_CODE
#pragma CONST_SEG CS_CONST             /* Constant section for this module */
/*
** ===================================================================
**     Method      :  CS_GetVal (component BitIO)
**
**     Description :
**         This method returns an input value.
**           a) direction = Input  : reads the input value from the
**                                   pin and returns it
**           b) direction = Output : returns the last written value
**         Note: This component is set to work in Output direction only.
**     Parameters  : None
**     Returns     :
**         ---             - Input value. Possible values:
**                           FALSE - logical "0" (Low level)
**                           TRUE - logical "1" (High level)

** ===================================================================
*/
/*
bool CS_GetVal(void)

**  This method is implemented as a macro. See CS.h file.  **
*/

/*
** ===================================================================
**     Method      :  CS_PutVal (component BitIO)
**
**     Description :
**         This method writes the new output value.
**     Parameters  :
**         NAME       - DESCRIPTION
**         Val             - Output value. Possible values:
**                           FALSE - logical "0" (Low level)
**                           TRUE - logical "1" (High level)
**     Returns     : Nothing
** ===================================================================
*/
void CS_PutVal(bool Val)
{
  if (Val) {
    setReg8Bits(PTT, 0x08U);           /* PTT3=0x01U */
  } else { /* !Val */
    clrReg8Bits(PTT, 0x08U);           /* PTT3=0x00U */
  } /* !Val */
}

/*
** ===================================================================
**     Method      :  CS_ClrVal (component BitIO)
**
**     Description :
**         This method clears (sets to zero) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void CS_ClrVal(void)

**  This method is implemented as a macro. See CS.h file.  **
*/

/*
** ===================================================================
**     Method      :  CS_SetVal (component BitIO)
**
**     Description :
**         This method sets (sets to one) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void CS_SetVal(void)

**  This method is implemented as a macro. See CS.h file.  **
*/


/* END CS. */
/*
** ###################################################################
**
**     This file was created by Processor Expert 3.02 [04.44]
**     for the Freescale HCS12X series of microcontrollers.
**
** ###################################################################
*/
