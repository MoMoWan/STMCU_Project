/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2009  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************
----------------------------------------------------------------------
File        : SIMConf.c
Purpose     : Windows Simulator configuration
---------------------------END-OF-HEADER------------------------------
*/

#ifdef WIN32

#include <windows.h>
#include <stdio.h>

#include "LCD_SIM.h"
#include "GUI_SIM_Win32.h"
#include "Sim.h"
#include "MainTask.h"

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       SIM_X_Config
*
* Purpose:
*   Called during the initialization process in order to configure
*   the simulator.
*/
void SIM_X_Config() {
  int i;

  for (i = 0; i < 5; i++) {
    SIM_HARDKEY_SetCallback(i, APP_cbKey);
  }
  SIM_GUI_UseCustomBitmaps();
  SIM_GUI_SetLCDPos(242, 397);         // Define the position of the LCD in the bitmap
  SIM_GUI_SetTransColor(0xff0000);     // Define the transparent color
}

#else

void SIMConf_C(void); // Avoid empty object files
void SIMConf_C(void) {}

#endif
