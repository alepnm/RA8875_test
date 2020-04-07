/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.44                          *
*        Compiled Nov 10 2017, 08:53:57                              *
*        (c) 2017 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
#include "main.h"
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_FRAMEWIN_0    (GUI_ID_USER + 0x00)
#define ID_BUTTON_0    (GUI_ID_USER + 0x01)
#define ID_BUTTON_1    (GUI_ID_USER + 0x02)
#define ID_DROPDOWN_0    (GUI_ID_USER + 0x03)
#define ID_DROPDOWN_1    (GUI_ID_USER + 0x04)
#define ID_DROPDOWN_2    (GUI_ID_USER + 0x05)
#define ID_TEXT_0    (GUI_ID_USER + 0x06)
#define ID_TEXT_1    (GUI_ID_USER + 0x07)
#define ID_TEXT_2    (GUI_ID_USER + 0x08)
#define ID_RADIO_0    (GUI_ID_USER + 0x09)


// USER START (Optionally insert additional defines)
#define ID_BTN_OK ID_BUTTON_0
#define ID_BTN_CANCEL ID_BUTTON_1
#define ID_DRP_BAUDRATE ID_DROPDOWN_0
#define ID_DRP_STOPBITS ID_DROPDOWN_1
#define ID_DRP_PARITY ID_DROPDOWN_2
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
WM_HWIN hWin1;
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "ModbusPortSettings", ID_FRAMEWIN_0, 80, 50, 340, 160, 0, 0x64, 0 },
  { BUTTON_CreateIndirect, "OK", ID_BUTTON_0, 110, 100, 60, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Cancel", ID_BUTTON_1, 180, 100, 60, 20, 0, 0x0, 0 },
  { DROPDOWN_CreateIndirect, "Baudr", ID_DROPDOWN_0, 230, 12, 85, 18, 0, 0x0, 0 },
  { DROPDOWN_CreateIndirect, "Stopb", ID_DROPDOWN_1, 230, 36, 85, 18, 0, 0x0, 0 },
  { DROPDOWN_CreateIndirect, "Parity", ID_DROPDOWN_2, 230, 60, 85, 18, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Baudrate", ID_TEXT_0, 140, 12, 60, 17, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Stop Bits", ID_TEXT_1, 140, 36, 60, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Parity", ID_TEXT_2, 140, 60, 60, 20, 0, 0x0, 0 },
  { RADIO_CreateIndirect, "Radio", ID_RADIO_0, 16, 13, 103, 51, 0, 0x1e02, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/

// USER START (Optionally insert additional static code)
// USER END

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    //
    // Initialization of 'ModbusPortSettings'
    //
    hItem = pMsg->hWin;
    FRAMEWIN_SetText(hItem, "Modbus port settings");
    FRAMEWIN_SetFont(hItem, GUI_FONT_16B_ASCII);
    FRAMEWIN_SetTextColor(hItem, GUI_MAKE_COLOR(0x00FF0000));
    FRAMEWIN_SetTitleHeight(hItem, 30);
    //
    // Initialization of 'Baudr'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_DROPDOWN_0);
    DROPDOWN_AddString(hItem, "9600");
    DROPDOWN_AddString(hItem, "19200");
    DROPDOWN_AddString(hItem, "57600");
    DROPDOWN_AddString(hItem, "115200");
    DROPDOWN_SetListHeight(hItem, 56);
    //
    // Initialization of 'Stopb'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_DROPDOWN_1);
    DROPDOWN_AddString(hItem, "1 bit");
    DROPDOWN_AddString(hItem, "1.5 bit");
    DROPDOWN_AddString(hItem, "2 bits");
    DROPDOWN_SetListHeight(hItem, 42);
    //
    // Initialization of 'Parity'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_DROPDOWN_2);
    DROPDOWN_AddString(hItem, "None");
    DROPDOWN_AddString(hItem, "Even");
    DROPDOWN_AddString(hItem, "Odd");
    DROPDOWN_SetListHeight(hItem, 42);
    //
    // Initialization of 'Baudrate'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_13H_1);
    //
    // Initialization of 'Stop Bits'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    TEXT_SetFont(hItem, GUI_FONT_13H_1);
    //
    // Initialization of 'Parity'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
    TEXT_SetFont(hItem, GUI_FONT_13H_1);
    TEXT_SetTextAlign(hItem, GUI_TA_LEFT | GUI_TA_VCENTER);
    //
    // Initialization of 'Radio'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_0);
    RADIO_SetText(hItem, "Port 1 (Modbus)", 0);
    RADIO_SetText(hItem, "Port 2 (COM)", 1);
    // USER START (Optionally insert additional code for further widget initialization)

    hItem = pMsg->hWin;
    FRAMEWIN_SetMoveable(hItem, 0);
    FRAMEWIN_SetTitleHeight(hItem, 20);
    FRAMEWIN_SetMoveable(hItem, 1);


    hItem = WM_GetDialogItem(pMsg->hWin, ID_DRP_BAUDRATE);
    DROPDOWN_SetSel(hItem, 1);

    hItem = WM_GetDialogItem(pMsg->hWin, ID_DRP_STOPBITS);
    DROPDOWN_SetSel(hItem, 0);

    hItem = WM_GetDialogItem(pMsg->hWin, ID_DRP_PARITY);
    DROPDOWN_SetSel(hItem, 0);


    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'OK'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)

        // issaugojam nustatymus ir iseinam

        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_1: // Notifications sent by 'Cancel'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        hItem = pMsg->hWin;
        //WM_HideWindow(hItem);
        GUI_EndDialog(hItem, 1);

        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_DROPDOWN_0: // Notifications sent by 'Baudr'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_SEL_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_DROPDOWN_1: // Notifications sent by 'Stopb'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_SEL_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_DROPDOWN_2: // Notifications sent by 'Parity'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_SEL_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_RADIO_0: // Notifications sent by 'Radio'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_VALUE_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateModbusPortSettings
*/
WM_HWIN CreateModbusPortSettings(void);
WM_HWIN CreateModbusPortSettings(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
