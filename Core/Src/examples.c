
#include "main.h"
#include "examples.h"


char st[20];



void EXAMPLE_ClearScreen(GUI_COLOR color){

  GUI_SetBkColor(color);
  GUI_Clear();
}



void EXAMPLE_ShowTimestamp(TEXT_Handle hObj){

  TEXT_SetTextColor(hObj, GUI_BLUE);

  //sprintf(st,"%ld", timestamp);
  TEXT_SetText(hObj, st);
}
