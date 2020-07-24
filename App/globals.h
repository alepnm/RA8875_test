#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED


#include "GUI.h"


const struct _img_data{
  uint16_t      xsize; // xSize
  uint16_t      ysize; // ySize
  uint16_t*     pData;  // Pointer to picture data
}img_data;

extern const struct _img_data img_krym;
extern const struct _img_data img_enot;






#endif /* GLOBALS_H_INCLUDED */
