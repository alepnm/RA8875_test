

void EX_AlphaChannel(void){

    GUI_SetColor(GUI_BLUE);
    GUI_FillCircle(100, 50, 49);
    GUI_SetColor(GUI_YELLOW);

    for (uint8_t i = 0; i < 100; i++)
    {
        U8 Alpha;
        Alpha = (i * 255 / 100);
        GUI_SetAlpha(Alpha);
        GUI_DrawHLine(i, 100 - i, 100 + i);
    }

    GUI_SetAlpha(0x80);
    GUI_SetColor(GUI_MAGENTA);
    GUI_SetFont(&GUI_Font24B_ASCII);
    GUI_SetTextMode(GUI_TM_TRANS);
    GUI_DispStringHCenterAt("Alphablending", 100, 10);
    GUI_SetAlpha(0);
    GUI_DrawBitmap(&bmSTLogo, 30, 30);

    GUI_SetColor(GUI_YELLOW);

    GUI_SetFont(&GUI_Font8x10_ASCII);
    GUI_SetTextMode(GUI_TM_NORMAL);

}
