
#include "main.h"
#include "usart.h"

static Port_TypeDef    Ports[2];

Port_TypeDef* pPrimaryPort = &Ports[PRIMARY_PORT];
Port_TypeDef* pSecondaryPort = &Ports[SECONDARY_PORT];



#if defined(MODBUS_PORT)
    #include "mbport.h"

    Port_TypeDef* pMbPort = &Ports[MODBUS_PORT];
#endif

const uint16_t baudrates[7] = { 2400u, 4800u, 9600u, 14400u, 19200u, 38400u, 57600u };


/*  */
void USART_Config(Port_TypeDef* port, uint32_t ulBaudRate, uint32_t ulDataBits,  uint8_t uiParity, uint8_t uiStopbits) {

    uint32_t parity = LL_USART_PARITY_NONE, datawidth = LL_USART_DATAWIDTH_8B, stopbits = LL_USART_STOPBITS_1;

    LL_USART_Disable(port->handle);

    LL_USART_SetTransferDirection(port->handle, LL_USART_DIRECTION_TX_RX);

    switch(uiParity)
    {
    case PARITY_ODD:
        parity = LL_USART_PARITY_ODD;
    case PARITY_EVEN:
        parity = LL_USART_PARITY_EVEN;
        datawidth = LL_USART_DATAWIDTH_9B;
        break;
    default:
        parity = LL_USART_PARITY_NONE;
        datawidth = LL_USART_DATAWIDTH_8B;
    }

    switch(uiStopbits)
    {
    case STOPBITS_1_5:
        stopbits = LL_USART_STOPBITS_1_5;
        break;
    case STOPBITS_2:
        stopbits = LL_USART_STOPBITS_2;
        break;
    default:
        stopbits = LL_USART_STOPBITS_1;
        break;
    }

    LL_USART_ConfigCharacter(port->handle, datawidth, parity, stopbits);
    LL_USART_SetHWFlowCtrl(port->handle, LL_USART_HWCONTROL_NONE);

    uint32_t periphclk = (port == pPrimaryPort) ?
            __LL_RCC_CALC_PCLK2_FREQ(SystemCoreClock, LL_RCC_GetAPB2Prescaler()) :
            __LL_RCC_CALC_PCLK1_FREQ(SystemCoreClock, LL_RCC_GetAPB1Prescaler());


#warning patikrinti LL_USART_OVERSAMPLING...
    LL_USART_SetBaudRate(port->handle, periphclk, LL_USART_OVERSAMPLING_16, ulBaudRate);

    LL_USART_Enable(port->handle);
}


/*  */
void USART_Send( Port_TypeDef* port, void* data, uint16_t len ) {

    while(len--) {
        while(!LL_USART_IsActiveFlag_TC(port->handle));
        LL_USART_TransmitData8(port->handle, *((uint8_t*)data++));
    }
}


/*  */
void USART_SendByte(Port_TypeDef* port, char data) {
    LL_USART_TransmitData8(port->handle, data);
}

/*  */
void USART_SendString( Port_TypeDef* port, const char* str ) {

    uint8_t i = 0;

    while( *(str+i) ) {
        while(!LL_USART_IsActiveFlag_TC(port->handle));
        LL_USART_TransmitData8(port->handle, *(str+i));
        i++;
    }
}


/*  */
void USART_ClearRxBuffer(Port_TypeDef* port) {

    uint8_t i = 0;

    while(i < RX_BUFFER_SIZE) {
        port->Registers.RxBuffer[i++] = 0;
    }

    port->Registers.RxBufferIndex = 0;
    port->Registers.ReceivedData = 0;
}


/*  */
void USART_ClearTxBuffer(Port_TypeDef* port) {

    uint8_t i = 0;

    while(i < TX_BUFFER_SIZE) {
        port->Registers.TxBuffer[i++] = 0;
    }

    port->Registers.TxBufferIndex = 0;
}



/*  */
void USART_IRQ_Handler(Port_TypeDef* port) {

#if defined(MODBUS_PORT)

    if(port == pMbPort){

        if( LL_USART_IsActiveFlag_RXNE(pMbPort->handle) && LL_USART_IsEnabledIT_RXNE(pMbPort->handle) )
        {
            (void)pxMBFrameCBByteReceived();
        }

        if( LL_USART_IsActiveFlag_TC(pMbPort->handle) && LL_USART_IsEnabledIT_TC(pMbPort->handle) )
        {
            (void)pxMBFrameCBTransmitterEmpty();
        }
    }else{
#endif
        if( LL_USART_IsActiveFlag_RXNE(port->handle) && LL_USART_IsEnabledIT_RXNE(port->handle) ) {

            port->Registers.ReceivedData = LL_USART_ReceiveData8(port->handle);

            *(port->Registers.RxBuffer + port->Registers.RxBufferIndex) = port->Registers.ReceivedData;

            port->Registers.RxBufferIndex++;

            port->Registers.PortTimer = 20;
        }

        LL_USART_ClearFlag_ORE(port->handle);

#if defined(MODBUS_PORT)
    }
#endif
}


/*  */
void USART_TimerHandler(Port_TypeDef* port) {

    if(port->Registers.PortTimer == 0) return;

    port->Registers.PortTimer--;

    if(port->Registers.PortTimer == 0){

#if defined(MODBUS_PORT)
        if(port == pMbPort){
            (void)pxMBPortCBTimerExpired( );
        }else{
#endif
            port->Registers.RxBufferIndex = 0;
            port->Registers.TaskNotifyFlag = 1;

#if defined(MODBUS_PORT)
        }
#endif

    }
}


/*  */
void USART_Init(Port_TypeDef* port){

    port->handle = (port == pPrimaryPort) ? USART1 : USART2;

    USART_Config(port, baudrates[port->Config.Baudrate], port->Config.DataBits,  port->Config.Parity, port->Config.StopBits );

    if(port == pPrimaryPort)
    {

#if defined(MODBUS_PORT)
        LL_USART_DisableIT_RXNE(port->handle);
#else
        LL_USART_EnableIT_RXNE(port->handle);
#endif
        LL_USART_DisableIT_TC(port->handle);
    }
    else
    {
        LL_USART_EnableIT_RXNE(port->handle);
        LL_USART_DisableIT_TC(port->handle);
    }
}


/*  */
void USART_SetDefaults(Port_TypeDef* port){

    port->handle = (port == pPrimaryPort) ? USART1 : USART2;

    port->Config.Baudrate = BR19200;
    port->Config.MbAddr = 10;
    port->Config.DataBits = 8;
    port->Config.Parity = PARITY_NONE;
    port->Config.StopBits = STOPBITS_1;
    port->Registers.PortError = 0;
    port->Registers.PortTimer = 0;
    port->Registers.ReceivedData = 0;
    port->Registers.RxBufferIndex = 0;
    port->Registers.TxBufferIndex = 0;
    port->Registers.TaskNotifyFlag = 0;

    USART_ClearRxBuffer(port);
    USART_ClearTxBuffer(port);
}
