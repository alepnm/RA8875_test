#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED


#define PRIMARY_PORT    0
#define SECONDARY_PORT  1

#define MODBUS_PORT     PRIMARY_PORT



#define RX_BUFFER_SIZE     64
#define TX_BUFFER_SIZE     32


typedef enum { PARITY_NONE = 0, PARITY_ODD, PARITY_EVEN } parity_t;
typedef enum { BR9600 = 0, BR19200, BR57600, BR115200 }baudrate_t;
typedef enum { STOPBITS_1 = 0, STOPBITS_1_5, STOPBITS_2 } stopbits_t;


typedef struct _port{

    USART_TypeDef           *handle;

    struct{
        uint8_t             MbAddr;
        baudrate_t          Baudrate;
        parity_t            Parity;
        stopbits_t          StopBits;
        uint8_t             DataBits;
    }Config;

    struct{
        uint8_t             PortError;
        volatile uint8_t    PortTimer;                  // porto taimeris
        uint8_t             TaskNotifyFlag;
        uint8_t             ReceivedData;               // priimtas baitas
        uint8_t             RxBufferIndex;              // porto RX buferio indeksas
        char                RxBuffer[RX_BUFFER_SIZE];   // porto RX buferis
        uint8_t             TxBufferIndex;              // porto TX buferio indeksas
        char                TxBuffer[TX_BUFFER_SIZE];   // porto TX buferis
    }Registers;

}Port_TypeDef;

extern Port_TypeDef* pPrimaryPort;
extern Port_TypeDef* pSecondaryPort;

#ifdef MODBUS_PORT
    extern Port_TypeDef* pMbPort;
#endif

extern const uint32_t baudrates[7];


void    USART_Init(Port_TypeDef* port);
void    USART_Config(Port_TypeDef* port, uint32_t ulBaudRate, uint32_t ulDataBits,  uint8_t uiParity, uint8_t uiStopbits);
void    USART_Send( Port_TypeDef* port, void* buf, uint16_t len );

void    USART_SendByte( Port_TypeDef* port, char data );
void    USART_SendString( Port_TypeDef* port, const char* str );
void    USART_IRQ_Handler( Port_TypeDef* port );
void    USART_TimerHandler( Port_TypeDef* port );
void    USART_ClearRxBuffer( Port_TypeDef* port );
void    USART_ClearTxBuffer( Port_TypeDef* port );
void    USART_SetDefaults( Port_TypeDef* port );

#endif /* USART_H_INCLUDED */


