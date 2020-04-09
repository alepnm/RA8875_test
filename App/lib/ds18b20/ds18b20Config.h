#ifndef	_DS18B20CONFIG_H
#define	_DS18B20CONFIG_H


//	Init timer on cube    1us per tick				example 72 MHz cpu >>> Prescaler=(72-1)      counter period=Max
//###################################################################################
#define	DS18B20_CONFIG_USE_FREERTOS		    				1
#define DS18B20_CONFIG_MAX_SENSORS		    				5
#define	DS18B20_CONFIG_GPIO									GPIOC
#define	DS18B20_CONFIG_PIN									LL_GPIO_PIN_11


#define DS18B20_TRYES_TO_READ                               5
#define	DS18B20_CONFIG_CONVERT_TIMEOUT_MS					1000

#if (DS18B20_CONFIG_USE_FREERTOS == 1)
    #define	DS18B20_CONFIG_UPDATE_INTERVAL_MS			    5000					//  (((	if==0  >> Ds18b20_ManualConvert()  )))    ((( if>0  >>>> Auto refresh )))
#endif

//###################################################################################

#endif


