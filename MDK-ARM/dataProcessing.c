
#include "dataProcessing.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "string.h"
#include "stdio.h"



// Pointer Yerine extern kullan !

extern uint8_t RGB_LED_red; // KEY CHAR R
extern uint8_t RGB_LED_green; // KEY CHAR R
extern uint8_t RGB_LED_blue; // KEY CHAR R
extern uint8_t RGB_LED_brightness; // KEY CHAR R
extern uint8_t DOOR_status; // KEY CHAR D
extern uint8_t PARK_status; // KEY CHAR P
extern uint8_t BUZZER_status; // KEY CHAR B
extern uint8_t GARDEN_LIGHT_status; // KEY CHAR G
extern char	getData[];
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern int sendDataSatatus;


/* ÖRNEK GELEN DATALAR

  -(R,r=138,g=112,b=23,B=55!!!!!!!)
	alacak degerler;
	
	RGB_LED_red=138
	RGB_LED_green=112
	RGB_LED_blue=23
	RGB_LED_brightness=55
	
	-(D,1!!!!!!!!!!!!!!!!!!!!!!!)
	alacak degerler;
	
	DOOR_status=1
	
*/



void process_command()
{
    if (getData[0] != '\0')
    {
        // Verinin ilk karakterine göre islem yap
        switch (getData[0])
        {
            case 'R':
                if (sscanf((char *)getData, "R,r=%hhu,g=%hhu,b=%hhu,B=%hhu", &RGB_LED_red, &RGB_LED_green, &RGB_LED_blue, &RGB_LED_brightness) == 4)
                {
                  
                }
                break;
            case 'D':
                if (sscanf((char *)getData, "D,%hhu", &DOOR_status) == 1)
                {
                
                }
                break;
            case 'P':
                if (sscanf((char *)getData, "P,%hhu", &PARK_status) == 1)
                {
                   
                }
                break;
            case 'B':
                if (sscanf((char *)getData, "B,%hhu", &BUZZER_status) == 1)
                {
                   
                }
                break;
            case 'G':
               if (sscanf((char *)getData, "G,%hhu", &GARDEN_LIGHT_status) == 1){
									if (GARDEN_LIGHT_status){
									
									}else{
								
									}
								}
                break;
								
								 case 'V':
               if (sscanf((char *)getData, "V,%hhu", &sendDataSatatus) == 1){
									if (GARDEN_LIGHT_status){
									
									}else{
								
									}
								}
                break;
								
								
            default:
                // Bilinmeyen komut
                break;
        }
    }
}

