
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
extern uint8_t	getData[];



/* ÖRNEK GELEN DATALAR

  -(R,r=138,g=112,b=23,B=55!!!!!!!!!!!)
	alacak degerler;
	
	RGB_LED_red=138
	RGB_LED_green=112
	RGB_LED_blue=23
	RGB_LED_brightness=55
	
	-(D,1!!!!!!!!!!!!!!!!!!!!!!!)
	alacak degerler;
	
	DOOR_status=1
	
*/

void process_command() {
    
    if (strlen( getData[0]) > 0) {
        // Gelen verinin ilk karakterine göre islem yap
        switch (getData[0]) {
            case 'R':
                // RGB LED kontrolü
             
                rgb_led_control();
                break;
            case 'D':
                // Kapi kontrolü
             
                kapi_control();
                break;
            case 'P':
                // Park kontrolü
              
                park_servo_control();
                break;
            case 'B':
                // Buzzer kontrolü
              
                buzzer_control();
                break;
            case 'G':
                // Bahçe isigi kontrolü
               
                normal_led_control();
                break;
            default:
                // Bilinmeyen komut
                break;
        }
    }
}


void rgb_led_control(){


}

void normal_led_control(){


}

void park_servo_control(){


}

void kapi_control(){


}

void buzzer_control(){


}