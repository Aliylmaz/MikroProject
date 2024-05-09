
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
                    rgb_led_control();
                }
                break;
            case 'D':
                if (sscanf((char *)getData, "D,%hhu", &DOOR_status) == 1)
                {
                    kapi_control();
                }
                break;
            case 'P':
                if (sscanf((char *)getData, "P,%hhu", &PARK_status) == 1)
                {
                    park_servo_control();
                }
                break;
            case 'B':
                if (sscanf((char *)getData, "B,%hhu", &BUZZER_status) == 1)
                {
                    buzzer_control();
                }
                break;
            case 'G':
               if (sscanf((char *)getData, "G,%hhu", &GARDEN_LIGHT_status) == 1){
									if (GARDEN_LIGHT_status){
										normal_led_on();
									}else{
										normal_led_off();
									}
								}
                break;
            default:
                // Bilinmeyen komut
                break;
        }
    }
}

void rgb_led_control()
{
    // PWM duty cycle degerleri
    uint32_t red_duty = (RGB_LED_red * RGB_LED_brightness) / 255;
    uint32_t green_duty = (RGB_LED_green * RGB_LED_brightness) / 255;
    uint32_t blue_duty = (RGB_LED_blue * RGB_LED_brightness) / 255;

    // TIM3_CH1 (PA6) için kirmizi bileseni ayarla
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, red_duty);

    // TIM3_CH2 (PA7) için yesil bileseni ayarla
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, green_duty);

    // TIM3_CH3 (PB0) için mavi bileseni ayarla
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, blue_duty);
}


void normal_led_on()
{
    // LED'i aç
   
}

void normal_led_off()
{
    // LED'i kapat
    
}


void park_servo_control()
{
    // Park servo kontrol fonksiyonu
    // PARK_status 0 veya 1 olabilir, buna göre servoyu konumlandir

    uint32_t pulse_width;

    if (PARK_status == 1)
    {
        // Servo 180 derece pozisyonunda, araba parkta
        pulse_width = 2000;  // 2000us
    }
    else
    {
        // Servo 0 derece pozisyonunda, araba disarida
        pulse_width = 1000;  // 1000us
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_width);
}

void kapi_control()
{
    // Kapi kontrol fonksiyonu
    // DOOR_status 0 veya 1 olabilir, buna göre kapiyi konumlandir

    uint32_t pulse_width;

    if (DOOR_status == 1)
    {
        // Kapi açik pozisyonu (90 derece)
        pulse_width = 1500;  // 1500us
    }
    else
    {
        // Kapi kapali pozisyonu (0 derece)
        pulse_width = 1000;  // 1000us
    }


}

void buzzer_control()
{
    // Buzzer kontrol fonksiyonu
}
