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
extern uint8_t targetHeat;
extern uint8_t resetAlarm;


/* ?RNEK GELEN DATALAR

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
    char *startMarker = strchr((char *)getData, '<');
    char *endMarker = strchr((char *)getData, '>');
    
    // Baslangi? ve bitis isaretlerini kontrol et
    if (startMarker == NULL || endMarker == NULL)
    {
        // Ge?ersiz veri, ?ik
        return;
    }

    // Veri i?inde baslangi? ve bitis isaretleri bulundu
    startMarker++; // '<' isaretini atlayarak verinin baslangicina gel
    *endMarker = '\0'; // '>' isaretini temizle

    // Veriyi d?zelt
    int shiftAmount = 0;
    // Dizideki her karakteri kontrol et
    for (int i = 0; i < strlen(startMarker); i++)
    {
        if (startMarker[i] == '<')
        {
            // '<' isareti bulundu, kaydirma miktarini belirle
            shiftAmount = i;
            break;
        }
    }

    if (shiftAmount > 0 && shiftAmount < strlen(startMarker))
    {
        // Diziyi kaydir
        char temp[32];
        strcpy(temp, startMarker + shiftAmount); // Kaydirilan kismi temp dizisine kopyala
        strcat(temp, startMarker); // Kalan kismi temp dizisine ekle
        strcpy(startMarker, temp); // temp dizisini basa kopyala
    }

    // Verinin ilk karakterine g?re islem yap
    switch (startMarker[0])
    {
        case 'R':
            if (sscanf(startMarker, "R,r=%hhu,g=%hhu,b=%hhu,B=%hhu", &RGB_LED_red, &RGB_LED_green, &RGB_LED_blue, &RGB_LED_brightness) == 4)
            {
                // Basarili sekilde okundu
            }
            break;
        case 'D':
            if (sscanf(startMarker, "D,%hhu", &DOOR_status) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
        case 'P':
            if (sscanf(startMarker, "P,%hhu", &PARK_status) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
        case 'B':
            if (sscanf(startMarker, "B,%hhu", &BUZZER_status) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
        case 'G':
            if (sscanf(startMarker, "G,%hhu", &GARDEN_LIGHT_status) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
        case 'V':
            if (sscanf(startMarker, "V,%hhu", &sendDataSatatus) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
						
				 case 'T':
            if (sscanf(startMarker, "T,%hhu", &targetHeat) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
						
						case 'A':
            if (sscanf(startMarker, "A,%hhu", &resetAlarm) == 1)
            {
                // Basarili sekilde okundu
            }
            break;
        default:
            // Bilinmeyen komut
            break;
    }
}
