/*
 * buzzing_task.c
 *
 *  Created on: 16 Sep 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "buzzing_task.h"
#include "typedefs.h"
#define BUZZER_HIGH 1760
#define BUZZER_LOW	440
#define DEBUG_LOW_FREQ 		370
#define DEBUG_HIGH_FREQ		2434
#define BUZZ_TIME 100
#define GAP_TIME 50
extern TaskHandle_t buzzing_task_handle;
extern QueueHandle_t buzzing_task_msg;

//Never gonna give you up~
//https://musescore.com/chlorondria/never-gonna-give-you-up



#define TEMPO	120
const uint16_t melody[] = {
REST, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_F4, NOTE_G4, NOTE_E4,
NOTE_D4, NOTE_C4, REST,
REST, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_D4, NOTE_C4,
NOTE_C5, REST, NOTE_C5, NOTE_G4, REST,
NOTE_D4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_D4, NOTE_F4, NOTE_G4, REST,
REST, NOTE_E4, NOTE_D4, NOTE_C4, REST,
REST, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_D4, NOTE_C4,
NOTE_G4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, REST,
NOTE_F4, NOTE_G4, NOTE_A4, NOTE_F4,
NOTE_G4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_C4,
REST, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_D4,
REST, NOTE_G4, NOTE_A4, NOTE_G4, NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4,
NOTE_A4, NOTE_A4, NOTE_G4, NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4,
NOTE_G4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_D4, NOTE_D4,
NOTE_F4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_C4,
NOTE_G4, NOTE_F4, NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4,
NOTE_A4, NOTE_A4, NOTE_G4, NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4,
NOTE_C5, NOTE_E4, NOTE_F4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_F4, NOTE_D4,
NOTE_F4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_C4,
NOTE_G4, NOTE_F4, REST, };

const float note_length[] = { 8, 2, 2, 2, 2, 2, 3, 1, 10, 4, 2, 2, 2, 2, 2, 4,
		2, 2, 2, 2, 6, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 6, 4, 2, 2, 2, 2, 2,
		2, 4, 2, 2, 2, 2, 4, 4, 10, 2, 2, 2, 2, 2, 2, 2, 4, 4, 8, 2, 2, 2, 2, 2,
		2, 2, 6, 1, 1, 1, 1, 3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 1, 4,
		2, 3, 1, 2, 2, 2, 4, 6, 1, 1, 1, 1, 3, 3, 6, 1, 1, 1, 1, 4, 2, 3, 1, 2,
		1, 1, 1, 1, 4, 2, 3, 1, 2, 2, 4, 8, 2,

};

/*

 //Pink panther
 #define TEMPO	120
 const uint16_t melody[] =
 {
 REST, REST, REST, NOTE_DS4,
 NOTE_E4, REST, NOTE_FS4, NOTE_G4, REST, NOTE_DS4,
 NOTE_E4, NOTE_FS4,  NOTE_G4, NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_B4,
 NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_E4, NOTE_D4, NOTE_E4,
 REST, REST, NOTE_DS4,
 NOTE_E4, REST, NOTE_FS4, NOTE_G4, REST, NOTE_DS4,
 NOTE_E4, NOTE_FS4,  NOTE_G4, NOTE_C5, NOTE_B4, NOTE_G4, NOTE_B4, NOTE_E5,
 NOTE_DS5,
 REST, REST, NOTE_DS4,
 NOTE_E4, REST, NOTE_FS4, NOTE_G4, REST, NOTE_DS4,
 NOTE_E4, NOTE_FS4,  NOTE_G4, NOTE_C5, NOTE_B4, NOTE_E4, NOTE_G4, NOTE_B4,
 NOTE_AS4, NOTE_A4, NOTE_G4, NOTE_E4, NOTE_D4,
 NOTE_E4,
 REST, REST, NOTE_E5, NOTE_D5, NOTE_B4, NOTE_A4, NOTE_G4, NOTE_E4,
 NOTE_AS4, NOTE_A4, NOTE_AS4, NOTE_A4, NOTE_AS4, NOTE_A4, NOTE_AS4, NOTE_A4,
 NOTE_G4, NOTE_E4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4
 };

 const float note_length[] =
 {
 8,4,2,2,
 3,3,2,3,3,2,
 2,2,2,2,2,2,2,2,
 9.33, 1.33, 1.33, 1.33, 1.33, 9.33,
 4,2,2,
 3,3,2,3,3,2,
 2,2,2,2,2,2,2,2,
 (16+4),
 4,2,2,
 3,3,2,3,3,2,
 2,2,2,2,2,2,2,2,
 9.33,1.33,1.33,1.33,1.33,9.33,
 8,
 4,2,2,2,2,2,2,
 1,3,1,3,1,3,1,3,
 1.33,1.33,1.33,2,2,8
 };
*/

 /*

 //Imperial march https://www.musicnotes.com/sheetmusic/mtd.asp?ppn=MN0103573
 #define TEMPO	108
 const uint16_t melody[] =
 {
 NOTE_F4,  NOTE_F4,  NOTE_F4,  NOTE_CS4, NOTE_GS4,
 NOTE_F4,  NOTE_CS4, NOTE_GS4, NOTE_F4,
 NOTE_C5,  NOTE_C5,  NOTE_C5,  NOTE_CS5, NOTE_GS4,
 NOTE_E4,  NOTE_CS4, NOTE_GS4, NOTE_F4,
 NOTE_F5,  NOTE_F4,  NOTE_F4,  NOTE_F5,  NOTE_E5, NOTE_DS5,
 NOTE_D5,  NOTE_CS5, NOTE_D5,  REST,     NOTE_FS4, NOTE_B4, NOTE_AS4, NOTE_A4,
 NOTE_GS4, NOTE_G4,  NOTE_GS4, REST,     NOTE_CS4, NOTE_E4, NOTE_CS4, NOTE_E4,
 NOTE_GS4, NOTE_F4,  NOTE_GS4, NOTE_C5,
 NOTE_F5,  NOTE_F4,  NOTE_F4,  NOTE_F5,  NOTE_E5,  NOTE_DS5,
 NOTE_D5,  NOTE_CS5, NOTE_D5,  REST,     NOTE_FS4, NOTE_B4, NOTE_AS4, NOTE_A4,
 NOTE_GS4, NOTE_FS4, NOTE_GS4, REST,     NOTE_CS4, NOTE_E4, NOTE_CS4, NOTE_GS4,
 NOTE_F4,  NOTE_CS4, NOTE_GS4, NOTE_F4
 };

 const float note_length[] =
 {
 4, 4, 4, 3, 1,
 4, 3, 1, 8,
 4, 4, 4, 3, 1,
 4, 3, 1, 8,
 4, 3, 1, 4, 3, 1,
 1, 1, 2, 2, 2, 4, 3, 1,
 1, 1, 2, 2, 2, 4, 3, 1,
 4, 3, 1, 8,
 4, 3, 1, 4, 3, 1,
 1, 1, 2, 2, 2, 4, 3, 1,
 1, 1, 2, 2, 2, 4, 3, 1,
 4, 3, 1, 8
 };
*/

//fun
void buzzing_task(void *argument) {
	uint8_t buzzing_type;
	TickType_t last_wake_time;
	uint16_t note_time;
	float semiquaver_time = 60000.0 / (4 * TEMPO);
	while (1) {
		xQueueReceive(buzzing_task_msg, &buzzing_type, portMAX_DELAY);
		switch (buzzing_type) {
		case song:
			for (uint16_t i = 0; i < sizeof(melody) / sizeof(uint16_t); i++) {
				last_wake_time = xTaskGetTickCount();
				buzzer(melody[i]);
				note_time = (semiquaver_time * note_length[i]);
				vTaskDelayUntil(&last_wake_time, note_time*0.95);
				buzzer(0);
				vTaskDelayUntil(&last_wake_time, note_time*0.05);
			}
			break;
		case ok:
			last_wake_time = xTaskGetTickCount();
			buzzer(BUZZER_HIGH);
			vTaskDelay(BUZZ_TIME);
			buzzer(0);
			vTaskDelay(GAP_TIME);
			buzzer(BUZZER_LOW);
			vTaskDelay(BUZZ_TIME);
			break;
		case not_ok:
			last_wake_time = xTaskGetTickCount();
			buzzer(BUZZER_HIGH);
			vTaskDelay(BUZZ_TIME);
			buzzer(0);
			vTaskDelay(GAP_TIME);
			buzzer(BUZZER_LOW);
			vTaskDelay(BUZZ_TIME);
			buzzer(0);
			vTaskDelay(GAP_TIME);
			buzzer(BUZZER_HIGH);
			vTaskDelay(BUZZ_TIME);
			buzzer(0);
			break;
		case control_control:
			last_wake_time = xTaskGetTickCount();
			buzzer(BUZZER_LOW);
			vTaskDelay(BUZZ_TIME);
			buzzer(0);
			vTaskDelay(GAP_TIME);
			buzzer(BUZZER_LOW);
			vTaskDelay(BUZZ_TIME);
			break;
		case control_keyboard:
			last_wake_time = xTaskGetTickCount();
			buzzer(BUZZER_HIGH);
			vTaskDelay(BUZZ_TIME);
			buzzer(0);
			vTaskDelay(GAP_TIME);
			buzzer(BUZZER_HIGH);
			vTaskDelay(BUZZ_TIME);
			buzzer(BUZZ_TIME);
			break;

		case control_sbc:
			buzzer(BUZZER_LOW);
			vTaskDelay(BUZZ_TIME);
			buzzer((BUZZER_HIGH+BUZZER_LOW)/2);
			vTaskDelay(BUZZ_TIME);
			buzzer(BUZZER_HIGH);;
			vTaskDelay(BUZZ_TIME);
			break;


		case bz_debug_low:
			buzzer(DEBUG_LOW_FREQ);
			vTaskDelay(BUZZ_TIME);
			break;

		case bz_debug_high:
			buzzer(DEBUG_HIGH_FREQ);
			vTaskDelay(BUZZ_TIME);
			break;

		case bz_debug_rest:
			buzzer(0);
			vTaskDelay(GAP_TIME);
			break;
		case bz_high:
			buzzer(BUZZER_HIGH);
			vTaskDelay(BUZZ_TIME);
			break;
		case bz_low:
			buzzer(BUZZER_LOW);
			vTaskDelay(BUZZ_TIME);
			break;
		}
		buzzer(0);
		vTaskDelay(GAP_TIME);
	}

//todo for showntelll
}
