/* --------------------------------------------------------------------------
  Hardware: kit Heltec LoRa Oled ESP32 ;
  Espressif SDK-IDF: v4.4.1
 *  --------------------------------------------------------------------------

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * Standard Lib C;
 */
#include <stdio.h>
#include <string.h>

/**
 * FreeRTOs;
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Lib Display SSD1306 Oled;
 * Atenção: Inclua este arquivo apenas 1x;
 * Caso contrário, inclua "lib_ss1306.h"
 */
#include "lib_heltec.h"

/**
 * Logs;
 */
#include "esp_err.h"
#include "esp_log.h"

/**
 * Debug?
 */
#define DEBUG 1

/**
 * Variaveis;
 */
static const char * TAG = "MAIN";

/**
 * Task responsável em imprimir no display Oled
 * o valor de uma variável contadora;
 */
void task_oled( void * vParameters )
{
   char buffer[20]; 
   int count = 0;

   for( ;; ) 
   {
      /**
       * Converte count para string;
       */
      snprintf( buffer, sizeof(buffer), "Cnt:%d ", count++ );

      /**
       * Imprime no display oled o valor de count FONTE 8X8;
       */
      ssd1306_out8( 3, 0, buffer, WHITE );

      /**
       * Imprime na saída do console;
       */
      if( DEBUG )
          ESP_LOGI( TAG, "Count: %s\n", buffer );

      /**
       * delay;
       */
      vTaskDelay( pdMS_TO_TICKS(1000) );
   }
}

void app_main( void )
{
   /**
    * Inicializa display Oled 128x64 SSD1306;
    * As configurações de pinagens do Oled são encontradas
    * em "lib_heltec.h";
    */
   ssd1306_start();

   /**
    * imprime na saída do console;
    */
   if( DEBUG )
       ESP_LOGI( TAG, "oled ssd1306 inicializado...");

   /**
    * Carrega imagem no Oled;
    * Sintaxe: ssd1306_image( string , fonte_color );
    * onde fonte_color = WHITE ou BLACK;
    */
   ssd1306_image( mikro_logo, WHITE );

   /**
    * Imprime usando fonte8x16;
    * Sintaxe: ssd1306_out16( linha, coluna, ftring , fonte_color );
    */
   ssd1306_out16( 0, 0, "Bom Dia", WHITE );

   /**
    * Cria task_oled responsável;
    */
   if( xTaskCreate( &task_oled, "task_oled", 1024 * 2 , NULL, 2, NULL ) != pdTRUE )
   {
      if( DEBUG )
         ESP_LOGI( TAG, "error - Nao foi possivel alocar task_oled.\r\n" );  
      return;   
   }
}