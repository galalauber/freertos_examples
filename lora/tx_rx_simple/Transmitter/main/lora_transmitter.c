/* --------------------------------------------------------------------------
   Hardware: kit Heltec LoRa Oled ESP32 ;
   Espressif SDK-IDF: v3.2 e v3.3
    --------------------------------------------------------------------------

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * Lib Standar C;
 */
#include <stdio.h>
#include <string.h>

/**
 * FreeRTOs;
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Lib LoRa;
 * Localizado em Componentes > lora;
 */
#include "lora.h"

/**
 * Logs;
 */
#include "esp_err.h"
#include "esp_log.h"

/**
 * Lib Display SSD1306 Oled;
 * Atenção: Inclua este arquivo apenas 1x;
 * Caso contrário, inclua "lib_ss1306.h"
 */
#include "lib_heltec.h"

/**
 * Debug?
 */
#define DEBUG 1

/**
 * Variáveis;
 */
const char * TAG = "main ";

/**
 * Task responsável pela transmissão Tx via LoRa;
 */
void task_tx( void *pvParameter )
{
  
   int packet_number = 0;
   char message[255];

   for( ;; ) 
   {
      /**
       * Transmite via LoRa ~255 bytes;
       */
      snprintf( message, sizeof(message), "[Num: %d] LoRa Technology is the DNA of IoT, "\
                                          "connecting sensors to the Cloud and enabling real-time"\
                                          "communication of data and analytics that can be utilized"\
                                          "to enhance efficiency and productivity. Semtech's LoRa devices"\
                                          "the open LoRaWAN protocol.", ++packet_number );
      
      lora_send_packet( (uint8_t*) message, strlen( message ) + 1 );

      if( DEBUG )
         ESP_LOGI( TAG, "packet sent... %s Size=%d\n", message, strlen( message ) + 1 );

      /**
       * Delay;
       */
      vTaskDelay( 500/portTICK_PERIOD_MS  );
   }

   /**
    * Esta linha não deveria ser executada...
    */
   vTaskDelete( NULL );
}

/**
 * Inicio do app;
 */
void app_main( void )
{
    /**
    * Inicializa display Oled 128x64 SSD1306;
    * As configurações de pinagens do Oled são encontradas
    * em "lib_heltec.h";
    */
   ssd1306_start();

   /**
    * Imprime usando fonte8x16;
    * Sintaxe: ssd1306_out16( linha, coluna, string , fonte_color );
    */
   ssd1306_out16( 0, 0, "Transmissor", WHITE );

   /**
    * Inicializa LoRa utilizando as configurações
    * definidas via menuconfig -> componentes -> lora
    * Spreading Factor  = 7;
    * Potência TX = 14 dbm;
    * Modo explicito: ( existe o campo header );
    */
   lora_init();

   /**
    * A frequência licenciada no Brasil é a 915Mhz; 
    * Verifique a frequência do seu dispositivo LoRa; 
    * conforme: 433E6 para Asia; 866E6 para Europa e 915E6 para EUA;
    */
   lora_set_frequency( 915e6 );

   /**
    * Deseja habilitar o CRC no Payload da mensagem?
    * <PAYLOAD><PAYLOAD_CRC>
    */
   lora_enable_crc();

   /**
    * Cria a task de transmissão LoRa;
    */
   if( xTaskCreate( task_tx, "task_tx", 1024*5, NULL, 2, NULL ) != pdTRUE )
   {
      if( DEBUG )
         ESP_LOGI( TAG, "error - Nao foi possivel alocar task_tx.\r\n" );  
      return;   
   }
}