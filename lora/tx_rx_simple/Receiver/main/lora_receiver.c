/* --------------------------------------------------------------------------
  Hardware: kit Heltec LoRa Oled ESP32 ;
  Espressif SDK-IDF: v4.2
 *  --------------------------------------------------------------------------

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
 * Task responsável pela recepção LoRa;
 */
void task_rx( void *pvParameter )
{
   int x;
   uint8_t buf[255];

   for( ;; ) 
   {
      /**
       * Algum byte foi recebido?
       * Realiza a leitura dos registradores de status do LoRa com o 
       * objetivo de verificar se algum byte recebido foi armazenado
       * na FIFO do rádio;
       */
      while( lora_received() ) 
      {
         /**
          * Sim, existe bytes na FIFO do rádio LoRa, portanto precisamos ler
          * esses bytes; A variável buf armazenará os bytes recebidos pelo LoRa;
          * x -> armazena a quantidade de bytes que foram populados em buf;
          */
         x = lora_receive_packet( buf, sizeof(buf) );

         /**
          * Imprime os bytes recebidos;
          */
         if( DEBUG )
             ESP_LOGI( TAG, "Recebidos: %.*s; Size=%d\n", x, (char*)buf, x );
      }

      /**
       * Delay entre cada leitura dos registradores de status do LoRa;
       */
      vTaskDelay( 100/portTICK_PERIOD_MS  );
   }
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
    * Sintaxe: ssd1306_out16( linha, coluna, ftring , fonte_color );
    */
   ssd1306_out16( 0, 0, "Receptor", WHITE );

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
   lora_set_frequency( 925.1e6 );
	 lora_set_tx_power(14);
	 lora_set_spreading_factor(7);
   lora_set_bandwidth(500E3);
   /**
    * Deseja habilitar o CRC no Payload da mensagem?
    */
   lora_enable_crc();

   /**
    * Cria a task de recepção LoRa;
    */
   if( xTaskCreate( task_rx, "task_rx", 1024*5, NULL, 5, NULL ) != pdTRUE )
   {
      if( DEBUG )
         ESP_LOGI( TAG, "error - Nao foi possivel alocar task_rx.\r\n" );  
      return;   
   }
}