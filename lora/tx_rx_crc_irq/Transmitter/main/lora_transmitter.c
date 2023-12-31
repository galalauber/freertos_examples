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
#include "freertos/queue.h"

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
 * Lora CRC;
 */
#include "lora_crc.h"

/**
 * Debug?
 */
#define DEBUG 1

/**
 * LoRa devices;
 */
#define LORA_TOTAL_NODES 2

/**
 * Endereçamento dos dispositivos da rede LoRa;
 * O MASTER sempre será o inicializa a comunicação com os SLAVES;
 * O MASTER possui endereço 0, enquanto os SLAVES são enumerados de 1 a 1000;
 */
#define MASTER_NODE_ADDRESS 0
#define LORA_RECEIVER_TIMEOUT_MS 5000

/**
 * Comandos;
 */
#define CMD_READ_COUNT 0
#define CMD_READ_ADC   1
#define CMD_SET_GPIO   2
#define CMD_PRINT_OLED 3
#define CMD_READ_RSSI  4

/**
 * Variáveis;
 */
const char * TAG = "main ";

static void read( void )
{
    int x;
    int cnt_1 = 0;
    uint8_t protocol[100];

    if( xQueueReceive( xQueue_LoRa, &cnt_1, LORA_RECEIVER_TIMEOUT_MS/portTICK_PERIOD_MS ) == pdTRUE )
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
             x = lora_receive_packet( protocol, sizeof(protocol) );
             
             /**
             * Protocolo;
             * <id_node_sender><id_node_receiver><command><payload_size><payload><crc>
             */
             if( x >= 6 && protocol[1] == MASTER_NODE_ADDRESS  )
             {                
                  /**
                   * Verifica CRC;
                   */
                  USHORT usCRC = usLORACRC16( protocol, 3 + protocol[3] + 1);
                  UCHAR ucLow =  (UCHAR)(usCRC & 0xFF);
                  UCHAR ucHigh = (UCHAR)((usCRC >> 8) & 0xFF);

                  if( ucLow == protocol[3 + protocol[3] + 1] && ucHigh == protocol[3 + protocol[3] + 2] )
                  {

                      switch( protocol[2] )
                      {
                          /**
                           * Imprime o valor de "Count" que foi incrementado pelo receptor a cada pacote
                           * recebido;
                          */
                          case CMD_READ_COUNT: 
                                ESP_LOGI( TAG, "Address = %d; Value = %d\n", protocol[0], ((protocol[5]<<8)|protocol[4]));
                                vTaskDelay( 30/portTICK_PERIOD_MS  );
                                return; 
                      }
                  }
              }

          }

        vTaskDelay( 1/portTICK_PERIOD_MS  );
    }
}

/**
 * Task responsável pela transmissão Tx via LoRa;
 */
static void task_tx( void *pvParameter )
{

   uint8_t protocol[100];

   for( ;; ) 
   {

      for( int i = 1; i <= LORA_TOTAL_NODES; ++i )
      {
         /**
         * Protocolo;
         * <id_node_sender><id_node_receiver><command><payload_size><payload><crc>
         */
          protocol[0] = MASTER_NODE_ADDRESS;  //master address node;
          protocol[1] = i;                    //slave address node;
          protocol[2] = CMD_READ_COUNT;        //comando de leitura do conversor ADCD0;
          protocol[3] = 0;                    //neste exemplo não há dados a serem enviados no payload;

          /**
           * Calcula o CRC do pacote;
           */
          USHORT usCRC = usLORACRC16( protocol, 4 );
          protocol[4] = (UCHAR)(usCRC & 0xFF); 
          protocol[5] = (UCHAR)((usCRC >> 8) & 0xFF);

          /**
           * Transmite protocol via LoRa;
           */
          lora_send_packet( protocol, 6 );

          /**
           * Após a transmissão a API configura o LoRa em modo de recepção;
           */

          if( DEBUG )
              ESP_LOGI( TAG, "Pacote Enviado para node = %d ", i ); 

          /**
           * Chama a função que irá receber o valor de Count, enviado pelo receptor;
           */
          read();

          /**
           * Delay;
           */
          vTaskDelay( 1000/portTICK_PERIOD_MS  );

      }
     


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

    char message[50];
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
   snprintf( message, sizeof(message), "Address=%d", MASTER_NODE_ADDRESS );
   ssd1306_out16( 0, 0, (char*) "Transmissor", WHITE );
   ssd1306_out16( 3, 0, (char*) message, WHITE );

   /**
    * Inicializa LoRa utilizando as configurações
    * definidas via menuconfig -> componentes -> lora;
    * Por padrão, a Api LoRa inicializa o rádio em modo de recepção;
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
    */
   lora_enable_crc();

   /**
    * Habilita a recepção LoRa via Interrupção Externa;
    */
   lora_enable_irq();

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