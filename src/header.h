#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <string> // ? Remover ?
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/pcnt.h" // Driver para Contagem de Pulsos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "soc/gpio_sig_map.h"
#include "flags.h"
#include "AESMessage.h"
#include <Update.h>
#include <HTTPClient.h>
#include <EEPROM.h>
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Wire.h>
#include <Adafruit_INA219.h> // INA219 logic

#define TRIGGER_PIN 13 // Função reset

#define GPIO_BOTAO 4
#define TEMPO_DEBOUNCE 1000 // ms
bool blocked = 0; // variavel bloqueio clique após incremento


#include "esp_task_wdt.h"

/***ESP 32 station***/
#define ss 18
#define rst 14
#define dio0 26
#define buttonPin 13
#define led 17
/****/
int value;
int previousvalue;

#define RED 0
#define SILVER 1
int currentstate = SILVER;

unsigned long nbRead = 0;

#define HIGH_THRESHOLD 60
#define LOW_THRESHOLD 30

long contentLength = 0;
bool isValidContentType = false;
String servidorOTA = "http://att.s2d.inf.br/firmware.bin"; // www.sistemas.s2d.inf.br/firmwareMPC100.bin
int portaOTA = 80;                                         // NÃO funciona com HTTPS   //será alterado automaticamente pela EEPROM
// String arquivoOTA = "";  // será alterado automaticamente pela EEPROM
//  Atualizações:
int newVersion = 0; // Variavel contendo a versão atual apartir de uma atualização
int timerOta = 0;
double EPVersion = VERSION;  // Versão atual dispositivo
const int bufferSize = 1024; // Tamanho do buffer em bytes (pode ajustar conforme necessário)
uint8_t buffer[bufferSize];  // Declaração do buffer como um array de bytes

// Tamanho padrão do id (id + fim da string)
#define ST_ID_LEN 9

// Tamanho Padrão valor persistente dos litros
#define VAL_LT_LEN 9

// Contador Absoluto Max: 4.294.967.295 // 8 Casas será necessário (Incremental) Valor deverá ser gravado na eeprom
volatile double ltrs = 0;

// Variaveis controle de tempo entre gravações na eeprom
int stampTimeWriten = 0;
int timeWriten = 15000;

// Fator de Multiplicação Litros = Pulsos * pulseValue
double pulseValue = 1;

bool DEBUG = true;

const char *TAG = "MAIN";

// *****************************************************************
// * Esqueletos de mensagens frequentementes usadas na comunicação

// Mensagem que informa a pressão de dois sensores
// Ex: {"id":"561XS8FR","id_sensor": 0001, "pressure1": 21, "pressure2": 23}
const char *PRESSURE_RESPONSE = "{\"id\":\"%s\",\"id_sensor\":%d,\"pressure1\":%.2f,\"pressure2\":%.2f}";

// Mensagem para a quantidade de sensores disponíveis
// Ex: {"id":"12345678","num_sensor":4}
const char *NUM_SENSORS_RESPONSE = "{\"id\":\"%s\",\"type\":\"%s\",\"num_sensor\":%d}";

// Mensagem com ok
// Ex: {"id":"12345678","message":"OK"}
const char *MESSAGE_RESPONSE = "{\"id\":\"%s\",\"message\":\"%s\"}";
// *****************************************************************