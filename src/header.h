#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <string> // ? Remover ?
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiManager.h>
#include "flags.h"
#include "AESMessage.h"
#include "EEPROMStorage.h"
#include "Temperature.h"

/***ESP 32 station***/
#define ss 18
#define rst 14
#define dio0 26

// Tamanho padrão do id (id + fim da string)
#define ST_ID_LEN 9

// OTA
long contentLength = 0;
bool isValidContentType = false;
String servidorOTA = "http://att.s2d.inf.br/firmware.bin"; // www.sistemas.s2d.inf.br/firmwareMPC100.bin
int portaOTA = 80;                                         // NÃO funciona com HTTPS   //será alterado automaticamente pela EEPROM
//  Atualizações:
int newVersion = 0; // Variavel contendo a versão atual apartir de uma atualização
int timerOta = 0;
double EPVersion = VERSION;  // Versão atual dispositivo
const int bufferSize = 512; // Tamanho do buffer em bytes (pode ajustar conforme necessário) //! Tamanho de memoria reduzido
uint8_t buffer[bufferSize];  // Declaração do buffer como um array de bytes



