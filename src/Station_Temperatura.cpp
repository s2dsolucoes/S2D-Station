#include "header.h"

AESMessage aes;       // Criptografia AES 128 CBC
EEPROMStorage eeprom; // Ler e escrever na EEPROM
Temperature *temp;     // Sensores de temperatura

#define BAND 915E6

// *****************************************************************
// * Esqueletos de mensagens frequentementes usadas na comunicação

// Mensagem para a temperatura de um sensor
// Ex: {"id":"243NB4GX","id_sensor":12345,"temp":18.69}
const char *TEMP_RESPONSE = "{\"id\":\"%s\",\"id_sensor\":%d,\"temp\":%.2f}";

// Mensagem para a quantidade de sensores disponíveis
// Ex: {"id":"12345678",""type":1,"num_sensor":4}
const char *NUM_SENSORS_RESPONSE = "{\"id\":\"%s\",\"type\":\"%s\",\"num_sensor\":%d}";

// Mensagem com ok
// Ex: {"id":"12345678","message":"OK"}
const char *MESSAGE_RESPONSE = "{\"id\":\"%s\",\"message\":\"%s\"}";
// *****************************************************************

bool overrideId = ST_WRITE_ID;

char stationId[ST_ID_LEN] = STATION_ID; // 28MX6AI2, 23GIP2HD, 23VDDAA9, 25Z9TBVD, 24EYGB4I, 243NB4GX

// Controle LoRa
char LoRaMsg[256];      // Mensagem recebida
char responseLoRa[256]; // Mensagem para enviar
bool hasLoRaMsg = false;
bool hasLoRaConnection = false; // Está trocando mensagens com o GW?

// Controle do sensor que foi medido
uint8_t currentSensor = 0;

// Controle da busca de novos sensores
unsigned long findSensorsTimer = 0;

// Timeout para reiniciar o ESP
unsigned long restartTimer = 0;
// OTA
double update;       // Versão do código a ser instalada
void atualizarOTA(); // Função para atualizar o gateway via MQTT
#define VERSION_NUM_POS 44

void atualizarOTA()
{
  if (DEBUG)
  {
    Serial.println(("---------------------------------------"));
    Serial.println(("------------  Update! OTA  ------------"));
    Serial.println("Conectando em: " + String(servidorOTA));
    Serial.println(("---------------------------------------"));
  }

  // sprintf(endpointMessageUPD, MESSAGE_MASK, "UPD_STARTED");

  // publishCfg(endpointMessageUPD);
  // Serial.println();
  // Serial.print(("endpointMessageUPD: "));
  // Serial.print(endpointMessageUPD);
  // Serial.println();

  HTTPClient http;

  String url = servidorOTA.c_str();

  // Inicia a conexão HTTP
  if (http.begin(url))
  {
    // Começa o update!
    if (Update.begin(UPDATE_SIZE_UNKNOWN))
    {
      int httpCode = http.GET();
      if (httpCode == HTTP_CODE_OK)
      {
        WiFiClient &stream = http.getStream();
        while (stream.available())
        {
          size_t len = stream.readBytes(buffer, sizeof(buffer));
          if (len > 0)
          {
            if ((Update.write(buffer, len) != len) && DEBUG)
            {
              Serial.println(("Erro na escrita durante a update!!"));
              break;
            }
          }
        }
      }
      else
      {
        Serial.printf("Erro ao buscar o arquivo de update! Código HTTP: %d\n", httpCode);
      }
      http.end();

      if ((Update.end(true)) && DEBUG)
      {
        Serial.print(("\nSDIP-01 Atualizado para Versão: "));
        Serial.println(newVersion);

        // Verifica se a versão armazenada é diferente da nova
        int stored_num_version = EEPROM.readInt(VERSION_NUM_POS);
        if (newVersion != stored_num_version)
        {
          EEPROM.writeInt(VERSION_NUM_POS, newVersion);
          EEPROM.commit();
        }
        delay(4000);
        ESP.restart();
      }
      else
      {
        if (DEBUG)
        {
          Serial.println(("Erro ao finalizar update!"));
        }
      }
    }
    else
    {
      if (DEBUG)
      {
        Serial.println(("Erro ao iniciar update!"));
      }
    }
  }
  else
  {
    if (DEBUG)
    {
      Serial.println(("Erro na conexão HTTP!"));
    }
  }
}

void setup()
{
  Serial.begin(9600);

  // Inicializa a EEPROM do ESP32
  EEPROM.begin(256);
  delay(2000);

  // Setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  SPI.begin(5, 19, 27, 18);
  // 433E6 for Asia - 866E6 for Europe - 915E6 for North America
  while (!LoRa.begin(BAND))
  {
    Serial.println(F("."));
    delay(200);
  }

  // Instancia o objeto das temperatuas
  temp = new Temperature();

  Serial.println(F("LoRa Initializing OK!"));

  // Realiza o armazenamento do id passado por flag ao iniciar o station
  if (ST_WRITE_ID)
  {
    Serial.println(F("\n**Salvando id na EEPROM**\n"));
    eeprom.writeString(eeprom.ID_POSITION, STATION_ID, eeprom.ID_LEN);
  }

  // Busca os últimos dados armazenados na EEPROM (id do station)
  char storedId[eeprom.ID_LEN];
  eeprom.readString(eeprom.ID_POSITION, storedId, eeprom.ID_LEN);
  if (!strlen(storedId) == 0 && strlen(storedId) < 10)
  {
    strncpy(stationId, storedId, ST_ID_LEN);
  }

  Serial.println(F("\n**Info"));
  Serial.print(F("ID: "));
  Serial.println(stationId);
  Serial.print(F("Debug: "));
  Serial.println(DEBUG);
  Serial.print(F("Debug Sensors: "));
  Serial.println(DEBUG_SENSORS);
  Serial.println(F("**\n"));
}

/**
 * Recebe os pacotes criptografados por LoRa e os converte em instruções.
 * Ao receber um pacote no LoRa essa função é chamada para capturar o pacote.
 *
 * @param packetSize Tamanho do pacote recebido por LoRa
 */
void onReceiveLoRa(int packetSize)
{
  // Nenhum pacote recebido
  if (packetSize == 0 || packetSize > 255)
    return;

  // Recebeu um pacote, mantém ligado o station
  restartTimer = millis();

  if (DEBUG)
  {
    Serial.print(F("Reading packet size:"));
    Serial.printf("[%d]\n", packetSize);
  }

  // Realiza a leitura do pacote recebido
  char encrypted[packetSize];
  for (int i = 0; i < packetSize; i++)
  {
    encrypted[i] = (char)LoRa.read();
  }
  encrypted[packetSize] = '\0'; // Finaliza a criação da string

  // Decriptografa a mensagem recebida
  char decrypted[256];
  aes.decryptMessage(encrypted, decrypted);

  if (DEBUG)
  {
    Serial.print(F("Received LoRa: "));
    Serial.println(decrypted);
  }

  // Copia os dados das instruções...
  strncpy(LoRaMsg, decrypted, strlen(decrypted) + 1);

  hasLoRaMsg = true;
  // yield(); //Efetua o feed do SW WDT.
}

/**
 * Envia uma mensagem criptografada via LoRa para o GW
 *
 * @param msg mensagem a ser enviada
 */
void sendEncryptedLoRa(char *msg)
{
  // Faz a criptografia da mensagem
  uint8_t msgLen = strlen(msg);
  uint8_t encLen = aes.cipherLength(msgLen);
  char encrypted[encLen];
  aes.encryptMessage(msg, encrypted);

  // Repassa a mensagem para o Gateway via LoRa
  LoRa.beginPacket();
  LoRa.print(encrypted);
  LoRa.endPacket();

  if (DEBUG)
  {
    Serial.print(F("Sending LoRa: "));
    Serial.println(msg);
  }
}

/**
 * Captura as temperaturas e envia o número atual de sensores,
 * sinalizando o GW para iniciar os requests das temperaturas
 */
void startRequest()
{
  Serial.println("ta no condicional do start");
  // Captura as temperaturas no momento do request
  temp->captureTemperatures();
  currentSensor = 0;

  // Envia JSON com número de sensores do station
  sprintf(responseLoRa, NUM_SENSORS_RESPONSE, stationId, TYPE_SENSOR, temp->numSensors);
  sendEncryptedLoRa(responseLoRa);
}

/**
 * Executa a instrução recebida pelo Gateway, são dois tipos diferentes
 * de instruções que podem ser recebidas:
 *
 * 1 - Instruções de comunicação:
 * * START_REQUEST: inicia as requisições, enviando o número de sensores
 * * REQUEST: envia dados de um sensor específico
 * * END_REQUEST: finaliza as requisições
 * * OK: ???
 * * ERROR: ocorreu um erro, inicia novamente o processo das requisições
 * Ex: {"id": "123abc1ab", "message": "START_REQUEST"}
 *
 * 2 - Instrução de configuração:
 * * NEW_ID: altera o ID do dispositivo conforme recebido
 * Ex: {"id": "23gip2hd","new_id": "99gip2he"}
 */
void parsePackage()
{
  hasLoRaMsg = false;
  StaticJsonDocument<96> doc;
  DeserializationError error = deserializeJson(doc, LoRaMsg);

  if (error)
  {
    Serial.print(F("DeserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Extrai os dados do JSON
  const char *id = doc["id"];
  const char *new_id = doc["new_id"];
  const char *message = doc["message"];
  int msg_num = doc["msg_num"];

  // A instrução é para esse station?
  if (strcmp(stationId, id) == 0)
  {

    // Troca de mensagens entre os dispositivos
    if (message)
    {
      // * Inicia a requisição das temperaturas
      if (strcmp(message, "START_REQUEST") == 0)
      {
        hasLoRaConnection = true;
        startRequest();
        return;
      }

      // * Request de sensor por indice
      if (strcmp(message, "REQUEST") == 0)
      {
        // O sensor requisitado existe?
        if (hasLoRaConnection)
        {
          // Pega dados do sensor requisitado
          int sensorId = temp->getSensorId(msg_num);
          float temperature = temp->getTemperature(msg_num);

          // Cria o JSON e envia para o GW
          sprintf(responseLoRa, TEMP_RESPONSE, stationId, sensorId, temperature);
          sendEncryptedLoRa(responseLoRa);
          return;
        }

        // Retornar um erro, algum sensor foi removido ou o station
        // reiniciou durante o processo de comunicação
        sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "ERROR");
        sendEncryptedLoRa(responseLoRa);
        return;
      }

      // * Finaliza as requisições
      if (strcmp(message, "END_REQUEST") == 0)
      {
        hasLoRaConnection = false;
        return;
      }

      // * Recebeu ok do GW
      if (strcmp(message, "OK") == 0)
      {
        return;
      }

      // * Ocorreu algum erro, iniciar uma nova requisição
      if (strcmp(message, "ERROR") == 0)
      {
        startRequest();
        return;
      }
    }

    // * Atualiza o id do station
    if (new_id)
    {
      // Retorna um ok para o GW
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);

      // Seta o novo id
      strncpy(stationId, new_id, ST_ID_LEN);

      // Armazena o novo id na EEPROM
      eeprom.writeString(eeprom.ID_POSITION, new_id, eeprom.ID_LEN);

      Serial.print(F("Novo Identificador: "));
      Serial.println(stationId);
      Serial.print(F("Offset: "));
      Serial.println(eeprom.ID_POSITION);
      ESP.restart();
    }
  }
}

/**
 * Recebe pacotes via LoRa e executa as suas instruções
 *
 * Faz a busca de novos sensores
 *
 * Reinicia o station caso ficar muito tempo sem comunicar por LoRa
 *
 * ! Não pode haver delays, há chance de pular algum pacote recebido
 *
 */
void loop()
{
  // Realiza a leitura de um pacote do LoRa
  onReceiveLoRa(LoRa.parsePacket());

  // Executa a config recebida
  if (hasLoRaMsg)
  {
    parsePackage();
  }

  unsigned long now = millis();

  // Realiza a busca de novos sensores entre um intervalo
  // de tempo, 10 segundos
  if ((now - findSensorsTimer) > 10000)
  {
    temp->findSensors();
    findSensorsTimer = now;
  }

  // Se o station ficar muito tempo sem receber mensagem, 15 min,
  // ele é reiniciado
  if ((now - restartTimer) > 900000)
  {
    if (DEBUG)
    {
      Serial.println(F("\n**Reiniciando o Station**"));
      Serial.println(F("**Time out LoRa**\n"));
    }
    ESP.restart();
  }
  yield();
}
