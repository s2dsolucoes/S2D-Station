#include "header.h"

AESMessage aes; // Criptografia AES 128 CBC

#define BAND 915E6

#define ID_POSITION 0
#define LTRS_POSITION 18
#define OUT_MAX_POS 24
#define OUT_MIN_POS 32
#define VERSION_NUM_POS 44
#define IN_MAX_POS 52
#define IN_MIN_POS 60
#define ID_LEN 9 // Tamanho ID

bool overrideId = ST_WRITE_ID;

char stationId[ST_ID_LEN] = STATION_ID;

bool wm_nonblocking = false;

WiFiManager wm;
WiFiManagerParameter custom_field;

// Controle LoRa
char LoRaMsg[256];              // Mensagem recebida
char responseLoRa[256];         // Mensagem para enviar
bool hasLoRaMsg = false;        // Recebeu uma mensagem LoRa
bool hasLoRaConnection = false; // Está trocando mensagens com o GW?

// Controle do sensor que foi medido
uint8_t currentSensor = 0;

// Controle da busca de novos sensores
unsigned long findSensorsTimer = 0;

unsigned long now = 0; // Variavel que pega o valor do millis
bool lastState = 0;    // Variavel de teste para coletar dados do pino

bool hasChanges = false;

//* Variaveis sensor de pressão
double IN_MIN = EEPROM.read(IN_MIN_POS);
double IN_MAX = EEPROM.read(IN_MAX_POS);
double OUT_MIN = 0;
double OUT_MAX = 9.65;
double err_in_min = 148.81;
double err_in_max = 733.5;
double err_out_max = 20;
double err_out_min = 4;
double pressure1 = 0.00;
double pressure2 = 0.00;
double corrente1 = 0.00;
double corrente2 = 0.00;
double corrente_err1 = 0.00;
double corrente_err2 = 0.00;
int sensorId = 0001;
Adafruit_INA219 sens_pressure_1(0x40); // Sensor de Pressão INA219
Adafruit_INA219 sens_pressure_2(0x41); // Sensor de Pressão INA219
//****************************

volatile bool subidaDetectada = false;  // Variável para indicar detecção de borda de subida
volatile bool descidaDetectada = false; // Variável para indicar detecção de borda de descida
unsigned long ultimaSubida = 0;         // Último tempo de detecção de borda de subida
unsigned long ultimaDescida = 0;        // Último tempo de detecção de borda de descida
unsigned long intervaloMinimo = 500;    // Intervalo mínimo entre bordas de subida e descida (em milissegundos)

// Timeout para reiniciar o ESP
unsigned long restartTimer = 0;

// Acionamentos
int contador_acionamentos = 0;
unsigned long timestamp_ultimo_acionamento = 0;
unsigned long timestamp_pos = 0;

int counter = 0;

// Função para atualizar a versão do dispositivo via LoRa
void atualizarOTA();
//! *******************************************************************************************************************************************************

//! Coleta o nome do servidor WiFi
String getParam(String name)
{
  String value;
  if (wm.server->hasArg(name)) // Se o servidor possui um argumento representado pela variavel 'name'
  {
    value = wm.server->arg(name); // O valor da variavel 'name' é atrubuido a variavel 'value'
  }
  return value;
}
//! *******************************************************************************************************************************************************

//! *******************************************************************************************************************************************************
void saveParamCallback()
{
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}
//! *******************************************************************************************************************************************************

//! Conecta ao WiFi
void connectWifi(bool active)
{
  if (active)
  {
    WiFi.mode(WIFI_STA); // Set para modo station

    if (wm_nonblocking)
    {
      wm.setConfigPortalBlocking(false);
    }

    const char *custom_radio_str = "<br/><label for='customfieldid'>Custom Field Label</label><input type='radio' name='customfieldid' value='1' checked> One<br><input type='radio' name='customfieldid' value='2'> Two<br><input type='radio' name='customfieldid' value='3'> Three";
    new (&custom_field) WiFiManagerParameter(custom_radio_str);

    wm.addParameter(&custom_field);
    wm.setSaveParamsCallback(saveParamCallback);

    std::vector<const char *> menu = {"wifi", "info", "param", "sep", "restart", "exit"};
    wm.setMenu(menu);

    // Página com tema escuro
    wm.setClass("invert");

    wm.setConnectTimeout(20);      // Tenta conectar por 20 segundos antes de abrir o portal
    wm.setConfigPortalTimeout(60); // Fecha o portal de configuração após 1 min

    bool res;
    res = wm.autoConnect("S2D MPC100", "password"); // Nome e senha da rede

    if (!res)
    {
      Serial.println("Failed to connect or hit timeout");
      ESP.restart();
    }
    else
    {
      Serial.println("***Conectado!***\n ATUALIZANDO");
    }
  }
  else
  {
    WiFi.disconnect(true, false);
  }
}
//! *******************************************************************************************************************************************************

//! Atualização Over the Air
void atualizarOTA()
{
  if (DEBUG)
  {
    Serial.println(("---------------------------------------"));
    Serial.println(("------------  Update! OTA  ------------"));
    Serial.println("Conectando em: " + String(servidorOTA));
    Serial.println(("---------------------------------------"));
  }

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
              Serial.println("Erro na escrita durante a update!!");
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

      if (Update.end(true))
      {
        if (DEBUG)
        {
          Serial.print(("\nSDIP-01 Atualizado para Versão: "));
          Serial.println(newVersion);
        }

        // Verifica se a versão armazenada é diferente da nova
        int stored_num_version = EEPROM.readInt(VERSION_NUM_POS);
        if (newVersion != stored_num_version)
        {
          EEPROM.writeInt(VERSION_NUM_POS, newVersion);
          delay(300);
          EEPROM.commit();
        }
        delay(4000);
        ESP.restart();
      }
      else
      {
        if (DEBUG)
        {
          Serial.println("Erro ao finalizar update!");
        }
      }
    }
    else
    {
      if (DEBUG)
      {
        Serial.println("Erro ao iniciar update!");
      }
    }
  }
  else
  {
    if (DEBUG)
    {
      Serial.println("Erro na conexão HTTP!");
    }
  }
}
//! *******************************************************************************************************************************************************

//! Recebimento de pacotes LoRa
/*
 * @param packetSize Tamanho do pacote recebido por LoRa
 */
void onReceiveLoRa(int packetSize)
{
  //* Nenhum pacote recebido
  if (packetSize == 0)
  {
    return;
  }

  //* Recebeu um pacote, mantém ligado o station
  restartTimer = millis();

  if (DEBUG)
  {
    Serial.printf("Reading Packet Size: [%d]\n", packetSize);
  }

  //* Realiza a leitura do pacote recebido
  char encrypted[packetSize + 1];

  for (int i = 0; i < packetSize; i++)
  {
    encrypted[i] = (char)LoRa.read();
  }
  encrypted[packetSize] = '\0'; //* Finaliza a criação da string

  //* Decriptografa a mensagem recebida
  char decrypted[256];

  aes.decryptMessage(encrypted, decrypted);

  if (DEBUG)
  {
    Serial.print(("Received LoRa: "));
    Serial.println(decrypted);
  }

  //* Copia os dados das instruções
  strncpy(LoRaMsg, decrypted, strlen(decrypted) + 1);

  hasLoRaMsg = true;
  yield();
}
//! *******************************************************************************************************************************************************

//! Envia a mensagem criptografada por LoRa
/*
 * @param msg mensagem a ser enviada
 */
void sendEncryptedLoRa(char *msg)
{
  //* Faz a criptografia da mensagem
  uint8_t msgLen = strlen(msg);
  uint8_t encLen = aes.cipherLength(msgLen);
  char encrypted[encLen];
  aes.encryptMessage(msg, encrypted);

  //* Repassa a mensagem para o Gateway via LoRa
  LoRa.beginPacket();
  LoRa.print(encrypted);
  LoRa.endPacket();

  if (DEBUG)
  {
    Serial.printf("Sending LoRa msg:");
    Serial.print(msg);
  }
  delay(100);
}
//! *******************************************************************************************************************************************************

//! Faz a verificação da String
bool checkString(const char *p)
{
  if (p == NULL)
  {
    Serial.println("NULL");
    return false;
  }
  if (*p == '\0')
  {
    return false;
    Serial.println("Empty string!");
  }
  else
  {
    Serial.print("String: \"");
    Serial.print(p);
    Serial.println("\"");
    return true;
  }
}
//! *******************************************************************************************************************************************************

//! Resposta ao "START_REQUEST". Coleta id e tipo e envia ao GW
void startRequest()
{
  currentSensor = 0;

  //* Envia JSON com número de sensores do station
  sprintf(responseLoRa, NUM_SENSORS_RESPONSE, stationId, TYPE_SENSOR, 2);
  sendEncryptedLoRa(responseLoRa);
}
//! *******************************************************************************************************************************************************

//! Trata as mensagens
/*
 * Executa a instrução recebida pelo Gateway, são dois tipos diferentes
 * de instruções que podem ser recebidas:
 *
 * 1 - Instruções de comunicação:
 * * START_REQUEST: inicia as requisições, enviando o número de sensores
 * * REQUEST: envia dados de um sensor específico
 * * END_REQUEST: finaliza as requisições
 * * OK: Confirmação de funcionamento de processo
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
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, LoRaMsg);
  if (DEBUG)
  {
    Serial.print("passou do deserialize");
  }
  if (error)
  {
    Serial.print("DeserializeJson() failed: ");
    Serial.println(error.f_str());
    return;
  }

  //* Extrai os dados do JSON
  const char *id = doc["id"];
  const char *new_id = doc["new_id"];
  const char *message = doc["message"];
  double in_max = doc["in_max"];
  double in_min = doc["in_min"];
  double out_max = doc["out_max"];
  double out_min = doc["out_min"];
  int wifiMode = doc["wifiMode"];
  bool setDebug = doc["debug"];

  //* ID's conferem?
  if (strcmp(stationId, id) == 0)
  {
    if (DEBUG)
    {
      Serial.print("\nA serial é igual!");
    }
    //* Troca de mensagens entre os dispositivos
    if (message)
    {
      //* Recebeu ok do GW
      if (strcmp(message, "OK") == 0)
      {
        if (DEBUG)
        {
          Serial.print("Igual a OK, nao fez nada");
        }
        return;
      }

      if (strcmp(message, "UPDATE") == 0)
      {
        if (DEBUG)
        {
          Serial.print("update enviado para o station!!!");
        }
        delay(200);
        atualizarOTA();
        return;
      }

      //* Inicia a requisição dos dados
      if (strcmp(message, "START_REQUEST") == 0)
      {
        hasLoRaConnection = true;
        startRequest();
        return;
      }

      //* Request de sensor por indice
      if (strcmp(message, "REQUEST") == 0)
      {
        if (DEBUG)
        {
          Serial.println("<<<<<<<<<<REQUEST>>>>>>>>>>");
        }
        //* O sensor requisitado existe?
        if (hasLoRaConnection)
        {
          delay(200);
          sensorId = 0001;

          //* Se possui um ID
          if (strlen(stationId) != 0)
          {
            sprintf(responseLoRa, PRESSURE_RESPONSE, stationId, sensorId, pressure1, pressure2);

            if (DEBUG)
            {
              Serial.println();
              Serial.println("Mensagem montada/enviada Lora Através Request: ");
              Serial.println(responseLoRa);
              Serial.println();
            }
            sendEncryptedLoRa(responseLoRa);
          }
          return;
        }
      }
    }
    //* Algum sensor foi removido ou o station reiniciou durante o processo de comunicação
    sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
    sendEncryptedLoRa(responseLoRa);

    //* Finaliza as requisições
    if (strcmp(message, "END_REQUEST") == 0)
    {
      hasLoRaConnection = false;
      return;
    }

    //* Ocorreu algum erro, iniciar uma nova requisição
    if (strcmp(message, "ERROR") == 0)
    {
      startRequest();
      return;
    }

    //* Ativa ou desativa as mensagens de debug
    if (setDebug)
    {
      if (DEBUG)
      {
        Serial.printf("!!!!ESTADO DE DEBUG ATUALIZADO DE %d PARA %d", DEBUG, setDebug);
      }
      DEBUG = setDebug;
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    //* Atualiza os valores de saída máxima
    if (out_max)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! toMax Atualizado de %f, para %f", OUT_MAX, out_max);
      }
      OUT_MAX = out_max;
      EEPROM.writeDouble(OUT_MAX_POS, out_max);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    //* Atualiza os valores de saída minima
    if (out_min)
    {

      if (DEBUG)
      {
        Serial.printf("!!!! toMin Atualizado de %f, para %f", OUT_MIN, out_min);
      }
      OUT_MIN = out_min;
      EEPROM.writeDouble(OUT_MIN_POS, out_min);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    //* Atualiza os valores de entrada máxima
    if (in_max)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! in_max Atualizado de %f, para %f", IN_MAX, in_max);
      }
      IN_MAX = in_max;
      EEPROM.writeDouble(IN_MAX_POS, in_max);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    //* Atualiza os valores de entrada minima
    if (in_min)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! in_min Atualizado de %f, para %f", IN_MIN, in_min);
      }
      IN_MIN = in_min;
      EEPROM.writeDouble(IN_MIN_POS, in_min);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }
    //***************************************************************************************

    //* Habilita ou desabilita o WiFi
    if (wifiMode == 1)
    {
      Serial.println("\n***Wifi Habilitado!***\n");
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
      connectWifi(1);
      hasChanges = true;
    }
    else if (wifiMode == 2)
    {
      Serial.printf("\n***Wifi Desabilitado!***\n");
      connectWifi(false);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }
    //***************************************************************************************

    //* Atualiza o id do station
    if (new_id)
    {
      //* Retorna um ok para o GW
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);

      //* Set do novo id
      strncpy(stationId, new_id, ST_ID_LEN);

      //* Armazena o novo id na EEPROM
      for (int i = 0; i < ID_LEN; i++)
      {
        EEPROM.write(ID_POSITION + i, new_id[i]);
      }
      hasChanges = true;

      Serial.print(("Novo Identificador: "));
      Serial.println(stationId);
      Serial.print(("Offset: "));
      Serial.println(ID_POSITION);
      ESP.restart();
    }
    if (hasChanges)
    {
      vTaskDelay(200 / portTICK_PERIOD_MS);
      EEPROM.commit();
      hasChanges = false;
      Serial.printf("\n***********Commited!!!!!!***********\n");
    }
  }
  else
  {
    yield();
    return;
  }
}
//! *******************************************************************************************************************************************************

//! Reset do WiFi
void wifiReset()
{
  //* Verifica se um botão foi pressionado
  if (digitalRead(TRIGGER_PIN) == LOW)
  {
    //* Debounce
    delay(50);
    if (digitalRead(TRIGGER_PIN) == LOW)
    {
      Serial.println("Button Pressed");
      //* Se ainda continua presionando o botão após dois segundos reseta as configurações
      delay(2000);
      if (digitalRead(TRIGGER_PIN) == LOW)
      {
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        wm.resetSettings();
        ESP.restart();
      }
      connectWifi(true);
    }
  }
}
//! *******************************************************************************************************************************************************

//! Reset da EEPROM
void eepromReset()
{
  while ((digitalRead(buttonPin) != 1) && (counter < 4))
  {
    delay(1000);
    counter++;
  }

  if ((counter == 4) || (RESETEEPROM == 1)) //* Acionada pelo botão ou flag
  {
    for (int i = 0; i < 256; i++)
    {
      EEPROM.writeString(i, "$");
    }

    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    EEPROM.commit();
    if (DEBUG)
    {
      Serial.println("EEPROM APAGADA POR COMPLETO");
    }
    counter = 0;
  }
}
//! *******************************************************************************************************************************************************

//! Setup
/*
 * Recebe pacotes via LoRa e executa as suas instruções
 * Faz a busca de novos sensores
 * Reinicia o station caso ficar muito tempo sem comunicar por LoRa
 * //! Não pode haver delays, há chance de pular algum pacote recebido
 */
void setup()
{
  //* Inicia a Serial
  Serial.begin(9600);

  //* Inicializa a EEPROM do ESP32
  EEPROM.begin(256);

  //* Ativa os sensores de pressão
  if (!sens_pressure_1.begin() || !sens_pressure_2.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
    {
      delay(10);
    }
  }

  sens_pressure_1.setCalibration_32V_1A();
  sens_pressure_2.setCalibration_32V_1A();
  Serial.setDebugOutput(true);
  pinMode(TRIGGER_PIN, INPUT);

  //* Setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  SPI.begin(5, 19, 27, 18);

  if (DEBUG)
  {
    Serial.printf("\n ***DEBUG HABILITADO*** \n");
  }
  else
  {
    Serial.printf("\n ***DEBUG DESABILITADO*** \n");
  }

  while (!LoRa.begin(BAND))
  {
    Serial.println(F("."));
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("LoRa Initializing OK!");

  for (uint8_t i = 0; i < ID_LEN; i++)
  {
    stationId[i] = EEPROM.read(ID_POSITION + i); //* Lê um char por vez
  }
  stationId[ID_LEN - 1] = '\0'; //* Finaliza criação da string

  //* Realiza o armazenamento do id passado por flag ao iniciar o station
  if (ST_WRITE_ID)
  {
    if (strcmp(stationId, STATION_ID) != 0)
    {
      Serial.print("\n**Salvando id na EEPROM**\n");
      for (int i = 0; i < ID_LEN; i++)
      {
        EEPROM.write(ID_POSITION + i, STATION_ID[i]); // Escreve um caractere por vez
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
      EEPROM.commit(); // Finaliza a escrita na EEPROM
    }
  }

  if ((strlen(stationId) != 0) && (strlen(stationId) < 10))
  {
    strncpy(stationId, stationId, ST_ID_LEN);
  }

  Serial.println("\n**Info");
  Serial.print("ID: ");
  Serial.println(stationId);
  Serial.print("Debug: ");
  Serial.println(DEBUG);
  Serial.print("Debug Sensors: ");
  Serial.println(DEBUG_SENSORS);
  Serial.println("**\n");

  stampTimeWriten = millis(); //* Inicializa o tempo do stamp de tempo para gravar dados na eeprom

  OUT_MAX = EEPROM.readDouble(OUT_MAX_POS);
  OUT_MIN = EEPROM.readDouble(OUT_MIN_POS);

  if (isnan(OUT_MAX) || isnan(OUT_MIN)) //* Se não tem valores definidos reseta para os padrões
  {
    OUT_MAX = 9.65;
    OUT_MIN = 0;
    EEPROM.writeDouble(OUT_MAX_POS, OUT_MAX);
    EEPROM.writeDouble(OUT_MIN_POS, OUT_MIN);
    delay(200);
    EEPROM.commit();
    Serial.printf("\n        *** Atenção! ***     \n*** Valores originais atualizados para valores padrão***\n");
  }
  Serial.printf("\nValores Iniciados:\nTo Max: %f\nTo Min: %f\n", OUT_MAX, OUT_MIN);

  IN_MAX = EEPROM.readDouble(IN_MAX_POS);
  IN_MIN = EEPROM.readDouble(IN_MIN_POS);

  if (isnan(IN_MAX) || isnan(IN_MIN)) //* Se não tem valores definidos reseta para os padrões
  {
    IN_MAX = 11.75;
    IN_MIN = 3.85;
    EEPROM.writeDouble(IN_MAX_POS, IN_MAX);
    EEPROM.writeDouble(IN_MIN_POS, IN_MIN);

    vTaskDelay(200 / portTICK_PERIOD_MS);
    EEPROM.commit();
    Serial.printf("\n        *** Atenção! ***     \n*** Valores originais atualizados para valores padrão***\n");
  }
  Serial.printf("\nValores Iniciados:\nIN Max: %f\nIN Min: %f\n", IN_MAX, IN_MIN);

  lastState = digitalRead(GPIO_BOTAO);
}
//! *******************************************************************************************************************************************************

//! Loop
void loop()
{
  //* Lógica de reset do módulo
  eepromReset();
  wifiReset();
  //************************************************************************************

  if (wm_nonblocking)
  {
    wm.process();
  }

  //* Realiza a leitura de um pacote do LoRa
  onReceiveLoRa(LoRa.parsePacket());

  //* Executa a config recebida
  if (hasLoRaMsg)
  {
    parsePackage();
  }

  unsigned long now = millis();

  //* Realiza a busca de dados entre um intervalo de tempo, 10 segundos
  if ((now - findSensorsTimer) > 10000)
  {
    findSensorsTimer = now;

    corrente1 = 0.00;
    corrente2 = 0.00;

    //* Cálculo sensores de pressão
    for (int i = 0; i < 10; i++)
    {
      corrente_err1 = corrente_err1 + sens_pressure_1.getCurrent_mA();
      delay(10);
      corrente_err2 = corrente_err2 + sens_pressure_2.getCurrent_mA();
      delay(10);
    }
    corrente_err1 = corrente_err1 / 10;
    corrente_err2 = corrente_err2 / 10;

    corrente1 = ((corrente_err1 - err_in_min) * (err_out_max - err_out_min) / (err_in_max - err_in_min)) + err_out_min; //* Cálculo de correção de dados
    corrente2 = ((corrente_err2 - err_in_min) * (err_out_max - err_out_min) / (err_in_max - err_in_min)) + err_out_min; //* Cálculo de correção de dados

    pressure1 = ((corrente1 - IN_MIN) * (OUT_MAX - OUT_MIN) / (IN_MAX - IN_MIN)) + OUT_MIN;
    pressure2 = ((corrente2 - IN_MIN) * (OUT_MAX - OUT_MIN) / (IN_MAX - IN_MIN)) + OUT_MIN;

    yield();
  }
  //* Se o station ficar muito tempo sem receber mensagem, 15 min, ele é reiniciado
  if ((now - restartTimer) > 900000)
  {
    if (DEBUG)
    {
      Serial.print("\n**Reiniciando o Station**");
      Serial.println("**Time out LoRa**\n");
    }
    ESP.restart();
  }
  esp_task_wdt_reset();
  yield();
  delay(40);
}