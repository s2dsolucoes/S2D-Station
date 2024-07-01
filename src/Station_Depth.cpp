#include "header.h"

AESMessage aes; // Criptografia AES 128 CBC

#define BAND 915E6

#define ID_POSITION 0
#define LTRS_POSITION 18
#define DEPTH_OUT_MAX_POS 24
#define DEPTH_OUT_MIN_POS 32
#define DEPTH_POS 36
#define VERSION_NUM_POS 44 // Posição Informação de Versão
#define MULT_LTRS_POS 50
#define DEPTH_IN_MIN_POS 55
#define DEPTH_IN_MAX_POS 60
#define ID_LEN 9 // Tamanho ID

// *****************************************************************
// * Esqueletos de mensagens frequentementes usadas na comunicação

// Mensagem para a profundidade e leitura de metros cubicos de um sensor
// Ex: {"id":"561XS8FR","id_sensor": "0001", "ltrs": "25689.541", "id_sensor2": "0002", "depth": "86.54"}
const char *DEPTH_RESPONSE = "{\"id\":\"%s\",\"id_sensor\":\"%d\",\"ltrs\":\"%.3lf\",\"id_sensor2\":\"%d\",\"depth\":\"%.2f\"}";

// Mensagem para a quantidade de sensores disponíveis
// Ex: {"id":"12345678","num_sensor":4}
const char *NUM_SENSORS_RESPONSE = "{\"id\":\"%s\",\"type\":\"%s\",\"num_sensor\":\"%d\"}";

// Mensagem com ok
// Ex: {"id":"12345678","message":"OK"}
const char *MESSAGE_RESPONSE = "{\"id\":\"%s\",\"message\":\"%s\"}";
// *****************************************************************

bool overrideId = ST_WRITE_ID;

char stationId[ST_ID_LEN] = STATION_ID; // 28MX6AI2, 23GIP2HD, 23VDDAA9, 25Z9TBVD, 24EYGB4I, 243NB4GX

bool wm_nonblocking = false; // change to true to use non blocking

WiFiManager wm;                    // global wm instance
WiFiManagerParameter custom_field; // global param (for non blocking w params)

// Controle LoRa
char LoRaMsg[256];      // Mensagem recebida
char responseLoRa[256]; // Mensagem para enviar
bool hasLoRaMsg = false;
bool hasLoRaConnection = false; // Está trocando mensagens com o GW?

// Controle do sensor que foi medido
uint8_t currentSensor = 0;

// Controle da busca de novos sensores
unsigned long findSensorsTimer = 0;

unsigned long now = 0; // Variavel que pega o valor do millis
bool lastState = 0;    // Variavel de teste para coletar dados do pino

double ltrsBurned;

double depth_out_max = 0.00;
double depth_out_min = 0.00;
bool hasChanges = false;
double setDepth = 0.00;
double multLtrs = 0.00;

volatile bool subidaDetectada = false;  // Variável para indicar detecção de borda de subida
volatile bool descidaDetectada = false; // Variável para indicar detecção de borda de descida
unsigned long ultimaSubida = 0;         // Último tempo de detecção de borda de subida
unsigned long ultimaDescida = 0;        // Último tempo de detecção de borda de descida
unsigned long intervaloMinimo = 500;    // Intervalo mínimo entre bordas de subida e descida (em milissegundos)

// Timeout para reiniciar o ESP
unsigned long restartTimer = 0;

// Objetos Pulse Counter

// Variáveis para Funcionamento ADC
double DEPTH_IN_MIN = EEPROM.read(DEPTH_IN_MIN_POS); //! Valores a atualizar
double DEPTH_IN_MAX = EEPROM.read(DEPTH_IN_MAX_POS); //! Valores a atualizar
double DEPTH_OUT_MIN = 0;                            //! Valores a atualizar
double DEPTH_OUT_MAX = 9.65;                         //! Valores a atualizar
double corrente = 0;
Adafruit_INA219 sens_pressure_1(0x40); //* Sensor de Pressão INA219
double depth;

int16_t count = 0;

unsigned long multPulses = 0;

int contador_acionamentos = 0;
unsigned long timestamp_ultimo_acionamento = 0;
unsigned long timestamp_pos = 0;

portBASE_TYPE res;

void atualizarOTA();

/* Função ISR (chamada quando há geração da interrupção) */
void IRAM_ATTR funcao_ISR()
{
  /* Conta acionamentos do botão considerando debounce */
  if ((millis() - timestamp_ultimo_acionamento) >= TEMPO_DEBOUNCE)
  {
    ltrs = ltrs + multLtrs;
    timestamp_ultimo_acionamento = millis();
  }
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
  {
    return;
  }

  // Recebeu um pacote, mantém ligado o station
  restartTimer = millis();

  if (DEBUG)
  {
    Serial.printf("Reading Packet Size: [%d]\n", packetSize);
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
  yield(); // Efetua o feed do SW WDT.
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
    Serial.printf("Sending LoRa msg: %c", *msg);
  }
  delay(500);
}
//! ****************************************************************************************************************************************

//! Start Request
/**
 * No momento captura o id e o tipo dos sensores e envia para o GW
 */
void startRequest()
{
  currentSensor = 0;

  // Envia JSON com número de sensores do station
  sprintf(responseLoRa, NUM_SENSORS_RESPONSE, stationId, TYPE_SENSOR, 2);
  sendEncryptedLoRa(responseLoRa);
}
//! ****************************************************************************************************************************************

//! Check String
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
//! ****************************************************************************************************************************************

//! ****************************************************************************************************************************************
String getParam(String name)
{
  String value;
  if (wm.server->hasArg(name)) // Se o servidor possui um argumento representado pela variavel 'name'
  {
    value = wm.server->arg(name); // O valor da variavel 'name' é atrubuido a variavel 'value'
  }
  return value;
} //! ****************************************************************************************************************************************

//! ****************************************************************************************************************************************
void saveParamCallback()
{
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}
//! ****************************************************************************************************************************************

//! WiFi Setup
void connectWifi(bool active)
{
  if (active)
  {
    WiFi.mode(WIFI_STA);

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

    wm.setClass("invert");

    wm.setConfigPortalTimeout(80);

    bool res;
    res = wm.autoConnect("S2D MPC100", "s2dsolucoes15");

    if (!res)
    {
      Serial.println("Failed to connect or hit timeout");
    }
    else
    {
      Serial.println("***Conectado!***\n ATUALIZANDO");
      atualizarOTA();
    }
  }
  else
  {
    WiFi.disconnect(true, false);
  }
}
//! ****************************************************************************************************************************************

//! Parse Package
/**
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
  if (DEBUG)
  {
    Serial.print("Entrou no ParsePackage");
  }
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

  // Extrai os dados do JSON
  const char *id = doc["id"];
  const char *new_id = doc["new_id"];
  const char *message = doc["message"];
  double depth_out_max = doc["depth_out_max"];
  double depth_out_min = doc["depth_out_min"];
  double depth_in_min = doc["depth_in_min"];
  double depth_in_max = doc["depth_in_max"];
  double setDepthValue = doc["setDepthValue"];
  double mCubics = doc["mCubics"];
  double multLtrsValue = doc["multLtrsValue"];
  int wifiMode = doc["wifiMode"];
  bool setDebug = doc["debug"];
  bool reset = doc["reset"];

  // A instrução é para esse station?
  if (strcmp(stationId, id) == 0)
  {
    if (DEBUG)
    {
      Serial.print("\nA serial é igual!");
    }
    // Troca de mensagens entre os dispositivos
    if (message)
    {
      // * Recebeu ok do GW
      if (strcmp(message, "OK") == 0)
      {
        return;
      }
      // * Inicia a requisição dos dados
      if (strcmp(message, "START_REQUEST") == 0)
      {
        hasLoRaConnection = true;
        startRequest();
        return;
      }

      // * Request de sensor por indice
      if (strcmp(message, "REQUEST") == 0)
      {
        Serial.println("<<REQUEST>>");
        // O sensor requisitado existe?
        if (hasLoRaConnection)
        {
          double cubic = ltrs / 1000.00;
          int sensorId = 0001;
          int sensorId2 = 0002;

          //*DEPTH_RESPONSE = "{\"id\":\"%s\",\"type\":\"%s\",\"id_sensor\":%d,\"ltrs\":%.3lf,\"id_sensor2\":%d,\"depth\":%.2f}";
          if (strlen(stationId) != 0)
          {
            sprintf(responseLoRa, DEPTH_RESPONSE, stationId, sensorId, cubic, sensorId2, depth);

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
        // Retornar um erro, algum sensor foi removido ou o station reiniciou durante o processo de comunicação
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

      // * Ocorreu algum erro, iniciar uma nova requisição
      if (strcmp(message, "ERROR") == 0)
      {
        startRequest();
        return;
      }
    }

    if (reset == 1)
    {
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
      if (DEBUG)
      {
        Serial.println("Resetando o station...");
      }
      delay(500);
      ESP.restart();
    }

    // * Ativa ou desativa as mensagens de debug
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

    //* Atualiza os valores de entrada máxima
    if (depth_in_max)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! in_max Atualizado de %f, para %f", DEPTH_IN_MAX, depth_in_max);
      }
      DEPTH_IN_MAX = depth_in_max;
      EEPROM.writeDouble(DEPTH_IN_MAX_POS, depth_in_max);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    //* Atualiza os valores de entrada minima
    if (depth_in_min)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! in_min Atualizado de %f, para %f", DEPTH_IN_MIN, depth_in_min);
      }
      DEPTH_IN_MIN = depth_in_min;
      EEPROM.writeDouble(DEPTH_IN_MIN_POS, depth_in_min);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    // * Atualiza os valores maximo
    if (depth_out_max)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! depth_out_max Atualizado de %f, para %f", DEPTH_OUT_MAX, depth_out_max);
      }
      DEPTH_OUT_MAX = depth_out_max;
      EEPROM.writeDouble(DEPTH_OUT_MAX_POS, depth_out_max);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    if (depth_out_min)
    {

      if (DEBUG)
      {
        Serial.printf("!!!! depth_out_min Atualizado de %f, para %f", DEPTH_OUT_MIN, depth_out_min);
      }
      DEPTH_OUT_MIN = depth_out_min;
      EEPROM.writeDouble(DEPTH_OUT_MIN_POS, depth_out_min);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    if (mCubics)
    {
      if (DEBUG)
      {
        Serial.printf("!!!! Litros Acumulados Atualizados  de %f, para %f", ltrs, mCubics);
      }
      ltrs = mCubics * 1000;
      EEPROM.writeDouble(LTRS_POSITION, ltrs);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    if (multLtrsValue)
    {
      if (multLtrsValue != multLtrs)
      {
        if (DEBUG)
        {
          Serial.printf("\n!!!! Metros Cubicos/Litros Hidrometro: de %fm³, para %fm³ !!!!\n", multLtrs, multLtrsValue);
        }
        multLtrs = multLtrsValue;
        EEPROM.writeDouble(MULT_LTRS_POS, multLtrsValue);
        hasChanges = true;
        sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
        sendEncryptedLoRa(responseLoRa);
      }
    }

    if (setDepthValue)
    {
      if (setDepthValue != setDepth)
      {
        if (DEBUG)
        {
          Serial.printf("\n!!!! Valor Profundidade Atualizado de %fm, para %fm\n", setDepth, setDepthValue);
        }
        setDepth = setDepthValue;
        EEPROM.writeDouble(DEPTH_POS, setDepthValue);
        hasChanges = true;
        sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
        sendEncryptedLoRa(responseLoRa);
      }
    }
    if (wifiMode == 1)
    {
      Serial.println("\n***Wifi Habilitado!***\n");
      connectWifi(1);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }
    else if (wifiMode == 2)
    {
      Serial.printf("\n***Wifi Desabilitado!***\n");
      connectWifi(false);
      hasChanges = true;
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);
    }

    // * Atualiza o id do station
    if (new_id)
    {
      // Retorna um ok para o GW
      sprintf(responseLoRa, MESSAGE_RESPONSE, stationId, "OK");
      sendEncryptedLoRa(responseLoRa);

      // Set do novo id
      strncpy(stationId, new_id, ST_ID_LEN);

      // Armazena o novo id na EEPROM
      for (int i = 0; i < ID_LEN; i++)
      {
        EEPROM.write(ID_POSITION + i, new_id[i]); // Escreve um char por vez
      }
      // EEPROM.commit(); // Finaliza a escrita na EEPROM
      hasChanges = true;

      Serial.print(F("Novo Identificador: "));
      Serial.println(stationId);
      Serial.print(F("Offset: "));
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
}
//! ****************************************************************************************************************************************

//! Checagem de botão
/**
 * @brief Lógica para dar reset nas configurações usando o botao prog na placa
 *
 */
void checkButton()
{
  // check for button press
  if (digitalRead(TRIGGER_PIN) == LOW)
  {
    // debounce
    delay(50);
    if (digitalRead(TRIGGER_PIN) == LOW)
    {
      Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings
      delay(3000);
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
//! ****************************************************************************************************************************************

//! Atualização via OTA
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

//! ****************************************************************************************************************************************

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
//! Setup
void setup()
{
  Serial.begin(9600);

  Serial.setDebugOutput(true);
  pinMode(TRIGGER_PIN, INPUT);

  // Inicializa a EEPROM do ESP32
  EEPROM.begin(256);

  //* Ativa os sensores de pressão
  while (!sens_pressure_1.begin())
  {
    if (DEBUG)
    {
      Serial.println("Failed to find INA219 chip");
    }
    delay(200);
  }

  sens_pressure_1.setCalibration_32V_1A();

  pinMode(GPIO_BOTAO, INPUT);
  attachInterrupt(GPIO_BOTAO, funcao_ISR, RISING);

  // Setup LoRa transceiver module
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
  Serial.println(F("LoRa Initializing OK!"));

  checkButton(); // Verifica se o botao está pressionado para dar reset

  for (uint8_t i = 0; i < ID_LEN; i++)
  {
    stationId[i] = EEPROM.read(ID_POSITION + i); // Lê um char por vez
  }
  stationId[ID_LEN - 1] = '\0'; // Finaliza criação da string

  // Realiza o armazenamento do id passado por flag ao iniciar o station
  if (ST_WRITE_ID)
  {
    if (strcmp(stationId, STATION_ID) != 0)
    {
      Serial.println("\n**Salvando id na EEPROM**\n");
      for (int i = 0; i < ID_LEN; i++)
      {
        EEPROM.write(ID_POSITION + i, STATION_ID[i]); // Escreve um char por vez
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
      EEPROM.commit(); // Finaliza a escrita na EEPROM
    }
  }

  if (strlen(stationId) != 0 && strlen(stationId) < 10)
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

  stampTimeWriten = millis(); // Inicializa o tempo do stamp de tempo para gravar dados na eeprom

  ltrs = EEPROM.readDouble(LTRS_POSITION);
  depth_out_max = EEPROM.readDouble(DEPTH_OUT_MAX_POS);
  depth_out_min = EEPROM.readDouble(DEPTH_OUT_MIN_POS);
  setDepth = EEPROM.readDouble(DEPTH_POS);
  multLtrs = EEPROM.readDouble(MULT_LTRS_POS);

  if (isnan(multLtrs))
  {
    EEPROM.writeDouble(MULT_LTRS_POS, 1.00);
    multLtrs = 1.00;
    ltrs = 1;
    depth_out_max = 1;
    depth_out_min = 1;
    setDepth = 1;

    vTaskDelay(200 / portTICK_PERIOD_MS);
    EEPROM.commit();
    Serial.printf("\n        *** Atenção! ***     \n*** Valor original atualizado para 1***\n");
  }
  Serial.printf("\nValores Iniciados:\nLitros acumulados: %f\nTo Max: %f\nTo Min: %f\n Profundidade: %f\n Multiplicador Litros: %f\n", ltrs, depth_out_max, depth_out_min, setDepth, multLtrs);

  ltrsBurned = ltrs;

  DEPTH_OUT_MAX = EEPROM.readDouble(DEPTH_OUT_MAX_POS);
  DEPTH_OUT_MIN = EEPROM.readDouble(DEPTH_OUT_MIN_POS);

  if (isnan(DEPTH_OUT_MAX) || isnan(DEPTH_OUT_MIN)) //* Se não tem valores definidos reseta para os padrões
  {
    DEPTH_OUT_MAX = 20; 
    DEPTH_OUT_MIN = 4;    
    EEPROM.writeDouble(DEPTH_OUT_MAX_POS, DEPTH_OUT_MAX);
    EEPROM.writeDouble(DEPTH_OUT_MIN_POS, DEPTH_OUT_MIN);
    delay(200);
    EEPROM.commit();
    Serial.printf("\n        *** Atenção! ***     \n*** Valores originais atualizados para valores padrão***\n");
  }
  Serial.printf("\nValores Iniciados:\nTo Max: %f\nTo Min: %f\n", DEPTH_OUT_MAX, DEPTH_OUT_MIN);

  DEPTH_IN_MAX = EEPROM.readDouble(DEPTH_IN_MAX_POS);
  DEPTH_IN_MIN = EEPROM.readDouble(DEPTH_IN_MIN_POS);

  if (isnan(DEPTH_IN_MAX) || isnan(DEPTH_IN_MIN)) //* Se não tem valores definidos reseta para os padrões
  {
    DEPTH_IN_MAX = 11.75; //! Valores a atualizar
    DEPTH_IN_MIN = 3.85;  //! Valores a atualizar
    EEPROM.writeDouble(DEPTH_IN_MAX_POS, DEPTH_IN_MAX);
    EEPROM.writeDouble(DEPTH_IN_MIN_POS, DEPTH_IN_MIN);

    vTaskDelay(200 / portTICK_PERIOD_MS);
    EEPROM.commit();
    Serial.printf("\n        *** Atenção! ***     \n*** Valores originais atualizados para valores padrão***\n");
  }
  Serial.printf("\nValores Iniciados:\nIN Max: %f\nIN Min: %f\n", DEPTH_IN_MAX, DEPTH_IN_MIN);

  lastState = digitalRead(GPIO_BOTAO);
}
//! ****************************************************************************************************************************************

//! Loop
void loop()
{
  // Atualiza o estado anterior do botão

  if (ltrs != ltrsAnterior)
  {
    Serial.printf(">>>Valor de LTRS NOVA UPDATE: %f<<<\n", ltrs);
    // Atualiza o valor anterior de ltrs
    ltrsAnterior = ltrs;
  }

  if (wm_nonblocking)
  {
    wm.process();
  }

  // Realiza a leitura de um pacote do LoRa
  onReceiveLoRa(LoRa.parsePacket());

  // Executa a config recebida
  if (hasLoRaMsg)
  {
    parsePackage();
  }

  unsigned long now = millis();

  // Realiza a busca de novos sensores entre um intervalo de tempo, 10 segundos
  if ((now - findSensorsTimer) > 10000)
  {
    findSensorsTimer = now;

    depth = 0;

    for (int i = 0; i < 10; i++)
    {
      depth = depth + sens_pressure_1.getCurrent_mA();
      delay(30);
    }

    depth = depth / 10;

    corrente = ((corrente - DEPTH_IN_MIN) * (DEPTH_OUT_MAX - DEPTH_OUT_MIN) / (DEPTH_IN_MAX - DEPTH_IN_MIN)) + DEPTH_OUT_MIN;
    Serial.printf("corrente: %.2f mA, Depth: %.2fm", corrente, depth);
    Serial.println();
  }

  if (ltrsBurned != ltrs)
  {
    if ((millis() - stampTimeWriten) > timeWriten) // A cada 15s grava ltrs na eeprom
    {
      EEPROM.writeDouble(LTRS_POSITION, ltrs);
      //?Delay?
      vTaskDelay(200 / portTICK_PERIOD_MS);
      EEPROM.commit(); // Finaliza a escrita na EEPROM
      Serial.printf("Valor de LTRS (If updated): %f\n", ltrs);
      ltrsBurned = ltrs;
      stampTimeWriten = millis();
    }
  }

  // Se o station ficar muito tempo sem receber mensagem, 15 min, ele é reiniciado
  if ((now - restartTimer) > 900000)
  {
    if (DEBUG)
    {
      Serial.println("\n**Reiniciando o Station**");
      Serial.println("**Time out LoRa**\n");
    }
    ESP.restart();
  }
  esp_task_wdt_reset();
  delay(40);
}