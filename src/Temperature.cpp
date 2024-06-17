#include "flags.h"
#include "Temperature.h"

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

/**
 * Concatena dois inteiros
 *
 * @param a primeiro inteiro
 * @param b segundo inteiro
 * @return número concatenado
 */
int concatInt(int a, int b)
{
  char s1[128];
  char s2[128];
  // Converte e concatena os inteiros
  sprintf(s1, "%d", a);
  sprintf(s2, "%d", b);
  strcat(s1, s2);
  return atoi(s1);
}

Temperature::Temperature()
{
  findSensors(); // Identifica os sensores disponíveis
}

void Temperature::findSensors()
{
  // Inicializa e busca sensores disponíveis
  sensors.begin();
  numSensors = sensors.getDeviceCount();

  // Função para reteste, caso venha zerado 
  if (numSensors == 0)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    sensors.begin();
    numSensors = sensors.getDeviceCount();
  }

  // Quando não houver sensores representar com -1, assim é possível
  // implementar lógica no GW
  if (numSensors == 0)
    numSensors = -1;

  // Armazena os UUID dos sensores encontrados
  for (uint8_t i = 0; i < numSensors; i++)
  {
    sensors.getAddress(thermometer, i);
    for (uint8_t j = 0; j < 8; j++)
      sensorsUUID[i][j] = thermometer[j];
  }

  if (DEBUG_SENSORS)
  {
    captureTemperatures();
    Serial.println("\n**Searching for sensors");
    for (uint8_t i = 0; i < numSensors; i++)
    {
      Serial.printf("Sensor[%d] - UUID ", i);
      for (uint8_t j = 0; j < 8; j++)
      {
        Serial.print(sensorsUUID[i][j]);
      }
      float temp = temperatures[i];
      Serial.printf(" -> Small Id: %d -> Temp: %.2f\n", getSensorId(i), temp);
    }
  }
}

void Temperature::captureTemperatures()
{
  // Mede as temperaturas dos sensores e as armazena
  sensors.requestTemperatures();
  for (uint8_t i = 0; i < numSensors; i++)
    temperatures[i] = sensors.getTempC(sensorsUUID[i]);
}

int Temperature::getSensorId(uint8_t current)
{
  // Extrai o identificador agrupando em pares
  // Ex: 40 255 55 75 81 24 129 113 -> 40255 5575 8124 129113
  int a = concatInt(sensorsUUID[current][0], sensorsUUID[current][1]);
  int b = concatInt(sensorsUUID[current][2], sensorsUUID[current][3]);
  int c = concatInt(sensorsUUID[current][4], sensorsUUID[current][5]);
  int d = concatInt(sensorsUUID[current][6], sensorsUUID[current][7]);

  // Usando os valores define uma seed para o pseudo aleatório
  srand(a + b + c + d);

  // O primeiro número gerado é o seu id, isso garante consistência de quais
  // serão os novos ids, em 1000 sensores aprox. 10 vão repetir
  return rand() % 99999;
}

float Temperature::getTemperature(uint8_t index)
{
  return temperatures[index];
}
