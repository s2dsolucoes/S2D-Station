#ifndef AC6FE9B7_73EB_44B8_AF46_2BF6A92938D3
#define AC6FE9B7_73EB_44B8_AF46_2BF6A92938D3
#ifndef TEMP_SENSORS
#define TEMP_SENSORS

#define oneWireBus 4
#define MAX_SENSORS 255

#include <OneWire.h>
#include <DallasTemperature.h>

class Temperature
{
private:
  DeviceAddress thermometer;

  uint8_t sensorsUUID[MAX_SENSORS][8]; // Identificadores únicos dos sensores
  float temperatures[MAX_SENSORS];

public:
  int numSensors;

  /**
  * Método Construtor
  */
  Temperature();

  /**
  * Busca novos sensores
  */
  void findSensors();

  /**
   * Captura as temperaturas dos sensores e as armazena
   * para serem utilizadas posteriormente
   */
  void captureTemperatures();

  /**
   * A partir do número do sensor gera um identificador compacto, é utilizado
   * números pseudo randômicos pré definidos pelo pŕoprio UUID do sensor
   *
   * @param current qual o sensor a ser buscado
   * @return representação do UUID de um sensor
   */
  int getSensorId(uint8_t current);

  /**
  * Retorna a temperatura de um determinado sensor
  *
  * @param index qual sensor vai ser medido
  * @return temperatura em Celsius
  */
  float getTemperature(uint8_t index);
};

#endif


#endif /* AC6FE9B7_73EB_44B8_AF46_2BF6A92938D3 */
