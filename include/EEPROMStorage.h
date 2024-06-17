#ifndef A9C7FEF7_B75C_4309_BDD9_D21D9224CD94
#define A9C7FEF7_B75C_4309_BDD9_D21D9224CD94
#ifndef EEPROM_STORAGE
#define EEPROM_STORAGE

#include <EEPROM.h>

class EEPROMStorage
{
public:
  uint8_t ID_POSITION;
  uint8_t ID_LEN;

  /**
  * Método Construtor
  *
  * Inicializa a lib do EEPROM, evitando esquecer de colocar no
  * setup do código
  *
  * Define as posições iniciais e o tamanho dos dados armazenados
  */
  EEPROMStorage();

  /**
  * Aramazena uma string na EEPROM
  *
  * @param addrOffset offset da posição inicial da string
  * @param str ponteiro da string a ser salva
  * @param len tamanho da string
  */
  void writeString(uint8_t addrOffset, const char *str, uint8_t len);

  /**
  * Realiza a leitura de uma string na EEPROM
  *
  * @param addrOffset offset da posição inicial da string
  * @param result ponteiro da string que irá receber o valor
  * @param len tamanho da string
  */
  void readString(uint8_t addrOffset, char *result, uint8_t len);
};

#endif


#endif /* A9C7FEF7_B75C_4309_BDD9_D21D9224CD94 */
