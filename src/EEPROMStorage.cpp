#include "EEPROMStorage.h"

EEPROMStorage::EEPROMStorage()
{
  // Inicializando os valores default dos dados que podem ser armazenados
  // Se houverem mais dados a serem armazenados seguir essa mesma lógica...
  ID_POSITION = 0;
  ID_LEN = 9; // 8 dos dados e 1 da finalização da string '\0'
}

void EEPROMStorage::writeString(uint8_t addrOffset, const char *str, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + i, str[i]); // Escreve um char por vez
  }
  EEPROM.commit(); // Finaliza a escrita na EEPROM
}

void EEPROMStorage::readString(uint8_t addrOffset, char *result, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    result[i] = EEPROM.read(addrOffset + i); // Lê um char por vez
  }
  result[len] = '\0'; // Finaliza criação da string
}
