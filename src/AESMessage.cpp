#include "AESMessage.h"

// Chave privada
byte aes_key[] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};

// Posição inicial do IV
byte aes_iv[N_BLOCK] = {0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A, 0x6A};

AESMessage::AESMessage()
{
  // Seta o tipo de padding padrão usado nas mensagens
  aesLib.set_paddingmode(paddingMode::CMS);
}

void AESMessage::resetIv()
{
  for (uint8_t i = 0; i < 16; i++)
  {
    aes_iv[i] = 0x6A;
  }
}

uint16_t AESMessage::cipherLength(uint16_t msgLen)
{
  return aesLib.get_cipher64_length(msgLen);
}

void AESMessage::encryptMessage(char *msg, char *result)
{
  int encLen = strlen(msg);
  aesLib.encrypt64((const byte*)msg, encLen, result, aes_key, sizeof(aes_key), aes_iv);
  resetIv(); // Reseta o IV depois de usar a lib
}

void AESMessage::decryptMessage(char *msg, char *result)
{
  uint16_t decLen = strlen(msg);
  aesLib.decrypt64(msg, decLen, (byte*)result, aes_key, sizeof(aes_key), aes_iv);
  resetIv(); // Reseta o IV depois de usar a lib
}
