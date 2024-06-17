#ifndef AES_MESSAGE
#define AES_MESSAGE

#include <AESLib.h>

class AESMessage
{
private:
  AESLib aesLib;

  /**
  * Reseta o IV do AES
  */
  void resetIv();

public:
  /**
  * Método Construtor
  *
  * Define o padding usado por padrão nessa criptografia
  */
  AESMessage();

  /**
  * Gera o tamanho que irá resultar a criptografia
  *
  * @param msgLen tamanho da mensagem
  * @return tamanho da mensagem criptografada
  */
  uint16_t cipherLength(uint16_t msgLen);

  /**
  * Realiza a criptografia de uma mensagem para AES-128-CBC
  *
  * @param msg mensagem a ser criptografada
  * @param result vetor de char que irá receber o resultado
  */
  void encryptMessage(char *msg, char *result);

  /**
  * Descriptografa uma mensagem para AES-128-CBC
  *
  * @param msg mensagem a ser descriptografada
  * @param result vetor de char que irá receber o resultado
  */
  void decryptMessage(char *msg, char *result);
};

#endif
