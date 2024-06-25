# ESP32 Station

O Station tem como objetivo realizar a captura das profundidas e litros d'agua e enviar ao gateway quando solicitado. Essa solicitação é realizada usando comunicação LoRa.

# LoRa

A comunicação do Station com o Gateway é realizada através do LoRa, usando a criptografia AES-128-CBC para criptografar os dados comunicados.

## Mensagens recebidas

Muda o identificador de um Station, responde um OK ou ERRO.

```js
{"id":"XXXX","new_id":"YYYY"}
```

Dá inicio as requisições das profundidades e quantidade d'agua, deve responder com o número de sensores.

```js
{"id": "12345678", "message": "START_REQUEST"}
```

## Mensagens enviadas

Resposta com a quantidade de sensores no station.

```js
{"id":"12345678","num_sensor":4}
```

Responde com a profundidade e os a quantidade d'agua em litros lida por um determinado sensor.

```js
{"id":"561XS8FR","id_sensor": "0001", "ltrs": "25689.541", "id_sensor2": "0002", "depth": "86.54"}
```

Resposta com mensagem para o Gateway.

```js
{"id":"12345678","message":"OK"}
```

# Flags de configurações disponíveis

Essas flags definem certos comportamentos como também a identificação do Gateway, sendo as duas flags `ST_ID` obrigatório para compilar.

```bash
-DST_ID 	# Seta o id do station (Obrigatório)

-DDEBUG 			# Ativa modo verboso de debug
-DDEBUG_SENSORS		# Ativa modo debug dos sensores

-DST_WRITE_ID 		# Escreve o id na EEPROM ao iniciar

-DVERSION           # Informa a versão atual do Station
-DTYPESENSOR        # Informa o tipo do sensor (qual tipo de dados ele envia)  
```

# Atualização via OTA
## 1. Preparação do Arquivo de Firmware

1. Obtenha o arquivo de firmware mais recente e atualizado.
2. Abra o Visual Studio Code (VSCode).
3. Realize a build do projeto no VSCode.
4. Navegue até o diretório `.pio/build/esp-station`.
5. Copie o arquivo `firmware.bin` gerado para a pasta `firmwares`.

## 2. Transferência do Firmware para o Servidor

1. Abra o Putty e conecte-se ao servidor.
2. Faça login no servidor.
3. Acesse a root do servidor utilizando o comando:

    ```bash
    sudo su
    ```

4. Copie o arquivo `firmware.bin` para o local correto no servidor com o comando:

    ```bash
    cp /arquivos/firmwares/firmware.bin /var/www/sistemas/public/
    ```

## 3. Configuração via MQTT

1. Acesse o MQTT.
2. Navegue até o gateway que você deseja atualizar.
3. Entre no tópico `cfg` do gateway, de modo que o caminho fique:

    ```text
    s2d/cg19g/*gateway que gerencia o station que queremos atualizar*/cfg
    ```

4. Envie o seguinte comando via MQTT para iniciar a atualização:

    ```json
    {
      "id_gateway": *id do gateway que gerencia o station*,
      "id": *id do station que será atualizado*,
      "wifiMode": 1
    }
    ```

## 4. Finalização
O station ativará sua conexão WiFi e se atualizará via OTA, desligando o WiFi assim que a atualização estiver completa.