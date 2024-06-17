# ESP8266 Station

O Station tem como objetivo realizar a captura das temperaturas com sensores e enviar ao gateway quando solicitado. Essa solicitação é realizada usando comunicação LoRa.

# LoRa

A comunicação do Station com o Gateway é realizada através do LoRa, usando a criptografia AES-128-CBC para cifrar os dados comunicados.

## Mensagens recebidas

Muda o identificador de um Station, responde um OK ou ERRO.

```js
{"id":"XXXX","new_id":"YYYY"}
```

Dá inicio as requisições das temperaturas, deve responder com o número de sensores.

```js
{"id": "12345678", "message": "START_REQUEST"}
```

## Mensagens enviadas

Resposta com a quantidade de sensores no station.

```js
{"id":"12345678","num_sensor":4}
```

Resposta a temperatura de um determinado sensor.

```js
{"id":"243NB4GX","id_sensor":12345,"temp":18.69}
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
```
