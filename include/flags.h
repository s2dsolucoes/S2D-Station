// Macros para gerar string de flags
#define STRINGIFY(s) STRINGIFY1(s)
#define STRINGIFY1(s) #s

// Define o id setada na flag
#define STATION_ID STRINGIFY(ST_ID)

// Define o tipo do sensor setado na flag
#define TYPE_SENSOR STRINGIFY(TYPESENSOR)

// Define se deve escrever o id na EEPROM ao iniciar
#ifndef ST_WRITE_ID
#define ST_WRITE_ID 0
#endif

// Define a flag de debug dos sensores
#ifndef DEBUG_SENSORS
#define DEBUG_SENSORS 0
#endif
