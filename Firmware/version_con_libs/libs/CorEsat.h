#ifndef CorEsat_h
#define CorEsat_h

//Declaración de los pines de comunicaciones de los modulos
#define CE_PIN 7                                     //Definicion del pin CE para el RF24
#define CSN_PIN 8 


//~~~~~~~~~~~~~~~~~~~Inicializacion de modulos~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~Prototipado de Funciones~~~~~~~~~~~~~~~~~~~~~
void configureSensorTSL2561(void);
void AdqTSL2561();
void configBmp180();
void AdqBmp180();
void escribirVectorBpm180();
void configIMU();
void AdqImu();
void configSerie();                               //Confoguracion del puerto serie
void configRadio();
int leerDato();
void detenerEscucha();
void comenzarEscucha();
void escribirDato(int16_t dato);
void escribirVectorIMU();
void escribirTSL2561();
void LEDs();
#endif
