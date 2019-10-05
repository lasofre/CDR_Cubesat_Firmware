/*Proyecto: CorEsat
 *Segmento: Satelite.
 *Creado por el grupo CorEsat del club de robotica de la 
 *Universidad tecnologica nacional Facultad regional Cordoba, Argentina.
 *Web: https://clubderobotica.github.io/
 */
#include <SFE_BMP180.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Wire.h>//para I2C
#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>
#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>
/*Definimos los pines para los leds testigos*/
#define LEDTX 4      
#define LED_Temp 3
#define LED_EPS 2
#define LED_NN 5
//Declaración de los pines de comunicaciones de los modulos
#define CE_PIN 7                                     //Definicion del pin CE para el RF24
#define CSN_PIN 8                                    //Definicion del pin CSN para el RF24
//Comandos
#define ComandoIMU     'i'              //pedir vector IMU.
#define ComandoBmp180  'p'              //pedir Vector bmp180.
#define ComandoTSL2561 't'              //pedir Valor  TSL2561.
#define ComandoNull    '0'              //Borra el comando. 
#define ComandoMenu    'm'              //Muestra nuevamente el menú.
#define StatusSen      'j'              //Pide el estado de los sensores vector (0:TSL2561|1:Bmp180|2:MPU-9250) 1 configurado,0 desconfigurado

//~~~~~~~~~~~~~~~~~~~Inicializacion de modulos~~~~~~~~~~~~~~~~~~~~~
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
MPU9250_DMP imu;                                    //SDA - Pin A4 SCL - Pin A5
RF24 radio(CE_PIN, CSN_PIN);                        // Hardware configuration: Set up nRF24L01 radio on SPI bus (pins 10, 11, 12, 13) plus pins 7 & 8
SFE_BMP180 bmp180; 
//~~~~~~~~~~~~~~~~~~~Variables Globales~~~~~~~~~~~~~~~~~~~~~
byte addresses[][6] = {"1Node","2Node"};
double IMU[9]={0};                                     //vector para imu
double VectorBmp180[3]={0};                            //0-T 1-P 2-A (vector para BMP180)
double TSL2561=0;                                    //valor lux
float LED1;
float LED2;
float LED3;
float PresionNivelMar = 1013.25;                   //presion sobre el nivel del mar en mbar
String Comand;
char sensors_status[3]={'0'};                         // 0 no se configuró correctamente , 1 se configuró correctamente
                                                    //Vector sensors_status|0:TSL2561|1:Bmp180|2:MPU-9250
//~~~~~~~~~~~~~~~~~~~Prototipado de Funciones~~~~~~~~~~~~~~~~~~~~~
void configureSensorTSL2561(void);                //Configura el sensor de luz.
void configSerie();                               //Configura del puerto serie.
void configRadio();                               //Configura la comunicacion por radio.
void configBmp180();                              //Configura el sensor de Presion.
void configIMU();                                 //Configura el Ascelerometro, magnetometro y giroscopio.
void AdqBmp180();                                 //Adquisicion del sensor de Presion.
void AdqTSL2561();                                //Adquisicion del sensor de luz.
void AdqImu();                                    //Adquisicion del Ascelerometro, magnetometro y giroscopio .
void comenzarEscucha();                           //Iniciar escucha de la radio (Ejecutar las tres funciones en secuencia).
int leerDato();                                   //Leer dato que le está llegando a la radio.
void detenerEscucha();                            //Detener escucha de la radio.
void escribirDato(int16_t dato);                  //Transmitir dato.
void escribirVectorIMU();                         //Transmitir IMU.
void escribirTSL2561();                           //Transmitir datos del sensor de luz.
void escribirVectorBpm180();                      //Transmitir datos del sensor de presion.
void escribirEstadodesensores();                  //Transmitir el estado de los sensores.
void LEDs();                                      //Manejar estado de los leds.

//~~~~~~~~~~~~~~~~~~~Funciones Propias de Arduino~~~~~~~~~~~~~~~~~~~~~





void setup() { 
 int i;
for(i=0;i<13;i++)pinMode(i,INPUT);
pinMode(A0,INPUT);pinMode(A1,INPUT);pinMode(A2,INPUT);pinMode(A3,INPUT);pinMode(A4,INPUT);pinMode(A5,INPUT);pinMode(A6,INPUT);
pinMode(LEDTX,OUTPUT);  // led de transmision
pinMode(LED_Temp,OUTPUT);
pinMode(LED_EPS,OUTPUT);
pinMode(LED_NN,OUTPUT);

for(int i=0; i<3; i++){
digitalWrite(LEDTX, HIGH);
digitalWrite(LED_Temp, HIGH);
digitalWrite(LED_EPS, HIGH);
digitalWrite(LED_NN, HIGH);

delay(200);
digitalWrite(LEDTX, LOW);
digitalWrite(LED_Temp, LOW);
digitalWrite(LED_EPS, LOW);
digitalWrite(LED_NN, LOW);
delay(200);

  }
for(i=0;i<13;i++)
  configBmp180();
  configIMU();
  configSerie();
  configRadio();
  configureSensorTSL2561();  


  
}
void loop() {
 int data=0;
 Comand[0] = ComandoNull;  
 AdqBmp180(); 
 LEDs();
  while(Comand[0] = ComandoNull){
    if (Serial.available()){
    Comand=Serial.readString();
    Serial.println("<Estacion> "+String(Comand[0]));
    break;
      } 
  }  
        switch(Comand[0]){
          case StatusSen:
          Serial.println(sensors_status);
          break;
          case ComandoIMU:    AdqImu();
          Serial.println("Accel: " + String(IMU[0]) + ", " + String(IMU[1]) + ", " + String(IMU[2]) + " g");
          Serial.println("Gyro: " + String(IMU[3]) + ", " + String(IMU[4]) + ", " + String(IMU[5]) + " dps");
          Serial.println("Mag: " + String(IMU[6]) + ", " + String(IMU[7]) + ", " + String(IMU[8]) + " uT");                        
          break;
          case ComandoBmp180: AdqBmp180();escribirVectorBpm180();
          Serial.println("Temperatura: " + String(VectorBmp180[0]) + "[°C], Presion:" + String(VectorBmp180[1]) + "[mbar], Altitud " + String(VectorBmp180[2]) + " [m]"); 
          break;
          case ComandoTSL2561:AdqTSL2561();
          Serial.println("Ilumninacion: " + String(TSL2561) + " Lux");            
          break;
          default:Serial.println("Comando erroneo! ");
        }          


}








//~~~~~~~~~~~~~~~~~~~Funciones prototipadas~~~~~~~~~~~~~~~~~~~~~

void configureSensorTSL2561(void) {
   /* Initialise the sensor */
  if(!tsl.begin())
  {
    Serial.print("**TSL2561 no detectado");
  }
  else{
    Serial.println(">TSL2561 iniciado");
    sensors_status[0]='1';
    tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  }

}
void AdqTSL2561(){
  if(sensors_status[0]=='1'){
  sensors_event_t event; /*Obtener un nuevo evento de sensor */ 
  tsl.getEvent(&event);
  TSL2561 = event.light;/*Muestra el resultado de la luz meduda en lux*/
  }

}
void configBmp180(){
    if (bmp180.begin()){
      Serial.println(">BMP180 iniciado");
      sensors_status[1]='1';
    } else  {
      Serial.println("**Error al iniciar el BMP180");
    }
}
void AdqBmp180(){
  char status;
  double T,P,A;
  status = bmp180.startTemperature(); //Inicio de lectura de temperatura
  if (status != 0)
  {   
    delay(status); //Pausa para que finalice la lectura
    status = bmp180.getTemperature(T); //Obtener la temperatura
    if (status != 0)
    {
      status = bmp180.startPressure(3); //Inicio lectura de presión
      if (status != 0)
      {        
        delay(status); //Pausa para que finalice la lectura        
        status = bmp180.getPressure(P,T); //Obtener la presión
        if (status != 0)
        {       
          VectorBmp180[0]=(float)T;   
          VectorBmp180[1]=(float)P;   
          A= bmp180.altitude(P,PresionNivelMar); //Calcular altura 
          VectorBmp180[2]=(float)A; 

        }      
      }      
    }   
  } 
}
void escribirVectorBpm180(){
  radio.write(&VectorBmp180, sizeof(VectorBmp180)); 
}
void configIMU(){
  if (imu.begin() != INV_SUCCESS)  {
           Serial.println("**Comunicacion con MPU-9250 desabilitada");
           delay(100);
  } 
  else{
  sensors_status[2]='1';
  Serial.println(">MPU-9250 iniciado");
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(10); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(50); // Set mag rate to 10Hz
  }
  }
void AdqImu(){
   if ( imu.dataReady() )  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    IMU[0]= imu.calcAccel(imu.ax);
    IMU[1]= imu.calcAccel(imu.ay);
    IMU[2]= imu.calcAccel(imu.az);
    IMU[3]= imu.calcGyro(imu.gx)/57.3;
    IMU[4]= imu.calcGyro(imu.gy)/57.3;
    IMU[5]= imu.calcGyro(imu.gz)/57.3;
    IMU[6] = imu.calcMag(imu.mx);
    IMU[7] = imu.calcMag(imu.my);
    IMU[8] = imu.calcMag(imu.mz);
  }  
}
void configSerie(){
  Serial.begin(9600);
  Serial.println("--CorEsat--"); 
}
void configRadio(){
  radio.begin();
  // Set the transmit power to lowest available to prevent power supply related issues
  radio.setPALevel(RF24_PA_MIN);
  // Set the speed of the transmission to the quickest available
  radio.setDataRate(RF24_2MBPS);
  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  radio.setChannel(124);
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();
}
int leerDato(){
  int rx;
    while (radio.available()) {
      radio.read( &rx, sizeof(int));//Lee y lo guarda en una variabe
    }
  return rx;
}
void detenerEscucha(){
  // Ensure we have stopped listening (even if we're not) or we won't be able to transmit
  // No more data to get so send it back but add 1 first just for kicks
  // First, stop listening so we can talk
  radio.stopListening(); 
}
void comenzarEscucha(){
   radio.startListening();
}
void escribirDato(int16_t dato){
  radio.write( &dato, sizeof(int16_t) ); 
}

void escribirVectorIMU(){
  radio.write(&IMU, sizeof(IMU)); 
}

void escribirTSL2561(){
  radio.write(&TSL2561, sizeof(float)); 
}
void LEDs(){
  configBmp180();
  char status;
  double t;
  
  
  sensors_event_t event;
  tsl.getEvent(&event);
  LED3 = event.light;
   status = bmp180.startTemperature(); //Inicio de lectura de temperatura
   delay(status);
   status = bmp180.getTemperature(t); //Obtener la temperatura

  if(t>25)
  {
    digitalWrite(LED_Temp, HIGH);}
  else {
    digitalWrite(LED_Temp, LOW);}
  

  if(LED3>700){
    digitalWrite(LED_EPS, HIGH);}
  else {
    digitalWrite(LED_EPS, LOW);}
    
  
}
void escribirEstadodesensores(){
  radio.write(&sensors_status, sizeof(sensors_status)); 
}
void menuSerie(){
  Serial.println("MOC de CORESat.");
  Serial.println("(m) menu.");
  Serial.println("(j) Estado de los sensores.");  
  Serial.println("(i) Datos IMU.");
  Serial.println("(p) Datos TPA.");
  Serial.println("(t) Datos Lux.");   
}
