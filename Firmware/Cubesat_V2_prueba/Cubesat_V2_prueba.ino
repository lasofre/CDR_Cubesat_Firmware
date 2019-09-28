/*Proyecto: 
 * 
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
#define ComandoLink    'C'
#define ComandoAdq     'A'
#define ComandoWrite   'W'
#define ComandoIMU     'I'                          //pedir vector IMU
#define ComandoBmp180  'P'                          //pedir Vector bmp180
#define ComandoTSL2561 'T'                          //pedir Valor  TSL2561
#define ComandoNull    '0'
#define StatusCom     'L'
#define StatusWrite   'R'
#define StatusAdq     'F'


//~~~~~~~~~~~~~~~~~~~Inicializacion de modulos~~~~~~~~~~~~~~~~~~~~~
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
MPU9250_DMP imu;                                    //SDA - Pin A4 SCL - Pin A5
RF24 radio(CE_PIN, CSN_PIN);                        // Hardware configuration: Set up nRF24L01 radio on SPI bus (pins 10, 11, 12, 13) plus pins 7 & 8
SFE_BMP180 bmp180; 

//~~~~~~~~~~~~~~~~~~~Variables Globales~~~~~~~~~~~~~~~~~~~~~

byte addresses[][6] = {"1Node","2Node"};
double IMU[9];                                     //vector para imu
double VectorBmp180[3];                            //0-T 1-P 2-A (vector para BMP180)
double TSL2561;                                    //valor lux
float LED1;
float LED2;
float LED3;
float PresionNivelMar = 1013.25;                   //presion sobre el nivel del mar en mbar
bool sensors_status[3]={0};                         // 0 no se configuró correctamente , 1 se configuró correctamente
                                                    //Vector sensors_status|0:TSL2561|1:Bmp180|2:MPU-9250
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


//~~~~~~~~~~~~~~~~~~~Funciones Propias de Arduino~~~~~~~~~~~~~~~~~~~~~
void setup() {  
  
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
  configBmp180();
  configIMU();
  configSerie();
  configRadio();
  configureSensorTSL2561();  


  
}
void loop() {  
 AdqBmp180(); 
 LEDs();
  // SI SE RECIBE UN COMANDO... 
  if ( radio.available()) {
     digitalWrite(LEDTX,HIGH); //ENCIENDE LED CADA VEZ QUE SE ENVIA UN DATO O SE RECIBE UN COMANDO.

    int data=0;
    data = leerDato(); // 
    detenerEscucha();
              
       if(data == ComandoAdq){  
          AdqImu();        
          AdqBmp180();
          AdqTSL2561();
          escribirDato(StatusAdq);
        }else if (data == ComandoLink){                            
          escribirDato(StatusCom);                    
        }else if (data == ComandoWrite){                  
          escribirDato(StatusWrite);                  
        }else if (data == ComandoIMU){
          AdqImu();          
          escribirVectorIMU();
        }else if (data == ComandoBmp180){
          AdqBmp180();      
          escribirVectorBpm180();
        }else if (data == ComandoTSL2561){
          AdqTSL2561();         
          escribirTSL2561();
          //Serial.println("IluminaciÓn: " + String(TSL2561) + " Lux");
        }else{
          Serial.println("Comando erroneo! ");
        } 
                      
    comenzarEscucha();
  }
 else {
    digitalWrite(LEDTX, LOW);}
    delay(1000);

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
    sensors_status[0]=1;
    tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  }

}
void AdqTSL2561(){
  if(sensors_status[0]==1){
  
  sensors_event_t event; /*Obtener un nuevo evento de sensor */ 
  tsl.getEvent(&event);
  TSL2561 = event.light;/*Muestra el resultado de la luz meduda en lux*/
  }

}
void configBmp180(){
    if (bmp180.begin()){
      Serial.println(">BMP180 iniciado");
      sensors_status[1]==1;
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
  sensors_status[2]==1;
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
