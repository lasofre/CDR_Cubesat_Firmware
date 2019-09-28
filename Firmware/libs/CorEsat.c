#include "CorEsat.h"
#include <SFE_BMP180.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Wire.h>//para I2C
#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>
#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

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
