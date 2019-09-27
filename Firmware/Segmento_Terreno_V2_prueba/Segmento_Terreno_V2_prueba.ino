//Master este seria la PC, EL MOC en la Estacion Terrena del sate
#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

// This is just the way the RF24 library works:
// Hardware configuration: Set up nRF24L01 radio on SPI bus (pins 10, 11, 12, 13) plus pins 7 & 8
//RF24 radio(7, 8);
//RADIO ***************************************** 
//Declaremos los pines CE y el CSN
#define CE_PIN 7
#define CSN_PIN 8
// This is just the way the RF24 library works:
// Hardware configuration: Set up nRF24L01 radio on SPI bus (pins 10, 11, 12, 13) plus pins 7 & 8
RF24 radio(CE_PIN, CSN_PIN);

#define ComandoLink    'C'//probar si esta disponible el sate
#define ComandoAdq     'A'//Realizar adquisicion
#define ComandoWrite   'W'//Escribir dato
#define ComandoIMU     'I'//pedir vector IMU
#define ComandoBmp180  'P'//pedir Vector bmp180
#define ComandoTSL2561 'T'//pedir Valor  TSL2561
#define ComandoNull    '0'
#define StatusCom     'L'
#define StatusWrite   'R'
#define StatusAdq     'F'

char Comando='0'; // variable para toma de orden por teclado
int  esperaRespuesta; 
byte addresses[][6] = {"1Node", "2Node"};
int data = 0;//para pedir datos de 1 a 20
byte i;
double IMU[9];
double VectorBmp180[3];//0-T 1-P 2-A
double TSL2561;
// -----------------------------------------------------------------------------
// SETUP   SETUP   SETUP   SETUP   SETUP   SETUP   SETUP   SETUP   SETUP
// -----------------------------------------------------------------------------
void setup() {
  
  configSerie();
  configRadio();
  
}

void configSerie(){
  Serial.begin(9600); 
  Serial.println("MOC de CORESat: Ingrese Comando: ");
  Serial.println("Link Conexion -C- * Adquisicion datos -A- * Escritura -W- * Datos IMU -I- * Datos TPA -P- * Datos Lux -T- *"); 
}

void configRadio(){
   // Initiate the radio object
  radio.begin();
  // Set the transmit power to lowest available to prevent power supply related issues
  radio.setPALevel(RF24_PA_MIN);
  // Set the speed of the transmission to the quickest available
  radio.setDataRate(RF24_2MBPS);
  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  radio.setChannel(124);
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
}
void empezarEscucha(){
    // Now listen for a response
  radio.startListening();
}

void detenerEscucha(){
  // Ensure we have stopped listening (even if we're not) or we won't be able to transmit
  radio.stopListening(); 
}

void esperaRx(int Demora){
    // But we won't listen for long, 200 milliseconds is enough
  unsigned long started_waiting_at = millis();

  // Loop here until we get indication that some data is ready for us to read (or we time out)
  while ( ! radio.available() ) {

    // Oh dear, no response received within our timescale
    if (millis() - started_waiting_at > Demora ) {
      Serial.println("No response received - timeout!");
      return;
    }
  }
}

void escribirDatos(int comando){
  // Did we manage to SUCCESSFULLY transmit that (by getting an acknowledgement back from the other Arduino)?
  // Even we didn't we'll continue with the sketch, you never know, the radio fairies may help us   
    int data = comando;  
    if (!radio.write( &data, sizeof(unsigned int) )) {
      Serial.println("No acknowledgement of transmission - receiving radio device connected?");    
    }     
}
int16_t leerDatosRf(){
  // Now read the data that is waiting for us in the nRF24L01's buffer  
  int16_t  dataRx=0;
  radio.read( &dataRx, sizeof(unsigned char) ); 
  return dataRx;
}

void leerVectorIMU(){
  // Now read the data that is waiting for us in the nRF24L01's buffer   
  radio.read( &IMU, sizeof(IMU) );   
}
void leerVectorBmp180(){
  // Now read the data that is waiting for us in the nRF24L01's buffer   
  radio.read( &VectorBmp180, sizeof(VectorBmp180) );   
}
void leerValorTSL2561(){
  // Now read the data that is waiting for us in the nRF24L01's buffer   
  radio.read( &TSL2561, sizeof(float) );   
}
 // -----------------------------------------------------------------------------
// LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP     LOOP
// -----------------------------------------------------------------------------
void loop() {
 
   while(true){
   
    Comando = ComandoNull;    
    delay(100);     
    Comando = (char)Serial.read();//tomo valor del teclado 
    
    if(Comando == ComandoAdq || Comando == ComandoLink || Comando == ComandoWrite || Comando == ComandoIMU || Comando == ComandoBmp180 || Comando == ComandoTSL2561){
        break;  
       }     
   }                             
   
   data =0; 
   
   switch (Comando) {
      
      case ComandoAdq://orden de Adquirir datos
        
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(ComandoAdq);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(3000);//ESPERO RESPUESTA
          data = leerDatosRf();//tomo dato que llego
            if(data == StatusAdq){
              Serial.println("CorEsat ADQ OK!!! -");
            }else{
              Serial.println("CorEsat ADQ FAIL!!! - Intente nuevamente");
            }
        break;        
          
      case ComandoBmp180://Pedido vector Sensor BPM180
            
            detenerEscucha();//DETENGO LA ESCUCHA
            escribirDatos(ComandoBmp180);
            empezarEscucha();  
            esperaRx(2000);//ESPERO RESPUESTA
            leerVectorBmp180();            
            Serial.println("Temperatura: " + String(VectorBmp180[0]) + "[°C], Presion:" + String(VectorBmp180[1]) + "[mbar], Altitud " + String(VectorBmp180[2]) + " [m]");                                                                                        
          break;
          
        case ComandoIMU://Pedido vector Sensor IMU
            
            detenerEscucha();//DETENGO LA ESCUCHA
            escribirDatos(ComandoIMU);//ENVIO pedido de vector IMU
            empezarEscucha();  
            esperaRx(2000);//ESPERO RESPUESTA
            leerVectorIMU();
            
            Serial.println("Accel: " + String(IMU[0]) + ", " + String(IMU[1]) + ", " + String(IMU[2]) + " g");
            Serial.println("Gyro: " + String(IMU[3]) + ", " + String(IMU[4]) + ", " + String(IMU[5]) + " dps");
            Serial.println("Mag: " + String(IMU[6]) + ", " + String(IMU[7]) + ", " + String(IMU[8]) + " uT");                        
          break;
        case ComandoTSL2561://Pedido valor Sensor Lux
            
            detenerEscucha();//DETENGO LA ESCUCHA
            escribirDatos(ComandoTSL2561);//ENVIO pedido de vector IMU
            empezarEscucha();  
            esperaRx(2000);//ESPERO RESPUESTA
            leerValorTSL2561();            
            Serial.println("Ilumninacion: " + String(TSL2561) + " Lux");
            
          break;
        
        case ComandoLink://verificacion de conexion con sate
          
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(ComandoLink);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(2000);//ESPERO RESPUESTA
          data = leerDatosRf();//tomo dato que llego
            if(data == StatusCom){
              Serial.println("CorEsat ACTIVO LINK OK!!!- ");
            }else{
              Serial.println("CorEsat INACTIVO LINK FAIL!!!- Intente nuevamente ");
            }
          
        break;
      case ComandoWrite://escritura de datos hacia el satelite
          
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(ComandoWrite);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(2000);//ESPERO RESPUESTA
          data = leerDatosRf();//tomo dato que llego          
            if(data == StatusWrite){
              Serial.println("CorEsat WRITE OK!!! ");
            }else{
              Serial.println("CorEsat WRITE FAIL!!! - Intente nuevamente ");
            }         
        break;      
      default:
       Serial.println("Comando incorrecto - MOC de CORESat");
        break;
  }
  
}
