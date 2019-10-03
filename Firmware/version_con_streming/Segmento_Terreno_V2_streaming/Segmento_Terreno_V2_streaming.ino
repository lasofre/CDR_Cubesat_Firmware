/*Proyecto: CorEsat
 *Segmento: Estacion Terrena.
 *Creado por el grupo CorEsat del club de robotica de la 
 *Universidad tecnologica nacional Facultad regional Cordoba, Argentina.
 *Web: https://clubderobotica.github.io/
 */
#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>


//Declaración de los pines de comunicaciones de los modulos
#define CE_PIN 7                                     //Definicion del pin CE para el RF24
#define CSN_PIN 8                                    //Definicion del pin CSN para el RF24

//Comandos
#define ComandoLink    'c'              //probar si esta disponible el sate.
#define ComandoAdq     'a'              //Realizar adquisicion.
#define ComandoWrite   'w'              //Escribir dato.
#define ComandoIMU     'i'              //pedir vector IMU.
#define ComandoBmp180  'p'              //pedir Vector bmp180.
#define ComandoTSL2561 't'              //pedir Valor  TSL2561.
#define ComandoNull    '0'              //Borra el comando. 
#define StatusCom      'l'              //Pregunta estado de comunicaccion.
#define StatusWrite    'r'              //Verifica estado de escritura.
#define StatusAdq      'f'              //Verifica estado de adquisicion.
#define ComandoMenu    'm'              //Muestra nuevamente el menú.
#define StatusSen      'j'              //Pide el estado de los sensores vector (0:TSL2561|1:Bmp180|2:MPU-9250) 1 configurado,0 desconfigurado
//~~~~~~~~~~~~~~~~~~~Inicializacion de modulos~~~~~~~~~~~~~~~~~~~~~
RF24 radio(CE_PIN, CSN_PIN);            // Configuracion de Hardware:Radio nRF24L01 en el bus SPI (pines 10, 11, 12, 13) mas pines 7 & 8

//~~~~~~~~~~~~~~~~~~~Variables Globales~~~~~~~~~~~~~~~~~~~~~
char Comando='0'; // variable para toma de orden por teclado
int  esperaRespuesta; 
byte addresses[][6] = {"1Node", "2Node"};
int data = 0;//para pedir datos de 1 a 20
byte i;
double IMU[9];
double VectorBmp180[3];//0-T 1-P 2-A
double TSL2561;
char sensors_status[3]={'\0'};  
//~~~~~~~~~~~~~~~~~~~Prototipado de Funciones~~~~~~~~~~~~~~~~~~~~~
void menuSerie();                           //Muestra el menu de opciones.
void configRadio();                         //Configura el modulo de radio NRF.
void empezarEscucha();                      //Empieza la escucha RF.(Para recepcion de datos seguir las 3 funciones en orden).(1)
void esperaRx(int Demora);                  //Recibe los datos que se estan mandando. (2)
void detenerEscucha();                      //Detiene la escucha de la comunicacion por RF. (3)
void escribirDatos(int comando);            //Envia los datos por RF.
int16_t leerDatosRf();                      //Decodifica los datos recividos por RF.
void leerVectorIMU();                       //Decodifica los datos recividos por RF correspondienes a la IMU.
void leerVectorBmp180();                    //Decodifica los datos recividos por RF correspondientes al Bmp180.
void leerValorTSL2561();                    //Decodifica los datos recividos por RF correspondientes al TSL2561.



//~~~~~~~~~~~~~~~~~~~Funciones Propias de Arduino~~~~~~~~~~~~~~~~~~~~~
void setup() {
  Serial.begin(9600); 
  menuSerie();
  configRadio();
  
}
void loop() {
    Comando = ComandoNull;    
    delay(100);     
    while(Comando == ComandoNull)Comando = (char)Serial.read();//tomo valor del teclado 
    Serial.println("<Estacion> "+Comando);                  
   data =0; 
   switch (Comando) {
      
      case ComandoAdq://orden de Adquirir datos
        
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(ComandoAdq);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(3000);//ESPERO RESPUESTA
          data = leerDatosRf();//tomo dato que llego
            if(data == StatusAdq){
              Serial.println("<CorEsat> Comando de adquisicion ejecutado correctamente.");
            }else{
              Serial.println("<CorEsat> Problemas al ejecutar comando - Intente nuevamente");
            }
        break;        
          
      case ComandoBmp180://Pedido vector Sensor BPM180
            
            detenerEscucha();//DETENGO LA ESCUCHA
            escribirDatos(ComandoBmp180);
            empezarEscucha();  
            esperaRx(2000);//ESPERO RESPUESTA
            leerVectorBmp180();            
            Serial.println("<CorEsat> \n Temperatura: " + String(VectorBmp180[0]) + "[°C], Presion:" + String(VectorBmp180[1]) + "[mbar], Altitud " + String(VectorBmp180[2]) + " [m]");                                                                                        
          break;
          
        case ComandoIMU://Pedido vector Sensor IMU
            
            detenerEscucha();//DETENGO LA ESCUCHA
            escribirDatos(ComandoIMU);//ENVIO pedido de vector IMU
            empezarEscucha();  
            esperaRx(2000);//ESPERO RESPUESTA
            leerVectorIMU();
            Serial.println("<CorEsat>");
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
            Serial.println("<CorEsat> \n Ilumninacion: " + String(TSL2561) + " Lux");            
          break;
        case ComandoLink://verificacion de conexion con sate
          
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(ComandoLink);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(2000);//ESPERO RESPUESTA
          data = leerDatosRf();//tomo dato que llego
            if(data == StatusCom){
              Serial.println("<CorEsat> Estado de conexion correcta. ");
            }else{
              Serial.println("** Conexion inactiva- Intente nuevamente ");
            }
          
        break;
      case ComandoWrite://escritura de datos hacia el satelite
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(ComandoWrite);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(2000);//ESPERO RESPUESTA
          data = leerDatosRf();//tomo dato que llego          
            if(data == StatusWrite){
              Serial.println("<CorEsat> Estado de conexion correcta.");
            }else{
              Serial.println("** Problemas en la transmicion - Intente nuevamente ");
            }         
        break;
      case StatusSen:
          detenerEscucha();//DETENGO LA ESCUCHA
          escribirDatos(StatusSen);//ENVIO SOLICITUD DE A 1 DATO
          empezarEscucha();  
          esperaRx(2000);//ESPERO RESPUESTA
          leersensorStatus();
         Serial.println("<CorEsat> Estado de Sensores:"+String(sensors_status));    
      case ComandoMenu:menuSerie();break;
      default:
       Serial.println("** Comando incorrecto.");
        break;
  }
  
}




//~~~~~~~~~~~~~~~~~~~Funciones prototipadas~~~~~~~~~~~~~~~~~~~~~
void menuSerie(){
  Serial.println("MOC de CORESat.");
  Serial.println("(m) menu.");
  Serial.println("(c) Link Conexion.");
  Serial.println("(a) Adquisicion datos.");
  Serial.println("(w) Escritura.");
  Serial.println("(i) Datos IMU.");
  Serial.println("(p) Datos TPA.");
  Serial.println("(t) Datos Lux.");   
}

void configRadio(){
   // Initiate the radio object
  radio.begin();//Inicializar objeto radio
  // Set the transmit power to lowest available to prevent power supply related issues
  radio.setPALevel(RF24_PA_MIN);// Configurar minima enercia de transmicion disponible.
  // Set the speed of the transmission to the quickest available
  radio.setDataRate(RF24_2MBPS);//Configurar velocidad de transmicion maxima disponible.
  // Use a channel unlikely to be used by Wifi, Microwave ovens etc
  radio.setChannel(124);//Usar canal distinto la usado por el wifi, microondas ,etc.
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);//Abrir escritura y lectura pipe en la radio , con direcciones distintas.
  radio.openReadingPipe(1, addresses[0]);
}
void empezarEscucha(){
  radio.startListening();
}

void detenerEscucha(){
  radio.stopListening(); 
}

void esperaRx(int Demora){
  unsigned long started_waiting_at = millis();
  while ( ! radio.available() ) {
    if (millis() - started_waiting_at > Demora ) {
      Serial.println("**No hay respuesta del receptor - error: timeout.");
      return;
    }
  }
}

void escribirDatos(int comando){
    int data = comando;  
    if (!radio.write( &data, sizeof(unsigned int) )) {
      Serial.println("**No se puedo transmitir informacion, Verifique la conexion del transmisor.");    
    }     
}
int16_t leerDatosRf(){
  int16_t  dataRx=0;
  radio.read( &dataRx, sizeof(unsigned char) ); 
  return dataRx;
}

void leerVectorIMU(){  
  radio.read( &IMU, sizeof(IMU) );   
}
void leerVectorBmp180(){  
  radio.read( &VectorBmp180, sizeof(VectorBmp180) );   
}
void leerValorTSL2561(){  
  radio.read( &TSL2561, sizeof(float) );   
}
void leersensorStatus(){  
  radio.read( &sensors_status, sizeof(sensors_status) );   
}
