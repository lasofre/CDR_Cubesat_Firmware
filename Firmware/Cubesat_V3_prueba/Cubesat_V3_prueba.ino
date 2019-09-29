/*Proyecto: 
 * 
 */
#include <CorEsat.h>

/*Definimos los pines para los leds testigos*/
#define LEDTX 4      
#define LED_Temp 3
#define LED_EPS 2
#define LED_NN 5

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
