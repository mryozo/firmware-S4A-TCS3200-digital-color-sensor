// NEW IN VERSION 1.6c (by Jorge Gomez):
// Fixed variable type in pin structure: pin.state should be int, not byte
// Optimized speed of execution while receiving data from computer in readSerialPort()

// NEW IN VERSION 1.6b (by Jorge Gomez):
// Added new structure arduinoPins to hold the pins information:
//  - This makes the code easier to read and modify (IMHO)
//  - Allows to change the type of pin more easily to meet non standard use of S4A
//  - Eliminates the need of having to deal with different kind of index access (ie: states[pin-4])
//  - By using an enum to hold all the possible output pin states the code is now more readable
// Changed all functions using old style pin access: configurePins(), resetPins(), readSerialPort(), updateActuator() and sendUpdateActuator()
// Fixed possible overflow every 70 minutes (2e32 us) in pulse() while using micros(). Changed for delayMicroseconds()
// Some minor coding style fixes

// NEW IN VERSION 1.6a  (by Jorge Gomez):
// Fixed compatibility with Arduino Leonardo by avoiding the use of timers
// readSerialPort() optimized:
//  - created state machine for reading the two bytes of the S4A message
//  - updateActuator() is only called if the state is changed 
// Memory use optimization
// Cleaning some parts of code 
// Avoid using some global variables

// NEW IN VERSION 1.6:
// Refactored reset pins
// Merged code for standard and CR servos
// Merged patch for Leonardo from Peter Mueller (many thanks for this!)

// NEW IN VERSION 1.5:
// Changed pin 8 from standard servo to normal digital output

// NEW IN VERSION 1.4:
// Changed Serial.print() for Serial.write() in ScratchBoardSensorReport function to make it compatible with latest Arduino IDE (1.0)

// NEW IN VERSION 1.3:
// Now it works on GNU/Linux. Also tested with MacOS and Windows 7.
// timer2 set to 20ms, fixing a glitch that made this period unstable in previous versions.
// readSerialport() function optimized.
// pulse() modified so that it receives pulse width as a parameter instead using a global variable.
// updateServoMotors changes its name as a global variable had the same name.
// Some minor fixes.

/***********************************************
    DEFINICIÓN DE CONSTANTES
************************************************/
#define PIN_S0 10  //Selector 0 de ajuste de escalado o sensibilidad de señal de salida  LOW-LOW APAGADO, LOW-HIGH 2%
#define PIN_S1 11  //Selector 1 de ajuste de escalado o sensibilidad de señal de salida  HIGH-LOW  20%,  HIGH-HIGH 100%
#define PIN_S2 12  //Selector 2 de selección de filtro de color LOW-LOW Rojo, LOW-HIGH Azul
#define PIN_S3 13  //Selector 3 de selección de filtro de color HIGH-LOW Sin filtro, HIGH-HIGH Verde
#define PIN_LECTURA 6 //Pin en el que conectamos el cable de entrada de señal digital del sensor CTS3200

typedef enum { 
  input, servomotor, pwm, digital } 
pinType;

typedef struct pin {
  pinType type;       //Type of pin
  int state;         //State of an output
  //byte value;       //Value of an input. Not used by now. TODO
};

pin arduinoPins[14];  //Array of struct holding 0-13 pins information
int valorRojo=0;
int valorVerde=0;
int valorAzul=0;

unsigned long lastDataReceivedTime = millis();



void setup(){
  Serial.begin(38400);
  Serial.flush();
  configurePins();
  resetPins();
  //Seleccionamos el ajuste de escalado de sensibiliad de señal del sensor ajustado al 20%
  digitalWrite(PIN_S0,HIGH);//Selector 0 de ajuste de escalado o sensibilidad de señal de salida  LOW-LOW APAGADO, LOW-HIGH 2%
  digitalWrite(PIN_S1,LOW);//Selector 1 de ajuste de escalado o sensibilidad de señal de salida  HIGH-LOW  20%,  HIGH-HIGH 100%
}
/************************************************************************************************************************
Función bucle principal de ejecución del programa
*************************************************************************************************************************/
void loop(){
  static unsigned long timerCheckUpdate = millis();//tiempo máximo en milisegundos que evita que se resetee Arduino. Si se supera, Arduino se resetea sucesivamente.
  unsigned int lecturasColor[5];
  if (millis()-timerCheckUpdate>=20){
    pinMode(3,OUTPUT); 		// ponemos el pin 3 como salida digital    
    pinMode(5,OUTPUT); 		// ponemos el pin 5 como salida digital    
    pinMode(8,OUTPUT); 		// ponemos el pin 6 como salida digital    
    //encendemos led RGB BLANCO
    digitalWrite(3,0);
    digitalWrite(5,0);
    digitalWrite(8,0);
    delay(20000);
    //sendUpdateServomotors();
    sendSensorValues();
    pinMode(PIN_LECTURA, INPUT); 		// ponemos el pin 6 como entrada digital    
    //--------------------------------------------------------------------------------------------------------/
    digitalWrite(PIN_S2,LOW); //Seleccionamos filtro color Rojo del sensor S2=LOW,S3=LOW
    digitalWrite(PIN_S3,LOW);
    for (int cont=0;cont< 5;cont++)
      lecturasColor[cont]=pulseIn(PIN_LECTURA,LOW);  // lee el tiempo que tardo el pulso en regresar echo     
    insertionSort(lecturasColor,5); //sort readings
    valorRojo=lecturasColor[2]; //select median reading          
    ScratchBoardSensorReport(3,valorRojo);   // Envia el valorRojo al editbox del ANALOG 3
    
    //encendemos led RGB ROJO
    digitalWrite(3,255);
    digitalWrite(5,255);
    digitalWrite(8,0);
    delay(20000);
    //--------------------------------------------------------------------------------------------------------/
    //valorRojo=pulseIn(PIN_LECTURA,LOW);  // lee el tiempo que tardo el pulso en regresar echo     OPCION pulseIn 1
    //valorRojo=pulseIn(PIN_LECTURA,digitalRead(PIN_LECTURA)==HIGH?LOW:HIGH);  // lee el tiempo que tardo e pulso en regresar echo OPCION 2
    digitalWrite(PIN_S2,HIGH); //Seleccionamos filtro color Verde del sensor S2=HIGH,S3=HIGH
    digitalWrite(PIN_S3,HIGH);
    for (int cont=0;cont< 5;cont++)
      lecturasColor[cont]=pulseIn(PIN_LECTURA,LOW);  // lee el tiempo que tardo el pulso en regresar echo    OPCION pulseIn 3 
    insertionSort(lecturasColor,5); //sort readings
    valorVerde=lecturasColor[2]; //select median reading      
    ScratchBoardSensorReport(4,valorVerde);  // Envia el valorVerde al editbox del ANALOG 4
    //encendemos led RGB VERDE
    digitalWrite(3,255);
    digitalWrite(5,0);
    digitalWrite(8,255);
    delay(20000);
    //--------------------------------------------------------------------------------------------------------/
    digitalWrite(PIN_S2,LOW); //Seleccionamos filtro color Verde del sensor S2=LOW,S3=HIGH
    digitalWrite(PIN_S3,HIGH);
    for (int cont=0;cont< 5;cont++)
      lecturasColor[cont]=pulseIn(PIN_LECTURA,LOW);  // lee el tiempo que tardo el pulso en regresar echo     
    insertionSort(lecturasColor,5); //sort readings
    valorAzul= lecturasColor[2]; //select median reading        
    ScratchBoardSensorReport(5,valorAzul);   // Envia el valorAzul al editbox del ANALOG 5
    //--------------------------------------------------------------------------------------------------------/
    //encendemos led RGB AZUL
    digitalWrite(3,0);
    digitalWrite(5,255);
    digitalWrite(8,255);
    delay(20000);
    timerCheckUpdate=millis();
  }
  readSerialPort();
}
/************************************************************************************************************************
Función que configura el tipo de entrada/salida de los pines con respecto a la especificación de S4A (Mapeo de pines)
*************************************************************************************************************************/
void configurePins(){
  arduinoPins[0].type=input;
  arduinoPins[1].type=input;
  arduinoPins[2].type=input;
  arduinoPins[3].type=input;
  arduinoPins[4].type=servomotor;
  arduinoPins[5].type=pwm;
  arduinoPins[6].type=pwm;
  arduinoPins[7].type=servomotor;
  arduinoPins[8].type=servomotor;
  arduinoPins[9].type=pwm;
  arduinoPins[10].type=digital;
  arduinoPins[11].type=digital;
  arduinoPins[12].type=digital;
  arduinoPins[13].type=digital;
}
/************************************************************************************************************************
Función que resetea el estado de los pines de Arduino con respecto a la especificación de S4A (Mapeo de pines)
*************************************************************************************************************************/
void resetPins() {
  for (byte index=0; index <=13; index++){
    if (arduinoPins[index].type!=input){
      pinMode(index, OUTPUT);
      if (arduinoPins[index].type==servomotor){
        arduinoPins[index].state = 255;
        servo (index, 255);
      }
      else{
        arduinoPins[index].state=0;
        digitalWrite(index,LOW);
      }
    }
  }
}
/************************************************************************************************************************
Función que envía los valores captados por los sensores al panel de sensores de S4A
*************************************************************************************************************************/
void sendSensorValues(){
  unsigned int sensorValues[6], readings[5];
  byte sensorIndex;
  //calculamos la mediana de 5 lecturas de sensor para evitar la variabilidad y picos de tensión en la señal  
  for (sensorIndex = 0; sensorIndex < 3; sensorIndex++){ //hemos cambiado MAX=6 (0 a 5)  a MAX=3 (0 a 2)
    for (byte p = 0; p < 5; p++)
      readings[p] = analogRead(sensorIndex);
    insertionSort(readings, 5); //sort readings
    sensorValues[sensorIndex] = readings[2]; //select median reading
  }
  //Enviamos los valores de los sensores analógicos 0,1,2. Los valores 3,4,5 serán para valores del sensor RGB obtenidos en lectura pin digital 6
  for (sensorIndex = 0; sensorIndex < 3; sensorIndex++){//hemos cambiado MAX=6 (0 a 5)  a MAX=3 (0 a 2) 
    ScratchBoardSensorReport(sensorIndex, sensorValues[sensorIndex]); 
  }
  //ScratchBoardSensorReport(3,valorRojo);Esta linea la incluimos en main loop
  //ScratchBoardSensorReport(4,valorVerde);Esta linea la incluimos en main loop
  //ScratchBoardSensorReport(5,valorAzul);Esta linea la incluimos en main loop
  ScratchBoardSensorReport(6, digitalRead(2)?1023:0);
  ScratchBoardSensorReport(7, digitalRead(3)?1023:0);
}
/************************************************************************************************************************
Función que ordena N valores insertados en un array que se recibe como puntero
*************************************************************************************************************************/
void insertionSort(unsigned int* array, unsigned int n){
  for (int i = 1; i < n; i++)
    for (int j = i; (j > 0) && ( array[j] < array[j-1] ); j--)
      swap( array, j, j-1 );
}
/************************************************************************************************************************
Función que intercambia 2 valores de las posiciones recibidas sobre un array cuya referencia se recibe como parámetro
*************************************************************************************************************************/
void swap(unsigned int* array, unsigned int a, unsigned int b){
  unsigned int temp = array[a];
  array[a] = array[b];
  array[b] = temp;
}
/************************************************************************************************************
Función que vuelca los valores de los sensores formateándolos para transmitirlos por el puerto serie 
con destino al marcador de sensores de S4A
************************************************************************************************************/
void ScratchBoardSensorReport(byte sensor, int value){ //PicoBoard protocol, 2 bytes per sensor
  Serial.write( B10000000 | ((sensor & B1111)<<3)| ((value>>7) & B111));
  Serial.write( value & B1111111);
}

/************************************************************************************************************
Función que resetea la conexión de Arduino si detecta desconexion 
(ocurre cuando se excede el tiempo máximo desde última recepción de datos del puerto serie
************************************************************************************************************/
void reset(){ //with xbee module, we need to simulate the setup execution that occurs when a usb connection is opened or closed without this module
  resetPins();        // reset pins
  sendSensorValues(); // protocol handshaking
  lastDataReceivedTime = millis();
}

void sendUpdateServomotors(){
  for (byte p = 0; p < 10; p++)
    if (arduinoPins[p].type == servomotor) servo(p, arduinoPins[p].state);
}
void pulse (byte pinNumber, unsigned int pulseWidth){
  digitalWrite(pinNumber, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pinNumber, LOW);
}
void updateActuator(byte pinNumber){
  if (arduinoPins[pinNumber].type==digital) digitalWrite(pinNumber, arduinoPins[pinNumber].state);
  else if (arduinoPins[pinNumber].type==pwm) analogWrite(pinNumber, arduinoPins[pinNumber].state);
}

void servo (byte pinNumber, byte angle){
  if (angle != 255)
    pulse(pinNumber, (angle * 10) + 600);
}
//the reset is necessary when using an wireless arduino board (because we need to ensure that arduino isn't waiting the actuators state from Scratch) or when scratch isn't sending information (because is how serial port close is detected)
void checkScratchDisconnection(){
  if (millis() - lastDataReceivedTime > 1000) reset(); //reset state if actuators reception timeout = one second
}
/************************************************************************************************************
Función que gestiona la transferencia de datos bidireccional ARDUINO/S4A a través del puerto serie
************************************************************************************************************/
void readSerialPort(){
  byte pin;
  int newVal;
  static byte actuatorHighByte, actuatorLowByte;
  static byte readingSM = 0;

  if (Serial.available()){
    if (readingSM == 0){
      actuatorHighByte = Serial.read();
      if (actuatorHighByte >= 128) readingSM = 1;
    }
    else if (readingSM == 1){
      actuatorLowByte = Serial.read();
      if (actuatorLowByte < 128) readingSM = 2;
      else readingSM = 0;
    }
    if (readingSM == 2){
      lastDataReceivedTime = millis();    
      pin = ((actuatorHighByte >> 3) & 0x0F);
      newVal = ((actuatorHighByte & 0x07) << 7) | (actuatorLowByte & 0x7F); 

      if(arduinoPins[pin].state != newVal){
        arduinoPins[pin].state = newVal;
        updateActuator(pin);
      }
      readingSM = 0;
    }
  }
  else checkScratchDisconnection();
}







