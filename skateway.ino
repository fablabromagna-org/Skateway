// Include la libreria del PID
#include "PID_v1.h"

// E' Richiesta la libreria Wire se  I2CDEV_ARDUINO_WIRE è usato nel I2Cdev.h
#include "Wire.h"

// I2Cdev(libreria I2C bus) e MPU6050 devono essere installate come librerie
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//inizializza la variabile per il sensore
MPU6050 mpu;

/* ====================================================================
NOTE: Questo sketch usa il piedino dell'interrupt che è collegato al piedino INT del sensore
Sull' ArduinoUno il piedino dell'interrupt è il piedino digitale 2.
* ====================================================================*/

// Scommenta "OUTPUT_READABLE_YAWPITCHROLL" se vuoi vedere yaw/
// pitch/roll angles (in degrees) calcolati con i  quaternioni presi dal FIFO. 
#define OUTPUT_READABLE_YAWPITCHROLL

//Variabili di controllo del sensore
bool dmpReady = false; // impostato vero se il test del DMP ha avuto successo
uint8_t mpuIntStatus; // contiene il valore dell'interupt del nostro sensore
uint8_t devStatus; // ritorna lo stato dopo ogni operazione del sensore (0 = successo, diverso da 0 = errore)
uint16_t packetSize; // dimensione del pacchetto dati del DMP  (di default è 42 bytes)
uint16_t fifoCount; // conta tutti i byte presenti nel FIFO
uint8_t fifoBuffer[64]; // buffer di appoggio del FIFO

// variabili di orientazione/movimento
Quaternion q; // [w, x, y, z] quaternioni
VectorFloat gravity; // [x, y, z] vettore delle gravita
float ypr[3]; // [yaw, pitch, roll]vettore dei yaw/pitch/roll 


// variabili del PID
double Setpoint, Input, Output;
unsigned long serialTime_FrontEnd; //questo ci aiuta per capire il tempo del processo
double Kp=28, Ki=150, Kd=1.2;

//Inizializza il PID
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);
//Variabili della scheda di potenza
int enable = 2;

int RPWM = 5;
int LPWM = 6;

// ================================================================
// === ROUTINE DI RILEVAMENTO INTERRUPT ===
// ================================================================

volatile bool mpuInterrupt = false; // indica se il piedino di interrupt è andato alto(3,3V)
void dmpDataReady() {
mpuInterrupt = true;
}

// ================================================================
// === INITIAL SETUP ===
// ================================================================

void setup() {
        //attiva la scheda di potenza
        digitalWrite(enable,HIGH);
        //inizializza i parametri del PID
        Setpoint = 1.35;//angolo di equilibrio  
        myPID.SetMode(AUTOMATIC);//il PID lavora in automatico
        myPID.SetOutputLimits(-255.0,255.0);//range del PID
        myPID.SetSampleTime(20);//tempo di campionamento(20 millisecondi)
  
// entra nel I2C Bus (la libreria I2Cdev non lo fa automaticamente)
Wire.begin();

// inizializza la comunicazione seriale
Serial.begin(115200);

// inizializza il sensore
Serial.println(F("Initializing I2C devices..."));
mpu.initialize();

// verifica la connessione
Serial.println(F("Testing device connections..."));
Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

// aspetta fino a che non è pronto
while (Serial.available() && Serial.read()); // buffer vuoto


// carica e configura il DMP
Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize();

// controlla se ha lavorato correttamente(0 se è vero)
if (devStatus == 0) {
// accende il DMP, ora è pronto!
Serial.println(F("Enabling DMP..."));
mpu.setDMPEnabled(true);

// abilita la routine di rilevamento di interrupt
Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();

// imposta il flag del DMP come pronto così il loop() sa che può essere usato
Serial.println(F("DMP ready! Waiting for first interrupt..."));
dmpReady = true;

// acquisisce la dimensione del pacchetto aspettata dal FIFO
packetSize = mpu.dmpGetFIFOPacketSize();
}
else {
// Errore!
// 1 = errore iniziale di caricamento memoria
// 2 = errore di configurazione DMP
Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
           
}

// ================================================================
// === LOOP PRINCIPALE ===
// ================================================================

void loop() {
// se il DMP è programmato male , non fa niente
if (!dmpReady) return;
        
//  aspetta l'interrupt del MPU o qualche pacchetto extra
while (!mpuInterrupt && fifoCount < packetSize) {

                Input = ypr[1] * 180/M_PI;//calcola l'angolo in gradi
                
                if(abs(Input) > 22)
                {                 
                  Output = 0;
                }
                else
                {
                  //esegue il PID
                  myPID.Compute();
                }
                                  
                 + if(abs(Output) == 0 )
                {
                  //qua lo skateboard sta fermo
                 digitalWrite(LPWM,HIGH);
                  digitalWrite(RPWM,HIGH);

                }
                if(Output > 0)
                {
             
     //qua lo skateboard va avanti
                   analogWrite(RPWM,int(map(Output,0,255,255,0)));
                   digitalWrite(LPWM,HIGH);
                 }
                 if(Output < -0)
                 {
                  // qua lo skateboard va indietro
                  analogWrite(LPWM,int(-1*map(Output,-0,-255,-255,-0)));
                   digitalWrite(RPWM,HIGH);
                 } 
    
}

//resetta il flag degli interrupt e ottiene INT_STATUS bytempuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();

// ottiene il numero di elementi nel FIFO
fifoCount = mpu.getFIFOCount();

// coontrolla per overflow 
if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//resetta cosi possiamo continuare 
mpu.resetFIFO();
Serial.println(F("FIFO overflow!"));

// viceversa, controlla per l'interrupt di pronto del DMP 
}
else if (mpuIntStatus & 0x02) {
// aspetta per la lunghezza correta del dato
while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

// leggi un pacchetto dal FIFO
mpu.getFIFOBytes(fifoBuffer, packetSize);

// traccia il numero di elementi nel FIFO quando c'è piu di un pacchetto disponibile
// (questo ci permettere di leggere immediatamente senza aspettare un altro interrupt)
fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
// mostra angoli di Eulero in gradi
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
// Serial.print("ypr\t");
// Serial.print(ypr[0] * 180/M_PI);
// Serial.print("\t");
// Serial.print(ypr[1] * 180/M_PI);
// Serial.print("\t");
// Serial.println(ypr[2] * 180/M_PI);
#endif
}
