
#include <Wire.h>
#include "heartRate.h"
#include "MAX30105.h"
#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Activar debugger por comnunicación serial

// Definición de pines
int myLed  = 13;  // Definición de led para identificar validación de conexión

MPU9250 myIMU;
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Incrementar para mas presición. 4 es bueno.
byte rates[RATE_SIZE]; //Array de mediciones de ritmo cardiaco
byte rateSpot = 0;
long lastBeat = 0; //Tiempo en que ocurre el latido previo

float beatsPerMinute;
int beatAvg;
int selec=12;
int prom=11;
int parar=10;

void setup()
{
  Wire.begin();
  Serial1.begin(19200);
  //Definición de pines que indican el inicio del la cominicación
  pinMode(selec, INPUT);
  pinMode(prom, INPUT);
  pinMode(parar, INPUT);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Inicializar sensor de ritmo cardiaco
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Utilizar puerto I2C por defecto, a 400kHz
  {
    Serial1.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial1.println("Coloca tu dedo con una presión constante en el sensor");

  particleSensor.setup(); //Configurar sensor con datos por defecto
  particleSensor.setPulseAmplitudeRed(0x0A); //Amplitud del Led rojo de MAX30102
  particleSensor.setPulseAmplitudeGreen(0); //No se utiliza, esta versión del sensor no tiene led verde

  // Prueba de comunciación del acelerómetro y giroscopio
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial1.println("MPU9250 Conectado...");

    // valores con los que inicia el acelerometro y giroscopio
    myIMU.MPU9250SelfTest(myIMU.SelfTest);

    //La siguiente linea solo debe dejar de comentarse para calibrar el acelerómetro y giroscopio
    //No hacerlo una vez que se defina la posición final en la que se colocará el sensor
          //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    Serial1.println("MPU9250 inicializado....");
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial1.println("AK8963 initialized for active data mode....");
  } 
}
//Inicialización de variables que se utilizarán para la identificación de cada estilo
int L_b = 0;
int M_b = 0;
int D_b = 0;
int P_b = 0;
char E = 'N';
char E_2 = 'N';
char E_3 = 'N';
char E_4 = 'N';
int x1=0;
unsigned long tempA=0;
int med1=0;
int med2=0;
int med3=0;
int medDif1=0;
int medDif2=0;
int DT=0;
int s=0;
int buttonState1=0;
int buttonState2=0;
int cont=0;
long prom_f=0;
long prom1_f=0;
int uno=0;
void loop()
{
  //Si el pin de interrupción está activado los datos se colocarán como cero
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  //Lectura de valores RAW de aceleración en X, Y y Z
    myIMU.getAres();

    // Calculo de valores de aceleración en G's
    // Depende de la escala programada; se puede modificar en el codigo de la libreria MPU9250.h
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  //Lectura de valores RAW de velocidad angular en X, Y y Z
    myIMU.getGres();

    // Calculo de valores de velocidad angular en grados/segundo
    // Depende de la escala programada; se puede modificar en el codigo de la libreria MPU9250
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Lectura de datos RAW del magnetómetro
    myIMU.getMres();
    // Corrección del eje x en miliGauss
    myIMU.magbias[0] = +470.;
    // Corrección del eje x en milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // Corrección del eje x en miliGauss
    myIMU.magbias[2] = +125.;

    // Calculo de valores de magnetómetro en milliGauss
    //Obtención de valores del magnetómetro, dependiendo de la escala en que se fijó, se puede modificar en le codigo de la libreria MPU9250
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Debe llamarse antes de utilizar el filtro de Cuaterniones
  myIMU.updateTime();
  
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    
  } 

// Conversión de valores x,y,z del giroscópio a yaw, pitch y roll
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      if(SerialDebug)
      { 
        
      }

// Fase 1, Empieza Estilo, Posición inicia
    // Detección de inicio de brazada en estilo frontal
     if((myIMU.roll<=15 and myIMU.roll>=-15 ) and (myIMU.pitch<=50 and myIMU.pitch>=20)and(E=!'F','M','L','D','P')){

      E='F';
      
      }
      
     // Detección de inició de brazada de estilo dorsal
      if((myIMU.roll<=65 and myIMU.roll>=25 ) and (myIMU.pitch<=60 and myIMU.pitch>=35)and(E=!'F','M','L','D','P')){

      E='D';
      
      }

// Fase 2, Detección de estilo, Agarre de brazada
        
      // Libre y Mariposa  
        if((myIMU.roll<=-120 and myIMU.roll>=-150 ) and (myIMU.pitch<=50 and myIMU.pitch>=20) and (E=='F' ) ){
          E_2='L';

          }
      //Pecho--Faltan parámetros
         if((myIMU.roll<=10 and myIMU.roll>=-20 ) and (myIMU.pitch<=35 and myIMU.pitch>=10) and (E=='F' ) ){
          E_2='P';

          }
        //Dorso--Faltan parámetros
         if((myIMU.roll<=10 and myIMU.roll>=-20 ) and (myIMU.pitch<=35 and myIMU.pitch>=10) and (E=='D' ) ){
          E_2='D';

          }  

// Fase 3, Seguimiento de Estilo, Tirón de brazada
    //Libre y Mariposa
        if((myIMU.roll<=179 and myIMU.roll>=150 ) and (myIMU.pitch<=30 and myIMU.pitch>=5) and (E_2=='L' ) ){
          E_3='L';

          }

       // Pecho--Faltan parámetros
          if((myIMU.roll<=-50 and myIMU.roll>=-90 ) and (myIMU.pitch<=30 and myIMU.pitch>=0) and (E_2=='P' ) ){
          E_3='P';

          }
          // Dorso--Faltan parámetros
          if((myIMU.roll<=-50 and myIMU.roll>=-90 ) and (myIMU.pitch<=30 and myIMU.pitch>=5) and (E_2=='D' ) ){
          E_3='D';

          }

// Fase 4, Seguimiento de estilo, Recuperación
      //Libre
          if((myIMU.roll<=-160 and myIMU.roll>=170 ) and (myIMU.pitch<=95 and myIMU.pitch>=65) and (E_3=='L' ) ){
          E_4='L';

          }

        //Mariposa
        if((myIMU.roll<=180 and myIMU.roll>=70 ) and (myIMU.pitch<=60 and myIMU.pitch>=40) and (E_3=='L' ) ){
          E_4='M';

          }
        //Pecho--Faltan parámetros
          if((myIMU.roll<=-50 and myIMU.roll>=-90 ) and (myIMU.pitch<=30 and myIMU.pitch>=5) and (E_2=='P' ) ){
          E_3='P';

          }
        //Dorso--Faltan parámetros
          if((myIMU.roll<=-50 and myIMU.roll>=-90 ) and (myIMU.pitch<=30 and myIMU.pitch>=5) and (E_2=='D' ) ){
          E_3='D';

          }

// Conteo de brazadas según estilo
        switch (E_4){
          case 'L':
          L_b = L_b + 2;
          E = 'N';
          E_2 = 'N';
          E_3 = 'N';
          E_4 = 'N';
          Serial.print("Libre ");Serial.println(L_b);
          break;
          case 'M':
          M_b = M_b + 1;
          E = 'N';
          E_2 = 'N';
          E_3 = 'N';
          E_4 = 'N';
          Serial.print("Mariposa ");Serial.println(M_b);
          break;
          case 'P':
          P_b = P_b + 1;
          E = 'N';
          E_2 = 'N';
          E_3 = 'N';
          E_4 = 'N';
          Serial.print("Pecho ");Serial.println(P_b);
          break;
          case 'D':
          D_b = D_b + 2;
          E = 'N';
          E_2 = 'N';
          E_3 = 'N';
          E_4 = 'N';
          Serial.print("Dorso ");Serial.println(D_b);
          break;
          }


  // prueba con heart rate
  {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {

    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 30 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE; 

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

buttonState2 = digitalRead(prom); 
    if(buttonState2==HIGH){
  Serial1.print(", Avg BPM=");
  Serial1.println(beatAvg);
    }

buttonState1 = digitalRead(selec);
  if (buttonState1==HIGH){
    x1=1;
    prom1_f=0;
  Serial1.print(" Inicia prueba de Umbral anaerobico ");
    }
if (x1==1){
    if (med1>10){
      if ((millis()-tempA>20000)& s==0){//indicar a los 40 segundos que se haga un cambio de ritmo
        med2=beatAvg;
        s=1;
         if (cont==0 && uno==0){
          Serial1.print("Velocidad promedio= "); Serial1.println("3.34 km/h");
          }
        if (cont==1 && uno==0){
          Serial1.print("Velocidad promedio= "); Serial1.println("4.87 km/h");
          }
          if (cont==0 && uno==1){
          Serial1.print("Velocidad promedio= "); Serial1.println("6.51 km/h");
          }
        if (cont==1 && uno==1){
          Serial1.print("Velocidad promedio= "); Serial1.println("8.05 km/h");
          }
        Serial1.println(" Medicion, Cambio de ritmo");
        
        Serial1.print(" , ");

      }
       
           if (millis()-tempA>40000){//indicar a los 120 segundos se haga un cambio de ritmo
             Serial1.print("M1= ");
             Serial1.print(med1);
             Serial1.print("  M2= ");
             Serial1.println(med2);
             
             
           // med3=beatAvg;
            cont=cont+1;
            if (cont==1){
              medDif1=med2-med1;
              Serial1.print(" Medicion, Cambio de ritmo ");
                if (uno==0){
                Serial1.print(" Velocidad promedio= "); Serial1.println(" 4.11 km/h");
                }
                if (uno==1){
                Serial1.print(" Velocidad promedio= "); Serial1.println("7.15 km/h");
                }
            }
            if (cont==2){
              medDif2=med2-med1;
              DT=medDif2-medDif1;
              cont=0;
              if (uno==0){
                Serial1.print(" Velocidad promedio= "); Serial1.println(" 5.52 km/h");
              }
              if (uno==1){
                Serial1.print(" Velocidad promedio= "); Serial1.println("8.89 km/h");
                }
              Serial1.println(DT);
              if ((DT>-7)){
                Serial1.println(" Aun en aerobico");
                Serial1.println(" Medicion, Cambio de ritmo");
                uno=1;
                
                }
              else{
                if (DT<-7){
                  Serial1.println(" En el umbral ");
                  x1=0;
                  prom1_f=0;
                  uno=0;
                  
                  }
                }
            }
            

               tempA=millis();
               s=0;
               med1=0;
               med2=0;
              // med3=0;
              // x1=0;
              }
        
       
       
    }
    else{
      med1=beatAvg;
      tempA=millis(); //unsigned lon

      }
      if (beatAvg>med1 && s==0){
        med1=beatAvg;
        }
        
      if (beatAvg>med2 && s==1){
        med2=beatAvg;
        }

        //potencia es fuerza x velocidad
        //el acelerómetro mide en miliGs de fuerza
        //9.8 Newtons = 1G
        //mediciones chicha
        //velocidad de caminadora en 2, fuerza en Z al momento del paso, 11.90N, peso ejercido 770N
        //esto es en dirección de la gravedad.
        //4.7N en Y al momento del paso, peso eje

}

}
  
  //Para entrar al modo de calculo del umbral anaeróbico se debe de activar por la
  //interface del dispositivo, y tener planteado, una prueba de aumento gradual de
  //intensidad en el ejercicio.
 // 
 //Se utiliza el de punto de deflexión de
 //frecuencia cardiaca. El cual de igual forma requerirá un monitoreo cada 30 segundos y
 //la prueba para identificar el umbral anaeróbico deberá de durar mas de 2 minutos.
 //(Lo recomendable son mas de 5 minutos para tener una buena precisión, en conjunto con
 //un adecuado plan de aumento gradual de intensidad de entrenamiento).
 

}
