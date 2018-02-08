/*********************************************************************************************************\
  Rocket Controller
  IL programma rileva i dati provenienti dall'altimetro e dall'accellerometro a 3 assi e li salva sulla
  SD creando un file CSV per la successiva lettura dei dati su foglio di calcolo.
  Le periferiche sono BMP180 per altimetro e sensore di temperatura e MMA7361 accelerometro 3 assi
  La gestione di tre servo comandi;
  -uno per l'attivare del paracadute in autonomia, se dall'altimetro viene
  rilevata un altezza minore della massima misurata  meno un delta prestabilito attiva il paracadute.
  -due per la gestione degli stadi, per il momento resta solo una predisposizione ma sicuramente saranno
  gestiti a tempo
  Uno solo led identifica lo stato del programma, diverse frequenze di lampeggio danno lo stato del sistema
  con eventuale errore, lampeggio 1,5econdo stato OK
  \*********************************************************************************************************/



#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>


SFE_BMP180 pressure;
Servo Paracadute;
Servo Stadio1;
Servo Stadio2;

String DataString = "";
String Filename = "Shot.csv";

double baseline; // baseline pressure
double Alt, Press;
double AltHold;
double Blink;
double Sample = 0;
double TimeOff =0;

int sum = 0;
int offSets[3] = {0, 0, 0};
int Pattern = 0;
char i =     0;

boolean MemTimeOff=0;
boolean LogMod =0;
boolean ParSta = 0;
boolean LedVM = 0;


#define xPin        A0
#define yPin        A1
#define zPin        A2
#define LedV        5
#define Delta       15
#define HMin        10
#define average     10
#define chipSelect  10
#define SampleT     10
#define ParOff      20  //131
#define ParOn       180 //115
#define DefTime		10000

int mapMMA7361V(int value);
int mapMMA7361G(int value);
int getXRaw();
int getYRaw();
int getZRaw();
int getXVolt();
int getYVolt();
int getZVolt();
int getXAccel();
int getYAccel();
int getZAccel();
void setOffSets(int xOffSet, int yOffSet, int zOffSet);
void calibrate();
int getOrientation();
double getPress();


void setup() {
  //------- I-O ------------
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(zPin, INPUT);
  pinMode(LedV, OUTPUT);

  analogReference(EXTERNAL);

  Paracadute.attach(8);
  //Stadio1.attach(7);
  //Stadio2.attach(6);
  Paracadute.write(ParOn);
  delay(1000);
  Paracadute.write(ParOff);
  //-------Comunicazione--------

  Serial.begin(115200);
  Pattern = 1500;

  // -------SD Card--------------
  Serial.print("Inizializzazione SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Sd Danneggiata o non presente");
    digitalWrite(LedV, LedVM);
    Pattern = 100;
    return;
  }
  Serial.println("SD inizializzata");
  //Controllo nome fai e generazione di unnuovo file
  for (i = 0; SD.exists(Filename); i++)Filename = ("Shot" + String(i, DEC) + ".csv");
  Serial.println(Filename);
  File dataFile = SD.open(Filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time;AccX;AccY;AccZ;AltM");
    dataFile.close();
  }


  calibrate();             //Calibrazione Accelerometro


  if (pressure.begin())
    Serial.println("Alt OK");
  else
  {
    //Serial.println("Altimeter fail");
    Pattern = 450;
  }
  baseline = getPress();    //Prendo la pressione al suolo per calcolare l'altezza relativa
  LogMod=1;
}//End Setup


void loop() {

  if (millis() >= Blink) {		//Segnalazione stato programma
    LedVM = !LedVM;
    Blink = (millis() + Pattern);
  }
  digitalWrite(LedV, LedVM);

  Press = getPress();
  Alt = pressure.altitude(Press, baseline);

  if (Alt > AltHold) AltHold = Alt;
  if (((AltHold - Delta) > Alt) && (ParSta == 0)) ParSta = 1;
  if (ParSta)Paracadute.write(ParOn);
  else Paracadute.write(ParOff);
  
  if (ParSta & Alt<HMin) {
	  if (!MemTimeOff){
		  TimeOff= millis()+DefTime;
          MemTimeOff=1;
	  }
	  if(MemTimeOff& millis()>TimeOff) {
	    Pattern = 5000;
	    LogMod=0;
	  }
   
  }
			
  
  Stadio1.write(30);
  Stadio2.write(30);


  //if (Alt >= 0.0) Serial.print(" "); //Spazio per numeri positivi
  //Serial.print(Alt, 1);
  //Serial.println("m");
//
//    Serial.print("X");
//    Serial.println(getXAccel());
//    Serial.print("Y");
//    Serial.println(getYAccel());
//    Serial.print("Z");
//    Serial.println(getZAccel());


  if ((millis() >= Sample)& LogMod) {
    Sample = (millis() + SampleT);
    //DataString = (String(getXAccel(), DEC) + ";" + String(getYAccel(), DEC) + ";" + String(getZAccel(), DEC) + ";" + String(Alt));
   DataString = (String( millis()) + ";" + String(getXAccel(), DEC) + ";" + String(getYAccel(), DEC) + ";" + String(getZAccel(), DEC) + ";" + String(Alt));
    File dataFile = SD.open(Filename, FILE_WRITE);
    if (dataFile) {
      dataFile.println(DataString);
      dataFile.close();
      //delay(5000);
    }
  }
} //End Loop


void setOffSets(int xOffSet, int yOffSet, int zOffSet) {
  offSets[0] = map(xOffSet, 0, 3300, 0, 1024);
  offSets[1] = map(yOffSet, 0, 3300, 0, 1024);
  offSets[2] = map(zOffSet, 0, 3300, 0, 1024);
}




int getXRaw() {
  return analogRead(xPin) + offSets[0] ;//+ 2;
}


int getYRaw() {
  return analogRead(yPin) + offSets[1];// + 2;
}


int getZRaw() {
  return analogRead(zPin) + offSets[2];
}


int getXVolt() {
  return mapMMA7361V(getXRaw());
}


int getYVolt() {
  return mapMMA7361V(getYRaw());
}


int getZVolt() {
  return mapMMA7361V(getZRaw());
}


int getXAccel() {
  sum = 0;
  for (int i = 0; i < average; i++)
  {
    sum = sum + mapMMA7361G(getXRaw());
  }
  return sum / average;
}


int getYAccel() {
  sum = 0;
  for (int i = 0; i < average; i++)
  {
    sum = sum + mapMMA7361G(getYRaw());
  }
  return sum / average;
}


int getZAccel() {
  sum = 0;
  for (int i = 0; i < average; i++)
  {
    sum = sum + mapMMA7361G(getZRaw());
  }
  return sum / average;
}

int mapMMA7361V(int value) {
  return map(value, 0, 1024, 0, 3300);
}

int mapMMA7361G(int value) {
  return map(value, 0, 1024, -825, 800); //Bassa Sensibilità
  // return map(value,0,1024,-206,206);   //Alta Sensibilità

}

void calibrate() {
  //Serial.println(getOrientation());
  // Serial.println("\nCalibrating MMA7361011");
#define var 5000
  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;
  for (int i = 0; i < var; i++)
  {
    sumX = sumX + getXVolt();
    sumY = sumY + getYVolt();
    sumZ = sumZ + getZVolt();
    // if (i % 100 == 0)
    // {
    // Serial.print(".");
    // }
  }

  setOffSets(1672 - sumX / var, 1671 - sumY / var, + 1876 - sumZ / var); //Bassa sensibilità
  //setOffSets(1650 - sumX / var,1650 - sumY / var, + 2450 - sumZ / var);  //Alta Sesnibilità

  // Serial.println(".");
  // Serial.println(getOrientation());
  if ((abs(getOrientation())) != 3)
  {
    Serial.println("unable to calibrate");
    Pattern = 300;
    setOffSets(0, 0, 0);
  }
  else
  {
    Serial.println("Acc Ok");
  }
}


int getOrientation()
{
#define gemiddelde  10
  int x = 0;
  int y = 0;
  int z = 0;
  int xAbs = 0;
  int yAbs = 0;
  int zAbs = 0;
  for ( i = 0; i < gemiddelde ; i++) { //We take in this case 10 measurements to average the error a little bit

    x = x + getXAccel();
    y = y + getYAccel();
    z = z + getZAccel();
  }
  x = x / gemiddelde;
  y = y / gemiddelde;
  z = z / gemiddelde;
  xAbs = abs(100 - abs(x));
  yAbs = abs(100 - abs(y));
  zAbs = abs(100 - abs(z));
  if (xAbs < yAbs && xAbs < zAbs)
  {
    if (x > 0)
    {
      return 1;
    }
    return -1;
  }
  if (yAbs < xAbs && yAbs < zAbs)
  {
    if (y > 0)
    {
      return 2;
    }
    return -2;
  }
  if (zAbs < xAbs && zAbs < yAbs)
  {
    if (z > 0)
    {
      return 3;
    }
    return -3;
  }
  return 0;
}



double getPress()
{
  char status;
  double T, P, p0, a;
  // You must first get a temperature measurement to perform a pressure reading.
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Pattern = 500; //Serial.println("error retrieving pressure measurement\n");
      }
      else Pattern = 550; //Serial.println("error starting pressure measurement\n");
    }
    else Pattern = 600; //Serial.println("error retrieving temperature measurement\n");
  }
  else Pattern = 650; //Serial.println("error starting temperature measurement\n");
}
