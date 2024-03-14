/*
  This is a library written for the AS726X Spectral Sensor (Visible or IR) with I2C firmware
  specially loaded. SparkFun sells these at its website: www.sparkfun.com

  Written by Nathan Seidle & Andrew England @ SparkFun Electronics, July 12th, 2017

  https://github.com/sparkfun/Qwiic_Spectral_Sensor_AS726X

  Do you like this library? Help support SparkFun. Buy a board!

  Development environment specifics:
  Arduino IDE 1.8.1

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include "AS726X.h"
#include "UbidotsEsp32Mqtt.h"
#include "esp_adc_cal.h"

/****************************************
  Define Constants
****************************************/

const char *UBIDOTS_TOKEN = "BBFF-5ZbbA3wyWGJmJ3uoek9FCQLFKg5Srf"; // Put here your Ubidots TOKEN
const char *WIFI_SSID = "IoTIBQ"; // Put here your Wi-Fi SSID
const char *WIFI_PASS = "PswIoTIBQ"; // Put here your Wi-Fi password

const char *DEVICE_LABEL = "PDG-DEVICE"; // Put here your Device label to which data will be published, replace No with your Kit-IoT box number

const char *VARIABLE_CONC = "concentration"; // Put here your Variable label


const char *VARIABLE_VOLT = "voltaje"; // Put here your Variable label

//to which data will be published, put "AnalogValue", "temperature" or "Humidity" for example
const int PUBLISH_FREQUENCY = 600000; // Update rate in milliseconds
unsigned long timer;
Ubidots ubidots(UBIDOTS_TOKEN);

//UPDATE
AS726X sensor1;//Creates the sensor object
AS726X sensor2;//Creates the sensor object

byte GAIN = 0;
byte MEASUREMENT_MODE = 0;

void PrintAS762();
void PrintAS763();

//TwoWire I2Cone = TwoWire(0); //AS7263
//TwoWire I2Ctwo = TwoWire(1); //AS7262
int sensor1Conected = 0;
int sensor2Conected = 0;
int Turbidity = 0;  // variable to store the value coming from the sensor


uint8_t PinTurbidity = 34; // Pin used to read data from GPIO34 ADC_CH6.


float ntu;
float concent;
float Volt;
float volt1;
float volt2;
float volt3;
float voltFinal;

//extracción de medio

int volta;
int bombaB;
int bombaC;


//Bomba peristaltica
const int pinControl1 = 18; // Pin de control 1
const int pinControl2 = 19; // Pin de control 2
const int pinControl3 = 21; // Pin de control 2



// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0
// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000
int TimeOn = 2000;
int power = 255;


/*
  Set the attenuation for all channels
  Default is 11db
* */
//void analogSetAttenuation(adc_attenuation_t attenuation);



/*
  Set the attenuation for a particular pin
  Default is 11db
* */
//void analogSetPinAttenuation(uint8_t analogPin, adc_attenuation_t attenuation);


void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}




void setup() {
  //Wire.begin();
  Serial.begin(115200);





  // ubidots.setDebug(true); // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  timer = millis();


  //bomba
  pinMode(pinControl1, OUTPUT);
  pinMode(pinControl2, OUTPUT);
  pinMode(pinControl3, OUTPUT);
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);

  concent = 0;
  Volt = 0;
  volt1 = 0;
  volt2 = 0;
  volt3 = 0;
  ntu = 0;
  voltFinal = 0;

  volta = 0;
  bombaB = 0;
  bombaC = 0;

  Serial.println("Setup----------------------- ");
  Wire.begin();
  if (sensor1Conected == 1) {
    sensor1.begin();
    //sensor1.begin(I2Cone, GAIN, MEASUREMENT_MODE);//Initializes the sensor with non default values
    Serial.println("Setup sensor1  I2Cone.begin ----------------------- ");
    delay(1000);
  }

  if (sensor2Conected == 1) {
    sensor2.begin();
    //sensor2.begin(I2Ctwo, GAIN, MEASUREMENT_MODE);//Initializes the sensor with non default values
    Serial.println("Setup  sensor 2  I2Ctwo.begin ----------------------- ");
    sensor2.disableIndicator();
    sensor2.disableBulb();
    Serial.println("disableBulb 1 ----------------------- ");
    delay(5000);

    sensor2.disableIndicator();
    sensor2.disableBulb();

    Serial.println("=============V3 7 nov 2023===================");
    Serial.println("A : take Measurements AS762 ");
    Serial.println("B : take Measurements AS763 ");
    Serial.println("S : take Measurements With Bulb AS763 ");
    Serial.println("S : take Measurements With Bulb AS762 ");
    Serial.println("T : Read Turbidity");
    Serial.println("txxx : time xxxx ms pump");
    Serial.println("pxxx : power [200 - 255] speed pump");
    Serial.println("f    : forward pump");
    Serial.println("r    : reverse pump");
    Serial.println("====================================");
  }
}

void loop() {
  //delay(100);
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    Serial.print("--> ");
    Serial.print(char(inByte));
    Serial.print(": ");
    switch (inByte) {


      case 'B':
        Serial.println("Bomba 2 ON");
        encenderBomba2();
        delay(TimeOn);
        apagarBomba2();
        inByte = 0;
        break;

      case 'C':
        Serial.println("Bomba 3 ON");
        encenderBomba3();
        delay(TimeOn);
        apagarBomba3();
        inByte = 0;
        break;

      case 'N':
        inByte = 0;
        ntu = 0;

        Volt = measureTurbidity();

        Serial.print("NTU= ");
        Serial.print(ntu);
        Serial.print(" ,  ");
        Serial.print(Volt);
        Serial.print(" V , ADC= ");
        Serial.println(analogRead(PinTurbidity));

        /*

          CALCULAR CONCENTRACIÓN

        */
        break;

      case 't':  //reset
        TimeOn = Serial.parseInt(); //dataIn now holds 314
        Serial.print("time ON = ");
        Serial.print(TimeOn);
        Serial.println(" ms");
        break;

      case 'p':
        power = (Serial.parseInt()); //dataIn now holds 314
        Serial.print("Power = ");
        Serial.println(power);
        //Serial.println("");
        break;


      case 'L':

        volta = 0;
        bombaB = 0;
        bombaC = 0;

        volta = Serial.parseInt();

        // extracción de medio
        bombaB = (volta - 33.699) / 0.0059; // bomba b
        Serial.print("Tiempo extracción bomba B: ");
        Serial.print(bombaB);
        Serial.println("s ");

        encenderBomba2();
        delay(bombaB);
        apagarBomba2();



        //ingreso de medio

        bombaC = (volta -   3.1456) / 0.0059;   // bomba C
        Serial.print("Tiempo ingreso bomba C: ");
        Serial.println(bombaB);
        Serial.println("s ");
        encenderBomba2();
        delay(bombaC);
        apagarBomba2();


        break;

      case 'M':
        /*
            ------ MEDICIONES --------
        */


        Serial.print("RV ");
        Serial.print(TimeOn);
        Serial.print(" ms;  ");
        Serial.println(power );
        Serial.println("|<");
        Serial.println("");


        Serial.println("");
        Serial.println(" ----- RECIRCULANDO ----- ");
        Serial.println("");

        recircularBomba1();


        /*
          --------- PRIMER MEDICIÓN -----------
        */

        Serial.println(" ----- Primer medición ----- ");

        volt1 = 0;                   // reestablecer el voltaje a 0
        ntu = 0;                     // reestablecer ntu a 0
        volt1 = measureTurbidity();  // Se mide la turbidez de la muestra

        Serial.print("NTU= ");
        Serial.print(ntu);
        Serial.print(" ,  ");
        Serial.print(volt1);
        Serial.print(" V , ADC= ");
        Serial.println(analogRead(PinTurbidity));


        Serial.println("");
        Serial.print("Concetración 1= ");
        Serial.println(2.6898 * exp(-0.859 * volt1));
        delay(1000);

        /*
          --------- RECIRCULAR -----------
        */
        Serial.println("");
        Serial.println(" ----- RECIRCULANDO ----- ");
        Serial.println("");

        recircularBomba1();


        /*
          --------- SEGUNDA MEDICIÓN -----------
        */
        Serial.println(" ----- Segunda medición ----- ");

        volt2 = 0;                   // reestablecer el voltaje a 0
        ntu = 0;                     // reestablecer ntu a 0
        volt2 = measureTurbidity();  // Se mide la turbidez de la muestra

        Serial.print("NTU= ");
        Serial.print(ntu);
        Serial.print(" ,  ");
        Serial.print(volt2);
        Serial.print(" V , ADC= ");
        Serial.println(analogRead(PinTurbidity));


        Serial.println("");
        Serial.print("Concetracion 2= ");
        Serial.println(2.6898 * exp(-0.859 * volt2));
        delay(1000);


        /*
          --------- RECIRCULAR -----------
        */
        Serial.println("");
        Serial.println(" ----- RECIRCULANDO ----- ");
        Serial.println("");

        recircularBomba1();

        /*
                 --------- TERCERA MEDICIÓN -----------
        */
        Serial.println(" ----- Tercera medición ----- ");

        volt3 = 0;                   // reestablecer el voltaje a 0
        ntu = 0;                     // reestablecer ntu a 0
        volt3 = measureTurbidity();  // Se mide la turbidez de la muestra

        Serial.print("NTU= ");
        Serial.print(ntu);
        Serial.print(" ,  ");
        Serial.print(volt3);
        Serial.print(" V , ADC= ");
        Serial.println(analogRead(PinTurbidity));


        Serial.println("");
        Serial.print("Concetracion 3= ");
        Serial.println(2.6898 * exp(-0.859 * volt3));
        delay(1000);


        voltFinal = (volt1 + volt2 + volt3) / 3.0;

        concent = 2.6898 * exp(-0.859 * voltFinal);
        //2,5813*EXP(-0,845*voltaje)
        Serial.println(voltFinal);
        Serial.print(concent);
        Serial.print(" g/L");
        Serial.println("");


        //UBIDOTS CONECTION
        if (!ubidots.connected())
        {
          ubidots.reconnect();
        }
        //if (abs(double(millis() - timer)) > PUBLISH_FREQUENCY) { // triggers the routine every 5 seconds


        ubidots.add(VARIABLE_CONC, concent); // Insert your variable Labels and the value to be sen
        ubidots.add(VARIABLE_VOLT, voltFinal);

        ubidots.publish(DEVICE_LABEL);
        // ubidots.publish();
        // timer = millis();
        // }

        break;

      default:
        //  Serial.print(".");
        break;
    }
  }

  //sensor.takeMeasurements();


  //sensor.setGain(3);



  /// Serial.println();




  //UBIDOTS CONECTION
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  if (abs(double(millis() - timer)) > PUBLISH_FREQUENCY) { // triggers the routine every 5 seconds

    /*

        SE REPITE CADA 15 MINUTOS


    */

    /*
               ------ MEDICIONES --------
    */


    Serial.print("RV ");
    Serial.print(TimeOn);
    Serial.print(" ms;  ");
    Serial.println(power );
    Serial.println("|<");
    Serial.println("");


    Serial.println("");
    Serial.println(" ----- RECIRCULANDO ----- ");
    Serial.println("");

    recircularBomba1();


    /*
      --------- PRIMER MEDICIÓN -----------
    */

    Serial.println(" ----- Primer medición ----- ");

    volt1 = 0;                   // reestablecer el voltaje a 0
    ntu = 0;                     // reestablecer ntu a 0
    volt1 = measureTurbidity();  // Se mide la turbidez de la muestra

    Serial.print("NTU= ");
    Serial.print(ntu);
    Serial.print(" ,  ");
    Serial.print(volt1);
    Serial.print(" V , ADC= ");
    Serial.println(analogRead(PinTurbidity));


    Serial.println("");
    Serial.print("Concetración 1= ");
    Serial.println(2.6898 * exp(-0.859 * volt1));
    delay(1000);

    /*
      --------- RECIRCULAR -----------
    */
    Serial.println("");
    Serial.println(" ----- RECIRCULANDO ----- ");
    Serial.println("");

    recircularBomba1();


    /*
      --------- SEGUNDA MEDICIÓN -----------
    */
    Serial.println(" ----- Segunda medición ----- ");

    volt2 = 0;                   // reestablecer el voltaje a 0
    ntu = 0;                     // reestablecer ntu a 0
    volt2 = measureTurbidity();  // Se mide la turbidez de la muestra

    Serial.print("NTU= ");
    Serial.print(ntu);
    Serial.print(" ,  ");
    Serial.print(volt2);
    Serial.print(" V , ADC= ");
    Serial.println(analogRead(PinTurbidity));


    Serial.println("");
    Serial.print("Concetracion 2= ");
    Serial.println(2.6898 * exp(-0.859 * volt2));
    delay(1000);


    /*
      --------- RECIRCULAR -----------
    */
    Serial.println("");
    Serial.println(" ----- RECIRCULANDO ----- ");
    Serial.println("");

    recircularBomba1();

    /*
             --------- TERCERA MEDICIÓN -----------
    */
    Serial.println(" ----- Tercera medición ----- ");

    volt3 = 0;                   // reestablecer el voltaje a 0
    ntu = 0;                     // reestablecer ntu a 0
    volt3 = measureTurbidity();  // Se mide la turbidez de la muestra

    Serial.print("NTU= ");
    Serial.print(ntu);
    Serial.print(" ,  ");
    Serial.print(volt3);
    Serial.print(" V , ADC= ");
    Serial.println(analogRead(PinTurbidity));


    Serial.println("");
    Serial.print("Concetracion 3= ");
    Serial.println(2.6898 * exp(-0.859 * volt3));
    delay(1000);


    voltFinal = (volt1 + volt2 + volt3) / 3.0;

    concent = 2.6898 * exp(-0.859 * voltFinal);
    //2,5813*EXP(-0,845*voltaje)
    Serial.println(voltFinal);
    Serial.print(concent);
    Serial.print(" g/L");
    Serial.println("");


    if (!ubidots.connected())
    {
      ubidots.reconnect();
    }


    ubidots.add(VARIABLE_CONC, concent); // Insert your variable Labels and the value to be sen
    ubidots.add(VARIABLE_VOLT, voltFinal);

    ubidots.publish(DEVICE_LABEL);
    //ubidots.publish();
    timer = millis();
  }








}






float measureTurbidity() {

  // inByte = 0;  // what is this?
  float volt = 0;

  for (int i = 0; i < 800; i++) {
    volt += ((float)analogRead(PinTurbidity) / 4093) * 5; //3.3
  }
  volt = volt / 800;
  volt = round_to_dp(volt, 3);
  if (volt < 2.5) {
    ntu = 3000;
  } else {
    ntu = -1120.4 * sq(volt) + 5742.3 * volt - 4353.8;
  }

  return volt;

}


float round_to_dp(float in_value, int decimal_place) {
  float multipler = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multipler) / multipler;
  return in_value;
}


void recircularBomba1() {
  encenderBomba1();      // Enciende bomba 1
  delay(6000);           // Espera 6 seg
  apagarBomba1();        // Apaga bomba 1
  delay(2000);          // Espera 15 seg
}


// Función para encender la bomba
void encenderBomba1() {
  analogWrite(pinControl1, power);
}


// Función para apagar la bomba
void apagarBomba1() {
  analogWrite(pinControl1, LOW);
}

void encenderBomba2() {
  analogWrite(pinControl2, power);
}


// Función para apagar la bomba
void apagarBomba2() {
  analogWrite(pinControl2, LOW);
}

void encenderBomba3() {
  analogWrite(pinControl3, power);
}


// Función para apagar la bomba
void apagarBomba3() {
  analogWrite(pinControl3, LOW);
}





void PrintAS762() {
  if (sensor2.getVersion() == SENSORTYPE_AS7262)
  {
    //Visible readings
    //sensor.enableBulb();
    Serial.print(" V: ");
    Serial.print(sensor2.getCalibratedViolet(), 2);
    Serial.print(" , B: ");
    Serial.print(sensor2.getCalibratedBlue(), 2);
    Serial.print(" , G: ");
    Serial.print(sensor2.getCalibratedGreen(), 2);
    Serial.print(" , Y: ");
    Serial.print(sensor2.getCalibratedYellow(), 2);
    Serial.print(" , O: ");
    Serial.print(sensor2.getCalibratedOrange(), 2);
    Serial.print(" , R: ");
    Serial.print(sensor2.getCalibratedRed(), 2);
    Serial.print(" , temp2: ");
    Serial.print(sensor2.getTemperature(), 1);
    Serial.print(" ");
    //sensor.disableBulb();
    Serial.println();

  }

}

void PrintAS763() {
  if (sensor1.getVersion() == SENSORTYPE_AS7263)
  {
    //Near IR readings
    // sensor.enableBulb();
    Serial.print("R: ");
    Serial.print(sensor1.getCalibratedR(), 2);
    Serial.print(", S: ");
    Serial.print(sensor1.getCalibratedS(), 2);
    Serial.print(", T: ");
    Serial.print(sensor1.getCalibratedT(), 2);
    Serial.print(", U: ");
    Serial.print(sensor1.getCalibratedU(), 2);
    Serial.print(", V: ");
    Serial.print(sensor1.getCalibratedV(), 2);
    Serial.print(", W: ");
    Serial.print(sensor1.getCalibratedW(), 2);
    Serial.print(", temp1: ");
    Serial.print(sensor1.getTemperature(), 1);
    Serial.print(" ");
    Serial.println();
    //  sensor.disableBulb();
  }

}
