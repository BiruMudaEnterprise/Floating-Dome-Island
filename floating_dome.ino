/*KONFIGURASI PIN
ETHERNET
SCK  - 13
MOSI - 11
MISO - 12
CS   - 10
INT  - 2

SERIAL
TXD - 1
RXD - 0

ADC
LM35 - A0
LDR  - A1
MQ2  - A2
IR   - A3
SOIL - A4

USIRR
TRIGGER - 8
ECHO    - 7

BUZZER 4
motor DC 5

DHT PAKE KOMUNIKASI 1 WIRE DENGAN MEMBERI RESISTOR 5K

LED 3 5
*/

/*
Support:  Tiequan Shao: support[at]sandboxelectronics.com

Lisence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)

Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application.
         */
         
#include <SPI.h>
#include <Ethernet.h>
#include <NewPing.h>
#include <DHT.h>

//setting ethernet
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 95, 6);
// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

//PIN DHT
#define DHTPIN 2
#define DHTTYPE DHT11

//PINGER
#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//VARIABEL SENSOR
int tilt;
int ldr;
int wl;
int flame;
unsigned long cm;
int humid;
int temp;
int smoke;
int pot;

int pitch;
int roll;

DHT dht(DHTPIN, DHTTYPE);

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (A4)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation

/**********************Application Related Macros**********************************/
#define         GAS_SMOKE                    (2)

float           SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


void setup() {

  noTone(4);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  //digitalWrite(5,HIGH);
  //delay(5000);
  //digitalWrite(5,LOW);
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  server.begin();
  dht.begin();
  Ro = MQCalibration(MQ_PIN);

}

void loop() {
  Serial.println("Data get");
  getdata();
  listenForEthernetClients();
  buzer();
  // delay(5000);
}


long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void getdata()
{
  tilt = analogRead(A0); // tilt
  ldr = analogRead(A1); //cahaya
  wl = analogRead(A2); //water level
  flame = analogRead(A3); //flame
  cm = sonar.ping_cm();
  humid = dht.readHumidity();
  temp = dht.readTemperature(); //suhu
  smoke = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE); //smoke
  pot = analogRead(A5); //roll

  //PITCH
  if (tilt < 570)//-20
  { pitch = 560;
  }
  else if (tilt >= 570 && tilt < 580)//-15
  { pitch = 570;
  }
  else if (tilt >= 580 && tilt < 590)//-10
  { pitch = 580;
  }
  else if (tilt >= 590 && tilt < 600)//-5
  { pitch = 590;
  }
  else if (tilt>=600 && tilt<610)//0
  {pitch=600;
  }
  else if (tilt>=610 && tilt<620)//5
  {pitch=610;
  }
  else if (tilt>=620 && tilt<630)//10
  {pitch=620;
  }
  else if (tilt>=630 && tilt<640)//15
  {pitch=630;
  }
  else if (tilt>=640)//20
  {pitch=640;
  }

//ROLL
  if (pot < 450)//-20
  { roll = 560;
  }
  else if (pot >= 450 && pot < 460)//-15
  { roll = 570;
  }
  else if (pot >= 460 && pot < 470)//-10
  { roll = 580;
  }
  else if (pot >= 470 && pot < 480)//-5
  { roll = 590;
  }
  else if (pot>=480 && pot<490)//0
  {roll=600;
  }
  else if (pot>=490 && pot<500)//5
  {roll=610;
  }
  else if (pot>=500 && pot<510)//10
  {roll=620;
  }
  else if (pot>=510 && pot<520)//15
  {roll=630;
  }
  else if (pot>=520)//20
  {roll=640;
  }
  /*
  Serial.print("pitch=");
  Serial.println(tilt);
  Serial.print("light=");
  Serial.println(ldr);
  Serial.print("water level=");
  Serial.println(wl);
  Serial.print("flame=");
  Serial.println(flame);
  Serial.print("dist=");
  Serial.println(cm);
  Serial.print("humid=");
  Serial.println(humid);
  Serial.print("temp=");
  Serial.println(temp);
  Serial.print("smoke=");
  Serial.println(smoke);
  Serial.print("roll=");
  Serial.println(pot);
  Serial.print("\r\n");
  
*/
}

void listenForEthernetClients() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    //Serial.println("Got a client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          // print the current readings, in HTML format:
          client.print("DATAMANTIS|");
          client.print(pitch);
          client.print("|");
          client.print(ldr);
          client.print("|");
          client.print(wl);
          client.print("|");
          client.print(flame);
          client.print("|");
          client.print(cm);
          client.print("|");
          client.print(humid);
          client.print("|");
          client.print(temp);
          client.print("|");
          client.print(smoke);
          client.print("|");
          client.print(600);
          client.print("|");
          client.print("DATAMANTIS");
          break;

        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
}

void buzer()
{
/*  if (pitch >= 640 || pitch <=560) //pitch
  {
    for (int i = 0; i < 3; i++)
    {
      tone(4, 262, 200); //c6
      delay(100);
      
    }
    noTone(4);
  }
  
  if (roll >= 640 || roll <=560) //roll
  {
    for (int i = 0; i < 3; i++)
    {
      tone(4, 2093, 200); //c7
      delay(100);
      
    }
    noTone(4);
  }
*/

  if (ldr < 600) //LDR
  { digitalWrite(3, HIGH);

  }
  else {
    digitalWrite(3, LOW);

  }

/*
if (wl > 700) //water level
  { for (int i = 0; i < 4; i++)
    {
      tone(4, 1175, 200); //d6
      delay(100);
      noTone(4);
    }
  }
  
  if (flame > 30) //FLAME
  { for (int i = 0; i < 4; i++)
    {
      tone(4, 1319, 200); //e6
      delay(100);
      noTone(4);
    }
  }
*/
  if ( cm>=1 && cm < 25) //USIRR
  { for (int i = 0; i < 4; i++)
    {
      tone(4, 1397, 200); //f6
      delay(100);
      noTone(4);
    }
  }

/*
    if (temp > 27) //TEMP
  { for (int i = 0; i < 3; i++)
    {
      tone(4, 1568, 150); //g6
      delay(100);
      noTone(4);
    }
  }
  
  if (smoke > 200) //SMOKE
  { digitalWrite(5, HIGH);
    for (int i = 0; i < 3; i++)
    {
      tone(4, 2093, 100); //C7
      delay(200);
      noTone(4);
    }
    delay(2000);
    digitalWrite(5,LOW);
    
  }
*/  
}


/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
