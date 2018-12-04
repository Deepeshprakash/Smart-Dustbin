
// Codes for Smart Waste Management Services
// 1. ultrasonic measurement return needed
// 2. Lcd Display of the garbage 
// 3. Gps Coordinate 
// 4. Motor drive 
// 5. Configuration Display Information and Battery Status
/***************************************************Header Inclusion portion**************************************************************************/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include<Wire.h>
#include<LiquidCrystal_I2C.h>

/******************************************************* Configuration Portion **************************************************************************************************
****************************************************Make Changes only in this section *******************************************************************************/


// Pin declearation part
#define Trigger_Pin 2   //  update the respective pin of nodemcu to make any changes
#define Echo_Pin 0  //

#define Upward 5        //Motor Control Pins 
#define Downward 4


#define RX_Pin 4
#define TX_Pin 5

#define GPSBaud 9600             //if Baud rate 9600 didn't work in your case then use 4800

char auth[] = "be11cc5cf83e46c18e4c722f83b538f4";              //Your Project authentication key
char ssid[] = "myhome";                                        // Name of your network (HotSpot or Router name)
char pass[] = "xerophyte@31527a";                              // Your Password of Router or HotSpot


/********************************************************************************************************************************************/

// variable decleratin part
long duration;
int distance;

//unsigned int move_index;         // moving index, to be used later
unsigned int move_index = 1;       // fixed location for now

SoftwareSerial ss(RX_Pin, TX_Pin);  // The serial connection to the GPS device
LiquidCrystal_I2C lcd(0X3F,16,2);

TinyGPSPlus gps; // The TinyGPS++ object
WidgetMap myMap(V0);  // V0 for virtual pin of Map Widget

BlynkTimer timer;





/***********************************Function to Measrue the distance of the Smart Dustbin*****************************************************
*************************************Measure distance in CM and return the value *************************************************************
**********************************************************************************************************************************************
 */
int Distance_Measured()
{
  digitalWrite(Trigger_Pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger_Pin,LOW);
  duration=pulseIn(Echo_Pin,HIGH);
  distance=duration*0.34/2;  // distance in MM
  Serial.println("Distance:");
  Serial.println(distance);
  delay(200);
  return distance;
}

/*******************************************************************************************************************************************/

/***********************************Function to Move the compressor up and down*************************************************************
*************************************Accept the distance value and move up/down accordingly *************************************************
**********************************************************************************************************************************************
 */
void Compressor_Control(int distance)
{
  digitalWrite(Downward,HIGH);
  delay(distance*4000+2000);
  digitalWrite(Upward,HIGH);
  delay(distance*4000);
  
  
}
/**********************************************************************************************************************************************/



/***********************************Function to check Gps Status ***************************************************************************
*************************************Display the failure or configured information  *******************************************************
**********************************************************************************************************************************************
 */
 void checkGPS(){
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
     
  }
}

/***********************************Function to display Gps Status ***************************************************************************
*************************************Display the failure or configured information  *******************************************************
**********************************************************************************************************************************************
 */

void displayInfo()
{ 

  if (gps.location.isValid() ) 
  {
    
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon. 27.700769 85.300140
    float longitude = (gps.location.lng()); 
    //float latitude=27.700769;
    //float longitude=85.300140;
    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);
    Blynk.virtualWrite(V1, String(latitude, 6));   
    Blynk.virtualWrite(V2, String(longitude, 6));  
    myMap.location(move_index, latitude, longitude, "GPS_Location");
   // spd = gps.speed.kmph();               //get speed
      // Blynk.virtualWrite(V3, spd);
       
      // sats = gps.satellites.value();    //get number of satellites
      // Blynk.virtualWrite(V4, sats);

      // bearing = TinyGPSPlus::cardinal(gps.course.value()); // get the direction
      // Blynk.virtualWrite(V5, bearing);               
      
    
  }
  

  Serial.println();
}
void setup() {
  
// put your setup code here, to run once:

Serial.begin(115200);
/**************************************Initializatin of Ultrasonic Sensor*********************************************************/

pinMode(Trigger_Pin,OUTPUT);
pinMode(Echo_Pin, INPUT);

/***************************************Initialization of Motor for Compressor Control********************************************/

pinMode(Upward,OUTPUT);    //Motor control Pins
pinMode(Downward,OUTPUT);  //Motor control Pins

/***************************************Initializatin of LCD******************************************************************/

Wire.begin(2,0);
lcd.init();               //initializing the LCD
lcd.backlight();          //Enable or Turn on the backlight
lcd.print(" Useful Information");

/*******************************************Initialization of Blinky App and GPS Module**********************************************/

ss.begin(GPSBaud); //gps module started
Blynk.begin(auth, ssid, pass);
timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once

//Serial.begin(9600);

}


void loop() {
     // put your main code here, to run repeatedly:
            //1. Read the present waste level 
            //2. Send the present waste level and its coordinate to blinky app for display
            //3. if the Waste level is greater than certain threshold compress the waste 
   

}