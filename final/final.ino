#include "DHT.h"                                  //include dht header file
#define DHTPIN 2                                  //define pin 2 to take output
#define DHTTYPE DHT11                             //select dht11 rather than dht22
DHT dht(DHTPIN, DHTTYPE);                         //function call from dht header file


                 //initialization and declaration part
unsigned  int  state = 0;              
float h=0;                                      //variable for humidity 
float t=0;                                      //variable for temperature 
int MOTOR = 6;                                   //motor pin1
int MOTOR1 = 7;                                 //motor pin2

void setup() 
{
              pinMode(MOTOR, OUTPUT);           //set motor pin1(6) as output
              digitalWrite(MOTOR, LOW);
              pinMode(MOTOR1, OUTPUT);          //set motor pin1(7) as output
              digitalWrite(MOTOR1, LOW);
              /--
              pinMode(MOTOR, OUTPUT);           //set motor pin1(6) as output
              digitalWrite(MOTOR, LOW);
              pinMode(MOTOR1, OUTPUT);          //set motor pin1(7) as output
              digitalWrite(MOTOR1, LOW);
              //--
              pinMode(MOTOR, OUTPUT);           //set motor pin1(6) as output
              digitalWrite(MOTOR, LOW);
              pinMode(MOTOR1, OUTPUT);          //set motor pin1(7) as output
              digitalWrite(MOTOR1, LOW);
       
              Serial.begin(9600);               //baud rate communication 
              dht.begin();                      //start communication with dht11
}

void loop()
{
  
    
            if(Serial.available() > 0)           //Get the number of bytes (characters) available for reading from the serial port
            {
              state = Serial.read();              //Read incoming data
            }
     
     
     
     switch (state)                              //select operation which is to be performed
     {
        
      case 'T':                                 //TEMPERATURE
     
                   delay(700);
                   t = dht.readTemperature();      // Read temperature from digital pin 2 
                   Serial.println(t);              //display and send temperature to xbee
                   break;
      
      case 'H':                               //HUMIDITY
                  delay(700);
                  h = dht.readHumidity();         // Read humidity from digital pin 2 
                  Serial.println(h);             //display and send humidity to xbee
                  break;
      
      case 'R':                               //MAKE MOTOR ON
                 digitalWrite(MOTOR, HIGH);
                 digitalWrite(MOTOR1, LOW);
                 Serial.println("MOTOR:ON");
                 break;
     
     
      case 'F':                           //STOP MOTOR
                 digitalWrite(MOTOR, LOW);
                 digitalWrite(MOTOR1, LOW);
                 Serial.println("MOTOR:OFF");
                 break;
     }

}
