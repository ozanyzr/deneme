/* -------------------- MACRO -----------------------*/
//#define GLIDER // else Container
//#define DEBUG

//LİBRARY
#ifdef GLIDER
  #include <Adafruit_Sensor.h>
  #include <Adafruit_LSM303_U.h>
  #include <DS1302.h>
  #include <SD.h>
#else
  #include <Servo.h>
#endif

#include <SFE_BMP180.h>
#include <Wire.h>
#include <XBee.h>
#include <EEPROM.h>

/* -------------------- Variable -----------------------*/
struct config_t
{
    bool IsStarted;
    double BaseLine;
    double PacketCount;
    unsigned long MissionTimeBase;
    unsigned int Mesafeler[3];
} HardData;

#ifdef GLIDER
  double Basinc,Speed,Heading;
  String Head= "5543,GLIDER,";
  #define PITO1 A0
  #define PITO2 A6
  #define VOLTAGE A7
  #define RST  6
#define DAT  7 
#define CLK  8
  DS1302 rtc(RST, DAT, CLK);
#else
  String Head= "5543,CONTAINER,";
  double Basinc;
  #define SERVOM 9
  Servo myservo;
  ZBRxResponse rx = ZBRxResponse();
#endif

#define chipSelect 5

#define LED  3
#define BUTTON 10
#define BUZZER 4

#define SERVOLOCKEDPOS 150
#define SERVORELASEPOS 60


XBee xbee = XBee();
SFE_BMP180 pressure;
double Sicaklik,Yukseklik,Voltage;
/*unsigned*/ double TSecond;
int PacketCount=0;
String Software_state;
String Telemetry=""; 
double baseline; // baseline pressure
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x4156561a);// SH + SL Address of receiving XBee
ZBTxRequest zbTx ;//= ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
//Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345); 
//sensors_event_t event;
/* -------------------- FUNCTION -----------------------*/
#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
#endif

#ifdef GLIDER
#else
  void Relase(){
    DEBUG_PRINT("R");
    myservo.write(SERVORELASEPOS); 
  }
#endif

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}
/* void displaySensorDetails(void)
 {
   sensor_t sensor;
   mag.getSensor(&sensor);
   Serial.println("------------------------------------");
   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
   Serial.println("------------------------------------");
   Serial.println("");
   delay(500);
 }*/
/*void printTime() {
  Time t = rtc.time();
  //const String zaman = (t.zaman);
  char buf[50];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
         //  day.c_str(),
           t.hr, t.min, t.sec);
  Serial.println(buf);
}*/

// You will need to create an SFE_BMP180 object, here called "pressure":

// create the XBee object
void HATA()
{
  digitalWrite(LED,HIGH);
}
void Start()
{
  DEBUG_PRINT("S");
  EEPROM_readAnything(0, HardData);
  if(HardData.IsStarted || (!digitalRead(BUTTON)) )
    return;
  HardData.IsStarted = true;
  HardData.BaseLine = 12;
  HardData.PacketCount = 0;
  EEPROM_writeAnything(0, HardData);
  //while(1);
}
void setup()
{
  Start();
  EEPROM_readAnything(0, HardData);
  HardData.Mesafeler[0] = HardData.Mesafeler[1] =HardData.Mesafeler[2]= 0;
  Serial.begin(9600);
  xbee.setSerial(Serial);
  //mag.enableAutoRange(true);
  #ifdef GLIDER
    DEBUG_PRINT("GLIDER");
    if (!SD.begin(chipSelect)) {
    HATA();
    DEBUG_PRINT("SD");
    // don't do anything more:
    //return;
  }
  #else
    DEBUG_PRINT("CONTAINER");
    myservo.attach(9);
    myservo.write(SERVOLOCKEDPOS); 
    //pinMode(SERVOM, OUTPUT);
  #endif
  //Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(LED,LOW);
  digitalWrite(BUZZER,LOW);
  // see if the card is present and can be initialized:
  
  DEBUG_PRINT("card initialized.");
  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
  {
    //DEBUG_PRINT("BMP180 init success");
  }
  else
  {
    DEBUG_PRINT("BMP");
   // Oops, something went wrong, this is usually a connection problem,
   // see the comments at the top of this sketch for the proper connections.
   //while(1); // Pause forever.
  }
  /*if(!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    DEBUG_PRINT("LSM");
    //while(1);
  }*/
  
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  // Get the baseline pressure:
  if(!setPressureAndTemp()){
     DEBUG_PRINT("BL");  //bu hata f onksiyonda gösteriliyor
     HATA();
  }
  else
  {
    baseline = Basinc;
    DEBUG_PRINT(baseline);
    DEBUG_PRINT("mb"); 
  }
  
  
}
 
void loop()
{
  EEPROM_readAnything(0, HardData);
  HardData.PacketCount++;

  /* -------------------- LOOP -----------------------*/
  #ifdef GLIDER
    Speed=25.4;
    Heading=4545;
    Voltage=analogRead(VOLTAGE);
    Time t = rtc.time();
  #else
  HardData.Mesafeler[0]=HardData.Mesafeler[1];
  HardData.Mesafeler[1]=HardData.Mesafeler[2];
  Yukseklik=HardData.Mesafeler[2] = pressure.altitude(Basinc,baseline);
  if (   HardData.Mesafeler[0]>HardData.Mesafeler[1]
      && HardData.Mesafeler[1]>HardData.Mesafeler[2]
      && HardData.Mesafeler[2]<400 
      && HardData.Mesafeler[2]>100 )
      {
        Relase();
      }
  #endif
  
  PacketCount=HardData.PacketCount;
  Software_state="idle";
  
  //mag.getEvent(&event);
  
  
  //t.hr, t.min, t.sec
  //Time t 
  TSecond = PacketCount;//t.sec+t.min*60;//t.hr*60*60 t.min, t.sec
  if(!setPressureAndTemp()){
     //DEBUG_PRINT("er:setPresATemp");  //bu hata f onksiyonda gösteriliyor
     HATA();
  }
 
  /*
    "TeamID","Source","MissionTime","PacketCount","Altitude","Pressure","Speed","Temp","Voltage","Heading","SoftwareState",
    "TeamID","Source","MissionTime","PacketCount","Altitude","Temp","Voltage","SoftwareState"
  */
  // telemetry
  Telemetry = String(Head);
  Telemetry+=String(TSecond);
  Telemetry+=",";
  Telemetry+=String(HardData.PacketCount);
  Telemetry+=",";
  Telemetry+=String(Yukseklik);
  Telemetry+=",";
  
   #ifdef GLIDER
    Telemetry+=String(Basinc);
    Telemetry+=",";
    Telemetry+=String(Speed);
    Telemetry+=",";
  #endif
  
  Telemetry+=String(Sicaklik);
  Telemetry+=",";
  Telemetry+=String(Voltage);
  Telemetry+=",";
  #ifdef GLIDER
    Telemetry+=String(Heading);
    Telemetry+=",";
  #endif
  Telemetry+=String(Software_state);
  //Telemetry+=",";
  
  
  
  
  // Telemetry+=String(event.magnetic.x);Telemetry+=":";
  // Telemetry+=String(event.magnetic.y);Telemetry+=":";
  // Telemetry+=String(event.magnetic.z);Telemetry+=":";
  // Telemetry+=",";
  
  
  
  //Serial.println(Telemetry);
  
  
 
  
  
  // Get a new pressure reading:
  zbTx = ZBTxRequest(addr64, Telemetry.c_str(), Telemetry.length());
  xbee.send(zbTx);
  
  
  // Show the relative altitude difference between
  // the new reading and the baseline reading:
  // after sending a tx request, we expect a status response
  // wait up to half second for the status response
  if (xbee.readPacket(500)) {
   // got a response!
  
   // should be a znet tx status              
   if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
     xbee.getResponse().getZBTxStatusResponse(txStatus);
  
     // get the delivery status, the fifth byte
     if (txStatus.getDeliveryStatus() == SUCCESS) {
       // success.  time to celebrate
       //DEBUG_PRINT("success");//flashLed(statusLed, 5, 50);
     } else {
       // the remote XBee did not receive our packet. is it powered on?
       DEBUG_PRINT("RX");//flashLed(errorLed, 3, 500);
     }
   }
  } 
  else if (xbee.getResponse().isError()) {
   //nss.print("Error reading packet.  Error code: ");  
   //nss.println(xbee.getResponse().getErrorCode());
  }
  else {
   // local XBee did not provide a timely TX Status Response -- should not happen
   //flashLed(errorLed, 2, 50);
   HATA();
   DEBUG_PRINT("ACK");
  }
  #ifdef GLIDER
    File dataFile= SD.open("log.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(Telemetry); //telemtry yazdırılacak ya.
      dataFile.close();
    }  
    else {
      HATA();
      DEBUG_PRINT("txt");
    } 
  #else
    xbee.readPacket ();
    
    if (xbee.getResponse().isAvailable()) {
      // got something
        Serial.println("1");
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        // got a zb rx packet
          Serial.println("2");
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
            Serial.println("3");
        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
            
            if(rx.getData(0)==54)
            {
              Relase();
            }
            if(rx.getData(0)==37)
            {
              
              HardData.IsStarted = false;
              EEPROM_writeAnything(0, HardData);
              Start();
            }
            
            //flashLed(statusLed, 10, 10);
            
        } else {
            // we got it (obviously) but sender didn't get an ACK
            //flashLed(errorLed, 2, 20);
        }
        // set dataLed PWM to value of the first byte in the data
        //analogWrite(dataLed, rx.getData(0));
      } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
        /*xbee.getResponse().getModemStatusResponse(msr);
        // the local XBee sends this response on certain events, like association/dissociation
        
        if (msr.getStatus() == ASSOCIATED) {
          // yay this is great.  flash led
          //flashLed(statusLed, 10, 10);
        } else if (msr.getStatus() == DISASSOCIATED) {
          // this is awful.. flash led to show our discontent
          //flashLed(errorLed, 10, 10);
        } else {
          // another status
          //flashLed(statusLed, 5, 10);
        }*/
      } else {
        // not something we were expecting
        //flashLed(errorLed, 1, 25);    
      }
    } else if (xbee.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");  
      //nss.println(xbee.getResponse().getErrorCode());
    }
  #endif

  EEPROM_writeAnything(0, HardData);
  delay(700);
}

bool setPressureAndTemp()
{
 char status;

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

   status = pressure.getTemperature(Sicaklik);
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

       status = pressure.getPressure(Basinc,Sicaklik);
       
       /*if (status != 0)
       {
         return(1);
       }

       else DEBUG_PRINT("er ret pres\n");*/
       
       if (status == 0)
       {
         DEBUG_PRINT("rp");
       }
     }
     else DEBUG_PRINT("Sp");
   }
   else DEBUG_PRINT("rt");
 }
 else DEBUG_PRINT("st");
 return status;
}
