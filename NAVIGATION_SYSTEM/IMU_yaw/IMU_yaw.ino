#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <JY901.h>
#include <ArduinoJson.h>

static const int RXPin = 8, TXPin = 7; 
static const uint32_t GPSBaud = 9600; 


// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


//using namespace BLA;

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

//------------------------------------
/****       IMU PARAMETERS    ****/
//------------------------------------

float a_x, a_y, a_z, yaw, pitch, roll, a_abs, pos, vel, vel_0, pos_0, Rp, yaw_d;
//float bias_ax, bias_ay, bias_az;

long timer = 0; //general purpuse timer
long timer_old;
float G_Dt; // delay between two updates of the filter


void setup() {

  Serial.begin(9600);
  ss.begin(GPSBaud);
  JY901.attach(Serial);
  timer = 0;


}

void loop() {

  timer_old = timer;
  timer = millis();

  if (timer > timer_old)
  {
    G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
  }

  JY901.receiveSerialData();
  a_x = ((float)JY901.getAccX());
  a_y = ((float)JY901.getAccY());
  a_z = ((float)JY901.getAccZ());
  roll = ((float)JY901.getRoll());
  pitch = ((float)JY901.getPitch());
  yaw = -ToRad((((float)JY901.getYaw()) + 90)); // originalmente yaw es negativo de norte a este 
  //yaw = (float)JY901.getYaw();

//  float a_x_p = ((8.238905 * pow(10, -7)) * (pitch * pitch * pitch)) + ((9.846454 * pow(10, -6)) * (pitch * pitch)) - (0.017688 * pitch) - 0.003106;
//  float a_x_d = (a_x - a_x_p) - 0.0041; //0.0041 ruido a_x_d
//  float a_x_dx = a_x_d * cos(ToRad(pitch));
//  float a_x_dy = a_x_d * sin(ToRad(pitch));
//
//  float a_y_p = ((-8.026296 * pow(10, -7)) * (roll * roll * roll)) - ((1.018808 * pow(10, -6)) * (roll * roll)) + (0.017629 * roll) + 0.000581;
//  float a_y_d =  (a_y - a_y_p) - (-0.0011); //-0.0011 ruido a_y_d
//  float a_y_dx = a_y_d * cos(ToRad(roll));
//  float a_y_dy = a_y_d * sin(ToRad(roll));
//
//  a_abs = sqrt(a_x_d * a_x_d + a_y_d * a_y_d);
//  a_abs = a_abs * 9.8;


  StaticJsonDocument<200> doc;
  doc["yaw_t"] = yaw;
  
  serializeJson(doc, ss);
  delay(20);

//  Serial.println('<');// Serial.print(F(","));
//  Serial.print(yaw); Serial.print(F(","));  Serial.print(yaw_d); Serial.print(F(" "));Serial.println('>');

}
