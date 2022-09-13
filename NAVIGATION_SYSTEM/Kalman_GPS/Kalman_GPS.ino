#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman1.h>
#include <JY901.h>
#include <SD.h>
//const int chipSelect = 10;

static const int RXPin = 4, TXPin = 3; //Comunicación GPS
//static const uint32_t GPSBaud = 9600; //Velocidad GPS

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

using namespace BLA;

//------------------------------------
/****       KALMAN PARAMETERS    ****/
//------------------------------------

// Dimensions of the matrices
#define Nstate 2 // length of the state vector
#define Nobs 2   // length of the measurement vector
#define Ncom 1   // length of the control vector

// measurement std (to be characterized from your sensors) GPS
#define n1 3.13 // noise on the 1st measurement component
#define n2 0.1 // noise on the 2nd measurement component 

// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define m1 0.01
#define m2 0.02

KALMAN<Nstate, Nobs, Ncom> K; // your Kalman filter
BLA::Matrix<Nobs> obs; // observation vector

//------------------------------------
/****       IMU PARAMETERS    ****/
//------------------------------------

//float G_Dt = 0; // delay between two updates of the filter
float a_x, a_y, a_z, yaw, pitch, roll, a_abs, axx, ayy, azz;
float bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz;

//------------------------------------
/****       GPS PARAMETERS    ****/
//------------------------------------

float latt, longi, alt, bias_latt, bias_longi, vel_gps;
//Coordenada LLT inicial
float la_o = 6.235994;
float lo_o = -75.547704;
float alt_o = 1666;
float ecef_a[3]; //coordenadas X, Y y Z de la posición actual
float ecef_o[3]; //coordenadas X, Y y Z de la posición inicial
float ned[3];
//float ned2[3];
float diff[3];
float r_a = 6378137.0; //IERS  WGS-84 ellipsoid, semi-major axis (a) 6378137.0
float r_b = 6356752.3142; //IERS  WGS-84 ellipsoid, semi-minor axis (b) 6356752.3142
float f = 1 / 298.257223563; //WGS-84
float e = f * (2 - f); // WGS-84 [Transforming Cartesian coordinates X,Y,Z to Geographical coordinates φ, λ, h]
float vn = 0;
//float Rm = 6314197.7987; //radio meridional de la tierra https://cosasdeingenierossite.wordpress.com/2017/06/12/filtro-de-kalman-fusion-sensorial-de-acelerometros-y-gps/
//float Rp = 6356752.3142; //radio trasversal de la tierra


long timer = 0; //general purpuse timer
long timer_old;
float DT; // delay between two updates of the filter


void setup() {

  pinMode(2, OUTPUT); // sets the digital pin 2 as output
  Serial.begin(9600);
  JY901.attach(Serial);
  ss.begin(9600);
  // example of evolution matrix. Size is <Nstate,Nstate>
  K.F = {1.0, 0.0,
         0.0, 1.0
        };

  K.B = {1.0,
         0.0
        };
  // example of measurement matrix. Size is <Nobs,Nstate>
  K.H = {1.0, 0.0,
         0.0, 1.0
        };
  // example of measurement covariance matrix. Size is <Nobs,Nobs>
  K.R = {n1 * n1,   0.0,
         0.0, n2 * n2
        };
  // example of model covariance matrix. Size is <Nstate,Nstate>
  K.Q = {m1 * m1,   0.0,
         0.0, m2 * m2
        };
  digitalWrite(2, LOW);
  
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(10)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  delay(1000);

}

void loop() {

  String dataString = "";
  DT = timer - timer_old;
  timer_old = timer;
  K.F = {1.0,  0.0,
         0.0,  DT
        };

  K.B = {(DT * DT)*a_abs / 2,
         DT * a_abs
        };

  // APPLY KALMAN FILTER

  //save_measure();
  // PRINT RESULTS: measures and estimated state
  //Serial << obs << ' ' << K.x << '\n';
  get_IMU_data();
  get_GPS_data();
  K.update(obs);
  dataString = "ax  " + String(obs(0)) + " a_y "+String(obs(1));
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  timer = millis();

}


void get_GPS_data() {

  while (ss.available())
  {
    if (gps.encode(ss.read())) {
      latt = validGPS(gps.location.lat(), gps.location.isValid());
      longi = validGPS(gps.location.lng(), gps.location.isValid());
      alt = validGPS(gps.altitude.meters(), gps.altitude.isValid());
      vel_gps = validGPS(gps.speed.mps(), gps.speed.isValid());
      llt2ecef(ecef_a, latt, longi, alt);
      llt2ecef(ecef_o, la_o, lo_o, alt_o);
      ecef2ned(la_o, lo_o, ecef_o[0], ecef_o[1], ecef_o[2], ecef_a[0], ecef_a[1], ecef_a[2]); //
      //      ecef2ned2(latt, longi, alt, la_o, lo_o, alt_o);
      obs(0) = sqrt(ned[0] * ned[0] + ned[1] * ned[1]);
      obs(1) = vel_gps;
    }
  }
}

void llt2ecef(float * ecef, float lattitud, float longitud, float high) {
  lattitud = ToRad(lattitud);
  longitud = ToRad(longitud);

  //  ecef[0] = (a_g + high) * cos(lattitud) * cos(longitud); //Sin elipsoide (tierra redonda)
  //  ecef[1] = (a_g + high) * cos(lattitud) * sin(longitud);
  //  ecef[2] = (a_g + high) * sin(lattitud);
  vn = r_a / sqrt(1 - e * (sin(lattitud) * sin(lattitud))); //Elipsoide
  ecef[0] = (vn + high) * cos(lattitud) * cos(longitud);
  ecef[1] = (vn + high) * cos(lattitud) * sin(longitud);
  ecef[2] = (vn * (1 - (e * e)) + high) * sin(lattitud);
}

void ecef2ned(float la1, float lo1, float x1, float y1, float z1, float x2, float y2, float z2) { //Combining High Rate GPS and Strong Motion Data: A Kalman Filter Formulation for Real-Time Displacement Waveforms
  la1 = ToRad(la1);
  lo1 = ToRad(lo1);
  diff[0] = x2 - x1;
  diff[1] = y2 - y1;
  diff[2] = z2 - z1;
  ned[0] = (-sin(la1) * cos(lo1) * diff[0]) + (- sin(lo1) * sin(la1) * diff[1]) + (cos(la1) * diff[2]); //Norte (y)
  ned[1] = (-sin(lo1) * diff[0]) + (cos(lo1) * diff[1]); // Este (x)
  ned[2] = (cos(la1) * cos(lo1) * diff[0]) + (cos(la1) * sin(lo1) * diff[1]) + (sin(la1) * diff[2]); //Down
}

//void ecef2ned2(float la1, float lo1, float h1, float la0, float lo0, float h0) {
//
//  la1 = ToRad(la1);
//  lo1 = ToRad(lo1);
//  la0 = ToRad(la0);
//  lo0 = ToRad(lo0);
//  ned2[0] = (Rm + h1) * (la1 - la0); //Norte (y)
//  ned2[1] = (Rp + h1) * cos(la1) * (lo1 - lo0); //Este (x)
//  ned2[2] = -(h1 - h0);// Down (z)
//}

float validGPS(float val, bool valid)
{
  if (valid)
  {
    return  val;
  }
  else
  {
    return  0;
  }
}

void get_IMU_data() {

  JY901.receiveSerialData();

  //  a_x = ((float)JY901.stcAcc.a[0] / 32768 * 16);
  //  a_y = ((float)JY901.stcAcc.a[1] / 32768 * 16);
  //  a_z = ((float)JY901.stcAcc.a[2] / 32768 * 16);
  //  yaw = ((float)JY901.stcAngle.Angle[2] / 32768 * 180);
  a_x = ((float)JY901.getAccX());
  a_y = ((float)JY901.getAccY());
  a_z = ((float)JY901.getAccZ());
  yaw = ((float)JY901.getYaw());
  a_abs = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_abs = abs(a_abs - 1); //se resta valor de la gravedad
}

void save_measure()
{
  File myFile = SD.open("data.txt", FILE_WRITE);//abrimos  el archivo
  if (myFile) {
    digitalWrite(2, HIGH);


    myFile.print("ax= "); myFile.print(a_x); myFile.print(" ay= "); myFile.print(a_y); myFile.print(" az= "); myFile.println(a_z);
    myFile.print(" obs(0)= "); myFile.print(obs(0)); myFile.print(" obs(1)= "); myFile.println(obs(1));
    myFile.print("kx(0)= "); myFile.print(K.x(0)); myFile.print(" K.x(1)= "); myFile.println(K.x(1));
    myFile.close(); //cerramos el archivo
  } else {
    Serial.println("Error al abrir el archivo");
    digitalWrite(2, LOW);
  }
}
