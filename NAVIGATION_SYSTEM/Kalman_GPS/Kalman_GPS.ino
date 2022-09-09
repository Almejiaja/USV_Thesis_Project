#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman1.h>
#include <JY901.h>

static const int RXPin = 4, TXPin = 3; //Comunicación GPS
static const uint32_t GPSBaud = 9600; //Velocidad GPS

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

float G_Dt = 0; // delay between two updates of the filter
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
float ned2[3];
float diff[3];
float r_a = 6378137.0; //IERS  WGS-84 ellipsoid, semi-major axis (a) 6378137.0
float r_b = 6356752.3142; //IERS  WGS-84 ellipsoid, semi-minor axis (b) 6356752.3142
float f = 1/298.257223563; //WGS-84
float e = f*(2-f); // WGS-84 [Transforming Cartesian coordinates X,Y,Z to Geographical coordinates φ, λ, h]
float vn =0;
float Rm = 6314197.7987; //radio meridional de la tierra https://cosasdeingenierossite.wordpress.com/2017/06/12/filtro-de-kalman-fusion-sensorial-de-acelerometros-y-gps/
float Rp = 6356752.3142; //radio trasversal de la tierra


long timer = 0; //general purpuse timer
long timer_old;
float DT; // delay between two updates of the filter


void setup() {

  pinMode(2, OUTPUT); // sets the digital pin 2 as output
  Serial.begin(9600);
  ss.begin(GPSBaud);

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
  timer = millis();
}

void loop() {

  DT = timer - timer_old; 
   
  timer_old = timer;
  get_IMU_data();
  timer = millis();
  
  get_GPS_data();
  K.F = {1.0,  0.0,
         0.0,  G_Dt
        };
  K.B = {(G_Dt * G_Dt)*a_abs / 2,
         G_Dt * a_abs
        };
  // APPLY KALMAN FILTER
  K.update(obs);
  
  save_measure();
  // PRINT RESULTS: measures and estimated state
  Serial << obs << ' ' << K.x << '\n';


}
