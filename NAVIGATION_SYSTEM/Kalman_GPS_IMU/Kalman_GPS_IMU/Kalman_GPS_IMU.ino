#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Kalman1.h>
#include <SD.h>
#include <SPI.h>
#include <JY901.h>

static const int RXPin = 4, TXPin = 3; //Comunicación GPS
static const uint32_t GPSBaud = 9600; //Velocidad GPS


// The TinyGPSPlus object
TinyGPSPlus gps;


// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

using namespace BLA;

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

//------------------------------------
/****       KALMAN PARAMETERS    ****/
//------------------------------------

// Dimensions of the matrices
#define Nstate 2 // length of the state vector
#define Nobs 2   // length of the measurement vector
#define Ncom 1   // length of the control vector

// measurement std (to be characterized from your sensors) GPS
#define n1 1.1 // noise on the 1st measurement component
#define n2 0.01 // noise on the 2nd measurement component 

// model std (~1/inertia). Freedom you give to relieve your evolution equation
//#define m1 0.01
//#define m2 0.02

#define m1 0.03
#define m2 0.03

#define r_a  6378137.0 //IERS  WGS-84 ellipsoid, semi-major axis (a) 6378137.0
#define r_b  6356752.3142 //IERS  WGS-84 ellipsoid, semi-minor axis (b) 6356752.3142
#define e ((1 / 298.257223563) * (2 - (1 / 298.257223563))) // WGS-84 [Transforming Cartesian coordinates X,Y,Z to Geographical coordinates φ, λ, h]

KALMAN<Nstate, Nobs, Ncom> K; // your Kalman filter
BLA::Matrix<Nobs> obs; // observation vector

//------------------------------------
/****       IMU PARAMETERS    ****/
//------------------------------------

float a_x, a_y, a_z, yaw, pitch, roll, a_abs, pos, vel, vel_0, pos_0;
//float bias_ax, bias_ay, bias_az;

//------------------------------------
/****       GPS PARAMETERS    ****/
//------------------------------------

float latt, longi, alt, vel_gps;
//Coordenada LLT inicial

float la_o = 6.236123;
float lo_o = -75.547935;
float alt_o = 1656;

float ecef_a[3]; //coordenadas X, Y y Z de la posición actual
float ecef_o[3]; //coordenadas X, Y y Z de la posición inicial
float ned[3]; //Posición NED GPS

long timer = 0; //general purpuse timer
long timer_old;
float G_Dt; // delay between two updates of the filter



void setup() {

  Serial.begin(9600);
  ss.begin(GPSBaud);
  JY901.attach(Serial);

  //  for (int i = 0; i < 5000; i++) // We take some readings...
  //  {
  //    JY901.receiveSerialData();
  //    bias_ax += ((float)JY901.getAccX());
  //    bias_ay += ((float)JY901.getAccY());
  //    bias_az += ((float)JY901.getAccZ());
  //    delay(10);
  //  }
  //
  //  bias_ax = bias_ax / 5000;
  //  bias_ay = bias_ay / 5000;
  //  bias_az = bias_az / 5000;

  //Serial.println(bias_ax, 3); Serial.println(bias_ay, 3); Serial.println(bias_az, 3);

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

  delay(5000);
  timer = millis();

}

void loop() {

  timer_old = timer;
  timer = millis();

  if (timer > timer_old)
  {
    G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)

    if (G_Dt > 0.2)
      G_Dt = 0; // ignore integration times over 100 ms
  }
  else {
    G_Dt = 0;
  }

  JY901.receiveSerialData();
  //    a_x = ((float)JY901.stcAcc.a[0] / 32768 * 16);
  //    a_y = ((float)JY901.stcAcc.a[1] / 32768 * 16);
  //    a_z = ((float)JY901.stcAcc.a[2] / 32768 * 16);
  //    roll =  ((float)JY901.stcAngle.Angle[0] / 32768 * 180);
  //    pitch = ((float)JY901.stcAngle.Angle[1] / 32768 * 180);
  //    yaw = ((float)JY901.stcAngle.Angle[2] / 32768 * 180);

  a_x = ((float)JY901.getAccX());
  a_y = ((float)JY901.getAccY());
  a_z = ((float)JY901.getAccZ());
  roll = ((float)JY901.getRoll());
  pitch = ((float)JY901.getPitch());
  yaw = ((float)JY901.getYaw());

  float a_x_p = ((8.238905 * pow(10, -7)) * (pitch * pitch * pitch)) + ((9.846454 * pow(10, -6)) * (pitch * pitch)) - (0.017688 * pitch);
  float a_x_d = (a_x - 0.003106 ) - a_x_p;
  float a_x_dx = a_x_d * cos(ToRad(pitch));
  float a_x_dy = a_x_d * sin(ToRad(pitch));


  float a_y_p = ((-8.026296 * pow(10, -7)) * (roll * roll * roll)) - ((1.018808 * pow(10, -6)) * (roll * roll)) + (0.017629 * roll);
  float a_y_d =  (a_y - 0.000581) - a_y_p;
  float a_y_dx = a_y_d * cos(ToRad(roll));
  float a_y_dy = a_y_d * sin(ToRad(roll));

  //    a_abs = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  //    a_abs = abs(a_abs - 1.055697); //se resta valor de la gravedad

  a_abs = sqrt(a_x_d * a_x_d + a_y_d * a_y_d);
  a_abs = a_abs * 9.8;


  if (a_abs > 0.4) { //https://sci-hub.se/10.1109/tencon.2017.8227906
    vel = vel_0 + (a_abs * G_Dt);
    pos = pos_0 + vel_0 * G_Dt + ((G_Dt * G_Dt) * 0.5 * a_abs); //Vector de posicion medido con el IMU
    vel_0 = vel;
    pos_0 = pos;
  } else {
    vel = 0;
  }

  K.F = {1.0,  0.0,
         0.0,  G_Dt
        };

  K.B = {(G_Dt * G_Dt)*a_abs / 2,
         G_Dt * a_abs
        };


  K.update(obs);

  ss.listen();

  while (ss.available() > 0) {

    if (gps.encode(ss.read())) {

      latt = validGPS(gps.location.lat(), gps.location.isValid());
      longi = validGPS(gps.location.lng(), gps.location.isValid());
      alt = validGPS(gps.altitude.meters(), gps.altitude.isValid());
      vel_gps = validGPS(gps.speed.mps(), gps.speed.isValid());
      llt2ecef(ecef_a, latt, longi, alt);
      llt2ecef(ecef_o, la_o, lo_o, alt_o);
      ecef2ned(la_o, lo_o, ecef_o[0], ecef_o[1], ecef_o[2], ecef_a[0], ecef_a[1], ecef_a[2]);
      obs(0) = sqrt(ned[0] * ned[0] + ned[1] * ned[1]);
      obs(1) = vel_gps;

    }
  }

  Serial.print('<');

  Serial.print(a_x, 3); Serial.print(F(",")); Serial.print(a_x_p, 3);  Serial.print(F(",")); Serial.print(a_x_d, 3); Serial.print(F(","));
  Serial.print(pitch); Serial.print(F(","));

  Serial.print(a_y, 3); Serial.print(F(",")); Serial.print(a_y_p, 3);  Serial.print(F(",")); Serial.print(a_y_d, 3); Serial.print(F(","));
  Serial.print(roll); Serial.print(F(","));

  Serial.print(a_abs, 4); Serial.print(F(",")); Serial.print(pos);  Serial.print(F(",")); Serial.print(vel); Serial.print(F(","));
  Serial.print(yaw); Serial.print(F(","));  Serial.print(G_Dt, 4); Serial.print(F(","));

  Serial.print(latt, 6); Serial.print(F(",")); Serial.print(longi, 6);  Serial.print(F(",")); Serial.print(ned[0]); Serial.print(F(","));
  Serial.print(ned[1]); Serial.print(F(",")); Serial.print(obs(0)); Serial.print(F(",")); Serial.print(obs(1));
  Serial.print(F(",")); Serial.print(K.x(0)); Serial.print(F(",")); Serial.print(K.x(1));

  Serial.println('>');


}

void llt2ecef(float * ecef, float lattitud, float longitud, float high) {
  lattitud = ToRad(lattitud);
  longitud = ToRad(longitud);
  float vn = r_a / sqrt(1 - e * (sin(lattitud) * sin(lattitud))); //Elipsoide
  ecef[0] = (vn + high) * cos(lattitud) * cos(longitud);
  ecef[1] = (vn + high) * cos(lattitud) * sin(longitud);
  ecef[2] = (vn * (1 - (e * e)) + high) * sin(lattitud);

}

void ecef2ned(float la1, float lo1, float x1, float y1, float z1, float x2, float y2, float z2) { //Combining High Rate GPS and Strong Motion Data: A Kalman Filter Formulation for Real-Time Displacement Waveforms
  la1 = ToRad(la1);
  lo1 = ToRad(lo1);
  float d_x = x2 - x1;
  float d_y = y2 - y1;
  float d_z = z2 - z1;
  ned[0] = (-sin(la1) * cos(lo1) * d_x) + (- sin(lo1) * sin(la1) * d_y) + (cos(la1) * d_z); //Norte (y)
  ned[1] = (-sin(lo1) * d_x) + (cos(lo1) * d_y); // Este (x)
  ned[2] = (cos(la1) * cos(lo1) * d_x) + (cos(la1) * sin(lo1) * d_y) + (sin(la1) * d_z); //Down
}

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
