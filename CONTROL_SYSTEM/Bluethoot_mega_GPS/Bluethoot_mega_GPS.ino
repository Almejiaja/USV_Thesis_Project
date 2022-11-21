#include <Servo.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

#define PI 3.1415926535897932384626433832795

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
//------------------------------------
/****       GPS PARAMETERS    ****/
//------------------------------------
#define r_a  6378137.0 //IERS  WGS-84 ellipsoid, semi-major axis (a) 6378137.0
#define b_r  6356752.3142 //IERS  WGS-84 ellipsoid, semi-minor axis (b) 6356752.3142
#define e_g ((1 / 298.257223563) * (2 - (1 / 298.257223563))) // WGS-84 [Transforming Cartesian coordinates X,Y,Z to Geographical coordinates φ, λ, h]

float latt, latt_a, longi, longi_a, alt, vel_gps, h_dop;
float xd = 0, ds, xp;

double cour;
int sat;
//Coordenada ruta inicial

float la_o = 6.186987;
float lo_o = -74.992626;
float alt_o = 1011;

//Coordenada ruta Final

float la_f = 6.187808;
float lo_f = -74.993057;
float alt_f = 1011;

//float la_o = 6.245662;
//float lo_o = -75.549241;
//float alt_o = 1620;

float ecef_a[3]; //coordenadas X, Y y Z de la posición actual
float ecef_o[3]; //coordenadas X, Y y Z de la posición inicial
float ecef_f[3]; //coordenadas X, Y y Z de la posición final
float ned_a[3]; //Posición NED GPS de la posición actual
float ned_o[3]; //Posición NED GPS de la posición inicial
float ned_f[3]; //Posición NED GPS de la posición final

float yaw_d, Rp = 4;
//------------------------------------
/****       CONTROL LQR    ****/
//------------------------------------


//float km11 = -1.3068, km12 = -2.2361, km21 = 1.3068, km22 = 2.2361, e, e1 = 0, u1, u2, u1_1, u2_1, m1, m2; //Parámetros LQR Matlab

float km11 = 1.3068, km12 = 2.2361, km21 = -1.3068, km22 = -2.2361, e, e1 = 0, u1, u2, u1_1, u2_1, m1, m2; //Parámetros LQR Matlab

//------------------------------------
/****       PWM MOTORS    ****/
//------------------------------------

Servo esc1, esc2;
char valor;
String estado;

int a = 1500;
float b = 1.02;

int min_esc = 1000;
int max_esc = 2000;

const byte numChars = 255;
char receivedChars[numChars];   // an array to store the received data

boolean start_save = false;
boolean newData = false;

char *strings[10]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;

//------------------------------------
/****       COMUNICATION SERIAL    ****/
//------------------------------------

String input_rasp, control_mov_1, control_mov_2, control_mov_3;


void setup()
{
  esc1.attach(6, min_esc, max_esc);// motor izquierdo m1 al pin 6
  esc2.attach(7, min_esc, max_esc); // motor derecho m2 al pin 7
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(0);
  Serial.begin(9600);   // Inicializamos  el puerto serie
  Serial1.begin(9600);   // Inicializamos  el puerto serie NANO
  Serial2.begin(9600);   // Inicializamos  el puerto serie BLT
  Serial3.begin(9600);   // Inicializamos  el puerto serie GPS
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  dataBT();
  if (Serial1.available())
  {
    if (Serial.available())
    {
      input_rasp = Serial.readString();
    } else {
      input_rasp = "";
    }
    update_evasion(input_rasp);
    
    while (Serial3.available() > 0) {
      if (gps.encode(Serial3.read())) {
        latt = validGPS(gps.location.lat(), gps.location.isValid());
        longi = validGPS(gps.location.lng(), gps.location.isValid());
        alt = validGPS(gps.altitude.meters(), gps.altitude.isValid());
        vel_gps = validGPS(gps.speed.mps(), gps.speed.isValid());
        //cour = validGPS(gps.course.deg(), gps.course.isValid());
        cour = gps.courseTo(latt, longi, la_f, lo_f);
        sat = validGPS(gps.satellites.value(), gps.satellites.isValid());
        h_dop = validGPS(gps.hdop.hdop(), gps.hdop.isValid());
        llt2ecef(ecef_a, latt, longi, alt); //Coordenadas XYZ de posición actual
        llt2ecef(ecef_o, la_o, lo_o, alt_o); //Coordenadas XYZ de posición inicial
        llt2ecef(ecef_f, la_f, lo_f, alt_f); //Coordenadas XYZ de posición final
        ecef2ned(ned_a, la_o, lo_o, ecef_o[0], ecef_o[1], ecef_o[2], ecef_a[0], ecef_a[1], ecef_a[2]);
        ecef2ned(ned_o, la_o, lo_o, ecef_o[0], ecef_o[1], ecef_o[2], ecef_o[0], ecef_o[1], ecef_o[2]);
        ecef2ned(ned_f, la_o, lo_o, ecef_o[0], ecef_o[1], ecef_o[2], ecef_f[0], ecef_f[1], ecef_f[2]);
        yaw_d = LOS(ned_o, ned_f, ned_a, Rp);
        //Serial.println(yaw_d);

      }
    }
    // Allocate the JSON document
    // This one must be bigger than the sender's because it must store the strings
    StaticJsonDocument<200> doc;
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc, Serial1);
    if (err == DeserializationError::Ok)
    {
      LQR_control(yaw_d, doc["yaw_t"].as<float>());
      
      Serial.print(sat); Serial.print(" , ");
      Serial.print(h_dop, 4); Serial.print(" , ");
      Serial.print(ned_a[0]); Serial.print(" , ");
      // Serial.print("ned_a[1]= ");
      Serial.print(ned_a[1]); Serial.print(" , ");
      //Serial.print("ned_f[0] = ");
      Serial.print(ned_f[0]); Serial.print(" , ");
      //Serial.print("ned_f[1]= ");
      Serial.print(ned_f[1]); Serial.print(" , ");
      Serial.print(input_rasp); Serial.print(" , ");
      //Serial.print("ang= ");
      //      Serial.print(cour); Serial.print(" , ");
      //      Serial.print(ang, 4); Serial.print(" , ");
      //      //Serial.print("yaw_t = ");
      Serial.print(doc["yaw_t"].as<float>(), 4); Serial.print(" , ");
      //Serial.print("yaw_d = ");
      Serial.print(yaw_d, 4); Serial.print(" , ");
      //Serial.print("ds = ");
      Serial.print(ds); Serial.print(" , ");
      //Serial.print("xp = ");
      Serial.println(xp);
    }
    else
    {
      // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());

      // Flush all bytes in the "link" serial port buffer
      while (Serial1.available() > 0)
        Serial1.read();
    }
    //delay(500);

  }


  if (start_save) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }


}


void dataBT () {
  if (Serial2.available()) { // Si llega un dato por el puerto BT se envía al monitor serial
    valor = Serial2.read();
    Serial.println(valor);
    switch (valor) {
      case 'A': //Mover los dos motores al tiempo a una potencia asignada desde app
        m12();
        break;
      case 'a':
        move_usv(round(a * b), a);
        Serial.println("Adelante");
        break;
      case 'b':
        move_usv(0, round(a * b));
        Serial.println("Izquierda"); //prende motor derecho

        break;
      case 'c':
        move_usv(a, 0);
        Serial.println("Derecha"); //prende motor izquierdo
        break;
      case 'd':
        parar();
        Serial.println("Parar");
        break;

      case 'T':
        start_save = false;
        break;

      case 'S':
        start_save = true;
        break;

      case 'q':
        circulo(); //Prueba circular
        Serial.println("Circulo");
        break;

      case 'w':
        zigzag(a, round(a * b)); //ZigZag
        Serial.println("ZigZag");
        break;

      case 'e':
        cuadrado(); // Prueba rectángular
        Serial.println("Cuadrado");
        break;
    }
  }
}

void m12() { //mover motores al valor que llegue por bt para diferentes PWM
  delay (10);
  while (Serial2.available()) {
    char c = Serial2.read();
    estado += c;
  }
  if (estado.length() > 0) {
    esc1.writeMicroseconds(estado.toInt()); //motor izquierdo
    esc2.writeMicroseconds((round(estado.toInt()*b))); //motor derecho
    Serial.println(estado.toInt());
    Serial.println(estado.toInt() * b);
    estado = "";
  }
}


void parar() {
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(0);
}

void move_usv(int m1, int m2) {
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
}


void zigzag(int m1, int m2) {

  for (int i = 0; i <= 2 ; i++) {
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(round(1300));
    delay (2000);
    esc1.writeMicroseconds(round (1300));
    esc2.writeMicroseconds(1500);
    delay (2000);
  }
  parar();
}

void circulo() {
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(0);
  delay(5000);
  parar();
}


void cuadrado() {
  for (int i = 0; i <= 4; i++) {
    esc1.writeMicroseconds(a);
    esc2.writeMicroseconds(round (a * b));
    delay (1000);
    esc1.writeMicroseconds(2000);
    esc2.writeMicroseconds(0);
    delay (3000);
  }
  parar();
}


void update_evasion(String input)
{
  if (input == "a") {
    evasion();
  } else if (input == "b") {
    parar();
  }
}

void evasion() {
  esc1.writeMicroseconds(1500);
  esc2.writeMicroseconds(0);
  delay (1000);
  esc1.writeMicroseconds(1500);
  esc2.writeMicroseconds(1500);
  delay (2000);
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(1500);
  delay (1000);
  esc1.writeMicroseconds(1500);
  esc2.writeMicroseconds(1500);
  delay (2000);
}

void LQR_control(float yaw_d, float yaw_t) {
  e = yaw_d - yaw_t;

  u1 = km11 * (e - e1) + km12 * (e - e1) + u1_1;
  u2 = km21 * (e - e1) + km22 * (e - e1) + u2_1;
  u1_1 = u1;
  u2_1 = u2;
  e1 = e;
  m1 = 139 * u1 + 1057.1; //PWM motor 1
  m2 = 139 * u2 + 1057.1; //PWM motor 2

  //    if (u1 < 0) {
  //      m1 = 0;
  //    }
  //
  //    if (u2 < 0) {
  //      m2 = 0;
  //    }
  //
  //    if (u1 > 5) {
  //      m1 = 1752;
  //    }
  //
  //    if (u2 > 5) {
  //      m2 = 1752;
  //    }
}




float LOS(float p_i[2], float p_f[2], float p_usv[2], int Rp)
{

  xp = atan2(p_f[1] - p_i[1], p_f[0] - p_i[0]); // Angulo de la ruta con Norte
  ds = -(p_usv[0] - p_i[0]) * sin(xp) + (p_usv[1] - p_i[1]) * cos(xp); // distacia USV ruta

  if (ds > Rp) {
    xd = xp - (PI / 2);

  } else if (ds < -Rp) {
    xd = xp + (PI / 2);

  } else {
    xd = xp - asin(ds / Rp);

  } return xd;
}


void llt2ecef(float * ecef, float lattitud, float longitud, float high) {
  lattitud = ToRad(lattitud);
  longitud = ToRad(longitud);
  float vn = r_a / sqrt(1 - e_g * (sin(lattitud) * sin(lattitud))); //Elipsoide
  ecef[0] = (vn + high) * cos(lattitud) * cos(longitud);
  ecef[1] = (vn + high) * cos(lattitud) * sin(longitud);
  ecef[2] = (vn * (1 - (e_g * e_g)) + high) * sin(lattitud);

}

void ecef2ned(float* ned, float la1, float lo1, float x1, float y1, float z1, float x2, float y2, float z2) { //Combining High Rate GPS and Strong Motion Data: A Kalman Filter Formulation for Real-Time Displacement Waveforms
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
