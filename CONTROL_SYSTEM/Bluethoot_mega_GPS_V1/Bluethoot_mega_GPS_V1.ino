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

float latt, latt_a, longi, longi_a, alt, vel_gps, h_dop, dist_final;
float xd = 0, ds, xp;

int sat;
//Coordenada ruta inicial


float p_in_l [3] = {6.240270, -74.899006, 782};
float p_fin_l [3] = {6.240517, -74.899006, 782};


//float p_in_l [3] = {6.188184, -74.992261, 1011}; //San Carlos
//float p_fin_l [3] = {6.187383, -74.991873, 1011};


//float p_in_l [3] = {6.187826, -74.993059, 1011}; //San Carlos
//float p_fin_l [3] = {6.186982, -74.992639, 1011};

float matriz_cuadrado[4][2] = {{1, 2}, {4, 5}, {7, 8}, {7, 8}};

//6.240527, -74.898846
//6.240620, -74.898688
//6.240475, -74.898573

//float la_o = 6.186987;
//float lo_o = -74.992626;
//float alt_o = 1011;
//
////Coordenada ruta Final
//
//float la_f = 6.187808;
//float lo_f = -74.993057;
//float alt_f = 1011;

//float la_o = 6.245662;
//float lo_o = -75.549241;
//float alt_o = 1620;

float ecef_a[3]; //coordenadas X, Y y Z de la posición actual
float ecef_o[3]; //coordenadas X, Y y Z de la posición inicial
float ecef_f[3]; //coordenadas X, Y y Z de la posición final
float ned_a[3]; //Posición NED GPS de la posición actual
float ned_o[3]; //Posición NED GPS de la posición inicial
float ned_f[3]; //Posición NED GPS de la posición final

float yaw_d, Rp = 8, yaw_t, diff;
//------------------------------------
/****       CONTROL LQR    ****/
//------------------------------------


//float km11 = -1.3068, km12 = -2.2361, km21 = 1.3068, km22 = 2.2361, e, e1 = 0, u1, u2, u1_1, u2_1, m1, m2; //Parámetros LQR Matlab

float km11 = 1.3068, km12 = 2.2361, km21 = -1.3068, km22 = -2.2361, e, e1 = 0, u1, u2, u1_1, u2_1, m1, m2; //Parámetros LQR Matlab motores m1 izquierda m2 derecha

//------------------------------------
/****       PWM MOTORS    ****/
//------------------------------------

Servo esc1, esc2;
char valor;
String estado;

int a = 1500;
//float b = 1.02;
float b = 1;

int min_esc = 1000;
int max_esc = 2000;

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

float dataNumber = 0;             // new for this version
boolean start_save = false;


char *strings[10]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;

//------------------------------------
/****       COMUNICATION SERIAL    ****/
//------------------------------------

//String data1;
bool rasp_data_a = false;
bool rasp_data_b = false;

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

  if (Serial1.available() > 0) {
    String data1 = Serial1.readStringUntil('\n');
    yaw_t = data1.toFloat();
    //    Serial.print("IMU sent me: ");
    //    Serial.println(data1);
  }

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data == "a") {
      rasp_data_a = true;
      rasp_data_b = false;
    } else if (data == "b") {
      rasp_data_b = true;
      rasp_data_a = false;
    }
    Serial.print("Rasp sent me: ");
    Serial.println(data);
  } else {
    rasp_data_b = false;
    rasp_data_a = false;
  }

  while (Serial3.available() > 0) { // GPS
    if (gps.encode(Serial3.read())) {
      latt = validGPS(gps.location.lat(), gps.location.isValid());
      longi = validGPS(gps.location.lng(), gps.location.isValid());
      alt = validGPS(gps.altitude.meters(), gps.altitude.isValid());
      vel_gps = validGPS(gps.speed.mps(), gps.speed.isValid());
      sat = validGPS(gps.satellites.value(), gps.satellites.isValid());
      h_dop = validGPS(gps.hdop.hdop(), gps.hdop.isValid());
      llt2ecef(ecef_a, latt, longi, alt); //Coordenadas XYZ de posición actual
      llt2ecef(ecef_o, p_in_l[0], p_in_l[1], p_in_l[2]); //Coordenadas XYZ de posición inicial
      llt2ecef(ecef_f, p_fin_l[0], p_fin_l[1], p_fin_l[2]); //Coordenadas XYZ de posición final
      ecef2ned(ned_a, p_in_l[0], p_in_l[1], ecef_o[0], ecef_o[1], ecef_o[2], ecef_a[0], ecef_a[1], ecef_a[2]);
      ecef2ned(ned_o, p_in_l[0], p_in_l[1], ecef_o[0], ecef_o[1], ecef_o[2], ecef_o[0], ecef_o[1], ecef_o[2]);
      ecef2ned(ned_f, p_in_l[0], p_in_l[1], ecef_o[0], ecef_o[1], ecef_o[2], ecef_f[0], ecef_f[1], ecef_f[2]);
      dist_final = calcularDistancia ( ned_a[1], ned_a[0], ned_f[1], ned_f[0]);
      yaw_d = LOS(ned_o, ned_f, ned_a, Rp);
      diff = yaw_t - yaw_d;
    }
  }

  if (newData) {
    if (dist_final > 3) {
      if (abs(diff) > 0.052) { //Si la diferencia de angulos es mayor a 3° corregir rumbo
        LQR_control(yaw_d, yaw_t);
        move_usv(m1, m2);
      } else {
        move_usv(1500, 1500);
      }
    } else {
      parar();
    }
  }

  if (rasp_data_a) {
    evasion();
  }
  if (rasp_data_b) {
    parar();
    delay(2000);
  }

  //Serial.print(m1); Serial.print(","); Serial.print(m2); Serial.print(","); Serial.print(newData); Serial.println(",");

  print_data();
  //  delay(5);
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

      case 'f':
        Serial.println("Activado modo automático");
        newData = true; // Modo automatico linea recta
        break;

      case 'g':
        newData = false;; // Modo automatico rectángulo
        Serial.println("Desactivado modo automático");
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
  m1 = 0;
  m2 = 0;
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
}

void move_usv(int m_1, int m_2) {
  m1 = m_1;
  m2 = m_2;
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
}

void LQR_control(float y_d, float y_t) {
  e = y_d - y_t;
  u1 = km11 * (e - e1) + km12 * (e - e1) + u1_1;
  u2 = km21 * (e - e1) + km22 * (e - e1) + u2_1;
  u1_1 = u1;
  u2_1 = u2;
  e1 = e;
  m1 = 139 * u1 + 1057.1; //PWM motor 1
  m2 = 139 * u2 + 1057.1; //PWM motor 2

  if (u1 < 0) {
    m1 = 0;
  }

  if (u2 < 0) {
    m2 = 0;
  }

  if (u1 > 5) {
    m1 = 1752;
  }

  if (u2 > 5) {
    m2 = 1752;
  }
}



void llt2ecef(float * ecef, float lattitud, float longitud, float high) {
  lattitud = ToRad(lattitud);
  longitud = ToRad(longitud);
  float vn = r_a / sqrt(1 - e_g * (sin(lattitud) * sin(lattitud))); //Elipsoide
  ecef[0] = (vn + high) * cos(lattitud) * cos(longitud);
  ecef[1] = (vn + high) * cos(lattitud) * sin(longitud);
  ecef[2] = (vn * (1 - (e_g * e_g)) + high) * sin(lattitud);

}

void ecef2ned(float * ned, float la1, float lo1, float x1, float y1, float z1, float x2, float y2, float z2) { //Combining High Rate GPS and Strong Motion Data: A Kalman Filter Formulation for Real-Time Displacement Waveforms
  la1 = ToRad(la1);
  lo1 = ToRad(lo1);
  float d_x = x2 - x1;
  float d_y = y2 - y1;
  float d_z = z2 - z1;
  ned[0] = (-sin(la1) * cos(lo1) * d_x) + (- sin(lo1) * sin(la1) * d_y) + (cos(la1) * d_z); //Norte (y)
  ned[1] = (-sin(lo1) * d_x) + (cos(lo1) * d_y); // Este (x)
  ned[2] = (cos(la1) * cos(lo1) * d_x) + (cos(la1) * sin(lo1) * d_y) + (sin(la1) * d_z); //Down
}

float LOS(float p_i[2], float p_f[2], float p_usv[2], int Rp)
{
  xp = atan2(p_f[1] - p_i[1], p_f[0] - p_i[0]); // Angulo de la ruta con Norte
  ds = -(p_usv[0] - p_i[0]) * sin(xp) + (p_usv[1] - p_i[1]) * cos(xp); // distacia USV ruta
  //Serial.println(ds);
  if (ds > Rp) {
    xd = xp - (PI / 2);

  } else if (ds < -Rp) {
    xd = xp + (PI / 2);

  } else {
    xd = xp - asin(ds / Rp);

  } return xd;
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

float calcularDistancia(float x1, float y1, float x2, float y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


void evasion() {
  Serial.println("Inicio evasion");
  print_data();
  m1 = 1500;
  m2 = 0;
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  delay (1500);
  print_data();
  m1 = 1500;
  m2 = 1500;
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  delay (1500);
  print_data();
  m1 = 0;
  m2 = 1500;
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  delay (1500);
  print_data();
  m1 = 1500;
  m2 = 1500;
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  delay (1500);
  print_data();
  Serial.println("Fin evasion");
}


void print_data() {

  Serial.print(yaw_t); Serial.print(","); 
  Serial.print(yaw_d); Serial.print(",");
  Serial.print(latt, 6); Serial.print(",");
  Serial.print(longi, 6); Serial.print(",");
  Serial.print(alt); Serial.print(",");
  Serial.print(vel_gps); Serial.print(",");
  Serial.print(m1); Serial.print(","); Serial.println(m2);
//  Serial.print(vel_gps); Serial.print(" , ");
//  Serial.print("hola"); Serial.print(" , ");
//  Serial.println(newData);


}
