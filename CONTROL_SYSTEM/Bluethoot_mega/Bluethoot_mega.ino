#include <Servo.h>
#include <ArduinoJson.h>

#define PI 3.1415926535897932384626433832795

//Control LQR

float km11 = -1.3068, km12 = -2.2361, km21 = 1.3068, km22 = 2.2361, e, e1 = 0, u1, u2, u1_1, u2_1, m1, m2; //Parámetros LQR Matlab

// PWM motores
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

void setup()
{
  esc1.attach(6, min_esc, max_esc);// motor izquierdo m1 al pin 6
  esc2.attach(7, min_esc, max_esc); // motor derecho m2 al pin 7
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(0);
  Serial.begin(9600);   // Inicializamos  el puerto serie
  Serial1.begin(9600);   // Inicializamos  el puerto serie
  Serial2.begin(9600);   // Inicializamos  el puerto serie

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  dataBT();

  if (Serial1.available())
  {
    // Allocate the JSON document
    // This one must be bigger than the sender's because it must store the strings
    StaticJsonDocument<300> doc;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc, Serial1);

    if (err == DeserializationError::Ok)
    {

      LQR_control(doc["yaw_d"].as<float>(), doc["yaw_t"].as<float>());
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
      //      Serial.print("yaw_d = ");
      //      Serial.println(doc["dt"].as<float>());
      //      Serial.print("yaw_t = ");
      //      Serial.println(doc["yaw_t"].as<float>());

      Serial.print("m1 = ");
      Serial.print(m1);Serial.print(" , ");
      Serial.print("m2 = ");
      Serial.print(m2);Serial.print(" , ");

      Serial.print("yaw_d = ");
      Serial.print(doc["yaw_d"].as<float>()); Serial.print(" , ");
      Serial.print("yaw_t = ");
      Serial.println(doc["yaw_t"].as<float>());
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
  }


  //  recvWithStartEndMarkers();
  //  showNewData();

  if (start_save) {
    //      recvWithStartEndMarkers();
    //      showNewData();
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  //delay(20);
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


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
        Serial.println(receivedChars);
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    //Serial.println(receivedChars);
    newData = false;
  }
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

  //  if (u1 < 0) {
  //    m1 = 0;
  //  }
  //
  //  if (u2 < 0) {
  //    m2 = 0;
  //  }
  //
  //  if (u1 > 5) {
  //    m1 = 1752;
  //  }
  //
  //  if (u2 > 5) {
  //    m2 = 1752;
  //  }



}
