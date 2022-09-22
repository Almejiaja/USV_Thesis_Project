#include <Servo.h>

Servo esc1, esc2;

char valor;
String estado;

int a = 1500;
float b = 1;

int min_esc = 1000;
int max_esc = 2000;

const byte numChars = 255;
char receivedChars[numChars];   // an array to store the received data

boolean start_save = false;
boolean newData = false;


void setup()
{
  esc1.attach(6, min_esc, max_esc);// motor izquierdo m1 al pin 5
  esc2.attach(7, min_esc, max_esc); // motor derecho m2 al pin 9
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
 

  if (start_save) {
    recvWithStartEndMarkers();
    showNewData();
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
        move_usv(a * b, a);
        Serial.println("Adelante");
        break;
      case 'b':
        move_usv(0, a);
        Serial.println("Izquierda"); //prende motor derecho pin 9

        break;
      case 'c':
        move_usv(a * b, 0);
        Serial.println("Derecha"); //prende motor izquierdo pin 9
        break;
      case 'd':
        parar();
        Serial.println("Parar"); //prende motor izquierdo pin 9
        break;

      case 'T':
        start_save = false;
        break;

      case 'S':
        start_save = true;
        break;

    }

  }

}


void m12() { //mover motores al valor que llegue por bt
  delay (10);
  while (Serial2.available()) {
    char c = Serial2.read();
    estado += c;
  }
  if (estado.length() > 0) {
    esc1.writeMicroseconds(estado.toInt() * b); //motor izquierdo (debe conectarse al 5)
    esc2.writeMicroseconds((estado.toInt())); //motor derecho (debe conectarse al 9)
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
    Serial.println(receivedChars);
    newData = false;
  }
}
