#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  
SoftwareSerial BT(10, 11);   // Definimos los pines RX y TX del Arduino conectados al Bluetooth
#include <Servo.h>

Servo esc1;
Servo esc2;

char valor;
String estado;

int a = 1500;
float b = 1;

int min_esc = 1000;
int max_esc = 2000;

void setup()
{
  esc1.attach(5, min_esc, max_esc);// motor izquierdo m1 al pin 5
  esc2.attach(9, min_esc, max_esc); // motor derecho m2 al pin 9
  delay(1000);
  BT.begin(9600);
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(0);
  Serial.begin(9600);   // Inicializamos  el puerto serie
}

void loop()
{
  if (BT.available())   // Si llega un dato por el puerto BT se envía al monitor serial
  {
    valor = BT.read();

    Serial.println(valor);
    switch (valor) {
      case 'A': //Mover los dos motores al tiempo a una potencia asignada desde app
        m12();
        break;
      case 'B': //
        m2();
        break;
      case 'a':
        move_usv(a * b, a);
        Serial.println("Adelante");
        Serial.println(a);
        Serial.println(a * b);
        break;
      case 'b':
        move_usv(0, 1800);
        Serial.println("Izquierda"); //prende motor derecho pin 9
        Serial.println(a);
        Serial.println(a * b);
        break;
      case 'c':
        move_usv(1800 * b, 0);
        Serial.println("Derecha"); //prende motor izquierdo pin 9
        Serial.println(1800);
        Serial.println(1800 * b);
        break;
      case 'd':
        parar();
        break;
      case 'P':
        zigzag(1000, 2);
        break;
    }

 
  }

}

void m1() {
  delay (30);
  while (BT.available()) {
    char c = BT.read();
    estado += c;
  }
  if (estado.length() > 0) {
    esc1.writeMicroseconds(estado.toInt());
    Serial.println(estado.toInt());
    estado = "";
  }

}

void m12() { //mover motores al valor que llegue por bt
  delay (30);
  while (BT.available()) {
    char c = BT.read();
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

void m2() {
  delay (30);
  while (BT.available()) {
    char c = BT.read();
    estado += c;
  }
  if (estado.length() > 0) {
    esc2.writeMicroseconds(estado.toInt());
    Serial.println(estado.toInt());
    estado = "";
  }

}

void parar() {
  esc1.writeMicroseconds(0);
  esc2.writeMicroseconds(0);
  Serial.println("Parar");
}

void move_usv(int m1, int m2) {
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
}

void zigzag(int t, int count) {

  for (int i = 0; i <= count; i++) {
    move_usv(1600, 0);
    delay(t);
    move_usv(1600, 1600);
    delay(t/2);
    move_usv(0, 1600);
    delay(t);
    move_usv(1600, 1600);
    delay(t/2);
  }
  parar();

}
