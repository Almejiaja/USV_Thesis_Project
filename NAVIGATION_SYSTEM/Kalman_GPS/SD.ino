#include <SD.h>

File myFile;

void save_measure()
{
  myFile = SD.open("datalog.txt", FILE_WRITE);//abrimos  el archivo

  if (myFile) {
    digitalWrite(2, HIGH);
    myFile.print("Obs= ");
    myFile.print(obs(0));
    myFile.print("Kx= ");
    myFile.println(K.x(0));
    myFile.close(); //cerramos el archivo

  } else {
    Serial.println("Error al abrir el archivo");
    digitalWrite(2, LOW);
  }
}
