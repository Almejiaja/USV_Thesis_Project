import serial, time

arduino = serial.Serial('/dev/ttyACM0', 9600)

while True:
  cadena = arduino.readline()
  
  if(cadena.decode() != ''):
    print(cadena.decode())
  
  time.sleep(1)

arduino.close()