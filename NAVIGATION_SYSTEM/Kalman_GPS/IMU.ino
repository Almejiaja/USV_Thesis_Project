//void get_IMU_data() {
//
//  JY901.receiveSerialData();
//
////  a_x = ((float)JY901.stcAcc.a[0] / 32768 * 16);
////  a_y = ((float)JY901.stcAcc.a[1] / 32768 * 16);
////  a_z = ((float)JY901.stcAcc.a[2] / 32768 * 16);
////  yaw = ((float)JY901.stcAngle.Angle[2] / 32768 * 180);
//  a_x = ((float)JY901.getAccX());
//  a_y = ((float)JY901.getAccY());
//  a_z = ((float)JY901.getAccZ());
//  yaw = ((float)JY901.getYaw());
//  a_abs = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
//  a_abs = abs(a_abs - 1); //se resta valor de la gravedad
//  Serial.println(a_abs);
//}
