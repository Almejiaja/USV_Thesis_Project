//void get_GPS_data() {
//
//  while (ss.available())
//  {
//    if (gps.encode(ss.read())) {
//      latt = validGPS(gps.location.lat(), gps.location.isValid());
//      longi = validGPS(gps.location.lng(), gps.location.isValid());
//      alt = validGPS(gps.altitude.meters(), gps.altitude.isValid());
//      vel_gps = validGPS(gps.speed.mps(), gps.speed.isValid());
//      llt2ecef(ecef_a, latt, longi, alt);
//      llt2ecef(ecef_o, la_o, lo_o, alt_o);
//      ecef2ned(la_o, lo_o, ecef_o[0], ecef_o[1], ecef_o[2], ecef_a[0], ecef_a[1], ecef_a[2]); //
////      ecef2ned2(latt, longi, alt, la_o, lo_o, alt_o);
//      obs(0) = sqrt(ned[0] * ned[0] + ned[1] * ned[1]);
//      obs(1) = vel_gps;
//    }
//  }
//}
//
//void llt2ecef(float * ecef, float lattitud, float longitud, float high) {
//  lattitud = ToRad(lattitud);
//  longitud = ToRad(longitud);
//  
////  ecef[0] = (a_g + high) * cos(lattitud) * cos(longitud); //Sin elipsoide (tierra redonda)
////  ecef[1] = (a_g + high) * cos(lattitud) * sin(longitud);
////  ecef[2] = (a_g + high) * sin(lattitud);
//  vn = r_a / sqrt(1 - e * (sin(lattitud) * sin(lattitud))); //Elipsoide
//  ecef[0] = (vn + high) * cos(lattitud) * cos(longitud);
//  ecef[1] = (vn + high) * cos(lattitud) * sin(longitud);
//  ecef[2] = (vn * (1 - (e * e)) + high) * sin(lattitud);
//}
//
//void ecef2ned(float la1, float lo1, float x1, float y1, float z1, float x2, float y2, float z2) { //Combining High Rate GPS and Strong Motion Data: A Kalman Filter Formulation for Real-Time Displacement Waveforms
//  la1 = ToRad(la1);
//  lo1 = ToRad(lo1);
//  diff[0] = x2 - x1;
//  diff[1] = y2 - y1;
//  diff[2] = z2 - z1;
//  ned[0] = (-sin(la1) * cos(lo1) * diff[0]) + (- sin(lo1) * sin(la1) * diff[1]) + (cos(la1) * diff[2]); //Norte (y)
//  ned[1] = (-sin(lo1) * diff[0]) + (cos(lo1) * diff[1]); // Este (x)
//  ned[2] = (cos(la1) * cos(lo1) * diff[0]) + (cos(la1) * sin(lo1) * diff[1]) + (sin(la1) * diff[2]); //Down
//}
//
////void ecef2ned2(float la1, float lo1, float h1, float la0, float lo0, float h0) {
////
////  la1 = ToRad(la1);
////  lo1 = ToRad(lo1);
////  la0 = ToRad(la0);
////  lo0 = ToRad(lo0);
////  ned2[0] = (Rm + h1) * (la1 - la0); //Norte (y)
////  ned2[1] = (Rp + h1) * cos(la1) * (lo1 - lo0); //Este (x)
////  ned2[2] = -(h1 - h0);// Down (z)
////}
//
//float validGPS(float val, bool valid)
//{
//  if (valid)
//  {
//    return  val;
//  }
//  else
//  {
//    return  0;
//  }
//}
