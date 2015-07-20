void elev(int16_t cmd){
  static uint16_t  n = 1500; //neutral
  static uint16_t  maxi = 2000; 
  static uint16_t  mini = 800;

  if (cmd == 0){
    elevator.writeMicroseconds(n);
  }
  else if (cmd > 0) {
    elevator.writeMicroseconds(map(cmd,1,32767,n+1,maxi));
  }
  else if (cmd < 0) {
    elevator.writeMicroseconds(map(cmd,-1,-32767,n-1,mini));
  }
}

void ail(int16_t cmd){
  static uint16_t  l_n = 1450; //neutral
  static uint16_t  r_n = 1650;
  static uint16_t  l_max = 2000; // minmax stroke
  static uint16_t  r_max = 2250;
  static uint16_t  r_min = 1050;
  static uint16_t  l_min = 800;

  if (cmd == 0){
    aileron_r.writeMicroseconds(r_n);
    aileron_l.writeMicroseconds(l_n);
  }
  else if (cmd > 0) {
    aileron_r.writeMicroseconds(map(cmd,1,32767,r_n+1,r_max));
    aileron_l.writeMicroseconds(map(cmd,1,32767,l_n+1,l_max));
  }
  else if (cmd < 0) {
    aileron_r.writeMicroseconds(map(cmd,-1,-32767,r_n-1,r_min));
    aileron_l.writeMicroseconds(map(cmd,-1,-32767,l_n-1,l_min));
  }
}

void rud(int16_t cmd){
  static uint16_t  n = 1450; //neutral
  static uint16_t  maxi = 2000; 
  static uint16_t  mini = 1050;

  if (cmd == 0){
    rudder.writeMicroseconds(n);
  }
  else if (cmd > 0) {
    rudder.writeMicroseconds(map(cmd,1,32767,n+1,maxi));
  }
  else if (cmd < 0) {
    rudder.writeMicroseconds(map(cmd,-1,-32767,n-1,mini));
  }
}

void thr(int16_t cmd){
  static uint16_t  n = 1000; //neutral
  static uint16_t  maxi = 2000; 
//  static uint16_t  mini = 1000;

  if (cmd == 0){
    throttle.writeMicroseconds(n);
  }
  else if (cmd > 0) {
    throttle.writeMicroseconds(map(cmd,1,32767,n+1,maxi));
  }
  else if (cmd < 0) {
    throttle.writeMicroseconds(n);
  }
}

void servo_update(){
      elev(state.elev);
      ail(state.ail);
      rud(state.rud);
      thr(state.throttle);
      // + die anderen später
}
