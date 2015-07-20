#include <XBee.h>
#include <Servo.h>
#include <PID_v1.h>

// UM7 Kommunikation:
struct UM7_packet_struct {
  uint8_t Address;
  uint8_t PT;
  uint16_t Checksum;
  uint8_t data_length;
  uint8_t data[30];
};
uint8_t parse_serial_data(uint8_t *rx_data, uint8_t rx_length, struct UM7_packet_struct* packet);
bool UM7_listen();
//+UM7

// States Struct
struct statestruct {
  int16_t theta;
  int16_t theta_dot;
  int16_t theta_cmd;

  int16_t phi;
  int16_t phi_dot;
  int16_t phi_cmd;

  int16_t psi;
  int16_t psi_dot;
  int16_t hdg_cmd;

  int16_t elev;
  int16_t ail;
  int16_t rud;
  int16_t flaps;
  int16_t throttle;
  int16_t mode;
  int16_t i;
  unsigned long t_test;
};
statestruct state;
void stateint(struct statestruct *state)
{
  state->theta         = 0;
  state->theta_cmd     = 0;
  state->phi           = 0;
  state->phi_cmd       = 0;
  state->psi           = 0;
  state->theta_dot     = 0;
  state->phi_dot       = 0;
  state->psi_dot       = 0;
  state->elev          = 0;
  state->ail           = 0;
  state->rud           = 0;
  state->flaps         = 0;
  state->throttle      = 0;
  state->mode          = 0;
  state->i             = 0;
}
//+States

//XBEE
struct cmdstatestruct {
  char intro1; // +
  char intro2; // +
  char intro3; // +
  int8_t lat;
  int8_t lon;
  int8_t rud;
  uint8_t spd;
  int8_t flp;
  int8_t mode;
  int8_t i;
  int8_t sum;
};
cmdstatestruct cmd;
elapsedMillis time_x_recv = 0;
unsigned int time_servo_cut = 10000;

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
//+X

// Aktuatoren/Servos
Servo elevator;
Servo rudder;
Servo aileron_r;
Servo aileron_l;
Servo throttle;
void elev(int16_t);
void ail(int16_t);
void rud(int16_t);
void thr(int16_t);
void servo_update();
//+Akt

// PID
double cmd_p2e, Input_p2e, Input_d_p2e, Output_p2e;
PID p2e(&Input_p2e, &Input_d_p2e, &Output_p2e, &cmd_p2e, 8, 3, 0.15, REVERSE);
double cmd_b2a, Input_b2a, Input_d_b2a, Output_b2a;
PID b2a(&Input_b2a, &Input_d_b2a, &Output_b2a, &cmd_b2a, 2.5, 0, 0.05, REVERSE);
//double cmd_b2a, Input_b2a, Input_d_b2a;
//PID hdg2b(&Input_hdg2b, &Input_d_hdg2b, &cmd_b2a, &cmd_hdg2b,1,0,0, DIRECT); //Reverse testen!
//+PID

void setup()
{
  elevator.attach(3);
  aileron_r.attach(20);
  aileron_l.attach(4);
  rudder.attach(6);
  throttle.attach(5);
  stateint(&state)
  cmdlost(&cmd);
  elev(state.elev);
  ail(state.ail);
  rud(state.rud);
  thr(state.throttle);

  p2e.SetMode(AUTOMATIC);
  p2e.SetOutputLimits(-32767, 32767);
  p2e.SetSampleTime(10);
  cmd_p2e = 0;
  b2a.SetMode(AUTOMATIC);
  b2a.SetOutputLimits(-32767, 32767);
  b2a.SetSampleTime(10);
  cmd_b2a = 0;
  //  hdg2b.SetMode(AUTOMATIC);
  //  hdg2b.SetOutputLimits(-15*91,15*91);
  //  hdg2b.SetSampleTime(10);
  //  cmd_hdg2b = 0;
  Serial2.begin(9600); // XBEE
  xbee.begin(Serial2);
  Serial3.begin(115200); // IMU

  elev((int) - 30000);
  ail((int) - 30000);
  rud((int) - 30000);
  delay(1000);
  elev((int) 30000);
  ail((int) 30000);
  rud((int)  30000);
  delay(1000);
  elev((int) 0);
  ail((int) 0);
  rud((int) 0);
}

void loop()
{
  // UM7 Recieve
  if (Serial3.available() > 0) {
    if (UM7_listen() == true) {
      state.theta += (int16_t) 180 * 91;
      state.phi = (int16_t) -1 * state.phi;
      state.phi_dot = (int16_t) -1 * state.phi_dot;
      Input_p2e = (double) state.theta;
      Input_d_p2e = (double) state.theta_dot;
      Input_b2a = (double) state.phi;
      Input_d_b2a = (double) state.phi_dot;
      if (time_x_recv <= time_servo_cut) {
        cmd_p2e = state.theta_cmd;//(double) state.theta_cmd;
        cmd_b2a = state.phi_cmd;//(double) state.phi_cmd;
      }
      else {
        cmd_p2e = 0;//(double) state.theta_cmd;
        cmd_b2a = 0;//(double) state.phi_cmd;
      }
      //        Input_hdg2b = (double) state.psi;
      //        Input_d_hdg2b = (double) state.psi_dot;
      p2e.Compute();
      //        hdg2b.Compute();
      b2a.Compute();
      state.elev = (int16_t) Output_p2e;
      state.ail = (int16_t) Output_b2a;
      if (time_x_recv <= time_servo_cut) {
        state.rud = (map(cmd.rud, 127, -127, -32767, 32767) / 4) * 3 - state.ail / 4;
        state.throttle = map(cmd.spd, 0, 256, 1, 32767);
      }
      else {
        state.rud = (map(0, 127, -127, -32767, 32767) / 4) * 3 - state.ail / 4;
        state.throttle = map(0, 0, 256, 1, 32767);
      }

      servo_update();
      //
    }
  }
  //+UM7

  //XBEE
  process_xbee();
  //+XBEE
}
