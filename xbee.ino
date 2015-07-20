void cmdlost(struct cmdstatestruct *cmd)
{
  cmd->intro1 = '+';
  cmd->intro2 = '+';
  cmd->intro2 = '+';
  cmd->lat     = 0;
  cmd->lon       = 0;
  cmd->rud      = 0;
  cmd->spd         = 0;
  cmd->flp      = 1;
  cmd->mode          = 0;
  cmd->i             = 0;
  cmd->sum	= 1;
}

void process_xbee() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {// got something
      
      
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
        // got a zb rx packet
        
        // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
            
        if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
            // the sender got an ACK
            flashLed(statusLed, 10, 10);
        } else {
            // we got it (obviously) but sender didn't get an ACK
            flashLed(errorLed, 2, 20);
        }
        // set dataLed PWM to value of the first byte in the data
        analogWrite(dataLed, rx.getData(0));
      } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
        xbee.getResponse().getModemStatusResponse(msr);
        // the local XBee sends this response on certain events, like association/dissociation
        
        if (msr.getStatus() == ASSOCIATED) {
          // yay this is great.  flash led
          flashLed(statusLed, 10, 10);
        } else if (msr.getStatus() == DISASSOCIATED) {
          // this is awful.. flash led to show our discontent
          flashLed(errorLed, 10, 10);
        } else {
          // another status
          flashLed(statusLed, 5, 10);
        }
      } else {
      	// not something we were expecting
        flashLed(errorLed, 1, 25);    
      }
    } else if (xbee.getResponse().isError()) {
      //nss.print("Error reading packet.  Error code: ");  
      //nss.println(xbee.getResponse().getErrorCode());
    }
  
  
  
  
  
  
  
  
  
  
  
  
  static int i_buf_xbee_rec = 0;
  static int msgsize = sizeof(struct cmdstatestruct);
  static char buf_xbee_rec[sizeof(struct cmdstatestruct)];
  buf_xbee_rec[i_buf_xbee_rec] = Serial2.read();
  i_buf_xbee_rec++;
  if (i_buf_xbee_rec >= msgsize) {
    i_buf_xbee_rec = 0;
    if (buf_xbee_rec[0] == '+') {
      if (buf_xbee_rec[2] == '+') {
        if ((int8_t) buf_xbee_rec[10] == (int8_t) (buf_xbee_rec[3] + buf_xbee_rec[4] + buf_xbee_rec[5] + buf_xbee_rec[6] + buf_xbee_rec[7] + buf_xbee_rec[8] + buf_xbee_rec[9])) {
          memcpy(&cmd, buf_xbee_rec, sizeof(buf_xbee_rec));
          state.theta_cmd = map(cmd.lat,127,-127,-15*91,15*91);
          state.phi_cmd = map(cmd.lon,127,-127,30*91,-30*91);
          time_x_recv = 0;        
        }
      }
    }
  }
}
