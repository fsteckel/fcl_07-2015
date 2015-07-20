bool UM7_listen(){
  static uint8_t buffer[1000];
  static uint8_t buflen = 0;
  static UM7_packet_struct UM7_packet;
  // Tempstates for Conversion UM7 Big to Arduino Little Endian
    static char theta_temp[2];
    static char phi_temp[2];
    static char psi_temp[2];
    static char theta_dot_temp[2];
    static char phi_dot_temp[2];
    static char psi_dot_temp[2];
  //+Tempstates
  
  buffer[buflen] = Serial3.read();
  if (buflen < 1000)
    buflen++;
  else
    buflen=0;

  if (buflen > 24){
    if (parse_serial_data(buffer, buflen, &UM7_packet) == 0){
//      state.i++;
      buflen=0;
      switch (UM7_packet.Address) {
      case 112: //Euler Paket
        theta_temp[0]     = UM7_packet.data[1];
        theta_temp[1]     = UM7_packet.data[0];
        phi_temp[0]       = UM7_packet.data[3];
        phi_temp[1]       = UM7_packet.data[2];
        psi_temp[0]       = UM7_packet.data[5];
        psi_temp[1]       = UM7_packet.data[4];
        theta_dot_temp[0] = UM7_packet.data[9];
        theta_dot_temp[1] = UM7_packet.data[8];
        phi_dot_temp[0]   = UM7_packet.data[11];
        phi_dot_temp[1]   = UM7_packet.data[10];
        psi_dot_temp[0]   = UM7_packet.data[13];
        psi_dot_temp[1]   = UM7_packet.data[12];        
        memcpy(&state.theta,      &theta_temp,2);
        memcpy(&state.phi,        &phi_temp,2);
        memcpy(&state.psi,        &psi_temp,2);
        memcpy(&state.theta_dot,  &theta_dot_temp,2);
        memcpy(&state.phi_dot,    &phi_dot_temp,2);
        memcpy(&state.psi_dot,    &psi_dot_temp,2);
        break;
        }
    }
  }
  if (buflen == 0)
    return true;
  else
    return false;
}


//UM7_packet_struct *UM7_packet; 

// parse_serial_data 
// This function parses the data in 'rx_data'  with length 'rx_length' and attempts to find a packet 
// in the data.  If a packet is found, the structure 'packet' is filled with the packet data. 
// If there is not enough data for a full packet in the provided array, parse_serial_data returns 1. 
// If there is enough data, but no packet header was found, parse_serial_data returns 2. 
// If a packet header was found, but there was insufficient data to parse the whole packet,  
// then parse_serial_data returns 3.  This could happen if not all of the serial data has been 
// received when parse_serial_data is called. 
// If a packet was received, but the checksum was bad, parse_serial_data returns 4. 
// If a good packet was received, parse_serial_data fills the UM7_packet structure and returns 0. 
uint8_t parse_serial_data(uint8_t *rx_data, uint8_t rx_length, struct UM7_packet_struct *packet) 
{ 
  uint8_t index;  
  // Make sure that the data buffer provided is long enough to contain a full packet 
  // The minimum packet length is 7 uint8_ts 
  if( rx_length < 7 ) 
  { 
    return 1; 
  }  
  // Try to find the 'snp' start sequence for the packet
  for(index = 0; index < (rx_length - 2); index++ ) 
  { 
    // Check for 'snp'.  If found, immediately exit the loop 
    if( rx_data[index] == 's' && rx_data[index+1] == 'n' && rx_data[index+2] == 'p' ) 
    { 
      break;
    } 
  }  
  uint8_t packet_index = index;  
  // Check to see if the variable 'packet_index' is equal to (rx_length - 2).  If it is, then the above 
  // loop executed to completion and never found a packet header. 
  if( packet_index == (rx_length - 2) ) 
  { 
    return 2; 
  }  
  // If we get here, a packet header was found.  Now check to see if we have enough room 
  // left in the buffer to contain a full packet.  Note that at this point, the variable 'packet_index'   
  // contains the location of the 's' character in the buffer (the first uint8_t in the header) 
  if( (rx_length - packet_index) < 7 ) 
  { 
    return 3; 
  }  
  // We've found a packet header, and there is enough space left in the buffer for at least 
  // the smallest allowable packet length (7 uint8_ts).  Pull out the packet type uint8_t to determine 
  // the actual length of this packet 
  uint8_t PT = rx_data[packet_index + 3]; 

  // Do some bit-level manipulation to determine if the packet contains data and if it is a batch 
  // We have to do this because the individual bits in the PT uint8_t specify the contents of the 
  // packet. 
  uint8_t packet_has_data = (PT >> 7) & 0x01;     // Check bit 7 (HAS_DATA) 
  uint8_t packet_is_batch = (PT >> 6) & 0x01;      // Check bit 6 (IS_BATCH) 
  uint8_t batch_length = (PT >> 2) & 0x0F;            // Extract the batch length (bits 2 through 5) 

  // Now finally figure out the actual packet length 
  uint8_t data_length = 0; 
  if( packet_has_data ) 
  { 
    if( packet_is_batch ) 
    { 
      // Packet has data and is a batch.  This means it contains 'batch_length' registers, each 
      // of which has a length of 4 uint8_ts 
      data_length = 4*batch_length; 
    } 
    else     // Packet has data but is not a batch.  This means it contains one register (4 uint8_ts) 
    { 
      data_length = 4; 
    } 
  } 
  else     // Packet has no data 
  { 
    data_length = 0; 
  }  
  // At this point, we know exactly how long the packet is.  Now we can check to make sure 
  // we have enough data for the full packet. 
  if( (rx_length - packet_index) < (data_length + 5) ) 
  { 
    return 3; 
  }  
  // If we get here, we know that we have a full packet in the buffer.  All that remains is to pull 
  // out the data and make sure the checksum is good. 
  // Start by extracting all the data 
  packet->Address = rx_data[packet_index + 4]; 
  packet->PT = PT; 

  // Get the data uint8_ts and compute the checksum all in one step 
  packet->data_length = data_length; 
  uint16_t computed_checksum = 's' + 'n' + 'p' + packet->PT + packet->Address; 
  for( index = 0; index < data_length; index++ ) 
  { 
    // Copy the data into the packet structure's data array 
    packet->data[index] = rx_data[packet_index + 5 + index]; 
    // Add the new uint8_t to the checksum 
    computed_checksum += packet->data[index]; 
  }  
  // Now see if our computed checksum matches the received checksum 
  // First extract the checksum from the packet 
  uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8); 
  received_checksum |= rx_data[packet_index + 6 + data_length];  
  // Now check to see if they don't match 
  if( received_checksum != computed_checksum ) 
  { 
    return 4; 
  }  
  // At this point, we've received a full packet with a good checksum.  It is already 
  // fully parsed and copied to the 'packet' structure, so return 0 to indicate that a packet was 
  // processed. 
  return 0; 
}
