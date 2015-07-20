void disp_init(){
  Wire.begin();
  SeeedOled.init();  //initialze SEEED OLED display
  DDRB|=0x21;        
  PORTB |= 0x21;
  SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
  SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
  SeeedOled.setPageMode();           //Set addressing mode to Page Mode
  //  SeeedOled.setTextXY(0,0);          //Set the cursor to Xth Page, Yth Column  
  //  SeeedOled.putString("Hello World!"); //Print the String
}

void disp(struct statestruct *state){
  SeeedOled.setTextXY(0,0);          //Set the cursor to Xth Page, Yth Column  
  SeeedOled.putString("Theta: "); //Print the String
  SeeedOled.putNumber(state->theta/91);
  SeeedOled.putString("   ");
  SeeedOled.setTextXY(1,0);  
  SeeedOled.putString("Theta_c: ");  
  SeeedOled.putNumber(state->theta_cmd/91);
  SeeedOled.putString("   ");
  SeeedOled.setTextXY(2,0);          //Set the cursor to Xth Page, Yth Column  
  SeeedOled.putString("Phi: "); //Print the String
  SeeedOled.putNumber(state->phi/91);
  SeeedOled.putString("   ");
  SeeedOled.setTextXY(3,0);  
  SeeedOled.putString("Phi:c: ");  
  SeeedOled.putNumber(state->phi_cmd/91);
  SeeedOled.putString("   ");
//  SeeedOled.setTextXY(5,0);          //Set the cursor to Xth Page, Yth Column  
//  SeeedOled.putString("Psi: "); //Print the String
//  SeeedOled.putNumber(state->psi/16);
//  SeeedOled.putString("   ");
  SeeedOled.setTextXY(6,0);          //Set the cursor to Xth Page, Yth Column  
  SeeedOled.putString("t "); //Print the String
  SeeedOled.putNumber(state->t_test);
  SeeedOled.putString("us ");
}
