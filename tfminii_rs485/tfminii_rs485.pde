/*
 * talk to tfmini-i RS485
 *
 * Datasheet 
 * https://cdn.robotshop.com/media/b/ben/rb-ben-22/pdf/benewake-tfmini-i-lidar-module-rs-485-can-12-m-datasheet.pdf
 * Manual
 * https://cdn.robotshop.com/media/b/ben/rb-ben-22/pdf/benewake-tfmini-i-lidar-module-rs-485-can-12-m-manual.pdf 
 * 
 */

import processing.serial.*;

boolean debug_print = true; 

String USBPORT = "COM12";
int baudrate = 115200;    // 115200 is default baud rate of tfmini-i 

Serial myPort;  
byte inByte = -1;    // Incoming serial data


int tfdist = 0;
int tfqual = 0;
int timestamp = 0;



// MODBUS MESSAGE / command
// [ id:1 ] [ function:1 ] [ register:2 ] [ readlength:2 ] [ crc:2 ] 
int POLL_FUNCTION = 3;  // 3 = read, 6 = write
int POLL_REGISTER = 0;   // 
int POLL_NUMREGISTERS = 2;   // 1 ... read distance, 2 ... read distance+strength
int POLL_ID = 2;          // default ID = 1

// MODBUS MESSAGE / reponse
// [ id:1 ] [ function:1 ] [ length:1 ] [ data:n ] [ crc:2 ]
int MSG_LENGTH = 9;    // expected length when polling for distance+strength
byte msg_buffer[];
int msg_pointer = 0;

PFont myFont;
PFont screenFont;
String incomingString = "";
String outgoingString = "";


int delayBetweenPolls = 50;  // in ms
int pollClock;
int responseTime = -1;
long askTime;
boolean retriggerPolling = true;
int retriggerPause = 1;
int outgoingPolls = 1;
int incomingMsgs = 1;
int msgErrorCount = 0;
boolean polling = true;

long timeSegment;
int timeSpanSeconds = 5;
int samplingRate = 0;


void setup() {
  size(900,350);
  myFont = createFont("Arial", 16);
  screenFont = createFont("Consolas", 16);
  textFont(myFont);
  
  printArray(Serial.list());
  myPort = new Serial(this, USBPORT, baudrate);
  
  // Sets the number of bytes to buffer before calling serialEvent()
  myPort.buffer(MSG_LENGTH);
  
  msg_buffer = new byte[MSG_LENGTH];
  
  askTime = millis();
  pollClock = delayBetweenPolls;
  
  timeSegment = millis();
  
  //setID(2);
  
  
  
  //byte b = -10;
  //println("byte "+b + "  "+hex(b,2));
  //println("signedInt: "+signedInt(b) + "   (int): "+(int)(b) + "  " + hex(signedInt(b),2));
  //int c = 130;
  //println("int "+c + "  "+hex(c,2));
  //println("unsignedByte: "+unsignedByte(c)+ "   (byte): "+(byte)(c) + "  " + "  " + hex(unsignedByte(c),2));
  
  
}


void draw() {
  
  background(0);
  fill(255);
  
  textFont(myFont);
  textSize(32);
  fill(255);
  text("TFmini-i polling  RS485", 10,50);
  text(tfdist+"cm", 10,120);
  text(tfqual+"quality", 310,120);
  //text(timestamp+"ms", 610,120);
  fill(0,200,150);
  rect(10,60,tfdist,20);
  
  
   ///////////////////////////////////////////////////////
  /////////////////////// draw debug info  //////////////
  ///////////////////////////////////////////////////////
  textSize(16);
  textFont(screenFont);
  int y = height-200;
  text("USB port:              "+USBPORT, 10, y+=20);
  text("baud-rate:             "+baudrate, 10, y+=20);
  text("POLL_ID:               "+POLL_ID, 10, y+=20);
  text("Last Sent:             " + outgoingString, 10, y+=20);
  text("Last Received:         " + incomingString, 10, y+=20);
  float responseRate = (float) incomingMsgs / (float) outgoingPolls;
  text("Response rate:         "+int(responseRate*100) + "%  ["+incomingMsgs+"/"+outgoingPolls+"]", 10, y+=20);
  text("Msg CRC error rate:    "+int(msgErrorCount), 10, y+=20);
  text("Sampling rate (Hz):    " + samplingRate, 10, y+=20);
  
  
  checkRepoll(false);
  
  sumResponses();

}



void sumResponses() {
  
  int timeSinceLast = int(millis()-timeSegment);
  
  if(timeSinceLast > timeSpanSeconds*1000) {
    //println("sumResponses");
    samplingRate = incomingMsgs/timeSpanSeconds;
    incomingMsgs = 0; 
    outgoingPolls = 0;
    msgErrorCount = 0;
    timeSegment = millis();
  }
  
}

void checkRepoll(boolean rightAway) {
  
  int timeSinceLast = int(millis()-askTime);
  int compareTo = pollClock;
  if(!rightAway) compareTo = delayBetweenPolls;

  if(polling && timeSinceLast > compareTo) {
    askForData();
    outgoingPolls++;
    pollClock = delayBetweenPolls - (timeSinceLast-pollClock);
    msg_pointer = 0; 
    incomingString = "";
  }
  
}



void serialEvent(Serial p) {
  while (p.available() > 0) {
    //println("available: "+p.available());
    readSerial();
  }
  //println("MSG:    "+incomingString);
  msg_decode(msg_pointer);
  
  if(msg_pointer>= MSG_LENGTH) {
    msg_pointer = 0;
  }
}

void readSerial() {
  
  inByte = (byte) myPort.read();
  //print("incoming: ");
  //print(inByte);
  //println();
  //incomingString+=hex(inByte,2)+"  ";
  msg_buffer[msg_pointer++] = inByte;
  
  
}


void msg_decode(int len) {
  
   if(debug_print) {
     print("msg_decode()    ");
     for(int i=0; i<len; i++) {
       print(hex(msg_buffer[i],2));
       print(" ");
     }
     println();
   }
   
   incomingString = "";
   for(int i=0; i<len; i++) {
     incomingString+=hex(msg_buffer[i],2);
     incomingString+="  ";
   }
   
   int id = msg_buffer[0];
   int functionID = msg_buffer[1]; 
   int data_length = msg_buffer[2];
   
   
   // CRC8 CHECK
   int expected_crc = crc16(msg_buffer, 3+data_length);
   byte highByte = (byte)(expected_crc>>8 & 0xFF);
   byte lowByte = (byte)(expected_crc & 0xFF);

   if (debug_print) {
     println("CRC expected is "+hex(highByte,2) + " " + hex(lowByte,2));
   }
   
   if (!(msg_buffer[len-2] == highByte && msg_buffer[len-1] == lowByte)) {
     if(debug_print) println("\t\tDECODE ERROR:   ("+id+") CRC error");
     msgErrorCount++;
     return;
   }
   
   if (id>15) {
    println("\t\tDECODE ERROR:   ("+id+") ID > 15");
    msgErrorCount++;
    return;
  }
  
   
  if (functionID == 3 && data_length <= 4) {
     // response to data request
     // could be software version or distance/signal
     tfdist = (int)((msg_buffer[3] & 0xFF)*256 + (msg_buffer[4] & 0xFF));
     tfqual = (int)((msg_buffer[5] & 0xFF)*256 + (msg_buffer[6] & 0xFF));
   }
  
  incomingMsgs++;
  return;

   
}



void setID(int _id) {
  
  byte[] write_request;
  write_request = new byte[8];
  
  write_request[0] = (byte)POLL_ID;   // 
  write_request[1] = 0x06;            // write
  write_request[2] = 0;
  write_request[3] = (byte) 0x85;     // ID setting
  write_request[4] = 0;
  write_request[5] = (byte)_id;
  
  sendData(write_request);
  // we don't listen to reponse to command
  delay(100);
  
  write_request[0] = (byte)POLL_ID;      // 
  write_request[1] = 0x06;              // write
  write_request[2] = 0;
  write_request[3] = (byte) 0x80;     // save settings
  write_request[4] = 0;
  write_request[5] = 0;
  
  sendData(write_request);
  // we don't listen to reponse to command
  delay(100);
  
  write_request[0] = (byte) POLL_ID;      // 
  write_request[1] = 0x06;              // write
  write_request[2] = 0;
  write_request[3] = (byte) 0x81;     // restart
  write_request[4] = 0;
  write_request[5] = 1;      
  
  sendData(write_request);
  // we don't listen to reponse to command
  
}




void askForData() {

  byte[] poll;
  poll = new byte[8];
  
  poll[0] = (byte)POLL_ID;        
  poll[1] = (byte)POLL_FUNCTION;  
  poll[2] = 0;
  poll[3] = (byte)POLL_REGISTER;
  poll[4] = 0;
  poll[5] = (byte)POLL_NUMREGISTERS; 
  //poll[6] = (byte)0xC8;  // 200
  //poll[7] = (byte)0x45;  // 69
  
  sendData(poll);
  
  askTime = millis();

}

void sendData(byte[] buffer) {
  
  // compute CRC check and add in last 2 buffer spots 
  int crc_number = crc16(buffer,6);
  byte highByte = (byte)(crc_number>>8 & 0xFF);
  byte lowByte = (byte)(crc_number & 0xFF);
  buffer[6] = highByte;
  buffer[7] = lowByte;
  //println("CRC returned: "+hex(crc_number,4)+ " : "+hex(highByte,2)+"-"+hex(lowByte,2));
  
  outgoingString = "";
  int i = 0;
  for(i=0; i<buffer.length; i++) {
    outgoingString+=hex(buffer[i],2)+"  ";
  }
  //for(i=0; i<poll.length; i++) {
  //  myPort.write(poll[i]);        // this doesn't work 
  //}
  myPort.write(buffer);
  
  
  if (debug_print) {
    print("outgoing:      ");
    for(i=0; i<buffer.length; i++) {
      print(hex(buffer[i],2));
      print(" ");
    }
    println();
  }
  
}


byte unsignedByte( int val ) { return (byte)( val > 127 ? val - 256 : val ); }

int signedInt( byte val ) { return (int)( val < 0 ? val + 256 : val ); }


int crc16(byte[] buffer, int buffer_length) {
  int i,j;
  int c;
  int CRC=0xFFFF;
  for(j = 0; j < buffer_length; j++) {
    c = signedInt(buffer[j]);
    CRC ^= c;    // bitwise xor assignment operator, 1 if either c or CRC-last-bit is not-zero
    for(i = 8; i != 0; i--){
      if ((CRC & 0x0001) != 0){
        CRC >>= 1;
        CRC ^= 0xA001;
      }
      else{
        CRC >>= 1; 
      }
    }
  }
  CRC = ((CRC & 0x00ff) << 8) | ((CRC & 0xff00) >> 8);
  return CRC;
}
