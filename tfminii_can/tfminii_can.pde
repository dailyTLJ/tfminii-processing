/*
 * talk to tfmini-i CAN
 * 
 * Datasheet 
 * https://cdn.robotshop.com/media/b/ben/rb-ben-22/pdf/benewake-tfmini-i-lidar-module-rs-485-can-12-m-datasheet.pdf
 * Manual
 * https://cdn.robotshop.com/media/b/ben/rb-ben-22/pdf/benewake-tfmini-i-lidar-module-rs-485-can-12-m-manual.pdf 
 * 
 */

import processing.serial.*;

boolean debug_print = true; 

String USBPORT = "COM7";
int baudrate = 2000000;   // 2000000 default CAN baud rate 

Serial myPort;  
byte inByte = -1;    // Incoming serial data

int distanceData = 0;


int tfdist = 0;
int tfqual = 0;
int timestamp = 0;



// MODBUS MESSAGE / request
// [ id:1 ] [ function:1 ] [ register:2 ] [ readlength:2 ] [ crc:2 ] 
int POLL_FUNCTION = 3;  // 3 = read, 6 = write
int POLL_REGISTER = 52;   // 0x34 = 52 = measurement distance
int POLL_ID = 3;          // defauylt: 0x50 = 80

// MODBUS MESSAGE / answer
// [ id:1 ] [ function:1 ] [ length:1 ] [ data:n ] [ crc:2 ]
int MSG_LENGTH = 13;    // expected length when polling for distance
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
boolean retriggerPolling = false;
int retriggerPause = 1;
int outgoingPolls = 1;
int incomingMsgs = 1;
int msgErrorCount = 0;
boolean polling = false;

long timeSegment;
int timeSpanSeconds = 2;
int samplingRate = 0;


void setup() {
  size(900,300);
  myFont = createFont("Arial", 16);
  screenFont = createFont("Consolas", 16);
  textFont(myFont);
  
  printArray(Serial.list());
  myPort = new Serial(this, USBPORT, baudrate);
  
  // Sets the number of bytes to buffer before calling serialEvent()
  myPort.buffer(MSG_LENGTH);
  
  msg_buffer = new byte[MSG_LENGTH];
  
  //askTime = millis();
  //pollClock = delayBetweenPolls;
  
  timeSegment = millis();
  
  //setID(2);
  
}


void draw() {
  
  background(0);
  fill(255);
  
  textFont(myFont);
  textSize(32);
  fill(255);
  text("TFmini-i polling  CAN", 10,50);
  text(tfdist+"cm", 10,120);
  text(tfqual+"quality", 310,120);
  text(timestamp+"ms", 610,120);
  fill(0,150,200);
  rect(10,60,tfdist,20);
  
  
   ///////////////////////////////////////////////////////
  /////////////////////// draw debug info  //////////////
  ///////////////////////////////////////////////////////
  textSize(16);
  textFont(screenFont);
  int y = height-150;
  text("USB port:              "+USBPORT, 10, y+=20);
  text("baud-rate:             "+baudrate, 10, y+=20);
  text("POLL_ID:               "+POLL_ID, 10, y+=20);
  text("Last Sent:             " + outgoingString, 10, y+=20);
  text("Last Received:         " + incomingString, 10, y+=20);
  //float responseRate = (float) incomingMsgs / (float) outgoingPolls;
  //text("Response rate:         "+int(responseRate*100) + "%  ["+incomingMsgs+"/"+outgoingPolls+"]", 10, y+=20);
  //text("Msg CRC error rate:    "+int(msgErrorCount), 10, y+=20);
  text("Sampling rate (Hz):    " + samplingRate, 10, y+=20);
  
  
  //checkRepoll(false);
  
  sumResponses();

}

void sumResponses() {
  
  int timeSinceLast = int(millis()-timeSegment);
  

  if(timeSinceLast > timeSpanSeconds*1000) {
    //println("sumResponses");
    samplingRate = incomingMsgs/timeSpanSeconds;
    incomingMsgs = 0; 
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
   
   tfdist = (int)((msg_buffer[5] & 0xFF)*256 + (msg_buffer[4] & 0xFF));
   tfqual = (int)((msg_buffer[7] & 0xFF)*256 + (msg_buffer[6] & 0xFF));
   timestamp = (int)((msg_buffer[11] & 0xFF)*256*256*256 + (msg_buffer[10] & 0xFF)*256*256  + (msg_buffer[9] & 0xFF)*256 + (msg_buffer[8] & 0xFF));
   
   incomingString = "";
   for(int i=0; i<len; i++) {
     incomingString+=hex(msg_buffer[i],2);
     incomingString+="  ";
   }
   
   incomingMsgs++;
 
}



void setID(int _id) {
  // MODADDR 06 00 1a 00 MODADDRL CRCH CRCL
  
  byte[] write_request;
  write_request = new byte[8];
  
  write_request[0] = (byte)POLL_ID;        // 0x50
  write_request[1] = 0x06;      // write
  write_request[2] = 0;
  write_request[3] = 0x1A;     // ID setting
  write_request[4] = 0;
  write_request[5] = (byte)_id;
  
  int crc_number = crc16(write_request,6);
  byte highByte = (byte)(crc_number>>8 & 0xFF);
  byte lowByte = (byte)(crc_number & 0xFF);
  write_request[6] = highByte;
  write_request[7] = lowByte;
  //println("CRC returned: "+hex(crc_number,4)+ " : "+hex(highByte,2)+"-"+hex(lowByte,2));
  
  outgoingString = "";
  int i = 0;
  for(i=0; i<write_request.length; i++) {
    outgoingString+=hex(write_request[i],2)+"  ";
  }
  //for(i=0; i<poll.length; i++) {
  //  myPort.write(poll[i]);        // this doesn't work 
  //}
  myPort.write(write_request);
  
  
  if (debug_print) {
    print("CHANGE ID FROM "+POLL_ID+" TO "+_id+"    ");
    for(i=0; i<write_request.length; i++) {
      print(hex(write_request[i],2));
      print(" ");
    }
    println();
  }
  
}




void askForData() {

  byte[] poll;
  poll = new byte[8];
  
  poll[0] = (byte)POLL_ID;        // 0x50
  poll[1] = (byte)POLL_FUNCTION;  // 0x03
  poll[2] = 0;
  poll[3] = (byte)POLL_REGISTER;  // 0x34
  poll[4] = 0;
  poll[5] = 1;
  poll[6] = (byte)0xC8;  // 200
  poll[7] = (byte)0x45;  // 69
  
  int crc_number = crc16(poll,6);
  byte highByte = (byte)(crc_number>>8 & 0xFF);
  byte lowByte = (byte)(crc_number & 0xFF);
  poll[6] = highByte;
  poll[7] = lowByte;
  //println("CRC returned: "+hex(crc_number,4)+ " : "+hex(highByte,2)+"-"+hex(lowByte,2));
  
  outgoingString = "";
  int i = 0;
  for(i=0; i<poll.length; i++) {
    outgoingString+=hex(poll[i],2)+"  ";
  }
  //for(i=0; i<poll.length; i++) {
  //  myPort.write(poll[i]);        // this doesn't work 
  //}
  myPort.write(poll);
  
  
  if (debug_print) {
    print("outgoing:      ");
    for(i=0; i<poll.length; i++) {
      print(hex(poll[i],2));
      print(" ");
    }
    println();
  }
  
  askTime = millis();

}



int crc16(byte[] buffer, int buffer_length) {
  int i,j;
  int c;
  int CRC=0xFFFF;
  for(j = 0; j < buffer_length; j++) {
    c = buffer[j];
    CRC ^= c;
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
