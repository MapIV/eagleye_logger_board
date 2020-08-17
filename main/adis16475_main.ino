// Copyright (c) 2020, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ADIS16470.h>
#include <SPI.h>
#include "USBHost_t36.h"
#include <FlexCAN_T4.h>
#include <math.h>

#define USBBAUD       230400
#define IMU_BUFF_SIZE 33
#define CAN_BUFF_SIZE 16

#define NUM_RX_MAILBOXES 6  /* CAN MESSAGE BOX NUM (MB) */
#define interval_value   20 /* IMU polling Interval value (ms) */

#define DEBUG_FLAG 0

// ---- IMU ----
ADIS16470 IMU(10,2,6);  //cs, dr, rst pin define.
uint8_t *burstByte;
uint8_t GXSH, GXSL, GYSH, GYSL, GZSH, GZSL = 0;
uint8_t AXSH, AXSL, AYSH, AYSL, AZSH, AZSL = 0;
uint8_t TEMPSH, TEMPSL = 0;

int MSC = 0;
int FLTR = 0;
int DECR = 0;

unsigned long prev;
int isReceived = 0;
int16_t imu_data[14];

// ----- GNSS -----
USBHost myusb;
USBSerial userial(myusb);
uint8_t rx_buffer[512];

// ---- CAN ----
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;
int isCANReceived = 0;

// ---- PPS ----
int pps_pin;
int isState;
unsigned long pps_cnt = 0;
uint8_t pps_buff[16];
int isPPS = 0;

// ---- General ----
unsigned char imu_buff[ IMU_BUFF_SIZE ];
unsigned char can_buff[ CAN_BUFF_SIZE ];
uint8_t sendbuff[ 1024 ];

int printCounter = 0;
int TXCapacity = 0;
// ---- ---- ----

void setup_CAN(){
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setClock(CLK_60MHz);

  Can0.setMaxMB(NUM_RX_MAILBOXES);
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    Can0.setMB((FLEXCAN_MAILBOX)i,RX);
  }
  
  Can0.setMBFilter(REJECT_ALL);
  Can0.enableMBInterrupts();

  /* set callBack MB */
  Can0.onReceive(MB0, canSniff);
  Can0.onReceive(MB1, canSniff);
  Can0.onReceive(MB2, canSniff);
  Can0.onReceive(MB3, canSniff);
  Can0.onReceive(MB4, canSniff);
  Can0.onReceive(MB5, canSniff);

  /* set can filter hardware */
  /* toyota (speed 0xAA 0xB4 , shift 0x127 0x3BC) */
  Can0.setMBFilter(MB0, 0xAA, 0xB4, 0x127, 0x3BC);

  /* honda  (speed 0x1D0 , shift 0x1A3 0x191) */
  Can0.setMBFilter(MB1, 0x1D0, 0x1A3, 0x191);

  /* nissan (speed 0x285 0x29A , shift 0x41F 0x421) */
  Can0.setMBFilter(MB2, 0x285, 0x29A, 0x41F, 0x421);

  /* mazda  (speed 0x1C , shift 0x228) , subaru (speed 0xD4 , shift 0x148) */
  Can0.setMBFilter(MB3, 0x1C, 0x228, 0xD4, 0x148);

  /* bmw    (speed 0xCE 0x1A0 , shift 0x198) */
  Can0.setMBFilter(MB4, 0x1D0, 0x1A0, 0x198);

  /* mercedes   (speed 0x203 , shift 0x6D) , vw  (speed 0x11E , shift 0x187) */
  Can0.setMBFilter(MB5, 0x203, 0x6D, 0x11E, 0x187);

  /* set can filter software */
  Can0.enhanceFilter(MB0);
  Can0.enhanceFilter(MB1);
  Can0.enhanceFilter(MB2);
  Can0.enhanceFilter(MB3);
  Can0.enhanceFilter(MB4);
  Can0.enhanceFilter(MB5);

  Can0.distribute();
}

void canSniff(const CAN_message_t &msg) {

  unsigned long receive_time = millis();
  memset(can_buff, 0x00, CAN_BUFF_SIZE);
  can_buff[0] = 0x50; // 'P' hex = 0x50

  can_buff[1] = (receive_time >> 24)&0xFF;
  can_buff[2] = (receive_time >> 16)&0xFF;
  can_buff[3] = (receive_time >> 8)&0xFF;
  can_buff[4] = (receive_time)&0xFF;

  can_buff[5] = (unsigned char)msg.len;
  uint16_t canid = (uint16_t)msg.id;
  can_buff[6] = (uint8_t)((canid >> 8) & 0xFF);
  can_buff[7] = (uint8_t)(canid & 0xFF);
  memcpy( &can_buff[8], &msg.buf[0], msg.len );
  isCANReceived = 1;
}

void sensorRead() {
  imu_data[0] = IMU.regRead(X_GYRO_LOW);
  imu_data[1] = IMU.regRead(X_GYRO_OUT);
  imu_data[2] = IMU.regRead(Y_GYRO_LOW);
  imu_data[3] = IMU.regRead(Y_GYRO_OUT);
  imu_data[4] = IMU.regRead(Z_GYRO_LOW);
  imu_data[5] = IMU.regRead(Z_GYRO_OUT);

  imu_data[6] = IMU.regRead(X_ACCL_LOW);
  imu_data[7] = IMU.regRead(X_ACCL_OUT);
  imu_data[8] = IMU.regRead(Y_ACCL_LOW);
  imu_data[9] = IMU.regRead(Y_ACCL_OUT);
  imu_data[10] = IMU.regRead(Z_ACCL_LOW);
  imu_data[11] = IMU.regRead(Z_ACCL_OUT);

  imu_data[12] = IMU.regRead(TEMP_OUT);
  imu_data[13] = IMU.regRead(TIME_STAMP);
}

void initializePPS(int _pin){
  pps_pin = _pin;
  pinMode(pps_pin, INPUT);
  isState = 0;
}

/* Low->High OneShut */
int isPPSHigh(){
  int result = 0;
  if ((isState != 1)&&(digitalRead(pps_pin) == HIGH)) {
    // Serial.printf( "HIGH\n" );
    isState = 1;
    result = isState;
  }
  if( (isState != 0 )&&(digitalRead(pps_pin) == LOW)) {
    // Serial.printf( "LOW\n" );
    isState = 0;
  }
  return result;
}


void setup()
{
  while (!Serial) ;
  Serial.begin(USBBAUD);
  IMU.configSPI();
  delay(500);
  IMU.regWrite(MSC_CTRL, 0xC1);
  IMU.regWrite(FILT_CTRL, 0x04);
  IMU.regWrite(DEC_RATE, 0x00),

  MSC = IMU.regRead(MSC_CTRL);
  FLTR = IMU.regRead(FILT_CTRL);
  DECR = IMU.regRead(DEC_RATE);

  setup_CAN();

  //PPS pin Set
  initializePPS(3);

  myusb.begin();

  memset(sendbuff, 0x00, 1024);
  memset(pps_buff, 0x00, 16);

  userial.begin(230400);
}

void loop()
{
  int send_length;
  uint16_t gnss_rx_len = 0;

  myusb.Task(); //USBHost -> USBDriver: Task() is execution CallBack Function.
  Can0.events();

  while (userial.available())
  {
    rx_buffer[gnss_rx_len] = (uint8_t)userial.read();
    gnss_rx_len++;
  }

  unsigned long curr = millis();
  if ((curr - prev) >= interval_value)
  {
    sensorRead();
    isReceived = 1;
    prev = curr;

    imu_buff[0] = 0x49; // 'I' hex = 0x49
    imu_buff[1] = (curr >> 24)&0xFF; // Timestap [1]-[4]
    imu_buff[2] = (curr >> 16)&0xFF;
    imu_buff[3] = (curr >> 8)&0xFF;
    imu_buff[4] = (curr)&0xFF;
    // Gyro
    imu_buff[5] = (imu_data[0]>>8)&0xFF;
    imu_buff[6] = (imu_data[0])&0xFF;
    imu_buff[7] = (imu_data[1]>>8)&0xFF;
    imu_buff[8] = (imu_data[1])&0xFF;
    imu_buff[9] = (imu_data[2]>>8)&0xFF;
    imu_buff[10] = (imu_data[2])&0xFF;
    imu_buff[11] = (imu_data[3]>>8)&0xFF;
    imu_buff[12] = (imu_data[3])&0xFF;
    imu_buff[13] = (imu_data[4]>>8)&0xFF;
    imu_buff[14] = (imu_data[4])&0xFF;
    imu_buff[15] = (imu_data[5]>>8)&0xFF;
    imu_buff[16] = (imu_data[5])&0xFF;
    // Accl
    imu_buff[17] = (imu_data[6]>>8)&0xFF;
    imu_buff[18] = (imu_data[6])&0xFF;
    imu_buff[19] = (imu_data[7]>>8)&0xFF;
    imu_buff[20] = (imu_data[7])&0xFF;
    imu_buff[21] = (imu_data[8]>>8)&0xFF;
    imu_buff[22] = (imu_data[8])&0xFF;
    imu_buff[23] = (imu_data[9]>>8)&0xFF;
    imu_buff[24] = (imu_data[9])&0xFF;
    imu_buff[25] = (imu_data[10]>>8)&0xFF;
    imu_buff[26] = (imu_data[10])&0xFF;
    imu_buff[27] = (imu_data[11]>>8)&0xFF;
    imu_buff[28] = (imu_data[11])&0xFF;
    //TEMP
    imu_buff[29] = (imu_data[12]>>8)&0xFF;
    imu_buff[30] = (imu_data[12])&0xFF;
    //TIME_STAMP
    imu_buff[31] = (imu_data[13]>>8)&0xFF;
    imu_buff[32] = (imu_data[13])&0xFF;
  }


  if(isPPSHigh()){
    pps_cnt++;

    unsigned long pps_time = millis();
    pps_buff[0] = 0x43; // 'C' hex = 0x43
    pps_buff[1] = (pps_time >> 24)&0xFF; // Timestap [1]-[4]
    pps_buff[2] = (pps_time >> 16)&0xFF;
    pps_buff[3] = (pps_time >> 8)&0xFF;
    pps_buff[4] = (pps_time)&0xFF;
    pps_buff[5] = (pps_cnt >> 24)&0xFF;
    pps_buff[6] = (pps_cnt >> 16)&0xFF;
    pps_buff[7] = (pps_cnt >> 8)&0xFF;
    pps_buff[8] = (pps_cnt)&0xFF;

    isPPS = 1;
  }


  if( (isReceived == 1)||(gnss_rx_len > 0)||(isCANReceived == 1)||(isPPS == 1) )
  {
    /* Set Header Data */
    sendbuff[0] = 0xF7;
    sendbuff[1] = 0xE0;
    send_length = 2;
    if(isReceived == 1){
      // Set IMU Data
      memcpy( &sendbuff[send_length], imu_buff, IMU_BUFF_SIZE );
      send_length += IMU_BUFF_SIZE;
      isReceived = 0;
    }

    if( isCANReceived == 1 ){
      /* Set CAN Data */
      memcpy( &sendbuff[send_length], can_buff, CAN_BUFF_SIZE );
      send_length += CAN_BUFF_SIZE;
      isCANReceived = 0;
    }

    if(gnss_rx_len > 0){
      // Set GNSS Data
      sendbuff[send_length] = 0x47; // 'G' hex = 0x47
      send_length += 1;
      sendbuff[send_length] = (gnss_rx_len>>8)&0xFF;
      send_length += 1;
      sendbuff[send_length] = gnss_rx_len&0xFF;
      send_length += 1;

      memcpy( &sendbuff[send_length], rx_buffer, gnss_rx_len );
      send_length += gnss_rx_len;
      gnss_rx_len = 0;
    }

    if(isPPS == 1){
      memcpy( &sendbuff[send_length], pps_buff, 16 );
      send_length += 16;
      isPPS = 0;
    }

#if DEBUG_FLAG
    for (int i = 0; i < send_length; i++) {
      Serial.printf("0x%02x|", sendbuff[i]);
    }
    Serial.printf("\n");
#else
    Serial.write( (byte*)sendbuff, send_length );
    memset(sendbuff, 0x00, send_length);
#endif
  }

}
