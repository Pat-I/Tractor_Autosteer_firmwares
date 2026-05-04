
#define GPS Serial5//Serial7
#define GPS_Dual Serial8
#define GPS_RTK Serial3
#define RTK_Baud 115200

char rxbuffer[512];         //Extra serial rx buffer
char txbuffer[1024];        //Extra serial tx buffer

char rxbuffer_GPS_Dual[512];   //Extra serial rx buffer
char rxbuffer_RTK[1024];       //Extra serial rx buffer

char nmeaBuffer[200];
int count=0;
bool stringComplete = false;

int test = 0;

byte CK_A = 0;
byte CK_B = 0;
byte ackPacket[72] = { 0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

int relposnedByteCount = 0;

//**************************************************************

void GPS_setup()
{
  if(gpsMode == 1 || gpsMode == 3)  GPS.begin(115200);
  else GPS.begin(460800);
  GPS.addMemoryForRead(rxbuffer, 512);
  GPS.addMemoryForWrite(txbuffer, 1023);

  GPS_RTK.begin(RTK_Baud);
  GPS_RTK.addMemoryForRead(rxbuffer_RTK, 1023);

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  //parser.addHandler("G-ZDA", ZDA_Handler);
  parser.addHandler("G-HPR", HPR_Handler);

    if (gpsMode == 1 || gpsMode == 3)  GPS_Dual.begin(115200);
    else GPS_Dual.begin(460800);
    GPS_Dual.addMemoryForRead(rxbuffer_GPS_Dual, 512);


}

void Panda_GPS()
{
    while (GPS.available())
    {
        parser << GPS.read();
    }


    while (GPS_Dual.available())
    {
        uint8_t incoming_char = GPS_Dual.read();

        // Just increase the byte counter for the first 3 bytes
        if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
        {
            relposnedByteCount++;
        }
        else if (relposnedByteCount > 3)
        {
            // Real data, put the received bytes in the buffer
            ackPacket[relposnedByteCount] = incoming_char;
            relposnedByteCount++;
        }
        else
        {
            // Reset the counter, becaues the start sequence was broken
            relposnedByteCount = 0;
        }

        // Check the message when the buffer is full
        if (relposnedByteCount > 71)
        {
            if (calcChecksum())
            {
                //digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED)
                useDual = true;
                relPosDecode();
            }

            relposnedByteCount = 0;
        }
    }
}

bool calcChecksum()
{
    CK_A = 0;
    CK_B = 0;

    for (int i = 2; i < 70; i++)
    {
        CK_A = CK_A + ackPacket[i];
        CK_B = CK_B + CK_A;
    }

    return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}

void relPosDecode() {

    int carrSoln;
    bool gnssFixOk, diffSoln, relPosValid;

    heading = (int32_t)ackPacket[30] + ((int32_t)ackPacket[31] << 8)
        + ((int32_t)ackPacket[32] << 16) + ((int32_t)ackPacket[33] << 24);
    heading *= 0.0001;

    heading += 900;
    if (heading >= 3600) heading -= 3600;
    if (heading < 0) heading += 3600;
    heading *= 0.1;

    baseline = (int32_t)ackPacket[26] + ((int32_t)ackPacket[27] << 8)
        + ((int32_t)ackPacket[28] << 16) + ((int32_t)ackPacket[29] << 24);
    double baseHP = (signed char)ackPacket[41];
    baseHP *= 0.01;
    baseline += baseHP;

    relPosD = (int32_t)ackPacket[22] + ((int32_t)ackPacket[23] << 8)
        + ((int32_t)ackPacket[24] << 16) + ((int32_t)ackPacket[25] << 24);
    double relPosHPD = (signed char)ackPacket[40];
    relPosHPD *= 0.01;
    relPosD += relPosHPD;

    uint32_t flags = ackPacket[66];

    gnssFixOk = flags & 1;
    diffSoln = flags & 2;
    relPosValid = flags & 4;
    carrSoln = (flags & 24) >> 3;

    //must be all ok
    if (!gnssFixOk || !diffSoln || !relPosValid) return;

    if (carrSoln == 2 && baseline > 1)
    {
        rollDual = (asin(relPosD / baseline)) * -RAD_TO_DEG;

        if (dualReadyGGA)
        {
            BuildNmea();
            dualReadyGGA = false;
        }
        else
        {
            dualReadyRelPos = true;   //RelPos ready is true so PAOGI will send when the GGA is also ready
        }
    }
    else
    {
        rollDual *= 0.9;
        dualReadyRelPos = false;   //RelPos ready is true so PAOGI will send when the GGA is also ready
    }

}

//**************************************************************

void Forward_GPS()
{
  while (GPS.available())
  {
    char c = GPS.read();
    nmeaBuffer[count++] = c;
    if(c == '\n')stringComplete = true;
    if(count == 200 || stringComplete == true)break;
  } 

  if(count == 200 || stringComplete == true){ 
    if (stringComplete == true){  
      Udp.beginPacket(ipDestination, AOGPort);
      Udp.write(nmeaBuffer,count);
      Udp.endPacket();
    }
    clearBufferArray();
    count = 0;
  }
  
 }

//**************************************************************

void Forward_Ntrip()
{

//Check for UDP Packet (Ntrip 2233)
    int NtripSize = NtripUdp.parsePacket();
    
    if (NtripSize) 
    {
        NtripUdp.read(NtripData, NtripSize);
        //Serial.print("Ntrip Data ="); 
        //Serial.write(NtripData, sizeof(NtripData)); 
        //Serial.write(10);
        //Serial.println("Ntrip Forwarded");
        GPS.write(NtripData, NtripSize); 
        //LEDs.queueBlueFlash(LED_ID::GPS);
    }

//Check for Radio RTK
    if (GPS_RTK.available())
    {
        GPS.write(GPS_RTK.read());
    }
}
    
//-------------------------------------------------------------------------------------------------

void clearBufferArray()
{
  /*
  for (int i=0; i<count; i++)
  {
    nmeaBuffer[i]=NULL;
    stringComplete = false;
  }
  */
  
  strcpy(nmeaBuffer, "");
  stringComplete = false;

}
     