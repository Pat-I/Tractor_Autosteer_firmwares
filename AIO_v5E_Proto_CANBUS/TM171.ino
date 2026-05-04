//Based on the work of Paulius a.k.a. babtai RTK!

//HardwareSerialIMXRT* SerialImu = &Serial5; // &Serial5 for RVC port
//uint16_t serial_buffer_size = 512;
//uint8_t SerialImurxbuffer[serial_buffer_size];  //Extra serial tx buffer
//uint8_t SerialImutxbuffer[serial_buffer_size];  //Extra serial tx buffer

//State machine from the almighty ChatGPT
enum TM171ParseState {
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  WAIT_LENGTH,
  WAIT_PAYLOAD
};

TM171ParseState parseState = WAIT_HEADER_1;
uint8_t packetLength = 0;
uint8_t payloadIndex = 0;
bool gotPacket = false;
uint16_t goodPackets = 0;

uint8_t ImuData[128];
uint8_t ImuDC = 0;

union Onion {
  uint8_t fBytes[sizeof(float)];
  float fValue;
};

Onion YawV;
Onion RollV;
Onion PitchV;

Onion TemperatureV;
uint8_t qos;

//#define TM171DEBUG

void TM171setup() {
  //SerialImu->begin(115200);
  //SerialImu->addMemoryForWrite(SerialImutxbuffer, serial_buffer_size);
  //SerialImu->addMemoryForRead(SerialImurxbuffer, serial_buffer_size);
}

void TM171process() {

  while (SerialImu->available() && !gotPacket) {
    uint8_t temp = SerialImu->read();
    switch (parseState) {
      case WAIT_HEADER_1:
        if (temp == 0xAA) {
          ImuData[0] = temp;
          parseState = WAIT_HEADER_2;
        }
        break;

      case WAIT_HEADER_2:
        if (temp == 0x55) {
          ImuData[1] = temp;
          parseState = WAIT_LENGTH;
        } else {
          parseState = WAIT_HEADER_1;  //Reset if second byte isn't 0x55
        }
        break;

      case WAIT_LENGTH:
        packetLength = temp;
        ImuData[2] = temp;
        payloadIndex = 3;  // We've stored 3 bytes so far
        parseState = WAIT_PAYLOAD;

        //check for unreasonable lengths
        if (packetLength + 5 > sizeof(ImuData)) {
          parseState = WAIT_HEADER_1;
          Serial.println("Too big data from TM171!");
        }
        break;

      case WAIT_PAYLOAD:
        ImuData[payloadIndex++] = temp;

        if (payloadIndex >= packetLength + 5) {
          gotPacket = true;
          parseState = WAIT_HEADER_1;
          payloadIndex = 0;
          break;
        }
        break;
    }
  }

  if (gotPacket) {
    gotPacket = false;
    //      if (GoodCRC(ImuData,packetLength+5))
    uint16_t cg = Checksum_Generate(&ImuData[2], packetLength + 1);
    uint16_t ck = ImuData[packetLength + 3] + (ImuData[packetLength + 4] << 8);

    if (cg == ck) {  //if (GoodCRC(ImuData, ImuData[2] + 5))
      TM171lastData = 0;
      useTM171 = true;
      goodPackets++;
      uint8_t functionCode = ImuData[3];  // Function ID (documented at 4th byte)
      switch (functionCode) {
        case 35:  //RPY Output
          RollV.fBytes[0] = ImuData[11];
          RollV.fBytes[1] = ImuData[12];
          RollV.fBytes[2] = ImuData[13];
          RollV.fBytes[3] = ImuData[14];

          PitchV.fBytes[0] = ImuData[15];
          PitchV.fBytes[1] = ImuData[16];
          PitchV.fBytes[2] = ImuData[17];
          PitchV.fBytes[3] = ImuData[18];

          YawV.fBytes[0] = ImuData[19];
          YawV.fBytes[1] = ImuData[20];
          YawV.fBytes[2] = ImuData[21];
          YawV.fBytes[3] = ImuData[22];

#ifdef TM171DEBUG
          Serial.print("yaw=");
          Serial.print(YawV.fValue);
          Serial.print(",pitch=");
          Serial.print(PitchV.fValue);
          Serial.print(",Roll=");
          Serial.println(RollV.fValue);
#endif
          break;

        case 22:  // Status Output
          TemperatureV.fBytes[0] = ImuData[11];
          TemperatureV.fBytes[1] = ImuData[12];
          TemperatureV.fBytes[2] = ImuData[13];
          TemperatureV.fBytes[3] = ImuData[14];
          qos = ImuData[17] & 0x07;
#ifdef TM171DEBUG
          Serial.print("Temp status:");
          Serial.print(TemperatureV.fValue);
          Serial.print(",QoS:");
          Serial.println(qos);
#endif
          break;

        default:
#ifdef TM171DEBUG
          Serial.print("Unhandled packet type: ");
          Serial.println(functionCode, HEX);
#endif
          break;
      }
    } else {
      /*
      Serial.println("CRC was bad :(");
      Serial.print("Length: ");
      Serial.println(ImuData[2]);
      Serial.print("Code: ");
      Serial.print(ImuData[3]);
      */
      //Serial.print(" nbr of good packets: ");
      Serial.print("gd pk: ");
      Serial.println(goodPackets);
      /*
      int totalLength = ImuData[2] + 5;  // Ta longueur calculée

      for (int i = 0; i < totalLength; i++) {
        if (ImuData[i] < 0x10) Serial.print("0");  // Ajoute un zéro pour avoir "0A" au lieu de "A"
        Serial.print(ImuData[i], HEX);
        Serial.print(" ");  // Espace pour la lisibilité
      }
      Serial.print(" ");  // Retour à la ligne à la fin
      Serial.print(" CRC read from data: ");
      Serial.print(ck);
      Serial.print(" CRC calculated: ");
      Serial.println(cg);
      */
      goodPackets = 0;
    }
    // Reset for next packet
    //memset(ImuData, 0, sizeof(ImuData));
    //parseState = WAIT_HEADER_1;
    /*
            ImuData[0] = 0;
            ImuData[1] = 0;
            ImuData[2] = 0;
      */
  }  //gotPacket
}

uint16_t Checksum_Generate(uint8_t* data, int dataLength) {
  uint16_t checkSum = 0xFFFF;  // Valeur de départ standard pour CRC-16

  for (int i = 0; i < dataLength; i++) {
    checkSum ^= (uint16_t)data[i];

    for (int j = 0; j < 8; j++) {
      if (checkSum & 0x0001) {
        checkSum = (checkSum >> 1) ^ 0xA001;
      } else {
        checkSum >>= 1;
      }
    }
  }
  return checkSum;
}