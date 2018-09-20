#include <EEPROM.h>

constexpr unsigned SIZE_BYTES = 8192;

char buffer[80];
  
inline void enableOutput(boolean value)
{
  if (value)
    PORTB &= ~0x10;
  else
    PORTB |= 0x10;
}

inline void enableWrite(boolean value)
{
  if (value)
    PORTB &= ~0x20;
  else
    PORTB |= 0x20;
}

inline void setIOMode(char mode)
{
  if (mode == OUTPUT)
  {
    enableOutput(false);
    DDRA = 0xFF;
  }
  else
  {
    DDRA = 0;
    PORTA = 0;    // disable pullups
    enableOutput(true);
  }
}

inline void setAddress(unsigned address)
{
  unsigned char addressLow = address;
  unsigned char addressHigh = address >> 8;
  unsigned char addressHighFixed = (addressHigh & 0x0F) | ((addressHigh & 0x10) << 3);
  
  PORTC = addressLow;
  PORTB = (PORTB & 0x70) | addressHighFixed;
}

void writeByte(char value)
{
  PORTA = value;
  enableWrite(true);
  enableWrite(false);
}

inline unsigned char readByte()
{
  return PINA;
}

bool waitWriteCycle(uint8_t prevWrittenByte)
{
  setIOMode(INPUT);

  bool writeSuccess = false;

  for (uint16_t i = 0; i < 1000; ++i)
  {
    if (readByte() == prevWrittenByte)
    {
      writeSuccess = true;
      break;
    }

    enableOutput(false);
    delayMicroseconds(999);
    enableOutput(true);
    delayMicroseconds(1);
  }

   setIOMode(OUTPUT);
   return writeSuccess;
   
//  constexpr byte BIT6_MASK = 0b01000000;
//  bool writeSuccess = false;
//
//  auto prevState = readByte() & BIT6_MASK;
//  for (uint8_t i = 0; i < 1000; ++i)
//  {
//    enableOutput(false);
//    
//    delayMicroseconds(100);
//    enableOutput(true);
//    delayMicroseconds(1);
//
//    auto newState = readByte() & BIT6_MASK;
//    if (prevState == newState)
//    {
//      writeSuccess = true;
//      break;
//    }
//      
//    prevState = newState;
//  }
//
//  return writeSuccess;
}

bool writeEEPROM(uint16_t address, const uint8_t* data, uint16_t length)
{
  setIOMode(OUTPUT);
  
  auto remaining = length;
  
  while (remaining > 0)
  {
    uint8_t blockSize = remaining > 64 ? 64 : remaining;
    remaining -= blockSize;

    //snprintf(buffer, sizeof(buffer), "Writing %u bytes to %04x (%u remaining)\n", blockSize, address, remaining);
    //Serial.write(buffer);
    //Serial.flush();

    while (blockSize--)
    {
      setAddress(address++);
      writeByte(*data++);
    }

    auto prevByte = *(data - 1);
    
    if (!waitWriteCycle(prevByte))
    {
      setIOMode(INPUT);
      snprintf(buffer, sizeof(buffer), "Failed to verify byte %02x at address %04x: read %02x\n", (unsigned)prevByte, address - 1, (unsigned)readByte());
      Serial.write(buffer);
      Serial.flush();
      return false;
    }
  }

  setIOMode(INPUT);
  return true;
}

static uint8_t eepromBuf[256];

bool clearEEPROM()
{
  static_assert(sizeof(eepromBuf) >= 256, "buffer too small");

  memset(eepromBuf, 0xff, 256);

  for (unsigned i = 0; i < SIZE_BYTES / 256; ++i)
    if (!writeEEPROM(i * 256, eepromBuf, 256))
      return false;

  return true;
}

void dumpEEPROM()
{
  constexpr unsigned BYTES_PER_ROW = 16;

  setIOMode(INPUT);

  uint8_t prevRow[BYTES_PER_ROW];
  bool sameFlag = false;

  for (unsigned i = 0; i < SIZE_BYTES / BYTES_PER_ROW; ++i)
  {
    unsigned baseAddress = BYTES_PER_ROW * i;
    
    uint8_t row[BYTES_PER_ROW];
    for (char j = 0; j < BYTES_PER_ROW; ++j)
    {
      setAddress(baseAddress + j);
      row[j] = readByte();
    }

    if (i == 0 || memcmp(prevRow, row, BYTES_PER_ROW) != 0)
    {
      sameFlag = false;
      snprintf(buffer, sizeof(buffer), "%04x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
               baseAddress,
               row[0], row[1], row[ 2], row[ 3], row[ 4], row[ 5], row[ 6], row[ 7],
               row[8], row[9], row[10], row[11], row[12], row[13], row[14], row[15]);
      Serial.write(buffer);
      Serial.flush();
    }
    else if (!sameFlag)
    {
      sameFlag = true;
      strncpy(buffer, "*\n", sizeof(buffer));
      Serial.write(buffer);
      Serial.flush();
    }

    memcpy(prevRow, row, BYTES_PER_ROW);
  }

  snprintf(buffer, sizeof(buffer), "%04x end\n", SIZE_BYTES);
  Serial.write(buffer);
  Serial.flush();  
}

bool updateEEPROM()
{
  Serial.write("Clearing EEPROM\n");

  if (!clearEEPROM())
    return false;

  dumpEEPROM();
  
//  unsigned char offsetValue = EEPROM.read(0);
//  EEPROM.write(0, offsetValue + 1);
  uint8_t offsetValue = 0;

  snprintf(buffer, sizeof(buffer), "Preparing buffer (%p)\n", eepromBuf);
  Serial.write(buffer);

  static_assert(sizeof(eepromBuf) >= 256, "buffer too small");
  for (uint16_t i = 0; i < 256; ++i)
  {
    eepromBuf[i] = i + offsetValue;
  }

  Serial.write("Beginning to write data\n");
  unsigned long startTime = millis();

//  uint8_t msg[] = "goodbye world";
//  writeEEPROM(0x43, msg, sizeof(msg));

  for (uint8_t i = 0; i < SIZE_BYTES / 256; ++i)
  {
    if (!writeEEPROM(i * sizeof(eepromBuf), eepromBuf, 256))
      break;
    
    Serial.write(".");
  }

  Serial.write("\n\n");

  setIOMode(INPUT);

  unsigned long duration = millis() - startTime;

  snprintf(buffer, sizeof(buffer), "Write finished (%lu ms)\n", duration);
  Serial.write(buffer);
  Serial.write("Verifying contents\n");

  constexpr unsigned BYTES_PER_ROW = 16;

  startTime = millis();

  dumpEEPROM();

//  for (unsigned i = 0; i < SIZE_BYTES / BYTES_PER_ROW; ++i)
//  {
//    unsigned baseAddress = BYTES_PER_ROW * i;
//    
//    unsigned bufferLength = 0;
//    bufferLength += snprintf(buffer, sizeof(buffer), "%04x:", baseAddress);
//
//    for (char j = 0; j < BYTES_PER_ROW; ++j)
//    {
//      setAddress(baseAddress + j);
//      unsigned data = readByte();
//      auto fmtSize = snprintf(buffer + bufferLength, sizeof(buffer) - bufferLength, " %02x", data & 0x00FF);
//      bufferLength += fmtSize;
//    }
//
//    Serial.write(buffer, bufferLength);
//    Serial.write("\n");
//    Serial.flush();
//  }

  duration = millis() - startTime;

  snprintf(buffer, sizeof(buffer), "Verification complete (%lu ms)\n", duration);
  Serial.write(buffer);

  return true;
}

//__attribute__((noinline)) bool writePong()
//{
//  static constexpr unsigned char pongStates[] = {
//    0b00000000,
//    0b00000001,
//    0b00000011,
//    0b00000101,
//    0b00001001,
//    0b00010001,
//    0b00100001,
//    0b01000001,
//    0b10000001,
//    0b11000001,
//    0b10100001,
//    0b10010001,
//    0b10001001,
//    0b10000101,
//    0b10000011,
//    0b10000111,
//    0b10001011,
//    0b10010011,
//    0b10100011,
//    0b11000011,
//    0b11100011,
//    0b11010011,
//    0b11001011,
//    0b11000111,
//    0b11001111,
//    0b11010111,
//    0b11100111,
//    0b11110111,
//    0b11101111,
//    0b11111111,
//  };
//  
//  Serial.write("Beginning to write data\n");
//  unsigned long startTime = millis();
//
//  static_assert(sizeof(eepromBuf) >= 256, "buffer too small");
//
//  unsigned i = 0;
//  while (i < SIZE_BYTES)
//  {
//    unsigned copied = 0;
//    while (copied < sizeof(eepromBuf))
//    {
//      memcpy(eepromBuf + copied, pongStates, sizeof(pongStates));
//      copied += 
//
//      
//  }
//  for (unsigned i = 0; i < SIZE_BYTES; ++i)
//  {
//    unsigned char state = i % (sizeof(pongStates) / sizeof(pongStates[0]));
//    unsigned char newByte = pongStates[state];
//
//    setAddress(i);
//    writeByte(newByte);
//
//    if ((i + 1) % 64 == 0)
//    {
//      unsigned percentComplete = 100UL * (i + 1) / SIZE_BYTES;
//      if (percentComplete % 5 == 0)
//      {
//        snprintf(buffer, sizeof(buffer), "Written %u%%\n", percentComplete);
//        Serial.write(buffer);
//      }
//
//      if (!waitWriteCycle(newByte))
//      {
//        snprintf(buffer, sizeof(buffer), "Failed to verify byte %02x at address %04x: read %02x\n", (unsigned)newByte, i, (unsigned)readByte());
//        Serial.write(buffer);
//        return false;
//      }
//
//      setIOMode(OUTPUT);
//    }
//  }
//
//  setIOMode(INPUT);
//
//  unsigned long duration = millis() - startTime;
//
//  snprintf(buffer, sizeof(buffer), "Write finished (%lu ms)\n", duration);
//  Serial.write(buffer);
//  Serial.write("Verifying contents\n");
//
//  constexpr unsigned BYTES_PER_ROW = 16;
//
//  startTime = millis();
//
//  for (unsigned i = 0; i < SIZE_BYTES / BYTES_PER_ROW; ++i)
//  {
//    unsigned baseAddress = BYTES_PER_ROW * i;
//    
//    unsigned bufferLength = 0;
//    bufferLength += snprintf(buffer, sizeof(buffer), "%04x:", baseAddress);
//
//    for (char j = 0; j < BYTES_PER_ROW; ++j)
//    {
//      setAddress(baseAddress + j);
//      unsigned data = readByte();
//      auto fmtSize = snprintf(buffer + bufferLength, sizeof(buffer) - bufferLength, " %02x", data & 0x00FF);
//      bufferLength += fmtSize;
//    }
//
//    Serial.write(buffer, bufferLength);
//    Serial.write("\n");
//  }
//
//  duration = millis() - startTime;
//
//  snprintf(buffer, sizeof(buffer), "Verification complete (%lu ms)\n", duration);
//  Serial.write(buffer);
//
//  return true;
//}
//
void runEEPROM()
{
  setIOMode(INPUT);

  setAddress(0);
  delay(1500);
  
  for (unsigned i = 0; i < SIZE_BYTES; ++i)
  {
    //snprintf(buffer, sizeof(buffer), "Address 0x%04x\n", i);
    //Serial.write(buffer);
    setAddress(i);
    delay(100);
  }
}

void unprotectedEEPROM()
{
  setIOMode(OUTPUT);
  
  setAddress(0x1555 << 6);
  writeByte(0xAA);
  setAddress(0x0AAA << 6);
  writeByte(0x55);
  setAddress(0x1555 << 6);
  writeByte(0x80);
  writeByte(0xAA);
  setAddress(0x0AAA << 6);
  writeByte(0x55);
  setAddress(0x1555 << 6);
  writeByte(0x20);

  delayMicroseconds(1);
  setIOMode(INPUT);
}

void setup() 
{
  delay(1000);

  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  
  setIOMode(INPUT);
  enableWrite(false);
  setAddress(0);

  PORTB = 0b00110000;
  PORTC = 0;
  DDRB = 0x3f;
  DDRC = 0xff;

  Serial.begin(115200);

  while (!Serial)
    ;

  setIOMode(INPUT);

  unprotectedEEPROM();
  updateEEPROM();
  
  // writePong();
  runEEPROM();
}

void loop() {

}
