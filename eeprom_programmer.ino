#include <EEPROM.h>

constexpr uint32_t SIZE_BYTES = 131072;

char buffer[80];

constexpr uint8_t CE = 0x40;
constexpr uint8_t WE = 0x20;
constexpr uint8_t OE = 0x10;

inline void enableChip(boolean value)
{
  if (value)
    PORTB &= ~CE;
  else
    PORTB |= CE;
}

inline void enableOutput(boolean value)
{
  if (value)
    PORTB &= ~OE;
  else
    PORTB |= OE;
}

inline void enableWrite(boolean value)
{
  if (value)
    PORTB &= ~WE;
  else
    PORTB |= WE;
}

inline void setIOMode(char mode, bool changeOE = true)
{
  if (mode == OUTPUT)
  {
    if (changeOE)
      enableOutput(false);
    DDRA = 0xFF;
  }
  else
  {
    DDRA = 0;
    PORTA = 0;    // disable pullups
    if (changeOE)
      enableOutput(true);
  }
}

inline void setAddress(uint32_t address)
{
  auto addressBytes = reinterpret_cast<const uint8_t*>(&address);
  
  PORTC = addressBytes[0];
  PORTB = (PORTB & 0xF0) | (addressBytes[1] & 0x0F);
  PORTF = ((addressBytes[2] & 0x01) << 4) | (addressBytes[1] >> 4);
}

inline void writeByte(uint8_t value)
{
  PORTA = value;
  enableWrite(true);
  enableWrite(false);
}

inline void writeByteAt(uint32_t address, uint8_t ch)
{
  setIOMode(OUTPUT);
  enableChip(true);
  _NOP();
  setAddress(address);
  writeByte(ch);
  enableChip(false);
  setIOMode(INPUT);
}

inline uint8_t readByte()
{
  enableChip(true);
  enableOutput(true);
  _NOP();
  uint8_t value = PINA;
  enableChip(false);
  enableOutput(false);

  return value;
}

inline uint8_t readByteAt(uint32_t address)
{
  setAddress(address);
  return readByte();
}

bool waitWriteComplete()
{
  setIOMode(INPUT);

  uint8_t firstByte;
  uint8_t secondByte = readByte();
  
  enableChip(false);
  enableOutput(false);

  do
  {
    delayMicroseconds(5);
    firstByte = secondByte;
    secondByte = readByte();
  } while (firstByte != secondByte);

  return readByte() == readByte();
}

bool writeEEPROM(uint32_t address, const uint8_t* data, uint16_t length)
{
  enableChip(true);
  setIOMode(OUTPUT);
  
  for (const uint8_t* ch = data; ch != data + length; ++ch, ++address)
  {
    //snprintf(buffer, sizeof(buffer), "Writing %u bytes to %04x (%u remaining)\n", blockSize, address, remaining);
    //Serial.write(buffer);
    //Serial.flush();

    // Software Data Protection unlock sequence
    writeByteAt(0x5555, 0xAA);
    writeByteAt(0x2AAA, 0x55);
    writeByteAt(0x5555, 0xA0);

    writeByteAt(address, *ch);
    if (!waitWriteComplete())
    {
      snprintf(buffer, sizeof(buffer), "Failed to write byte %02x at address %05lx\n", (unsigned)*ch, address);
      Serial.write(buffer);
      Serial.flush();
    }
    
    setIOMode(OUTPUT);
  }

  setIOMode(INPUT);
  enableChip(false);
  return true;
}

void readEEPROM(uint32_t address, uint8_t* buffer, uint16_t size)
{
  setIOMode(INPUT);
  enableChip(true);

  while(size--)
    *buffer++ = readByteAt(address++);

  enableChip(false);
}

void dumpEEPROM()
{
  constexpr unsigned BYTES_PER_ROW = 16;
  constexpr unsigned TOTAL_ROWS = SIZE_BYTES / BYTES_PER_ROW;

  setIOMode(INPUT);
  enableChip(true);

  uint8_t prevRow[BYTES_PER_ROW];
  bool sameFlag = false;

  for (unsigned i = 0; i < TOTAL_ROWS; ++i)
  {
    auto baseAddress = static_cast<uint32_t>(BYTES_PER_ROW) * i;
    
    uint8_t row[BYTES_PER_ROW];
    for (char j = 0; j < BYTES_PER_ROW; ++j)
    {
      setAddress(baseAddress + j);
      row[j] = readByte();
    }

    if (i == 0 || memcmp(prevRow, row, BYTES_PER_ROW) != 0)
    {
      sameFlag = false;
      snprintf(buffer, sizeof(buffer), "%05lx: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
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

  snprintf(buffer, sizeof(buffer), "%05lx end\n", SIZE_BYTES);
  Serial.write(buffer);
  Serial.flush();  

  enableChip(false);
}

bool updateEEPROM(uint8_t offsetValue = 0)
{
  uint8_t eepromBuf[256];
  
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

  for (uint32_t address = 0; address < SIZE_BYTES; address += sizeof(eepromBuf))
  {
    if (!writeEEPROM(address, eepromBuf, 256))
      break;

    if ((address & 0x7FF) == 0)
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

  for (uint32_t address = 0; address < SIZE_BYTES; address += 256)
  {
    uint8_t tmp[256];
    readEEPROM(address, tmp, sizeof(tmp));

    for (uint16_t i = 0; i < sizeof(tmp); ++i)
    {
      if (eepromBuf[i] != tmp[i])
      {
        snprintf(buffer, sizeof(buffer), "Verification failed at %05lx: expected %02x actual %02x\n", address + i, eepromBuf[i], tmp[i]);
        Serial.write(buffer);
        Serial.flush();
        return false;
      }
    }
  }

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
//      if (!waitWriteComplete(newByte))
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
  enableChip(true);

  setAddress(0);
  delay(1500);
  
  for (unsigned i = 0; i < SIZE_BYTES; ++i)
  {
    //snprintf(buffer, sizeof(buffer), "Address 0x%04x\n", i);
    //Serial.write(buffer);
    setAddress(i);
    delay(50);
  }

  enableChip(false);
}

void readID()
{
  setIOMode(OUTPUT);
  enableChip(true);

  writeByteAt(0x5555, 0xAA);
  writeByteAt(0x2AAA, 0x55);
  writeByteAt(0x5555, 0x90);
  
//  setAddress(0x5555);
//  writeByte(0xAA);
//  setAddress(0x2AAA);
//  writeByte(0x55);
//  setAddress(0x5555);
//  writeByte(0x90);

  enableChip(false);
  _NOP();
  _NOP();
  _NOP();
  setIOMode(INPUT);
  enableChip(true);
  
  uint8_t idBytes[2];
  setAddress(0x0000);
  idBytes[0] = readByte();
  setAddress(0x0001);
  idBytes[1] = readByte();

  setIOMode(OUTPUT);
  setAddress(0x5555);
  writeByte(0xAA);
  setAddress(0x2AAA);
  writeByte(0x55);
  setAddress(0x5555);
  writeByte(0xF0);
  enableChip(false);
  delayMicroseconds(20);

  snprintf(buffer, sizeof(buffer), "ID bytes: %02x %02x\n", (unsigned)idBytes[0], (unsigned)idBytes[1]);
  Serial.write(buffer);
}

void eraseSector(uint8_t sector)
{
  snprintf(buffer, sizeof(buffer), "Erasing sector %04x\n", sector);
  Serial.write(buffer);
  
  auto address = static_cast<uint32_t>(sector) << 12;
  auto start = millis();

  setIOMode(OUTPUT);
  enableChip(true);
  writeByteAt(0x5555, 0xAA);
  writeByteAt(0x2AAA, 0x55);
  writeByteAt(0x5555, 0x80);
  writeByteAt(0x5555, 0xAA);
  writeByteAt(0x2AAA, 0x55);
  writeByteAt(address, 0x30);

  if (!waitWriteComplete())
  {
    snprintf(buffer, sizeof(buffer), "Failed to erase sector %04x\n", address);
    Serial.write(buffer);
    Serial.flush();
    return;
  }

  auto end = millis();
  snprintf(buffer, sizeof(buffer), "Sector erase finished (%lu ms)\n", end - start);
  Serial.write(buffer);
}

void eraseChip()
{
  auto start = millis();

  setIOMode(OUTPUT);
  enableChip(true);
  writeByteAt(0x5555, 0xAA);
  writeByteAt(0x2AAA, 0x55);
  writeByteAt(0x5555, 0x80);
  writeByteAt(0x5555, 0xAA);
  writeByteAt(0x2AAA, 0x55);
  writeByteAt(0x5555, 0x10);

  if (!waitWriteComplete())
  {
    Serial.write("Failed to erase chip\n");
    Serial.flush();
    return;
  }

  auto end = millis();
  snprintf(buffer, sizeof(buffer), "Chip erase finished (%lu ms)\n", end - start);
  Serial.write(buffer);
  Serial.flush();
}

void setup() 
{
  delay(1000);

  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  PORTF = 0;
  
  enableChip(false);
  setIOMode(INPUT);
  enableWrite(false);
  setAddress(0);

  DDRB = 0xff;
  DDRC = 0xff;
  DDRF = 0xff;

  Serial.begin(230400);

  while (!Serial)
    ;

  setIOMode(INPUT);

  // eraseSector(0);
  eraseChip();
  updateEEPROM();
  // writePong();
  runEEPROM();
  // readID();
}

void loop() {

}
