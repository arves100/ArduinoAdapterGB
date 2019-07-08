#include <EEPROM.h>

/*
 * Generic Pinout:
 *  GBA SO : MISO
 *  GBA SI : MOSI
 *  GBA SC : SCLK
 *  GBA SD : SS
 * 
 * NodeMCU v3 Pinout:
 *  GBA SO : D6 (GPIO12/HMISO)
 *  GBA SI : D7 (GPIO13/RXD2/HMOSI)
 *  GBA SC : D5 (GPIO14/HSCLK)
 *  GBA SD : D8 (GPSIO15/TXD2/HSC)
 *  
 * Arduino UNO Pinout:
 *  GBA SO : 12/ICSP-1
 *  GBA SI : 11/ICSP-4
 *  GBA SC : 13/ICSP-3
 *  GBA SD : 10
 */

enum TransferState
{
  TRANSFER_WAITING = 0,
  TRANSFER_PREAMBLE,
  TRANSFER_PACKET_START,
  TRANSFER_PACKET_01,
  TRANSFER_PACKET_02,
  TRANSFER_PACKET_LEN,
  TRANSFER_PACKET_BODY,
  TRANSFER_CHECKSUM_1,
  TRANSFER_CHECKSUM_2,
  TRANSFER_DEVICE_ID,
  TRANSFER_STATUS_BYTE,
};

struct packet_t
{
  byte command;
  byte len;
  byte data[255];
  unsigned short checksum;
  byte processedData;
};

struct adapter_t
{
  byte transferStatus;
  bool isBusy;
  bool isSending;
  packet_t packet;
};

static volatile adapter_t adapter;

volatile long idleTime = 0, lastAction = 0;

void resetAdapter()
{
  adapter.packet.command = 0x00;
  adapter.packet.len = 0;

  unsigned short i = 0;
  for (; i < 255; i++)
    adapter.packet.data[i] = 0;

  adapter.packet.checksum = 0;
  //adapter.packet.readedChecksum = 0;
  adapter.packet.processedData = 0;
  adapter.transferStatus = TRANSFER_WAITING;
  adapter.isBusy = false;
  adapter.isSending = false;
}

unsigned char process(unsigned char command)
{
  byte out = 0x4B;
  if (command == 0x4B)
    return out;

  if (adapter.transferStatus == TRANSFER_WAITING)
  {
    if (command == 0x99 || adapter.isSending)
    {
      adapter.transferStatus = TRANSFER_PREAMBLE;
      out = 0x99;
    }
    else
    {
      out = 0xFF;
    }
  }
  else if (adapter.transferStatus == TRANSFER_PREAMBLE)
  {
    if (command == 0x66 || adapter.isSending)
    {
      adapter.transferStatus = TRANSFER_PACKET_START;
      out = 0x66;
    }
    else
    {
      adapter.transferStatus = TRANSFER_WAITING;
      out = 0xFF; // Error code?
    }
  }
  else if (adapter.transferStatus == TRANSFER_PACKET_START)
  {
    if (adapter.isSending)
    {
      out = adapter.packet.command;
      adapter.packet.checksum = adapter.packet.command;
    }
    else
    {
      adapter.packet.command = command;
      //out = 0x00;
    }

    adapter.transferStatus = TRANSFER_PACKET_01;
  }
  else if (adapter.transferStatus == TRANSFER_PACKET_01)
  {
    adapter.transferStatus = TRANSFER_PACKET_02;
    out = 0x00;
  }
  else if (adapter.transferStatus == TRANSFER_PACKET_02)
  {
    adapter.transferStatus = TRANSFER_PACKET_LEN;
    out = 0x00;
  }
  else if (adapter.transferStatus == TRANSFER_PACKET_LEN)
  {
    if (adapter.isSending)
    {
      out = adapter.packet.len;
      adapter.packet.checksum += adapter.packet.len;
    }
    else
    {
      adapter.packet.len = command;
      //out = adapter.packet.len;
    }

    adapter.packet.processedData = 0;

    if (adapter.packet.len > 0)
      adapter.transferStatus = TRANSFER_PACKET_BODY;
    else
      adapter.transferStatus = TRANSFER_CHECKSUM_1;
  }
  else if (adapter.transferStatus == TRANSFER_DEVICE_ID)
  {
    adapter.transferStatus = TRANSFER_STATUS_BYTE;
    out = 0x88;
  }
  else if (adapter.transferStatus == TRANSFER_PACKET_BODY)
  {
    if (!adapter.isSending)
    {
      adapter.packet.data[adapter.packet.processedData] = command;     
    }
    else
    {
      out = adapter.packet.data[adapter.packet.processedData];
      adapter.packet.checksum += adapter.packet.data[adapter.packet.processedData];
    }

    adapter.packet.processedData++; 
         
    if (adapter.packet.processedData >= adapter.packet.len)
      adapter.transferStatus = TRANSFER_CHECKSUM_1;
  }
  else if (adapter.transferStatus == TRANSFER_CHECKSUM_1)
  {
    if (adapter.isSending)
      out = adapter.packet.checksum >> 8;
    else
    {
      adapter.packet.checksum = command << 8;
      //out = 0x00;
    }

    adapter.transferStatus = TRANSFER_CHECKSUM_2;
  }
  else if (adapter.transferStatus == TRANSFER_CHECKSUM_2)
  {
    if (adapter.isSending)
    {
      out = adapter.packet.checksum & 0xFF;
    }
    else
    {
      adapter.packet.checksum += command;
      //out = 0x00;
  
      //if (adapter.packet.readedChecksum != adapter.packet.checksum && !adapter.isSending)
      //{
      //  Serial.print("Warning: Checksum ");
      //  Serial.print(adapter.packet.readedChecksum);
      //  Serial.print(" mismatches from ");
      //  Serial.println(adapter.packet.checksum);
      //}
    }

    adapter.transferStatus = TRANSFER_DEVICE_ID;
   }
  else if (adapter.transferStatus == TRANSFER_STATUS_BYTE)
  {
    adapter.transferStatus = TRANSFER_WAITING;

    if (adapter.isSending)
    {
      out = 0x00;
      adapter.isSending = false;
    }
    else
    {
      out = craftResponsePacket();
      adapter.isSending = true;
    }
  }

  lastAction = millis();

  return out;
}

ISR (SPI_STC_vect)
{
  unsigned char in = SPDR;
  unsigned char out = process(in);
  
  Serial.print("Input data is: ");
  Serial.println(in, HEX);
  Serial.print("Output data is: ");
  Serial.println(out, HEX);
  SPDR = out;
}

void setup() {
  Serial.begin(2000000);
  
  resetAdapter();

  // 0x0C = SERIAL_MODE_3 (0x0C) & SPI_MODE_MASK (0x0C). See arduino/avr/libraries/SPI/src/SPI.h
  // 0x02 = Clock divided by 5
  SPCR |= _BV(SPE) | _BV(CPOL) | _BV(CPHA) /*| 0x02*/;
  //SPSR |= 0x02; // Clock div 5
  SPCR |= _BV(SPIE);

  Serial.println("Mobile Arduino GB v0.1");
}

void loop() {

}

byte craftResponsePacket()
{
  byte return_value = 0x80 ^ adapter.packet.command;

  Serial.print(">> ");
  Serial.println(adapter.packet.command, HEX);
  
  switch (adapter.packet.command)
  {
    case 0x10:
      break;
    default:
      Serial.println("Unknown!");
  }

  return return_value;
}
