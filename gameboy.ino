#include <EEPROM.h>
#include "pokemon.h"
#include "output.h"

#define MOSI_ 51
#define MISO_ 50
#define SCLK_ 52

connection_state_t connection_state = NOT_CONNECTED;
trade_centre_state_t trade_centre_state = INIT;
int counter = 0;

int trade_pokemon = -1;

uint8_t transferByte(uint8_t out) {
    uint8_t in = 0;
    for(int i = 8; i; i--) {
        digitalWrite(MOSI_, out & 0x80 ? HIGH : LOW);
        out <<= 1;
        
        digitalWrite(SCLK_, LOW);
        delayMicroseconds(500);
        
        digitalWrite(SCLK_, HIGH);
        delayMicroseconds(500);

        in <<= 1;
        in |= digitalRead(MISO_);
    }
    return in;
}

void setup() {
    Serial.begin(115200);
    pinMode(SCLK_, OUTPUT);
    pinMode(MISO_, INPUT);
    pinMode(MOSI_, OUTPUT);
    
    Serial.print("hello world\n");
    
    digitalWrite(MOSI_, LOW);
    digitalWrite(SCLK_, HIGH);
}

uint8_t next = PKMN_MASTER;
void loop() {
    uint8_t in = transferByte(next);
    next = handleIncomingByte(in);
    Serial.print(in, HEX);
    Serial.print(" ");
    Serial.print(next, HEX);
    Serial.print("\n");
    delay(100);
}

byte handleIncomingByte(byte in) {
  byte send = 0x00;

  switch(connection_state) {
  case NOT_CONNECTED:
    if(in == PKMN_SLAVE)
      send = PKMN_MASTER;
    else if(in == PKMN_CONNECTED) {
      send = PKMN_CONNECTED;
      connection_state = CONNECTED;
    } else
      send = PKMN_MASTER;
    break;

  case CONNECTED:
    if(in == PKMN_CONNECTED)
      send = PKMN_CONNECTED;
    else if(in == PKMN_TRADE_CENTRE)
      connection_state = TRADE_CENTRE;
    else if(in == PKMN_COLOSSEUM)
      connection_state = COLOSSEUM;
    else if(in == PKMN_BREAK_LINK || in == PKMN_MASTER) {
      connection_state = NOT_CONNECTED;
      send = PKMN_BREAK_LINK;
    } else {
      send = in;
    }
    break;

  case TRADE_CENTRE:
    if(trade_centre_state == INIT && in == 0x00) {
      trade_centre_state = READY_TO_GO;
      send = 0x00;
    } else if(trade_centre_state == READY_TO_GO && in == 0xFD) {
      trade_centre_state = SEEN_FIRST_WAIT;
      send = 0xFD;
    } else if(trade_centre_state == SEEN_FIRST_WAIT && in != 0xFD) {
                        // random data of slave is ignored.
      send = in;
      trade_centre_state = SENDING_RANDOM_DATA;
    } else if(trade_centre_state == SENDING_RANDOM_DATA && in == 0xFD) {
      trade_centre_state = WAITING_TO_SEND_DATA;
      send = 0xFD;
    } else if(trade_centre_state == WAITING_TO_SEND_DATA && in != 0xFD) {
      counter = 0;
      // send first byte
      send = pgm_read_byte(&(DATA_BLOCK[counter]));
      INPUT_BLOCK[counter] = in;
      trade_centre_state = SENDING_DATA;
      counter++;
    } else if(trade_centre_state == SENDING_DATA) {
      // if EEPROM is not initialised, please use the pgm data only.
      send = pgm_read_byte(&(DATA_BLOCK[counter]));
      INPUT_BLOCK[counter] = in;
      counter++;
      if(counter == PLAYER_LENGHT) {
        trade_centre_state = SENDING_PATCH_DATA;
      }
    } else if(trade_centre_state == SENDING_PATCH_DATA && in == 0xFD) {
      counter = 0;
      send = 0xFD;
    } else if(trade_centre_state == SENDING_PATCH_DATA && in != 0xFD) {
      send = in;
      counter++;
      if(counter == 197) {
        trade_centre_state = TRADE_PENDING;
      }
    } else if(trade_centre_state == TRADE_PENDING && (in & 0x60) == 0x60) {
      if (in == 0x6f) {
        trade_centre_state = READY_TO_GO;
        send = 0x6f;
      } else {
        send = 0x60; // first pokemon
        trade_pokemon = in - 0x60;
      }
    } else if(trade_centre_state == TRADE_PENDING && in == 0x00) {
      send = 0;
      trade_centre_state = TRADE_CONFIRMATION;
    } else if(trade_centre_state == TRADE_CONFIRMATION && (in & 0x60) == 0x60) {
      send = in;
      if (in  == 0x61) {
        trade_pokemon = -1;
        trade_centre_state = TRADE_PENDING;
      } else {
        trade_centre_state = DONE;
      }
    } else if(trade_centre_state == DONE && in == 0x00) {
      send = 0;
      trade_centre_state = INIT;
    } else {
      send = in;
    }
    break;

  default:
    send = in;
    break;
  }

  return send;
}

