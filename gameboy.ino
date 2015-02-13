#include <EEPROM.h>
#include "pokemon.h"
#include "output.h"

#define MOSI_ 22
#define MISO_ 23
#define SCLK_ 24

int bytes = 0;
uint8_t shift = 0;
uint8_t in_data = 0;
uint8_t out_data = 0;

connection_state_t connection_state = NOT_CONNECTED;
trade_centre_state_t trade_centre_state = INIT;
int counter = 0;

int trade_pokemon = -1;

unsigned long last_bit;

void transferBit(void) {
    in_data |= digitalRead(MISO_) << (7-shift);

    if(++shift > 7) {
        shift = 0;
        bytes++;
        out_data = handleIncomingByte(in_data);
        //Serial.print(bytes);
        //Serial.print(" ");
        Serial.print(trade_centre_state);
        Serial.print(" ");
        Serial.print(in_data, HEX);
        Serial.print(" ");
        Serial.print(out_data, HEX);
        Serial.print("\n");
        in_data = 0;
    }
    
    while(!digitalRead(SCLK_));
    //Serial.print(out_data & 0x80 ? HIGH : LOW);
    digitalWrite(MOSI_, out_data & 0x80 ? HIGH : LOW);
    out_data <<= 1;
}

void setup() {
    Serial.begin(115200);
    pinMode(SCLK_, INPUT);
    pinMode(MISO_, INPUT);
    pinMode(MOSI_, OUTPUT);
    
    Serial.print("hello world\n");
    
    for (int i=0; i<44; i++) {
      Serial.print(" ");
      Serial.print(EEPROM.read(i), HEX);
    }
    Serial.print("\n");
    
    digitalWrite(MOSI_, LOW);
    out_data <<= 1;

}

void loop() {
    last_bit = micros();
    while(digitalRead(SCLK_)) {
      if (micros() - last_bit > 1000000) {
        // the Game Boy is silent. A good time to do book keeping.
        Serial.print("idle\n");
        last_bit = micros();
        shift = 0;
        in_data = 0;
        if(trade_pokemon >= 0 && trade_centre_state < TRADE_PENDING) {
          // a trade has been confrimed
          int start = 19 + (trade_pokemon * 44);
          for (int i=0; i<44; i++) {
            EEPROM.write(i, INPUT_BLOCK[start+i]);
            Serial.print(" ");
            Serial.print(INPUT_BLOCK[start+i], HEX);
          }
          trade_pokemon = -1;
          Serial.print("trade saved\n");
        }
      }
    }
    transferBit();
}

byte handleIncomingByte(byte in) {
  byte send = 0x00;

  switch(connection_state) {
  case NOT_CONNECTED:
    if(in == PKMN_MASTER)
      send = PKMN_SLAVE;
    else if(in == PKMN_BLANK)
      send = PKMN_BLANK;
    else if(in == PKMN_CONNECTED) {
      send = PKMN_CONNECTED;
      connection_state = CONNECTED;
    }
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
      if (counter == 12) {
        send = EEPROM.read(0); // pokemon species
      } else if(counter >= 19 && counter < 19+44) {
        send = EEPROM.read(counter-19);
      } else {
        send = pgm_read_byte(&(DATA_BLOCK[counter]));
      }
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
