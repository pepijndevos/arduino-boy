#include <EEPROM.h>
#include <SPI.h>
#include "pokemon.h"
#include "pokemon_constants.h"
#include "output.h"
#include <Adafruit_GPS.h>
#include <math.h>

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
IntervalTimer gpstimer;

connection_state_t connection_state = NOT_CONNECTED;
trade_centre_state_t trade_centre_state = INIT;
go_state_t go_state = GO_INIT;
int counter = 0;

int trade_pokemon = -1;

#define EEPROM_REV 2
#define EEPROM_REV_ADDR 24
#define EEPROM_ADDR 25
#define MIN_WALK 20
#define MAX_WALK 100
#define ENCOUNTER_RANDOMNESS 10

walk_t walk = {0, ENCOUNTER_RANDOMNESS};
float next_encounter = 0;
const wild_pokemon_t* next_pkm = NULL;


uint8_t transferByte(uint8_t out) {
  return SPI.transfer(out);
}

void setup() {
  //delay(5000);
  Serial.begin(115200);
  Serial.print("hello world\n");

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000, MSBFIRST, SPI_MODE3));

  GPS.begin(9600);
  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10s update rate
  //GPS.sendCommand(PGCMD_ANTENNA);
  
  gpstimer.begin(gpsread, 1000);  // read GPS every ms

  // Intitialiso/load persistent distance metrics
  if (EEPROM[EEPROM_REV_ADDR] == EEPROM_REV) {
    Serial.println("Loading data from EEPROM");
    EEPROM.get(EEPROM_ADDR, walk);
    next_encounter = walk.distance + random(MIN_WALK, MAX_WALK);
  } else {
    Serial.println("Initializing EEPROM");
    EEPROM[EEPROM_REV_ADDR] = EEPROM_REV;
    EEPROM.put(EEPROM_ADDR, walk);
  }



}

void gpsread(void) {
  GPS.read();
}

uint8_t next = PKMN_MASTER;
void loop() {
  uint8_t in = transferByte(next);
  next = handleIncomingByte(in);
  //Serial.print(in, HEX); Serial.print(" "); Serial.println(next, HEX);
  delay(100);
    
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());

    if (GPS.fix) {
      if(GPS.speed > 1 && GPS.speed < 20) { // Reasonable walking/running speeds (?)
        walk.distance += GPS.speed*5.144; // knots to meters per 10 seconds'
        //walk.distance+=100;
        if(walk.distance > next_encounter) {
            next_encounter = walk.distance + random(MIN_WALK, MAX_WALK);
            walk.encounter_idx++;
            int rn = random(-ENCOUNTER_RANDOMNESS, ENCOUNTER_RANDOMNESS);
            int id = walk.encounter_idx + rn;
            next_pkm = &wild_pokemon[id];
            Serial.print("Progress: "); Serial.print(walk.encounter_idx); Serial.print(" + "); Serial.println(rn);
            Serial.print("Pokemon: "); Serial.println(next_pkm->species, HEX);
            Serial.print("Level: "); Serial.println(next_pkm->level);
        }
      }
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitudeDegrees, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Distance walked(meter): "); Serial.println(walk.distance);
    }
  }
}

byte handleIncomingByte(byte in) {
  byte send = in; // By default just echo

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
    else if(in == PKMN_GO) {
      connection_state = POKEMON_GO;
    }
    else if(in == PKMN_BREAK_LINK || in == PKMN_MASTER) {
      connection_state = NOT_CONNECTED;
      send = PKMN_BREAK_LINK;
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
      if(counter == PLAYER_LENGTH) {
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
    }
    break;
  case POKEMON_GO:
    if (go_state == GO_INIT && in == 0xff) { // GameBoy disconnected
      Serial.println("GameBoy disconnected, saving...");
      EEPROM.put(EEPROM_ADDR, walk);
      connection_state = NOT_CONNECTED;
    } else if (go_state == GO_INIT && next_pkm == NULL) {
      send = SERIAL_NO_DATA_BYTE;
    } else if (go_state == GO_INIT && in == SERIAL_PREAMBLE_BYTE) {
      send = SERIAL_PREAMBLE_BYTE;
      go_state = GO_SEND;
      counter=0;
    } else if (go_state == GO_SEND && counter < 4) {
      send = SERIAL_PREAMBLE_BYTE;
    } else if (go_state == GO_SEND && counter == 4) {
      send = next_pkm->species;
    } else if (go_state == GO_SEND && counter == 5) {
      send = next_pkm->level;
      next_encounter = NULL;
      go_state = GO_INIT;
    }
    break;

  }

  return send;
}

