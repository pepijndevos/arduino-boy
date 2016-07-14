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
int counter = 0;

int trade_pokemon = -1;

#define EEPROM_DIST_ADDR 86
#define MIN_WALK 20
#define MAX_WALK 100
#define ENCOUNTER_RANDOMNESS 10
float distance_walked = 0;
float next_encounter = 0;
unsigned int encounter_idx = ENCOUNTER_RANDOMNESS;

uint8_t transferByte(uint8_t out) {
  return SPI.transfer(out);
}

void setup() {
  Serial.begin(115200);
  Serial.print("hello world\n");

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000, MSBFIRST, SPI_MODE3));

  GPS.begin(9600);
  GPS.sendCommand(PMTK_ENABLE_SBAS);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10s update rate
  //GPS.sendCommand(PGCMD_ANTENNA);
  gpstimer.begin(gpsread, 1000);  // blinkLED to run every 0.15 seconds

  //EEPROM.put(EEPROM_DIST_ADDR, distance_walked);
  EEPROM.get(EEPROM_DIST_ADDR, distance_walked);
  next_encounter = distance_walked + random(MIN_WALK, MAX_WALK);
}

void gpsread(void) {
  GPS.read();
}

uint8_t next = PKMN_MASTER;
void loop() {
  if(connection_state!=POKEMON_GO) {
    uint8_t in = transferByte(next);
    next = handleIncomingByte(in);
    //Serial.print(in, HEX);
    //Serial.print(" ");
    //Serial.print(next, HEX);
    //Serial.print("\n");
    delay(100);
  } else {
    pokemon_go();
  }
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    if (GPS.fix) {
      if(GPS.speed > 1 && GPS.speed < 20) // Reasonable walking/running speeds (?)
        distance_walked += GPS.speed*5.144; // knots to meters per 10 seconds
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitudeDegrees, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Distance walked(meter): "); Serial.println(distance_walked);
    }
  }
}
 
#define R 6371
#define TO_RAD (3.1415926536 / 180)
double dist(double th1, double ph1, double th2, double ph2)
{
  double dx, dy, dz;
  ph1 -= ph2;
  ph1 *= TO_RAD, th1 *= TO_RAD, th2 *= TO_RAD;
 
  dz = sin(th1) - sin(th2);
  dx = cos(ph1) * cos(th1) - cos(th2);
  dy = sin(ph1) * cos(th1);
  return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
}

void pokemon_go(void) {
  if(distance_walked > next_encounter) {
      next_encounter = distance_walked + random(MIN_WALK, MAX_WALK);
      EEPROM.get(EEPROM_DIST_ADDR, distance_walked);
      int id = encounter_idx + random(-ENCOUNTER_RANDOMNESS, ENCOUNTER_RANDOMNESS);
      wild_pokemon_t pkm = wild_pokemon[id];
      Serial.println(pkm.species, HEX); 
      Serial.println(pkm.level); 
      transferByte(pkm.species);
      delay(100);
      transferByte(pkm.level);
  }
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
    else if(in == PKMN_GO)
      connection_state = POKEMON_GO;
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

