#include "pokemon.h"
#include "output.h"

#define MOSI_ 22
#define MISO_ 23
#define SCLK_ 24

volatile int bytes = 0;
volatile uint8_t shift = 0;
volatile uint8_t in_data = 0;
volatile uint8_t out_data = 0;

void transferBit(void) {
    in_data |= digitalRead(MISO_) << (7-shift);

    if(++shift > 7) {
        shift = 0;
        bytes++;
        out_data = handleIncomingByte(in_data);
        //Serial.print(bytes);
        //Serial.print(" ");
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
    
    //Serial.print("0");
    digitalWrite(MOSI_, LOW);
    out_data <<= 1;

}

void loop() {
    while(digitalRead(SCLK_));
    transferBit();
}

volatile connection_state_t connection_state = NOT_CONNECTED;
volatile trade_centre_state_t trade_centre_state = INIT;
volatile int counter = 0;
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
			if(counter++ == 5) {
				trade_centre_state = READY_TO_GO;
			}
			send = in;
		} else if(trade_centre_state == READY_TO_GO && (in & 0xF0) == 0xF0) {
			trade_centre_state = SEEN_FIRST_WAIT;
			send = in;
		} else if(trade_centre_state == SEEN_FIRST_WAIT && (in & 0xF0) != 0xF0) {
			// TODO - send some random data instead of mirroring
			send = in;

			counter = 0;
			trade_centre_state = SENDING_RANDOM_DATA;
		} else if(trade_centre_state == SENDING_RANDOM_DATA && (in & 0xF0) == 0xF0) {
			if(counter++ == 5) {
				trade_centre_state = WAITING_TO_SEND_DATA;
			}
			send = in;
		} else if(trade_centre_state == WAITING_TO_SEND_DATA && (in & 0xF0) != 0xF0) {
			counter = 0;
			// send first byte
			send = DATA_BLOCK[counter++];
			trade_centre_state = SENDING_DATA;
		} else if(trade_centre_state == SENDING_DATA) {
			send = DATA_BLOCK[counter++];
			if(counter == 415) {
				trade_centre_state = DATA_SENT;
			}
		} else {
			send = in;
		}
		break;

	case COLOSSEUM:
		send = in;
		break;

	default:
		send = in;
		break;
	}

	return send;
}
