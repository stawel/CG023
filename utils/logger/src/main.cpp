#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <RF24_config.h>
#include "xn297.h"


// Hardware configuration
#define RF_CE_PIN                      (9)
#define RF_CS_PIN                      (10)
#define RF_IRQ_PIN                     (2)


//radio config
#define RF_PAYLOAD_SIZE             15+1   //+1 - package_nr
#define RF_CHANNEL                  5
#define RF_ADDR_WIDTH               5
#define RADIO_ID                    {0xcc, 0xcc, 0xcc, 0xcc, 0xcc}


//#define SERIAL_BAUDRATE                   (115200)
//see ../arbitrary-baud/arbitrary-baud
#define SERIAL_BAUDRATE                   (2000000)

//send directly to serial
#define FAST_FORWARD
#define BINARY_OUTPUT
//#define PRINT_PACKAGE_NR

//#define ENABLE_2MBPS_DEBUG

// Set up nRF24L01 radio on SPI bus plus CE/CS pins
static RF24 radio(RF_CE_PIN, RF_CS_PIN);

uint8_t oldbuf[256];
int pos = 0;
int size = 0;
int chars = 0;
int current_package_nr = 0;

void dumpData(uint8_t* p, int len)
{
#ifdef FAST_FORWARD
    Serial.write('P'+128);
    Serial.write(p,len);
#else

#ifndef BINARY_OUTPUT
    while (len--) { printf("%02x", *p++); }
    Serial.println();
#else


    size -= pos;
    for(int i = 0; i<size;i++) {
        oldbuf[i]=oldbuf[i+pos];
    }
    pos = 0;

    uint8_t package_nr = p[0];
    if((package_nr - current_package_nr + 256) % 256 != 1) {
        Serial.println();
        Serial.print("[PACKAGE LOST, last: ");
        Serial.print(current_package_nr);
        Serial.print(" new: ");
        Serial.print(package_nr);
        Serial.println(" ]");
    }
    current_package_nr = package_nr;

#ifdef PRINT_PACKAGE_NR
    Serial.print('[');
    Serial.print(package_nr);
    Serial.print(']');
#endif

    for(int i = 1; i<len;i++) {
        oldbuf[size++]=p[i];
    }


    for(;pos<size;pos++) {
    if(oldbuf[pos] > 128) {
        if(size-pos < 5) break;
        char c = oldbuf[pos]-128;
        if(c == 'i') {
            int32_t data;
            uint8_t * d_ptr = (uint8_t *)&data;
            pos++;
            d_ptr[0]=oldbuf[pos++];
            d_ptr[1]=oldbuf[pos++];
            d_ptr[2]=oldbuf[pos++];
            d_ptr[3]=oldbuf[pos];
            Serial.print(data);
            chars+=4;
        } else if (c == 'f') {
            float data;
            uint8_t * d_ptr = (uint8_t *)&data;
            pos++;
            d_ptr[0]=oldbuf[pos++];
            d_ptr[1]=oldbuf[pos++];
            d_ptr[2]=oldbuf[pos++];
            d_ptr[3]=oldbuf[pos];
            Serial.print(data);
            chars+=4;
        } else if (c == '8') {
            pos++;
            Serial.print(((uint8_t)oldbuf[pos]));
            chars+=1;
        } else if (c == 'F') {
            Serial.println();
            Serial.println("[COPTER BUFFER FULL]");
        }
    } else if (oldbuf[pos] == '\n') {
        Serial.println();
    } else {
        if(oldbuf[pos] != 0)
            Serial.write(oldbuf[pos]);
    }
    chars++;
        if(chars >= RF_PAYLOAD_SIZE) {
            chars -= RF_PAYLOAD_SIZE;
//            Serial.print('|');
        }
    }
#endif //BINARY_OUTPUT

#endif //FAST_FORWARD
}

void handleNrfIrq()
{
    // Loop until RX buffer(s) contain no more packets.
    while (radio.available()) {
        uint8_t packetLen = radio.getPayloadSize();
        uint8_t buf[RF_PAYLOAD_SIZE];
        if (packetLen > RF_PAYLOAD_SIZE)
            packetLen = RF_PAYLOAD_SIZE;

        radio.read(buf, packetLen);
        xn297_scramble_data(buf, packetLen, RF_ADDR_WIDTH);
        dumpData(buf, packetLen);
    }
}  

void setupRadio(void)
{
    radio.begin();
    // Disable shockburst
    radio.setAutoAck(false);
    radio.setRetries(0, 0);

    // Configure nRF IRQ input
    pinMode(RF_IRQ_PIN, INPUT);


    // radio config
    radio.setChannel(RF_CHANNEL);

#ifdef ENABLE_2MBPS_DEBUG
    radio.setDataRate(RF24_2MBPS);
#else
    radio.setDataRate(RF24_1MBPS);
#endif

    radio.setCRCLength(RF24_CRC_DISABLED);
    radio.setPayloadSize(RF_PAYLOAD_SIZE);

    radio.setAddressWidth(RF_ADDR_WIDTH);
    uint8_t  adr_rf[RF_ADDR_WIDTH] = RADIO_ID;

    xn297_scramble_address(adr_rf, RF_ADDR_WIDTH);
    radio.openReadingPipe(0, *((uint64_t*)adr_rf));
    radio.startListening();
}


int my_putc( char c, FILE *t ){
  Serial.write(c);
}

void setup(void)
{
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("-- start --");
    //printf
    fdevopen( &my_putc, 0);

    setupRadio();
}

void loop(void)
{
    handleNrfIrq();
}
