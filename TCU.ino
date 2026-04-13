// TCU Project

#include <SPI.h>
#include <TimerOne.h>

// define specific pin numbers later &&

#define GPIO 4// Sync pin for SPI

#define SyncIN 5
#define SyncOUT 6


void sendFrame(long funcID) {
    SPI.beginTransaction(SPISettings(15625, MSBFIRST, SPI_MODE0));
    digitalWrite(GPIO, LOW);
    SPI.transfer(0x00);   // bits 0-7
    SPI.transfer(0x00);   // bits 8-12 
    digitalWrite(GPIO, HIGH);
    SPI.endTransaction();

    configBits = 0b11101;
    // Pack 5 bits into 1 byte, MSB-aligned: [b4 b3 b2 b1 b0 x x x]
    uint8_t payload = (configBits & 0x1F) << 3;

    SPI.beginTransaction(SPISettings(19531, MSBFIRST, SPI_MODE0));
    digitalWrite(GPIO, LOW);
    SPI.transfer(payload);  // 5 meaningful bits, 3 padding zeros at end
    digitalWrite(GPIO, HIGH);
    SPI.endTransaction();
}

int controller_sel = 0;// used to decide if this TCU is the controller or peripheral

//INSERT SPECIFIC FUNCTION ID FOR THE FUNCTIONS
//Everything else before that is the same for each function
//Turn module on, set phase to 0, set antenna to 0. (13 bits)

// CLOCK CONFIG ---- 

// volatile because to have accurate timing you need the cpu to not optimize it
volatile unsigned long ticks = 0;

// used for the Attatchinterupt function
void onTick() {
    ticks++; 
}

// function to retrieve time without screwing up overall timing
unsigned long getTime() {
    noInterrupts();
    unsigned long t = ticks;
    interrupts();
    return t;
}

// SYNC IN/OUT LOGIC -----
volatile unsigned long sequenceStart = 0;

void onSyncReceived() {
    sequenceStart = ticks;
}

void sendSync() {
    sequenceStart = ticks;       
    digitalWrite(SyncOUT, HIGH); 
    digitalWrite(SyncOUT, LOW);
}



void setup(){

    Serial.begin(38400); // baud rate subject to change &&

    pinMode(SPI1, OUTPUT);
    pinMode(SPI2, OUTPUT);
    pinMode(SPI3, INPUT);

    pinMode(GPIO, OUTPUT);

    pinMode(SyncIN, INPUT);
    pinmode(SyncOUT, OUTPUT);

    Timer1.initialize(1);
    Timer1.attachInterrupt(onTick);

    SPI.begin();

    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);

    if(controller_sel){
        sendSync();
    }
}

void loop(){
    sendFrame(long 0b0011001);
}
