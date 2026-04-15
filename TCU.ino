// TCU Project
#include <SPI.h>
#include <TimerOne.h>

// define specific pin numbers later &&

#define GPIO 4 //spi chip select
#define STROBE 7
#define SyncIN 5
#define SyncOUT 6


void sendFrame(uint8_t DATA) { 
    digitalWrite(STROBE,HIGH);
    SPI.beginTransaction(SPISettings(15625, MSBFIRST, SPI_MODE0));
    digitalWrite(GPIO, LOW);
    SPI.transfer(DATA);
    digitalWrite(GPIO, HIGH);
    SPI.endTransaction();
    digitalWrite(STROBE,LOW);
}

int controller_sel = 0;// used to decide if this TCU is the controller or peripheral

//INSERT SPECIFIC FUNCTION ID FOR THE FUNCTIONS
//Everything else before that is the same for each function
//Turn module on, set phase to 0, set antenna to 0. (13 bits)

// CLOCK CONFIG ---- 

// volatile because to have accurate timing you need the cpu to not optimize it
volatile unsigned long ticks = 0;
unsigned long lastSlot = 0;

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
    delayMicroseconds(10);      // wide enough for peripheral to catch it
    digitalWrite(SyncOUT, LOW);
}



void setup(){

    Serial.begin(38400); // baud rate subject to change &&


    pinMode(GPIO, OUTPUT);
    pinMode(STROBE,OUTPUT);
    pinMode(SyncIN, INPUT);
    pinMode(SyncOUT, OUTPUT);

    digitalWrite(GPIO,HIGH);
    digitalWrite(STROBE,LOW);
    

    Timer1.initialize(1);
    Timer1.attachInterrupt(onTick);

    SPI.begin();

    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);

    if(controller_sel){
        sendSync();
    }
}

void loop(){
    unsigned long now = getTime();
    unsigned long elapsed  = now - sequenceStart;
    unsigned long thisSlot = elapsed / 64;  

    if (thisSlot > lastSlot) {
        lastSlot = thisSlot;
        sendFrame(0x00);        // fires once per slot, locked to sequenceStart
    }

    // controller re-syncs periodically so clocks don't drift
    if (controller_sel && elapsed > 10000) {  
        sendSync();
        lastSlot = 0;
    }
}
