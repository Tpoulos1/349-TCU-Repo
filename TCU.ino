// TCU Project

#include <SPI.h>
#include <TimerOne.h>

// define specific pin numbers later &&

#define SPI1 1// Serial clock
#define SPI2 2// Master out Slave in
#define SPI3 3// Master in Slave out

#define GPIO 4// Sync pin for SPI

#define SyncIN 5
#define SyncOUT 6

int controller_sel = 0;// used to decide if this TCU is the controller or peripheral

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

    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);

    if(controller_sel){
        sendSync();
    }
}

void loop(){

}