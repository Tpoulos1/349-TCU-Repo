// TCU Project

#include <SPI.h>
#include <TimeLib.h>
#include <TimerOne.h>

// define specific pin numbers later &&

#define SPI1 1// Serial clock
#define SPI2 2// Master out Slave in
#define SPI3 3// Master in Slave out

#define GPIO 4// Sync pin for SPI

#define SyncIN 5
#define SyncOUT 6



void setup(){

    Serial.begin(38400); // baud rate subject to change &&

    pinMode(SPI1, OUTPUT);
    pinMode(SPI2, OUTPUT);
    pinMode(SPI3, INPUT);

    pinMode(GPIO, OUTPUT);

    pinMode(SyncIN, INPUT);
    pinmode(SyncIN, OUTPUT);

    Timer1.initialize(1000);         // trigger every 1000 microseconds = 1ms
    Timer1.attachInterrupt([]() {    // this runs automatically every 1ms
    ms++;
    });

}

void loop(){

}