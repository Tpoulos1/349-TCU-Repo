#ifndef TCU_HEADER_H
#define TCU_HEADER_H

#include <Arduino.h>
#include <SPI.h>

// pin definitions that USSIM cares about &&
#define CHIP_SEL  10
#define STROBE     9
// SCK = SPI clock
// MO = MOSI
// MI = MISO

// TCU sync pins
#define SyncIN    11
#define SyncOUT   12

// bit positions in SPI byte
#define TXEN    0x80
#define DPSK    0x40
#define TOFRO   0x20
#define SBSTART 0x10

// timing constants
#define EL2_OFFSET_US  30000UL  // &&
#define PREAMBLE_SLOTS 25UL
#define EL_TOTAL_US    5600UL
#define BDW_TOTAL_US   3100UL
#define ADWA_TOTAL_US  5900UL

// SEQ1 dataword offsets — same for AZ and BAZ mode
#define SEQ1_BDW1_START  5600UL
#define SEQ1_BAZ_BDW1    5600UL
#define SEQ1_BDW2_START  25900UL   // AZ only
#define SEQ1_BAZ_BDW2    50000UL   // BAZ only — locked after BAZ ends
#define SEQ1_BDW3_START  50000UL   // AZ
#define SEQ1_BAZ_BDW3    60000UL   // BAZ
#define SEQ1_BDW4_START  60000UL   // AZ
#define SEQ1_BAZ_BDW4    63100UL   // BAZ

// SEQ2 dataword offsets — same for AZ and BAZ mode
#define SEQ2_BDW5_START  5600UL
#define SEQ2_BDW6_START  25900UL
#define SEQ2_ADWA1_START 35600UL

// sizing constants
#define BDW_BITS_COUNT   20
#define ADWA_BITS_COUNT  64
#define CYCLE_LEN        16
#define MAX_PHASE_EVENTS 24

// mode select
#define MODE_AZ  0
#define MODE_EL  1
#define MODE_BAZ 2

struct Step {
    bool          isSeq;
    unsigned long duration;
};

// constant data
extern const uint8_t DPSK_BARKER[];
extern const uint8_t BDW1_BITS[];
extern const uint8_t BDW2_BITS[];
extern const uint8_t BDW3_BITS[];
extern const uint8_t BDW4_BITS[];
extern const uint8_t BDW5_BITS[];
extern const uint8_t BDW6_BITS[];
extern const uint8_t ADWA1_BITS[];
extern const Step CYCLE[];

// global state
extern unsigned long phaseStart;      // micros timestamp when current cycle step began
extern unsigned long lastSlot;        // last slot number that fired a strobe
extern bool          spiSentForSlot;  // prevents SPI firing twice in the same slot
extern bool          evSpiSent;       // prevents event SPI firing twice before its strobe
extern int           currentStep;     // which step in CYCLE we are on 
extern int           k;               // k constant
extern int           currentMode;     // MODE_AZ, MODE_EL, or MODE_BAZ
extern int           controller_sel;  // 0=peripheral 1=controller
extern unsigned long phaseEvents[];   // sorted list of special strobe times for current step
extern int           numPhaseEvents;  // how many events are in phaseEvents
extern int           nextEventIdx;    // index of the next event yet to fire
extern volatile unsigned long sequenceStart; // micros when sync pulse was received

// scales a microsecond value by k. Set as inline so the compiler directly substitutes the function code in to remove overhead timing delays
inline unsigned long scale(unsigned long us) {
    return us * (unsigned long)k;
}

// function prototypes
void      sendSPI(uint8_t DATA);
void      strobeNow();
void      sendSync();
void      onSyncReceived();
void      advancePhase();
void      buildPhaseEvents();
uint8_t   preamble(unsigned long slot);
uint8_t   elData(unsigned long slot);
uint8_t   azData(unsigned long slot);
uint8_t   bazData(unsigned long slot);
uint8_t   datawordByte(const uint8_t* bits, int numBits, unsigned long slot);
uint8_t   frameAt(unsigned long elapsed, bool isSeqStep, unsigned long slotSize, uint8_t kBit);
unsigned long elStart(int n);
void      regularSlot(uint8_t frame, unsigned long thisSlot, unsigned long timeInSlot, unsigned long slotSize, bool specialImminent);

#endif