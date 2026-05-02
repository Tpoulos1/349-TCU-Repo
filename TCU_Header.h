#ifndef TCU_HEADER_H
#define TCU_HEADER_H

#include <Arduino.h>
#include <SPI.h>

// Pin definitions
// && marks pins requiring final hardware confirmation
#define CHIP_SEL  10  // &&
#define STROBE     9  // &&
#define SyncIN    11  // &&
#define SyncOUT   12  // &&

// SPI frame bit masks
#define TXEN    0x80
#define DPSK    0x40
#define TOFRO   0x20
#define SBSTART 0x10

// Timing constants
#define EL2_OFFSET_US  30000UL  // && placeholder, needs real value
#define PREAMBLE_SLOTS 25UL
#define EL_TOTAL_US    5600UL   // end-event boundary (spec value)
#define BDW_TOTAL_US   3100UL
#define ADWA_TOTAL_US  5900UL

// Sizing constants
#define BDW_BITS_COUNT   20
#define ADWA_BITS_COUNT  64
#define CYCLE_LEN        16
#define MAX_PHASE_EVENTS 15
#define NUM_DATAWORDS     7

// Mode selection
#define MODE_AZ 0
#define MODE_EL 1

// Structs
struct DataWord {
    const uint8_t* bits;
    int            numBits;
    int            step;
    unsigned long  offset;
    unsigned long  totalUs;
};

struct Step {
    bool          isSeq;
    unsigned long duration;
};

// Constant data (defined in TCU_main.cpp)
extern const uint8_t DPSK_BARKER[];
extern const uint8_t BDW1_BITS[];
extern const uint8_t BDW2_BITS[];
extern const uint8_t BDW3_BITS[];
extern const uint8_t BDW4_BITS[];
extern const uint8_t BDW5_BITS[];
extern const uint8_t BDW6_BITS[];
extern const uint8_t ADWA1_BITS[];
extern const DataWord DATAWORDS[];
extern const Step     CYCLE[];

// Global state (defined in TCU_main.cpp)
extern unsigned long phaseStart;
extern unsigned long lastSlot;
extern bool          spiSentForSlot;
extern bool          evSpiSent;
extern int           currentStep;
extern int           k;
extern int           currentMode;
extern int           controller_sel;
extern unsigned long phaseEvents[];
extern int           numPhaseEvents;
extern int           nextEventIdx;
extern volatile unsigned long sequenceStart;

// Function prototypes
void      sendSPI(uint8_t DATA);
void      strobeNow();
void      sendSync();
void      onSyncReceived();
void      advancePhase();
void      buildPhaseEvents();
uint8_t   preamble(unsigned long slot);
uint8_t   elData(unsigned long slot);
uint8_t   azData(unsigned long slot);
uint8_t   datawordByte(const uint8_t* bits, int numBits, unsigned long slot);
uint8_t   frameAt(unsigned long elapsed, bool isSeqStep, unsigned long slotSize, uint8_t kBit);
unsigned long elStart(int n);
void      regularSlot(uint8_t frame, unsigned long thisSlot, unsigned long timeInSlot,
                      unsigned long slotSize, bool specialImminent);

#endif // TCU_HEADER_H