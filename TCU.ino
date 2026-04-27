// TCU Project
#include <SPI.h>

// Pin definitions
#define CHIP_SEL    10   // SPI chip select
#define STROBE       9   // tells receiver to latch data
#define SyncIN      11   // receives sync pulse from controller
#define SyncOUT     12   // sends sync pulse to peripheral

// Timing
#define PREAMBLE_US    1600UL  // preamble duration
#define PREAMBLE_SLOTS 25UL    // preamble is always 25 slots regardless of k
#define EL_DATA_US     4000UL  // EL data duration
#define EL_TOTAL_US    5600UL  // preamble + data

#define SEQ1_DUR_US    66700UL
#define SEQ2_DUR_US    66800UL

#define EL2_OFFSET_US  30000UL  // middle EL start time within sequence &&

// mode select — change currentMode to switch between functions
#define MODE_EL  0
#define MODE_AZ  1
#define MODE_BAZ 2

// bit positions in the SPI byte
#define TOFRO   (1 << 5)  // triggers switch between to/fro
#define SBSTART (1 << 4)  // scanning beam start
#define TXEN    (1 << 7)  // tx enable — on unless in a pause or gap

int currentMode = MODE_EL;

// special strobes fire at exact microsecond times that dont land on clean slot boundaries
const unsigned long AZ_SPECIAL_STROBES_US[] = {
    8760,   // PAUSE
    9360,   // FRO SCAN
    15900   // END
};
#define AZ_SPECIAL_COUNT 3
int azStrobeIdx = 0;

const unsigned long EL_SPECIAL_STROBES_US[] = {
    3406,   // PAUSE start
    3806,   // SB back on
    5600    // END
};
#define EL_SPECIAL_COUNT 3
int elStrobeIdx = 0;

// DATAWORD DEFINITIONS 
const uint8_t BDW1_BITS[] = {1,0,1,1,1,1,0,0,1,1,1,0,0,1,0,1,1,1,1,1};
const uint8_t BDW2_BITS[] = {1,0,0,0,0,1,1,0,0,1,0,0,1,0,1,0,0,1,0,0};
const uint8_t BDW3_BITS[] = {1,1,1,0,0,1,0,1,1,0,1,0,1,1,0,1,1,0,0,0};
const uint8_t BDW4_BITS[] = {1,1,1,1,0,0,1,0,1,1,1,0,0,1,0,1,0,1,0,1};
const uint8_t BDW5_BITS[] = {0,1,1,1,0,0,1,1,0,1,1,0,1,0,0,1,1,0,1,0};
const uint8_t BDW6_BITS[] = {1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,1,1,0};
const uint8_t ADWA1_BITS[] = { 0,0,0,0,1,1,1,1,1,0,0,1,1,0,1,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,0,1,0,1,1,0, 0,0,1,1,0,0,1,1,1,1,0,0,0,0,0,0};

#define BDW_BITS_COUNT  20
#define ADWA_BITS_COUNT 64
#define BDW_DATA_US    (BDW_BITS_COUNT  * 64UL)   // 1280µs
#define ADWA_DATA_US   (ADWA_BITS_COUNT * 64UL)   // 4096µs
#define BDW_TOTAL_US   (PREAMBLE_US + BDW_DATA_US)   // 2880µs
#define ADWA_TOTAL_US  (PREAMBLE_US + ADWA_DATA_US)  // 5696µs

// which CYCLE step each dataword fires in &&
#define BDW1_STEP    1
#define BDW2_STEP    1
#define BDW3_STEP    3
#define BDW4_STEP    3
#define BDW5_STEP    3
#define BDW6_STEP    5
#define ADWA1_STEP   5

// offset from start of that step in µs &&
#define BDW1_OFFSET   0UL
#define BDW2_OFFSET   0UL
#define BDW3_OFFSET   0UL
#define BDW4_OFFSET   0UL
#define BDW5_OFFSET   0UL
#define BDW6_OFFSET   0UL
#define ADWA1_OFFSET  0UL

// each step is either a sequence (isSeq=true) or a silent gap (isSeq=false)
struct Step {
    bool          isSeq;
    unsigned long duration;
};

// dataword struct — holds bit data, length, which step it fires in, offset, and total duration
struct DataWord {
    const uint8_t* bits;
    int            numBits;
    int            step;
    unsigned long  offset;
    unsigned long  totalUs;
};

const DataWord DATAWORDS[] = {
    {BDW1_BITS,  BDW_BITS_COUNT,  BDW1_STEP,  BDW1_OFFSET,  BDW_TOTAL_US},
    {BDW2_BITS,  BDW_BITS_COUNT,  BDW2_STEP,  BDW2_OFFSET,  BDW_TOTAL_US},
    {BDW3_BITS,  BDW_BITS_COUNT,  BDW3_STEP,  BDW3_OFFSET,  BDW_TOTAL_US},
    {BDW4_BITS,  BDW_BITS_COUNT,  BDW4_STEP,  BDW4_OFFSET,  BDW_TOTAL_US},
    {BDW5_BITS,  BDW_BITS_COUNT,  BDW5_STEP,  BDW5_OFFSET,  BDW_TOTAL_US},
    {BDW6_BITS,  BDW_BITS_COUNT,  BDW6_STEP,  BDW6_OFFSET,  BDW_TOTAL_US},
    {ADWA1_BITS, ADWA_BITS_COUNT, ADWA1_STEP, ADWA1_OFFSET, ADWA_TOTAL_US},
};
#define NUM_DATAWORDS 7

// full repeating cycle — 8 sequences and 8 gaps
const Step CYCLE[] = {
    {true,  SEQ1_DUR_US},  // step 0
    {false, 1000UL},       // step 1
    {true,  SEQ2_DUR_US},  // step 2
    {false, 13000UL},      // step 3
    {true,  SEQ1_DUR_US},  // step 4
    {false, 19000UL},      // step 5
    {true,  SEQ2_DUR_US},  // step 6
    {false, 2000UL},       // step 7
    {true,  SEQ1_DUR_US},  // step 8
    {false, 20000UL},      // step 9
    {true,  SEQ2_DUR_US},  // step 10
    {false, 6000UL},       // step 11
    {true,  SEQ1_DUR_US},  // step 12
    {false, 0UL},          // step 13
    {true,  SEQ2_DUR_US},  // step 14
    {false, 18000UL},      // step 15
};
#define CYCLE_LEN 16

int currentStep = 0; // which step of the cycle we are on

// k=1 is standard 64µs slots, k=10 means 640µs slots etc &&
int k = 1;
const uint8_t DPSK_BARKER[] = {1, 1, 1, 0, 1};

// preamble is the same for every function — TXEN always on, barker code on slots 13-17
uint8_t preamble(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t b = 0x00;
    b |= (1 << 7);
    b |= kBit;
    if (slot >= 13 && slot <= 17) {
        b |= (DPSK_BARKER[slot - 13] << 6);
    }
    return b;
}

// DPSK encode a bit at a given index — ref=0, 1=flip
uint8_t dpskEncode(const uint8_t* bits, int bitIndex) {
    uint8_t ref = 0;
    for (int i = 0; i <= bitIndex; i++) {
        if (bits[i] == 1) ref ^= 1;
    }
    return ref;
}

// returns the correct byte for a dataword slot — preamble first then DPSK encoded bits
uint8_t datawordByte(const uint8_t* bits, int numBits, unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;

    if (slot < PREAMBLE_SLOTS) {
        return preamble(slot);
    }

    unsigned long dataSlot = slot - PREAMBLE_SLOTS;
    if (dataSlot >= (unsigned long)numBits) {
        return 0x00;
    }

    uint8_t bit = dpskEncode(bits, dataSlot);
    uint8_t b = 0x00;
    b |= TXEN;
    b |= (bit << 6);  // DPSK on pin 2
    b |= kBit;
    return b;
}

// AZ data — antenna cycles through 5 patterns then holds, scanning beam active between 2.560ms and pause
uint8_t azData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b001, 0b101};

    uint8_t ant;
    if (slot < 5) {
        ant = (ANT_SEQ[slot] << 1);
    } else {
        ant = (ANT_SEQ[4] << 1);
    }

    if (slot == 0) {
        return TXEN | kBit;
    }
    if (slot >= 1 && slot <= 6) {
        return TXEN | ant | kBit;
    }
    if (slot >= 7 && slot <= 14) {
        return TXEN | ant | kBit;
    }
    if (slot == 15) {
        return TXEN | TOFRO | SBSTART | ant | kBit;
    }
    if (slot >= 16 && slot < 112) {
        return TXEN | SBSTART | ant | kBit;
    }
    if (slot >= 112 && slot < 121) {
        return 0x00;  // pause — TXEN off
    }
    if (slot == 121) {
        return TXEN | TOFRO | SBSTART | ant | kBit;
    }
    if (slot >= 122 && slot < 223) {
        return TXEN | SBSTART | ant | kBit;
    }
    return 0x00;
}

// EL data — TXEN always on except during pause (slots 28-33)
uint8_t elData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;

    if (slot < 4) {
        return TXEN | kBit;
    }
    if (slot == 4) {
        return TXEN | TOFRO | SBSTART | kBit;
    }
    if (slot >= 5 && slot < 28) {
        return TXEN | SBSTART | kBit;
    }
    if (slot >= 28 && slot < 34) {
        return 0x00;  // pause — TXEN off
    }
    if (slot == 34) {
        return TXEN | TOFRO | SBSTART | kBit;
    }
    if (slot >= 35 && slot < 62) {
        return TXEN | SBSTART | kBit;
    }
    return 0x00;
}

bool spiSentForSlot = false; // prevents SPI from firing twice in the same slot

// sends one byte over SPI at 1MHz
void sendSPI(uint8_t DATA) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CHIP_SEL, LOW);
    SPI.transfer(DATA);
    digitalWrite(CHIP_SEL, HIGH);
    SPI.endTransaction();
}

// pulses STROBE to tell receiver to latch whatever it just received over SPI
void strobeNow() {
    digitalWrite(STROBE, HIGH);
    digitalWrite(STROBE, LOW);
}

int controller_sel = 0; // 0 = peripheral, 1 = controller

unsigned long getTime() {
    return micros();
}

// SYNC IN/OUT LOGIC -----
volatile unsigned long sequenceStart = 0;

// peripheral catches sync pulse and records the time
void onSyncReceived() {
    sequenceStart = micros();
}

// controller fires sync pulse at start of each cycle so both units stay aligned
void sendSync() {
    sequenceStart = micros();
    digitalWrite(SyncOUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(SyncOUT, LOW);
}

unsigned long phaseStart = 0;
unsigned long lastSlot   = 0;
int activeEL             = -1; // which EL firing is active — -1 = none, 0/1/2 = first/middle/last
int activeDW             = -1; // which dataword is active — -1 = none

// returns when each EL firing starts within the sequence
unsigned long elStart(int n) {
    if (n == 0) return 0;             // first EL — immediately
    if (n == 1) return EL2_OFFSET_US; // middle EL &&
    return 54400UL;                   // last EL — always ends at sequence boundary
}

// moves to next step in cycle table, resets all counters, resyncs if controller at end of full cycle
void advancePhase() {
    currentStep = (currentStep + 1) % CYCLE_LEN;
    if (currentStep == 0 && controller_sel) {
        sendSync();
    }
    phaseStart     = getTime();
    lastSlot       = 0;
    activeEL       = -1;
    activeDW       = -1;
    spiSentForSlot = false;
    azStrobeIdx    = 0;
    elStrobeIdx    = 0;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("TCU upload successful");

    pinMode(CHIP_SEL, OUTPUT);
    pinMode(STROBE,   OUTPUT);
    pinMode(SyncIN,   INPUT);
    pinMode(SyncOUT,  OUTPUT);

    digitalWrite(CHIP_SEL, HIGH);
    digitalWrite(STROBE,   LOW);

    SPI.begin();

    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);

    phaseStart = getTime();

    if (controller_sel) {
        sendSync();
    }
}

void loop() {
    unsigned long now      = getTime();
    unsigned long elapsed  = now - phaseStart;
    unsigned long slotSize = 64UL * (unsigned long)k;  // slot size scales with k

    // advance to next step when current step expires
    if (elapsed >= CYCLE[currentStep].duration) {
        advancePhase();
        return;
    }

    // skip 0ms gap steps immediately
    if (CYCLE[currentStep].duration == 0) {
        advancePhase();
        return;
    }

    unsigned long thisSlot   = elapsed / slotSize;
    unsigned long timeInSlot = elapsed % slotSize;

    uint8_t nextFrame = 0x00; // default — overridden if inside a function window

    if (CYCLE[currentStep].isSeq) {

        if (currentMode == MODE_EL) {
            // check all 3 EL firing windows
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                unsigned long end   = start + EL_TOTAL_US;

                if (elapsed >= start && elapsed < end) {
                    // reset slot counter when entering a new EL firing
                    if (activeEL != i) {
                        activeEL       = i;
                        lastSlot       = 0;
                        spiSentForSlot = false;
                        elStrobeIdx    = 0;
                    }

                    unsigned long elElapsed  = elapsed - start;
                    unsigned long elSlot     = elElapsed / slotSize;
                    unsigned long nextElSlot = elSlot + 1;

                    // first 25 slots = preamble, after that = EL data
                    if (nextElSlot < PREAMBLE_SLOTS) {
                        nextFrame = preamble(nextElSlot);
                    } else {
                        nextFrame = elData(nextElSlot - PREAMBLE_SLOTS);
                    }

                    // fire special strobes at exact spec times
                    if (elStrobeIdx < EL_SPECIAL_COUNT) {
                        if (elElapsed >= EL_SPECIAL_STROBES_US[elStrobeIdx]) {
                            sendSPI(nextFrame);
                            strobeNow();
                            elStrobeIdx++;
                        }
                    }

                    break;
                }
            }

        } else if (currentMode == MODE_AZ) {
            unsigned long start = 10000UL;  // AZ fires 10ms into each sequence
            unsigned long end   = start + 15900UL;

            if (elapsed >= start && elapsed < end) {
                if (activeEL != 0) {
                    activeEL       = 0;
                    lastSlot       = 0;
                    spiSentForSlot = false;
                    azStrobeIdx    = 0;
                }

                unsigned long azElapsed  = elapsed - start;
                unsigned long azSlot     = azElapsed / slotSize;
                unsigned long nextAzSlot = azSlot + 1;

                // first 25 slots = preamble, after that = AZ data
                if (nextAzSlot < PREAMBLE_SLOTS) {
                    nextFrame = preamble(nextAzSlot);
                } else {
                    nextFrame = azData(nextAzSlot - PREAMBLE_SLOTS);
                }

                // fire special strobes at exact spec times
                if (azStrobeIdx < AZ_SPECIAL_COUNT) {
                    if (azElapsed >= AZ_SPECIAL_STROBES_US[azStrobeIdx]) {
                        sendSPI(nextFrame);
                        strobeNow();
                        azStrobeIdx++;
                    }
                }
            }
        }
    }

    // check datawords — only in AZ or BAZ mode, works in both seq and gap steps
    if (currentMode == MODE_AZ || currentMode == MODE_BAZ) {
        for (int d = 0; d < NUM_DATAWORDS; d++) {
            if (currentStep != DATAWORDS[d].step) continue;

            unsigned long dwStart = DATAWORDS[d].offset;
            unsigned long dwEnd   = dwStart + DATAWORDS[d].totalUs;

            if (elapsed >= dwStart && elapsed < dwEnd) {
                if (activeDW != d) {
                    activeDW       = d;
                    lastSlot       = 0;
                    spiSentForSlot = false;
                }

                unsigned long dwElapsed  = elapsed - dwStart;
                unsigned long dwSlot     = dwElapsed / slotSize;
                unsigned long nextDWSlot = dwSlot + 1;

                nextFrame = datawordByte(
                    DATAWORDS[d].bits,
                    DATAWORDS[d].numBits,
                    nextDWSlot
                );
                break;
            }
        }

        // reset activeDW when outside all dataword windows
        if (activeDW != -1) {
            unsigned long dwEnd = DATAWORDS[activeDW].offset + DATAWORDS[activeDW].totalUs;
            if (elapsed >= dwEnd) {
                activeDW = -1;
            }
        }
    }

    // SPI fires at midpoint of slot so receiver has data before strobe
    if (timeInSlot >= (slotSize / 2) && !spiSentForSlot) {
        sendSPI(nextFrame);
        spiSentForSlot = true;
    }

    // strobe at slot boundary tells receiver to latch
    if (thisSlot > lastSlot) {
        lastSlot       = thisSlot;
        spiSentForSlot = false;
        strobeNow();
    }
}