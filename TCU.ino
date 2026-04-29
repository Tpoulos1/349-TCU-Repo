#include <SPI.h>

// pin definitions
#define CHIP_SEL    10
#define STROBE       9
#define SyncIN      11
#define SyncOUT     12

// timing
#define PREAMBLE_US    1600UL
#define PREAMBLE_SLOTS 25UL
#define EL_DATA_US     4000UL
#define EL_TOTAL_US    5600UL
#define SEQ1_DUR_US    66700UL
#define SEQ2_DUR_US    66800UL
#define EL2_OFFSET_US  30000UL // &&

// mode select
#define MODE_EL  0
#define MODE_AZ  1
#define MODE_BAZ 2

// bit positions in SPI byte
#define TOFRO   (1 << 5)
#define SBSTART (1 << 4)
#define TXEN    (1 << 7)

int currentMode = MODE_EL;

// special strobe times that dont land on clean slot boundaries
const unsigned long AZ_SPECIAL_STROBES_US[] = { 8760, 9360, 15900 };
#define AZ_SPECIAL_COUNT 3
int azStrobeIdx = 0;

const unsigned long EL_SPECIAL_STROBES_US[] = { 3406, 3806, 5600 };
#define EL_SPECIAL_COUNT 3
int elStrobeIdx = 0;

// dataword bit arrays pre-encoded in DPSK
const uint8_t DPSK_BARKER[] = {1, 0, 1, 1, 0};
const uint8_t BDW1_BITS[]   = {1,1,0,1,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0};
const uint8_t BDW2_BITS[]   = {1,1,1,1,1,0,1,1,1,0,0,0,1,1,0,0,0,1,1,1};
const uint8_t BDW3_BITS[]   = {1,0,1,1,1,0,0,1,0,0,1,1,0,1,1,0,1,1,1,1};
const uint8_t BDW4_BITS[]   = {1,0,1,0,0,0,1,1,0,1,0,0,0,1,1,0,0,1,1,0};
const uint8_t BDW5_BITS[]   = {0,1,0,1,1,1,0,1,1,0,1,1,0,0,0,1,0,0,1,1};
const uint8_t BDW6_BITS[]   = {1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,0,1,1};
const uint8_t ADWA1_BITS[]  = {0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1};

#define BDW_BITS_COUNT  20
#define ADWA_BITS_COUNT 64
#define BDW_TOTAL_US    3100UL
#define ADWA_TOTAL_US   5900UL

// dataword placement
#define BDW1_STEP  3
#define BDW2_STEP  3
#define BDW3_STEP  3
#define BDW4_STEP  3
#define BDW5_STEP  5
#define BDW6_STEP  5
#define ADWA1_STEP 5

#define BDW1_OFFSET  0UL
#define BDW2_OFFSET  3100UL
#define BDW3_OFFSET  6200UL
#define BDW4_OFFSET  9300UL
#define BDW5_OFFSET  0UL
#define BDW6_OFFSET  3100UL
#define ADWA1_OFFSET 6200UL

struct Step { bool isSeq; unsigned long duration; };

// dataword struct
struct DataWord {
    const uint8_t* bits;
    int numBits;
    int step;
    unsigned long offset;
    unsigned long totalUs;
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

// full repeating cycle of sequences and gaps
const Step CYCLE[] = {
    {true,  SEQ1_DUR_US}, // step 0
    {false, 1000UL},      // step 1
    {true,  SEQ2_DUR_US}, // step 2
    {false, 13000UL},     // step 3
    {true,  SEQ1_DUR_US}, // step 4
    {false, 19000UL},     // step 5
    {true,  SEQ2_DUR_US}, // step 6
    {false, 2000UL},      // step 7
    {true,  SEQ1_DUR_US}, // step 8
    {false, 20000UL},     // step 9
    {true,  SEQ2_DUR_US}, // step 10
    {false, 6000UL},      // step 11
    {true,  SEQ1_DUR_US}, // step 12
    {false, 0UL},         // step 13
    {true,  SEQ2_DUR_US}, // step 14
    {false, 18000UL},     // step 15
};
#define CYCLE_LEN 16

int currentStep = 0;
int k = 1; // &&

// preamble — TXEN always on, barker on slots 13-17
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

// dataword byte — preamble first then DPSK bits
uint8_t datawordByte(const uint8_t* bits, int numBits, unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    if (slot < PREAMBLE_SLOTS) return preamble(slot);
    unsigned long dataSlot = slot - PREAMBLE_SLOTS;
    if (dataSlot >= (unsigned long)numBits) return TXEN | kBit;
    uint8_t b = 0x00;
    b |= TXEN;
    b |= (bits[dataSlot] << 6);
    b |= kBit;
    return b;
}

// AZ data — antenna cycles slots 7-14, resets at SBSTART
uint8_t azData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b100, 0b101};
    uint8_t ant = 0x00;
    if (slot >= 7 && slot <= 11)       ant = (ANT_SEQ[slot - 7] << 1);
    else if (slot >= 12 && slot <= 14) ant = (ANT_SEQ[4] << 1);

    if (slot == 0)                 return TXEN | kBit;
    if (slot >= 1  && slot <= 6)   return TXEN | kBit;
    if (slot >= 7  && slot <= 14)  return TXEN | ant | kBit;
    if (slot == 15)                return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 16 && slot < 112)  return TXEN | SBSTART | kBit;
    if (slot >= 112 && slot < 121) return 0x00;
    if (slot == 121)               return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 122 && slot < 223) return TXEN | SBSTART | kBit;
    return 0x00;
}

// EL data — TXEN on except during pause slots 28-33
uint8_t elData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    if (slot < 4)                return TXEN | kBit;
    if (slot == 4)               return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 5  && slot < 28) return TXEN | SBSTART | kBit;
    if (slot >= 28 && slot < 34) return 0x00;
    if (slot == 34)              return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 35 && slot < 62) return TXEN | SBSTART | kBit;
    return 0x00;
}

// SPI send and strobe
void sendSPI(uint8_t DATA) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CHIP_SEL, LOW);
    SPI.transfer(DATA);
    digitalWrite(CHIP_SEL, HIGH);
    SPI.endTransaction();
}

void strobeNow() {
    digitalWrite(STROBE, HIGH);
    digitalWrite(STROBE, LOW);
}

// returns true if special strobe fires within 64µs
bool specialSoon(const unsigned long* strobes, int count, int idx, unsigned long elapsed) {
    if (idx >= count) return false;
    if (strobes[idx] <= elapsed) return false;
    return (strobes[idx] - elapsed) <= 64UL;
}

// globals declared before regularSlot so it can access them
unsigned long phaseStart = 0;
unsigned long lastSlot   = 0;
int activeEL             = -1;
int activeDW             = -1;
bool spiSentForSlot      = false;

// fires regular SPI at midpoint and strobe at boundary, skips both if special imminent
void regularSlot(uint8_t frame, unsigned long thisSlot, unsigned long timeInSlot,
                 unsigned long slotSize, bool specialImminent) {
    if (timeInSlot >= (slotSize / 2) && !spiSentForSlot && !specialImminent) {
        sendSPI(frame);
        spiSentForSlot = true;
    }
    if (thisSlot > lastSlot && !specialImminent) {
        lastSlot       = thisSlot;
        spiSentForSlot = false;
        strobeNow();
    }
}

int controller_sel = 0;
unsigned long getTime() { return micros(); }

// sync in/out
volatile unsigned long sequenceStart = 0;
void onSyncReceived() { sequenceStart = micros(); }

void sendSync() {
    sequenceStart = micros();
    digitalWrite(SyncOUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(SyncOUT, LOW);
}

// returns when each EL firing starts within a sequence
unsigned long elStart(int n) {
    if (n == 0) return 0;
    if (n == 1) return EL2_OFFSET_US;
    return 54400UL;
}

// advances to next step and resets counters
void advancePhase() {
    currentStep = (currentStep + 1) % CYCLE_LEN;
    if (currentStep == 0 && controller_sel) sendSync();
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

    pinMode(CHIP_SEL, OUTPUT); digitalWrite(CHIP_SEL, HIGH);
    pinMode(STROBE,   OUTPUT); digitalWrite(STROBE,   LOW);
    pinMode(SyncIN,   INPUT);
    pinMode(SyncOUT,  OUTPUT);

    SPI.begin();
    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);
    phaseStart = getTime();
    if (controller_sel) sendSync();
}

void loop() {
    unsigned long now      = getTime();
    unsigned long elapsed  = now - phaseStart;
    unsigned long slotSize = 64UL * (unsigned long)k;

    if (elapsed >= CYCLE[currentStep].duration) { advancePhase(); return; }
    if (CYCLE[currentStep].duration == 0)        { advancePhase(); return; }

    unsigned long thisSlot   = elapsed / slotSize;
    unsigned long timeInSlot = elapsed % slotSize;

    if (CYCLE[currentStep].isSeq) {

        if (currentMode == MODE_EL) {
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                unsigned long end   = start + EL_TOTAL_US;
                if (elapsed >= start && elapsed < end) {
                    if (activeEL != i) {
                        activeEL = i; lastSlot = 0;
                        spiSentForSlot = false; elStrobeIdx = 0;
                    }
                    unsigned long elElapsed = elapsed - start;
                    unsigned long elSlot    = elElapsed / slotSize;
                    uint8_t frame = (elSlot < PREAMBLE_SLOTS) ? preamble(elSlot) : elData(elSlot - PREAMBLE_SLOTS);

                    bool imminent = specialSoon(EL_SPECIAL_STROBES_US, EL_SPECIAL_COUNT, elStrobeIdx, elElapsed);
                    regularSlot(frame, thisSlot, timeInSlot, slotSize, imminent);

                    if (elStrobeIdx < EL_SPECIAL_COUNT && elElapsed >= EL_SPECIAL_STROBES_US[elStrobeIdx]) {
                        sendSPI(frame); strobeNow();
                        lastSlot = thisSlot; spiSentForSlot = true; elStrobeIdx++;
                    }
                    break;
                }
            }

        } else if (currentMode == MODE_AZ) {
            unsigned long start = 10000UL;
            unsigned long end   = start + 15900UL;
            if (elapsed >= start && elapsed < end) {
                if (activeEL != 0) {
                    activeEL = 0; lastSlot = 0;
                    spiSentForSlot = false; azStrobeIdx = 0;
                }
                unsigned long azElapsed = elapsed - start;
                unsigned long azSlot    = azElapsed / slotSize;
                uint8_t frame = (azSlot < PREAMBLE_SLOTS) ? preamble(azSlot) : azData(azSlot - PREAMBLE_SLOTS);

                bool imminent = specialSoon(AZ_SPECIAL_STROBES_US, AZ_SPECIAL_COUNT, azStrobeIdx, azElapsed);
                regularSlot(frame, thisSlot, timeInSlot, slotSize, imminent);

                if (azStrobeIdx < AZ_SPECIAL_COUNT && azElapsed >= AZ_SPECIAL_STROBES_US[azStrobeIdx]) {
                    sendSPI(frame); strobeNow();
                    lastSlot = thisSlot; spiSentForSlot = true; azStrobeIdx++;
                }
            }
        }

    } else {
        // gaps — send 0x00 on regular slot timing
        regularSlot(0x00, thisSlot, timeInSlot, slotSize, false);
    }

    // datawords fire in AZ or BAZ mode during any step
    if (currentMode == MODE_AZ || currentMode == MODE_BAZ) {
        for (int d = 0; d < NUM_DATAWORDS; d++) {
            if (currentStep != DATAWORDS[d].step) continue;
            unsigned long dwStart = DATAWORDS[d].offset;
            unsigned long dwEnd   = dwStart + DATAWORDS[d].totalUs;
            if (elapsed >= dwStart && elapsed < dwEnd) {
                if (activeDW != d) { activeDW = d; lastSlot = 0; spiSentForSlot = false; }
                unsigned long dwSlot = (elapsed - dwStart) / slotSize;
                uint8_t frame = datawordByte(DATAWORDS[d].bits, DATAWORDS[d].numBits, dwSlot);
                regularSlot(frame, thisSlot, timeInSlot, slotSize, false);
                break;
            }
        }
        if (activeDW != -1 && elapsed >= DATAWORDS[activeDW].offset + DATAWORDS[activeDW].totalUs)
            activeDW = -1;
    }
}