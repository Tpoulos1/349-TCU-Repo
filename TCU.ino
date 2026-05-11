// TCU_main.cpp

#include "tcu_header.h"

// constant data

const uint8_t DPSK_BARKER[] = {1, 0, 1, 1, 0};

// pre-encoded DPSK bit arrays for datawords
const uint8_t BDW1_BITS[]  = {1,1,0,1,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0};
const uint8_t BDW2_BITS[]  = {1,1,1,1,1,0,1,1,1,0,0,0,1,1,0,0,0,1,1,1};
const uint8_t BDW3_BITS[]  = {1,0,1,1,1,0,0,1,0,0,1,1,0,1,1,0,1,1,1,1};
const uint8_t BDW4_BITS[]  = {1,0,1,0,0,0,1,1,0,1,0,0,0,1,1,0,0,1,1,0};
const uint8_t BDW5_BITS[]  = {0,1,0,1,1,1,0,1,1,0,1,1,0,0,0,1,0,0,1,1};
const uint8_t BDW6_BITS[]  = {1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,0,1,1};
const uint8_t ADWA1_BITS[] = {0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1};

// full repeating cycle of sequences and gaps
const Step CYCLE[CYCLE_LEN] = {
    {true,  66700UL},  // step 0  — SEQ1
    {false,  1000UL},  // step 1  — GAP
    {true,  66800UL},  // step 2  — SEQ2
    {false, 13000UL},  // step 3  — GAP
    {true,  66700UL},  // step 4  — SEQ1
    {false, 19000UL},  // step 5  — GAP
    {true,  66800UL},  // step 6  — SEQ2
    {false,  2000UL},  // step 7  — GAP
    {true,  66700UL},  // step 8  — SEQ1
    {false, 20000UL},  // step 9  — GAP
    {true,  66800UL},  // step 10 — SEQ2
    {false,  6000UL},  // step 11 — GAP
    {true,  66700UL},  // step 12 — SEQ1
    {false,  1000UL},  // step 13 — GAP
    {true,  66800UL},  // step 14 — SEQ2
    {false, 18000UL},  // step 15 — GAP
};

// global state
unsigned long phaseStart      = 0;
unsigned long lastSlot        = 0;
bool          spiSentForSlot  = false;
bool          evSpiSent       = false;
int           currentStep     = 0;
int           k               = 1;     // &&
int           currentMode     = MODE_AZ;
int           controller_sel  = 0;

unsigned long phaseEvents[MAX_PHASE_EVENTS];
int           numPhaseEvents  = 0;
int           nextEventIdx    = 0;

volatile unsigned long sequenceStart = 0;

// SPI send and strobe
void sendSPI(uint8_t DATA) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CHIP_SEL, LOW);
    SPI.transfer(DATA);
    digitalWrite(CHIP_SEL, HIGH);
    SPI.endTransaction();
}

void strobeNow() {
    digitalWrite(STROBE,HIGH);
    digitalWrite(STROBE,LOW);
}

// sync in/out
void onSyncReceived() {
    sequenceStart = micros();
}

void sendSync() {
    sequenceStart = micros();
    digitalWrite(SyncOUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(SyncOUT, LOW);
}

// returns when each EL firing starts within a sequence
unsigned long elStart(int n) {
    if (n == 0) return 0UL;
    if (n == 1) return EL2_OFFSET_US;  // &&
    return 54400UL;
}

// preamble — D5 during non-barker slots, D0 + DPSK during slots 13-17
uint8_t preamble(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D5   = (0b101 << 1);
    if (slot >= 13 && slot <= 17) {
        return TXEN | (uint8_t)(DPSK_BARKER[slot - 13] << 6) | kBit;
    }
    return TXEN | D5 | kBit;
}

// EL data — D2 for OCI, D4 for scanning beam, D5 for pause/off
uint8_t elData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D2   = (0b010 << 1);
    uint8_t D4   = (0b100 << 1);
    uint8_t D5   = (0b101 << 1);
    if (slot < 2)                return TXEN | D5 | kBit;
    if (slot >= 2 && slot < 4)   return TXEN | D2 | kBit;
    if (slot == 4)               return TXEN | TOFRO | SBSTART | D4 | kBit;
    if (slot >= 5  && slot < 28) return TXEN | SBSTART | D4 | kBit;
    if (slot >= 28 && slot < 34) return D5 | kBit;
    if (slot == 34)              return TXEN | TOFRO | SBSTART | D4 | kBit;
    if (slot >= 35 && slot < 62) return TXEN | SBSTART | D4 | kBit;
    return D5 | kBit;
}

// AZ data — OCI cycles 1,2,3,0,5 then D5 everywhere else
uint8_t azData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D5   = (0b101 << 1);
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b000, 0b101};
    uint8_t ant = D5;
    if (slot >= 7  && slot <= 11) ant = (uint8_t)(ANT_SEQ[slot - 7] << 1);
    else if (slot >= 12 && slot <= 14) ant = (uint8_t)(ANT_SEQ[4] << 1);

    if (slot <= 6)                 return TXEN | D5 | kBit;
    if (slot >= 7  && slot <= 14)  return TXEN | ant | kBit;
    if (slot == 15)                return TXEN | TOFRO | SBSTART | D5 | kBit;
    if (slot >= 16 && slot < 112)  return TXEN | SBSTART | D5 | kBit;
    if (slot >= 112 && slot < 121) return D5 | kBit;
    if (slot == 121)               return TXEN | TOFRO | SBSTART | D5 | kBit;
    if (slot >= 122 && slot < 223) return TXEN | SBSTART | D5 | kBit;
    return D5 | kBit;
}

// BAZ data — same structure as AZ but different pause/fro/end slot boundaries
uint8_t bazData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D5   = (0b101 << 1);
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b000, 0b101};
    uint8_t ant = D5;
    if (slot >= 7  && slot <= 11) ant = (uint8_t)(ANT_SEQ[slot - 7] << 1);
    else if (slot >= 12 && slot <= 14) ant = (uint8_t)(ANT_SEQ[4] << 1);

    if (slot <= 6)                return TXEN | D5 | kBit;
    if (slot >= 7  && slot <= 14) return TXEN | ant | kBit;
    if (slot == 15)               return TXEN | TOFRO | SBSTART | D5 | kBit;
    if (slot >= 16 && slot < 80)  return TXEN | SBSTART | D5 | kBit;
    if (slot >= 80 && slot < 90)  return D5 | kBit;
    if (slot == 90)               return TXEN | TOFRO | SBSTART | D5 | kBit;
    if (slot >= 91 && slot < 161) return TXEN | SBSTART | D5 | kBit;
    return D5 | kBit;
}

// dataword byte — preamble first then DPSK bits
uint8_t datawordByte(const uint8_t* bits, int numBits, unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    if (slot < PREAMBLE_SLOTS) return preamble(slot);
    unsigned long dataSlot = slot - PREAMBLE_SLOTS;
    if (dataSlot >= (unsigned long)numBits) return TXEN | kBit;
    return TXEN | (uint8_t)(bits[dataSlot] << 6) | kBit;
}

// helper — check and return dataword frame if elapsed is within a dataword window
static uint8_t checkDW(unsigned long elapsed, unsigned long start, const uint8_t* bits,
                        int numBits, unsigned long total, unsigned long slotSize) {
    if (elapsed >= start && elapsed < start + total) {
        unsigned long s = (elapsed - start) / slotSize;
        return datawordByte(bits, numBits, s);
    }
    return 0xFF;  // sentinel — not in this window
}

// returns correct frame byte for current elapsed time and mode
uint8_t frameAt(unsigned long elapsed, bool isSeqStep, unsigned long slotSize, uint8_t kBit) {
    if (!isSeqStep) return kBit;  // gap steps — silent heartbeat

    if (currentMode == MODE_EL) {
        for (int i = 0; i < 3; i++) {
            unsigned long start = elStart(i);
            if (elapsed >= start && elapsed < start + EL_TOTAL_US) {
                unsigned long elSlot = (elapsed - start) / slotSize;
                if (elSlot < PREAMBLE_SLOTS) return preamble(elSlot);
                return elData(elSlot - PREAMBLE_SLOTS);
            }
        }
        return kBit;
    }

    if (currentMode == MODE_AZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        // EL firings
        for (int i = 0; i < 3; i++) {
            unsigned long start = elStart(i);
            if (elapsed >= start && elapsed < start + EL_TOTAL_US) {
                unsigned long elSlot = (elapsed - start) / slotSize;
                if (elSlot < PREAMBLE_SLOTS) return preamble(elSlot);
                return elData(elSlot - PREAMBLE_SLOTS);
            }
        }

        // AZ function
        if (elapsed >= 10000UL && elapsed < 25900UL) {
            unsigned long azSlot = (elapsed - 10000UL) / slotSize;
            if (azSlot < PREAMBLE_SLOTS) return preamble(azSlot);
            return azData(azSlot - PREAMBLE_SLOTS);
        }

        // SEQ1 datawords
        if (isSeq1) {
            uint8_t r;
            if ((r = checkDW(elapsed, SEQ1_BDW1_START, BDW1_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ1_BDW2_START, BDW2_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ1_BDW3_START, BDW3_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ1_BDW4_START, BDW4_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
        } else {
            // SEQ2 datawords
            uint8_t r;
            if ((r = checkDW(elapsed, SEQ2_BDW5_START,  BDW5_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ2_BDW6_START,  BDW6_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ2_ADWA1_START, ADWA1_BITS, ADWA_BITS_COUNT, ADWA_TOTAL_US, slotSize)) != 0xFF) return r;
        }
        return kBit;
    }

    if (currentMode == MODE_BAZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        if (isSeq1) {
            // EL firings
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                if (elapsed >= start && elapsed < start + EL_TOTAL_US) {
                    unsigned long elSlot = (elapsed - start) / slotSize;
                    if (elSlot < PREAMBLE_SLOTS) return preamble(elSlot);
                    return elData(elSlot - PREAMBLE_SLOTS);
                }
            }

            // AZ function
            if (elapsed >= 10000UL && elapsed < 25900UL) {
                unsigned long azSlot = (elapsed - 10000UL) / slotSize;
                if (azSlot < PREAMBLE_SLOTS) return preamble(azSlot);
                return azData(azSlot - PREAMBLE_SLOTS);
            }

            // BAZ function
            if (elapsed >= 38100UL && elapsed < 50000UL) {
                unsigned long bazSlot = (elapsed - 38100UL) / slotSize;
                if (bazSlot < PREAMBLE_SLOTS) return preamble(bazSlot);
                return bazData(bazSlot - PREAMBLE_SLOTS);
            }

            // SEQ1 datawords — BDW2 locked after BAZ
            uint8_t r;
            if ((r = checkDW(elapsed, SEQ1_BDW1_START, BDW1_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ1_BAZ_BDW2,   BDW2_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ1_BAZ_BDW3,   BDW3_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ1_BAZ_BDW4,   BDW4_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) return r;
        } else {
            // SEQ2 — same as AZ
            uint8_t r;
            if ((r = checkDW(elapsed, SEQ2_BDW5_START,  BDW5_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ2_BDW6_START,  BDW6_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) return r;
            if ((r = checkDW(elapsed, SEQ2_ADWA1_START, ADWA1_BITS, ADWA_BITS_COUNT, ADWA_TOTAL_US, slotSize)) != 0xFF) return r;
        }
        return kBit;
    }

    return kBit;
}

// fires regular SPI at midpoint and strobe at boundary, skips both if special imminent
void regularSlot(uint8_t frame, unsigned long thisSlot, unsigned long timeInSlot,unsigned long slotSize, bool specialImminent) {
    if (thisSlot > lastSlot && !specialImminent) {
        lastSlot = thisSlot;
        spiSentForSlot = false;
        strobeNow();
    }
    if (thisSlot == lastSlot && timeInSlot >= (slotSize / 2) && !spiSentForSlot && !specialImminent) {
        sendSPI(frame);
        spiSentForSlot = true;
    }
}

// insertion sort for phaseEvents array
static void sortEvents(unsigned long* arr, int n) {
    for (int i = 1; i < n; i++) {
        unsigned long key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// builds list of special event times for current step and mode
void buildPhaseEvents() {
    numPhaseEvents = 0;

    if (!CYCLE[currentStep].isSeq) {
        sortEvents(phaseEvents, numPhaseEvents);
        return;
    }

    if (currentMode == MODE_EL) {
        phaseEvents[numPhaseEvents++] = 3470UL;
        phaseEvents[numPhaseEvents++] = 5664UL;

        unsigned long el1 = elStart(1);
        phaseEvents[numPhaseEvents++] = el1;
        phaseEvents[numPhaseEvents++] = el1 + 3406UL;
        phaseEvents[numPhaseEvents++] = el1 + 5600UL;

        phaseEvents[numPhaseEvents++] = 54400UL;
        phaseEvents[numPhaseEvents++] = 54400UL + 3406UL;
        phaseEvents[numPhaseEvents++] = 54400UL + 5600UL;

    } else if (currentMode == MODE_AZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        // EL boundaries
        for (int i = 0; i < 3; i++) {
            unsigned long start = elStart(i);
            phaseEvents[numPhaseEvents++] = start;
            phaseEvents[numPhaseEvents++] = start + EL_TOTAL_US;
        }

        // AZ special strobes
        phaseEvents[numPhaseEvents++] = 10000UL;
        phaseEvents[numPhaseEvents++] = 10000UL + 8760UL;
        phaseEvents[numPhaseEvents++] = 10000UL + 9360UL;
        phaseEvents[numPhaseEvents++] = 10000UL + 15900UL;

        // dataword boundaries
        if (isSeq1) {
            phaseEvents[numPhaseEvents++] = SEQ1_BDW1_START;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW1_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW2_START;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW2_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW3_START;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW3_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW4_START;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW4_START + BDW_TOTAL_US;
        } else {
            phaseEvents[numPhaseEvents++] = SEQ2_BDW5_START;
            phaseEvents[numPhaseEvents++] = SEQ2_BDW5_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ2_BDW6_START;
            phaseEvents[numPhaseEvents++] = SEQ2_BDW6_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ2_ADWA1_START;
            phaseEvents[numPhaseEvents++] = SEQ2_ADWA1_START + ADWA_TOTAL_US;
        }

    } else if (currentMode == MODE_BAZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        if (isSeq1) {
            // EL boundaries
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                phaseEvents[numPhaseEvents++] = start;
                phaseEvents[numPhaseEvents++] = start + EL_TOTAL_US;
            }

            // AZ special strobes
            phaseEvents[numPhaseEvents++] = 10000UL;
            phaseEvents[numPhaseEvents++] = 10000UL + 8760UL;
            phaseEvents[numPhaseEvents++] = 10000UL + 9360UL;
            phaseEvents[numPhaseEvents++] = 10000UL + 15900UL;

            // BAZ special strobes
            phaseEvents[numPhaseEvents++] = 38100UL;
            phaseEvents[numPhaseEvents++] = 38100UL + 6760UL;
            phaseEvents[numPhaseEvents++] = 38100UL + 7360UL;
            phaseEvents[numPhaseEvents++] = 38100UL + 11900UL;

            // BAZ dataword boundaries
            phaseEvents[numPhaseEvents++] = SEQ1_BDW1_START;
            phaseEvents[numPhaseEvents++] = SEQ1_BDW1_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ1_BAZ_BDW2;
            phaseEvents[numPhaseEvents++] = SEQ1_BAZ_BDW2 + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ1_BAZ_BDW3;
            phaseEvents[numPhaseEvents++] = SEQ1_BAZ_BDW3 + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ1_BAZ_BDW4;
            phaseEvents[numPhaseEvents++] = SEQ1_BAZ_BDW4 + BDW_TOTAL_US;
        } else {
            // SEQ2 same as AZ
            phaseEvents[numPhaseEvents++] = SEQ2_BDW5_START;
            phaseEvents[numPhaseEvents++] = SEQ2_BDW5_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ2_BDW6_START;
            phaseEvents[numPhaseEvents++] = SEQ2_BDW6_START + BDW_TOTAL_US;
            phaseEvents[numPhaseEvents++] = SEQ2_ADWA1_START;
            phaseEvents[numPhaseEvents++] = SEQ2_ADWA1_START + ADWA_TOTAL_US;
        }
    }

    sortEvents(phaseEvents, numPhaseEvents);
}

// advances to next step and resets all counters
void advancePhase() {
    currentStep = (currentStep + 1) % CYCLE_LEN;
    if (currentStep == 0 && controller_sel) sendSync();
    phaseStart     = micros();
    lastSlot       = 0;
    spiSentForSlot = false;
    evSpiSent      = false;
    nextEventIdx   = 0;
    buildPhaseEvents();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("TCU upload successful");

    pinMode(CHIP_SEL, OUTPUT); 
    digitalWrite(CHIP_SEL, HIGH);
    pinMode(STROBE,   OUTPUT); 
    digitalWrite(STROBE,   LOW);
    pinMode(SyncIN,   INPUT);
    pinMode(SyncOUT,  OUTPUT);

    SPI.begin();
    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);

    phaseStart = micros();
    buildPhaseEvents();

    if (controller_sel) sendSync();
}

// main loop — event system drives all strobe and SPI activity
void loop() {
    unsigned long now      = micros();
    unsigned long elapsed  = now - phaseStart;
    unsigned long slotSize = 64UL * (unsigned long)k;
    uint8_t       kBit     = (k != 1) ? 0x01 : 0x00;

    if (elapsed >= CYCLE[currentStep].duration) {
        advancePhase();
        return;
    }

    unsigned long thisSlot   = elapsed / slotSize;
    unsigned long timeInSlot = elapsed % slotSize;
    bool isSeq = CYCLE[currentStep].isSeq;

    uint8_t currentFrame = frameAt(elapsed, isSeq, slotSize, kBit);

    // event-driven strobe/SPI — suppresses regular slot when special event is imminent
    bool imminent = false;
    if (nextEventIdx < numPhaseEvents) {
        unsigned long evT = phaseEvents[nextEventIdx];

        imminent = (evT <= elapsed + slotSize);

        if (evT >= slotSize / 2 && elapsed >= evT - slotSize / 2 && !evSpiSent) {
            uint8_t evFrame = frameAt(evT, isSeq, slotSize, kBit);
            sendSPI(evFrame);
            evSpiSent      = true;
            spiSentForSlot = true;
        }

        if (elapsed >= evT) {
            strobeNow();
            lastSlot       = thisSlot + 1;
            spiSentForSlot = false;
            evSpiSent      = false;
            nextEventIdx++;
        }
    }

    regularSlot(currentFrame, thisSlot, timeInSlot, slotSize, imminent);
}