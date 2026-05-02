// TCU_main.cpp

#include "tcu_header.h"

// ── Constant data

// DPSK Barker code for preamble slots 13-17
const uint8_t DPSK_BARKER[] = {1, 0, 1, 1, 0};

// Pre-encoded DPSK bit arrays for datawords
const uint8_t BDW1_BITS[]  = {1,1,0,1,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0};
const uint8_t BDW2_BITS[]  = {1,1,1,1,1,0,1,1,1,0,0,0,1,1,0,0,0,1,1,1};
const uint8_t BDW3_BITS[]  = {1,0,1,1,1,0,0,1,0,0,1,1,0,1,1,0,1,1,1,1};
const uint8_t BDW4_BITS[]  = {1,0,1,0,0,0,1,1,0,1,0,0,0,1,1,0,0,1,1,0};
const uint8_t BDW5_BITS[]  = {0,1,0,1,1,1,0,1,1,0,1,1,0,0,0,1,0,0,1,1};
const uint8_t BDW6_BITS[]  = {1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,0,1,1};
const uint8_t ADWA1_BITS[] = {0,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,1,1,0,1,1,1,1,0,1,1,1,0,1,0,1,1,11,1,1,1};

// All datawords referenced by step and phase-relative offset
const DataWord DATAWORDS[] = {
    {BDW1_BITS,  BDW_BITS_COUNT,  3,    0UL, BDW_TOTAL_US},
    {BDW2_BITS,  BDW_BITS_COUNT,  3, 3164UL, BDW_TOTAL_US},
    {BDW3_BITS,  BDW_BITS_COUNT,  3, 6328UL, BDW_TOTAL_US},
    {BDW4_BITS,  BDW_BITS_COUNT,  3, 9492UL, BDW_TOTAL_US},
    {BDW5_BITS,  BDW_BITS_COUNT,  5,    0UL, BDW_TOTAL_US},
    {BDW6_BITS,  BDW_BITS_COUNT,  5, 3164UL, BDW_TOTAL_US},
    {ADWA1_BITS, ADWA_BITS_COUNT, 5, 6328UL, ADWA_TOTAL_US},
};

// Cycle definition: alternating sequence and gap steps
const Step CYCLE[CYCLE_LEN] = {
    {true,  66700UL},  // step 0  — SEQ
    {false,  1000UL},  // step 1  — GAP
    {true,  66800UL},  // step 2  — SEQ
    {false, 13000UL},  // step 3  — GAP (BDW1-4)
    {true,  66700UL},  // step 4  — SEQ
    {false, 19000UL},  // step 5  — GAP (BDW5-6, ADWA1)
    {true,  66800UL},  // step 6  — SEQ
    {false,  2000UL},  // step 7  — GAP
    {true,  66700UL},  // step 8  — SEQ
    {false, 20000UL},  // step 9  — GAP
    {true,  66800UL},  // step 10 — SEQ
    {false,  6000UL},  // step 11 — GAP
    {true,  66700UL},  // step 12 — SEQ
    {false,  1000UL},  // step 13 — GAP
    {true,  66800UL},  // step 14 — SEQ
    {false, 18000UL},  // step 15 — GAP
};

// ── Global state
unsigned long phaseStart      = 0;
unsigned long lastSlot        = 0;
bool          spiSentForSlot  = false;
bool          evSpiSent       = false;  // separate flag so event SPI fires even if regular SPI already ran this slot
int           currentStep     = 0;
int           k               = 1;     // && slot multiplier, currently 1
int           currentMode     = MODE_AZ;
int           controller_sel  = 0;     // 1 = this unit is sync controller

unsigned long phaseEvents[MAX_PHASE_EVENTS];
int           numPhaseEvents  = 0;
int           nextEventIdx    = 0;

volatile unsigned long sequenceStart = 0;

// ── Low-level I/O 

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

void onSyncReceived() {
    sequenceStart = micros();
}

void sendSync() {
    sequenceStart = micros();
    digitalWrite(SyncOUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(SyncOUT, LOW);
}

// ── EL sub-function timing

// Return the phase-relative start time of EL sub-function n (0, 1, 2)
unsigned long elStart(int n) {
    if (n == 0) return 0UL;
    if (n == 1) return EL2_OFFSET_US;  // && placeholder
    return 54400UL;
}

// ── Frame builders

// Build preamble byte for a given slot (0-24); TXEN always set, Barker on slots 13-17
uint8_t preamble(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t b = TXEN | kBit;
    if (slot >= 13 && slot <= 17) {
        b |= (uint8_t)(DPSK_BARKER[slot - 13] << 6);
    }
    return b;
}

// Build EL data byte for slot relative to post-preamble start
uint8_t elData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    if (slot < 4)                return TXEN | kBit;
    if (slot == 4)               return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 5  && slot < 28) return TXEN | SBSTART | kBit;
    if (slot >= 28 && slot < 34) return kBit;
    if (slot == 34)              return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 35 && slot < 62) return TXEN | SBSTART | kBit;
    return kBit;
}

// Build AZ data byte for slot relative to post-preamble start
uint8_t azData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b100, 0b101};
    uint8_t ant = 0x00;
    if (slot >= 7  && slot <= 11) ant = (uint8_t)(ANT_SEQ[slot - 7] << 1);
    else if (slot >= 12 && slot <= 14) ant = (uint8_t)(ANT_SEQ[4] << 1);

    if (slot <= 6)                 return TXEN | kBit;
    if (slot >= 7  && slot <= 14)  return TXEN | ant | kBit;
    if (slot == 15)                return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 16 && slot < 112)  return TXEN | SBSTART | kBit;
    if (slot >= 112 && slot < 121) return kBit;
    if (slot == 121)               return TXEN | TOFRO | SBSTART | kBit;
    if (slot >= 122 && slot < 223) return TXEN | SBSTART | kBit;
    return kBit;
}

// Build dataword byte: preamble for first PREAMBLE_SLOTS slots, then DPSK data bits
uint8_t datawordByte(const uint8_t* bits, int numBits, unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    if (slot < PREAMBLE_SLOTS) return preamble(slot);
    unsigned long dataSlot = slot - PREAMBLE_SLOTS;
    if (dataSlot >= (unsigned long)numBits) return TXEN | kBit;
    return TXEN | (uint8_t)(bits[dataSlot] << 6) | kBit;
}

// Compute the SPI frame byte for any elapsed time in the current phase
uint8_t frameAt(unsigned long elapsed, bool isSeqStep, unsigned long slotSize, uint8_t kBit) {
    if (isSeqStep) {
        if (currentMode == MODE_EL) {
            // Check each of the three EL sub-functions
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                if (elapsed >= start && elapsed < start + EL_TOTAL_US) {
                    unsigned long elSlot = (elapsed - start) / slotSize;
                    if (elSlot < PREAMBLE_SLOTS) return preamble(elSlot);
                    return elData(elSlot - PREAMBLE_SLOTS);
                }
            }
            return kBit;  // between EL sub-functions: heartbeat
        } else {
            // AZ mode: single function starting at 10000us
            unsigned long azStart = 10000UL;
            unsigned long azEnd   = azStart + 15900UL;
            if (elapsed >= azStart && elapsed < azEnd) {
                unsigned long azSlot = (elapsed - azStart) / slotSize;
                if (azSlot < PREAMBLE_SLOTS) return preamble(azSlot);
                return azData(azSlot - PREAMBLE_SLOTS);
            }
            return kBit;
        }
    } else {
        // Gap step: check all datawords for current step
        for (int d = 0; d < NUM_DATAWORDS; d++) {
            if (DATAWORDS[d].step == currentStep &&
                elapsed >= DATAWORDS[d].offset &&
                elapsed <  DATAWORDS[d].offset + DATAWORDS[d].totalUs) {
                unsigned long dwSlot = (elapsed - DATAWORDS[d].offset) / slotSize;
                return datawordByte(DATAWORDS[d].bits, DATAWORDS[d].numBits, dwSlot);
            }
        }
        return kBit;
    }
}

// ── Slot handler 

// Handle regular 64us slot boundaries: strobe at slot edge, SPI at midpoint
void regularSlot(uint8_t frame, unsigned long thisSlot, unsigned long timeInSlot,
                 unsigned long slotSize, bool specialImminent) {
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

// ── Phase event system

// Insertion sort for small phaseEvents array
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

// Populate phaseEvents[] with all special event times for the current step/mode
void buildPhaseEvents() {
    numPhaseEvents = 0;

    if (CYCLE[currentStep].isSeq) {
        if (currentMode == MODE_EL) {
            // EL0 special strobe events — offset +64us so times are relative to first strobe (t=64)
            phaseEvents[numPhaseEvents++] = 3470UL;  // PAUSE:    3406us from first strobe
            phaseEvents[numPhaseEvents++] = 3870UL;  // FRO SCAN: 3806us from first strobe
            phaseEvents[numPhaseEvents++] = 5664UL;  // END:      5600us from first strobe

            // EL1 start and special strobe events
            unsigned long el1 = elStart(1);
            phaseEvents[numPhaseEvents++] = el1;
            phaseEvents[numPhaseEvents++] = el1 + 3406UL;
            phaseEvents[numPhaseEvents++] = el1 + 3806UL;
            phaseEvents[numPhaseEvents++] = el1 + 5600UL;

            // EL2 start and special strobe events
            phaseEvents[numPhaseEvents++] = 54400UL;
            phaseEvents[numPhaseEvents++] = 54400UL + 3406UL;
            phaseEvents[numPhaseEvents++] = 54400UL + 3806UL;
            phaseEvents[numPhaseEvents++] = 54400UL + 5600UL;
        } else {
            // AZ mode: AZ function start, pause, fro scan, and end events
            phaseEvents[numPhaseEvents++] = 10000UL;           // AZ start
            phaseEvents[numPhaseEvents++] = 10000UL + 8760UL;  // pause point
            phaseEvents[numPhaseEvents++] = 10000UL + 9360UL;  // fro scan start
            phaseEvents[numPhaseEvents++] = 10000UL + 15900UL; // AZ end
        }
    } else {
        // Gap step dataword events
        if (currentStep == 3) {
            phaseEvents[numPhaseEvents++] =  3100UL;  // BDW1 end
            phaseEvents[numPhaseEvents++] =  3164UL;  // BDW2 start
            phaseEvents[numPhaseEvents++] =  6264UL;  // BDW2 end
            phaseEvents[numPhaseEvents++] =  6328UL;  // BDW3 start
            phaseEvents[numPhaseEvents++] =  9428UL;  // BDW3 end
            phaseEvents[numPhaseEvents++] =  9492UL;  // BDW4 start
            phaseEvents[numPhaseEvents++] = 12592UL;  // BDW4 end
        } else if (currentStep == 5) {
            phaseEvents[numPhaseEvents++] =  3100UL;  // BDW5 end
            phaseEvents[numPhaseEvents++] =  3164UL;  // BDW6 start
            phaseEvents[numPhaseEvents++] =  6264UL;  // BDW6 end
            phaseEvents[numPhaseEvents++] =  6328UL;  // ADWA1 start
            phaseEvents[numPhaseEvents++] = 12228UL;  // ADWA1 end
        }
        // All other gap steps have no special events
    }

    sortEvents(phaseEvents, numPhaseEvents);
}

// Advance to the next cycle step and reinitialise phase tracking
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

// ── Arduino entry points

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

    phaseStart = micros();
    buildPhaseEvents();

    if (controller_sel) sendSync();
}

// Main timing loop: unified event system drives all strobe and SPI activity
void loop() {
    unsigned long now      = micros();
    unsigned long elapsed  = now - phaseStart;
    unsigned long slotSize = 64UL * (unsigned long)k;
    uint8_t       kBit     = (k != 1) ? 0x01 : 0x00;

    // Advance phase when step duration is exhausted
    if (elapsed >= CYCLE[currentStep].duration) {
        advancePhase();
        return;
    }

    unsigned long thisSlot   = elapsed / slotSize;
    unsigned long timeInSlot = elapsed % slotSize;
    bool isSeq = CYCLE[currentStep].isSeq;

    // Compute the continuous frame for current elapsed position
    uint8_t currentFrame = frameAt(elapsed, isSeq, slotSize, kBit);

    // Event-driven strobe/SPI: fires at special boundaries, suppresses regular slot during approach
    bool imminent = false;
    if (nextEventIdx < numPhaseEvents) {
        unsigned long evT = phaseEvents[nextEventIdx];

        // Suppress regular slot activity when a special event is within one slot
        imminent = (evT <= elapsed + slotSize);

        // Fire event SPI at evT - slotSize/2 (32us before the event strobe)
        // Uses evSpiSent (not spiSentForSlot) so it fires even if a regular slot SPI already ran
        if (evT >= slotSize / 2 && elapsed >= evT - slotSize / 2 && !evSpiSent) {
            uint8_t evFrame = frameAt(evT, isSeq, slotSize, kBit);
            sendSPI(evFrame);
            evSpiSent      = true;
            spiSentForSlot = true;  // also block regular slot SPI from firing again
        }

        // Fire event strobe at exactly evT, then advance event index
        if (elapsed >= evT) {
            strobeNow();
            lastSlot       = thisSlot + 1;
            spiSentForSlot = false;
            evSpiSent      = false;  // reset for next event
            nextEventIdx++;
        }
    }

    // Regular 64us slot handler runs unless a special event is imminent
    regularSlot(currentFrame, thisSlot, timeInSlot, slotSize, imminent);
}