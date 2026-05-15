// TCU_main.cpp

#include "tcu_header.h"


//barker code constant
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
    {false,  0UL},  // step 13 — GAP
    {true,  66800UL},  // step 14 — SEQ2
    {false, 18000UL},  // step 15 — GAP
};

// global states
unsigned long phaseStart      = 0;
unsigned long lastSlot        = 0;
bool          spiSentForSlot  = false;
bool          evSpiSent       = false;
int           currentStep     = 0;
int           k               = 1;     // &&
int           currentMode     = MODE_EL;
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

// stobe func
void strobeNow() {
    digitalWrite(STROBE,HIGH);
    digitalWrite(STROBE,LOW);
}

// sync in/out (IRRELEVANT TO USSIM)
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
    if (n == 1) return scale(EL2_OFFSET_US);  // &&
    return scale(54400UL);
}

// preamble function since preamble is always the same
uint8_t preamble(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D5   = (0b101 << 1);
    if (slot >= 13 && slot <= 17) {
        return TXEN | (uint8_t)(DPSK_BARKER[slot - 13] << 6) | kBit;
    }
    return TXEN | D5 | kBit;
}

// EL data Byte builder based on slot
uint8_t elData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D2   = (0b010 << 1);
    uint8_t D4   = (0b100 << 1);
    uint8_t D5   = (0b101 << 1);

    if (slot < 2) {
        return TXEN | D5 | kBit;
    }
    if (slot >= 2 && slot < 4) {
        return TXEN | D2 | kBit;
    }
    if (slot == 4) {
        return TXEN | TOFRO | SBSTART | D4 | kBit;
    }
    if (slot >= 5 && slot < 28) {
        return TXEN | SBSTART | D4 | kBit;
    }
    if (slot >= 28 && slot < 34) {
        return D5 | kBit;
    }
    if (slot == 34) {
        return TXEN | TOFRO | SBSTART | D4 | kBit;
    }
    if (slot >= 35 && slot < 62) {
        return TXEN | SBSTART | D4 | kBit;
    }
    return D5 | kBit;
}

// AZ data Byte builder based on slot
uint8_t azData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D5   = (0b101 << 1);
    uint8_t D4   = (0b100 << 1);
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b000, 0b101};
    uint8_t ant = D5;
    if (slot >= 7 && slot <= 11) {
        ant = (uint8_t)(ANT_SEQ[slot - 7] << 1);
    } else if (slot >= 12 && slot <= 14) {
        ant = (uint8_t)(ANT_SEQ[4] << 1);
    }

    if (slot <= 6) {
        return TXEN | D5 | kBit;
    }
    if (slot >= 7 && slot <= 14) {
        return TXEN | ant | kBit;
    }
    if (slot == 15) {
        return TXEN | TOFRO | SBSTART | D4 | kBit;
    }
    if (slot >= 16 && slot < 112) {
        return TXEN | SBSTART | D4 | kBit;
    }
    if (slot >= 112 && slot < 121) {
        return D5 | kBit;
    }
    if (slot == 121) {
        return TXEN | TOFRO | SBSTART | D4 | kBit;
    }
    if (slot >= 122 && slot < 223) {
        return TXEN | SBSTART | D4 | kBit;
    }
    return D5 | kBit;
}

// BAZ data byte builder based on slot
uint8_t bazData(unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    uint8_t D5   = (0b101 << 1);
    uint8_t D4   = (0b100 << 1);
    const uint8_t ANT_SEQ[] = {0b001, 0b010, 0b011, 0b000, 0b101};
    uint8_t ant = D5;
    if (slot >= 7 && slot <= 11) {
        ant = (uint8_t)(ANT_SEQ[slot - 7] << 1);
    } else if (slot >= 12 && slot <= 14) {
        ant = (uint8_t)(ANT_SEQ[4] << 1);
    }

    if (slot <= 6) {
        return TXEN | D5 | kBit;
    }
    if (slot >= 7 && slot <= 14) {
        return TXEN | ant | kBit;
    }
    if (slot == 15) {
        return TXEN | TOFRO | SBSTART | D4 | kBit;
    }
    if (slot >= 16 && slot < 80) {
        return TXEN | SBSTART | D4 | kBit;
    }
    if (slot >= 80 && slot < 90) {
        return D5 | kBit;
    }
    if (slot == 90) {
        return TXEN | TOFRO | SBSTART | D4 | kBit;
    }
    if (slot >= 91 && slot < 161) {
        return TXEN | SBSTART | D4 | kBit;
    }
    return D5 | kBit;
}

// dataword byte builder based on slot
uint8_t datawordByte(const uint8_t* bits, int numBits, unsigned long slot) {
    uint8_t kBit = (k != 1) ? 0x01 : 0x00;
    if (slot < PREAMBLE_SLOTS) {
        return preamble(slot);
    }
    unsigned long dataSlot = slot - PREAMBLE_SLOTS;
    if (dataSlot >= (unsigned long)numBits) {
        return TXEN | kBit;
    }
    return TXEN | (uint8_t)(bits[dataSlot] << 6) | kBit;
}

// helper to check if a data word should trigger
static uint8_t checkDW(unsigned long elapsed, unsigned long start, const uint8_t* bits, int numBits, unsigned long total, unsigned long slotSize) {
    if (elapsed >= start && elapsed < start + scale(total)) {
        unsigned long s = (elapsed - start) / slotSize;
        return datawordByte(bits, numBits, s);
    }
    return 0xFF; 
}

// Logic to return the correct frame based on the timing and mode we are in
uint8_t frameAt(unsigned long elapsed, bool isSeqStep, unsigned long slotSize, uint8_t kBit) {
    if (!isSeqStep) {
        return kBit;
    }

    if (currentMode == MODE_EL) {
        for (int i = 0; i < 3; i++) {
            unsigned long start = elStart(i);
            if (elapsed >= start && elapsed < start + scale(EL_TOTAL_US)) {
                unsigned long elSlot = (elapsed - start) / slotSize;
                if (elSlot < PREAMBLE_SLOTS) {
                    return preamble(elSlot);
                }
                return elData(elSlot - PREAMBLE_SLOTS);
            }
        }
        return kBit;
    }

    if (currentMode == MODE_AZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        if (elapsed >= scale(10000UL) && elapsed < scale(25900UL)) {
            unsigned long azSlot = (elapsed - scale(10000UL)) / slotSize;
            if (azSlot < PREAMBLE_SLOTS) {
                return preamble(azSlot);
            }
            return azData(azSlot - PREAMBLE_SLOTS);
        }

        if (isSeq1) {
            uint8_t r;
            if ((r = checkDW(elapsed, scale(SEQ1_BDW1_START), BDW1_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ1_BDW2_START), BDW2_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ1_BDW3_START), BDW3_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ1_BDW4_START), BDW4_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
        } else {
            uint8_t r;
            if ((r = checkDW(elapsed, scale(SEQ2_BDW5_START),  BDW5_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ2_BDW6_START),  BDW6_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ2_ADWA1_START), ADWA1_BITS, ADWA_BITS_COUNT, ADWA_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
        }
        return kBit;
    }

    if (currentMode == MODE_BAZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        if (isSeq1) {
            if (elapsed >= scale(38100UL) && elapsed < scale(50000UL)) {
                unsigned long bazSlot = (elapsed - scale(38100UL)) / slotSize;
                if (bazSlot < PREAMBLE_SLOTS) {
                    return preamble(bazSlot);
                }
                return bazData(bazSlot - PREAMBLE_SLOTS);
            }

            uint8_t r;
            if ((r = checkDW(elapsed, scale(SEQ1_BAZ_BDW1), BDW1_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ1_BAZ_BDW2), BDW2_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ1_BAZ_BDW3), BDW3_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ1_BAZ_BDW4), BDW4_BITS, BDW_BITS_COUNT, BDW_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
        } else {
            uint8_t r;
            if ((r = checkDW(elapsed, scale(SEQ2_BDW5_START),  BDW5_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ2_BDW6_START),  BDW6_BITS,  BDW_BITS_COUNT,  BDW_TOTAL_US,  slotSize)) != 0xFF) {
                return r;
            }
            if ((r = checkDW(elapsed, scale(SEQ2_ADWA1_START), ADWA1_BITS, ADWA_BITS_COUNT, ADWA_TOTAL_US, slotSize)) != 0xFF) {
                return r;
            }
        }
        return kBit;
    }

    return kBit;
}

// fires regular SPI at midpoint and strobe at boundary, skips both if special imminent (this is for preamble and some things after preamble)
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

// sorts the events in the array in order by earliest occurance to latest
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

// builds list of special event times for current step and mode (this needs to be here since we need to rebuild the events based on what mode we are in/ if we need datawords or not)
void buildPhaseEvents() {
    numPhaseEvents = 0;

    if (!CYCLE[currentStep].isSeq) {
        sortEvents(phaseEvents, numPhaseEvents);
        return;
    }

    if (currentMode == MODE_EL) {
        phaseEvents[numPhaseEvents++] = scale(3470UL);
        phaseEvents[numPhaseEvents++] = scale(5664UL);

        unsigned long el1 = elStart(1);
        phaseEvents[numPhaseEvents++] = el1;
        phaseEvents[numPhaseEvents++] = el1 + scale(3406UL);
        phaseEvents[numPhaseEvents++] = el1 + scale(5600UL);

        phaseEvents[numPhaseEvents++] = scale(54400UL);
        phaseEvents[numPhaseEvents++] = scale(54400UL) + scale(3406UL);
        phaseEvents[numPhaseEvents++] = scale(54400UL) + scale(5600UL);

    } else if (currentMode == MODE_AZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        // EL boundaries
        for (int i = 0; i < 3; i++) {
            unsigned long start = elStart(i);
            phaseEvents[numPhaseEvents++] = start;
            phaseEvents[numPhaseEvents++] = start + scale(EL_TOTAL_US);
        }

        // AZ special strobes
        phaseEvents[numPhaseEvents++] = scale(10000UL);
        phaseEvents[numPhaseEvents++] = scale(10000UL) + scale(8760UL);
        phaseEvents[numPhaseEvents++] = scale(10000UL) + scale(9360UL);
        phaseEvents[numPhaseEvents++] = scale(10000UL) + scale(15900UL);

        // dataword boundaries
        if (isSeq1) {
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW1_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW1_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW2_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW2_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW3_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW3_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW4_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW4_START) + scale(BDW_TOTAL_US);
        } else {
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW5_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW5_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW6_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW6_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_ADWA1_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_ADWA1_START) + scale(ADWA_TOTAL_US);
        }

    } else if (currentMode == MODE_BAZ) {
        bool isSeq1 = (CYCLE[currentStep].duration == 66700UL);

        if (isSeq1) {
            // EL boundaries
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                phaseEvents[numPhaseEvents++] = start;
                phaseEvents[numPhaseEvents++] = start + scale(EL_TOTAL_US);
            }

            // AZ special strobes
            phaseEvents[numPhaseEvents++] = scale(10000UL);
            phaseEvents[numPhaseEvents++] = scale(10000UL) + scale(8760UL);
            phaseEvents[numPhaseEvents++] = scale(10000UL) + scale(9360UL);
            phaseEvents[numPhaseEvents++] = scale(10000UL) + scale(15900UL);

            // BAZ special strobes
            phaseEvents[numPhaseEvents++] = scale(38100UL);
            phaseEvents[numPhaseEvents++] = scale(38100UL) + scale(6760UL);
            phaseEvents[numPhaseEvents++] = scale(38100UL) + scale(7360UL);
            phaseEvents[numPhaseEvents++] = scale(38100UL) + scale(11900UL);

            // BAZ dataword boundaries
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW1_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BDW1_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BAZ_BDW2);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BAZ_BDW2) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BAZ_BDW3);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BAZ_BDW3) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BAZ_BDW4);
            phaseEvents[numPhaseEvents++] = scale(SEQ1_BAZ_BDW4) + scale(BDW_TOTAL_US);
        } else {
            // SEQ2 same as AZ
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW5_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW5_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW6_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_BDW6_START) + scale(BDW_TOTAL_US);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_ADWA1_START);
            phaseEvents[numPhaseEvents++] = scale(SEQ2_ADWA1_START) + scale(ADWA_TOTAL_US);
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

// main loop that contains event system that drives all strobe and SPI activity
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

    // event-driven strobe/SPI that will suppress the normally scheduled strobe/spi when a special event is incoming
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