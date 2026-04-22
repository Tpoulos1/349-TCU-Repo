// TCU Project
#include <SPI.h>

// define specific pin numbers later &&

#define CHIP_SEL    10   // SPI chip select 
#define STROBE   9 
#define SyncIN  11  
#define SyncOUT 12 

// Timing (when everything is gonna happen)
#define PREAMBLE_US    1600UL
#define EL_DATA_US     4000UL
#define EL_TOTAL_US    5600UL   // preamble + data

#define SEQ1_DUR_US    66700UL
#define SEQ2_DUR_US    66800UL

#define EL2_OFFSET_US  30000UL  // middle EL — change when known &&

#define MODE_EL  0
#define MODE_AZ  1
#define MODE_BAZ 2

#define TOFRO   (1 << 5)
#define SBSTART (1 << 4)
#define TXEN    (1 << 7)

int currentMode = MODE_EL;  // change this to switch&&

const unsigned long AZ_SPECIAL_STROBES_US[] = {
    8760,   // PAUSE
    9360,   // FRO SCAN
    15900   // END
};
#define AZ_SPECIAL_COUNT 3

int azStrobeIdx = 0;

const unsigned long EL_SPECIAL_STROBES_US[] = {
    3406,   // PAUSE start
    3806,   // FRO SCAN
    5600    // END
};
#define EL_SPECIAL_COUNT 3

int elStrobeIdx = 0;

// full cycle = 16 steps (8 SEQs + 8 gaps) struct to store if we are in a seq or not
struct Step {
    bool          isSeq;
    unsigned long duration;
};

// full repeating cycle of each sequence and when it should trigger
const Step CYCLE[] = {
    {true,  SEQ1_DUR_US},  // SEQ1
    {false, 1000UL},       // 1ms
    {true,  SEQ2_DUR_US},  // SEQ2
    {false, 13000UL},      // 13ms
    {true,  SEQ1_DUR_US},  // SEQ1
    {false, 19000UL},      // 19ms
    {true,  SEQ2_DUR_US},  // SEQ2
    {false, 2000UL},       // 2ms
    {true,  SEQ1_DUR_US},  // SEQ1
    {false, 20000UL},      // 20ms
    {true,  SEQ2_DUR_US},  // SEQ2
    {false, 6000UL},       // 6ms
    {true,  SEQ1_DUR_US},  // SEQ1
    {false, 0UL},          // 0ms
    {true,  SEQ2_DUR_US},  // SEQ2
    {false, 18000UL},      // 18ms
};

#define CYCLE_LEN 16

int currentStep = 0; // which step of the cycle your in

int k = 1; // k constant subject to change &&
const uint8_t DPSK_BARKER[] = {1, 1, 1, 0, 1};

// func to send preamble (should be the same at any point)
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
        return 0x00;
    }
    if (slot == 121) {
        return TXEN | TOFRO | SBSTART | ant | kBit;
    }
    if (slot >= 122 && slot < 223) {
        return TXEN | SBSTART | ant | kBit;
    }
    return 0x00;
}

// where the data is set to what it needs to be set on specific timings
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
        return 0x00;
    }
    if (slot == 34) {
        return TXEN | TOFRO | SBSTART | kBit;
    }
    if (slot >= 35 && slot < 62) {
        return TXEN | SBSTART | kBit;
    }
    return 0x00;
}

bool spiSentForSlot = false; // tracks if SPI has been sent for current slot

// send the SPI signal with a 1MHz clock
void sendSPI(uint8_t DATA) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CHIP_SEL, LOW);
    SPI.transfer(DATA);
    digitalWrite(CHIP_SEL, HIGH);
    SPI.endTransaction();
}

// strobe the GPIO to tell them to latch data
void strobeNow() {
    digitalWrite(STROBE, HIGH);
    digitalWrite(STROBE, LOW);
}

int controller_sel = 0; // used to decide if this TCU is the controller or peripheral

// returns current time in microseconds
unsigned long getTime() {
    return micros();
}

// SYNC IN/OUT LOGIC -----
volatile unsigned long sequenceStart = 0;

// if we receive a sync set it up
void onSyncReceived() {
    sequenceStart = micros();
}

// send sync out to peripheral
void sendSync() {
    sequenceStart = micros();
    digitalWrite(SyncOUT, HIGH);
    delayMicroseconds(10);
    digitalWrite(SyncOUT, LOW);
}

unsigned long phaseStart = 0;
int activeEL = -1; // -1 is none, 0,1,2 are each el firing in the seq

// lookup for how long current step in long seq should last
unsigned long PhaseDuration() {
    return CYCLE[currentStep].duration;
}

// chooses each of the el sequences to start
unsigned long elStart(int n) {
    if (n == 0) {
        return 0;
    }
    if (n == 1) {
        return EL2_OFFSET_US;
    }
    return 54400UL;
}

// moves to next step on the cycle table and wraps back to zero
void advancePhase() {
    currentStep = (currentStep + 1) % CYCLE_LEN;
    if (currentStep == 0 && controller_sel) {
        sendSync();
    }
    phaseStart     = getTime();
    lastSlot       = 0;
    activeEL       = -1;
    spiSentForSlot = false;
    azStrobeIdx    = 0;
    elStrobeIdx = 0;
}

unsigned long lastSlot = 0;

void setup() {
    Serial.begin(115200);

    pinMode(CHIP_SEL,    OUTPUT);
    pinMode(STROBE,  OUTPUT);
    pinMode(SyncIN,  INPUT);
    pinMode(SyncOUT, OUTPUT);

    digitalWrite(CHIP_SEL,   HIGH);
    digitalWrite(STROBE, LOW);

    SPI.begin();

    attachInterrupt(digitalPinToInterrupt(SyncIN), onSyncReceived, RISING);

    phaseStart = getTime();

    if (controller_sel) {
        sendSync();
    }
}

void loop() {
    unsigned long now     = getTime();
    unsigned long elapsed = now - phaseStart;

    if (elapsed >= CYCLE[currentStep].duration) {
        advancePhase();
        return;
    }

    if (CYCLE[currentStep].duration == 0) {
        advancePhase();
        return;
    }

    unsigned long thisSlot   = elapsed / 64;
    unsigned long timeInSlot = elapsed % 64;

    uint8_t nextFrame = 0x00;

    if (CYCLE[currentStep].isSeq) {

        if (currentMode == MODE_EL) {
            for (int i = 0; i < 3; i++) {
                unsigned long start = elStart(i);
                unsigned long end   = start + EL_TOTAL_US;

                if (elapsed >= start && elapsed < end) {
                    if (activeEL != i) {
                        activeEL       = i;
                        lastSlot       = 0;
                        spiSentForSlot = false;
                        elStrobeIdx    = 0;
                    }

                    unsigned long elElapsed  = elapsed - start;
                    unsigned long elSlot     = elElapsed / 64;
                    unsigned long nextElSlot = elSlot + 1;

                    if ((nextElSlot * 64) < PREAMBLE_US) {
                        nextFrame = preamble(nextElSlot);
                    } else {
                        nextFrame = elData(nextElSlot - (PREAMBLE_US / 64));
                    }

                    if (elStrobeIdx < EL_SPECIAL_COUNT) {
                        if (elElapsed >= EL_SPECIAL_STROBES_US[elStrobeIdx]) {
                            strobeNow();
                            elStrobeIdx++;
                        }
                    }

                    break;
                }
            }

        } else if (currentMode == MODE_AZ) {
            unsigned long start = 10000UL;
            unsigned long end   = start + 15900UL;

            if (elapsed >= start && elapsed < end) {
                if (activeEL != 0) {
                    activeEL       = 0;
                    lastSlot       = 0;
                    spiSentForSlot = false;
                    azStrobeIdx    = 0;
                }

                unsigned long azElapsed  = elapsed - start;
                unsigned long azSlot     = azElapsed / 64;
                unsigned long nextAzSlot = azSlot + 1;

                if ((nextAzSlot * 64) < PREAMBLE_US) {
                    nextFrame = preamble(nextAzSlot);
                } else {
                    nextFrame = azData(nextAzSlot - (PREAMBLE_US / 64));
                }

                if (azStrobeIdx < AZ_SPECIAL_COUNT) {
                    if (azElapsed >= AZ_SPECIAL_STROBES_US[azStrobeIdx]) {
                        strobeNow();
                        azStrobeIdx++;
                    }
                }
            }
        }
    }

    if (timeInSlot >= 32 && !spiSentForSlot) {
        sendSPI(nextFrame);
        spiSentForSlot = true;
    }

    if (thisSlot > lastSlot) {
        lastSlot       = thisSlot;
        spiSentForSlot = false;
        strobeNow();
    }
}