// TCU Project
#include <SPI.h>

// define specific pin numbers later &&

#define GPIO 4 //spi chip select
#define STROBE 7
#define SyncIN 5
#define SyncOUT 6

// Timing (when everything is gonna happen)
#define PREAMBLE_US    1600UL
#define EL_DATA_US     4000UL
#define EL_TOTAL_US    5600UL   // preamble + data

#define SEQ1_DUR_US    66700UL
#define SEQ2_DUR_US    66800UL

#define EL2_OFFSET_US  30000UL  // middle EL — change when known &&

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

bool k = 0; // k constant subject to change &&
const uint8_t DPSK_BARKER[] = {1, 0, 1, 1, 0};

// func to send preamble (should be the same at any point)
uint8_t preamble(unsigned long slot) {
    uint8_t b = 0x00;
    if (slot == 0) {
        b |= (1 << 7);
        b |= (k & 0x01);
    } else if (slot >= 13 && slot <= 17) {
        b |= (DPSK_BARKER[slot - 13] << 6);
    }
    return b;
}

// where the data is set to what it needs to be set on specific timings
uint8_t elData(unsigned long slot) {
    #define TOFRO   (1 << 5)
    #define SBSTART (1 << 4)

    if (slot == 4) {
        return TOFRO | SBSTART;
    }
    if (slot >= 5 && slot < 28) {
        return SBSTART;
    }
    if (slot >= 28 && slot < 34) {
        return 0x00;
    }
    if (slot == 34) {
        return TOFRO | SBSTART;
    }
    if (slot >= 35 && slot < 62) {
        return SBSTART;
    }

    return 0x00;
}

bool spiSentForSlot = false; // tracks if SPI has been sent for current slot

// send the SPI signal with a 1MHz clock
void sendSPI(uint8_t DATA) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GPIO, LOW);
    SPI.transfer(DATA);
    digitalWrite(GPIO, HIGH);
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
        sendSync(); // resync every full cycle
    }
    phaseStart     = getTime();
    lastSlot       = 0;
    activeEL       = -1;
    spiSentForSlot = false;
}

unsigned long lastSlot = 0;

void setup() {
    Serial.begin(115200);

    pinMode(GPIO,    OUTPUT);
    pinMode(STROBE,  OUTPUT);
    pinMode(SyncIN,  INPUT);
    pinMode(SyncOUT, OUTPUT);

    digitalWrite(GPIO,   HIGH);
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

    // always run slot timing regardless of gap or seq
    unsigned long thisSlot   = elapsed / 64;
    unsigned long timeInSlot = elapsed % 64;

    uint8_t nextFrame = 0x00; // default but overridden if inside an EL window

    if (CYCLE[currentStep].isSeq) {
        for (int i = 0; i < 3; i++) {
            unsigned long start = elStart(i);
            unsigned long end   = start + EL_TOTAL_US;

            if (elapsed >= start && elapsed < end) {
                if (activeEL != i) {
                    activeEL       = i;
                    lastSlot       = 0;
                    spiSentForSlot = false;
                }

                unsigned long elElapsed  = elapsed - start;
                unsigned long elSlot     = elElapsed / 64;
                unsigned long nextElSlot = elSlot + 1;

                if ((nextElSlot * 64) < PREAMBLE_US) {
                    nextFrame = preamble(nextElSlot);
                } else {
                    nextFrame = elData(nextElSlot - (PREAMBLE_US / 64));
                }
                break;
            }
        }
    }

    // 32µs before slot boundary, send next slot's data
    if (timeInSlot >= 32 && !spiSentForSlot) {
        sendSPI(nextFrame);
        spiSentForSlot = true;
    }

    // at slot boundary, strobe to latch
    if (thisSlot > lastSlot) {
        lastSlot       = thisSlot;
        spiSentForSlot = false;
        strobeNow();
    }
}