// INPUT PINS
#define START_BTN     2
#define COUNTER_SW    3 // Must be interrupt pin
#define KNIFE_O_SW    4
#define KNIFE_C_SW    5
#define CONFIG_READ   17 // (A3)
#define POSTPONE_BTN  18 // (A4)
#define FILMBREAK_BTN 19 // (A5)

// OUTPUT PINS
#define RELAYS_SCLK  6
#define RELAYS_LATCH 7
#define RELAYS_OE    8
#define RELAYS_SDI   9
#define LCD_SDI      10
#define LCD_SCLK     11
#define LCD_LATCH    12
#define BEEPER       13
#define CONFIG_SCLK  14 // (A0)
#define CONFIG_LATCH 15 // (A1)
#define CONFIG_SDI   16 // (A2)

// RELAYS (1 is skipped to ease connecting outputs to 74HC595 shift register)
#define COUNTER_RELAY     2
#define KNIFE_OPEN_RELAY  4
#define KNIFE_CLOSE_RELAY 8
#define ROTOR_ON_RELAY    16
#define ROTOR_FAST_RELAY  32
#define MAIN_RELAYS       64

// ERROR CODES
#define ERROR_CODE_FILM  1
#define ERROR_CODE_KNIFE 2

// TIMES
#define DEBOUNCE_DELAY      50
#define FILM_BREAK_TIMEOUT  500
#define SLOW_START_DELAY    1000
#define SLOW_STOP_DELAY     1000
#define ALERT_BEEP_DURATION 2000

// ENUMS

// States
enum rotorState
{
    rotorStopped,
    rotorSlow,
    rotorFast,
    rotorSlowPaused,
    rotorFastPaused
};

enum knifeState
{
    knifeStopped,
    knifeClosing,
    knifeOpening
};

// Event types
enum counterEventType
{
    counterNone,
    counterFalling,
    counterRising
};

enum knifeEventType
{
    knifeNone,
    knifeClosed,
    knifeOpened,
    knifeUnlocked
};

// CONSTANTS

const uint8_t letterE = B01100001;
const uint8_t digits [] = {
    B00000011, // 0
    B10011111, // 1
    B00100101, // 2
    B00001101, // 3
    B10011001, // 4
    B01001001, // 5
    B01000001, // 6
    B00011111, // 7
    B00000001, // 8
    B00001001  // 9
};


// VARIABLES

// Config
uint8_t rawConfig = 0;
uint8_t maxRevs = 10;

// Input switches
uint8_t startBtn = HIGH;
uint64_t startBtnTime = 0;

uint8_t counterSw = HIGH;
uint64_t counterSwTime = 0;

uint8_t knifeOpenSw = HIGH;
uint64_t knifeOpenSwTime = 0;

uint8_t knifeClosedSw = HIGH;
uint64_t knifeClosedSwTime = 0;

uint8_t postponeBtn = HIGH;
uint64_t postponeBtnTime = 0;

uint8_t filmBreakBtn = HIGH;
uint64_t filmBreakBtnTime = 0;

// Event flags
bool startPressed = false;
counterEventType counterEvent = counterNone;
knifeEventType knifeEvent = knifeNone;
bool postponePressed = false;
bool filmBreakPressed = false;

// State
bool running = false;
uint8_t revCount = 0;
rotorState rotor = rotorStopped;
knifeState knife = knifeStopped;
uint64_t counterFallingTime = 0;
bool knifeAlarm = false;
bool filmBreak = false;
bool releaseFilm = false;
bool finishing = false;
bool cutFilm = false;
bool abortFinish = false;

// Timers (value of 0 = inactive)
uint64_t rotorSlowStartTime = 0;
uint64_t rotorSlowStopTime = 0;
uint64_t alertBeepStartTime = 0;

// Outputs (bit fields, all outputs are active-low)
uint8_t outputsOld = 255;
uint8_t outputsNew = 255;



// PROTOTYPES (apparently needed for functions with arguments if invoked before definition)

void rotorPause(bool emergencyStop=false);



// MAIN PARTS



void loadConfig()
{
    // Clear config variable
    rawConfig = 0;

    // Check all 8 switches
    for (uint8_t i = 0; i < 8; i++)
    {
        // Set byte with one "1" corresponding to the switch being checked
        uint8_t bitValue = 1<<i;

        // Send this byte to the shift register
        shiftOut(CONFIG_SDI, CONFIG_SCLK, MSBFIRST, 255 - bitValue);

        // Cycle the shift register storage clock / latch pin to output the stored value
        digitalWrite(CONFIG_LATCH, HIGH);
        digitalWrite(CONFIG_LATCH, LOW);

        // Add the value of the byte to rawConfig if checked switch is on
        rawConfig += digitalRead(CONFIG_READ) == LOW ? bitValue : 0;
    }

    // Save last 6 bits as maxRevs
    maxRevs = rawConfig & B00111111;

    // First 2 bits are discarded (for now)

    // If start button is pressed
    if (digitalRead(START_BTN) == LOW)
    {
        // Display configured revs
        lcdShowNumber(maxRevs);

        // Wait until button is released
        while (digitalRead(START_BTN) == LOW)
        {
        }

        // Wait a bit for debounce (to not trigger actual start accidentally)
        delay(DEBOUNCE_DELAY);
    }
}



void setup()
{
    Serial.begin(9600);

    // Set pin modes for inputs
    pinMode(COUNTER_SW, INPUT_PULLUP);
    pinMode(KNIFE_O_SW, INPUT_PULLUP);
    pinMode(KNIFE_C_SW, INPUT_PULLUP);
    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(CONFIG_READ, INPUT_PULLUP);
    pinMode(POSTPONE_BTN, INPUT_PULLUP);
    pinMode(FILMBREAK_BTN, INPUT_PULLUP);

    // Read and save initial input states
    counterSw = digitalRead(COUNTER_SW);
    startBtn = digitalRead(START_BTN);
    knifeClosedSw = digitalRead(KNIFE_C_SW);
    knifeOpenSw = digitalRead(KNIFE_O_SW);
    postponeBtn = digitalRead(POSTPONE_BTN);
    filmBreakBtn = digitalRead(FILMBREAK_BTN);

    // Set pin modes for outputs
    pinMode(RELAYS_SDI, OUTPUT);
    pinMode(RELAYS_SCLK, OUTPUT);
    pinMode(RELAYS_LATCH, OUTPUT);
    pinMode(BEEPER, OUTPUT);
    pinMode(LCD_SDI, OUTPUT);
    pinMode(LCD_SCLK, OUTPUT);
    pinMode(LCD_LATCH, OUTPUT);
    pinMode(CONFIG_SDI, OUTPUT);
    pinMode(CONFIG_SCLK, OUTPUT);
    pinMode(CONFIG_LATCH, OUTPUT);

    // Configure ISR for counter signal
    attachInterrupt(digitalPinToInterrupt(COUNTER_SW), counterChangeInterrupt, CHANGE);

    // Load config
    loadConfig();

    // Reset program state
    resetProgram();

    // Set output-enable for relay shift register
    digitalWrite(RELAYS_OE, LOW);
    pinMode(RELAYS_OE, OUTPUT);
}



void loop()
{
    // Check inputs, save their state and generate event flags
    checkInputs();

    // Check event flags
    checkEventFlags();

    // Disregard rest of loop if program is not running
    if (!running)
    {
        return;
    }

    // Check timers
    checkTimers();

    // Set outputs from state
    stateToOutputs();
}



// LOOP PARTS



void checkInputs()
{
    // DEBOUNCED INPUTS (except counter falling/rising, those are done in their ISR)

    // If start button is either pressed or released, taking bounce into account
    if (digitalRead(START_BTN) != startBtn && millis() - startBtnTime > DEBOUNCE_DELAY)
    {
        // Toggle button status
        startBtn = negateHiLo(startBtn);

        // Save current time for future debounce
        startBtnTime = millis();

        // Set flag if button is now on LOW
        if (startBtn == LOW)
        {
            startPressed = true;
        }
    }

    // If knife open switch is either pressed or released, taking bounce into account
    if (digitalRead(KNIFE_O_SW) != knifeOpenSw && millis() - knifeOpenSwTime > DEBOUNCE_DELAY)
    {
        // Toggle switch status
        knifeOpenSw = negateHiLo(knifeOpenSw);

        // Save current time for future debounce
        knifeOpenSwTime = millis();

        // Set flag if switch is now on LOW
        if (knifeOpenSw == LOW)
        {
            knifeEvent = knifeOpened;
        }
        // If this switch was released, and the other one is not already pressed
        else if (knifeClosedSw == HIGH)
        {
            // Set flag
            knifeEvent = knifeUnlocked;
        }
    }

    // If knife close switch is either pressed or released, taking bounce into account
    if (digitalRead(KNIFE_C_SW) != knifeClosedSw && millis() - knifeClosedSwTime > DEBOUNCE_DELAY)
    {
        // Toggle switch status
        knifeClosedSw = negateHiLo(knifeClosedSw);

        // Save current time for future debounce
        knifeClosedSwTime = millis();

        // Set flag if switch is now on LOW
        if (knifeClosedSw == LOW)
        {
            knifeEvent = knifeClosed;
        }
        // If this switch was released, and the other one is not already pressed
        else if (knifeOpenSw == HIGH)
        {
            // Set flag
            knifeEvent = knifeUnlocked;
        }
    }

    // If postpone button is either pressed or released, taking bounce into account
    if (digitalRead(POSTPONE_BTN) != postponeBtn && millis() - postponeBtnTime > DEBOUNCE_DELAY)
    {
        // Toggle switch status
        postponeBtn = negateHiLo(postponeBtn);

        // Save current time for future debounce
        postponeBtnTime = millis();

        // Set flag if switch is now on LOW
        if (postponeBtn == LOW)
        {
            // Set flag
            postponePressed = true;
        }
    }

    // If film break button is either pressed or released, taking bounce into account
    if (digitalRead(FILMBREAK_BTN) != filmBreakBtn && millis() - filmBreakBtnTime > DEBOUNCE_DELAY)
    {
        // Toggle switch status
        filmBreakBtn = negateHiLo(filmBreakBtn);

        // Save current time for future debounce
        filmBreakBtnTime = millis();

        // Set flag if switch is now on LOW
        if (filmBreakBtn == LOW)
        {
            // Set flag
            filmBreakPressed = true;
        }
    }
}



void checkEventFlags()
{
    // If start button was pressed
    if (startPressed)
    {
        // Start program if not running
        if (!running)
        {
            // Beep-beep
            okBeep();

            // Set running flag
            running = true;

            // Ensure counter is reset and show on lcd
            // NOTE: Needed? Doesn't resetProgram() already do this?
            revCount = 0;
            lcdShowNumber(revCount);

            // Start rotor
            rotorSlowStart();

            if (knifeOpenSw == HIGH && knifeClosedSw == HIGH)
            {
                knifeEvent = knifeUnlocked;
            }
        }
        // Speed up rotor again if slow stopping
        else if (rotor == rotorSlow && rotorSlowStopTime != 0)
        {
            // Beep
            okBeep();

            // Speed up rotor
            rotor = rotorFast;

            // Reset slow stop timer
            rotorSlowStopTime = 0;
        }
        // Pause rotor if on half or full speed
        else if (rotor == rotorSlow || rotor == rotorFast)
        {
            // Beep
            okBeep();

            // Pause rotor
            rotorPause();
        }
        // Restart rotor if paused
        else if (rotor == rotorSlowPaused || rotor == rotorFastPaused)
        {
            // Beep
            okBeep();

            // Restart rotor
            rotorRestart();

            // If film break flag is set
            if (filmBreak)
            {
                // Clear flag
                filmBreak = false;

                // Set release film flag if knife is closed
                if (knifeClosedSw == LOW)
                {
                    releaseFilm = true;
                }

                // Set LCD to show counter and not error msg
                lcdShowNumber(revCount);
            }
        }

        // Reset flag
        startPressed = false;
    }

    // Disregard and reset rest of events if program is not running
    if (!running)
    {
        counterEvent = counterNone;
        knifeEvent = knifeNone;
        postponePressed = false;
        filmBreakPressed = false;

        return;
    }

    // If knife was opened
    if (knifeEvent == knifeOpened)
    {
        // Stop knife if was opening
        if (knife == knifeOpening)
        {
            knife = knifeStopped;

            // If release film flag is set
            if (releaseFilm)
            {
                // Speed up rotor
                rotorSpeedUp();

                // Reset flag
                releaseFilm = false;
            }

            // If abort finish flag is set
            if (abortFinish)
            {
                // Restart rotor
                rotorSlowStart();

                // Reset flag
                abortFinish = false;
            }
        }

        // Clear knife alarm if active
        if (knifeAlarm)
        {
            // Set LCD to show counter and not error msg
            lcdShowNumber(revCount);

            // Beep
            okBeep();

            // Reset flag
            knifeAlarm = false;
        }

        // Reset flag
        knifeEvent = knifeNone;
    }
    // If knife was closed
    else if (knifeEvent == knifeClosed)
    {
        // Stop knife if was closing
        if (knife == knifeClosing)
        {
            knife = knifeStopped;

            // Reset if cut film flag is set
            if (cutFilm)
            {
                resetProgram();
            }
        }

        // Clear knife alarm if active
        if (knifeAlarm)
        {
            // Set LCD to show counter and not error msg
            lcdShowNumber(revCount);

            // Beep
            okBeep();

            // Reset flag
            knifeAlarm = false;
        }

        // Reset flag
        knifeEvent = knifeNone;
    }
    // If knife was unlocked (both limit switches off)
    if (knifeEvent == knifeUnlocked)
    {
        // If knife is not supposed to be moving and rotor is moving
        if (knife == knifeStopped && (rotor == rotorSlow || rotor == rotorFast))
        {
            // Pause rotor
            rotorPause(true);

            // Alert user
            alert(ERROR_CODE_KNIFE);

            // Set flag
            knifeAlarm = true;
        }
    }

    // If counter switch was activated
    if (counterEvent == counterFalling)
    {
        // If film break flag is not set
        if (!filmBreak)
        {
            // Increase rev count
            revCount++;

            // Set release film flag if just finished first rev and knife is not open
            if (revCount == 1 && knifeOpenSw == HIGH)
            {
                releaseFilm = true;
            }

            // Update LCD if knife alarm is also not set
            if (!knifeAlarm)
            {
                lcdShowNumber(revCount);
            }
        }

        // If finishing
        if (finishing)
        {
            // Stop rotor
            rotor = rotorStopped;

            // Set cut film flag
            // NOTE: Shouldn't the knife close here directly?
            cutFilm = true;
        }
        // Else if not finishing
        else
        {
            // Save time for film break detection
            counterFallingTime = millis();
        }

        // Reset flag
        counterEvent = counterNone;
    }
    // If counter switch was deactivated
    else if (counterEvent == counterRising)
    {
        // Treat as film break if counter switch was just activated (unless finishing flag is set)
        if (millis() - counterFallingTime < FILM_BREAK_TIMEOUT && !finishing)
        {
            // Slow down rotor
            rotorSlowDown();

            // Set flag
            filmBreak = true;

            // Alert user
            alert(ERROR_CODE_FILM);
        }
        // If film break flag is set
        else if (filmBreak)
        {
            // Pause rotor
            rotorPause();

            // Discard last rev (no need to display as error code will be shown)
            revCount--;

            // Fake postpone button press if finishing flag is set
            if (finishing)
            {
                // Set postpone event flag
                postponePressed = true;
            }
        }
        // If release film flag is set
        else if (releaseFilm)
        {
            // Slow down rotor
            rotorSlowDown();

            // Open knife
            knife = knifeOpening;
        }
        // If maxRevs is reached, postpone button is not pressed and cut film flag is not set
        else if (revCount >= maxRevs - 1 && postponeBtn == HIGH && !cutFilm)
        {
            // Set release film flag to open knife if closed
            if (knifeOpenSw == HIGH)
            {
                releaseFilm = true;
            }
            // Slow down and set finishing flag if knife is open
            else
            {
                // Slow down
                rotorSlowDown();

                // Set flag
                finishing = true;
            }
        }
        // If cut film flag is set
        else if (cutFilm)
        {
            // Close knife
            // NOTE: is this right? closing knife without stopping rotor. Shouldn't knife close at counterFalling (forward position)?
            // > cutFilm flag is set at counterFalling but reacted to here? whut?
            knife = knifeClosing;
        }

        // Reset flag
        counterEvent = counterNone;
    }

    // If postpone button was pressed
    if (postponePressed)
    {
        // If finishing is set and rotor is not yet stopped
        if (finishing && rotor != rotorStopped)
        {
            // Speed up rotor
            rotorSpeedUp();
        }
        else if (finishing && rotor == rotorStopped)
        {
            // If knife is not open
            if (knifeOpenSw == HIGH)
            {
                // Open knife
                knife = knifeOpening;

                // Set flag
                abortFinish = true;
            }
            // If knife is still open
            else
            {
                // Ensure knife is stopped
                knife = knifeStopped;

                // Restart rotor
                rotorSlowStart();
            }
        }

        // Clear finishing flag
        finishing = false;

        // Clear flag
        postponePressed = false;
    }

    // If film break button was pressed
    if (filmBreakPressed)
    {
        // Slow down rotor
        rotorSlowDown();

        // Set flag
        filmBreak = true;

        // Clear finishing flag if set
        finishing = false;

        // Beep
        okBeep();

        // Clear flag
        filmBreakPressed = false;
    }
}



void checkTimers()
{
    // If rotor slow start is active and finished
    if (rotorSlowStartTime != 0 && millis() - rotorSlowStartTime > SLOW_START_DELAY)
    {
        // Reset timer
        rotorSlowStartTime = 0;

        // Speed up rotor
        rotorSpeedUp();
    }
    // If rotor slow stop is active and finished
    else if (rotorSlowStopTime != 0 && millis() - rotorSlowStopTime > SLOW_STOP_DELAY)
    {
        // Stop rotor
        rotor = rotorFastPaused;

        // Reset timer
        rotorSlowStopTime = 0;
    }

    // If alert beep is active and finished
    if (alertBeepStartTime != 0 && millis() - alertBeepStartTime > ALERT_BEEP_DURATION)
    {
        // Cancel beeper
        alertCancel();

        // Reset timer
        alertBeepStartTime = 0;
    }
}



void stateToOutputs()
{
    // Reset desired output states
    outputsNew = 255;

    // If program is running
    if (running)
    {
        // Redirect counter signal to arduino
        outputsNew &= ~COUNTER_RELAY;
    }

    // If rotor is on slow
    if (rotor == rotorSlow)
    {
        // Enable rotor
        outputsNew &= ~ROTOR_ON_RELAY;
    }
    // If rotor is on fast
    else if (rotor == rotorFast)
    {
        // Enable rotor and fast speed
        outputsNew &= ~ROTOR_ON_RELAY & ~ROTOR_FAST_RELAY;
    }

    // If knife is on closing
    if (knife == knifeClosing)
    {
        // Enable knife closing
        outputsNew &= ~KNIFE_CLOSE_RELAY;
    }
    // If knife is on opening
    else if (knife == knifeOpening)
    {
        // Enable knife opening
        outputsNew &= ~KNIFE_OPEN_RELAY;
    }

    // Activate MAIN_RELAYS if any output other than COUNTER_RELAY is active (=0)
    if ((uint8_t)~(outputsNew | COUNTER_RELAY))
    {
        outputsNew &= ~MAIN_RELAYS;
    }

    // If output state has changed from last iteration
    if (outputsNew != outputsOld)
    {
        // Update output shift register
        sendToRelays(outputsNew);

        // Save new state for next iteration
        outputsOld = outputsNew;
    }
}



void resetProgram()
{
    // Reset event flags
    startPressed = false;
    counterEvent = counterNone;
    knifeEvent = knifeNone;
    postponePressed = false;
    filmBreakPressed = false;

    // Reset state
    running = false;
    revCount = 0;
    rotor = rotorStopped;
    knife = knifeStopped;
    counterFallingTime = 0;
    knifeAlarm = false;
    releaseFilm = false;
    filmBreak = false;
    cutFilm = false;
    finishing = false;

    // Reset timers
    rotorSlowStartTime = 0;
    rotorSlowStopTime = 0;
    alertBeepStartTime = 0;

    // Reset output states
    outputsOld = 255;
    outputsNew = 255;

    // Reset output pins
    digitalWrite(RELAYS_SDI, LOW);
    digitalWrite(RELAYS_SCLK, LOW);
    digitalWrite(RELAYS_LATCH, LOW);
    digitalWrite(BEEPER, LOW);
    digitalWrite(LCD_SDI, LOW);
    digitalWrite(LCD_SCLK, LOW);
    digitalWrite(LCD_LATCH, LOW);
    digitalWrite(CONFIG_SDI, LOW);
    digitalWrite(CONFIG_SCLK, LOW);
    digitalWrite(CONFIG_LATCH, LOW);

    // Reset relay outputs
    sendToRelays(255);

    // Indicate ready
    // lcdShow(B10100111, B11001011);
    lcdShow(B11000101, B11010101);
    okBeep();

    Serial.println("on");
}



// INTERRUPTS



void counterChangeInterrupt()
{
    // Debounce counter switch, if actually changed then set falling/rising flags
    if (digitalRead(COUNTER_SW) != counterSw && millis() - counterSwTime > DEBOUNCE_DELAY)
    {
        counterSw = negateHiLo(counterSw);
        counterSwTime = millis();

        // Set flag according to counter pin value
        counterEvent = counterSw == LOW ? counterFalling : counterRising;
    }
}



// ACTIONS



void rotorSlowStart()
{
    // Set rotor to slow
    rotor = rotorSlow;

    // Set timer for acceleration
    rotorSlowStartTime = millis();
}



void rotorPause(bool emergencyStop = false)
{
    // If rotor is on slow and not only because of slow start or slow stop
    if (rotor == rotorSlow && rotorSlowStartTime == 0 && rotorSlowStopTime == 0)
    {
        // Set rotor to slow paused
        rotor = rotorSlowPaused;
    }
    // If rotor is on slow but because of slow start
    else if (rotor == rotorSlow && rotorSlowStartTime != 0)
    {
        // Set rotor to fast paused
        rotor = rotorFastPaused;
    }
    // If rotor is on fast
    else if (rotor == rotorFast)
    {
        // Stop immediately (no slow-stop) if emergency stop parameter is true
        if (emergencyStop)
        {
            rotor = rotorFastPaused;
        }
        // Slow down and start slow stop timer if emergency stop parameter is false or not set
        else
        {
            // Set rotor to slow
            rotor = rotorSlow;

            // Set slow stop timer
            rotorSlowStopTime = millis();
        }
    }
    // If rotor is on slow because of slow stop and emergency stop parameter is true
    else if (rotor == rotorSlow && rotorSlowStopTime != 0 && emergencyStop)
    {
        // Stop rotor
        rotor = rotorFastPaused;

        // Reset slow stop timer
        rotorSlowStopTime = 0;
    }
}



void rotorRestart()
{
    // Start rotor if set to slowPaused
    if (rotor == rotorSlowPaused)
    {
        rotor = rotorSlow;
    }
    // Slow start rotor if set to fastPaused
    else if (rotor == rotorFastPaused)
    {
        rotorSlowStart();
    }
}



void rotorSpeedUp()
{
    // Set rotor to fast if on slow and not only because of slow stop but not if slow start is already active
    if (rotor == rotorSlow && rotorSlowStopTime == 0 && rotorSlowStartTime == 0)
    {
        rotor = rotorFast;
    }

    // Set rotor to fast paused if on slow paused
    if (rotor == rotorSlowPaused)
    {
        rotor = rotorFastPaused;
    }
}



void rotorSlowDown()
{
    // Set rotor to slow if on fast
    if (rotor == rotorFast)
    {
        rotor = rotorSlow;
    }

    // Set rotor to slow paused if on fast paused
    if (rotor == rotorFastPaused)
    {
        rotor = rotorSlowPaused;
    }

    // Cancel any queued slow start
    rotorSlowStartTime = 0;
}



void alert(uint8_t code)
{
    // Send to LCD
    lcdShow(letterE, digits[code]);

    Serial.println('E' + String(code));

    // Start beeping and save current time
    digitalWrite(BEEPER, HIGH);
    alertBeepStartTime = millis();
}



void alertCancel()
{
    digitalWrite(BEEPER, LOW);
}



void okBeep()
{
    // Short beep-beep
    digitalWrite(BEEPER, HIGH);
    delay(100);
    digitalWrite(BEEPER, LOW);
    delay(100);
    digitalWrite(BEEPER, HIGH);
    delay(100);
    digitalWrite(BEEPER, LOW);
}



void lcdShowNumber(uint8_t n)
{
    // Calculate digits
    uint8_t firstDigit = n / 10;
    uint8_t secondDigit = n - (firstDigit * 10);

    // Send to LCD
    lcdShow(digits[firstDigit], digits[secondDigit]);

    Serial.println(n);
}



void lcdShow(uint8_t a, uint8_t b)
{
    // Shift out to LCD
    shiftOut(LCD_SDI, LCD_SCLK, LSBFIRST, b);
    shiftOut(LCD_SDI, LCD_SCLK, LSBFIRST, a);

    // Toggle latch pin
    digitalWrite(LCD_LATCH, LOW);
    digitalWrite(LCD_LATCH, HIGH);
}



void sendToRelays(uint8_t value)
{
    shiftOut(RELAYS_SDI, RELAYS_SCLK, MSBFIRST, value);
    digitalWrite(RELAYS_LATCH, HIGH);
    digitalWrite(RELAYS_LATCH, LOW);
}



// GENERAL UTILITIES



uint8_t negateHiLo(uint8_t value)
{
    return value == HIGH ? LOW : HIGH;
}
