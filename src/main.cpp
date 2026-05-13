#include <Arduino.h>
#include <RadioLib.h>
#include <XPowersLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>

// --- Pins ---
#define LORA_NSS 7
#define LORA_DIO1 33
#define LORA_NRST 8
#define LORA_BUSY 34
#define LORA_SCK 5
#define LORA_MISO 3
#define LORA_MOSI 6
#define PULSE_IN 16
#define PULSE_OUT 38
#define I2C_SDA 18
#define I2C_SCL 17
#define RST_PIN_AUX 36

// --- Debug flag ---
const bool DEBUG = true; // Just serial monitor stuff
const bool CLEANING_MODE = false; // clean memory, same as RST_PIN_AUX to ground

// --- General config ---
// THIS IS THE IMPORTANT PART for how it works
const int PULSE_DEBOUNCE_MS = 100; // Minimum time between pulses to avoid false counts
const int PULSE_DURATION_EXPECTED_MS = 50; // How long we expect, how long we produce
const int SCREEN_REFRESH_RATE_MS = 1000; // Caution, 40ms refresh time
const int SYNC_INTERVAL = 10; // number of pulses between syncs
const int INACTIVITY_TIMEOUT = 60000; // Time in ms to consider connection lost after no pulses
const int SYNC_TIMEOUT = 2000; // Time in ms to wait for sync response before considering it a failure
const int MAXIMUM_BUFFERED_PULSES = 10; // How many pulses are considered bufferable; above this counts as reboot, and does not resend pulses
const int MAX_SYNC_TRIES = 10; // How many times we try to sync before we reboot, should be low, like 10 or so

// --- Radio config ---
// THIS IS THE IMPORTANT PART for radio range
// Those values work well with SX1262, but not terribly well
// Don't run without antenna, 14dBm can fry receiver
// For better range SLOW DOWN
const int TRANSMIT_POWER = 14; // i think -9 to +22 for SX1262, later set up at max allowed (14dBm EU?)
const float LORA_BANDWIDTH = 125.0; // in kHz, can be 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 or 500
const int LORA_SPREADING_FACTOR = 9; // 6-12, higher is more robust but slower
const int LORA_CODING_RATE = 8; // 5-8, higher is more robust but slower
const int LORA_PREAMBLE_LENGTH = 16; // in symbols, can be 2-65535

// Only for keeping track
struct SystemStats {
    uint32_t totalPulses;
    uint32_t totalRebootsSender;
    uint32_t totalRebootsReceiver;
    uint32_t totalPulsesLost;
};

// --- Objects ---
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
XPowersAXP2101 PMU;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// --- Communication flags ---
// Used short strings, but not too short. Handle with care - keys are important for checking ghost messages.
const String ACK_FLAG = "ACK";
const String CONNECTION_REQUEST = "CONNREQ";
const String CONNECTION_ACK = "ACKCONN";
const String PULSE_MSG_PREFIX = "P";
const String SYNC_MSG = "SYNC";
const String RECEIVER_BOOT_MSG = "REC_BOOT_MSG";
const String SENDER_BOOT_ACK = "SEN_BOOT_ACK";
const String SENDER_BOOT_MSG = "SEN_BOOT_MSG";
const String RECEIVER_BOOT_ACK = "REC_BOOT_ACK";
const String RECEIVER_KEY = "REC";
const String SENDER_KEY = "SEN";


// --- Shared Vars ---
// Too little here, to much specific, will update this later on cleanup, maybe. 
static unsigned long lastScreenUpdate = 0;
volatile bool operationDone = false;
void IRAM_ATTR setFlag(void) { operationDone = true; }
bool isConnected = false;
SystemStats volatileStats = {0, 0, 0, 0};

// All saving and reading functions are not too great and really should not be trusted. 
// Those were meant for orientation, not for reliabile logging
void saveStatsToFlash(uint32_t PulseCount, uint32_t SenderReboots, uint32_t ReceiverReboots, uint32_t PulsesLost) {
    File file = LittleFS.open("/stats.bin", FILE_WRITE);
    if (file) {
        SystemStats stats = { PulseCount, SenderReboots, ReceiverReboots, PulsesLost };
        file.write((uint8_t*)&stats, sizeof(SystemStats));
        file.close();
        if (DEBUG) Serial.println("Flash: Stats backed up successfully.");
    }
}

SystemStats loadStatsFromFlash() {
    SystemStats stats = {0, 0, 0, 0};
    if (LittleFS.exists("/stats.bin")) {
        File file = LittleFS.open("/stats.bin", FILE_READ);
        if (file.read((uint8_t*)&stats, sizeof(SystemStats))) {
            // Stats loaded successfully
        } else {
            if (DEBUG) Serial.println("Flash: Failed to read stats, using defaults.");
            return stats;
        }
        file.close();
    }
    return stats;
}

SystemStats biggerStatsWin(SystemStats receiver, SystemStats sender) {
    SystemStats result;
    result.totalPulses = max(receiver.totalPulses, sender.totalPulses);
    result.totalRebootsSender = max(receiver.totalRebootsSender, sender.totalRebootsSender);
    result.totalRebootsReceiver = max(receiver.totalRebootsReceiver, sender.totalRebootsReceiver);
    result.totalPulsesLost = max(receiver.totalPulsesLost, sender.totalPulsesLost);
    return result;
}

SystemStats parseStatsFromMessage(String msg, String expectedPrefix) {
    SystemStats stats = {0, 0, 0, 0};
    if (msg.startsWith(expectedPrefix)) {
        int comma1 = msg.indexOf(',');
        int comma2 = msg.indexOf(',', comma1 + 1);
        int comma3 = msg.indexOf(',', comma2 + 1);
        stats.totalPulses = msg.substring(expectedPrefix.length(), comma1).toInt();
        stats.totalRebootsSender = msg.substring(comma1 + 1, comma2).toInt();
        stats.totalRebootsReceiver = msg.substring(comma2 + 1, comma3).toInt();
        stats.totalPulsesLost = msg.substring(comma3 + 1).toInt();
    }
    return stats;
}

void resetStats() {
    if (LittleFS.exists("/stats.bin")) {
        LittleFS.remove("/stats.bin");
    }
    volatileStats = {0, 0, 0, 0};
}

// --- Initialization Helper ---
void commonSetup() {
    Serial.begin(115200);

    if (DEBUG) delay(6000); // This is how long I need to switch to serial monitor tab and start 2 of those
    if (DEBUG) Serial.println("Starting up...");

    if(!LittleFS.begin(true)) { // 'true' means format if mount fails
        if (DEBUG) Serial.println("LittleFS Mount Failed");
        return;
    }
    if (DEBUG) Serial.println("LittleFS Mounted.");

    Wire.begin(I2C_SDA, I2C_SCL);

    // This only works if you do it on both devices
    // Resetting one will just give you stats from the other on startup
    // Do it the "look both ways" way, reset 1, reset 2, reset 1 again
    pinMode(RST_PIN_AUX, INPUT_PULLUP);
    if(digitalRead(RST_PIN_AUX) == LOW || CLEANING_MODE) {
        if (DEBUG) Serial.println("Reset pin held low, resetting stats...");
        resetStats();
    }
    
    // I have no idea why this is necessary, but it sometimes fails without it, sometimes not
    if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
        if (DEBUG) Serial.println("PMU Failed!");
    }
    PMU.setALDO2Voltage(3300); PMU.enableALDO2(); // LoRa
    PMU.setALDO4Voltage(3300); PMU.enableALDO4(); // OLED
    PMU.setBLDO1Voltage(3300); PMU.enableBLDO1(); // OLED

    delay(1000); // Let PMU stabilize before powering peripherals
    
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.printf("Booting up...\n");
    display.display();

    isConnected = false;

    volatileStats = loadStatsFromFlash();

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    Serial.print(F("Radio Init... "));
    int state = radio.begin(868.0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODING_RATE, 0xAB, TRANSMIT_POWER, LORA_PREAMBLE_LENGTH, 1.6);
    if (state == RADIOLIB_ERR_NONE) {
        if (DEBUG) Serial.println(F("OK!"));
    } else {
        if (DEBUG) Serial.printf("Failed (%d)\n", state);
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.printf("Radio Init Failed (%d)", state);
        display.display();
        delay(10000);
        ESP.restart();
    }
    radio.setPacketReceivedAction(setFlag);
    radio.setPacketSentAction(setFlag);
}

// --- Main Logic - SENDER ---

#ifdef ROLE_SENDER

volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
int lastAckedID = 0;
int currentSendingID = 0;
unsigned long transmitTimer = 0;
bool isWaitingForAck = false;
bool connectionEstablished = false;
int syncRetryCount = 0;
bool isSyncing = false;

// Interrupt is okay, works relatively nice
// Pulse debounce is important, I think it will crap out below 100ms
void IRAM_ATTR onPulse() {
    unsigned long now = millis();
    if (now - lastPulseTime > PULSE_DEBOUNCE_MS) {
        pulseCount++;
        lastPulseTime = now;
    }
}

// Works extremely well, though looks a bit sus
bool establishConnection() {
    unsigned long startTime = millis();
    int retryInterval = 1000; // Send request every 1s
    unsigned long lastSend = 0;

    Serial.println("Attempting Handshake...");
    while (millis() - startTime < 60000) { // 60 second total timeout
        // 1. Send the request periodically
        if (millis() - lastSend > retryInterval) {
            String rebootMsg = SENDER_BOOT_MSG + String(volatileStats.totalPulses) + "," + String(volatileStats.totalRebootsSender) + "," + String(volatileStats.totalRebootsReceiver) + "," + String(volatileStats.totalPulsesLost);
            radio.transmit(rebootMsg.c_str()); 
            radio.startReceive(); // Switch to listen mode
            lastSend = millis();
            Serial.println("Connection request sent...");
        }

        // 2. Check for response
        if (operationDone) {
            operationDone = false;
            String str;
            if (radio.readData(str) == RADIOLIB_ERR_NONE) {
                if (str.startsWith(RECEIVER_BOOT_ACK)) {
                    // Parse their stats
                    int comma1 = str.indexOf(',');
                    int comma2 = str.indexOf(',', comma1 + 1);
                    int comma3 = str.indexOf(',', comma2 + 1);
                    SystemStats receiverStats = parseStatsFromMessage(str, RECEIVER_BOOT_ACK);
                    // Determine new baseline using bigger wins logic
                    if (DEBUG) Serial.printf("Stats received: Pulses: %d, Sender Reboots: %d, Receiver Reboots: %d, Pulses Lost: %d\n", receiverStats.totalPulses, receiverStats.totalRebootsSender, receiverStats.totalRebootsReceiver, receiverStats.totalPulsesLost);
                    volatileStats = biggerStatsWin(volatileStats, receiverStats);
                    volatileStats.totalRebootsSender++; // Log reboot event for sender
                    if (currentSendingID < volatileStats.totalPulses) {
                        currentSendingID = volatileStats.totalPulses; // Catch up to what they have, we will resend the missing pulses in the normal flow
                    }
                    if (pulseCount < volatileStats.totalPulses) {
                        pulseCount = volatileStats.totalPulses; // Catch up pulse count as well, because important
                    }
                    if (DEBUG) Serial.printf("After bigger wins, stats are: Pulses: %d, Sender Reboots: %d, Receiver Reboots: %d, Pulses Lost: %d\n", volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
                    saveStatsToFlash(volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
                    if (DEBUG) Serial.println("Stats synchronized after sender reboot.");
                    return true; // Connection established!
                }
            }
            // If it wasn't the ACK, go back to listening
            radio.startReceive();
        }
        yield(); // Let ESP32 handle background tasks
    }
    return false;
}

void setup() {
    commonSetup();
    pinMode(PULSE_IN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PULSE_IN), onPulse, RISING);
    if (DEBUG) Serial.println("SENDER MODE");

    // Send connection request handshake
    connectionEstablished = establishConnection();
    if (!connectionEstablished) {
        if (DEBUG) Serial.println("Failed to establish connection. Check receiver and try again.");
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(2);
        display.printf("Connection Failed!");
        display.display();
        delay(10000);
        ESP.restart();
    }
}

void loop() {
    // Screen Update
    if (millis() - lastScreenUpdate > SCREEN_REFRESH_RATE_MS) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1.5);
        unsigned long totalMs = millis();
        unsigned long hours = totalMs / 3600000;
        unsigned long minutes = (totalMs % 3600000) / 60000;
        unsigned long seconds = (totalMs % 60000) / 1000;        
        display.printf("TX Count: %d\nAcked: %d\nRSSI: %.1f\nSNR: %.1f\nUptime: %02luh:%02lum:%02lus\nSTAT: P:%d,R.S:%d,R.R:%d,L:%d\n", pulseCount, lastAckedID, radio.getRSSI(), radio.getSNR(), hours, minutes, seconds, volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
        if (!isConnected) {
          display.printf("!!!Disconnected!!!\n");
        }
        display.display();
        lastScreenUpdate = millis();
    }

    // 1. Normal Operation: Send pulses in the window beteen syncs
    // We are just firing off whenever possible
    // No delay between sends, the lag of the radio and the receiver will take care of it
    if (!isSyncing && currentSendingID < pulseCount) {
        // Only send if we haven't hit the end of our current window
        if ((currentSendingID + 1) % SYNC_INTERVAL != 0) {
            currentSendingID++;
            radio.transmit((PULSE_MSG_PREFIX + String(currentSendingID)).c_str());
            radio.startReceive(); 
            if (DEBUG) Serial.printf("Sent Pulse #%d\n", currentSendingID);
        } else { // if we have hit the end of the window, we need to sync before sending more
            // First, send pulse as normal
            currentSendingID++;
            radio.transmit((PULSE_MSG_PREFIX + String(currentSendingID)).c_str());
            if (DEBUG) Serial.printf("Sent Pulse #%d\n", currentSendingID);
            delay(20); // Short delay to ensure pulse is sent before we switch to sync mode
            // Window reached! Time to ask for status
            isSyncing = true;
            transmitTimer = millis();
            radio.transmit(SYNC_MSG.c_str()); // S prefix for Status Query
            radio.startReceive();
            if (DEBUG) Serial.println("Window reached. Querying Status with sync...");
            // response handling in operationDone, also this is querying about if the receiver reboot or something
        }
    }

    // 2. Handle Radio Events
    if (operationDone) {
        operationDone = false;

        if (radio.getPacketLength() == 0) {
            radio.startReceive(); // Switch to listen mode
            return;               // EXIT: Do not parse as an ACK
        }
        // this check actually does not suffice, it just is there because it helps a bit
        if (radio.getPacketLength() > 0) {
            String str;
            radio.readData(str);

            if (str.startsWith(PULSE_MSG_PREFIX) || str == SYNC_MSG || str.startsWith(SENDER_KEY)) {
                // This is an unexpected sender message, likely from a previous window. Just ignore and listen again.
                // There is a loooot of ghost messages
                if (DEBUG) Serial.printf("Filtered echo pulse message: %s\n", str.c_str());
                radio.startReceive();
                return;
            }

            if (DEBUG) Serial.printf("Received: %s\n", str.c_str()); // Debug print of received message

            // This is just normal sync, probably
            if (str.startsWith(ACK_FLAG)) {
                int lastRx = str.substring(ACK_FLAG.length()).toInt();
                if (DEBUG) Serial.printf("Sync Success! Receiver is at: %d\n", lastRx);
                isConnected = true;
                lastAckedID = lastRx;
                currentSendingID = lastRx; // Reset our cursor to what they actually have
                isSyncing = false;
                syncRetryCount = 0; 
                radio.startReceive();
            } else if (str.startsWith(RECEIVER_BOOT_MSG)) {
                // This is where the receiver rebooted before or something else happened, resync please
                if (DEBUG) Serial.println("Receiver reboot detected during sync. Resending current window...");
                isConnected = true;
                String rebootMsg = SENDER_BOOT_ACK + String(volatileStats.totalPulses) + "," + String(volatileStats.totalRebootsSender) + "," + String(volatileStats.totalRebootsReceiver) + "," + String(volatileStats.totalPulsesLost);
                radio.transmit(rebootMsg.c_str()); 
                if (DEBUG) Serial.println("Data sent, no ACK expected.");
                radio.startReceive();
                SystemStats receiverStats = parseStatsFromMessage(str, RECEIVER_BOOT_MSG);
                if (DEBUG) Serial.printf("Stats received: Pulses: %d, Sender Reboots: %d, Receiver Reboots: %d, Pulses Lost: %d\n", receiverStats.totalPulses, receiverStats.totalRebootsSender, receiverStats.totalRebootsReceiver, receiverStats.totalPulsesLost);
                volatileStats = biggerStatsWin(volatileStats, receiverStats);
                if (currentSendingID < volatileStats.totalPulses) {
                  currentSendingID = volatileStats.totalPulses; // Catch up to what they have, we will resend the missing pulses in the normal flow
                }
                if (pulseCount < volatileStats.totalPulses) {
                  pulseCount = volatileStats.totalPulses; // Catch up pulse count as well, because important
                }
                if (DEBUG) Serial.printf("After bigger wins, stats are: Pulses: %d, Sender Reboots: %d, Receiver Reboots: %d, Pulses Lost: %d\n", volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
                volatileStats.totalRebootsReceiver++; // Log reboot event for receiver
                saveStatsToFlash(volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
            }         
        } else {
            radio.startReceive(); // Pulse/Query finished sending
        }
    }

    // 3. Sync Timeout & Reboot Logic
    if (isSyncing && (millis() - transmitTimer > SYNC_TIMEOUT)) {
        // This just waits until timeout, which is quite short
        // we can't afford to just wait, we move on and fire off pulses 
        // after a number of failed syncs, we reboot
        // make it low, like 10 or so
        syncRetryCount++;
        if (DEBUG) Serial.printf("Sync Timeout %d/%d\n", syncRetryCount, MAX_SYNC_TRIES);
        isConnected = false;
        isSyncing = false; // Reset syncing flag to allow normal operation before we retry
        
        if (syncRetryCount >= MAX_SYNC_TRIES) {
            if (DEBUG) Serial.println("CRITICAL FAILURE: Rebooting...");
            saveStatsToFlash(volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
            ESP.restart();
        }
    }

}
#endif

// --- Main Logic - RECEIVER ---

#ifdef ROLE_RECEIVER
int lastReceivedID = 0;
unsigned long pulseStartTime = 0;
unsigned long pulseEndTime = 0;
bool pulseActive = false;
int missedCount = 0;
int missedBuffer = 0;
unsigned long lastPulseTime = 0;

bool wasRebootFlag = false; // This is used to hold the flag until sync is called, because otherwise sender won't listen

// This radio absolutely needs its statuses in order, unfortunately
// This is one of the worst solutions, but it is a solution, and it is extremely light on load
// It works better than doing it any other way in the end
// Reliable as hell
enum RadioState { LISTENING, SENDING };
RadioState currentRadioState = LISTENING;

void setup() {
  // here we don't wait for anything, just boot and wait for signal
    commonSetup();
    pinMode(PULSE_OUT, OUTPUT);
    digitalWrite(PULSE_OUT, HIGH);
    radio.startReceive();
    Serial.println("RECEIVER MODE");
}

void loop() {
    // Screen Update 
    if (millis() - lastScreenUpdate > SCREEN_REFRESH_RATE_MS) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1.5);
        unsigned long totalMs = millis();
        unsigned long hours = totalMs / 3600000;
        unsigned long minutes = (totalMs % 3600000) / 60000;
        unsigned long seconds = (totalMs % 60000) / 1000;
        display.printf("RX ID: %d\nRSSI: %.1f\nSNR: %.1f\nLost pulses: %d\nCurrent buffer: %d\nUptime: %02luh:%02lum:%02lus\nStats: P:%d,R.S:%d,R.R:%d,L:%d\n", lastReceivedID, radio.getRSSI(), radio.getSNR(), missedCount, missedBuffer, hours, minutes, seconds, volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
        if (!isConnected) {
          display.printf("!!!Disconnected!!!");
        }
        display.display();
        lastScreenUpdate = millis();
    }
    
    // 2. Handle a timed pulse
    if (pulseActive && (millis() - pulseStartTime >= PULSE_DURATION_EXPECTED_MS)) {
        digitalWrite(PULSE_OUT, HIGH); // End the pulse
        pulseActive = false;
        if (DEBUG) Serial.printf("Pulse %d Ended (%d)\n", lastReceivedID, PULSE_DURATION_EXPECTED_MS); // yes, this is actually incorrect
    }

    if (millis() - lastPulseTime > INACTIVITY_TIMEOUT) {
        // Consider connection lost after a period of inactivity
        // mostly for display / debug
        isConnected = false;
    }

    if (missedBuffer > 0 && millis() - pulseEndTime > PULSE_DURATION_EXPECTED_MS)
    {
      // This fires because pulses can get stuck in various ways
      // it waits at least a pulse duration after other pulse to fire
      // so at 1/pulse duration Hz there might be an issue, obviously
        digitalWrite(PULSE_OUT, LOW);
        pulseStartTime = millis();
        pulseActive = true;
        missedBuffer--; 
    }

    if (operationDone) {
      operationDone = false;
        
      if (radio.getPacketLength() > 0) {
        String str;
        radio.readData(str);

        if (str.startsWith(ACK_FLAG) || str.startsWith(RECEIVER_KEY)) {
            // This is an unexpected ack message, likely from a previous window. Just ignore and listen again.
            if (DEBUG) Serial.printf("Filtered echo ack message: %s\n", str.c_str());
            radio.startReceive();
            return;
        }

        if (DEBUG) Serial.printf("Received: %s\n", str.c_str()); // Debug print of received message
        if (DEBUG) Serial.printf("Last id: %d\n", lastReceivedID); // Debug print of last received ID for comparison

        // CASE 0: Check for Handshake
        if (str.startsWith(SENDER_BOOT_MSG)) {
          // this is sender being reboot, or booting up first time in general
            volatileStats.totalPulses = lastReceivedID; // Update pulse count to last received, because we are resyncing due to reboot or something
            volatileStats.totalPulsesLost = missedCount; // Update lost pulses before sending stats, so that it is included in the reboot ACK
            currentRadioState = SENDING;
            String rebootMsg = RECEIVER_BOOT_ACK + String(volatileStats.totalPulses) + "," + String(volatileStats.totalRebootsSender) + "," + String(volatileStats.totalRebootsReceiver) + "," + String(volatileStats.totalPulsesLost);
            radio.transmit(rebootMsg.c_str());
            radio.startReceive(); 
            isConnected = true;
            if (DEBUG) Serial.println("Sender reset detected. Handshake sent.");

            SystemStats senderStats = parseStatsFromMessage(str, SENDER_BOOT_MSG);

            // Determine new baseline using bigger wins logic
            volatileStats = biggerStatsWin(volatileStats, senderStats);
            lastReceivedID = volatileStats.totalPulses; // Catch up to what they have, we will resend the missing pulses in the normal flow
            missedCount = volatileStats.totalPulsesLost; // Update missed count to reflect the bigger wins result, so that it is accurate after a reboot
            volatileStats.totalRebootsSender++; // Log reboot event for sender
            saveStatsToFlash(volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
            if (DEBUG) Serial.println("Stats synchronized after sender reboot.");
        } 

        // CASE A: Status Query
        else if (str == SYNC_MSG) {
          if (!wasRebootFlag) {
            if (DEBUG) Serial.printf("Status Query Received. Replying %d...\n", lastReceivedID);
            delay(20); 
            radio.transmit((ACK_FLAG + String(lastReceivedID)).c_str());
            radio.startReceive();
            isConnected = true;
          } else {
            wasRebootFlag = false; // Reset the flag after acknowledging the reboot
            if (DEBUG) Serial.println("Status Query Received, but reboot flag is set. Sending reboot ACK instead...");
            volatileStats.totalPulses = lastReceivedID; // Update pulse count to last received, because we are resyncing due to reboot or something
            volatileStats.totalPulsesLost = missedCount; // Update lost pulses before sending stats, so that it is included in the reboot ACK
            String rebootAckMsg = RECEIVER_BOOT_MSG + String(volatileStats.totalPulses) + "," + String(volatileStats.totalRebootsSender) + "," + String(volatileStats.totalRebootsReceiver) + "," + String(volatileStats.totalPulsesLost);
            radio.transmit(rebootAckMsg.c_str());
            radio.startReceive();
            SystemStats senderStats = parseStatsFromMessage(str, SENDER_BOOT_ACK);

            // Determine new baseline using bigger wins logic
            volatileStats = biggerStatsWin(volatileStats, senderStats);
            lastReceivedID = volatileStats.totalPulses; // Catch up to what they have, we will resend the missing pulses in the normal flow
            missedCount = volatileStats.totalPulsesLost; // Update missed count to reflect the bigger wins result, so that it is accurate after a reboot
            volatileStats.totalRebootsSender++; // Log reboot event for sender
            saveStatsToFlash(volatileStats.totalPulses, volatileStats.totalRebootsSender, volatileStats.totalRebootsReceiver, volatileStats.totalPulsesLost);
            if (DEBUG) Serial.println("Stats synchronized after receiver reboot.");

          }
            
        }
        
        // CASE B: Pulse Message
        else if (str.startsWith(PULSE_MSG_PREFIX)) {
          int id = str.substring(PULSE_MSG_PREFIX.length()).toInt();
          isConnected = true;
          if (id == lastReceivedID + 1) {
            if (pulseActive || millis() - pulseEndTime < PULSE_DURATION_EXPECTED_MS) {
              // Too fast
              lastReceivedID = id;
              missedBuffer++;
              lastPulseTime = millis();
            } else {
              // Perfect sequence
              lastReceivedID = id;
              digitalWrite(PULSE_OUT, LOW);
              pulseStartTime = millis();
              lastPulseTime = millis();
              pulseActive = true;
            }
          } else if (id > lastReceivedID + 1) {
            // We have a gap, but is it a buffer or a reboot?
            if (id - lastReceivedID - 1 > MAXIMUM_BUFFERED_PULSES) {
              // Reboot per rules, or the gap was too extensive, need resync
              wasRebootFlag = true; // Set the reboot flag, so that next sync will call for a stats sync
              if (DEBUG) Serial.printf("Gap! Expected %d, got %d\n", lastReceivedID + 1, id);
              missedCount = missedCount + id - lastReceivedID - 1;
              lastReceivedID = id; // temporary
              digitalWrite(PULSE_OUT, LOW); // temporary
              pulseStartTime = millis(); // temporary
              lastPulseTime = millis(); // temporary
              pulseActive = true; // temporary
            } else {
              // Just a small missed buffer, we can handle it
              missedBuffer = missedBuffer + id - lastReceivedID - 1;
              lastReceivedID = id; // Move forward in this case, normally
              digitalWrite(PULSE_OUT, LOW);
              pulseStartTime = millis();
              lastPulseTime = millis();
              pulseActive = true;
            }
             
          }
        }
        radio.startReceive();
      } else {
          radio.startReceive(); // ACK finished sending
      }
    }
}
#endif
