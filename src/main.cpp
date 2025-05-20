#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "config.h"

// Set to broadcast to all ESP-Now-capable chips
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t clientMacAddress[6];

// Add these variables at the top with your other globals
unsigned long sendAttemptTime = 0;
const int SEND_TIMEOUT_MS = 10000;
// GPIO pin for forced pairing
#define PAIRING_BUTTON_PIN 0 // GPIO0 for pairing button
#define MAX_CHANNEL 13

// Deep Sleep constants 
#define uS_TO_S_FACTOR 1000000ULL

// Define three possible message types 
enum MessageType {PAIRING, DATA, CTRL};
MessageType messageType;

// Structured message
typedef struct sensorData {
    uint8_t msgType;
    char nodeID[8];
    float temp;
    float humidity;
    long moisture;
} sensorData;

// New structure for pairing
typedef struct structPairing {       
    uint8_t msgType;
    char nodeID[8];
    uint8_t macAddr[6];
    uint8_t channel;
} structPairing;

// Preferences for storing paired MAC and channel
Preferences preferences;

long anaValue = 0;
long anaPercentage = 0;

// Flag for sensor data sent
bool isSensorDataSent = false;

bool validValue = true;
// Instance of sensor data
sensorData myData;
// Instance of data to pair with the Hub
structPairing pairingData;

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;

DHT dht(DHT_PIN, DHT_TYPE);

esp_now_peer_info_t peerInfo;

int channel = 1;
bool isPaired = false;

unsigned long currentMillis = millis();
// Stores last time temperature was published 
unsigned long previousMillis = 0; 
// Used to measure Pairing time     
unsigned long start;                
unsigned int readingId = 0;   

void readGetMacAddress() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK) {
      Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                    baseMac[0], baseMac[1], baseMac[2],
                    baseMac[3], baseMac[4], baseMac[5]);
    } 
    else {
      Serial.println("Failed to read MAC address");
    }

    for (int i = 0; i < 6; i++) {
        clientMacAddress[i] = baseMac[i];
    }
}

void printMAC(const uint8_t * mac_addr) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
}

void addPeer(const uint8_t * mac_addr, uint8_t chan) {
    esp_now_peer_info_t peer;
    ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
    esp_now_del_peer(mac_addr);
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    peer.channel = chan;
    peer.encrypt = false;
    memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
    if (esp_now_add_peer(&peer) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    memcpy(broadcastAddress, mac_addr, sizeof(uint8_t[6]));
}

// Save paired MAC to flash
void savePairedMACToFlash(const uint8_t * mac_addr, uint8_t chan) {
    preferences.begin("espnow", false);
    preferences.putBytes("hubMac", mac_addr, 6);
    preferences.putUChar("channel", chan);
    preferences.putBool("paired", true);
    preferences.end();
    Serial.println("Hub MAC and channel saved to flash");
}

// Load paired MAC from flash if available
bool loadPairedMACFromFlash() {
    preferences.begin("espnow", false);
    bool paired = preferences.getBool("paired", false);
    
    if (paired) {
        uint8_t savedMac[6];
        uint8_t savedChannel;
        
        if (preferences.getBytes("hubMac", savedMac, 6) == 6) {
            savedChannel = preferences.getUChar("channel", 1);
            
            Serial.print("Found saved hub MAC: ");
            printMAC(savedMac);
            Serial.print(" on channel: ");
            Serial.println(savedChannel);
            
            // Use the stored MAC and channel
            memcpy(broadcastAddress, savedMac, sizeof(uint8_t[6]));
            channel = savedChannel;
            
            // Set up connection with saved peer
            addPeer(broadcastAddress, channel);
            
            preferences.end();
            return true;
        }
    }
    
    preferences.end();
    return false;
}
// Clear paired MAC from flash
void clearPairedMAC() {
    preferences.begin("espnow", false);
    preferences.clear();
    preferences.end();
    Serial.println("Cleared saved hub MAC");
}

// Handle incoming data
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
    uint8_t type = incomingData[0];
    if (type==PAIRING) {
        memcpy(&pairingData, incomingData, sizeof(pairingData));
        // Make sure it comes from a Hub, whose ID's initial is H
        if (pairingData.nodeID[0] == 'H') {
            Serial.print("Pairing done for MAC Address: ");
            printMAC(pairingData.macAddr);
            Serial.print(" on channel " );
            Serial.print(pairingData.channel);  
            addPeer(pairingData.macAddr, pairingData.channel);
            // TODO: If u want to save the channel, do it here
            // Save MAC and channel to flash
            savePairedMACToFlash(pairingData.macAddr, pairingData.channel);
            // Update status
            pairingStatus = PAIR_PAIRED;
        }
    }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\nLast Packet Send Status: ");
    Serial.println( status == ESP_NOW_SEND_SUCCESS  ? "Delivery Success" : "Delivery Fail");
    // Sleep activation must be here, otherwise data would not be sent
    if (status == ESP_NOW_SEND_SUCCESS && isSensorDataSent) {
        
        esp_deep_sleep_start();
    }
}

void readSoilMoisture() {
	anaValue = analogRead(MOISTURE_PIN);
	if (anaValue > AIR_VALUE) {
        anaPercentage = 0;
    }
    else if (anaValue < WATER_VALUE) {
        anaPercentage = 100;
    }
    else {
        // Note: We swap AIR_VALUE and WATER_VALUE in map() because:
        // - Higher analog reading = drier soil = lower percentage
        // - Lower analog reading = wetter soil = higher percentage
        anaPercentage = map(anaValue, AIR_VALUE, WATER_VALUE, 0, 100);
    }
}

void readSensorsData() {
    validValue = true;
    dht.begin();

    // Set values to send
    strcpy(myData.nodeID, NODE_ID);
    myData.msgType = DATA;
    myData.temp = dht.readTemperature();
    myData.humidity = dht.readHumidity();
    readSoilMoisture();
    myData.moisture = anaPercentage;

    if (isnan(myData.temp)) {
        myData.temp = DEFAULT_MEASUREMENT;
        Serial.println("Failed to read temp data");
        validValue = false;
    }
    else {
        Serial.print("\nTemp Value: ");
        Serial.print(myData.temp);
        Serial.println(" C");
    }

    if (isnan(myData.humidity)) {
        myData.humidity = DEFAULT_MEASUREMENT;
        Serial.println("Failed to read humidity data");
        validValue = false;
    }
    else {
        Serial.print("\nHumidity Value: ");
        Serial.print(myData.humidity);
        Serial.println(" %");
    }

    //TODO: Add some checks for Moisture 
    Serial.print("Moisture value: ");
    Serial.println(anaValue);
    Serial.print("Moisture percentage: ");
    Serial.print(anaPercentage);
    Serial.println("%");
    
}

PairingStatus autoPairing() {
    switch(pairingStatus) {
        case PAIR_REQUEST:
            Serial.print("Pairing request on channel "  );
            Serial.println(channel);
  
            // set WiFi channel   
            ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
            if (esp_now_init() != ESP_OK) {
                Serial.println("Error initializing ESP-NOW");
            }
  
            // set callback routines
            esp_now_register_send_cb(OnDataSent);
            esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
      
            // set pairing data to send to the server
            pairingData.msgType = PAIRING;
            strcpy(pairingData.nodeID, NODE_ID);   
            pairingData.channel = channel;
            for (int i = 0; i < 6; i++) {
                pairingData.macAddr[i] = clientMacAddress[i];
            }
  
            // add peer and send request
            addPeer(broadcastAddress, channel);
            esp_now_send(broadcastAddress, (uint8_t *) &pairingData, sizeof(pairingData));
            previousMillis = millis();
            pairingStatus = PAIR_REQUESTED;
            break;
  
        case PAIR_REQUESTED:
            // time out to allow receiving response from server
            currentMillis = millis();
            if(currentMillis - previousMillis > 1000) {
                previousMillis = currentMillis;
                // time out expired,  try next channel
                channel ++;
                if (channel > MAX_CHANNEL){
                    channel = 1;
                }   
                pairingStatus = PAIR_REQUEST;
            }
            break;
    
        case PAIR_PAIRED:
            // nothing to do here 
            break;
    }
    return pairingStatus;
}  
 
void setup() {
    // Init Serial Monitor
    Serial.begin(115200);

    // Configure pairing button with internal pullup
    pinMode(PAIRING_BUTTON_PIN, INPUT_PULLUP);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    Serial.print("Sensor node MAC Address:  ");
    readGetMacAddress();
    // ESP_NOW does not need WiFi beyond initial calibration
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    strcpy(pairingData.nodeID, NODE_ID);

    // Set deep sleep timer
    esp_sleep_enable_timer_wakeup(SLEEP_TIMER * uS_TO_S_FACTOR);
    isSensorDataSent = false;

    // Check if button is pressed for forced pairing
    bool forceNewPairing = (digitalRead(PAIRING_BUTTON_PIN) == LOW);
    
    if (forceNewPairing) {
        Serial.println("Pairing button pressed - forcing new pairing");
        clearPairedMAC();
        pairingStatus = PAIR_REQUEST;
    } else {
        // Try to load stored MAC
        isPaired = loadPairedMACFromFlash();
        
        if (isPaired) {
            pairingStatus = PAIR_PAIRED;
            Serial.println("Using stored hub MAC address");
        } else {
            Serial.println("No stored hub MAC, starting auto pairing");
            pairingStatus = PAIR_REQUEST;
        }
    }

    // Only do auto pairing if needed
    if (pairingStatus != PAIR_PAIRED) {
        Serial.println("Starting auto pairing process...");
        while (autoPairing() != PAIR_PAIRED) {
            delay(10); // Small delay to prevent watchdog trigger
        }
    }

    // After pairing is complete, send the sensor data
    readSensorsData();

    // Send message via ESP-NOW using the paired address
    Serial.println("Sending data to paired hub:");
    Serial.print("MAC: ");
    printMAC(broadcastAddress);
    Serial.print(" on channel: ");
    Serial.println(channel);
    
    if (!validValue) {
        Serial.println("Invalid sensor data, not sending");
        esp_deep_sleep_start();
    }
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
        Serial.println("Sent initial data successfully");
        isSensorDataSent = true;
    }
    else {
        Serial.println("Error sending the data will sleep anyway after timeout" );
        sendAttemptTime = millis();
    }
    
    // Give some time for the callback to trigger and send us to sleep
    delay(100);
}
 
void loop() {
    // Check if pairing button is pressed during wait
    if (digitalRead(PAIRING_BUTTON_PIN) == LOW) {
        Serial.println("Pairing button pressed during wait - forcing new pairing");
        clearPairedMAC();
        pairingStatus = PAIR_REQUEST;
        
        // Start the auto pairing process
        while (autoPairing() != PAIR_PAIRED) {
            delay(10); // Small delay to prevent watchdog trigger
        }
        
        // After pairing is complete, send the sensor data again
        readSensorsData();
        
        // Send message via ESP-NOW using the newly paired address
        Serial.println("Sending data to new paired hub:");
        Serial.print("MAC: ");
        printMAC(broadcastAddress);
        Serial.print(" on channel: ");
        Serial.println(channel);
        
        if (!validValue) {
        Serial.println("Invalid sensor data, not sending");
        esp_deep_sleep_start();
    }
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        
        if (result == ESP_OK) {
            Serial.println("Sent data successfully after re-pairing");
            isSensorDataSent = true;
        }
        else {
            Serial.println("Error sending the data after re-pairing");
        }
        
        // Reset timeout counter
        sendAttemptTime = millis();
    }
    
    // If we've been awake too long and sending failed or timed out, go to sleep anyway
    if (millis() - sendAttemptTime > SEND_TIMEOUT_MS) {
        Serial.println("Send timeout reached, going to sleep anyway");
        esp_deep_sleep_start();
    }
    
    delay(100); // Small delay to prevent watchdog trigger and allow ESP-NOW callback to work
}