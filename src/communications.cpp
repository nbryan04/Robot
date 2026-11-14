#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "communications.hpp"

//=========================
// camera definitions
//========================

#define ESP32_S3_CAM

#if defined(AI_THINKER_1)
uint8_t broadcastAddress[] = {0x70, 0x4B, 0xCA, 0x69, 0x73, 0xF4};
#elif defined(AI_THINKER_2)
uint8_t broadcastAddress2[] = {0xD4, 0xE9, 0xF4, 0xEE, 0xC6, 0x5C};
#elif defined(ESP32_S3_CAM)
uint8_t broadcastAddress3[] = {0x28, 0x84, 0x85, 0x8C, 0xF6, 0x38};
#endif

namespace Communications {

detectionResult dataReceived;
bool newMessage = false;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.println("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                  : "Delivery Fail");
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&dataReceived, incomingData, sizeof(dataReceived));
    newMessage = true;
}

bool init(void) {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    newMessage = false;
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }

    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return false;
    }
    return true;
}

bool send_message(cameraTrigger data) {
    esp_err_t result =
        esp_now_send(broadcastAddress3, (uint8_t*)&data, sizeof(data));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
        return true;
    } else {
        Serial.println("Error sending the data");
        return false;
    }
}

bool hasNewMessage() { return newMessage; }

void resetHasNewMessage() { newMessage = false; }

bool teletubbyFound() { return dataReceived.teletubbyFound; }

int teletubbyCount() { return dataReceived.teletubbyCount; }

}  // namespace Communications
