#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "communications.hpp"

namespace Communications {

uint8_t broadcastAddress[] = {0x70, 0x4B, 0xCA, 0x69, 0x73, 0xF4};

//struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.println("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                  : "Delivery Fail");
}

bool init(void) {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }

    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return false;
    }
    return true;
}

bool send_message(struct_message myData) {
    esp_err_t result =
        esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
	return true;
    } else {
        Serial.println("Error sending the data");
	return false;
    }
}

}  // namespace Communications
