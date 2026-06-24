#include "udp.h"
#include "cfg.h"
#include <WiFi.h>
#include <WiFiUdp.h>

IPAddress remoteIP(192,168,1,211);
const int unityPort = 5005;
WiFiUDP wifiUdp;

void ConnectWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
}
void SendUDPData(const char* data)
{
    wifiUdp.beginPacket(remoteIP, unityPort);
    wifiUdp.print(data);
    wifiUdp.endPacket();
}