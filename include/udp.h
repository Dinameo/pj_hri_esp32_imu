#ifndef UDP_H
#define UDP_H

#include <WiFi.h>
#include <WiFiUdp.h>

void ConnectWiFi();
void SendUDPData(const char* data);

#endif