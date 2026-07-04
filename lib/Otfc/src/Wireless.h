#pragma once

#include "Model.h"
#include "Device/SerialDeviceAdapter.h"

#ifdef OTFC_SERIAL_SOFT_0_WIFI
#if defined(OTFC_WIFI_ALT)
#include <ESP8266WiFi.h>
#elif defined(OTFC_WIFI)
#include <WiFi.h>
#endif

namespace Otfc {

class Wireless
{
  enum Status {
    STOPPED,
    STARTED,
  };
  public:
    Wireless(Model& model);

    int begin();
    int update();

    void startAp();
    int connect();
    void wifiEventConnected(const String& ssid, int channel);
    void wifiEventApConnected(const uint8_t* mac);
    void wifiEventGotIp(const IPAddress& ip);
    void wifiEventDisconnected();

  private:
    Model& _model;
    Status _status;
    WiFiServer _server;
    WiFiClient _client;
    Device::SerialDeviceAdapter<WiFiClient> _adapter;
#ifdef OTFC_WIFI_ALT
    WiFiEventHandler _events[4];
#endif
};

}

#endif
