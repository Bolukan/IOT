/*
 *   Efficient WiFi
 *   https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-2/
 *
 */

/* WiFi settings */
#include "../secrets.h"
#include <WiFi>

String MACdefaultAddress(void) {
    uint8_t mac[6];
    char macStr[18] = { 0 };
    esp_err_t error;
    error = esp_efuse_mac_get_default(mac);
    if (error==ESP_OK)
    {
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    return String(macStr);
}

void setup(void)
{
  // WiFi
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin(); // only works if WIFI_OFF
  delay(1);

  // Serial
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("Booted");
  Serial.println("Status: WiFi disabled");
}

void loop() {
  // DO STUFF

  // Wake WiFi
  WiFi.forceSleepWake();
  delay(1);
  WiFi.persistent(false); // use WiFi.config()
  WiFi.mode(WIFI_STA);
  WiFi.config(WLAN_staticIP, WLAN_gateway, WLAN_subnet, WLAN_dns1, WLAN_dns2);
  WiFi.begin(WLAN_SSID, WLAN_PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
     delay(1);
  }
  Serial.println("WiFi connected");

  // SEND STUFF

  // Prepare for deep sleep
  WiFi.disconnect(true);
  delay(1);
  WiFi.forceSleepBegin();
  delay(1);

  // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
  ESP.deepSleep( 60000000, WAKE_RF_DISABLED ); // 1 minute
}
