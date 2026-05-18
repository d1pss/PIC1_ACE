#include "init.hpp"


/*
void initSendPeerPackage(const uint8_t *my_addr,esp_now_peer_info_t *my_broadInfo)
{
    uint8_t data[11]; //[0: Channel][1-6: Address of the station][7-10: Peer code]
    data[0] = CHANNEL;
    memcpy(&data[1], (uint8_t *)my_addr, 6);
    memcpy(&data[1 + 6], (uint8_t *)PEER_XY, 4); // #define PEER_XY   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7} "ACE Stat"
    esp_now_send(my_broadInfo->peer_addr, data, 11);
}
*/

void initRoutine1(){
  Serial.print(" ======== Routine 1 ======== \r\n");

  if(!WiFi.mode(WIFI_MODE_STA)) 
  {
    Serial.print("Error: initStat - wifi mode\r\n");
    ESP.restart();
  }

  delay(10);

  if ( esp_now_init() != ESP_OK )
  {
    Serial.println("Error: initStat - esp_init\r\n");
    ESP.restart();
  }
}


void initBroadcast(esp_now_peer_info_t *broadInfo, const uint8_t *my_addr, esp_now_recv_cb_t OnDataRecv, esp_now_send_cb_t OnDataSend){

    Serial.print(F(" ======== Init Broadcast ======== \r\n"));
    //memset((void*)&peerInfo,0,sizeof(peerInfo));

    memset((void*)broadInfo,0,sizeof(broadInfo));
    // Primary network configuration ---> The station will broadcast a signal in their initiation.
    memcpy(broadInfo->peer_addr, BROAD_MAC, 6);
    broadInfo->channel = CHANNEL;
    broadInfo->encrypt = false; // Non encrypted

    if (esp_now_add_peer(broadInfo) != ESP_OK) {
        Serial.println("Error: initBroadcast - esp_now_add_peer");
        ESP.restart();
    } 

    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    // Channel Bandwidth is HT20: Primary choosen, Secondary Ignored
    Serial.printf("\r\n");



    if ( esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)) != ESP_OK )
    {
      Serial.println("Error: initStat - RECV_CB\r\n");
      ESP.restart();
    }

    if ( esp_now_register_send_cb(esp_now_send_cb_t(OnDataSend)) != ESP_OK )
    {
      Serial.println("Error: initStat - RECV_CB\r\n");
      ESP.restart();
    }

    // Sends peer info in a broadcast mode (every device detects it) - Sends 10 requests
    Serial.println(" Now Broadcasting:");
    sendPeerBroadcast((const uint8_t *)my_addr, broadInfo);
    Serial.printf("\r\n");
}

esp_err_t sendPing(const uint8_t *my_addr,esp_now_peer_info_t *my_peerInfo)
{
    uint8_t data[11]; //[0: Channel][1-6: Address of the station][7-10: Peer code]
    uint8_t idx = 0;
    data[idx++] = (uint8_t)'X';
    data[idx++] = (uint8_t)'Y';
    memcpy(&data[idx++], (uint8_t *)PING_XY, 4); // #define PEER_XY   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7} "ACE Stat"
    idx = idx+4;
    data[idx++] = (uint8_t)'X';
    data[idx++] = (uint8_t)'Y';
    return esp_now_send(my_peerInfo->peer_addr, data, idx);
}

void sendPeerBroadcast(const uint8_t *my_addr,esp_now_peer_info_t *my_broadInfo)
{
    delay(10*(rand()%100));
    uint8_t data[128]; //[0-1: XY][2: ping counter]
    uint8_t idx = 0;
    data[idx++] = (uint8_t)'X'; 
    data[idx++] = (uint8_t)'Y';
    data[idx++] = (uint8_t) CHANNEL;
    memcpy(&data[idx], (uint8_t *)PEER_XY, 4);
    idx = idx+4;
    memcpy(&data[idx], (uint8_t *)my_addr, 6);
    idx = idx+6;
    data[idx++] = (uint8_t)'X';
    data[idx++] = (uint8_t)'Y';

    for(uint8_t i = 1;i <= 10;i++) {
        Serial.printf("%d ", i);
        esp_now_send(my_broadInfo->peer_addr, data, idx);
        delay(50);

    }
}
