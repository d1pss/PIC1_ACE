#ifndef INIT_HPP
#define INIT_HPP

#include "headlib.hpp"
#include "telemetry.hpp"


void initRoutine1();
void initBroadcast(esp_now_peer_info_t *broadInfo, const uint8_t *my_addr, esp_now_recv_cb_t OnDataRecv, esp_now_send_cb_t OnDataSend);
void initPeerRequest(const uint8_t *my_addr,esp_now_peer_info_t *my_broadInfo);

esp_err_t sendPing(const uint8_t *my_addr,esp_now_peer_info_t *my_broadInfo);
void sendPeerBroadcast(const uint8_t *my_addr,esp_now_peer_info_t *my_broadInfo);

#endif
