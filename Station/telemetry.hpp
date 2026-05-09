#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP



#include "headlib.hpp"
#include "utils.hpp"
#include "init.hpp"

void peering_stat(const uint8_t *newPeerAddr, esp_now_peer_info_t *my_peerInfo);
void setup_stat(uint8_t *my_addr);
void OnDataRecv(const esp_now_recv_info_t *rcvInfo , const uint8_t *rcv_data, int data_len);
void OnDataSend(const esp_now_send_info_t *tx_info , esp_now_send_status_t status);


#endif
