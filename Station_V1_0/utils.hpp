#ifndef UTILS_HPP
#define UTILS_HPP

#include "esp_now.h"

#include "headlib.hpp"
#include "telemetry.hpp"
#include "init.hpp"




bool knownMAC_XY(const esp_now_recv_info_t *peerInfo, uint8_t *idx, const STAT_Data *db_XY);
bool knownMAC_XX(const esp_now_recv_info_t *peerInfo, uint8_t *idx, const DRONE_Data *db_XX);
bool addXY(const uint8_t *new_addr, uint8_t *stat_N, STAT_Data *db_aux);
bool addXX(const uint8_t *new_addr, uint8_t *drone_N, DRONE_Data *db_aux);

void update_MyXXDatabase(DRONE_Data *db_XX,const uint8_t *rcv_data, bool isOnline);
void update_MySTADatabase(STAT_Data *db_STA,const uint8_t *rcv_data, bool isOnline);

#endif