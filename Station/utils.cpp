#include "esp_now.h"
#include "utils.hpp"


bool knownMAC_XX(const esp_now_recv_info_t *peerInfo, uint8_t *idx, const DRONE_Data *db_XX)
{
  const uint8_t *aux = peerInfo->src_addr;
  for(uint8_t i = 0; i < DRONE_N;i++)
    if(memcmp(aux,db_XX[i].peerInfo.peer_addr,MAC_SZ) == 0){
      *idx = i;
      return 1;
    }

  idx = NULL;
  return 0;
}



bool knownMAC_XY(const esp_now_recv_info_t *peerInfo, uint8_t *idx, const STAT_Data *db_XY)
{
  const uint8_t *aux = peerInfo->src_addr;
  for(uint8_t i = 0; i < STAT_N;i++) 
    if(memcmp(aux,db_XY[i].peerInfo.peer_addr,STAT_N) == 0){
      *idx = i;
      return 1;
    }

  idx = NULL;
  return 0;
}



bool addXY(const uint8_t *new_addr, uint8_t *stat_N, STAT_Data *db_aux)
{
  if(*stat_N < STAT_N ){
    peering_stat(new_addr, &(db_aux->peerInfo));
    db_aux->online = true;
    db_aux->neighN = 0; // Para saber que está inicializado

    *stat_N++;
    return 1;
  }

  Serial.println("Connection refused: Too many peers");
  return 0;
}

bool addXX(const uint8_t *new_addr, uint8_t *drone_N, DRONE_Data *db_aux)
{
  if(*drone_N < DRONE_N){
    peering_stat(new_addr, &(db_aux->peerInfo));
    db_aux->connected = true;
    db_aux->available = false; 
    db_aux->mission = 0; 
    db_aux->rank = 0; 
    db_aux->fly_bat_voltage = 0.0f; 
    db_aux->altitude = 0.0f;
    db_aux->tof_front = 0;
    *drone_N++;
    return 1;
  }

  Serial.println("Connection refused: Too many peers");
  return 0;
}
