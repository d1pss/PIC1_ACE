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
    db_aux->online = true;
    db_aux->available = false; 
    db_aux->mission = 0; 
    db_aux->rank = 0; 
    db_aux->fly_bat_voltage = 0.0f; 
    db_aux->altitude = 0.0f;
    db_aux->tof_front = 0;
    *drone_N++;

    //confirm_peer();
    return 1;
  }

  Serial.println("Connection refused: Too many peers");
  return 0;
}


void update_MyXXDatabase(DRONE_Data *db_XX,const uint8_t *rcv_data, bool isOnline)
{

// --- STATION DATA SECTION ---
    //[XY]:             0-1
    //[Channel]:        2
    //[0x00]:           3
    //[Available]:      4
    //[N_neigh]:        5
    //[N_rand]:         6
    //[Stat ADDR]:      7-12    <-- Station sddress
    //[0x00]:           13
    //[0x00]:           14

// --- DRONE DATA SECTION ---
    //[XX]:             15-16   
    //[0x00]:           17
    //[0x00]:           18
    //[Available]:      19      
    //[rank]:           20
    //[mission]:        21
    //[land_request]:   22
    //[battery]:        23-26
    //[altitude]:       27-30
    //[tof]:            31-32
    //[Drone ADDR]:     33-38   <-- Drone address
    //[0x00]:           39
    //[0x00]:           40

    // = 41 bytes total



    //OLD PACKAGE
    //[XX]:             0-1
    //[Channel]:           2
    //[PING_KEY]:           3-6
    //[Available]:      7
    //[rank]:           8
    //[mission]:        9
    //[land_request]:   10
    //[battery]:        11-14
    //[altitude]:       15-18
    //[tof]:            19-20

    //[PHI]:            21-24
    //[THETA]:          25-28
    //[PSI]:            29-32
    //[XX]:             33-34



    //GENERIC PACKAGE
    //[XX]:             0-1
    //[MODE]:           2

    //[FLOAT_P1]:
    //[FLOAT_P2]:
    //[FLOAT_P3]:

    //[FLOAT_P4]:
    //[FLOAT_P5]:
    //[FLOAT_P6]:

    //[FLOAT_P7]:

    //[FLOAT_P8]:

    //[FLOAT_P9]:
    //[XX]:  

    // =
  
  db_XX->channel          = rcv_data[2];
  db_XX->online           = isOnline;
  db_XX->available        = rcv_data[7];
  db_XX->mission          = rcv_data[8];
  db_XX->rank             = rcv_data[9];
  db_XX->altitude         = *((float *)&(rcv_data[10]));
  db_XX->fly_bat_voltage  = *((float *)&(rcv_data[14]));
  db_XX->tof_front        = *((uint16_t *)&(rcv_data[18]));
}

void update_MySTADatabase(STAT_Data *db_STA,const uint8_t *rcv_data, bool isOnline)
{
  db_STA->channel = rcv_data[2];
  db_STA->neighN  = 0;
  db_STA->online  = isOnline;
}




