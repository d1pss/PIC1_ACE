#include "HardwareSerial.h"
#include <cstdint>
#include "telemetry.hpp"
#include "prompts.hpp"
#include "utils.hpp"



#define TRESHOLD_CALLS 1000


esp_now_peer_info_t broadInfo;

STAT_Data db_STA[STAT_N];

static DRONE_Data  db_XX[DRONE_N];

uint8_t stat_N = 0;
uint8_t drone_N = 0;

int XX_ctr = 0, XY_ctr = 0;

extern uint8_t sensor;


uint8_t aux_ctr = 0;


void peering_stat(const uint8_t *newPeerAddr, esp_now_peer_info_t *my_peerInfo)
{
  memcpy(my_peerInfo->peer_addr,newPeerAddr,MAC_SZ);
  my_peerInfo->channel = CHANNEL;
  my_peerInfo->encrypt = false;
  if(esp_now_add_peer(my_peerInfo) != ESP_OK) Serial.println(" === PEERING FAILURE === ");
  else Serial.println(" === PEERING SUCCESS === ");
}

/*
void buildPingMsg(const esp_now_peer_info_t *my_peerInfo, const uint8_t N_to_send){
    uint8_t data[3]; //[0-1: XY][2: ping counter]
    uint8_t size = 0;
    data[0] = (uint8_t)'X';
    data[1] = (uint8_t)'Y';
    data[2] = (uint8_t)PING_XY[0]; size++;
    data[3] = (uint8_t)PING_XY[1]; size++;
    data[4] = (uint8_t)PING_XY[2]; size++;
    data[5] = (uint8_t)PING_XY[3]; size++;
    data[6] = (uint8_t)PING_XY[4]; size++;
    data[7] = (uint8_t)PING_XY[5]; size++;
    data[8] = (uint8_t)PING_XY[6]; size++;
    esp_now_send(my_peerInfo->peer_addr, data, size);
}
*/

void setup_stat(uint8_t *my_addr){
    Serial.print("Setting up... \r\n");

    initRoutine1();

    esp_wifi_get_mac(WIFI_IF_STA, (uint8_t *)my_addr);
    Serial.printf("my_addr ---> %02x:%02x:%02x:%02x:%02x:%02x\n",my_addr[0],my_addr[1],my_addr[2],my_addr[3],my_addr[4],my_addr[5]);

    initBroadcast(&broadInfo, my_addr,esp_now_recv_cb_t(OnDataRecv), esp_now_send_cb_t(OnDataSend));



    Serial.print("Station: Setting Up\n\n");
    Serial.print(" ======== Station: ON-AIR ======== \n");

}

void OnDataRecv(const esp_now_recv_info_t *rcv_Info, const uint8_t *rcv_data, int data_len)
{

  #if 0

  
  if(rcv_data[0] == 'X'){
      uint8_t idx;
      if(rcv_data[1] == 'X'){ // Drone

          //If the address isn't known checks for pairing message

          //else gets the index;

          ///*
            if(!knownMAC_XX(rcv_Info,&idx,db_XX)){

            //if it has peer message, registers addr in database.
            if((memcmp(rcv_data+3,PEER_XX,4) == 0) && (rcv_data[2] == CHANNEL) && (rcv_data[20] == 'X') && (rcv_data[21] == 'X'))
            {
              addXX(rcv_Info->src_addr,&drone_N,&(db_XX[drone_N]));
              Serial.printf("New peer - %02x:%02x:%02x:%02x:%02x:%02x:\r\n", rcvd_addr[0], rcvd_addr[1], rcvd_addr[2], rcvd_addr[3], rcvd_addr[4], rcvd_addr[5]);
            }

            //if not, ignores.
          }else{

            //Serial.println("I know this drone:");
            if(memcmp(rcv_data+3,PING_XX,4) == 0)
            {
              //update_MyXXDatabase(&(db_XX[idx]),rcv_data,true);
              

              ///*
              if(TRESHOLD_CALLS <= XX_ctr) XX_ctr++;
              else {
                //unpackGeneric(rcvd_addr,rcv_data);
                //Serial.println("I know this drone:");
                XX_ctr = 0;
              }
              //*/

            }
          //*/

              

          }

      }else if(rcv_data[1] == 'Y'){ // Station

          //If the address isn't known checks for pairing message

          //else gets the index;
          if(!knownMAC_XY(rcv_Info,&idx,db_STA)){
            Serial.println("I don't know this stat");
            if(memcmp(rcv_data+3,PEER_XY,4) == 0)
              {
                addXY(rcv_Info->src_addr,&stat_N,&(db_STA[stat_N]));
            
              /*
                Serial.printf("Recieved data: === %c%c - %hhu - %02x:%02x:%02x:%02x:%02x:%02x - %c%c === \r\n"
                  ,rcv_data[0],rcv_data[1]
                  ,rcv_data[2]
                  //,rcv_data[3],rcv_data[4],rcv_data[5],rcv_data[6]
                  ,rcv_data[7],rcv_data[8],rcv_data[9],rcv_data[10],rcv_data[11],rcv_data[12]
                  ,rcv_data[13],rcv_data[14]
                );
              */
              
              }
            //if it has peer message, registers addr in database.
            //if not, ignores.


          }else{

            //If it is known, reads the message:
            // - if ping request -- pings back
            // - if comm request -- tackles package


            if(memcmp(rcv_data+3,PING_XY,4) == 0)
            {
              //update_MySTADatabase(&(db_STA[idx]),rcv_data,true);
              
              if(TRESHOLD_CALLS <= XY_ctr) XY_ctr++;
              else {
                Serial.println("I know this drone:");
                XY_ctr = 0;
              }
            }
          }
      }

  } // else, ignores the message
  #else
    //aux_ctr++;
    //if(aux_ctr >= 50){
    //aux_ctr = 0;
    if(rcv_data[3] == PING_XX[0] && rcv_data[4] == PING_XX[1] && rcv_data[5] == PING_XX[2] && rcv_data[6] == PING_XX[3]) unpackF4(rcv_Info->src_addr , rcv_data);
    //}
  #endif
}

void OnDataSend(const esp_now_send_info_t *tx_info , esp_now_send_status_t status)
{
  Serial.println("msg sent");
}
