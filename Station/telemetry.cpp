#include "telemetry.hpp"




esp_now_peer_info_t broadInfo;
STAT_Data   db_XY[STAT_N];
DRONE_Data  db_XX[DRONE_N];

uint8_t stat_N = 0;
uint8_t drone_N = 0;






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
  if(rcv_data[0] == 'X'){
      uint8_t device_index;
      if(rcv_data[1] == 'X'){ // Drone

          //If the address isn't known checks for pairing message

          //else gets the index;

          if(!knownMAC_XX(rcv_Info,&device_index,db_XX)){
            Serial.println("I don't know this drone:");
            //if it has peer message, registers addr in database.
            if(memcmp(rcv_data+3,PEER_XX,4) == 0) addXX(rcv_Info->src_addr,&drone_N,&(db_XX[drone_N]));

            //if not, ignores.
          }else{

            //If it is known, reads the message:
            // - if ping request -- pings back
            // - if comm request -- tackles package


            Serial.println("I know this drone:");
          }

      }else if(rcv_data[1] == 'Y'){ // Station

          //If the address isn't known checks for pairing message

          //else gets the index;
          if(!knownMAC_XY(rcv_Info,&device_index,db_XY)){
            Serial.println("I don't know this stat");
            Serial.printf("Recieved data: === %c%c - %hhu - %02x:%02x:%02x:%02x - %02x:%02x:%02x:%02x:%02x:%02x - %c%c === \r\n"
              ,rcv_data[0],rcv_data[1]
              ,rcv_data[2]
              ,rcv_data[3],rcv_data[4],rcv_data[5],rcv_data[6]
              ,rcv_data[7],rcv_data[8],rcv_data[9],rcv_data[10],rcv_data[11],rcv_data[12]
              ,rcv_data[13],rcv_data[14]
            );

            if(memcmp(rcv_data+3,PEER_XY,4) == 0) addXY(rcv_Info->src_addr,&stat_N,&(db_XY[stat_N]));
            //if it has peer message, registers addr in database.
            //if not, ignores.


          }else{
            //If it is known, reads the message:
            // - if ping request -- pings back
            // - if comm request -- tackles package


            Serial.println("I know this stat:");
          }


      }

  } // else, ignores the message


}

void OnDataSend(const esp_now_send_info_t *tx_info , esp_now_send_status_t status)
{
  Serial.println("msg sent");
}
