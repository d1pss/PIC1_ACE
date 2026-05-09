#include "headlib.hpp"
#include "telemetry.hpp"
#include "init.hpp"




esp_now_peer_info_t aux_peerInfo;

uint8_t my_addr[6] = {0};
//uint8_t aux_addr[6] = {0};


void setup(){
  Serial.begin(115200);
  Serial.print(" ======== Setup ======== \r\n");
  setup_stat((uint8_t *)&my_addr);

}


void loop(){
  sendPing((const uint8_t *)my_addr,&aux_peerInfo);
  //delay(50);
  //cmdCheckUP();

  delay(5000); // 5 segundos
}



/*
esp_now_peer_info_t broadInfo;
esp_now_peer_info_t peerInfo;
uint8_t ctr = 0;
void initSendPeerPackage(const uint8_t *my_addr,esp_now_peer_info_t *my_broadInfo)
{
    uint8_t data[11]; //[0: Channel][1-6: Address of the station][7-10: Peer code]
    data[0] = CHANNEL;
    memcpy(&data[1], (uint8_t *)my_addr, 6);
    memcpy(&data[1 + 6], (uint8_t *)PEER_XY, 4); // #define PEER_XY   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7} "ACE Stat"
    esp_now_send(my_broadInfo->peer_addr, data, 11);
}
void pingPackage(esp_now_peer_info_t *my_peerInfo, const uint8_t N_to_send)
{
    uint8_t data[3]; //[0-1: XY][2: ping counter]
    data[0] = (uint8_t)'X';
    data[1] = (uint8_t)'Y';
    data[2] = N_to_send;
    esp_now_send(my_peerInfo->peer_addr, data, 3);
}
void send_ping(esp_now_peer_info_t *my_peerInfo, uint8_t *my_ctr){
  if (!(*my_ctr < 256)) *my_ctr = 0;
  pingPackage(my_peerInfo,*my_ctr);
}
void peering_stat(const uint8_t *newPeerAddr, esp_now_peer_info_t *my_peerInfo){
  memcpy(my_peerInfo->peer_addr,newPeerAddr,6);
  my_peerInfo->channel = CHANNEL;
  my_peerInfo->encrypt = false;
  if(esp_now_add_peer(my_peerInfo) != ESP_OK) Serial.println(" === PEERING FAILURE === ");
  else Serial.println(" === PEERING SUCCESS === ");
}
void OnDataRecv(const esp_now_recv_info_t *my_recvInfo, const uint8_t *data, int data_len){
  Serial.println(" === LISTENING === ");
  uint8_t *aux_ptr = peerInfo.peer_addr;
  uint8_t *aux_ptr2 = my_recvInfo->src_addr;
  if(!aux_ptr[0] && !aux_ptr[1] && !aux_ptr[2] && !aux_ptr[3] && !aux_ptr[4] && !aux_ptr[5]){
    Serial.println(" === ADDRESS DETECTED === ");
    Serial.printf("Addr from my_peerInfo: %02x:%02x:%02x:%02x:%02x:%02x\n",aux_ptr[0],aux_ptr[1],aux_ptr[2],aux_ptr[3],aux_ptr[4],aux_ptr[5]);
    Serial.printf("Addr from my_recvInfo: %02x:%02x:%02x:%02x:%02x:%02x\n",aux_ptr2[0],aux_ptr2[1],aux_ptr2[2],aux_ptr2[3],aux_ptr2[4],aux_ptr2[5]);
    uint8_t channel = data[0];
    char addr_str[19];
    snprintf(addr_str,18,"%02x:%02x:%02x:%02x:%02x:%02x",data[1],data[2],data[3],data[4],data[5],data[6]);
    char key[13];
    snprintf(key,(4+4+4),"%02x:%02x:%02x:%02x",data[7],data[8],data[9],data[10]);
    Serial.printf("Recieved data:\n\tChannel: %d\n\tRecieved address: %s\n\tPeering Key: %s\n\tData length: %d\t\n",channel,addr_str,key,data_len);
    aux_ptr[0] = aux_ptr2[0];
    aux_ptr[1] = aux_ptr2[1];
    aux_ptr[2] = aux_ptr2[2];
    aux_ptr[3] = aux_ptr2[3];
    aux_ptr[4] = aux_ptr2[4];
    aux_ptr[5] = aux_ptr2[5];
    
    Serial.println(" === I WANT TO CONNECT === ");
    
    peering_stat(aux_ptr2,&peerInfo);
    
  }else if(memcmp(aux_ptr2,aux_ptr,6) == 0){
    if(data[0] == 88 && data[1] == 89){
      char mac_str[19];
      snprintf(mac_str,18,"%02x:%02x:%02x:%02x:%02x:%02x",aux_ptr2[0],aux_ptr2[1],aux_ptr2[2],aux_ptr2[3],aux_ptr2[4],aux_ptr2[5]);
      Serial.printf("Recieved PING:");
      Serial.printf("\t%c%c (%s) -> %d\n",(char)data[0],(char)data[1],mac_str,data[2]);
      ctr = data[2] + 1;
    }
  }
}

void setup() {
    Serial.begin(115200);
    // Inicializa o WIFI
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
    memset((void*)&peerInfo,0,sizeof(peerInfo));
    memset((void*)&broadInfo,0,sizeof(broadInfo));
    Serial.print("initBroadcast\n");
    // Primary network configuration ---> The station will broadcast a signal in their initiation.
    memcpy(broadInfo.peer_addr, BROAD_MAC, 6);
    broadInfo.channel = CHANNEL;
    broadInfo.encrypt = false; // Non encrypted
    if (esp_now_add_peer(&broadInfo) != ESP_OK) {
        Serial.println("Error: initBroadcast - esp_now_add_peer");
        return;
    }
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    // Channel Bandwidth is HT20: Primary choosen, Secondary Ignored
    Serial.printf("\r\n");
    esp_wifi_get_mac(WIFI_IF_STA, (uint8_t *)&my_addr );
    Serial.printf("my_addr ---> %02x:%02x:%02x:%02x:%02x:%02x\n",my_addr[0],my_addr[1],my_addr[2],my_addr[3],my_addr[4],my_addr[5]);
    
    if ( esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)) != ESP_OK )
    {
      Serial.println("Error: initStat - RECV_CB\r\n");
      ESP.restart();
    }
    // Sends peer info in a broadcast mode (every device detects it) - Sends 10 requests
    Serial.println(" Now Broadcasting:");
    for (uint16_t i = 0; i < 10; i++){
        initSendPeerPackage((const uint8_t *)&my_addr,&broadInfo);
        delay(50);
        Serial.printf("%d ", i);
    }
    Serial.printf("\r\n");
      
    //initStat(my_addr);
    //Serial.print("ACE Station: Setting Up\n\n");
    //Serial.print(" ======== ACE Station: ON-AIR ======== \n");
void loop() {
  
  //netCheckUP(&peerInfo,&ctr);
  delay(500);
  //cmdCheckUP(my_addr,monitoring);
}
*/
