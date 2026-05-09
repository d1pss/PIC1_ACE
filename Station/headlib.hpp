#ifndef HEADLIB_HPP
#define HEADLIB_HPP


#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"



//#include "prompt.hpp"
//#include "cmd.hpp"


#define NETWORK_ENCRYPT ""

#define BROAD_MAC (uint8_t[]) { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
#define PEER_XY   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7}
#define PING_XY   (uint8_t []){0x77, 99, 0x02, 0x09}

#define PEER_XX   (uint8_t []){0xAC, 0xE0, 0x57, 0xA7}
#define PING_XX   (uint8_t []){0x77, 99, 0x02, 0x09}

#define CHANNEL   3

#define DRONE_N 3
#define STAT_N 3
#define MAC_SZ 6


//e0:8c:fe:5d:e5:80
//8c:4f:00:28:06:24


typedef struct{
    esp_now_peer_info_t peerInfo;

    bool    online;
    uint8_t neighN;
} STAT_Data;



typedef struct{
    esp_now_peer_info_t peerInfo;

    bool    connected;
    bool    available;
    uint8_t rank;
    uint8_t mission;

    float   fly_bat_voltage;
    float   altitude;
    int16_t tof_front;
} DRONE_Data;






#endif
