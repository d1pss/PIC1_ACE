#ifndef PROMPTS_HPP
#define PROMPTS_HPP

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <esp_now.h>
#include <WiFi.h>

#include "headlib.hpp"
#include "telemetry.hpp"



//// Communication test orders:

// Prompt by DEVICE
#define NO_COMM     '0' // No command
#define STAT_1      '1' // Status da rede lv1
#define STAT_2      '2' // Status da rede lv2
#define START_MON   '3' // Prompt da resposta da rede
#define END_MON     '4'
#define MY_MAC      '5'
#define LEAVE       '6'
#define REBOOT      '7'


// Prompt by NETWORK (Maybe later...)
#define FULL_NOTIF  '8'
#define NOTIF_R     '9'     // Sem missão/ordem (muda ranking)
#define NOTIF_M     'A'    // Sem ranking (muda missão/ordem)
#define SET_AVAL    'B'    // Muda disponibilidade (Y/N)
#define SENDER 'C'    // 0 - no filter; 1 - DRONE ONLY; 2 - STAT ONLY;
#define MODE 'D'    // 0 - No format (HEX); 1 - mode 1; 2 - mode 2;
//#define FORGET_DEV  'E'    // Forgets the existence of the drone

// Help Command
#define HELP        'F'


// Command line stuff:
#define CMD_BUFF 1024


// Serial monitor
void promptHelp();
void promptXY(STAT_Data data); // Station prompt
void promptXX(DRONE_Data data); // Drone prompt
void promptMyMAC(const uint8_t *addr);
void promptMyDATA();

void cmdCheckUP(const uint8_t *my_addr);
void cmdProcess(char* cmd, const uint8_t *my_addr, bool *monitoring);

void unpackHEX(const uint8_t *author_mac,const uint8_t *rcv_data);
void unpackCHAR(const uint8_t *author_mac,const uint8_t *rcv_data);
void unpackF1(const uint8_t *author_mac,const uint8_t *rcv_data);
void unpackF2(const uint8_t *author_mac,const uint8_t *rcv_data);
void unpackF3(const uint8_t *author_mac,const uint8_t *rcv_data);
void unpackF4(const uint8_t *author_mac,const uint8_t *rcv_data);


extern STAT_Data db_STA[STAT_N];

#endif
