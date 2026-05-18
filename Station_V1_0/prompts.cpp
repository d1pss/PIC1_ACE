#include "headlib.hpp"
#include "telemetry.hpp"
#include "prompts.hpp"

#define PRESENCE_FLAG_NUMB 3


bool monitoring = false;
uint8_t sender = 0;
uint8_t mode = 0;

uint8_t broad_addr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};



const char* presence_FLAG[PRESENCE_FLAG_NUMB] = {       "Im in init_copter()",
                                                        "Im in loop_400Hz()",
                                                        "Im in autonomous_flight()"
                                                        };


//User command help chart
void promptHelp() {
    Serial.println(F("================================================"));
    Serial.println(F("          ESP32 DRONE NETWORK COMMANDS          "));
    Serial.println(F("================================================"));
    
    // Device Management Section
    Serial.println(F("[ DEVICE COMMANDS ]"));
    Serial.print(F("  ")); Serial.print(STAT_1);    Serial.println(F(" : Network Status Lv1"));
    Serial.print(F("  ")); Serial.print(STAT_2);    Serial.println(F(" : Network Status Lv2"));
    Serial.print(F("  ")); Serial.print(START_MON); Serial.println(F(" : Start Response Monitor"));
    Serial.print(F("  ")); Serial.print(END_MON);   Serial.println(F(" : End Response Monitor"));
    Serial.print(F("  ")); Serial.print(MY_MAC);    Serial.println(F(" : Show Device MAC Address"));
    Serial.print(F("  ")); Serial.print(LEAVE);     Serial.println(F(" : Disconnect / Leave Network"));
    Serial.print(F("  ")); Serial.print(REBOOT);    Serial.println(F(" : System Reboot"));
    
    Serial.println(F("------------------------------------------------"));
    
    // Network Section
    Serial.println(F("[ NETWORK NOTIFICATIONS ]"));
    Serial.print(F("  ")); Serial.print(FULL_NOTIF); Serial.println(F(" : Enable Full Notifications -- [8-R-M]"));
    Serial.print(F("  ")); Serial.print(NOTIF_R);    Serial.println(F(" : Ranking Update (No Mission) -- [9-R]"));
    Serial.print(F("  ")); Serial.print(NOTIF_M);    Serial.println(F(" : Mission Update (No Ranking) -- [A-M]"));
    
    Serial.println(F("------------------------------------------------"));
    
    // System Section
    Serial.print(F("  ")); Serial.print(HELP);       Serial.println(F(" : Show this Help Menu"));
    Serial.print(F("  ")); Serial.print(NO_COMM);    Serial.println(F(" : Null / No Command"));
    
    Serial.println(F("================================================"));
    Serial.println(F(" Enter Command ID Number and press Enter...     "));
}


void promptXY(STAT_Data data) 
{
    const uint8_t *aux_addr = data.peerInfo.peer_addr;
    Serial.printf("=== %s === | %02x:%02x:%02x:%02x:%02x:%02x's STA data: TX-CH: %hhu - NEIGH_Nº: %02x\r\n"
        ,data.online ? "ONLINE" : "OFFLINE"
        ,aux_addr[0],aux_addr[1],aux_addr[2],aux_addr[3],aux_addr[4],aux_addr[5]
        ,data.channel
        ,data.neighN
    );
}

void promptXX(DRONE_Data data)
{
    const uint8_t *aux_addr = data.peerInfo.peer_addr;
    Serial.printf("=== %s === | %02x:%02x:%02x:%02x:%02x:%02x's STA data: TX-CH: %hhu - AVAIL: %02x - TASK: %02x - RANK: %02x - VOLT: %2.1f - Altitude: %2.1f - ToF: %hu\r\n"
        ,data.online ? "ONLINE" : "OFFLINE"
        ,aux_addr[0],aux_addr[1],aux_addr[2],aux_addr[3],aux_addr[4],aux_addr[5]
        ,data.channel
        ,data.available
        ,data.mission
        ,data.rank
        ,data.fly_bat_voltage
        ,data.altitude
        ,data.tof_front
    );

}

void promptMyMAC(const uint8_t *addr)
{
    Serial.print(F("\n\n"));
    Serial.println(F("================================================"));
    Serial.printf("STATION ADDR - %02x:%02x:%02x:%02x:%02x:%02x\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
    Serial.println(F("================================================"));
    Serial.print(F("\n"));
}

void promptMyDATA()
{
    promptXY(db_STA[0]);
}



void promptMAC(const uint8_t *addr){
    Serial.printf(" == %02x:%02x:%02x:%02x:%02x:%02x == ",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
}


void cmdCheckUP(const uint8_t *my_addr) {
    // Check if there's anything in the hardware buffer
    uint16_t aux_idx = 0;
    char aux_buff[CMD_BUFF];

    memset(aux_buff,0,sizeof(CMD_BUFF));
    while (Serial.available() > 0) {
        char comm = Serial.read();

        // If it's a newline, the command is complete
        if (comm == '\n' || comm == '\r') {
            aux_buff[aux_idx] = '\0'; // Null-terminate the string
            cmdProcess(aux_buff,my_addr,&monitoring);     // Call your processing logic
            aux_idx = 0;                  // Reset for next command
        } 
        // Otherwise, add to buffer if there is room
        else if (aux_idx < CMD_BUFF - 1) {
            aux_buff[aux_idx++] = comm;
        }
    }
}

void cmdProcess(char* cmd, const uint8_t *my_addr, bool *monitoring) {

    //if(monitoring){
        Serial.print("Input detected: ");
        Serial.print(cmd[0]);
        Serial.print("\r\n");

        //testEcho(statData[0]);
    //}

    switch (cmd[0]) {
        case NO_COMM:
            // logic to arm
            break;
        case STAT_1:
            // logic to disarm
            Serial.println("Echoing my info.:");
            promptMyDATA();
            break;
        case STAT_2:
            // logic to disarm
            break;
        case START_MON:
            // logic to disarm
            Serial.println(F("MONITOR ON"));
            *monitoring = true;
            break;
        case END_MON:
            // logic to disar´
            Serial.println(F("MONITOR OFF"));
            *monitoring = false;
            break;
        case MY_MAC:
            // logic to disarm
            promptMyMAC(my_addr);
            break;
        case LEAVE:
            // logic to disarm
            break;
        case REBOOT:
            // logic to disarm
            ESP.restart();
            break;
        case FULL_NOTIF:
            // logic to disarm
            break;
        case NOTIF_R:
            // logic to disarm

            //build_mail();
            break;
        case NOTIF_M:
            // logic to disarm

            //build_mail();
            break;

        case SET_AVAL:
            // logic to disarm
            
            //build_mail();
            break;

        case SENDER:
            sender++;
            if(sender > 2) sender = 0;
            Serial.printf("Sender: %hhu\n",sender);
            //build_mail();
            break;

        case MODE:
            mode++;
            if(mode > 3) mode = 0;
            Serial.printf("Mode: %hhu\n",mode);
            
            //build_mail();
            break;
        case HELP:
            // logic to disarm
            promptHelp();
            break;
        default: 
            Serial.printf(" ========== Invalid command : Print 'F' for help ========== \r\n\n\n");
            //promptHelp();
            break;
            
    }
}

void unpackHEX(const uint8_t *author_mac,const uint8_t *rcv_data){
    //GENERIC PACKAGE:
    Serial.printf("By ");
    promptMAC(author_mac);
    Serial.printf(" - %02x",rcv_data[0]);
    uint8_t idx = 1;
    while(idx < 250) Serial.printf(":%02x",rcv_data[idx++]);
    Serial.println();
    
}

void unpackCHAR(const uint8_t *author_mac,const uint8_t *rcv_data){
    //GENERIC PACKAGE:
    Serial.printf("By ");
    promptMAC(author_mac);
    Serial.printf(" - %c",(char)rcv_data[0]);
    uint8_t idx = 1;
    while(idx < 250) Serial.printf(":%c",(char)rcv_data[idx++]);
    Serial.println();
    

}


void unpackF1(const uint8_t *author_mac,const uint8_t *rcv_data){
    //promptMAC(reciever_mac);
    if(rcv_data[3] == PING_XX[0] && rcv_data[4] == PING_XX[1] && rcv_data[5] == PING_XX[2] && rcv_data[6] == PING_XX[3]){
        //Format 1: Drone stats info
        float battery = 0.0f;
        float altitude = 0.0f;
        uint16_t ToF = 0;
        memcpy(&battery,&(rcv_data[10]),sizeof(float));
        memcpy(&altitude,&(rcv_data[14]),sizeof(float));
        memcpy(&ToF,&(rcv_data[18]),sizeof(uint16_t));

        Serial.printf("By ");
        promptMAC(author_mac);
        Serial.printf(" -> %c%c - Channel: %hhu - Key: %02x:%02x:%02x:%02x - Avail: %s - Mission: %02x - Rank: %02x - Battery: %01.2f - Altitude: %1.2f - ToF: %hu -\r\n"
            ,rcv_data[0],rcv_data[1]
            ,rcv_data[2]
            ,rcv_data[3],rcv_data[4],rcv_data[5],rcv_data[6] // PEER ADDR
            ,rcv_data[7] ? "true" : "false"
            ,rcv_data[8] 
            ,rcv_data[9]
            ,battery
            ,altitude
            ,ToF
            ,rcv_data[13],rcv_data[14]
        );
    }
    
}

void unpackF2(const uint8_t *author_mac,const uint8_t *rcv_data){
    //Format 2: PITCH, YAW, ROLL  (p, q, r)

    if(rcv_data[3] == PING_XX[0] && rcv_data[4] == PING_XX[1] && rcv_data[5] == PING_XX[2] && rcv_data[6] == PING_XX[3]){
        //Format 1: Drone stats info
        float p = 0.0f;
        float q = 0.0f;
        float r = 0.0f;

        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        memcpy(&p,&(rcv_data[15]),sizeof(float));
        memcpy(&q,&(rcv_data[19]),sizeof(float));
        memcpy(&r,&(rcv_data[23]),sizeof(float));

        Serial.printf("By ");
        promptMAC(author_mac);
        Serial.printf(" -> %c%c - Channel: %hhu - Key: %02x:%02x:%02x:%02x | Angle(º) | Roll: %02.2f - Pitch: %02.2f - Yaw: %02.2f | Acell. | - X: %02.2f - Y: %02.2f - Y: %02.2f -\r\n"
            ,rcv_data[0],rcv_data[1]
            ,rcv_data[2]
            ,rcv_data[3],rcv_data[4],rcv_data[5],rcv_data[6] // PEER ADDR
            ,p
            ,q
            ,r
            ,ax
            ,ay
            ,az
            //,rcv_data[13],rcv_data[14]
        );
    }
    
}

void unpackF3(const uint8_t *author_mac,const uint8_t *rcv_data){
    //Format 2: ACC_X, ACC_Y, ACC_Z, VEL_Z

    Serial.printf("== %c%c == ",rcv_data[0],rcv_data[1]);
    uint8_t idx = 1;
    
}

void unpackF4(const uint8_t *author_mac,const uint8_t *rcv_data){
    Serial.printf("By ");
    uint8_t idx = 0;
    promptMAC(author_mac);
    Serial.printf(" - %c%c - Channel: %hhu - Key: %02x:%02x:%02x:%02x - Mode: %hhu - Code Position: %s -\n"
        ,rcv_data[idx++] ,rcv_data[idx++]
        ,rcv_data[idx++]
        ,rcv_data[idx++],rcv_data[idx++],rcv_data[idx++],rcv_data[idx++]
        ,rcv_data[idx++]
        ,presence_FLAG[rcv_data[idx++]]
        
        );
    
}
