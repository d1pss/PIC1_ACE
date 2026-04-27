#ifndef USER_CUSTOM_H
#define USER_CUSTOM_H

#include <Arduino.h>
#include "esp_now.h"

// Drone codename           --> FLY
// Controller codename      --> ATOM
// Station kind codename    --> CHRG (charger)


extern const uint8_t EXTERNAL_ESP_MAC[]; 


// A state variable for your useful function (e.g., a toggle)
extern bool special_mode_active;




void register_external_peer();  // Registar peer para conexão
void send_ping_to_external(const uint8_t *peer_mac);   // Aferir conexão (CTRLR - REMOTE) por ESPNOW





// Controller Variable Remapping
void handle_custom_buttons();




// General Use Auxiliary Functions

/**
 * @brief Evaluates the equality of two mac addresses
 * 
 * @param a 
 * @param b 
 * @return true 
 * @return false 
 * 
 * @side effect: Trusts a and b are an array of 6 bytes length
 */
bool is_mac_equal(const uint8_t *a, const uint8_t *b);




#endif