#include "user_custom.h"
#include "main.h" 

bool special_mode_active = false;









void handle_custom_buttons() {
    static bool last_btn_state = false;
    // Let's say you want to use the Left Stick Click (ARM_BUTTON) 
    // to toggle a "Special Mode" instead of its default behavior
    bool current_state = getArmButton(); 

    if (current_state && !last_btn_state) {
        special_mode_active = !special_mode_active;
        // Add a small beep to confirm the toggle
        // buzzer_sound(2000, 100); 
    }
    last_btn_state = current_state;
}



// General use functions's body

bool is_mac_equal(const uint8_t *a, const uint8_t *b){
    return (memcmp(a, b, 6) == 0);
}