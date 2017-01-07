#define FIX 0
#define TIM 1
#define INC 2
#define LED 3

void GET_command(uint8_t id_var);
void SET_command(uint8_t id_var, uint8_t * value);
void VER_command();