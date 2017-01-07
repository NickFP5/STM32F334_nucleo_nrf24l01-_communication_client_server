



void low_csn();
void high_csn();

void low_ce();

void high_ce();

void flush_tx_fifo();

void flush_rx_fifo();

void write_register(uint8_t reg, uint8_t value);

uint8_t read_register(uint8_t reg);

void write_bit(uint8_t reg, uint8_t pos, uint8_t value);

uint8_t read_bit(uint8_t reg, uint8_t pos);

void set_rx_mode();

void set_tx_mode();

void initialize_nrf24l01();

void send_payload(uint8_t *b);

void read_payload();