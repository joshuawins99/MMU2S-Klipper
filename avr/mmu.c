#include "mmu.h"

static uint8_t mmu_rx_buf[MMU_RX_SIZE];
static volatile uint8_t mmu_rx_head = 0;
static volatile uint8_t mmu_rx_tail = 0;
static volatile uint32_t mmu_rx_overflows = 0;

static inline void mmu_rx_push(uint8_t b) {
    uint8_t next = (mmu_rx_head + 1) & MMU_RX_MASK;
    if (next == mmu_rx_tail) {
        mmu_rx_overflows++;
        return;
    }
    mmu_rx_buf[mmu_rx_head] = b;
    mmu_rx_head = next;
}

uint8_t mmu_uart_read(uint8_t *out, uint8_t maxlen) {
    uint8_t count = 0;
    while (mmu_rx_tail != mmu_rx_head && count < maxlen) {
        out[count++] = mmu_rx_buf[mmu_rx_tail];
        mmu_rx_tail = (mmu_rx_tail + 1) & MMU_RX_MASK;
    }
    return count;
}

void mmu_uart_init(void) {
    // Match Prusa: PH0 input with pull-up (RX), PH1 is TX
    DDRH &= ~0x01;
    PORTH |= 0x01;

    // Double-speed mode
    UCSR2A |= (1 << U2X2);

    // Baud
    UBRR2 = (uint16_t)MMU_BAUD_SELECT(MMU_BAUD, CONFIG_CLOCK_FREQ);

    // 8N1
    UCSR2C = (1<<UCSZ21) | (1<<UCSZ20);

    // Enable RX, TX, RX interrupt
    UCSR2B = (1<<RXEN2) | (1<<TXEN2) | (1<<RXCIE2);
}

static inline char mmu_hex_digit(uint8_t v) {
    v &= 0xF;
    return (v < 10) ? ('0' + v) : ('A' + (v - 10));
}

ISR(USART2_RX_vect) {
    uint8_t data = UDR2;
    mmu_rx_push(data);
}

void mmu_uart_send(uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        while (!(UCSR2A & (1<<UDRE2)));
        UDR2 = data[i];
    }
}

void command_mmu_write(uint32_t *args) {
    uint8_t len = args[0];
    uint8_t *data = command_decode_ptr(args[1]);

    mmu_uart_send(data, len);
}

void command_mmu_read(uint32_t *args) {
    uint8_t max = args[0];
    uint8_t buf[64];

    if (max > sizeof(buf)) {
        max = sizeof(buf);
    }

    uint8_t count = mmu_uart_read(buf, max);
    
    if (count == 0) {
        sendf("mmu_read_response data=%*s count=%u overflow=%u", 0, "", (uint32_t)count, (uint32_t)mmu_rx_overflows);
        return;
    }

    // Convert to hex ASCII
    char hexbuf[MMU_RX_SIZE * 2];
    uint8_t p = 0;

    for (uint8_t i = 0; i < count; i++) {
        hexbuf[p++] = mmu_hex_digit(buf[i] >> 4);
        hexbuf[p++] = mmu_hex_digit(buf[i] & 0xF);
    }
    sendf("mmu_read_response data=%*s count=%u overflow=%u", p, hexbuf, (uint32_t)count, (uint32_t)mmu_rx_overflows);
}