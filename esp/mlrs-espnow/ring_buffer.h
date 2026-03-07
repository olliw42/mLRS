//*******************************************************
// mLRS ESP-NOW
// Copyright (c) www.olliw.eu, OlliW, OlliW42
// License: GPL v3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// Ring Buffer
//*******************************************************
// 7. Mar. 2026
//*******************************************************
#ifndef RING_BUFFER_H
#define RING_BUFFER_H


// power-of-two ring buffer for ISR -> loop data transfer.
// push is safe to call from ISR / promiscuous callback.
// pop is called only from loop().

#define RXBUF_SIZE  2048

uint8_t rxbuf[RXBUF_SIZE];
volatile uint16_t rxbuf_head;
volatile uint16_t rxbuf_tail;

void rxbuf_init(void)
{
    rxbuf_head = 0;
    rxbuf_tail = 0;
}

void rxbuf_push(const uint8_t* data, int len)
{
    for (int i = 0; i < len; i++) {
        uint16_t next = (rxbuf_head + 1) & (RXBUF_SIZE - 1);
        if (next == rxbuf_tail) break; // full, drop
        rxbuf[rxbuf_head] = data[i];
        rxbuf_head = next;
    }
}

int rxbuf_pop(uint8_t* buf, int maxlen)
{
    int cnt = 0;
    while (rxbuf_tail != rxbuf_head && cnt < maxlen) {
        buf[cnt++] = rxbuf[rxbuf_tail];
        rxbuf_tail = (rxbuf_tail + 1) & (RXBUF_SIZE - 1);
    }
    return cnt;
}


#endif // RING_BUFFER_H
