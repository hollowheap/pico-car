#ifndef TEST_RADIO_PORT_H
#define TEST_RADIO_PORT_H

#define CYW43_EIO 9
#define CYW43_ETIMEDOUT 2
#define CYW43_NO_NETUTILS 1
#define MIN(a, b) ((a) < (b) ? (a) : (b))
void cyw43_thread_enter(void);
void cyw43_thread_exit(void);

#endif
