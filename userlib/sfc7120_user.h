#ifndef SFC7120_USER_H
#define SFC7120_USER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/ioccom.h>

#include "../sfc7120_uapi.h"
#include "modmap.h"

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

#define DEVSFC7120 "/dev/sfc7120pol1" // our actual module in tree
#define DEVMODMAP  "/dev/modmap"

typedef struct sfc7120_if { // state struct, everything we need from kernel stub 
    int     fd;
    int     modmap_fd;

    cap_req_t cap_req;

    uint8_t mac_addr[6];

    void   *tx_buffer;
    void   *rx_buffer;

    void   *cap_token;  /* malloc'd page — raw material for CAPIO_ATTACH seal */
} sfc7120_if_t;

int  sfc7120_init(sfc7120_if_t *sfc);
void sfc7120_destroy(sfc7120_if_t *sfc);
int  sfc7120_tx(sfc7120_if_t *sfc, const void *buf, size_t len);
int  sfc7120_rx(sfc7120_if_t *sfc, void *buf, size_t *len_out);

#endif /* SFC7120_USER_H */
