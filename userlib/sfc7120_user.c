#include "sfc7120_user.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

static void *
map_buffer(sfc7120_if_t *sfc, sfc7120_vm_map_type_t map_type, size_t len)
{
    user_map_req_t map_req;
    map_req.user_cap   = sfc->cap_req.user_cap;
    map_req.sealed_cap = sfc->cap_req.sealed_cap;
    map_req.map_type   = (int)map_type;

    mmap_req_user_t req;
    req.addr  = NULL;
    req.len   = len;
    req.prot  = PROT_READ | PROT_WRITE;
    req.flags = MAP_SHARED;
    req.fd    = sfc->fd;
    req.pos   = 0;
    req.extra = (void * __capability)(&map_req);

    if (ioctl(sfc->modmap_fd, MODMAPIOC_MAP, &req) < 0) {
        perror("sfc7120: MODMAPIOC_MAP failed");
        return NULL;
    }
    return req.addr;
}

int
sfc7120_init(sfc7120_if_t *sfc)
{
    sfc->fd        = -1;
    sfc->modmap_fd = -1;
    sfc->tx_buffer = NULL;
    sfc->rx_buffer = NULL;

    const char *dev = sfc->dev_path != NULL ? sfc->dev_path : DEVSFC7120;
    sfc->fd = open(dev, O_RDWR);
    if (sfc->fd < 0) {
        fprintf(stderr, "sfc7120_init: open %s: ", dev);
        perror(NULL);
        return -1;
    }

    sfc->modmap_fd = open(DEVMODMAP, O_RDWR);
    if (sfc->modmap_fd < 0) {
        perror("sfc7120_init: open " DEVMODMAP);
        goto fail;
    }

    sfc->cap_token = malloc(PAGE_SIZE); // random page malloced that we can use to verify using the kernel to see if we actually own this memory in the process
    if (sfc->cap_token == NULL) {
        perror("sfc7120_init: malloc cap_token");
        goto fail;
    }
    sfc->cap_req.user_cap = sfc->cap_token;

    if (ioctl(sfc->fd, CAPIO_ATTACH, &sfc->cap_req) < 0) { // the above random page is used as part of capio_attach 
        perror("sfc7120_init: CAPIO_ATTACH");
        goto fail;
    }

    sfc->tx_buffer = map_buffer(sfc, SFC7120_TX_BUFFER,
                                SFC7120_NUM_TX_DESC * SFC7120_TX_BUFFER_SIZE);
    if (sfc->tx_buffer == NULL) {
        perror("sfc7120_init: map TX buffer");
        goto fail;
    }

    sfc->rx_buffer = map_buffer(sfc, SFC7120_RX_BUFFER,
                                SFC7120_NUM_RX_DESC * SFC7120_RX_BUFFER_SIZE);
    if (sfc->rx_buffer == NULL) {
        perror("sfc7120_init: map RX buffer");
        goto fail;
    }

    sfc7120_mac_req_t mac_req;
    mac_req.user_cap   = sfc->cap_req.user_cap;
    mac_req.sealed_cap = sfc->cap_req.sealed_cap;
    if (ioctl(sfc->fd, SFC7120_GET_MAC, &mac_req) < 0) {
        perror("sfc7120_init: SFC7120_GET_MAC");
        goto fail;
    }
    memcpy(sfc->mac_addr, mac_req.mac_addr, 6);

    return 0;

fail:
    sfc7120_destroy(sfc);
    return -1;
}

/*
 * sfc7120_tx — send one packet through the kernel ioctl path (phase 1).
 *
 * Packages buf/len into a sfc7120_tx_req_t and hands it to the kernel via
 * SFC7120_TX. The kernel copies the packet into the TX DMA buffer, writes
 * a descriptor, rings the doorbell, and polls the EVQ for the TX completion
 * event before returning. Blocking — returns 0 on success, -1 on failure.
 */
int
sfc7120_tx(sfc7120_if_t *sfc, const void *buf, size_t len)
{
    sfc7120_tx_req_t req;
    req.user_cap    = sfc->cap_req.user_cap;
    req.sealed_cap  = sfc->cap_req.sealed_cap;
    req.tx_buf_addr = (uint8_t * __capability)buf;
    req.length      = len;
    req.status      = 0;

    if (ioctl(sfc->fd, SFC7120_TX, &req) < 0) {
        perror("sfc7120_tx: SFC7120_TX");
        return -1;
    }
    return 0;
}

/*
 * sfc7120_rx — receive one packet through the kernel ioctl path (phase 1).
 *
 * Passes buf to the kernel via SFC7120_RX. The kernel polls the EVQ until
 * an RX event arrives, copies the packet from the DMA buffer into buf, and
 * writes the byte count into *len_out. Blocking — returns 0 on success,
 * -1 on failure (including ETIMEDOUT if no packet arrives in time).
 */
int
sfc7120_rx(sfc7120_if_t *sfc, void *buf, size_t *len_out)
{
    sfc7120_rx_req_t req;
    req.user_cap        = sfc->cap_req.user_cap;
    req.sealed_cap      = sfc->cap_req.sealed_cap;
    req.raw_buffer      = buf;
    req.length_received = 0;
    req.status          = 0;
    req.error           = 0;

    if (ioctl(sfc->fd, SFC7120_RX, &req) < 0) {
        perror("sfc7120_rx: SFC7120_RX");
        return -1;
    }

    if (len_out != NULL)
        *len_out = req.length_received;
    return 0;
}

void
sfc7120_destroy(sfc7120_if_t *sfc)
{
    if (sfc->cap_token != NULL) {
        free(sfc->cap_token);
        sfc->cap_token = NULL;
    }
    if (sfc->modmap_fd >= 0)
        close(sfc->modmap_fd);
    if (sfc->fd >= 0)
        close(sfc->fd);
}
