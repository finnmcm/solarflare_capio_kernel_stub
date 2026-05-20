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

    sfc->fd = open(DEVSFC7120, O_RDWR);
    if (sfc->fd < 0) {
        perror("sfc7120_init: open " DEVSFC7120);
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
