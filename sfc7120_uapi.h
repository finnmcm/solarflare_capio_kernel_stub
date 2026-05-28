#ifndef SFC7120_UAPI_H
#define SFC7120_UAPI_H

#include <stdint.h>
#include <stddef.h>
#include <sys/ioccom.h>

/* Queue geometry — must match the kernel ring allocations. */
#define SFC7120_NUM_TX_DESC    512
#define SFC7120_NUM_RX_DESC    512
#define SFC7120_NUM_EVQ_ENTRY  512
#define SFC7120_TX_BUFFER_SIZE 2048
#define SFC7120_RX_BUFFER_SIZE 2048

#define SFC7120_EVQ_ENTRY_SIZE 8
#define SFC7120_TX_DESC_SIZE   8
#define SFC7120_RX_DESC_SIZE   8

/* Memory region indices — must match sfc7120_vm_map_type_t in sfc7120.h. */
typedef enum {
    SFC7120_TX_BUFFER,
    SFC7120_RX_BUFFER,
    SFC7120_MMIO_REGION,
    SFC7120_REGION_COUNT
} sfc7120_vm_map_type_t;

/* Every CAPIO IOCTL struct must begin with user_cap + sealed_cap. */

typedef struct sfc7120_mac_req {
    void * __capability user_cap;
    void * __capability sealed_cap;

    uint8_t mac_addr[6];
} sfc7120_mac_req_t;

typedef struct sfc7120_user_packet_descriptor {
    uint8_t *start_ptr;
    size_t   length;
} sfc7120_user_packet_descriptor_t;

typedef struct sfc7120_tx_req {
    void * __capability    user_cap;
    void * __capability    sealed_cap;

    uint8_t * __capability tx_buf_addr;
    size_t                 length;
    uint8_t                status;
} sfc7120_tx_req_t;

typedef struct sfc7120_rx_req {
    void * __capability user_cap;
    void * __capability sealed_cap;

    uint8_t                          *raw_buffer;
    sfc7120_user_packet_descriptor_t  descriptors[SFC7120_NUM_RX_DESC];
    size_t                            descriptor_length;
    size_t                            length_received;

    uint8_t status;
    uint8_t error;
} sfc7120_rx_req_t;

/* IOCTLs — 'S' group matches the kernel definition. */
#define SFC7120_RX      _IOWR('S', 1, sfc7120_rx_req_t)
#define SFC7120_TX      _IOWR('S', 2, sfc7120_tx_req_t)
#define SFC7120_GET_MAC _IOWR('S', 3, sfc7120_mac_req_t)

#endif /* SFC7120_UAPI_H */
