#ifndef CAPIO_H
#define CAPIO_H

#include <sys/types.h>
#include <sys/param.h>       
#include <sys/conf.h>        
#include <sys/mutex.h>       
#include <sys/ioccom.h>      
#include <cheri/cheric.h>    
#include <cheri/cheri.h>     

typedef bool    (is_dying_t)(void *sc);
typedef size_t  (get_buffer_size_t)(void *sc, int type);

typedef struct cap_req {
    void* __capability user_cap;
    void* __capability sealed_cap;
} cap_req_t;

typedef struct slice_def {
    uint64_t offset;
    const char* name;
    bool read_only;
    uint64_t length;
} slice_def_t;

typedef struct {
    void* __capability user_cap;
    void* __capability sealed_cap;

    int map_type;

    // return values
    bool            is_sliced;
    slice_def_t*    slice_definitions;
	size_t			slice_def_length;
} map_req_t;

typedef struct {
    int             type;
    bool            is_physical;
    void*           addr;
    vm_paddr_t      paddr;
    vm_offset_t     offset;
    size_t          len;
    bool            mapped;
    bool            is_sliced;
    slice_def_t*    slice_definitions;
    size_t          slice_def_len;
    void*           vm_object_handle;  /* persistent handle for cdev_pager */
    vm_memattr_t    memattr;           /* 0 = default; non-zero = override page attr */
} shared_mem_region_t;

typedef struct {
    d_ioctl_t           *ioctl;
    is_dying_t          *is_dying;
    get_buffer_size_t   *get_buffer_size;
} capio_ops_t;

typedef struct {
    void * __capability original_cap;
    void * __capability sealed_cap;
} sealed_cap_state_t;

typedef struct {
    device_t            dev;
    void* __capability  sealing_key;
    sealed_cap_state_t  cap_state;
    struct mtx          cap_mtx;
    bool                mapped;
    size_t              shared_mem_regs_len;
    shared_mem_region_t *shared_mem_regs;
    capio_ops_t         *capio_ops;
} capio_softc_t;

typedef struct {
    device_t dev;
    capio_softc_t capio_sc;
} capio_softc_header_t;

typedef struct {
    void *sc;
    int type;
} vm_object_handle_t;

void init_capio_sc(void *sc, capio_ops_t *capio_ops, size_t smem_len, shared_mem_region_t *smem);
void capio_destroy(void *sc);
int grant_access(void *sc, cap_req_t* req);
int revoke_access(void *sc);
bool is_mapped(void *sc);
struct cdev *make_dev_capio(struct cdevsw *_devsw, void *sc, int _unit, uid_t _uid, gid_t _gid,
		int _perms, const char *_fmt, ...) __printflike(7, 8);

int	capio_pager_fault(vm_object_t obj, vm_ooffset_t offset,
    int prot, vm_page_t *mres);

#define CAPIO_ATTACH        _IOWR('C', 1, cap_req_t)
#define CAPIO_GOODBYE       _IOWR('C', 2, cap_req_t)

#endif