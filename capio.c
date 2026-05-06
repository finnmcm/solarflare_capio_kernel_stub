#include "capio.h"

#include "machine/vm.h"
#include "sys/types.h"
#include <machine/stdarg.h>

#include <sys/systm.h>        
#include <sys/kernel.h>       
#include <sys/bus.h>          
#include <sys/lock.h>         
#include <sys/rwlock.h>       
#include <sys/malloc.h>       
#include <sys/proc.h>         
#include <sys/ucred.h>        


#include <vm/vm.h>            
#include <vm/vm_param.h>      
#include <vm/pmap.h>          
#include <vm/vm_map.h>        
#include <vm/vm_object.h>     
#include <vm/vm_page.h>       
#include <vm/vm_pager.h>

#if CROSS_COMPILE
#include "../CheriModmap/modmap_api.h"
#else
#include "modmap_api.h"
#endif

MALLOC_DECLARE(M_CAPIO);
MALLOC_DEFINE(M_CAPIO, "capio", "capio");

#define CAP_LOCK_INIT(sc) mtx_init(&(sc)->cap_mtx, device_get_nameunit((sc)->dev), "capio softc lock", MTX_DEF)
#define CAP_LOCK(sc)      mtx_lock(&(sc)->cap_mtx)
#define CAP_UNLOCK(sc)    mtx_unlock(&(sc)->cap_mtx)
#define CAP_LOCK_DESTROY(sc) mtx_destroy(&(sc)->cap_mtx)

static int	capio_pager_ctor(void *handle, vm_ooffset_t size,
    vm_prot_t prot, vm_ooffset_t foff, struct ucred *cred, u_short *color);
static void	capio_pager_dtor(void *handle);
int	capio_pager_fault(vm_object_t obj, vm_ooffset_t offset,
    int prot, vm_page_t *mres);

static struct cdev_pager_ops capio_cdev_pager_ops = {
	.cdev_pg_ctor = capio_pager_ctor,
	.cdev_pg_dtor = capio_pager_dtor,
	.cdev_pg_fault = capio_pager_fault,
};

static d_ioctl_t capio_ioctl_handler;
static d_mmap_single_extra_t capio_mmap_single_extra;

static int capio_vaddr_callback(struct thread *td, vm_offset_t vaddr, vm_size_t size, void* arg, void* extra){
    capio_softc_header_t* header = (capio_softc_header_t *)arg;
    capio_softc_t *sc = &header->capio_sc;
    map_req_t *req = (map_req_t*)extra;
    bool region_found = false;

    if(sc == NULL){
        return EINVAL;
    }

    CAP_LOCK(sc);
    for(size_t i = 0; i < sc->shared_mem_regs_len; i++){
        shared_mem_region_t *smem = &sc->shared_mem_regs[i];
        if(smem->type == req->map_type){
            smem->offset = vaddr;
            smem->len = size;
            region_found = true;
            break;
        }
    }
    CAP_UNLOCK(sc);

    if(!region_found){
        return EINVAL;
    }

    return 0;
}

static region_sizes_t capio_slice_length_callback(int map_type, void* arg){
    capio_softc_header_t* header = (capio_softc_header_t *)arg;
    capio_softc_t *sc = &header->capio_sc;
    region_sizes_t sizes;

    sizes.region_length = 0;
    sizes.slice_def_length = 0;

    if(sc == NULL){
        return sizes;
    }

    CAP_LOCK(sc);
    for(size_t i = 0; i < sc->shared_mem_regs_len; i++){
        shared_mem_region_t *smem = &sc->shared_mem_regs[i];
        if(smem->type == map_type){
            sizes.region_length = smem->len;
            sizes.slice_def_length = smem->slice_def_len;

            CAP_UNLOCK(sc);
            return sizes;
        }
    }
    CAP_UNLOCK(sc);

    return sizes;
}

static void* __capability
create_sealing_key(size_t id){
    if(id >= cheri_getbase(kernel_root_sealcap)){
        return NULL;
    }

    void * __capability derived = cheri_setaddress(kernel_root_sealcap, cheri_getbase(kernel_root_sealcap) + id);
    derived = cheri_setbounds(derived, 1);

    return derived;
}

void init_capio_sc(void *sc, capio_ops_t *capio_ops, size_t smem_len, shared_mem_region_t *smem){
    capio_softc_header_t *header = (capio_softc_header_t*)sc;

    header->capio_sc.dev = header->dev;
    CAP_LOCK_INIT(&header->capio_sc);

    header->capio_sc.sealing_key = create_sealing_key(0x1234);
    header->capio_sc.shared_mem_regs_len    = smem_len;
    header->capio_sc.shared_mem_regs        = smem;
    header->capio_sc.capio_ops              = capio_ops;
}

void capio_destroy(void *sc){
    capio_softc_header_t *header = (capio_softc_header_t*)sc;

    CAP_LOCK_DESTROY(&header->capio_sc);
}

struct cdev *
make_dev_capio(struct cdevsw *devsw, void *sc, int unit, uid_t uid, gid_t gid, int mode,
    const char *fmt, ...)
{
	struct cdev *dev;
	va_list ap;
	int ret;
    char* str;

	va_start(ap, fmt);
    ret = vasprintf(&str, M_CAPIO, fmt, ap);
	va_end(ap);

    if (ret == -1) {
        return NULL;
    }

    devsw->d_ioctl = capio_ioctl_handler;
    devsw->d_mmap_single_extra = capio_mmap_single_extra;

    dev = make_dev(devsw, unit, uid, gid, mode, "%s", str);

    modmap_register_vaddr_callback(dev, capio_vaddr_callback, sc);
    modmap_register_slice_callback(dev, capio_slice_length_callback, sc);

    free(str, M_CAPIO);

	return dev;
}
static int check_cap_token(capio_softc_t* sc, void* __capability cap_token){
    if(sc->cap_state.sealed_cap == NULL){
        return EINVAL;
    }

    if(cap_token == NULL){
        return EINVAL;
    }

    void* __capability unsealed_token = cheri_unseal(cap_token, sc->sealing_key);
    if(!cheri_ptr_equal_exact(unsealed_token, sc->cap_state.original_cap)){
        return EPERM;
    }

    return 0;
}

static int
delete_mapping_from_user(capio_softc_t *sc, vm_offset_t offset, size_t len, struct thread *td){
    struct proc *p;
    vm_map_t map;
    vm_map_entry_t out_entry;
    vm_object_t object;
    vm_pindex_t pindex;
    vm_prot_t out_prot;
    boolean_t wired;
    int error;

    p = td->td_proc;
    if (p->p_vmspace == NULL) {
        device_printf(sc->dev, "No vmspace, mapping already cleaned up\n");
        return 0;
    }

    map = &p->p_vmspace->vm_map;

    error = vm_map_lookup(&map, offset, VM_PROT_READ, &out_entry, &object, &pindex, &out_prot, &wired);

    if(error != KERN_SUCCESS){
        device_printf(sc->dev, "vm_map_lookup failed: %d (mapping likely already removed)\n", error);
        return 0;
    }

    vm_map_lookup_done(map, out_entry);

    // I don't think we're supposed to aquire the vm lock when calling this function
    error = vm_map_remove(map, offset, offset + len);
    switch(error){
        case KERN_SUCCESS:
            return 0;
        case KERN_INVALID_ADDRESS:
            device_printf(sc->dev, "Mapping was removed by another thread\n");
            return 0;        
        default:
            device_printf(sc->dev, "vm_map_remove failed: %d\n", error);
            return EIO;
    }
}

// probably will be expanded in the future to revoke all caps in the vm object and such
static int revoke_cap_token(capio_softc_t* sc, struct thread *td){
    int error = 0;

    device_printf(sc->dev, "revoking cap token\n");

    CAP_LOCK(sc);
    sc->cap_state.original_cap = NULL;
    sc->cap_state.sealed_cap = NULL;
    CAP_UNLOCK(sc);

    for(size_t i = 0; i < sc->shared_mem_regs_len; i++){
        shared_mem_region_t smem;

        CAP_LOCK(sc);
        smem = sc->shared_mem_regs[i];
        CAP_UNLOCK(sc);

        if(smem.offset != 0){
            device_printf(sc->dev, "Deleting tx mapping for user\n");

            error = delete_mapping_from_user(sc, smem.offset, smem.len, td);
            if(!error){
                // only clear on success because we know the mapping is gone
                CAP_LOCK(sc);
                sc->shared_mem_regs[i].offset = 0;
                sc->shared_mem_regs[i].len = 0;
                CAP_UNLOCK(sc);
            }
        }
        else{
            device_printf(sc->dev, "No offset for user\n");
        }
    }

    return error;
}

static int set_cap_token(capio_softc_t* sc, cap_req_t* req){
    uprintf("Entering set cap token\n");
    CAP_LOCK(sc);

    if( req->user_cap == NULL ||
        sc->cap_state.sealed_cap != NULL){
        CAP_UNLOCK(sc);
        return EINVAL;
    }

    // seal the cap the user provided and give it to them
    sc->cap_state.original_cap = req->user_cap;
    sc->cap_state.sealed_cap = cheri_seal(req->user_cap, sc->sealing_key);

    req->sealed_cap = sc->cap_state.sealed_cap;

    CAP_UNLOCK(sc);
    return (0);
}

int grant_access(void* sc, cap_req_t* req) {
    capio_softc_header_t *header = (capio_softc_header_t*)sc;

    if(header == NULL){
        return EINVAL;
    }

    set_cap_token(&header->capio_sc, req);

    return 0;
}

int revoke_access(void *sc){
    if(sc == NULL){
        return EINVAL;
    }

    capio_softc_header_t *header = (capio_softc_header_t*)sc;

    int err = revoke_cap_token(&header->capio_sc, curthread);

    return err;
}

static int capio_mmap_single_extra(struct cdev *cdev, vm_ooffset_t *offset, vm_size_t size, vm_object_t *object, int nprot, void * __kerncap extra, vm_map_t map){
    void *sc = cdev->si_drv1;
    capio_softc_header_t *header_sc = (capio_softc_header_t*)sc;
    capio_softc_t *cap_sc; 
	vm_object_t obj;
    cap_req_t* __kerncap req = NULL;
    map_req_t* __kerncap map_req = NULL;
    int error;

    // need to have a user request at all to make this work
    if( extra == NULL ||
        sc == NULL){
        return EINVAL;
    }

    req = (cap_req_t* __kerncap)extra;
    cap_sc = &header_sc->capio_sc;

    // validate user has access to device
    error = check_cap_token(cap_sc, req->sealed_cap);
    if(error){
        return error;
    }

    if (nprot & (VM_PROT_READ_CAP | VM_PROT_WRITE_CAP)) {
        device_printf(cap_sc->dev, "Rejecting mmap with capability permissions - device memory cannot store capabilities\n");
        return EINVAL;
    }

    map_req = (map_req_t* __kerncap)extra;

    // validate that request is properly formed
	CAP_LOCK(cap_sc);
    if(offset != NULL && *offset != 0){
		CAP_UNLOCK(cap_sc);
        return EINVAL;
    }
    CAP_UNLOCK(cap_sc);

    // capio ops shouldn't change after setting them
    if(cap_sc->capio_ops->is_dying(sc)){
        return ENXIO;
    }

    // make sure tag provided is valid
    if(!cheri_gettag(req->user_cap)){
        return EINVAL;
    }

    size_t buffer_size = cap_sc->capio_ops->get_buffer_size(sc, map_req->map_type);

    if (size != buffer_size){
        return EINVAL;
    }

    bool is_sliced = false;
    slice_def_t* slice_def = NULL;
    vm_memattr_t mem_attributes = VM_MEMATTR_DEFAULT;
    size_t slice_def_len = 0;

    CAP_LOCK(cap_sc);
    for(size_t i = 0; i < cap_sc->shared_mem_regs_len; i++){
        shared_mem_region_t smem = cap_sc->shared_mem_regs[i];
        if(smem.type == map_req->map_type){
            is_sliced = smem.is_sliced;
            slice_def = smem.slice_definitions;
            slice_def_len = smem.slice_def_len;

            device_printf(cap_sc->dev, "Found region: type=%d, is_sliced=%d, is_physical=%d\n",
                     smem.type, smem.is_sliced, smem.is_physical);

            if(smem.is_physical){
                mem_attributes = VM_MEMATTR_UNCACHEABLE;
            }

            if(smem.mapped){
                return EINVAL;
            }

            break;
        }
    }
    CAP_UNLOCK(cap_sc);

    vm_object_handle_t *handle = malloc(sizeof(vm_object_handle_t), M_DEVBUF, M_ZERO | M_WAITOK);
    if (handle == NULL) {
        return ENOMEM;
    }

    handle->sc = sc;
    handle->type = map_req->map_type;

    // create vm object for user
	obj = cdev_pager_allocate(handle, OBJT_DEVICE, &capio_cdev_pager_ops,
	    OFF_TO_IDX(buffer_size), nprot, *offset, curthread->td_ucred);
	if (obj == NULL)
		return (ENXIO);

    if (mem_attributes != VM_MEMATTR_DEFAULT) {
        VM_OBJECT_WLOCK(obj);
        obj->memattr = mem_attributes;
        VM_OBJECT_WUNLOCK(obj);
    }

	/*
	 * If an unload started while we were allocating the VM
	 * object, dying will now be set and the unloading thread will
	 * be waiting in destroy_dev().  Just release the VM object
	 * and fail the mapping request.
	 */
	if(cap_sc->capio_ops->is_dying(sc)){
        return ENXIO;
    }

    map_req->is_sliced = is_sliced;
    map_req->slice_definitions = slice_def;
    map_req->slice_def_length = slice_def_len;
	*object = obj;

	return (0);
}

static int
capio_pager_ctor(void *handle, vm_ooffset_t size, vm_prot_t prot, vm_ooffset_t foff, struct ucred *cred, u_short *color){
    vm_object_handle_t *obj_handle = (vm_object_handle_t *)handle;
    capio_softc_t *sc = &((capio_softc_header_t*)obj_handle->sc)->capio_sc;
    int error = 0;

	CAP_LOCK(sc);
	sc->mapped = true;
    for(size_t i = 0; i < sc->shared_mem_regs_len; i++){
        shared_mem_region_t *smem = &sc->shared_mem_regs[i];
        if(smem->type == obj_handle->type){
            smem->mapped = true;
        }
    }
    CAP_UNLOCK(sc);

	*color = 0;
	return error;
}

static void
capio_pager_dtor(void *handle){
    vm_object_handle_t *obj_handle = (vm_object_handle_t *)handle;
    capio_softc_t *sc = &((capio_softc_header_t*)obj_handle->sc)->capio_sc;

	CAP_LOCK(sc);
    bool no_mapped = true;
    for(size_t i = 0; i < sc->shared_mem_regs_len; i++){
        shared_mem_region_t *smem = &sc->shared_mem_regs[i];
        if(smem->type == obj_handle->type){
            smem->mapped = false;
        }

        if(smem->mapped){
            no_mapped = false;
        }
    }

    if(no_mapped){
        sc->mapped = false;
    }
	CAP_UNLOCK(sc);

    free(obj_handle, M_DEVBUF);
}

int
capio_pager_fault(vm_object_t obj, vm_ooffset_t offset, int prot, vm_page_t *mres){
    vm_object_handle_t *handle = (vm_object_handle_t *)obj->handle;
    capio_softc_t *sc = &((capio_softc_header_t*)handle->sc)->capio_sc;
	vm_page_t page;
	vm_paddr_t paddr;
    bool paddr_found = false;
    vm_memattr_t mem_attribute = VM_MEMATTR_DEFAULT;

    device_printf(sc->dev, "Fault: offset=0x%lx, pindex=%lu, mres pindex=%lu, PAGE_SIZE=%d\n",
              offset, OFF_TO_IDX(offset), (*mres)->pindex, PAGE_SIZE);

    // shared_mem_regs_len also should't change
    // basically requires static declaration of all memory regions you intend to share with the user
    // additionally only the offset should change for 
    for(size_t i = 0; i < sc->shared_mem_regs_len; i++){
        shared_mem_region_t *smem = &sc->shared_mem_regs[i];

        if(smem->type == handle->type){
            if(smem->is_physical){
                paddr = smem->paddr + offset;
                mem_attribute = VM_MEMATTR_UNCACHEABLE;
                device_printf(sc->dev, "Mapping physical addr: 0x%lx (base: 0x%lx + offset: 0x%lx)\n", 
                             paddr, smem->paddr, offset);
            }
            else{
                paddr = pmap_kextract(cheri_getaddress(smem->addr) + offset);
            }
            paddr_found = true;
            break;
        }
    }

    if(!paddr_found){
        return (VM_PAGER_BAD);
    }

    device_printf(sc->dev, "Creating page: paddr=0x%lx, offset=0x%lx\n", paddr, offset);
    

	/* See the end of old_dev_pager_fault in device_pager.c. */
	if (((*mres)->flags & (PG_FICTITIOUS)) != 0) {
		page = *mres;
        device_printf(sc->dev, "Updating existing fictitious page\n");
		vm_page_updatefake(page, paddr, mem_attribute);
	} else {
		VM_OBJECT_WUNLOCK(obj);
        device_printf(sc->dev, "Creating fictitious page with memattr=%d (UNCACHEABLE=%d, DEFAULT=%d)\n",
              mem_attribute, VM_MEMATTR_UNCACHEABLE, VM_MEMATTR_DEFAULT);
		page = vm_page_getfake(paddr, mem_attribute);
        device_printf(sc->dev, "vm_page_getfake returned: pa=0x%lx, memattr=%d\n", 
              VM_PAGE_TO_PHYS(page), page->md.pv_memattr);
		VM_OBJECT_WLOCK(obj);
		vm_page_replace(page, obj, (*mres)->pindex, *mres);
		*mres = page;
	}

    device_printf(sc->dev, "Page created: pa=0x%lx, valid=%d\n", 
                  VM_PAGE_TO_PHYS(page), vm_page_all_valid(page));


    device_printf(sc->dev, "After replace - page pa=0x%lx, pindex=%lu, flags=0x%x, valid=0x%x\n",
              VM_PAGE_TO_PHYS(page), (*mres)->pindex, page->flags, page->valid);

    vm_page_valid(page);

    device_printf(sc->dev, "After vm_page_valid - page pa=0x%lx, valid=0x%x\n",
              VM_PAGE_TO_PHYS(page), page->valid);
    
	return (VM_PAGER_OK);
}

static int
capio_ioctl_handler(struct cdev *dev, u_long cmd, caddr_t addr, int flags,
    struct thread *td){
    cap_req_t* cap_req = (cap_req_t*)addr;
    capio_softc_header_t *sc = dev->si_drv1;
    int error = 0;

    if(sc == NULL){
        return EINVAL;
    }

    if(cmd != CAPIO_ATTACH && check_cap_token(&sc->capio_sc, cap_req->sealed_cap)){
        return EPERM;
    }

    switch(cmd){
        case CAPIO_ATTACH:
            error = set_cap_token(&sc->capio_sc, cap_req);
            break;
        case CAPIO_GOODBYE:
            error = revoke_cap_token(&sc->capio_sc, td);
            break;
        default:
            error = sc->capio_sc.capio_ops->ioctl(dev, cmd, addr, flags, td);
            break;
    }

    return error;
}
