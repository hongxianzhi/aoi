#ifndef _AOI_H
#define _AOI_H

#include <stdint.h>
#include <stddef.h>

struct aoi_space;

typedef void * (*aoi_Alloc)(void *ud, void * ptr, size_t sz);
typedef void (aoi_Callback)(void *ud, uint32_t watcher, uint32_t marker);
typedef void (user_callback)(struct aoi_space* space, void* handlers_data, void* userdata);

struct aoi_space * aoi_create(aoi_Alloc alloc, void *ud);
struct aoi_space * aoi_new();
void aoi_release(struct aoi_space *);
int aoi_gen_id(struct aoi_space *space);
void aoi_position(struct aoi_space *space, uint32_t id, float pos[3]);

// w(atcher) m(arker) d(rop)
void aoi_update(struct aoi_space * space , uint32_t id, const char * mode , float pos[3]);
void aoi_message(struct aoi_space *space, aoi_Callback cb, void *ud);

//move
void aoi_move_to(struct aoi_space *space, uint32_t id, float speed, float pos[3]);
void aoi_set_speed(struct aoi_space *space, uint32_t id, float speed);
void aoi_apply_move(struct aoi_space *space, float delta);
void aoi_cancel_move(struct aoi_space *space, uint32_t id);

//user callback
void* aoi_add_user_handlers(struct aoi_space *space);
void* aoi_alloc_handlers_data(struct aoi_space *space, void* handlers, size_t sz);
void* aoi_get_handlers_data(struct aoi_space *space, void* handlers);
void aoi_del_user_handles(struct aoi_space *space, void* handlers);
void aoi_set_user_callback(struct aoi_space *space, void* handlers, int type, user_callback* cb);
void aoi_fire_user_callback(struct aoi_space *space, int type, void* userdata);

#endif
