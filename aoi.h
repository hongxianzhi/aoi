#ifndef _AOI_H
#define _AOI_H

#include <stdint.h>
#include <stddef.h>

typedef void * (*aoi_Alloc)(void *ud, void * ptr, size_t sz);
typedef void (aoi_Callback)(void *ud, uint32_t watcher, uint32_t marker);

struct aoi_space;

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

#endif
