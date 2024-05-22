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
//预定义事件类型、数据结构
#define USER_HANDLER_TYPE_UPDATE 0
typedef struct _update_callback_data {
	float delat;
} update_callback_data;

#define USER_HANDLER_TYPE_AOI_CREATE_OBJECT 1
#define USER_HANDLER_TYPE_AOI_DELETE_OBJECT 2
#define USER_HANDLER_TYPE_AOI_MOVED 3
typedef struct _aoi_object_callback_data {
	uint32_t id;
	float pos[3];
} aoi_create_object_data;

#define USER_HANDLER_TYPE_AOI_GEN_PAIR 4
#define USER_HANDLER_TYPE_AOI_DROP_PAIR 5
typedef struct _aoi_pair_callback_data {
	uint32_t watcher;
	uint32_t marker;
} aoi_create_pair_data;

#define USER_HANDLER_TYPE_FREE_AOI_SPACE 6
typedef struct _free_aoi_space_data {
	struct aoi_space* space;
} free_aoi_space_data;

void aoi_add_handler_host(struct aoi_space *space, uint32_t hostid);
void* aoi_alloc_handler_host_data(struct aoi_space *space, uint32_t hostid, size_t sz);
void* aoi_get_handler_hsot_data(struct aoi_space *space, uint32_t hostid);
void aoi_del_user_handler_host(struct aoi_space *space, uint32_t hostid);
void aoi_set_user_callback(struct aoi_space *space, uint32_t hostid, int type, user_callback* cb);
void aoi_fire_user_callback(struct aoi_space *space, int type, void* userdata);

#endif
