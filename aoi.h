#ifndef _AOI_H
#define _AOI_H

#include <stdint.h>
#include <stddef.h>

struct aoi_space;

typedef void * (*aoi_Alloc)(void *ud, void * ptr, size_t sz);
typedef void (message_handler)(struct aoi_space* space, void* userdata);

struct aoi_space * aoi_create(aoi_Alloc alloc, void *ud);
struct aoi_space * aoi_new();
void aoi_release(struct aoi_space *);
int aoi_gen_id(struct aoi_space *space);
void aoi_position(struct aoi_space *space, uint32_t id, float pos[3]);

// w(atcher) m(arker) d(rop)
void aoi_update(struct aoi_space * space , uint32_t id, const char * mode , float pos[3], float radius);
void aoi_message(struct aoi_space *space);

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

//_CREATE_OBJECT
//_DELETE_OBJECT
//_OBJECT_MOVED
typedef struct _aoi_object_callback_data {
	void* obj;
	float pos[3];
} aoi_create_object_data;

//_CREATE_PAIR
//_DELETE_PAIR
typedef struct _aoi_pair_callback_data {
	void* pair;
} aoi_create_pair_data;

//_FREE_AOI_SPACE
typedef struct _free_aoi_space_data {
	struct aoi_space* space;
} free_aoi_space_data;

void* aoi_get_user_data(struct aoi_space *space, char* data_id);
void* aoi_create_user_data(struct aoi_space *space, char* data_id, size_t sz);
void aoi_push_message_handler(struct aoi_space *space, char* message_id, message_handler* cb);
void aoi_pop_message_handler(struct aoi_space *space, char* message_id, message_handler* cb);
void aoi_fire_message(struct aoi_space *space, char* message_id, void* userdata);
char* aoi_current_message_id(struct aoi_space *space);

//neighbor
int aoi_begin_parse_neighbor(struct aoi_space *space, uint32_t id);
int aoi_next_neighbor(struct aoi_space *space, uint32_t* id);
int aoi_is_neighbor(struct aoi_space *space, uint32_t id);
int aoi_end_parse_neighbor(struct aoi_space *space);

#endif
