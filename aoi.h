#ifndef _AOI_H
#define _AOI_H

#include <stdint.h>
#include <stddef.h>

#define AOI_META_DATA "AOI_META_DATA"

#if defined(__cplusplus)
extern "C" {
#endif

//添加导出宏AOI_BUILD_AS_DLL， AOI_API
#if defined(_WIN32)
	#if defined(AOI_BUILD_AS_DLL)	/* { */
		#define AOI_API __declspec(dllexport)
	#else						/* }{ */
		#define AOI_API __declspec(dllimport)
	#endif						/* } */
#else
	#define AOI_API extern
#endif

struct aoi_space;

typedef void * (*aoi_Alloc)(void *ud, void * ptr, size_t sz);
typedef void (message_handler)(struct aoi_space* space, void* userdata);

AOI_API struct aoi_space * aoi_create(aoi_Alloc alloc, void *ud);
AOI_API struct aoi_space * aoi_new();
AOI_API void aoi_release(struct aoi_space *);
AOI_API int aoi_gen_id(struct aoi_space *space);

// w(atcher) m(arker) d(rop)
AOI_API void aoi_update(struct aoi_space * space , uint32_t id, const char * mode , float pos[3], float radius);
AOI_API void aoi_message(struct aoi_space *space);

//move
AOI_API void aoi_move_to(struct aoi_space *space, uint32_t id, float speed, float pos[3]);
AOI_API void aoi_set_speed(struct aoi_space *space, uint32_t id, float speed);
AOI_API void aoi_apply_move(struct aoi_space *space, float delta);
AOI_API void aoi_cancel_move(struct aoi_space *space, uint32_t id);

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
	float radius;
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

AOI_API void* aoi_get_user_data(struct aoi_space *space, const char* data_id);
AOI_API void* aoi_create_user_data(struct aoi_space *space, const char* data_id, size_t sz);
AOI_API void aoi_push_message_handler(struct aoi_space *space, const char* message_id, message_handler* cb);
AOI_API void aoi_pop_message_handler(struct aoi_space *space, const char* message_id, message_handler* cb);
AOI_API void aoi_fire_message(struct aoi_space *space, const char* message_id, void* userdata);
AOI_API const char* aoi_current_message_id(struct aoi_space *space);

//neighbor
AOI_API int aoi_begin_parse_neighbor(struct aoi_space *space, uint32_t id);
AOI_API int aoi_next_neighbor(struct aoi_space *space, uint32_t* id);
AOI_API int aoi_end_parse_neighbor(struct aoi_space *space);
AOI_API int aoi_is_pair(struct aoi_space *space, uint32_t id, uint32_t tid);

#if defined(__cplusplus)
}
#endif

#endif
