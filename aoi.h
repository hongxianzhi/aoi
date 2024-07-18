#ifndef _AOI_H
#define _AOI_H

#include <stdint.h>
#include <stddef.h>

#define MODE_WATCHER 1
#define MODE_MARKER 2
#define MODE_MOVE 4
#define MODE_DROP 8

#define AOI_META_DATA "AOI_META_DATA"
#define ID_FREE_AOI_SPACE "_FREE_AOI_SPACE"
#define ID_CREATE_OBJECT "_CREATE_OBJECT"
#define ID_DELETE_OBJECT "_DELETE_OBJECT"
#define ID_OBJECT_MOVED "_OBJECT_MOVED"
#define ID_CREATE_PAIR "_CREATE_PAIR"
#define ID_DELETE_PAIR "_DELETE_PAIR"
#define ID_NEIGHBOR_ENTER "_NEIGHBOR_ENTER"
#define ID_NEIGHBOR_LEAVE "_NEIGHBOR_LEAVE"
#define ID_UPDATE "_UPDATE"

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
AOI_API struct aoi_space * aoi_new(int w, int h, float f);
AOI_API void aoi_release(struct aoi_space *);
AOI_API int aoi_gen_id(struct aoi_space *space);
AOI_API int aoi_make_grid_id(struct aoi_space *space, int x, int y);
AOI_API int aoi_break_grid_id(struct aoi_space *space, int id, int *x, int *y);
AOI_API void aoi_get_size(struct aoi_space *space, int *w, int *h, float *f);
AOI_API int aoi_begin_parse_grid(struct aoi_space *space, int grid);
AOI_API int aoi_next_object(struct aoi_space *space, uint32_t* id);
AOI_API int aoi_end_parse_grid(struct aoi_space *space);

AOI_API void aoi_insert(struct aoi_space* space, uint32_t id, uint64_t mask, float pos[3], float radius);
AOI_API int aoi_erase(struct aoi_space* space, uint32_t id);
AOI_API void aoi_radius(struct aoi_space* space, uint32_t id, float r);
AOI_API void aoi_location(struct aoi_space* space, uint32_t id, float pos[3]);
AOI_API void aoi_message(struct aoi_space *space);

//move
AOI_API void aoi_move_to(struct aoi_space *space, uint32_t id, float speed, float pos[3]);
AOI_API void aoi_set_speed(struct aoi_space *space, uint32_t id, float speed);
AOI_API void aoi_apply_move(struct aoi_space *space, float delta);
AOI_API void aoi_cancel_move(struct aoi_space *space, uint32_t id);

//user callback
//预定义事件类型、数据结构
#define USER_HANDLER_TYPE_UPDATE 0
typedef struct update_callback_data {
	float delat;
} update_callback_data;

//_CREATE_OBJECT
//_DELETE_OBJECT
typedef struct aoi_object_data {
	int id;
	float last[3];
	float radius;
} aoi_object_data;

//_OBJECT_MOVED
typedef struct object_moved_data {
	int id;
	float last[3];
	float prev[3];
	float radius;
} object_moved_data;

//_CREATE_PAIR
//_DELETE_PAIR
typedef struct aoi_pair_data {
	void* pair;
	uint64_t id;
	float dis2;
	int watcher;
	int marker;
} aoi_pair_data;

AOI_API void* aoi_get_user_data(struct aoi_space *space, const char* data_id);
AOI_API void* aoi_create_user_data(struct aoi_space *space, const char* data_id, size_t sz);
AOI_API void aoi_push_message_handler(struct aoi_space *space, const char* message_id, message_handler* cb);
AOI_API void aoi_pop_message_handler(struct aoi_space *space, const char* message_id, message_handler* cb);
AOI_API void aoi_fire_message(struct aoi_space *space, const char* message_id, void* userdata);
AOI_API const char* aoi_current_message_id(struct aoi_space *space);

AOI_API int aoi_get_object_position(struct aoi_space *space, uint32_t id, float* pos, int* mode);

//neighbor
AOI_API int aoi_has_neighbor(struct aoi_space *space, uint32_t id, int mask);
AOI_API int aoi_begin_parse_neighbor(struct aoi_space *space, uint32_t id);
AOI_API int aoi_next_neighbor(struct aoi_space *space, uint32_t* id);
AOI_API int aoi_end_parse_neighbor(struct aoi_space *space);
AOI_API int aoi_is_pair(struct aoi_space *space, uint32_t id, uint32_t tid);

AOI_API char* str_dup(const char* str);
AOI_API int str_eq(const char *a, const char *b);

#if defined(__cplusplus)
}
#endif

#endif
