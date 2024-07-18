#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <stdlib.h>
#include "aoi.h"
#include "uthash.h"

#define AOI_RADIS 10.0f
#define TOO_NEAR_DIST2 0.5f

#define INVALID_ID (~0)
#define PRE_ALLOC 16
#define MODE_WATCHER_MASK (((uint64_t)1) << 32)
#define MODE_MARKER_MASK 0xFFFFFFFF
#define AOI_RADIS2 (AOI_RADIS * AOI_RADIS)
#define DIST2(p1,p2) ((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]))

struct pair_list;
static void drop_pair(struct aoi_space * space, struct pair_list *p);
static void free_aoi_space_callback(struct aoi_space *space, void* userdata);
static void drop_object_callback(struct aoi_space* space, void* userdata);
static void gen_pair_callback(struct aoi_space* space, void* userdata);
static void drop_pair_callback(struct aoi_space* space, void* userdata);
static void set_push_back(struct aoi_space* space, struct object_set* set, struct object* obj);

struct neighbor_list{
	void* pair;
	struct neighbor_list* next;
	struct neighbor_list* prev;
};

struct object {
	int ref;
	uint32_t id;
	uint64_t mask;
	int mode;
	int neighbor_mask;
	float last[3];
	float position[3];
	float radius;
	struct neighbor_list * neighbors;
};

struct object_set {
	int cap;
	int number;
	struct object ** slot;
};

struct pair_list {
	uint64_t id;
	float dis2;
	struct object * watcher;
	struct object * marker;
	UT_hash_handle hh;
};
typedef void (pair_parser)(void* ud, struct pair_list* pair);

struct map_slot {
	uint32_t id;
	struct object * obj;
	int next;
};

struct map {
	int size;
	int lastfree;
	struct map_slot * slot;
};

struct obj_moved {
	uint32_t id;
	float speed;
	float pos[3];
	float vec[3];
	float distance;
	struct obj_moved * next;
	struct obj_moved * prev;
};

struct data_link_list {
	void* data;
	struct data_link_list *next;
	struct data_link_list *prev;
};

struct user_data_dict {
	char* id;
	void* data;
	UT_hash_handle hh;
};

struct free_ids
{
	uint32_t* ids;
	int count;
	int capacity;
};

struct grid_objects
{
	int grid;
	struct object_set* objects;
	UT_hash_handle hh;
};

struct aoi_space {
	int w;
	int h;
	float f;
	aoi_Alloc alloc;
	void * alloc_ud;
	struct map * object;
	struct object_set * watcher_static;
	struct object_set * marker_static;
	struct object_set * watcher_move;
	struct object_set * marker_move;
	struct pair_list * hot;
	struct obj_moved * moved;

	//parse neighbor
	struct object * neighbor_host;
	struct neighbor_list * neighbor;
	struct object* object_current;

	struct free_ids * frees;
	uint32_t id_begin;

	struct user_data_dict* user_datas;
	const char* current_message_id;
	struct user_data_dict* message_handlers;

	//grid objects
	struct grid_objects* grid_objects;
	struct grid_objects* grid_objects_parsing;
	int grid_objects_parsing_index;
};

//快速比较两个字符串是否相等
int str_eq(const char *a, const char *b) {
	if(a != b && (a == NULL || b == NULL))
	{
		return 0;
	}
	
	while (*a && *b) {
		if (*a != *b) {
			return 0;
		}
		++a;
		++b;
	}
	return *a == 0 && *b == 0;
}

//拷贝字符串
char* str_dup(const char* str)
{
	if(str == NULL)
	{
		return NULL;
	}
	int len = strlen(str);
	char* dup = malloc(len + 1);
	memcpy(dup, str, len);
	dup[len] = '\0';
	return dup;
}

inline void parse_neighbors(struct object* obj, pair_parser cb, void* ud)
{
	if (obj == NULL || obj->neighbors == NULL)
	{
		return;
	}

	int neighbor_mask = 0;
	struct neighbor_list* p = obj->neighbors;
	while (p)
	{
		struct neighbor_list* next = p->next;
		struct pair_list* pair = p->pair;
		if (cb)
		{
			cb(ud, pair);
		}

		if (pair->watcher == obj)
		{
			neighbor_mask |= (pair->marker->mask & 0xFFFFFFFF);
		}
		else
		{
			neighbor_mask |= (pair->watcher->mask & 0xFFFFFFFF);
		}
		p = next;
	}
	obj->neighbor_mask = neighbor_mask;
}

inline static struct user_data_dict*
_get_user_data(struct user_data_dict* dict, const char* data_id)
{
	struct user_data_dict* result = NULL;
	HASH_FIND_STR(dict, data_id, result);
	return result;
}

inline static void 
copy_position(float des[3], float src[3]) {
	des[0] = src[0];
	des[1] = src[1];
	des[2] = src[2];
}

static inline void
fire_object_message(struct aoi_space *space, struct object* obj, char* message_id)
{
	struct aoi_object_data data;
	data.id = obj->id;
	data.radius = obj->radius;
	copy_position(data.last, obj->last);
	aoi_fire_message(space, message_id, &data);
}

static inline void
fire_pair_message(struct aoi_space *space, struct pair_list* p, char* message_id)
{
	struct aoi_pair_data data;
	data.pair = p;
	data.id = p->id;
	data.dis2 = p->dis2;
	data.watcher = p->watcher->id;
	data.marker = p->marker->id;
	aoi_fire_message(space, message_id, &data);
}

inline static uint64_t
gen_key(uint32_t watcher, uint32_t marker) {
	uint32_t min = watcher > marker ? marker : watcher;
	uint32_t max = watcher > marker ? watcher : marker;
	return ((uint64_t)min << 32) | max;
}

inline static void
grab_neighbor(struct aoi_space *space, struct object* obj, void* pair)
{
	struct neighbor_list* neighbor = space->alloc(space->alloc_ud, NULL, sizeof(*neighbor));
	neighbor->pair = pair;
	neighbor->next = obj->neighbors;
	neighbor->prev = NULL;
	if(obj->neighbors)
	{
		obj->neighbors->prev = neighbor;
	}
	obj->neighbors = neighbor;
}

inline static void
drop_neighbor(struct aoi_space *space, struct object* obj, void* pair)
{
	struct neighbor_list* neighbor = obj->neighbors;
	while(neighbor)
	{
		if(neighbor->pair == pair)
		{
			break;
		}
		neighbor = neighbor->next;
	}

	if(neighbor)
	{
		if(neighbor->prev)
		{
			neighbor->prev->next = neighbor->next;
		}
		else
		{
			obj->neighbors = neighbor->next;
		}

		if(neighbor->next)
		{
			neighbor->next->prev = neighbor->prev;
		}

		space->alloc(space->alloc_ud, neighbor, sizeof(*neighbor));
	}
}

static struct object *
new_object(struct aoi_space * space, uint32_t id) {
	struct object * obj = space->alloc(space->alloc_ud, NULL, sizeof(*obj));
	memset(obj, 0, sizeof(*obj));
	obj->ref = 1;
	obj->id = id;
	return obj;
}

static inline struct map_slot *
mainposition(struct map *m , uint32_t id) {
	uint32_t hash = id & (m->size-1);
	return &m->slot[hash];
}

static void rehash(struct aoi_space * space, struct map *m);

static void
map_insert(struct aoi_space * space , struct map * m, uint32_t id , struct object *obj) {
	struct map_slot *s = mainposition(m,id);
	if (s->id == INVALID_ID) {
		s->id = id;
		s->obj = obj;
		return;
	}
	if (mainposition(m, s->id) != s) {
		struct map_slot * last = mainposition(m,s->id);
		while (last->next != s - m->slot) {
			assert(last->next >= 0);
			last = &m->slot[last->next];
		}
		uint32_t temp_id = s->id;
		struct object * temp_obj = s->obj;
		last->next = s->next;
		s->id = id;
		s->obj = obj;
		s->next = -1;
		if (temp_obj) {
			map_insert(space, m, temp_id, temp_obj);
		}
		return;
	}
	while (m->lastfree >= 0) {
		struct map_slot * temp = &m->slot[m->lastfree--];
		if (temp->id == INVALID_ID) {
			temp->id = id;
			temp->obj = obj;
			temp->next = s->next;
			s->next = (int)(temp - m->slot);
			return;
		}
	}
	rehash(space,m);
	map_insert(space, m, id , obj);
}

static void
rehash(struct aoi_space * space, struct map *m) {
	struct map_slot * old_slot = m->slot;
	int old_size = m->size;
	m->size = 2 * old_size;
	m->lastfree = m->size - 1;
	m->slot = space->alloc(space->alloc_ud, NULL, m->size * sizeof(struct map_slot));
	int i;
	for (i=0;i<m->size;i++) {
		struct map_slot * s = &m->slot[i];
		s->id = INVALID_ID;
		s->obj = NULL;
		s->next = -1;
	}
	for (i=0;i<old_size;i++) {
		struct map_slot * s = &old_slot[i];
		if (s->obj) {
			map_insert(space, m, s->id, s->obj);
		}
	}
	space->alloc(space->alloc_ud, old_slot, old_size * sizeof(struct map_slot));
}

static struct object *
map_query(struct aoi_space *space, struct map * m, uint32_t id, int create) {
	struct map_slot *s = mainposition(m, id);
	for (;;) {
		if (s->id == id) {
			if (s->obj == NULL && create) {
				s->obj = new_object(space, id);
			}
			return s->obj;
		}
		if (s->next < 0) {
			break;
		}
		s=&m->slot[s->next];
	}

	struct object * obj = NULL;
	if(create)
	{
		obj = new_object(space, id);
		map_insert(space, m , id , obj);
	}
	return obj;
}

static void
map_foreach(struct map * m , void (*func)(void *ud, struct object *obj), void *ud) {
	int i;
	for (i=0;i<m->size;i++) {
		if (m->slot[i].obj) {
			func(ud, m->slot[i].obj);
		}
	}
}

static struct object *
map_drop(struct map *m, uint32_t id) {
	uint32_t hash = id & (m->size-1);
	struct map_slot *s = &m->slot[hash];
	for (;;) {
		if (s->id == id) {
			struct object * obj = s->obj;
			s->obj = NULL;
			return obj;
		}
		if (s->next < 0) {
			return NULL;
		}
		s=&m->slot[s->next];
	}
}

static void
map_delete(struct aoi_space *space, struct map * m) {
	space->alloc(space->alloc_ud, m->slot, m->size * sizeof(struct map_slot));
	space->alloc(space->alloc_ud, m , sizeof(*m));
}

static struct map *
map_new(struct aoi_space *space) {
	int i;
	struct map * m = space->alloc(space->alloc_ud, NULL, sizeof(*m));
	m->size = PRE_ALLOC;
	m->lastfree = PRE_ALLOC - 1;
	m->slot = space->alloc(space->alloc_ud, NULL, m->size * sizeof(struct map_slot));
	for (i=0;i<m->size;i++) {
		struct map_slot * s = &m->slot[i];
		s->id = INVALID_ID;
		s->obj = NULL;
		s->next = -1;
	}
	return m;
}

inline static void
grab_object(struct object *obj) {
	++obj->ref;
}

static void
delete_object(void *s, struct object * obj) {
	struct aoi_space * space = s;
	//释放邻居列表
	struct neighbor_list* neighbor = obj->neighbors;
	while(neighbor)
	{
		struct neighbor_list* next = neighbor->next;
		space->alloc(space->alloc_ud, neighbor, sizeof(*neighbor));
		neighbor = next;
	}
	obj->neighbors = NULL;
	space->alloc(space->alloc_ud, obj, sizeof(*obj));
	if(space->object_current == obj)
	{
		space->object_current = NULL;
	}
}

inline static void
drop_object(struct aoi_space * space, struct object *obj) {
	--obj->ref;
	if (obj->ref <=0) {
		//释放邻居列表
		uint32_t ids[2] = {0, 0};
		struct neighbor_list* neighbor = obj->neighbors;
		while(neighbor)
		{
			struct pair_list* pair = (struct pair_list*)neighbor->pair;
			ids[0] = pair->watcher->id;
			ids[1] = pair->marker->id;
			aoi_fire_message(space, ID_NEIGHBOR_LEAVE, ids);

			struct neighbor_list* next = neighbor->next;
			space->alloc(space->alloc_ud, neighbor, sizeof(*neighbor));
			neighbor = next;
		}
		obj->neighbors = NULL;

		fire_object_message(space, obj, ID_DELETE_OBJECT);
		map_drop(space->object, obj->id);
		delete_object(space, obj);
	}
}

static struct object_set *
set_new(struct aoi_space * space) {
	struct object_set * set = space->alloc(space->alloc_ud, NULL, sizeof(*set));
	set->cap = PRE_ALLOC;
	set->number = 0;
	set->slot = space->alloc(space->alloc_ud, NULL, set->cap * sizeof(struct object *));
	return set;
}

struct aoi_space * 
aoi_create(aoi_Alloc alloc, void *ud) {
	struct aoi_space *space = alloc(ud, NULL, sizeof(*space));
	memset(space, 0, sizeof(*space));
	space->alloc = alloc;
	space->alloc_ud = ud;
	space->object = map_new(space);
	space->watcher_static = set_new(space);
	space->marker_static = set_new(space);
	space->watcher_move = set_new(space);
	space->marker_move = set_new(space);
	space->id_begin = 1;
	space->frees = alloc(ud, NULL, sizeof(*space->frees));
	memset(space->frees, 0, sizeof(*space->frees));
	space->frees->capacity = PRE_ALLOC;
	space->frees->ids = alloc(ud, NULL, sizeof(uint32_t) * space->frees->capacity);

	aoi_push_message_handler(space, ID_FREE_AOI_SPACE, free_aoi_space_callback);
	//添加对象的回调
	aoi_push_message_handler(space, ID_DELETE_OBJECT, drop_object_callback);
	//添加邻居列表的回调
	aoi_push_message_handler(space, ID_CREATE_PAIR, gen_pair_callback);
	aoi_push_message_handler(space, ID_DELETE_PAIR, drop_pair_callback);

	return space;
}

static void
delete_pair_list(struct aoi_space * space) {
	struct pair_list * p = NULL;
	struct pair_list * tmp = NULL;
	HASH_ITER(hh, space->hot, p, tmp) {
		drop_pair(space, p);
	}
}

static void
delete_set(struct aoi_space *space, struct object_set * set) {
	if (set->slot) {
		space->alloc(space->alloc_ud, set->slot, sizeof(struct object *) * set->cap);
	}
	space->alloc(space->alloc_ud, set, sizeof(*set));
}

void 
aoi_release(struct aoi_space *space) {
	aoi_fire_message(space, ID_FREE_AOI_SPACE, NULL);

	map_foreach(space->object, delete_object, space);
	map_delete(space, space->object);
	delete_pair_list(space);
	delete_set(space,space->watcher_static);
	delete_set(space,space->marker_static);
	delete_set(space,space->watcher_move);
	delete_set(space,space->marker_move);

	//释放moved链表
	struct obj_moved* moved = space->moved;
	space->moved = NULL;
	while(moved)
	{
		struct obj_moved* next = moved->next;
		space->alloc(space->alloc_ud, moved, sizeof(*moved));
		moved = next;
	}

	//释放ids数据
	struct free_ids *frees = space->frees;
	if(frees->ids)
	{
		space->alloc(space->alloc_ud, frees->ids, sizeof(uint32_t) * frees->capacity);
	}
	space->alloc(space->alloc_ud, frees, sizeof(*frees));

	//释放user_datas字典
	struct user_data_dict * p = NULL;
	struct user_data_dict * tmp = NULL;
	HASH_ITER(hh, space->user_datas, p, tmp) {
		HASH_DEL(space->user_datas, p);
		free(p->id);
		if(p->data)
		{
			space->alloc(space->alloc_ud, p->data, sizeof(p->data));
		}
		space->alloc(space->alloc_ud, p, sizeof(*p));
	}

	//释放message_handlers字典
	HASH_ITER(hh, space->message_handlers, p, tmp) {
		HASH_DEL(space->message_handlers, p);
		free(p->id);
		//释放handler链表
		struct data_link_list* handler = (struct data_link_list*)p->data;
		while(handler)
		{
			struct data_link_list* next = handler->next;
			space->alloc(space->alloc_ud, handler, sizeof(*handler));
			handler = next;
		}
		space->alloc(space->alloc_ud, p, sizeof(*p));
	}

	//释放 grid_objects
	struct grid_objects* grid_obj = NULL;
	struct grid_objects* tmp_grid_obj = NULL;
	HASH_ITER(hh, space->grid_objects, grid_obj, tmp_grid_obj) {
		HASH_DEL(space->grid_objects, grid_obj);
		delete_set(space, grid_obj->objects);
		space->alloc(space->alloc_ud, grid_obj, sizeof(*grid_obj));
	}

	space->alloc(space->alloc_ud, space, sizeof(*space));
}

int aoi_gen_id(struct aoi_space *space)
{
	while (true)
	{
		int result = 0;
		if(space->frees == NULL || space->frees->count == 0)
		{
			result = space->id_begin++;
		}
		else
		{
			result = space->frees->ids[--space->frees->count];
		}
		assert(result >= 0);
		if(map_query(space, space->object, result, 0) == NULL)
		{
			return result;
		}
	}
}

int aoi_make_grid_id(struct aoi_space *space, int x, int y)
{
	if(x < 0 || x >= space->w || y < 0 || y >= space->h)
	{
		return -1;
	}
	return x * space->h + y;
}

int aoi_break_grid_id(struct aoi_space *space, int id, int *x, int *y)
{
	if(id < 0 || id >= space->w * space->h)
	{
		return 0;
	}
	if(x)
	{
		*x = id / space->h;
	}
	if(y)
	{
		*y = id % space->h;
	}
	return 1;
}

void aoi_get_size(struct aoi_space *space, int *w, int *h, float *f)
{
	if(w)
	{
		*w = space->w;
	}
	if(h)
	{
		*h = space->h;
	}
	if(f)
	{
		*f = space->f;
	}
}

int aoi_begin_parse_grid(struct aoi_space *space, int grid)
{
	space->grid_objects_parsing = NULL;
	space->grid_objects_parsing_index = 0;
	if(grid < 0 || grid >= space->w * space->h)
	{
		return 0;
	}
	struct grid_objects* grid_obj = NULL;
	HASH_FIND_INT(space->grid_objects, &grid, grid_obj);
	if(grid_obj == NULL || grid_obj->objects->number <= 0)
	{
		return 0;
	}
	space->grid_objects_parsing = grid_obj;
	return 1;
}

int aoi_next_object(struct aoi_space *space, uint32_t* id)
{
	if(space->grid_objects_parsing == NULL)
	{
		return 0;
	}
	if(space->grid_objects_parsing_index >= space->grid_objects_parsing->objects->number)
	{
		return 0;
	}
	space->object_current = space->grid_objects_parsing->objects->slot[space->grid_objects_parsing_index++];
	*id = space->object_current->id;
	return 1;
}

int aoi_end_parse_grid(struct aoi_space *space)
{
	space->grid_objects_parsing = NULL;
	return 1;
}

static bool
change_mode(struct object * obj, bool set_watcher, bool set_marker) {
	bool change = false;
	if (obj->mode == 0) {
		if (set_watcher) {
			obj->mode = MODE_WATCHER;
		}
		if (set_marker) {
			obj->mode |= MODE_MARKER;
		}
		return true;
	}
	if (set_watcher) {
		if (!(obj->mode & MODE_WATCHER)) {
			obj->mode |= MODE_WATCHER;
			change = true;
		}
	} else {
		if (obj->mode & MODE_WATCHER) {
			obj->mode &= ~MODE_WATCHER;
			change = true;
		}
	}
	if (set_marker) {
		if (!(obj->mode & MODE_MARKER)) {
			obj->mode |= MODE_MARKER;
			change = true;
		}
	} else {
		if (obj->mode & MODE_MARKER) {
			obj->mode &= ~MODE_MARKER;
			change = true;
		}
	}
	return change;
}

inline static bool
is_near(float p1[2], float p2[2])
{
	return DIST2(p1,p2) < TOO_NEAR_DIST2;
}

inline static float fdis(float f1, float f2, float f3)
{
	float d = f1 - f2;
	if (d < 0)
	{
		d = -d;
	}
	d = d - f3;
	return d > 0 ? d : 0;
}

inline static float calc_dist2(float x, float y, float r, float x1, float y1, float r1)
{
	float radius = r + r1;
	float disx = fdis(x, x1, radius);
	float disy = fdis(y, y1, radius);
	return (disx * disx) + (disy * disy);
}

inline static float
dist2(struct object *p1, struct object *p2) {
	return calc_dist2(p1->position[0], p1->position[1], p1->radius, p2->position[0], p2->position[1], p2->radius);
}

inline static void
flush_move_sign(struct aoi_space *space, struct object * obj)
{
	if(obj == NULL || (obj->mode & MODE_DROP))
	{
		return;
	}

	if(obj->last[0] == obj->position[0] && obj->last[1] == obj->position[1] && obj->last[2] == obj->position[2])
	{
		return;
	}

	//维护 grid_objects
	int grid_last = aoi_make_grid_id(space, (int)obj->last[0], (int)obj->last[1]);
	int grid = aoi_make_grid_id(space, (int)obj->position[0], (int)obj->position[1]);
	if(grid_last != grid)
	{
		struct grid_objects* grid_obj = NULL;
		HASH_FIND_INT(space->grid_objects, &grid_last, grid_obj);
		if(grid_obj)
		{
			//从旧的grid中删除
			int i;
			struct object_set* objects = grid_obj->objects;
			for(i = 0; i < objects->number; ++i)
			{
				if(objects->slot[i] == obj)
				{
					break;
				}
			}
			
			for(; i < objects->number - 1; ++i)
			{
				objects->slot[i] = objects->slot[i + 1];
			}
			--objects->number;
			
			if(objects->number == 0)
			{
				HASH_DEL(space->grid_objects, grid_obj);
				delete_set(space, objects);
				space->alloc(space->alloc_ud, grid_obj, sizeof(*grid_obj));
			}
		}

		grid_obj = NULL;
		HASH_FIND_INT(space->grid_objects, &grid, grid_obj);
		if(grid_obj == NULL)
		{
			grid_obj = space->alloc(space->alloc_ud, NULL, sizeof(*grid_obj));
			grid_obj->grid = grid;
			grid_obj->objects = set_new(space);
			HASH_ADD_INT(space->grid_objects, grid, grid_obj);
		}
		set_push_back(space, grid_obj->objects, obj);
	}

	obj->mode |= MODE_MOVE;

	object_moved_data data;
	data.id = obj->id;
	data.radius = obj->radius;
	copy_position(data.prev, obj->last);
	copy_position(obj->last, obj->position);
	copy_position(data.last, obj->last);
	aoi_fire_message(space, ID_OBJECT_MOVED, &data);
}

inline static void
set_position(struct aoi_space *space, struct object * obj, float pos[3])
{
	if(obj == NULL || (obj->mode & MODE_DROP))
	{
		return;
	}
	copy_position(obj->position, pos);
	if(is_near(pos, obj->last))
	{
		return;
	}
	flush_move_sign(space, obj);
}

void aoi_insert(struct aoi_space* space, uint32_t id, uint64_t mask, float pos[3], float radius)
{
	int is_new_obj = 0;
	struct object* obj = map_query(space, space->object, id, 0);
	if(obj == NULL)
	{
		is_new_obj = 1;
		obj = map_query(space, space->object, id, 1);
	}

	bool set_watcher = ((mask & MODE_WATCHER_MASK) != 0);
	bool set_marker = ((mask & MODE_MARKER_MASK) != 0);
	if (obj->mask != mask)
	{
		obj->mask = mask;
	}

	if (obj->mode & MODE_DROP)
	{
		obj->mode &= ~MODE_DROP;
		grab_object(obj);
	}

	bool changed = change_mode(obj, set_watcher, set_marker);
	obj->radius = radius >= 0 ? radius : 0;
	if(is_new_obj)
	{
		copy_position(obj->last, pos);
		//维护 grid_objects
		int grid = aoi_make_grid_id(space, (int)pos[0], (int)pos[1]);
		struct grid_objects* grid_obj = NULL;
		HASH_FIND_INT(space->grid_objects, &grid, grid_obj);
		if(grid_obj == NULL)
		{
			grid_obj = space->alloc(space->alloc_ud, NULL, sizeof(*grid_obj));
			grid_obj->grid = grid;
			grid_obj->objects = set_new(space);
			HASH_ADD_INT(space->grid_objects, grid, grid_obj);
		}
		set_push_back(space, grid_obj->objects, obj);

		fire_object_message(space, obj, ID_CREATE_OBJECT);
	}
	else
	{
		set_position(space, obj, pos);
	}
}

int aoi_erase(struct aoi_space* space, uint32_t id)
{
	struct object* obj = map_query(space, space->object, id, 0);
	if(obj == NULL)
	{
		return 0;
	}

	if(obj->mode & MODE_DROP)
	{
		return 0;
	}

	obj->mode |= MODE_DROP;
	drop_object(space, obj);
	return 1;
}

void aoi_radius(struct aoi_space* space, uint32_t id, float r)
{
	struct object* obj = map_query(space, space->object, id, 0);
	if(obj == NULL || (obj->mode & MODE_DROP) || obj->radius == r)
	{
		return;
	}

	obj->radius = r;
	if(obj->neighbors)
	{
		//重新计算邻居
		obj->mode |= MODE_MOVE;
	}
}

void aoi_location(struct aoi_space* space, uint32_t id, float pos[3])
{
	struct object* obj = map_query(space, space->object, id, 0);
	if(obj == NULL || (obj->mode & MODE_DROP))
	{
		return;
	}

	set_position(space, obj, pos);
}

static void
drop_pair(struct aoi_space * space, struct pair_list *p) {
	drop_object(space, p->watcher);
	drop_object(space, p->marker);
	HASH_DEL(space->hot, p);
	space->alloc(space->alloc_ud, p, sizeof(*p));
}

static void
flush_pair(struct aoi_space * space) {
	struct pair_list * p = NULL;
	struct pair_list * tmp = NULL;
	HASH_ITER(hh, space->hot, p, tmp) {
		struct object* marker = p->marker;
		struct object* watcher = p->watcher;
		if ((watcher->mode & MODE_DROP) == MODE_DROP || (marker->mode & MODE_DROP) == MODE_DROP)
		{
			fire_pair_message(space, p, ID_DELETE_PAIR);
			drop_pair(space, p);
		}
		else if ((watcher->mode & MODE_MOVE) == MODE_MOVE || (marker->mode & MODE_MOVE) == MODE_MOVE)
		{
			p->dis2 = dist2(p->watcher , p->marker);
			if (p->dis2 > AOI_RADIS2 * 1.2f)
			{
				fire_pair_message(space, p, ID_DELETE_PAIR);
				drop_pair(space, p);
			}
		}
	}
}

static void set_push_back(struct aoi_space * space, struct object_set * set, struct object *obj)
{
	if (set->number >= set->cap) {
		int cap = set->cap * 2;
		void * tmp =  set->slot;
		set->slot = space->alloc(space->alloc_ud, NULL, cap * sizeof(struct object *));
		memcpy(set->slot, tmp ,  set->cap * sizeof(struct object *));
		space->alloc(space->alloc_ud, tmp, set->cap * sizeof(struct object *));
		set->cap = cap;
	}
	set->slot[set->number] = obj;
	++set->number;
}

static void
set_push(void * s, struct object * obj) {
	struct aoi_space * space = s;
	int mode = obj->mode;
	if (mode & MODE_WATCHER) {
		if (mode & MODE_MOVE) {
			set_push_back(space, space->watcher_move , obj);
			obj->mode &= ~MODE_MOVE;
		} else {
			set_push_back(space, space->watcher_static , obj);
		}
	} 
	if (mode & MODE_MARKER) {
		if (mode & MODE_MOVE) {
			set_push_back(space, space->marker_move , obj);
			obj->mode &= ~MODE_MOVE;
		} else {
			set_push_back(space, space->marker_static , obj);
		}
	}
}

static void
gen_pair(struct aoi_space * space, struct object * watcher, struct object * marker) {
	if (watcher == marker) {
		return;
	}

	struct pair_list * p = NULL;
	uint64_t key = gen_key(watcher->id, marker->id);
	HASH_FIND(hh, space->hot, &key, sizeof(key), p);
	if (p)
	{
		return;
	}

	float dis2 = dist2(watcher, marker);
	if(dis2 > AOI_RADIS2)
	{
		return;
	}

	p = space->alloc(space->alloc_ud, NULL, sizeof(*p));
	p->watcher = watcher;
	grab_object(watcher);
	p->marker = marker;
	grab_object(marker);
	p->dis2 = dis2;
	p->id = key;
	HASH_ADD(hh, space->hot, id, sizeof(p->id), p);

	fire_pair_message(space, p, ID_CREATE_PAIR);
}

static void
gen_pair_list(struct aoi_space *space, struct object_set * watcher, struct object_set * marker) {
	int i,j;
	for (i=0;i<watcher->number;i++) {
		for (j=0;j<marker->number;j++) {
			gen_pair(space, watcher->slot[i], marker->slot[j]);
		}
	}
}

void 
aoi_message(struct aoi_space *space) {
	flush_pair(space);
	space->watcher_static->number = 0;
	space->watcher_move->number = 0;
	space->marker_static->number = 0;
	space->marker_move->number = 0;
	map_foreach(space->object, set_push , space);
	gen_pair_list(space, space->watcher_static, space->marker_move);
	gen_pair_list(space, space->watcher_move, space->marker_static);
	gen_pair_list(space, space->watcher_move, space->marker_move);
}

static void *
default_alloc(void * ud, void *ptr, size_t sz) {
	if (ptr == NULL) {
		void *p = malloc(sz);
		return p;
	}
	free(ptr);
	return NULL;
}

struct aoi_space * 
aoi_new(int w, int h, float f) {
	struct aoi_space * space = aoi_create(default_alloc, NULL);
	space->w = w;
	space->h = h;
	space->f = f;
	return space;
}

inline static void
release_moved(struct aoi_space * space, struct obj_moved * moved) {
	if(moved == NULL)
	{
		return;
	}

	if (moved->prev) {
		moved->prev->next = moved->next;
	} else {
		space->moved = moved->next;
	}
	if (moved->next) {
		moved->next->prev = moved->prev;
	}
	space->alloc(space->alloc_ud, moved, sizeof(*moved));
}

void
aoi_move_to(struct aoi_space *space, uint32_t id, float speed, float pos[3]) {
	struct object * obj = map_query(space, space->object, id, 0);
	if(obj == NULL || (obj->mode & MODE_DROP))
	{
		return;
	}

	//计算移动方向
	float vec[3] = {0};
	float distance = sqrt(DIST2(obj->position, pos));
	if(distance > 0)
	{
		vec[0] = (pos[0] - obj->position[0]) / distance;
		vec[1] = (pos[1] - obj->position[1]) / distance;
		vec[2] = (pos[2] - obj->position[2]) / distance;
	}
	else
	{
		return;
	}

	struct obj_moved* moved = space->moved;
	while(moved)
	{
		if(moved->id == id)
		{
			break;
		}
		moved = moved->next;
	}

	if(moved == NULL)
	{
		moved = space->alloc(space->alloc_ud, NULL, sizeof(*moved));
		moved->next = space->moved;
		moved->prev = NULL;
		if(space->moved)
		{
			space->moved->prev = moved;
		}
		space->moved = moved;
	}

	moved->id = id;
	moved->speed = speed;
	copy_position(moved->pos, pos);
	copy_position(moved->vec, vec);
	moved->distance = distance;
}

void
aoi_set_speed(struct aoi_space *space, uint32_t id, float speed)
{
	struct obj_moved* moved = space->moved;
	while(moved)
	{
		if(moved->id == id)
		{
			break;
		}
		moved = moved->next;
	}

	if(moved)
	{
		moved->speed = speed;

		//如果速度为0，则刷新位置
		if(speed == 0)
		{
			struct object * obj = map_query(space, space->object, id, 0);
			if(obj == NULL || (obj->mode & MODE_DROP))
			{
				return;
			}

			if(obj->position[0] != moved->pos[0] && obj->position[1] != moved->pos[1] && obj->position[2] != moved->pos[2])
			{
				copy_position(obj->position, moved->pos);
				flush_move_sign(space, obj);
			}
		}
	}
}

void
aoi_apply_move(struct aoi_space *space, float delta)
{
	if(delta <= 0)
	{
		return;
	}
	
	struct obj_moved* moved = space->moved;
	while(moved)
	{
		float distance = moved->speed * delta;
		if(distance == 0)
		{
			moved = moved->next;
			continue;
		}

		struct object * obj = map_query(space, space->object, moved->id, 0);
		if(obj == NULL || obj->mode & MODE_DROP)
		{
			release_moved(space, moved);
			continue;
		}

		//获取object，并计算当前位置到终点的距离
		moved->distance = sqrt(DIST2(obj->position, moved->pos));
		if(distance >= moved->distance)
		{
			copy_position(obj->position, moved->pos);
			struct obj_moved* next = moved->next;
			release_moved(space, moved);
			moved = next;

			flush_move_sign(space, obj);
		}
		else
		{
			moved->distance -= distance;
			moved->pos[0] += moved->vec[0] * distance;
			moved->pos[1] += moved->vec[1] * distance;
			moved->pos[2] += moved->vec[2] * distance;
			moved = moved->next;

			set_position(space, obj, moved->pos);
		}
	}
}

void
aoi_cancel_move(struct aoi_space *space, uint32_t id) {
	struct obj_moved* moved = space->moved;
	while(moved)
	{
		if(moved->id == id)
		{
			break;
		}
		moved = moved->next;
	}

	if(moved)
	{
		release_moved(space, moved);
		struct object * obj = map_query(space, space->object, id, 0);
		if(obj == NULL || (obj->mode & MODE_DROP))
		{
			return;
		}

		if(obj->position[0] != moved->pos[0] && obj->position[1] != moved->pos[1] && obj->position[2] != moved->pos[2])
		{
			flush_move_sign(space, obj);
		}
	}
}

void*
aoi_get_user_data(struct aoi_space *space, const char* data_id)
{
	if(space == NULL)
	{
		return NULL;
	}

	struct user_data_dict* dict = _get_user_data(space->user_datas, data_id);
	if(dict)
	{
		return dict->data;
	}
	return NULL;
}

void*
aoi_create_user_data(struct aoi_space *space, const char* data_id, size_t sz)
{
	if(space == NULL)
	{
		return NULL;
	}
	
	struct user_data_dict* dict = _get_user_data(space->user_datas, data_id);
	assert(dict == NULL);
	
	dict = space->alloc(space->alloc_ud, NULL, sizeof(*dict));
	dict->id = str_dup(data_id);
	dict->data = space->alloc(space->alloc_ud, NULL, sz);
	memset(dict->data, 0, sz);
	HASH_ADD_STR(space->user_datas, id, dict);
	return dict->data;
}

void
aoi_push_message_handler(struct aoi_space *space, const char* message_id, message_handler* cb)
{
	struct user_data_dict* dict = _get_user_data(space->message_handlers, message_id);
	if (dict)
	{
		struct data_link_list* handlers = dict->data;
		while(handlers)
		{
			if(handlers->data == cb)
			{
				return;
			}
			handlers = handlers->next;
		}
	}

	if(dict == NULL)
	{
		dict = space->alloc(space->alloc_ud, NULL, sizeof(*dict));
		dict->id = str_dup(message_id);
		dict->data = NULL;
		HASH_ADD_STR(space->message_handlers, id, dict);
	}

	struct data_link_list* handler = space->alloc(space->alloc_ud, NULL, sizeof(*handler));
	handler->data = cb;
	handler->next = NULL;
	handler->prev = NULL;
	//添加到链表末尾
	if(dict->data == NULL)
	{
		dict->data = handler;
	}
	else
	{
		struct data_link_list* last = dict->data;
		while(last->next)
		{
			last = last->next;
		}
		last->next = handler;
		handler->prev = last;
	}
}

void
aoi_pop_message_handler(struct aoi_space *space, const char* message_id, message_handler* cb)
{
	struct user_data_dict* dict = _get_user_data(space->message_handlers, message_id);
	if(dict == NULL)
	{
		return;
	}

	struct data_link_list* handler = dict->data;
	while(handler)
	{
		if(handler->data == cb)
		{
			break;
		}
		handler = handler->next;
	}

	if(handler == NULL)
	{
		return;
	}

	if(handler->prev)
	{
		handler->prev->next = handler->next;
	}
	else
	{
		dict->data = handler->next;
	}

	if(handler->next)
	{
		handler->next->prev = handler->prev;
	}

	space->alloc(space->alloc_ud, handler, sizeof(*handler));
}

void aoi_fire_message(struct aoi_space *space, const char* message_id, void* userdata)
{
	struct user_data_dict* dict = _get_user_data(space->message_handlers, message_id);
	if(dict == NULL)
	{
		return;
	}

	const char* current_message_id = space->current_message_id;
	space->current_message_id = message_id;
	struct data_link_list* handler = dict->data;
	while(handler)
	{
		message_handler* cb = handler->data;
		handler = handler->next;
		if(cb)
		{
			cb(space, userdata);
		}
	}
	space->current_message_id = current_message_id;
}

const char*
aoi_current_message_id(struct aoi_space *space)
{
	return space->current_message_id;
}

static void free_aoi_space_callback(struct aoi_space *space, void* userdata)
{
	aoi_pop_message_handler(space, ID_DELETE_OBJECT, drop_object_callback);
	aoi_pop_message_handler(space, ID_CREATE_PAIR, gen_pair_callback);
	aoi_pop_message_handler(space, ID_DELETE_PAIR, drop_pair_callback);
}

static void drop_object_callback(struct aoi_space* space, void* userdata)
{
	int objid = ((struct aoi_object_data*)userdata)->id;
	struct free_ids *frees = space->frees;
	if(frees->count >= frees->capacity)
	{
		uint32_t* new_ids = space->alloc(space->alloc_ud, NULL, sizeof(uint32_t) * frees->capacity * 2);
		memcpy(new_ids, frees->ids, sizeof(uint32_t) * frees->capacity);
		space->alloc(space->alloc_ud, frees->ids, sizeof(uint32_t) * frees->capacity);
		frees->ids = new_ids;
		frees->capacity *= 2;
	}
	frees->ids[frees->count++] = objid;
}

static void gen_pair_callback(struct aoi_space* space, void* userdata)
{
	struct pair_list * pair = ((struct aoi_pair_data*)userdata)->pair;
	grab_neighbor(space, pair->watcher, pair);
	grab_neighbor(space, pair->marker, pair);

	uint32_t ids[2] = {pair->watcher->id, pair->marker->id};
	aoi_fire_message(space, ID_NEIGHBOR_ENTER, ids);
}

static void drop_pair_callback(struct aoi_space* space, void* userdata)
{
	struct pair_list * pair = ((struct aoi_pair_data*)userdata)->pair;
	drop_neighbor(space, pair->watcher, pair);
	drop_neighbor(space, pair->marker, pair);

	uint32_t ids[2] = {pair->watcher->id, pair->marker->id};
	aoi_fire_message(space, ID_NEIGHBOR_LEAVE, ids);
}

int aoi_get_object_position(struct aoi_space *space, uint32_t id, float* pos, int* mode)
{
	struct object* obj = NULL;
	if(space->object_current && space->object_current->id == id)
	{
		obj = space->object_current;
	}
	else
	{
		obj = map_query(space, space->object, id, 0);
	}
	
	if(obj == NULL)
	{
		return 0;
	}
	if(mode)
	{
		*mode = obj->mode;
	}
	if(pos)
	{
		copy_position(pos, obj->position);
	}
	return 1;
}

int aoi_has_neighbor(struct aoi_space *space, uint32_t id, int mask)
{
	struct object* obj = map_query(space, space->object, id, 0);
	if(obj == NULL || (obj->mode & MODE_DROP) || obj->neighbors == NULL)
	{
		return 0;
	}
	return (obj->neighbor_mask & mask) != 0 ? 1 : 0;
}

int aoi_begin_parse_neighbor(struct aoi_space *space, uint32_t id)
{
	struct object* obj = map_query(space, space->object, id, 0);
	if(obj == NULL || (obj->mode & MODE_DROP) || obj->neighbors == NULL)
	{
		return 0;
	}

	if(space->neighbor_host && space->neighbor_host != obj)
	{
		return 0;
	}
	grab_object(obj);
	space->neighbor = NULL;
	space->neighbor_host = obj;
	return 1;
}

int aoi_next_neighbor(struct aoi_space *space, uint32_t* id)
{
	if(space->neighbor_host == NULL)
	{
		return 0;
	}

	struct neighbor_list* neighbor = space->neighbor;
	if(neighbor == NULL)
	{
		neighbor = space->neighbor_host->neighbors;
	}
	else
	{
		neighbor = neighbor->next;
	}
	space->neighbor = neighbor;

	if(neighbor == NULL)
	{
		return 0;
	}
	struct pair_list* pair = (struct pair_list*)neighbor->pair;
	if(pair->watcher->id == space->neighbor_host->id)
	{
		space->object_current = pair->marker;
	}
	else
	{
		space->object_current = pair->watcher;
	}
	*id = space->object_current->id;
	return 1;
}

int aoi_is_pair(struct aoi_space *space, uint32_t id, uint32_t tid)
{
	if(id == tid)
	{
		return 0;
	}
	struct pair_list* pair = NULL;
	uint64_t key = gen_key(id, tid);
	HASH_FIND(hh, space->hot, &key, sizeof(key), pair);
	return pair != NULL ? 1 : 0;
}

int aoi_end_parse_neighbor(struct aoi_space *space)
{
	if(space->neighbor_host == NULL)
	{
		return 0;
	}
	drop_object(space, space->neighbor_host);
	space->object_current = NULL;
	space->neighbor_host = NULL;
	space->neighbor = NULL;
	return 1;
}