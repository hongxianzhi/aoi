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
#define AOI_RADIS2 (AOI_RADIS * AOI_RADIS)
#define DIST2(p1,p2) ((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]))
#define MODE_WATCHER 1
#define MODE_MARKER 2
#define MODE_MOVE 4
#define MODE_DROP 8

struct pair_list;
static void drop_pair(struct aoi_space * space, struct pair_list *p);
static void free_aoi_space_callback(struct aoi_space *space, void* handlers_data, void* userdata);
static void drop_object_callback(struct aoi_space* space, void* handlers_data, void* userdata);
static void gen_neighbor_callback(struct aoi_space* space, void* handlers_data, void* userdata);
static void drop_neighbor_callback(struct aoi_space* space, void* handlers_data, void* userdata);

struct neighbor_list{
	void* pair;
	struct neighbor_list* next;
	struct neighbor_list* prev;
};

struct object {
	int ref;
	uint32_t id;
	int mode;
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

struct id_list {
	uint32_t id;
	struct id_list * next;
	struct id_list * prev;
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

struct aoi_handler_host {
	void *datas;
	uint32_t id;
	user_callback *user_cb[32];
	struct aoi_handler_host *next;
	struct aoi_handler_host *prev;
};

struct aoi_space {
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

	struct id_list * ids;
	uint32_t id_begin;
	uint32_t user_handlers_mask;
	struct aoi_handler_host * user_handlers;
};

inline static void 
copy_position(float des[3], float src[3]) {
	des[0] = src[0];
	des[1] = src[1];
	des[2] = src[2];
}

static inline void
fire_object_callback(struct aoi_space *space, struct object* obj, int type)
{
	struct _aoi_object_callback_data data;
	data.obj = obj;
	copy_position(data.pos, obj->position);
	aoi_fire_user_callback(space, type, &data);
}

static inline void
fire_pair_callback(struct aoi_space *space, struct pair_list* p, int type)
{
	struct _aoi_pair_callback_data data;
	data.pair = p;
	aoi_fire_user_callback(space, type, &data);
}

static inline void
fire_free_space_callback(struct aoi_space *space)
{
	struct _free_aoi_space_data data;
	data.space = space;
	aoi_fire_user_callback(space, USER_HANDLER_TYPE_FREE_AOI_SPACE, &data);
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
	obj->ref = 1;
	obj->id = id;
	obj->mode = 0;
	obj->radius = 0;
	obj->neighbors = NULL;
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
map_query(struct aoi_space *space, struct map * m, uint32_t id) {
	struct map_slot *s = mainposition(m, id);
	for (;;) {
		if (s->id == id) {
			if (s->obj == NULL) {
				s->obj = new_object(space, id);

				fire_object_callback(space, s->obj, USER_HANDLER_TYPE_AOI_CREATE_OBJECT);
			}
			return s->obj;
		}
		if (s->next < 0) {
			break;
		}
		s=&m->slot[s->next];
	}
	struct object * obj = new_object(space, id);
	map_insert(space, m , id , obj);

	fire_object_callback(space, obj, USER_HANDLER_TYPE_AOI_CREATE_OBJECT);
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
	space->alloc(space->alloc_ud, obj, sizeof(*obj));
}

inline static void
drop_object(struct aoi_space * space, struct object *obj) {
	--obj->ref;
	if (obj->ref <=0) {
		fire_object_callback(space, obj, USER_HANDLER_TYPE_AOI_DELETE_OBJECT);

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
	space->alloc = alloc;
	space->alloc_ud = ud;
	space->object = map_new(space);
	space->watcher_static = set_new(space);
	space->marker_static = set_new(space);
	space->watcher_move = set_new(space);
	space->marker_move = set_new(space);
	space->hot = NULL;
	space->ids = NULL;
	space->id_begin = 1;
	space->moved = NULL;
	space->user_handlers = NULL;
	space->user_handlers_mask = 0;

	aoi_add_handler_host(space, 0);
	aoi_set_user_callback(space, 0, USER_HANDLER_TYPE_FREE_AOI_SPACE, free_aoi_space_callback);
	//添加对象的回调
	aoi_set_user_callback(space, 0, USER_HANDLER_TYPE_AOI_DELETE_OBJECT, drop_object_callback);
	//添加邻居列表的回调
	aoi_set_user_callback(space, 0, USER_HANDLER_TYPE_AOI_GEN_PAIR, gen_neighbor_callback);
	aoi_set_user_callback(space, 0, USER_HANDLER_TYPE_AOI_DROP_PAIR, drop_neighbor_callback);

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
	fire_free_space_callback(space);

	map_foreach(space->object, delete_object, space);
	map_delete(space, space->object);
	delete_pair_list(space);
	delete_set(space,space->watcher_static);
	delete_set(space,space->marker_static);
	delete_set(space,space->watcher_move);
	delete_set(space,space->marker_move);

	struct id_list *ids = space->ids;
	while(ids)
	{
		struct id_list *next = ids->next;
		space->alloc(space->alloc_ud, ids, sizeof(*ids));
		ids = next;
	}
	space->ids = NULL;

	space->alloc(space->alloc_ud, space, sizeof(*space));
}

int aoi_gen_id(struct aoi_space *space)
{
	while (true)
	{
		int result = 0;
		if(space->ids == NULL)
		{
			result = space->id_begin++;
		}
		else
		{
			//从链表中取出一个id
			struct id_list *item = space->ids;
			space->ids = item->next;
			result = item->id;
			space->alloc(space->alloc_ud, item, sizeof(*item));
		}
		assert(result >= 0);
		if(map_query(space, space->object, result) == NULL)
		{
			return result;
		}
	}
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

	copy_position(obj->last, obj->position);
	obj->mode |= MODE_MOVE;
	fire_object_callback(space, obj, USER_HANDLER_TYPE_AOI_MOVED);
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

void
aoi_position(struct aoi_space *space, uint32_t id, float pos[3])
{
	set_position(space, map_query(space, space->object, id), pos);
}

void
aoi_update(struct aoi_space * space , uint32_t id, const char * modestring , float pos[3]) {
	struct object * obj = map_query(space, space->object,id);
	int i;
	bool set_watcher = false;
	bool set_marker = false;

	for (i=0;modestring[i];++i) {
		char m = modestring[i];
		switch(m) {
		case 'w':
			set_watcher = true;
			break;
		case 'm':
			set_marker = true;
			break;
		case 'd':
			if (!(obj->mode & MODE_DROP)) {
				obj->mode = MODE_DROP;
				drop_object(space, obj);
			}
			return;
		}
	}

	if (obj->mode & MODE_DROP) {
		obj->mode &= ~MODE_DROP;
		grab_object(obj);
	}

	bool changed = change_mode(obj, set_watcher, set_marker);
	copy_position(obj->position, pos);	
	if (changed || !is_near(pos, obj->last)) {
		// new object or change object mode
		// or position changed
		flush_move_sign(space, obj);
	}
}

static void
drop_pair(struct aoi_space * space, struct pair_list *p) {
	fire_pair_callback(space, p, USER_HANDLER_TYPE_AOI_DROP_PAIR);

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
			drop_pair(space, p);
		}
		else if ((watcher->mode & MODE_MOVE) == MODE_MOVE || (marker->mode & MODE_MOVE) == MODE_MOVE)
		{
			p->dis2 = dist2(p->watcher , p->marker);
			if (p->dis2 > AOI_RADIS2 * 1.2f)
			{
				drop_pair(space, p);
			}
		}
	}
}

static void
set_push_back(struct aoi_space * space, struct object_set * set, struct object *obj) {
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

	fire_pair_callback(space, p, USER_HANDLER_TYPE_AOI_GEN_PAIR);
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
aoi_new() {
	return aoi_create(default_alloc, NULL);
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
	struct object * obj = map_query(space, space->object, id);
	if (obj->mode & MODE_DROP) {
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
			struct object * obj = map_query(space, space->object, id);
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

		struct object * obj = map_query(space, space->object, moved->id);
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
		struct object * obj = map_query(space, space->object, id);
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

inline static void
update_user_handler_mask(struct aoi_space *space)
{
	space->user_handlers_mask = 0;
	struct aoi_handler_host * p = space->user_handlers;
	while(p)
	{
		for(int i = 0; i < sizeof(p->user_cb) / sizeof(user_callback*); ++i)
		{
			if(p->user_cb[i] != NULL)
			{
				space->user_handlers_mask |= (1 << i);
			}
		}
		p = p->next;
	}
}

inline static struct aoi_handler_host*
get_user_callbacks(struct aoi_space *space, uint32_t hostid)
{
	assert(space != NULL);
	struct aoi_handler_host * p = space->user_handlers;
	while(p)
	{
		if(p->id == hostid)
		{
			return p;
		}
		p = p->next;
	}
	return NULL;
}

void
aoi_add_handler_host(struct aoi_space *space, uint32_t hostid)
{
	assert(space != NULL);
	struct aoi_handler_host * p = space->user_handlers;
	while(p)
	{
		if(p->id == hostid)
		{
			assert(0);
			return;
		}
		p = p->next;
	}

	struct aoi_handler_host * handlers = space->alloc(space->alloc_ud, NULL, sizeof(struct aoi_handler_host));
	memset(handlers, 0, sizeof(struct aoi_handler_host));
	handlers->next = space->user_handlers;
	handlers->id = hostid;
	if(space->user_handlers)
	{
		space->user_handlers->prev = handlers;
	}
	space->user_handlers = handlers;
}

void*
aoi_alloc_handler_host_data(struct aoi_space *space, uint32_t hostid, size_t sz)
{
	struct aoi_handler_host * callbacks = get_user_callbacks(space, hostid);
	if(callbacks == NULL)
	{
		return NULL;
	}
	if(callbacks->datas != NULL)
	{
		space->alloc(space->alloc_ud, callbacks->datas, sz);
		callbacks->datas = NULL;
	}
	callbacks->datas = space->alloc(space->alloc_ud, NULL, sz);
	return callbacks->datas;
}

void*
aoi_get_handler_hsot_data(struct aoi_space *space, uint32_t hostid)
{
	struct aoi_handler_host * callbacks = get_user_callbacks(space, hostid);
	return callbacks ? callbacks->datas : NULL;
}

void
aoi_del_user_handler_host(struct aoi_space *space, uint32_t hostid)
{
	struct aoi_handler_host * callbacks = get_user_callbacks(space, hostid);
	if(callbacks == NULL)
	{
		return;
	}

	if(callbacks->prev)
	{
		callbacks->prev->next = callbacks->next;
	}
	else
	{
		space->user_handlers = callbacks->next;
	}

	if(callbacks->next)
	{
		callbacks->next->prev = callbacks->prev;
	}

	if(callbacks->datas)
	{
		space->alloc(space->alloc_ud, callbacks->datas, sizeof(callbacks->datas));
	}

	space->alloc(space->alloc_ud, callbacks, sizeof(struct aoi_handler_host));

	update_user_handler_mask(space);
}

void
aoi_set_user_callback(struct aoi_space *space, uint32_t hostid, int type, user_callback* cb)
{
	struct aoi_handler_host * callbacks = get_user_callbacks(space, hostid);
	if(callbacks == NULL)
	{
		return;
	}

	if(type < 0 || type >= sizeof(callbacks->user_cb) / sizeof(user_callback*))
	{
		return;
	}

	callbacks->user_cb[type] = cb;

	update_user_handler_mask(space);
}

void
aoi_fire_user_callback(struct aoi_space *space, int type, void* userdata)
{
	if((space->user_handlers_mask & (1 << type)) == 0)
	{
		return;
	}

	struct aoi_handler_host * p = space->user_handlers;
	while(p)
	{
		if(p->user_cb[type])
		{
			p->user_cb[type](space, p->datas, userdata);
		}
		p = p->next;
	}
}

static void free_aoi_space_callback(struct aoi_space *space, void* handlers_data, void* userdata)
{
	aoi_set_user_callback(space, 0, USER_HANDLER_TYPE_AOI_DELETE_OBJECT, NULL);
}

static void drop_object_callback(struct aoi_space* space, void* handlers_data, void* userdata)
{
	struct object* obj = ((struct _aoi_object_callback_data*)userdata)->obj;
	struct id_list *id = space->alloc(space->alloc_ud, NULL, sizeof(*id));
	id->id = obj->id;
	id->next = space->ids;
	space->ids = id;
	if(id->next)
	{
		id->next->prev = id;
	}
	id->id = obj->id;
}

static void gen_neighbor_callback(struct aoi_space* space, void* handlers_data, void* userdata)
{
	struct pair_list * pair = ((struct _aoi_pair_callback_data*)userdata)->pair;
	grab_neighbor(space, pair->watcher, pair);
	grab_neighbor(space, pair->marker, pair);
}

static void drop_neighbor_callback(struct aoi_space* space, void* handlers_data, void* userdata)
{
	struct pair_list * pair = ((struct _aoi_pair_callback_data*)userdata)->pair;
	drop_neighbor(space, pair->watcher, pair);
	drop_neighbor(space, pair->marker, pair);
}

int aoi_begin_parse_neighbor(struct aoi_space *space, uint32_t id)
{
	struct object* obj = map_query(space, space->object, id);
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
		*id = pair->marker->id;
	}
	else
	{
		*id = pair->watcher->id;
	}
	return 1;
}

int aoi_is_neighbor(struct aoi_space *space, uint32_t id)
{
	if(space->neighbor_host == NULL || space->neighbor_host->id == id || space->neighbor_host->neighbors == NULL)
	{
		return 0;
	}

	uint32_t hostid = space->neighbor_host->id;
	struct neighbor_list* neighbor = space->neighbor_host->neighbors;
	while(neighbor)
	{
		struct pair_list* pair = (struct pair_list*)neighbor->pair;
		if(pair->watcher->id == hostid && pair->marker->id == id)
		{
			return 1;
		}
		if(pair->watcher->id == id && pair->marker->id == hostid)
		{
			return 1;
		}
		neighbor = neighbor->next;
	}
	return 0;
}

int aoi_end_parse_neighbor(struct aoi_space *space)
{
	if(space->neighbor_host == NULL)
	{
		return 0;
	}
	drop_object(space, space->neighbor_host);
	space->neighbor_host = NULL;
	space->neighbor = NULL;
	return 1;
}