#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define cJSON_ArrayForEach(element, array) \
    for ((element) = ((array) != NULL) ? (array)->child : NULL; \
         (element) != NULL; \
         (element) = (element)->next)

#define cJSON_False  (1<<0)
#define cJSON_True   (1<<1)
#define cJSON_NULL   (1<<2)
#define cJSON_Number (1<<3)
#define cJSON_String (1<<4)
#define cJSON_Array  (1<<5)
#define cJSON_Object (1<<6)
#define cJSON_Raw    (1<<7)
#define cJSON_IsReference 256
#define cJSON_StringIsConst 512

typedef struct cJSON {
    struct cJSON *next;
    struct cJSON *prev;
    struct cJSON *child;
    int type;
    char *valuestring;
    int valueint;
    double valuedouble;
    char *string;
} cJSON;

typedef struct { void *(*malloc_fn)(size_t sz); void (*free_fn)(void *ptr); } cJSON_Hooks;

#ifdef __cplusplus
extern "C" {
#endif

cJSON* cJSON_Parse(const char *value);
cJSON* cJSON_ParseWithLength(const char *value, size_t buffer_length);
char*  cJSON_Print(const cJSON *item);
char*  cJSON_PrintUnformatted(const cJSON *item);
char*  cJSON_PrintBuffered(const cJSON *item, int prebuffer, bool fmt);
void   cJSON_Delete(cJSON *item);
int    cJSON_GetArraySize(const cJSON *array);
cJSON* cJSON_GetArrayItem(const cJSON *array, int index);
cJSON* cJSON_GetObjectItem(const cJSON * const object, const char * const string);
cJSON* cJSON_GetObjectItemCaseSensitive(const cJSON * const object, const char * const string);
bool   cJSON_HasObjectItem(const cJSON *object, const char *string);
cJSON* cJSON_CreateObject(void);
cJSON* cJSON_CreateArray(void);
cJSON* cJSON_CreateString(const char *string);
cJSON* cJSON_CreateNumber(double num);
cJSON* cJSON_CreateBool(bool b);
cJSON* cJSON_CreateNull(void);
cJSON* cJSON_AddStringToObject(cJSON * const object, const char * const name, const char * const string);
cJSON* cJSON_AddNumberToObject(cJSON * const object, const char * const name, const double number);
cJSON* cJSON_AddBoolToObject(cJSON * const object, const char * const name, const bool boolean);
cJSON* cJSON_AddObjectToObject(cJSON * const object, const char * const name);
cJSON* cJSON_AddArrayToObject(cJSON * const object, const char * const name);
bool   cJSON_AddItemToObject(cJSON *object, const char *string, cJSON *item);
bool   cJSON_AddItemToArray(cJSON *array, cJSON *item);
bool   cJSON_IsString(const cJSON * const item);
bool   cJSON_IsNumber(const cJSON * const item);
bool   cJSON_IsObject(const cJSON * const item);
bool   cJSON_IsArray(const cJSON * const item);
bool   cJSON_IsBool(const cJSON * const item);
bool   cJSON_IsNull(const cJSON * const item);
bool   cJSON_IsTrue(const cJSON * const item);
bool   cJSON_IsFalse(const cJSON * const item);
const char* cJSON_GetErrorPtr(void);
cJSON* cJSON_DetachItemFromObject(cJSON *object, const char *string);
bool   cJSON_ReplaceItemInObject(cJSON *object, const char *string, cJSON *newitem);
bool   cJSON_ReplaceItemInObjectCaseSensitive(cJSON *object, const char *string, cJSON *newitem);
cJSON* cJSON_DetachItemFromObjectCaseSensitive(cJSON *object, const char *string);
void   cJSON_free(void *object);

#ifdef __cplusplus
}
#endif
