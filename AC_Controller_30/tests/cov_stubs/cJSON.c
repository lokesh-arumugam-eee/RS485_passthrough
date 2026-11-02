/*
 * Minimal but functional cJSON implementation for the coverage harness.
 * Replaces the static-inline stub so that cJSON_Parse actually returns a tree,
 * allowing all JSON-parsing firmware code to execute under gcov.
 */
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

static const char *g_error_ptr = NULL;

/* ---------- allocator --------------------------------------------------- */
static cJSON *cj_alloc(void) { return (cJSON*)calloc(1, sizeof(cJSON)); }

/* ---------- string parser ----------------------------------------------- */
/* Advances *p past closing '"', returns malloc'd string or NULL on error.  */
static char *parse_str(const char **p) {
    if (**p != '"') return NULL;
    const char *start = *p + 1;

    /* First pass: measure output length (escape sequences collapse to 1 char) */
    const char *s = start;
    size_t len = 0;
    while (*s && *s != '"') {
        if (*s == '\\') { s++; if (!*s) return NULL; }
        s++; len++;
    }
    if (*s != '"') return NULL;

    char *out = (char*)malloc(len + 1);
    if (!out) return NULL;

    /* Second pass: copy with escape handling */
    *p = start;
    char *d = out;
    while (**p != '"') {
        if (**p == '\\') {
            (*p)++;
            switch (**p) {
                case 'n':  *d++ = '\n'; break;
                case 't':  *d++ = '\t'; break;
                case 'r':  *d++ = '\r'; break;
                case '"':  *d++ = '"';  break;
                case '\\': *d++ = '\\'; break;
                case '/':  *d++ = '/';  break;
                default:   *d++ = **p;  break;
            }
        } else {
            *d++ = **p;
        }
        (*p)++;
    }
    *d = '\0';
    (*p)++; /* skip closing '"' */
    return out;
}

/* ---------- forward declaration ----------------------------------------- */
static cJSON *parse_value(const char **p);

/* ---------- object parser ----------------------------------------------- */
static cJSON *parse_object(const char **p) {
    cJSON *obj = cj_alloc();
    if (!obj) return NULL;
    obj->type = cJSON_Object;
    (*p)++; /* skip '{' */

    for (;;) {
        while (**p && (unsigned char)**p <= ' ') (*p)++;
        if (**p == '}') { (*p)++; return obj; }
        if (**p != '"') { cJSON_Delete(obj); return NULL; }

        char *key = parse_str(p);
        if (!key) { cJSON_Delete(obj); return NULL; }

        while (**p && (unsigned char)**p <= ' ') (*p)++;
        if (**p != ':') { free(key); cJSON_Delete(obj); return NULL; }
        (*p)++;
        while (**p && (unsigned char)**p <= ' ') (*p)++;

        cJSON *val = parse_value(p);
        if (!val) { free(key); cJSON_Delete(obj); return NULL; }

        val->string = key;
        /* append to child list */
        if (!obj->child) {
            obj->child = val;
        } else {
            cJSON *c = obj->child;
            while (c->next) c = c->next;
            c->next = val;
            val->prev = c;
        }

        while (**p && (unsigned char)**p <= ' ') (*p)++;
        if (**p == ',') { (*p)++; continue; }
        if (**p == '}') { (*p)++; break; }
        cJSON_Delete(obj); return NULL;
    }
    return obj;
}

/* ---------- array parser ------------------------------------------------ */
static cJSON *parse_array(const char **p) {
    cJSON *arr = cj_alloc();
    if (!arr) return NULL;
    arr->type = cJSON_Array;
    (*p)++;

    for (;;) {
        while (**p && (unsigned char)**p <= ' ') (*p)++;
        if (**p == ']') { (*p)++; return arr; }

        cJSON *val = parse_value(p);
        if (!val) { cJSON_Delete(arr); return NULL; }

        if (!arr->child) {
            arr->child = val;
        } else {
            cJSON *c = arr->child;
            while (c->next) c = c->next;
            c->next = val;
            val->prev = c;
        }

        while (**p && (unsigned char)**p <= ' ') (*p)++;
        if (**p == ',') { (*p)++; continue; }
        if (**p == ']') { (*p)++; break; }
        cJSON_Delete(arr); return NULL;
    }
    return arr;
}

/* ---------- value dispatcher -------------------------------------------- */
static cJSON *parse_value(const char **p) {
    while (**p && (unsigned char)**p <= ' ') (*p)++;
    if (!**p) return NULL;

    switch (**p) {
        case '{': return parse_object(p);
        case '[': return parse_array(p);
        case '"': {
            cJSON *it = cj_alloc();
            if (!it) return NULL;
            it->type = cJSON_String;
            it->valuestring = parse_str(p);
            if (!it->valuestring) { free(it); return NULL; }
            return it;
        }
        case 't':
            if (strncmp(*p, "true", 4) == 0) {
                *p += 4;
                cJSON *b = cj_alloc(); if (b) b->type = cJSON_True; return b;
            }
            return NULL;
        case 'f':
            if (strncmp(*p, "false", 5) == 0) {
                *p += 5;
                cJSON *b = cj_alloc(); if (b) b->type = cJSON_False; return b;
            }
            return NULL;
        case 'n':
            if (strncmp(*p, "null", 4) == 0) {
                *p += 4;
                cJSON *n = cj_alloc(); if (n) n->type = cJSON_NULL; return n;
            }
            return NULL;
        default:
            if (**p == '-' || isdigit((unsigned char)**p)) {
                cJSON *n = cj_alloc();
                if (!n) return NULL;
                n->type = cJSON_Number;
                char *end;
                n->valuedouble = strtod(*p, &end);
                n->valueint    = (int)n->valuedouble;
                *p = end;
                return n;
            }
            return NULL;
    }
}

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */

cJSON *cJSON_Parse(const char *value) {
    if (!value) return NULL;
    g_error_ptr = NULL;
    const char *p = value;
    cJSON *r = parse_value(&p);
    if (!r) g_error_ptr = p;
    return r;
}

cJSON *cJSON_ParseWithLength(const char *value, size_t len) {
    (void)len;
    return cJSON_Parse(value);
}

void cJSON_Delete(cJSON *item) {
    while (item) {
        cJSON *next = item->next;
        if (item->child) cJSON_Delete(item->child);
        if (item->valuestring) free(item->valuestring);
        if (item->string) free(item->string);
        free(item);
        item = next;
    }
}

/* ---------- accessors --------------------------------------------------- */
int cJSON_GetArraySize(const cJSON *a) {
    if (!a) return 0;
    int n = 0;
    const cJSON *c = a->child;
    while (c) { n++; c = c->next; }
    return n;
}

cJSON *cJSON_GetArrayItem(const cJSON *a, int idx) {
    if (!a) return NULL;
    cJSON *c = a->child;
    while (c && idx > 0) { c = c->next; idx--; }
    return c;
}

cJSON *cJSON_GetObjectItem(const cJSON * const o, const char * const s) {
    if (!o || !s) return NULL;
    cJSON *c = o->child;
    while (c) {
        if (c->string && strcasecmp(c->string, s) == 0) return c;
        c = c->next;
    }
    return NULL;
}

cJSON *cJSON_GetObjectItemCaseSensitive(const cJSON * const o, const char * const s) {
    if (!o || !s) return NULL;
    cJSON *c = o->child;
    while (c) {
        if (c->string && strcmp(c->string, s) == 0) return c;
        c = c->next;
    }
    return NULL;
}

bool cJSON_HasObjectItem(const cJSON *o, const char *s) {
    return cJSON_GetObjectItem(o, s) != NULL;
}

/* ---------- type checks ------------------------------------------------- */
bool cJSON_IsString(const cJSON * const i) { return i && (i->type == cJSON_String); }
bool cJSON_IsNumber(const cJSON * const i) { return i && (i->type == cJSON_Number); }
bool cJSON_IsObject(const cJSON * const i) { return i && (i->type == cJSON_Object); }
bool cJSON_IsArray(const cJSON * const i)  { return i && (i->type == cJSON_Array);  }
bool cJSON_IsBool(const cJSON * const i)   { return i && (i->type & (cJSON_True|cJSON_False)); }
bool cJSON_IsNull(const cJSON * const i)   { return i && (i->type == cJSON_NULL);   }
bool cJSON_IsTrue(const cJSON * const i)   { return i && (i->type == cJSON_True);   }
bool cJSON_IsFalse(const cJSON * const i)  { return i && (i->type == cJSON_False);  }
const char *cJSON_GetErrorPtr(void)        { return g_error_ptr; }

/* ---------- create helpers ---------------------------------------------- */
static cJSON *new_typed(int t) { cJSON *i = cj_alloc(); if (i) i->type = t; return i; }

cJSON *cJSON_CreateObject(void) { return new_typed(cJSON_Object); }
cJSON *cJSON_CreateArray(void)  { return new_typed(cJSON_Array);  }
cJSON *cJSON_CreateNull(void)   { return new_typed(cJSON_NULL);   }

cJSON *cJSON_CreateBool(bool b) { return new_typed(b ? cJSON_True : cJSON_False); }

cJSON *cJSON_CreateNumber(double n) {
    cJSON *i = new_typed(cJSON_Number);
    if (i) { i->valuedouble = n; i->valueint = (int)n; }
    return i;
}

cJSON *cJSON_CreateString(const char *s) {
    cJSON *i = new_typed(cJSON_String);
    if (i && s) {
        i->valuestring = strdup(s);
        if (!i->valuestring) { free(i); return NULL; }
    }
    return i;
}

/* ---------- attach helpers ---------------------------------------------- */
static void append_child(cJSON *parent, cJSON *child) {
    if (!parent || !child) return;
    if (!parent->child) { parent->child = child; return; }
    cJSON *c = parent->child;
    while (c->next) c = c->next;
    c->next = child;
    child->prev = c;
}

static void attach_named(cJSON *obj, const char *name, cJSON *item) {
    if (!obj || !name || !item) return;
    if (item->string) free(item->string);
    item->string = strdup(name);
    append_child(obj, item);
}

cJSON *cJSON_AddStringToObject(cJSON * const o, const char * const n, const char * const s) {
    cJSON *i = cJSON_CreateString(s); attach_named(o, n, i); return i;
}
cJSON *cJSON_AddNumberToObject(cJSON * const o, const char * const n, const double v) {
    cJSON *i = cJSON_CreateNumber(v); attach_named(o, n, i); return i;
}
cJSON *cJSON_AddBoolToObject(cJSON * const o, const char * const n, const bool b) {
    cJSON *i = cJSON_CreateBool(b); attach_named(o, n, i); return i;
}
cJSON *cJSON_AddObjectToObject(cJSON * const o, const char * const n) {
    cJSON *i = cJSON_CreateObject(); attach_named(o, n, i); return i;
}
cJSON *cJSON_AddArrayToObject(cJSON * const o, const char * const n) {
    cJSON *i = cJSON_CreateArray(); attach_named(o, n, i); return i;
}
bool cJSON_AddItemToObject(cJSON *o, const char *s, cJSON *i) {
    if (!o || !s || !i) return false;
    attach_named(o, s, i); return true;
}
bool cJSON_AddItemToArray(cJSON *a, cJSON *i) {
    if (!a || !i) return false;
    append_child(a, i); return true;
}

/* ---------- detach / replace -------------------------------------------- */
cJSON *cJSON_DetachItemFromObject(cJSON *o, const char *s) {
    if (!o || !s) return NULL;
    cJSON *i = cJSON_GetObjectItem(o, s);
    if (!i) return NULL;
    if (i->prev) i->prev->next = i->next; else o->child = i->next;
    if (i->next) i->next->prev = i->prev;
    i->next = i->prev = NULL;
    return i;
}
cJSON *cJSON_DetachItemFromObjectCaseSensitive(cJSON *o, const char *s) {
    return cJSON_DetachItemFromObject(o, s);
}
bool cJSON_ReplaceItemInObject(cJSON *o, const char *s, cJSON *newitem) {
    if (!o || !s || !newitem) return false;
    cJSON *old = cJSON_DetachItemFromObject(o, s);
    if (old) cJSON_Delete(old);
    attach_named(o, s, newitem);
    return true;
}
bool cJSON_ReplaceItemInObjectCaseSensitive(cJSON *o, const char *s, cJSON *n) {
    return cJSON_ReplaceItemInObject(o, s, n);
}

/* ---------- serialiser -------------------------------------------------- */
static int ser_val(const cJSON *item, char *buf, int pos, int max);

static int ser_str(const char *s, char *buf, int pos, int max) {
    if (pos < max) buf[pos++] = '"';
    for (; s && *s && pos < max - 2; s++) {
        if (*s == '"' || *s == '\\') { if (pos < max) buf[pos++] = '\\'; }
        if (pos < max) buf[pos++] = *s;
    }
    if (pos < max) buf[pos++] = '"';
    return pos;
}

static int ser_val(const cJSON *item, char *buf, int pos, int max) {
    if (!item || pos >= max) return pos;
    switch (item->type & 0xFF) {
        case cJSON_NULL:  { const char *t = "null";  int l=4; if(pos+l<max){memcpy(buf+pos,t,l);pos+=l;} break; }
        case cJSON_True:  { const char *t = "true";  int l=4; if(pos+l<max){memcpy(buf+pos,t,l);pos+=l;} break; }
        case cJSON_False: { const char *t = "false"; int l=5; if(pos+l<max){memcpy(buf+pos,t,l);pos+=l;} break; }
        case cJSON_Number: {
            char tmp[32];
            if (item->valuedouble == (double)(int)item->valuedouble)
                snprintf(tmp, sizeof(tmp), "%d", (int)item->valuedouble);
            else
                snprintf(tmp, sizeof(tmp), "%g", item->valuedouble);
            int l = (int)strlen(tmp);
            if (pos + l < max) { memcpy(buf+pos, tmp, l); pos += l; }
            break;
        }
        case cJSON_String:
            pos = ser_str(item->valuestring ? item->valuestring : "", buf, pos, max);
            break;
        case cJSON_Object: {
            if (pos < max) buf[pos++] = '{';
            bool first = true;
            for (cJSON *c = item->child; c; c = c->next) {
                if (!first && pos < max) buf[pos++] = ',';
                first = false;
                pos = ser_str(c->string ? c->string : "", buf, pos, max);
                if (pos < max) buf[pos++] = ':';
                pos = ser_val(c, buf, pos, max);
            }
            if (pos < max) buf[pos++] = '}';
            break;
        }
        case cJSON_Array: {
            if (pos < max) buf[pos++] = '[';
            bool first = true;
            for (cJSON *c = item->child; c; c = c->next) {
                if (!first && pos < max) buf[pos++] = ',';
                first = false;
                pos = ser_val(c, buf, pos, max);
            }
            if (pos < max) buf[pos++] = ']';
            break;
        }
        default:
            if (pos < max) buf[pos++] = '0';
            break;
    }
    return pos;
}

char *cJSON_Print(const cJSON *item) {
    if (!item) return NULL;
    const int SZ = 16384;
    char *buf = (char*)malloc(SZ);
    if (!buf) return NULL;
    int end = ser_val(item, buf, 0, SZ - 1);
    buf[end] = '\0';
    return buf;
}
char *cJSON_PrintUnformatted(const cJSON *item) { return cJSON_Print(item); }
char *cJSON_PrintBuffered(const cJSON *item, int pb, bool fmt) { (void)pb; (void)fmt; return cJSON_Print(item); }

void cJSON_free(void *o) { free(o); }
