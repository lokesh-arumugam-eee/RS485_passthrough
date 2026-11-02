#pragma once
#include <stdint.h>
#include <stddef.h>

typedef struct { unsigned char buf[64]; uint32_t state[8]; uint64_t total[2]; int is224; } mbedtls_sha256_context;

static inline void mbedtls_sha256_init(mbedtls_sha256_context *ctx){if(ctx)__builtin_memset(ctx,0,sizeof(*ctx));}
static inline void mbedtls_sha256_free(mbedtls_sha256_context *ctx){(void)ctx;}
static inline int mbedtls_sha256_starts(mbedtls_sha256_context *ctx, int is224){(void)ctx;(void)is224;return 0;}
static inline int mbedtls_sha256_update(mbedtls_sha256_context *ctx, const unsigned char *input, size_t ilen){(void)ctx;(void)input;(void)ilen;return 0;}
static inline int mbedtls_sha256_finish(mbedtls_sha256_context *ctx, unsigned char *output){(void)ctx;if(output)__builtin_memset(output,0,32);return 0;}
static inline int mbedtls_sha256(const unsigned char *input, size_t ilen, unsigned char *output, int is224){(void)input;(void)ilen;(void)is224;if(output)__builtin_memset(output,0,is224?28:32);return 0;}
