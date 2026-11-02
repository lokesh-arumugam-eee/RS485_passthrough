#ifndef _JIO_EA_SAFE_FUNCTIONS_H_
#define _JIO_EA_SAFE_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#define STRTOK_DELIM_MAX_LEN (16) 
#define JIO_EA_DBG_BUFFSIZE 64
#define RSIZE_MIN_STR      ( 1 )
#define RCNEGATE(x) (-(x))
#define sl_default_handler Jio_032101_ignore_handler_s
#ifdef JIO_EA_SIG_PROJECT
#define DP21PD "a9beb8ece9fd0ec547f5fe37ca578d54ad26"
#define DP12PD "8a474d9126049bff1844690a88b5517fa815d7165e26"
#endif
#ifdef JIO_EA_MIG_PROJECT
#define DP21PD "92a0bba5eaed11c17492fe2ecf1f8e11"
#define DP12PD "4ece26049eee181e530bbeb5500a825cd57d5239851e22c5"
#endif
#ifdef JIO_EA_BIG_PROJECT
#define DP21PD "a9adb8eceafe16c54390fe37cd579555ad26"
#define DP12PD "7cdaab504ece26049eee18056b6e80a5667ea751"
#endif
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
/* maximum sring length */
#define RSIZE_MAX_STR      ( 4UL << 10 )      /* 4KB */

#ifndef ESNULLP
#define ESNULLP         ( 400 )       /* null ptr                    */
#endif

#ifndef ESZEROL
#define ESZEROL         ( 401 )       /* length is zero              */
#endif

#ifndef ESLEMIN
#define ESLEMIN         ( 402 )       /* length is below min         */
#endif

#ifndef ESLEMAX
#define ESLEMAX         ( 403 )       /* length exceeds max          */
#endif

#ifndef ESOVRLP
#define ESOVRLP         ( 404 )       /* overlap undefined           */
#endif

#ifndef ESEMPTY
#define ESEMPTY         ( 405 )       /* empty string                */
#endif

#ifndef ESNOSPC
#define ESNOSPC         ( 406 )       /* not enough space for s2     */
#endif

#ifndef ESUNTERM
#define ESUNTERM        ( 407 )       /* unterminated string         */
#endif

#ifndef ESNODIFF
#define ESNODIFF        ( 408 )       /* no difference               */
#endif

#ifndef ESNOTFND
#define ESNOTFND        ( 409 )       /* not found                   */
#endif

/* EOK may or may not be defined in errno.h */
#ifndef EOK
#define EOK             ( 0 )

// Error codes enum
typedef enum {
    STRTOK_S_SUCCESS,
    STRTOK_S_NULL_DMAX,
    STRTOK_S_ZERO_DMAX,
    STRTOK_S_NULL_SRC,
    STRTOK_S_NULL_PTR,
    STRTOK_S_DEST_UNTERMINATED,
    STRTOK_S_SRC_UNTERMINATED
} Jio_EA_StrtokSErrorCode;

extern int8_t Jio_032112_EA_memcpy_s(void* p_vp_dest, size_t p_sz_destSize, const void* p_cvp_src, size_t p_sz_srcSize);
extern int8_t Jio_032112_EA_memset_s(void* p_vp_dest, size_t p_sz_destSize, const int i_val, size_t p_sz_srcSize);
extern size_t Jio_032110_EA_Get_String_Size(const char* p_ccp_str);
extern size_t Jio_032109_EA_strnlen_s(const char *p_ccp_rxstr, size_t p_i_rxstr_sz);
extern int Jio_032105_EA_Jio_sscanf_s(const char *__restrict buffer, const char *__restrict fmt, ...);
extern void Jio_032113_EA_handle_memcpy_s_error(int8_t ret, const char* p_func_name);
extern char *Jio_032108_EA_strtok_s(char *p_cp_dest, size_t *p_szp_dmax, const char *p_ccp_src, char **p_cpp_ptr);
extern void Jio_032107_EA_strtok_s_reset_error();
extern Jio_EA_StrtokSErrorCode Jio_032106_EA_strtok_s_get_error();
extern int Jio_032104_EA_jio_sprintf_s(char * __restrict dest, size_t dmax, const char * __restrict fmt, ...);
extern signed char Jio_032114_EA_memcmp_s(const void *p_cvp_dest, size_t p_sz_dmax, 
						const void *p_cvp_src, size_t p_sz_smax, int32_t *p_ip_diff);
extern void Jio_032115_handle_memcmp_s_error(int32_t p_i32_err_code, const char* p_cp_i8_func_name);
extern int8_t Jio_032116_EA_sprintf_dbglogs_s(char * __restrict dest, size_t dmax, uint16_t p_debug_level, const char * __restrict fmt, ...);
extern int8_t Jio_032117_EA_wo_null_memcpy_s(void* p_vp_dest, size_t p_sz_destSize, const void* p_cvp_src, size_t p_sz_srcSize);
extern int8_t Jio_EA_strncpy_s(char* p_vp_dest, size_t p_sz_destSize, const char* p_cvp_src, size_t p_sz_srcSize);

#define memcpynull_j Jio_032112_EA_memcpy_s
#define memcpy_j Jio_032117_EA_wo_null_memcpy_s
#define strlen_j Jio_032109_EA_strnlen_s
#define sscanf_j Jio_032105_EA_Jio_sscanf_s
#define strtok_j Jio_032108_EA_strtok_s
#define sprintf_j Jio_032104_EA_jio_sprintf_s
#define memcmp_j Jio_032114_EA_memcmp_s
#define memset_j Jio_032112_EA_memset_s
#define strcpy_j Jio_EA_strncpy_s


#ifdef __cplusplus
}
#endif


#endif
#endif /*_JIO_EA_Safe_FUNCTIONS_H_*/
