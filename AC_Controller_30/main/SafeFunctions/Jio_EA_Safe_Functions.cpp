/**
 * @file Jio_EA_Safe_Functions.cpp
 * @This file is used to Handle all safe function related activities such as mempy,
 *sprintf, sscanf and so on
 *
 * This file contains the implementation of functions related to the Reverse Command 
 *
 *  SL No.			Change Details					Ver				Date				Name
 *   1				 RefID : Baselined changes      00.01			11/10/2023			Janardhan
 *
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <inttypes.h>
/*
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <inttypes.h>
#include <errno.h>
#include <poll.h>
*/
#include "Jio_EA_Safe_Functions.h"
// #include "Jio_EA_FSM_Layer3/Jio_EA_DebugLogs_Handler/Jio_EA_Debug_Logs.h"
#include "main.h"
typedef int errno_t;
// int8_t g_i8_dbglogs[JIO_EA_DBG_PAYLOAD_BUFF_SIZE];
typedef void (*constraint_handler_t)(const char *__restrict /* msg */, void *__restrict /* ptr */, errno_t /* error */);
static  constraint_handler_t str_handler = NULL;
constraint_handler_t thrd_str_handler = NULL;
extern void JIO_031705_EA_AppUtils_getTimeInPrintFormat2(uint32_t,\
 unsigned char *);
extern uint32_t Jio_030112_EA_getEpochInSec(void);
extern int8_t Jio_031908_Store_dbglogs_buff(void);
/*
* Function - Jio_032101_ignore_handler_s
* @Paramin - msg - pointer to character string that describes the error
* @Paramin - ptr - pointer to an implementation-defined object or a null pointer.
* @Paramin - error - the error about to be returned by the calling function
*/
void Jio_032101_ignore_handler_s(const char *__restrict msg, void *__restrict ptr, errno_t error) {
    (void)ptr;
#ifndef DEBUG
    (void)msg;
    (void)error;
#endif
    printf("IGNORE CONSTRAINT HANDLER: (%d) %s\n", (int)error,(msg) ? msg : "Null message");
    return;
}

/*  Function - constraint_handler_t Jio_032102_EA_thrd_set_str_constraint_handler_s
*	Description - The set_str_constraint_handler_s function sets the runtime-constraint handler to be handler*	
*	@Return - handler
*/
constraint_handler_t Jio_032102_EA_thrd_set_str_constraint_handler_s(constraint_handler_t handler) {
    constraint_handler_t prev_handler = thrd_str_handler;
    if (NULL == handler) {
        thrd_str_handler = sl_default_handler;
    } else {
        thrd_str_handler = handler;
    }
    return prev_handler;
}

/*
*Function - Jio_032103_EA_invoke_safe_str_constraint_handler
*Description - Invokes the constraint handler or the default
*@Paramin - *msg	Pointer to the message describing the error
*@Paramin - *ptr	Pointer to aassociated data. Can be NULL.
*@Paramin - error	The error code encountered.
*@Return - None
*/

void Jio_032103_EA_invoke_safe_str_constraint_handler(const char *__restrict msg, void *__restrict ptr, errno_t error) {
    if (NULL != thrd_str_handler) {
        thrd_str_handler(msg, ptr, error);
    } else if (NULL != str_handler) {
        str_handler(msg, ptr, error);
    } else {
        sl_default_handler(msg, ptr, error);
    }
}

/*
*Function - Jio_032104_EA_jio_sprintf_s
*Description - Invokes the constraint handler or the default
*@Paramin - *dest - Pointer to Destination Buffer
*@Paramin - dmax - size dest buffer to be filled
*@Paramin - fmt - Format.
*@Return - None
*/


int Jio_032104_EA_jio_sprintf_s(char * __restrict dest, size_t dmax, const char * __restrict fmt, ...)
 {
	 //#define SPRINTF_S_DEBUG

     va_list ap;
     int ret = -1;
     if (dmax > RSIZE_MAX_STR) {
         Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dmax exceeds max", NULL, ESLEMAX);
         return RCNEGATE(ESLEMAX);
     }
     if (dest == NULL) {
         Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dest is null", NULL, ESNULLP);
         return RCNEGATE(ESNULLP);
     }
     if (fmt == NULL) {
         Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: fmt is null", NULL, ESNULLP);
         return RCNEGATE(ESNULLP);
     }
     if (dmax == 0) {
         Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dmax is 0", NULL, ESZEROL);
         return RCNEGATE(ESZEROL);
     }
	 #ifdef SPRINTF_S_DEBUG
		printf("dest - %s\n",dest);
	 #endif
     va_start(ap, fmt);
     ret = vsnprintf(dest, (size_t)dmax, fmt, ap);
     if (ret > (int)dmax) {
		 
		#ifdef SPRINTF_S_DEBUG
		 printf("ret = %d\n dmax = %d\n",ret,dmax);
		 #endif
         Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: len exceeds dmax", NULL, ESNOSPC);
         *dest = 0;
         ret = RCNEGATE(ESNOSPC);
     }
     va_end(ap); 
     return ret;
 }

int Jio_032105_EA_Jio_sscanf_s(const char *__restrict buffer, const char *__restrict fmt, ...) {
    va_list ap;
    int ret;
	///printf("Inside Jio_032105_EA_Jio_sscanf_s\n");
    if (UNLIKELY(buffer == NULL)) {
        printf("buffer is null\n");
        Jio_032103_EA_invoke_safe_str_constraint_handler(" buffer is null", NULL, ESNULLP);
        errno = ESNULLP;
        return EOF;
    }

    if (UNLIKELY(fmt == NULL)) {
        printf("Format is null\n");
        Jio_032103_EA_invoke_safe_str_constraint_handler(" fmt is null", NULL, ESNULLP);
        errno = ESNULLP;
        return EOF;
    }

    errno = 0;
    va_start(ap, fmt);
    ret = vsscanf(buffer, fmt, ap);
    va_end(ap);

    if (UNLIKELY(ret < 0)) { /* always -1 EOF */
        char errstr[128] = "sscanf_s: ";
		size_t l_sz_buff_remaining_size;
		/*
		KW-00.04.41.08
		KW-PBM -	─$ grep -ir "strcat("
				     strcat(errstr, strerror(errno));
		AKN TKN - replaced strcat with Jio_032112_EA_memcpy_s
		*/
        //strcat(errstr, strerror(errno));
		l_sz_buff_remaining_size = sizeof(errstr) - Jio_032109_EA_strnlen_s(errstr, sizeof(errstr));
		ret = Jio_032112_EA_memcpy_s(&errstr[Jio_032109_EA_strnlen_s("sscanf_s: ", sizeof("sscanf_s: "))], l_sz_buff_remaining_size, strerror(errno), Jio_032110_EA_Get_String_Size(strerror(errno))); 
        Jio_032113_EA_handle_memcpy_s_error(ret, "Jio_sscanf_s memcpy error");
		Jio_032103_EA_invoke_safe_str_constraint_handler(errstr, NULL, errno);
    }
    return ret;
}

// Error indicator to store the error status
static Jio_EA_StrtokSErrorCode strtok_s_error = STRTOK_S_SUCCESS;

// Function to get the error status
Jio_EA_StrtokSErrorCode Jio_032106_EA_strtok_s_get_error() {
    return strtok_s_error;
}

// Reset the error indicator to success
void Jio_032107_EA_strtok_s_reset_error() {
    strtok_s_error = STRTOK_S_SUCCESS;
}

/**
 * @brief Custom implementation of strtok_s function.
 *
 * This function tokenizes a string based on a set of delimiters.
 * It returns a pointer to the next token in the string or NULL if no more tokens are found.
 *
 * @param p_cp_dest Pointer to the destination string or NULL to continue from the previous context.
 * @param p_szp_dmax Pointer to the size of the destination string.
 * @param p_ccp_src Pointer to the source string.
 * @param p_cpp_ptr Pointer to a context pointer used to store the position within the source string.
 * @return Pointer to the next token in the string or NULL if no more tokens are found.
 */
char *Jio_032108_EA_strtok_s(char *p_cp_dest, size_t *p_szp_dmax, const char *p_ccp_src, char **p_cpp_ptr)
{
    const char *l_ccp_pt;     // Pointer to iterate over the delimiter string
    char *l_cp_ptoken;       // Pointer to the beginning of a token
    size_t l_sz_dlen;        // Length of the destination string
    size_t l_sz_slen;        // Length of the delimiter string
	
	Jio_032107_EA_strtok_s_reset_error();
	
    if (p_szp_dmax == NULL) {
        printf("strtok_s: dmax is NULL\n");
		strtok_s_error = STRTOK_S_NULL_DMAX;
        return (NULL);
    }

    if (*p_szp_dmax == 0) {
        //printf("strtok_s: dmax is 0\n");
		//strtok_s_error = STRTOK_S_ZERO_DMAX;
        return (NULL);
    }

    if (p_ccp_src == NULL) {
        printf("strtok_s: src is null\n");
		strtok_s_error = STRTOK_S_NULL_SRC;
        return (NULL);
    }

    if (p_cpp_ptr == NULL) {
        printf("strtok_s: context ptr is null\n");
		strtok_s_error = STRTOK_S_NULL_PTR;
        return (NULL);
    }

    // If dest is NULL, continue from the previous context 
    //--> if the source was NULL, use the tokenizer context
    if (p_cp_dest == NULL)
        p_cp_dest = *p_cpp_ptr;

    // Scan dest for a delimiter
    l_sz_dlen = *p_szp_dmax;
    l_cp_ptoken = NULL;

    while (*p_cp_dest != '\0' && !l_cp_ptoken) {
        if (l_sz_dlen == 0) {
            *p_cpp_ptr = NULL;
			printf("strtok_s: dest is unterminated\n");
			strtok_s_error = STRTOK_S_DEST_UNTERMINATED;
            return (NULL);
        }

        // Must scan the entire delimiter list. ISO should have included a delimiter string limit!!
        l_sz_slen = STRTOK_DELIM_MAX_LEN;
        l_ccp_pt = p_ccp_src;

        while (*l_ccp_pt != '\0') {
            if (l_sz_slen == 0) {
                *p_cpp_ptr = NULL;
                printf("strtok_s: src is unterminated\n");
				strtok_s_error = STRTOK_S_SRC_UNTERMINATED;
                return (NULL);
            }
            l_sz_slen--;

            if (*p_cp_dest == *l_ccp_pt) {
                l_cp_ptoken = NULL;
                break;
            } else {
                l_ccp_pt++;
                l_cp_ptoken = p_cp_dest;
            }
        }

        p_cp_dest++;
        l_sz_dlen--;
    }

    // If the beginning of a token was not found, then no need to continue the scan.
    if (l_cp_ptoken == NULL) {
        *p_szp_dmax = l_sz_dlen;
        return (l_cp_ptoken);
    }

    // Now we need to locate the end of the token
    while (*p_cp_dest != '\0') {
        if (l_sz_dlen == 0) {
            *p_cpp_ptr = NULL;
            //printf("strtok_s: dest is unterminated\n");
			strtok_s_error = STRTOK_S_DEST_UNTERMINATED;
            return (NULL);
        }

        l_sz_slen = STRTOK_DELIM_MAX_LEN;
        l_ccp_pt = p_ccp_src;

        while (*l_ccp_pt != '\0') {
            if (l_sz_slen == 0) {
                *p_cpp_ptr = NULL;
                printf("strtok_s: src is unterminated\n");
				strtok_s_error = STRTOK_S_SRC_UNTERMINATED;
                return (NULL);
            }
            l_sz_slen--;

            if (*p_cp_dest == *l_ccp_pt) {
                // Found a delimiter, set to null and return context ptr to next char
                *p_cp_dest = '\0';
                *p_cpp_ptr = (p_cp_dest + 1);  // Return pointer for next scan
                *p_szp_dmax = l_sz_dlen - 1;   // Account for the nulled delimiter
                return (l_cp_ptoken);
            } else {
                // Simply scanning through the delimiter string
                l_ccp_pt++;
            }
        }

        p_cp_dest++;
        l_sz_dlen--;
    }

    *p_szp_dmax = l_sz_dlen;
    return (l_cp_ptoken);
}

/**
 * @brief Calculates the length of a string, limiting the count to the provided size.
 *
 * @param p_ccp_rxstr The string to calculate the length of.
 * @param p_i_rxstr_sz The maximum size of the string.
 * @return The length of the string within the provided size.
 */
size_t Jio_032109_EA_strnlen_s(const char *p_ccp_rxstr, size_t p_i_rxstr_sz)
{
    int l_i_count = 0;

    /// Check received string Pointer should not be NULL
    if (p_ccp_rxstr == NULL) {
        printf("p_ccp_rxstr is NULL\n");
        return -7;
    }

    // Check the received string size should not be zero
    if (p_i_rxstr_sz == 0) {
        printf("Received str size is zero.\n");
        return -8;
    }

    /*This ensures that the loop continues as long as both rxstr_sz and the
    character pointed to by rxstr are non-zero.*/
    while (p_i_rxstr_sz && *p_ccp_rxstr)
    {
        l_i_count++;
        p_i_rxstr_sz--;
        p_ccp_rxstr++;
    }

    //printf("strnlen_s: %u\n", l_i_count);
    return l_i_count;
}

/**
 * @brief Gets the size of a null-terminated string.
 *
 * @param p_ccp_str The string to determine the size of.
 * @return The size of the string including the null terminator.
 */
size_t Jio_032110_EA_Get_String_Size(const char* p_ccp_str) {
    const char* l_ccp_ptr = p_ccp_str;
    size_t l_sz_size = 0;

    if(p_ccp_str == NULL){
        printf("Error: Input string pointer is NULL.\n");
        return -6;
    }
	
    while (*l_ccp_ptr != '\0') {
        l_sz_size++;
        l_ccp_ptr++;
    }

    return l_sz_size+1;
}

void Jio_032111_EA_handle_strnlen_s_error(int ret, const char* p_func_name) {
     if (ret != 0) {
        printf("\n Jio_032111_EA_handle_strnlen_s_error failed with error (error code = %d), p_func_name = %s\n", ret, (const char*)p_func_name);
        // Handle the error accordingly, including using the p_info_value if needed
        // For example, you can print or manipulate p_info_value here.
    } else {
        printf("\n Jio_032111_EA_handle_strnlen_s_error successful %s\n", p_func_name);
        // Continue with the rest of the program
    }
}

/**
 * @brief Copies memory from source to destination, ensuring the destination size is not exceeded.
 *
 * @param p_vp_dest The destination memory address.
 * @param p_i_destSize The size of the destination memory.
 * @param p_cvp_src The source memory address.
 * @param p_i_srcSize The number of bytes to copy from the source.
 * @return 0 if successful, otherwise an error code.
 */

int8_t Jio_032112_EA_memcpy_s(void* p_vp_dest, size_t p_sz_destSize, const void* p_cvp_src, size_t p_sz_srcSize)
{
   /// printf("p_sz_destSize:%zu p_sz_srcSize:%zu\n", p_sz_destSize, p_sz_srcSize);
    if (p_vp_dest == NULL || p_cvp_src == NULL)
    {
        printf("Invalid memory pointers.\n");
        return -1;

    }

    if (p_sz_destSize == 0)
    {
        printf("Destination size is zero.\n");
        return -2;
    }

    if(p_sz_destSize < p_sz_srcSize){
        printf("p_sz_destSize:%zu p_sz_srcSize:%zu\n", p_sz_destSize, p_sz_srcSize);
        printf("Destination size is less than source.\n");
        return -3;
    }


   if (p_sz_srcSize == 0)
    {
        printf("Number of bytes to copy is zero.\n");
        return -4;
    }

    unsigned char* l_ucp_dp = (unsigned char*)p_vp_dest;
    const unsigned char* l_cucp_sp = (const unsigned char*)p_cvp_src;
    /*
     * overlap is undefined behavior, do not allow
     */
    if (((l_ucp_dp > l_cucp_sp) && (l_ucp_dp < l_cucp_sp + p_sz_srcSize)) || ((l_cucp_sp > l_ucp_dp) && (l_cucp_sp < l_ucp_dp + p_sz_srcSize)))
    {
        printf("Overlapping memory regions.\n");
        return -5;
    }
    
	size_t i = 0;
    for (i = 0; i < p_sz_srcSize; i++)
    {
        l_ucp_dp[i] = l_cucp_sp[i];
    }

    return 0;
}

/**
 * @brief Safe String Copy
 * @param p_vp_dest     Destination buffer
 * @param p_sz_destSize Total size of destination buffer (including room for \0)
 * @param p_cvp_src     Source string
 * @param p_sz_srcSize  Maximum bytes to copy from source
 * @return 0 on success, negative error code on failure
 */
int8_t Jio_EA_strncpy_s(char* p_vp_dest, size_t p_sz_destSize, const char* p_cvp_src, size_t p_sz_srcSize)
{
    // 1. Basic Sanity Checks
    if (p_vp_dest == NULL || p_cvp_src == NULL) {
        return -1; // Invalid pointers
    }

    if (p_sz_destSize == 0) {
        return -2; // Nowhere to copy to
    }

    // 2. Determine how much we CAN copy
    // We must leave at least 1 byte for the null terminator
    size_t l_sz_maxCopy = p_sz_destSize - 1;
    size_t l_sz_actualCopy = (p_sz_srcSize < l_sz_maxCopy) ? p_sz_srcSize : l_sz_maxCopy;

    // 3. Perform the copy
    size_t i;
    for (i = 0; i < l_sz_actualCopy && p_cvp_src[i] != '\0'; i++) {
        p_vp_dest[i] = p_cvp_src[i];
    }

    // 4. THE CRITICAL STEP: Always Null Terminate
    // This prevents the "garbage character" logs you were seeing
    p_vp_dest[i] = '\0';

    // 5. Diagnostic (Optional: check if we truncated data)
    if (i < p_sz_srcSize && p_cvp_src[i] != '\0') {
        // This means the source was longer than the destination could hold
        // In an AC Controller, this might mean a truncated MQTT Topic or SSID
        return -3; 
    }

    return 0;
}

/**
 * @brief Copies memory from source to destination, ensuring the destination size is not exceeded.
 *
 * @param p_vp_dest The destination memory address.
 * @param p_i_destSize The size of the destination memory.
 * @param i_val value to be set in all loc.
 * @param p_i_srcSize The number of bytes to copy from the source.
 * @return 0 if successful, otherwise an error code.
 */
int8_t Jio_032112_EA_memset_s(void* p_vp_dest, size_t p_sz_destSize, const int i_val, size_t p_sz_srcSize)
{
   /// printf("p_sz_destSize:%zu p_sz_srcSize:%zu\n", p_sz_destSize, p_sz_srcSize);
    if (p_vp_dest == NULL)
    {
        printf("Invalid memory pointer.\n");
        return -1;

    }

    if (p_sz_destSize == 0)
    {
        printf("Destination size is zero.\n");
        return -2;
    }

    if(i_val > 255){
        printf("Set value greater than 255.\n");
        return -3;
    }


   if (p_sz_srcSize == 0)
    {
        printf("Number of bytes to copy is zero.\n");
        return -4;
    }

    unsigned char* l_ucp_dp = (unsigned char*)p_vp_dest;
    unsigned char c_val = (unsigned char )i_val;

	size_t i = 0;
    for (i = 0; i < p_sz_srcSize; i++)
    {
        l_ucp_dp[i] = c_val;
    }

    return 0;
}

/**
 * @brief Copies memory from source to destination, ensuring the destination size is not exceeded.
 *
 * @param p_vp_dest The destination memory address.
 * @param p_i_destSize The size of the destination memory.
 * @param p_cvp_src The source memory address.
 * @param p_i_srcSize The number of bytes to copy from the source.
 * @return 0 if successful, otherwise an error code.
 */
int8_t Jio_032117_EA_wo_null_memcpy_s(void* p_vp_dest, size_t p_sz_destSize, const void* p_cvp_src, size_t p_sz_srcSize)
{
   /// printf("p_sz_destSize:%zu p_sz_srcSize:%zu\n", p_sz_destSize, p_sz_srcSize);
    if (p_vp_dest == NULL || p_cvp_src == NULL)
    {
        printf("Invalid memory pointers.\n");
        return -1;

    }

    if (p_sz_destSize == 0)
    {
        printf("Destination size is zero.\n");
        return -2;
    }

    if(p_sz_destSize < p_sz_srcSize){
        printf("p_sz_destSize:%zu p_sz_srcSize:%zu\n", p_sz_destSize, p_sz_srcSize);
        printf("Destination size is less than source.\n");
        return -3;
    }


   if (p_sz_srcSize == 0)
    {
        printf("Number of bytes to copy is zero.\n");
        return -4;
    }

    unsigned char* l_ucp_dp = (unsigned char*)p_vp_dest;
    const unsigned char* l_cucp_sp = (const unsigned char*)p_cvp_src;
    /*
     * overlap is undefined behavior, do not allow
     */
    if (((l_ucp_dp > l_cucp_sp) && (l_ucp_dp < l_cucp_sp + p_sz_srcSize)) || ((l_cucp_sp > l_ucp_dp) && (l_cucp_sp < l_ucp_dp + p_sz_srcSize)))
    {
        printf("Overlapping memory regions.\n");
        return -5;
    }
    
	size_t i = 0;
    for (i = 0; i < p_sz_srcSize; i++)
    {
        l_ucp_dp[i] = l_cucp_sp[i];
    }

    //l_ucp_dp[i] = '\0';
    //printf("l_ucp_dp: %s", l_ucp_dp);
    return 0;
}
// Function for handling Jio_032112_EA_memcpy_s error with general p_info_value parameter
void Jio_032113_EA_handle_memcpy_s_error(int8_t ret, const char* p_func_name) {

	if (ret != 0) {
        printf("\n Jio_032112_EA_memcpy_s failed with error (error code = %d), p_func_name = %s\n", ret, (const char*)p_func_name);
        // Handle the error accordingly, including using the p_info_value if needed
        // For example, you can print or manipulate p_info_value here.
        //printf("++++++++++++++Killed the process bcz memcpy failed:\n");
		//system("killall sig.exe");
		//Jio_032116_EA_sprintf_dbglogs_s(g_cb_print_buffer,sizeof(g_cb_print_buffer),l_u8_debug_level,"Jio_032112_EA_memcpy_s failed with error (error code = %d), p_func_name = %s\n", ret, (const char*)p_func_name);
	} else {
       // printf("\n Jio_032112_EA_memcpy_s successful %s\n", p_func_name);
        // Continue with the rest of the program
    }
}

/**
 * NAME
 *    memcmp_s - Compares memory
 *
 * DESCRIPTION
 *    Compares memory until they differ, and their difference is
 *    returned in diff.  If the block of memory is the same, diff=0.
 *
 * INPUT PARAMETERS
 *    dest      pointer to memory to compare against
 *    dmax      maximum length of dest, in bytess
 *    src       pointer to the source memory to compare with dest
 *    smax      length of the source memory block
 *    *diff     pointer to the diff which is an integer greater
 *              than, equal to or less than zero according to
 *              whether the object pointed to by dest is
 *              greater than, equal to or less than the object
 *              pointed to by src.
 *
 *  OUTPUT PARAMETERS
 *    none
 *	RETURN VALUE
 *    EOK        successful operation
 *    
 *	EOK	when operation is successful
 */
 
signed char Jio_032114_EA_memcmp_s(const void *p_cvp_dest, size_t p_sz_dmax, 
						const void *p_cvp_src, size_t p_sz_smax, int32_t *p_ip_diff)
{
    const uint8_t *l_cu8_destptr;
    const uint8_t *l_cu8_srcptr;

    l_cu8_destptr = (const uint8_t*)p_cvp_dest;
    l_cu8_srcptr = (const uint8_t*)p_cvp_src;

    /*
     * must be able to return the p_ip_diff
     */
    if (p_ip_diff == NULL) {
        
        return -9;
    }
    *p_ip_diff = -1;  /* default p_ip_diff */

    if (l_cu8_destptr == NULL) {
        
        return -10;
    }

    if (l_cu8_srcptr == NULL) {
        
        return -11;
    }

    if (p_sz_dmax == 0) {
        
        return -12;
    }

    if (p_sz_dmax > SIZE_MAX) {
        
        return -13;
    }

    if (p_sz_smax == 0) {
       
        return -14;
    }

    if (p_sz_smax > p_sz_dmax) {
       
        return -15;
    }

    /*
     * no need to compare the same memory
     */
    if (l_cu8_destptr == l_cu8_srcptr) {
        *p_ip_diff = 0;
        return EOK;
    }

    /*
     * now compare l_cu8_srcptr to l_cu8_destptr
     */
    *p_ip_diff = 0;
    while (p_sz_dmax > 0 && p_sz_smax > 0) {
        if (*l_cu8_destptr != *l_cu8_srcptr) {
            /***  *p_ip_diff = *l_cu8_destptr - *l_cu8_srcptr; ***/
            *p_ip_diff = *l_cu8_destptr < *l_cu8_srcptr ? -1 : 1;
            break;
        }

        p_sz_dmax--;
        p_sz_smax--;

        l_cu8_destptr++;
        l_cu8_srcptr++;
    }

    return EOK;
}

// Function to handle error messages
void Jio_032115_handle_memcmp_s_error(int32_t p_i32_err_code, const char* p_cp_i8_func_name) {
    switch (p_i32_err_code) 
	{
        case -9:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: p_ip_diff pointer is null in p_func_name = %s\n", 
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        case -10:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: dest pointer is null in p_func_name = %s\n", 
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        case -11:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: src pointer is null in p_func_name = %s\n", 
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        case -12:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: p_sz_dmax is 0 p_func_name = %s\n",
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        case -13:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: p_sz_dmax exceeds the maximum allowed memory size \
                         in p_func_name = %s\n", (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        case -14:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: p_sz_smax is 0 in p_func_name = %s\n", 
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        case -15:
             printf("\nJio_EA_memcmp_s failed with error code = %ld: p_sz_smax exceeds p_sz_dmax in p_func_name = %s\n", 
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
        default:
            printf("\nJio_EA_memcmp_s failed with error code = %ld: Unknown error occurred in p_func_name = %s\n", 
                    (long)p_i32_err_code, (const char*)p_cp_i8_func_name);
            break;
    }
}

/*
*Function - //Jio_032116_EA_sprintf_dbglogs_s
*Description - Invokes the constraint handler or the default
*@Paramin - *dest - Pointer to Destination Buffer
*@Paramin - dmax - size dest buffer to be filled
*@Paramin - fmt - Format.
*@Return - None
*/
// int8_t Jio_032116_EA_sprintf_dbglogs_s(char * __restrict dest, size_t dmax, uint16_t p_debug_level, const char * __restrict fmt, ...)
//  {
// 	 //#define SPRINTF_S_DEBUG
// 	 int8_t l_i8_buffer[50] = {0};
// 	 int8_t l_i8_ret = -1;
// 	 size_t l_sz_string_len;
// 	 p_debug_level = 0;
//      va_list ap;
//      int ret = -1;
// 	 #ifdef SPRINTF_S_DEBUG
// 	 printf("Jio_032116_EA_sprintf_dbglogs_s - Entry Point\n");
// 	 #endif
//      if (dmax > RSIZE_MAX_STR) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dmax exceeds max", NULL, ESLEMAX);
//          return RCNEGATE(ESLEMAX);
//      }
//      if (dest == NULL) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dest is null", NULL, ESNULLP);
//          return RCNEGATE(ESNULLP);
//      }
//      if (fmt == NULL) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: fmt is null", NULL, ESNULLP);
//          return RCNEGATE(ESNULLP);
//      }
//      if (dmax == 0) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dmax is 0", NULL, ESZEROL);
//          return RCNEGATE(ESZEROL);
//      }
	 
//      va_start(ap, fmt);
//      ret = vsnprintf(dest, (size_t)dmax, fmt, ap);
//      if (ret > (int)dmax) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: len exceeds dmax", NULL, ESNOSPC);
//          *dest = 0;
//          ret = RCNEGATE(ESNOSPC);
//      }
//      va_end(ap); 
	 
// 	 if(ret > -1)
// 	 {
// 		 //printf("Jio_032116 : ret > -1\n");
// 		 //()Call a function that pushes log in to circular debug buffer 
// 		 memset(&g_i8_dbglogs[0], 0x00, sizeof(g_i8_dbglogs));
// 		 memset(&g_i8_dbglogs[0], ' ', (sizeof(g_i8_dbglogs) - 1));
// 		 memset(&l_i8_buffer[0], 0x00, sizeof(l_i8_buffer));
		 
// 		 JIO_031705_EA_AppUtils_getTimeInPrintFormat2(
// 		 Jio_030112_EA_getEpochInSec() + 19800, (unsigned char*)l_i8_buffer);
		 
// 		 l_sz_string_len = Jio_032109_EA_strnlen_s((const char*)l_i8_buffer,
// 		 sizeof(l_i8_buffer));
// 		 if(l_sz_string_len <= 0)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()At this point of time focus is on handling cond l_i8_ret> 0 
// 		 }
// 		 //memcpy(&g_i8_dbglogs[0],&l_i8_buffer[0],strlen((const char*)l_i8_buffer));
// 		 #if 1
// 		 l_i8_ret = Jio_032112_EA_memcpy_s(&g_i8_dbglogs[0], sizeof(g_i8_dbglogs),
// 		 l_i8_buffer, l_sz_string_len);
// 		 #endif
		 
// 		 if(l_i8_ret < 0)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()At this point of time focus is on handling cond l_i8_ret> 0 
// 		 }
// 		 //memcpy(&g_i8_dbglogs[strlen((const char*)l_i8_buffer)], dest, ret);
// 		 //()NLJ: 05/09/2023 - Have a logic to defragment debug logs in to debug buffer
// 		 //()if strlen of dest is greater than 40 bytes, break that chunk in to multiple
// 		 //()chunks of size (multiple of 40 bytes)
// 		 #if 1
// 		 l_i8_ret = Jio_032112_EA_memcpy_s(&g_i8_dbglogs[l_sz_string_len],
// 		 (sizeof(g_i8_dbglogs) - l_sz_string_len), dest, ret);
		 
// 		 if(l_i8_ret < 0)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()At this point of time focus is on handling cond l_i8_ret> 0 
// 			 return l_i8_ret;
// 		 }
		 
// 		 #endif
// 		 //memset(&g_cb_print_buffer[0], 0x00, sizeof(g_cb_print_buffer));
// 		 if(Jio_031908_Store_dbglogs_buff() == FAILURE)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()NLJ : 07/09/2023 - Send a diag packet
// 		 }
// 		 else
// 		 {
// 			 #ifdef SPRINTF_S_DEBUG
// 				printf("DbgLogs - %s\n",g_i8_dbglogs);
// 			 #endif
// 		 }
// 	 }
	 
// 	 memset(dest, 0x00, sizeof(g_cb_print_buffer)); 
// 	 return ret;
//  }
// #if 0
// int8_t Jio_032116_EA_sprintf_dbglogs_s(char * __restrict dest, size_t dmax, uint16_t p_debug_level, const char * __restrict fmt, ...)
//  {
// 	 #define SPRINTF_S_DEBUG
// 	 int8_t l_i8_buffer[50] = {0};
// 	 int8_t l_i8_ret = -1;
// 	 size_t l_sz_string_len;
// 	 p_debug_level = 0;
//      va_list ap;
//      int ret = -1;
//      if (dmax > RSIZE_MAX_STR) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dmax exceeds max", NULL, ESLEMAX);
//          return RCNEGATE(ESLEMAX);
//      }
//      if (dest == NULL) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dest is null", NULL, ESNULLP);
//          return RCNEGATE(ESNULLP);
//      }
//      if (fmt == NULL) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: fmt is null", NULL, ESNULLP);
//          return RCNEGATE(ESNULLP);
//      }
//      if (dmax == 0) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: dmax is 0", NULL, ESZEROL);
//          return RCNEGATE(ESZEROL);
//      }
	 
//      va_start(ap, fmt);
//      ret = vsnprintf(dest, (size_t)dmax, fmt, ap);
//      if (ret > (int)dmax) {
//          Jio_032103_EA_invoke_safe_str_constraint_handler("sprintf_s: len exceeds dmax", NULL, ESNOSPC);
//          *dest = 0;
//          ret = RCNEGATE(ESNOSPC);
//      }
//      va_end(ap); 
	 
// 	 if(ret > -1)
// 	 {
// 		 //printf("Jio_032116 : ret > -1\n");
// 		 //()Call a function that pushes log in to circular debug buffer 
// 		 memset(&g_i8_dbglogs[0], 0x00, sizeof(g_i8_dbglogs));
// 		 memset(&l_i8_buffer[0], 0x00, sizeof(l_i8_buffer));
		 
// 		 JIO_031705_EA_AppUtils_getTimeInPrintFormat2(
// 		 Jio_030112_EA_getEpochInSec() + 19800, (unsigned char*)l_i8_buffer);
		 
// 		 l_sz_string_len = Jio_032109_EA_strnlen_s((const char*)l_i8_buffer,
// 		 sizeof(l_i8_buffer));
// 		 if(Jio_032109_EA_strnlen_s <= 0)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()At this point of time focus is on handling cond l_i8_ret> 0 
// 		 }
// 		 //memcpy(&g_i8_dbglogs[0],&l_i8_buffer[0],strlen((const char*)l_i8_buffer));
// 		 #if 1
// 		 l_i8_ret = Jio_032112_EA_memcpy_s(&g_i8_dbglogs[0], sizeof(g_i8_dbglogs),
// 		 l_i8_buffer, l_sz_string_len);
// 		 #endif
		 
// 		 if(l_i8_ret < 0)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()At this point of time focus is on handling cond l_i8_ret> 0 
// 		 }
// 		 //memcpy(&g_i8_dbglogs[strlen((const char*)l_i8_buffer)], dest, ret);
// 		 //()NLJ: 05/09/2023 - Have a logic to defragment debug logs in to debug buffer
// 		 //()if strlen of dest is greater than 40 bytes, break that chunk in to multiple
// 		 //()chunks of size (multiple of 40 bytes)
// 		 #if 1
// 		 l_i8_ret = Jio_032112_EA_memcpy_s(&g_i8_dbglogs[l_sz_string_len],
// 		 (sizeof(g_i8_dbglogs) - l_sz_string_len), dest, ret);
		 
// 		 if(l_i8_ret < 0)
// 		 {
// 			 //()Need to define further course of action.
// 			 //()At this point of time focus is on handling cond l_i8_ret> 0 
// 		 }
// 		 #endif
// 		 memset(&g_cb_print_buffer[0], 0x00, sizeof(g_cb_print_buffer));
// 		 if(Jio_031908_Store_dbglogs_buff() == FAILURE)
// 		 {
// 			 //()Need to define further course of action.
// 		 }
// 		 else
// 		 {
// 			 #ifdef SPRINTF_S_DEBUG
// 				printf("%s\n",g_i8_dbglogs);
// 			 #endif
// 		 }
// 	 }
// 	 memset(dest, 0x00, sizeof(g_cb_print_buffer)); 
// 	 return ret;
//  }
//  #endif
