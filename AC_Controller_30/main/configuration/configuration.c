#include "main.h"
#include <unistd.h>
#include <math.h>       /* BUG-FIX #20: isfinite() for NaN/Inf rejection */
#include "cJSON.h"
#include <stdio.h>
#include "esp_spiffs.h"
#include <sys/stat.h>
#include "esp_log.h"
#include "configuration.h"
#include <sys/socket.h>
#include "xmodem.h"
#include "http.h"
// Dynamic global storage
// Global Pointers

ip_config_t *ip_config = NULL;
int total_ip_configs=0;

hlt_config_t *hlt_configs = NULL;
int total_hlt_configs = 0;

mbr_config_t *system_configs = NULL;
int total_configs = 0;

urt_config_t *system_urts = NULL;
int total_urts = 0;

profile_config_t *profile_parms = NULL;
int total_profiles = 0;

profile_rdn_t *profile_rdn_parms;
int total_rdn_profiles;
int rdn_data_ready=0;

uint32_t *last_poll_times = NULL;

uint32_t *last_push_min = NULL;

bool *mbr_due_flags = NULL;

char *config_path = "/spiffs/config.txt";

static const char *TAG = "SYS_INIT";
esp_err_t delete_config_file(const char* path) {
    // Check if file exists first
    struct stat st;
    if (stat(path, &st) != 0) {
        ESP_LOGW("SPIFFS", "File %s does not exist, nothing to delete.", path);
        return ESP_ERR_NOT_FOUND;
    }

    // unlink is the standard C function to delete a file
    if (unlink(path) == 0) {
        ESP_LOGI("SPIFFS", "File %s deleted successfully.", path);
        return ESP_OK;
    } else {
        ESP_LOGE("SPIFFS", "Failed to delete file %s.", path);
        return ESP_FAIL;
    }
}

// Move this helper function ABOVE save_config_json
static void add_json_string_as_object(cJSON *root, const char *key, const char *json_str) {
    cJSON *temp_obj = cJSON_Parse(json_str);
    if (temp_obj) {
        cJSON_AddItemToObject(root, key, temp_obj);
    } else {
        // Fallback: if it's not valid JSON, add it as a plain string
        cJSON_AddStringToObject(root, key, json_str);
    }
}

void save_config_json() {
    // 1. Create the root object
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return;
    }
    
    // 2. Add the nested values using our C helper function
    add_json_string_as_object(root, "hlt", "{\"0\":{\"dhlt1\":\"900:900\"}}");
    add_json_string_as_object(root, "urt", "{\"0\":{\"curt0\":\"0:9600:0:8:1:1:600:0:1:0:0:1:0\"}}");
    add_json_string_as_object(root, "ipc",
        "{\"0\":{\"static_ip\":\"192.168.1.0\",\"gw\":\"192.168.1.1\",\"nm\":\"255.255.255.0\","
        "\"mb_port\":502,\"pt_port\":503,\"http_port\":80,\"timeout\":60,\"pt_retry\":0,\"static_en\":1}}");
    add_json_string_as_object(root, "mbr", "{\"0\":{\"dmbr1\":\"1:20:300:300:tpa:DEFAULT:192.168.1.100:502\"}}");
    add_json_string_as_object(root, "ota", "{\"0\":{\"ota0\":\"10.63.107.102:0:Version2.bin:0\"}}");
    add_json_string_as_object(root, "dmbr1", "{\"vrp\":\"5:0:14:0:4:0:0:0:1.0:0:0\"}");

    // 3. Convert object to string
    char *json_string = cJSON_PrintUnformatted(root);

    if (json_string != NULL) {
        // 4. Write to SPIFFS
        FILE* f = fopen("/spiffs/config.txt", "w");
        if (f != NULL) {
            fputs(json_string, f);
            fclose(f);
            ESP_LOGI("CONFIG", "Config saved to SPIFFS");
        }
        free(json_string);
    }

    // 5. Clean up
    cJSON_Delete(root);
}

char* read_config_to_buffer(const char* path) {
    struct stat st;

    // 1. Get file status to find the size
    if (stat(path, &st) != 0) {
        ESP_LOGE("STORAGE", "File %s not found", path);
        return NULL;
    }

    // 2. Open file for reading
    FILE* f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE("STORAGE", "Failed to open file");
        return NULL;
    }

    // 3. Allocate memory (+1 for the null terminator '\0')
    char* buffer = malloc(st.st_size + 1);
    if (buffer == NULL) {
        ESP_LOGE("STORAGE", "Memory allocation failed for buffer");
        fclose(f);
        return NULL;
    }

    // 4. Read data from SPIFFS into RAM
    size_t read_bytes = fread(buffer, 1, st.st_size, f);
    buffer[read_bytes] = '\0'; // Crucial for cJSON to know where string ends

    fclose(f);
    ESP_LOGI("STORAGE", "Read %d bytes from %s", read_bytes, path);

    return buffer; // Remember to free() this later in app_main!
}

void parse_mbr_string(char* input, mbr_config_t* mbrconfig) {

    char* token;
    int index = 0;
    char ip_address[16];

    if (input == NULL || mbrconfig == NULL) {
    	printf("Null Characters");
    	return;
    }

    char *orig = strdup(input);
    if (orig == NULL) return;
    char *rest = orig; /* strtok_r will advance rest; free orig at the end */

    // Initialize strings to empty to ensure no garbage is printed if parsing stops early
    memset_j(mbrconfig->Future_use,sizeof(mbrconfig->Future_use), 0, sizeof(mbrconfig->Future_use));
    memset_j(mbrconfig->Model_no, sizeof(mbrconfig->Model_no), 0, sizeof(mbrconfig->Model_no));
    memset_j((char *)&mbrconfig->poll_ip, sizeof(mbrconfig->poll_ip), 0, sizeof(mbrconfig->poll_ip));

    mbrconfig->modbus_port = 502;            // Default Modbus Port

    while ((token = strtok_r(rest, ":", &rest))) {
        switch(index) {
	    // 1. Slave ID
            case 0: mbrconfig->slave_id = atoi(token); break;
	    // 2. Poll Frequency
            case 1: mbrconfig->poll_frq = atoi(token); break;

            // 3. Push Frequency M
            case 2: mbrconfig->push_frq_m = atoi(token); break;
            // 4. Push Frequency B
            case 3: mbrconfig->push_frq_b = atoi(token); break;
            // 5. Future Use / Device Tag
            case 4:  strcpy_j(mbrconfig->Future_use, MODEL_NO_MAX_LEN, token, strlen(token));
             break;

            // 6. Model No
            case 5:
            strcpy_j(mbrconfig->Model_no, MODEL_NO_MAX_LEN, token, strlen(token));
            break;
            // Optional IP
            case 6:

            strncpy(ip_address, token, sizeof(ip_address) - 1);
            inet_pton(AF_INET, ip_address, &mbrconfig->poll_ip.addr);
            break;

            case 7: // Optional Port
            mbrconfig->modbus_port = atoi(token);
            break;

        }
        index++;
    }
    free(orig); /* free the original strdup allocation, not the strtok_r-modified pointer */
}

bool file_exists(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return true; // File exists
    }
    return false; // File does not exist
}


// void parse_mbr_string(const char* input, mbr_config_t* config) {
//     if (input == NULL || config == NULL) {
//     	printf("Null Characters");
//     	return;
//     }

//     char *saveptr;
//     // We MUST copy the string because strtok_r inserts '\0' characters into it
//     char *temp_str = strdup(input);
//     if (temp_str == NULL) return;

//     // Initialize strings to empty to ensure no garbage is printed if parsing stops early
//     memset_j(config->Future_use,sizeof(config->Future_use), 0, sizeof(config->Future_use));
//     memset_j(config->Model_no, sizeof(config->Model_no), 0, sizeof(config->Model_no));

//     // 1. Slave ID
//     char *token = strtok_r(temp_str, ":", &saveptr);
//     if (token) config->slave_id = atoi(token);

//     // 2. Poll Frequency
//     token = strtok_r(NULL, ":", &saveptr);
//     if (token) config->poll_frq = atoi(token);

//     // 3. Push Frequency M
//     token = strtok_r(NULL, ":", &saveptr);
//     if (token) config->push_frq_m = atoi(token);

//     // 4. Push Frequency B
//     token = strtok_r(NULL, ":", &saveptr);
//     if (token) config->push_frq_b = atoi(token);

//     // 5. Future Use / Device Tag
//     token = strtok_r(NULL, ":", &saveptr);
//     if (token) {
//         strcpy_j(config->Future_use, MODEL_NO_MAX_LEN, token, strlen(token));
//     }

//     // 6. Model No
//     token = strtok_r(NULL, ":", &saveptr);
//     if (token) {
//         strcpy_j(config->Model_no, MODEL_NO_MAX_LEN, token, strlen(token));
//     }

//     free(temp_str); // Important: Free the memory allocated by strdup
// }

void parse_mbr_settings(const char* json_buffer) {

    cJSON *root = cJSON_Parse(json_buffer);
    if (!root) {
        ESP_LOGE(TAG, "Critical: Could not parse JSON buffer");
        return;
    }

    cJSON *mbr_obj = cJSON_GetObjectItem(root, "mbr");
    /* BUG-FIX #11: leaked the cJSON tree on every malformed/no-mbr payload
       (≈190B/leak). Repeated bad payloads from cloud → eventual heap OOM. */
    if (!cJSON_IsObject(mbr_obj)) { cJSON_Delete(root); return; }

    // 1. First, count TOTAL dmbr entries across all devices to allocate memory
    int total_dmbr_count = 0;
    cJSON *device_node = NULL;
    cJSON_ArrayForEach(device_node, mbr_obj) {
        total_dmbr_count += cJSON_GetArraySize(device_node);
    }
    /* BUG-FIX #2: cap at MAX_DEVICES so an attacker-controlled JSON cannot
       drive total_dmbr_count to 2^28+ where (count*sizeof) integer-overflows
       a small malloc and the subsequent fill loop trashes the heap. */
    #define MAX_DMBR_DEVICES 64
    if (total_dmbr_count < 0 || total_dmbr_count > MAX_DMBR_DEVICES) {
        ESP_LOGE(TAG, "Rejecting mbr payload: count=%d exceeds cap=%d",
                 total_dmbr_count, MAX_DMBR_DEVICES);
        cJSON_Delete(root);
        return;
    }

    if (system_configs) {
        free(system_configs);
        system_configs = NULL; // Prevent "Dangling Pointer"
    }

    // 2. Only allocate if there's actually something to store
    if (total_dmbr_count > 0) {
        system_configs = malloc(total_dmbr_count * sizeof(mbr_config_t));
        
        if (system_configs == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for %d configs", total_dmbr_count);
            cJSON_Delete(root); /* BUG-FIX #11 */
            return;
        }
    } else {
        ESP_LOGW(TAG, "No configurations found to allocate.");
        system_configs = NULL;
    }
	
	if (last_poll_times){
        free(last_poll_times);
        last_poll_times = NULL;
    } 

    if(total_dmbr_count > 0){
        last_poll_times = calloc(total_dmbr_count, sizeof(uint32_t));
        if (last_poll_times == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for %d configs", total_dmbr_count);
            cJSON_Delete(root); /* BUG-FIX #11 */
            return;
        }
    }

    // 2. Second, fill the array (reset counter since system_configs was just reallocated)
    total_configs = 0;
    cJSON_ArrayForEach(device_node, mbr_obj) {
        const char *current_device_id = device_node->string; // e.g., "1"

        cJSON *dmbr_item = NULL;
        cJSON_ArrayForEach(dmbr_item, device_node) {
            // Store Device ID
            strcpy_j(system_configs[total_configs].device_id, DEVICE_ID_MAX_LEN, current_device_id, strlen(current_device_id));

            // Store the dmbr key (e.g., "dmbr4")
            strcpy_j(system_configs[total_configs].key_name, KEY_NAME_MX_LEN, dmbr_item->string, strlen(dmbr_item->string));

            // Parse values "1:20:300:300"
            if (cJSON_IsString(dmbr_item)) {

            	// 2. Call the new robust parser
            	parse_mbr_string(dmbr_item->valuestring, &system_configs[total_configs]);

                total_configs++;
            }
        }
    }
    ESP_LOGI(TAG, "Parsed %d total configuration jobs", total_configs);
    // --- Dynamic Memory Initialization ---
    if (last_push_min) free(last_push_min);
    last_push_min = (uint32_t *)malloc(total_configs * sizeof(uint32_t));
    for(int i = 0; i < total_configs; i++) {
        last_push_min[i] = (uint32_t)-1;
    }

    if (mbr_due_flags) free(mbr_due_flags);
    mbr_due_flags = (bool *)malloc(total_configs * sizeof(bool));
    for(int i = 0; i < total_configs; i++) {
        mbr_due_flags[i] = false;
    }
}


void init_system_settings(const char* json_buffer) {
    cJSON *root = cJSON_Parse(json_buffer);
    if (!root) {
        ESP_LOGE(TAG, "Critical: Could not parse JSON buffer");
        return;
    }

    // --- Part B: Handle URT (Nested Objects) ---
    cJSON *urt_obj = cJSON_GetObjectItem(root, "urt");
    if (cJSON_IsObject(urt_obj)) {
        int count = cJSON_GetArraySize(urt_obj);
        if (system_urts) free(system_urts);
        system_urts = (urt_config_t *)malloc(count * sizeof(urt_config_t));

        int idx = 0;
        cJSON *id_node = NULL;
        cJSON_ArrayForEach(id_node, urt_obj) {
            cJSON *curt_val = cJSON_GetObjectItem(id_node, "curt0");
            if (cJSON_IsString(curt_val)) {
                strcpy_j(system_urts[idx].key_name, KEY_NAME_MX_LEN, id_node->string, strlen(id_node->string));
                sscanf_j(curt_val->valuestring, "%d:%ld:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d",
                       &system_urts[idx].urt_id, &system_urts[idx].baud,
                       &system_urts[idx].parity, &system_urts[idx].data_bits,
                       &system_urts[idx].stop_bits, &system_urts[idx].retry_cnt,
                       &system_urts[idx].retry_period, &system_urts[idx].DisconnectonNoA,
                       &system_urts[idx].DefConnStatus, &system_urts[idx].InterwriteDelay,
                       &system_urts[idx].InterReadDelay, &system_urts[idx].AutoConnect,
                       &system_urts[idx].swapcrc);
                idx++;
            }
        }
        total_urts = idx;
    }

    // --- Part C: IP CONFIG (Nested Objects) ---

    cJSON *ip_obj = cJSON_GetObjectItem(root, "ipc");
    if (cJSON_IsObject(ip_obj)) {
        int count = cJSON_GetArraySize(ip_obj);
        if (ip_config) free(ip_config);
        /* BUG-FIX #17: previously malloc()'d (uninitialized); a malformed
           "ipc0" string missing trailing fields (see below) left
           St_ip_En_dis holding whatever garbage was already on the heap
           instead of a defined 0/1, so whether the static IP applied was
           effectively random. calloc() gives a safe all-zero (disabled)
           fallback. */
        ip_config = (ip_config_t *)calloc(count, sizeof(ip_config_t));

        int idx = 0;
        cJSON *id_node = NULL;
        cJSON_ArrayForEach(id_node, ip_obj) {
            if (!cJSON_IsObject(id_node)) {
                continue;
            }
            strcpy_j(ip_config[idx].key_name, KEY_NAME_MX_LEN, id_node->string, strlen(id_node->string));

            /* BUG-FIX #18: replaced the old positional "ipc0" colon string
               ("ip:gw:mask:mb_port:pt_port:http_port:timeout:retry:enable")
               with named fields (same names as the HTTP /api/config JSON
               endpoint). The old format silently mis-parsed or left fields
               uninitialized whenever a field was missing/reordered — e.g. a
               command missing the trailing "enable" field left St_ip_En_dis
               undefined, so the device fell back to DHCP with no clear
               error. Named fields let each one be validated individually
               with a specific error message instead. */
            const char *ip_str = NULL, *gw_str = NULL, *nm_str = NULL;
            bool ok = true;
            cJSON *j;

#define REQUIRE_STRING(key, out) \
            j = cJSON_GetObjectItemCaseSensitive(id_node, key); \
            if (!cJSON_IsString(j) || !j->valuestring) { \
                ESP_LOGE("IP_PARSER", "ipc entry '%s' missing/invalid string field '%s'", id_node->string, key); \
                ok = false; \
            } else { out = j->valuestring; }
#define REQUIRE_INT(key, out) \
            j = cJSON_GetObjectItemCaseSensitive(id_node, key); \
            if (!cJSON_IsNumber(j)) { \
                ESP_LOGE("IP_PARSER", "ipc entry '%s' missing/invalid int field '%s'", id_node->string, key); \
                ok = false; \
            } else { out = j->valueint; }

            REQUIRE_STRING("static_ip", ip_str);
            REQUIRE_STRING("gw",        gw_str);
            REQUIRE_STRING("nm",        nm_str);
            REQUIRE_INT("mb_port",   ip_config[idx].port);
            REQUIRE_INT("pt_port",   ip_config[idx].pass_through_port);
            REQUIRE_INT("http_port", ip_config[idx].http_port);
            REQUIRE_INT("timeout",   ip_config[idx].server_timeout);
            REQUIRE_INT("pt_retry",  ip_config[idx].Passthr_retry_cnt);
            REQUIRE_INT("static_en", ip_config[idx].St_ip_En_dis);
#undef REQUIRE_STRING
#undef REQUIRE_INT

            if (!ok) {
                ESP_LOGE("IP_PARSER", "ipc entry '%s' incomplete — disabling static IP for this entry", id_node->string);
                ip_config[idx].St_ip_En_dis = 0;
                idx++;
                continue;
            }

            char ip_address[16];
            strcpy_j(ip_address, sizeof(ip_address), ip_str, strlen(ip_str));

            printf("Parsed IP Config: %s, %s, %s, Port: %d,Pass through port:%d,http_port:%d,server_timeout:%d,Passthrough_Retry:%d,Enabled: %d \n",
                ip_address, nm_str, gw_str,
                ip_config[idx].port,
                ip_config[idx].pass_through_port,
                ip_config[idx].http_port,
                ip_config[idx].server_timeout,
                ip_config[idx].Passthr_retry_cnt,
                ip_config[idx].St_ip_En_dis);

            /* If the last octet is 0, substitute it with the last byte of
               the STA MAC (e.g. 192.168.1.0 + MAC 7C -> 192.168.1.124).
               Avoid producing 0 or 255 which would be unrouteable/broadcast. */
            {
                unsigned int a, b, c, d;
                if (sscanf(ip_address, "%u.%u.%u.%u", &a, &b, &c, &d) == 4 && d == 0) {
                    unsigned int new_d = mac_Wifi[5];
                    if (new_d == 0)   new_d = 1;
                    if (new_d == 255) new_d = 254;
                    snprintf(ip_address, 16, "%u.%u.%u.%u", a, b, c, new_d);
                    ESP_LOGI("IP_PARSER",
                             "Last octet was 0, substituted with STA MAC byte 0x%02X -> %s",
                             mac_Wifi[5], ip_address);
                }
            }

            if (inet_pton(AF_INET, ip_address, &ip_config[idx].st_ip_info.ip.addr) != 1) {
                ESP_LOGE("IP_PARSER", "Invalid IP address format: %s", ip_address);
                // Fallback to a safe default or handle error
                ip_config[idx].St_ip_En_dis=0;
            }
             if (inet_pton(AF_INET, gw_str, &ip_config[idx].st_ip_info.gw.addr) != 1) {
                ESP_LOGE("IP_PARSER", "Invalid IP address format: %s", gw_str);
                // Fallback to a safe default or handle error
                ip_config[idx].St_ip_En_dis=0;
            }
             if (inet_pton(AF_INET, nm_str, &ip_config[idx].st_ip_info.netmask.addr) != 1) {
                ESP_LOGE("IP_PARSER", "Invalid IP address format: %s", nm_str);
                // Fallback to a safe default or handle error
                ip_config[idx].St_ip_En_dis=0;
            }
            /* BUG-FIX #4: previously inet_pton accepted "0.0.0.0" as a
               syntactically valid IP and the static-IP path stayed
               enabled — device would boot with an unroutable address
               and become unreachable forever (truck-roll recovery).
               Reject zero or broadcast IP/gw/netmask and fall back to
               DHCP for safety. */
            if (ip_config[idx].St_ip_En_dis == 1) {
                uint32_t ip_n   = ip_config[idx].st_ip_info.ip.addr;
                uint32_t gw_n   = ip_config[idx].st_ip_info.gw.addr;
                uint32_t mask_n = ip_config[idx].st_ip_info.netmask.addr;
                if (ip_n == 0 || gw_n == 0 || mask_n == 0 ||
                    ip_n == 0xFFFFFFFF || gw_n == 0xFFFFFFFF) {
                    ESP_LOGE("IP_PARSER",
                             "Refusing static IP %s/%s gw=%s — disabling, falling back to DHCP",
                             ip_address, nm_str, gw_str);
                    ip_config[idx].St_ip_En_dis = 0;
                }
            }
            if(ip_config[idx].server_timeout<=2 || ip_config[idx].server_timeout>65535 ){
                ip_config[idx].server_timeout = 10; // Set a default timeout of 10 sec
            }
            if(ip_config[idx].port==0 || ip_config[idx].port>65535 ){
                ip_config[idx].port = 502; // Set a default timeout of 10 sec
            }

            if(ip_config[idx].pass_through_port==0 || ip_config[idx].pass_through_port>65535 ){
                ip_config[idx].pass_through_port = 503; // Set a default timeout of 10 sec
            }
            if(ip_config[idx].http_port<=0 || ip_config[idx].http_port>65535){
                ip_config[idx].http_port = 80;
            }
            if(ip_config[idx].Passthr_retry_cnt<=0 || ip_config[idx].Passthr_retry_cnt>65535){
                ip_config[idx].Passthr_retry_cnt = 0;
            }
            idx++;
        }
        total_ip_configs = idx;
    }
    ESP_LOGI("CONFIG", "Applied to Struct -> IP: " IPSTR ", Mask: " IPSTR ", GW: " IPSTR,
            IP2STR(&ip_config[0].st_ip_info.ip),
            IP2STR(&ip_config[0].st_ip_info.netmask),
            IP2STR(&ip_config[0].st_ip_info.gw));

    // --- Part D: Handle HLT (Nested Objects) ---
    cJSON *hlt_obj = cJSON_GetObjectItem(root, "hlt");
    if (cJSON_IsObject(hlt_obj)) {
        int count = cJSON_GetArraySize(hlt_obj);
        if (hlt_configs) free(hlt_configs);
        hlt_configs = (hlt_config_t *)malloc(count * sizeof(hlt_config_t));

        int idx = 0;
        cJSON *id_node = NULL;
        cJSON_ArrayForEach(id_node, hlt_obj) {
            cJSON *dhlt_val = cJSON_GetObjectItem(id_node, "dhlt1");
            if (cJSON_IsString(dhlt_val)) {
                strcpy_j(hlt_configs[idx].key_name, KEY_NAME_MX_LEN, id_node->string, strlen(id_node->string));
                sscanf_j(dhlt_val->valuestring, "%d:%d",
                       &hlt_configs[idx].poll_frq, &hlt_configs[idx].push_frq);
                idx++;
            }
        }
        total_hlt_configs = idx;
    }
    //---------Checking for ota update-----------------------------------------------------
    cJSON *ota = cJSON_GetObjectItemCaseSensitive(root, "ota");
    if (cJSON_IsObject(ota)) {
        cJSON *zero_obj = cJSON_GetObjectItemCaseSensitive(ota, "0");
        if (cJSON_IsObject(zero_obj)) {
            cJSON *ota0 = cJSON_GetObjectItemCaseSensitive(zero_obj, "ota0");
            
            if (cJSON_IsString(ota0) && (ota0->valuestring != NULL)) {
                ESP_LOGI(TAG, "Retrieved OTA String: %s", ota0->valuestring);
                /* Field widths bound writes to url[128] and filepath[64]. */
                sscanf(ota0->valuestring, "%127[^:]:%hu:%63[^:]:%hu", httpserver_params.url, &httpserver_params.port, httpserver_params.filepath, &httpserver_params.is_ota);
                printf("url:%s:%d,filepath:%s, Enabled: %d\n", httpserver_params.url, httpserver_params.port, httpserver_params.filepath, httpserver_params.is_ota);
            }
        }
    }

    cJSON_Delete(root);
}


void parse_profile_settings(const char* json_buffer) {
    ESP_LOGI(TAG, "Parsing profile settings for %d configurations", total_configs);
    cJSON *root = cJSON_Parse(json_buffer);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON root for profiles");
        return;
    }

    // 1. Reset Global Profile Memory
    if (profile_parms) {
        free(profile_parms);
        profile_parms = NULL;
    }
    total_profiles = 0;

    // 2. Iterate through our FLATTENED system_configs array
    // This handles Device 1:dmbr1, Device 1:dmbr4, Device 2:dmbr1, etc.
    for (int i = 0; i < total_configs; i++) {

        // Use the key_name (e.g., "dmbr1" or "dmbr4") to find the register map
        cJSON *p_obj = cJSON_GetObjectItem(root, system_configs[i].key_name);

        if (cJSON_IsObject(p_obj)) {
            int n = cJSON_GetArraySize(p_obj);

            // 3. Expand the profile array to accommodate 'n' new registers
            profile_config_t *new_ptr = realloc(profile_parms, (total_profiles + n) * sizeof(profile_config_t));
            if (new_ptr == NULL) {
                ESP_LOGE(TAG, "OOM: Could not allocate memory for profile %s", system_configs[i].key_name);
                continue;
            }
            profile_parms = new_ptr;

            // 4. Map the starting index and count for THIS specific job
            system_configs[i].profile_start_idx = total_profiles;
            system_configs[i].profile_count = n;

            // 5. Extract each register definition (e.g., "vrp", "vyp")
            cJSON *reg_item = NULL;
            cJSON_ArrayForEach(reg_item, p_obj) {
                if (cJSON_IsString(reg_item)) {
                    // Store register key (e.g., "vrp")
                    strcpy_j(profile_parms[total_profiles].key_name, KEY_NAME_MX_LEN, reg_item->string, strlen(reg_item->string));

                    // Parse the 11-parameter string
                    sscanf_j(reg_item->valuestring, "%d:%d:%d:%d:%d:%d:%d:%d:%f:%d:%d",
                           &profile_parms[total_profiles].reg_type,
                           &profile_parms[total_profiles].access_type,
                           &profile_parms[total_profiles].mod_add,
                           &profile_parms[total_profiles].group,
                           &profile_parms[total_profiles].bytes,
                           &profile_parms[total_profiles].dta_dwnlnk_type,
                           &profile_parms[total_profiles].dta_uplnk_type,
                           &profile_parms[total_profiles].Endian_type,
                           &profile_parms[total_profiles].multiplier,
                           &profile_parms[total_profiles].start_bit,
                           &profile_parms[total_profiles].bit_cnt
                    );
                    profile_parms[total_profiles].Isdata_available = 0;
                    total_profiles++;
                }
            }
            ESP_LOGI(TAG, "Linked %d registers for Job %d (%s:%s)",
                     n, i, system_configs[i].device_id, system_configs[i].key_name);
        } else {
            // If no profile found, this dmbr job has no registers to read
            system_configs[i].profile_count = 0;
            ESP_LOGW(TAG, "No profile found for key: %s", system_configs[i].key_name);
        }
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Successfully linked %d total registers across all devices", total_profiles);
}


void Check_config()
{
	/* Defensive: the diagnostic loop below dereferences system_configs[i].
	   If a caller invokes Check_config with system_configs==NULL but
	   total_configs>0 (invariant violation), guard against NULL deref —
	   the reset blocks below will rebuild the buffer anyway. */
	if (system_configs == NULL) total_configs = 0;
	for(int i=0;i<total_configs;i++)
	{
	  printf("Start Add:%d profile_count:%d",system_configs[i].profile_start_idx,system_configs[i].profile_count);
	}
	//---------------------Check UART Settting-----------------------------------------
	if (system_urts == NULL || total_urts !=1)
	{
		if (system_urts) free(system_urts);
		system_urts = (urt_config_t *)malloc(1 * sizeof(urt_config_t));
		system_urts[0].urt_id =0;
		system_urts[0].baud =9600;
		system_urts[0].parity = 0;
		system_urts[0].data_bits = 8;
		system_urts[0].stop_bits = 1;
		system_urts[0].retry_cnt = 1;
		system_urts[0].retry_period = 600;
        system_urts[0].DisconnectonNoA=0;
        system_urts[0].DefConnStatus=0;
        system_urts[0].InterwriteDelay=0;
        system_urts[0].InterReadDelay=0;
        system_urts[0].AutoConnect=0;
        system_urts[0].swapcrc = 0;
		total_urts =1;
		ESP_LOGI("CONFIG", "Default uart setting loaded");
	}
	if (system_urts[0].baud != 1200 && system_urts[0].baud != 2400 && system_urts[0].baud != 4800 && system_urts[0].baud != 9600 && system_urts[0].baud != 19200 && system_urts[0].baud != 38400 && system_urts[0].baud != 57600 && system_urts[0].baud != 115200)
			system_urts[0].baud =9600;

	if (system_urts[0].parity>2) 
			system_urts[0].parity =0;

	if (system_urts[0].data_bits!=7 && system_urts[0].data_bits!=8) 
			system_urts[0].data_bits =8;

	if (system_urts[0].stop_bits!=1 && system_urts[0].stop_bits!=2) 
			system_urts[0].stop_bits =1;

	if (system_urts[0].retry_cnt<1 || system_urts[0].retry_cnt>255) 
			system_urts[0].retry_cnt=1;

    if (system_urts[0].retry_period<0||system_urts[0].retry_period>65535) 
    		system_urts[0].retry_period = 600;
    
    if(system_urts[0].swapcrc<0 || system_urts[0].swapcrc>1)
        system_urts[0].swapcrc=0;

    system_urts[0].DisconnectonNoA=0;
    system_urts[0].DefConnStatus=0;
    system_urts[0].InterwriteDelay=0;
    system_urts[0].InterReadDelay=0;
    system_urts[0].AutoConnect=0;

    //-------------------------------hlt config--------------------------

    if (hlt_configs == NULL ||total_hlt_configs!=1)
    {
    	if (hlt_configs) free(hlt_configs);
    	hlt_configs = (hlt_config_t *)malloc(1 * sizeof(hlt_config_t));
    	strcpy_j((char *)&hlt_configs[0].key_name, KEY_NAME_MX_LEN, "dhlt1", 6);
    	hlt_configs[0].poll_frq = 600;
    	hlt_configs[0].push_frq = 600;
    	total_hlt_configs = 1;
    	ESP_LOGI("CONFIG", "Default health setting loaded");
    }
    if (hlt_configs[0].poll_frq<60||hlt_configs[0].poll_frq>65535) 
    		hlt_configs[0].poll_frq = 600;

    if (hlt_configs[0].push_frq<60||hlt_configs[0].push_frq>65535) 
    		hlt_configs[0].push_frq = 600;

    //---------------------------------System Config--------------------------------------------------
    
    if (system_configs == NULL ||total_configs>50)
    {
    	if (system_configs) free(system_configs);
    	system_configs = (mbr_config_t *)malloc(1 * sizeof(mbr_config_t));
    	strcpy_j((char *)&system_configs[0].key_name, KEY_NAME_MX_LEN, "dmbr1", 6);
    	system_configs[0].slave_id =1;
    	system_configs[0].poll_frq = 600;
    	system_configs[0].push_frq_m = 600;
    	total_configs = 1; /* BUG-FIX #3: was `total_hlt_configs = 1` typo; left
    	                     total_configs at the >50 input value, causing the
    	                     validation for-loop below to walk OOB on the
    	                     freshly malloc'd 1-entry buffer and corrupt heap. */
        system_configs[0].profile_start_idx = 0;
        system_configs[0].profile_count = 0;
    	ESP_LOGI("CONFIG", "Default System setting loaded");
    }
    for(int i=0;i<total_configs;i++){
        if (system_configs[i].slave_id < 1 || system_configs[i].slave_id >255)
            system_configs[i].slave_id =1;
            
        if (system_configs[i].poll_frq<2||system_configs[i].poll_frq>65535)
                system_configs[i].poll_frq = 30;

        if (system_configs[i].push_frq_m<60||system_configs[i].push_frq_m>65535) 
                system_configs[i].push_frq_m = 600;
        
        if (system_configs[i].push_frq_b<60||system_configs[i].push_frq_b>65535) 
                system_configs[i].push_frq_b = 600;
    }
    //---------------------------------profile parameters---------------------------------------------

    if(profile_parms == NULL || total_profiles==0)
    {
    	if (profile_parms) free(profile_parms);
    	profile_parms = (profile_config_t *)malloc(1 * sizeof(profile_config_t));

    	if (profile_parms != NULL) {
    		strcpy_j(profile_parms[0].key_name, KEY_NAME_MX_LEN, "vry", 4);
	        profile_parms[0].reg_type = 3;
	        profile_parms[0].access_type = 0;
	        profile_parms[0].mod_add = 14;
	        profile_parms[0].group = 0;
	        profile_parms[0].bytes = 4;
	        profile_parms[0].dta_dwnlnk_type = 0;
	        profile_parms[0].dta_uplnk_type = 0;
	        profile_parms[0].Endian_type= 0;
			profile_parms[0].multiplier =1.0;
			profile_parms[0].start_bit = 0;
			profile_parms[0].bit_cnt=0;
			total_profiles =1;
			profile_config_t *temp = realloc(profile_parms, sizeof(profile_config_t));
			if (temp != NULL) {
				profile_parms = temp;
			} else {
				free(profile_parms);
				profile_parms = NULL;
			}
	        system_configs[0].profile_count = 1;
			ESP_LOGI("CONFIG", "Default profile setting loaded");
    	} else {
    		total_profiles = 0;
    		system_configs[0].profile_count = 0;
    		ESP_LOGI("CONFIG", "Failed to allocate memory for profile settings");
    	}
    }

    for(int i=0;i<total_profiles;i++)
    {
    	if(strlen(profile_parms[i].key_name)==0)
    	   sprintf_j((char *)&profile_parms[i].key_name,KEY_NAME_MX_LEN, "par%d",i);
    	if(profile_parms[i].reg_type<1 || profile_parms[i].reg_type>5)  // Intentionally allowed five in order to diable read
    		profile_parms[i].reg_type =3;
    	if(profile_parms[i].access_type<0 || profile_parms[i].reg_type>2)
    		profile_parms[i].access_type =0;
    	if(profile_parms[i].mod_add<1 || profile_parms[i].mod_add>65535)
    		profile_parms[i].mod_add =10;
    	if(profile_parms[i].bytes<1 || profile_parms[i].bytes>255)
    		profile_parms[i].bytes =4;
    	if(profile_parms[i].dta_dwnlnk_type>17)
    		profile_parms[i].dta_dwnlnk_type =0;
    	if(profile_parms[i].dta_uplnk_type>17)
    		profile_parms[i].dta_uplnk_type =0;
    	if(profile_parms[i].Endian_type>4)
    		profile_parms[i].Endian_type =0;
    	/* BUG-FIX #20: !isfinite() catches NaN/Inf which compare false with both
    	   < and > and previously slipped through, propagating NaN through Modbus
    	   register multiplications and silently corrupting all readings. */
    	if(!isfinite(profile_parms[i].multiplier) ||
    	   profile_parms[i].multiplier<-10000 || profile_parms[i].multiplier>10000)
    		profile_parms[i].multiplier =1.0;
    	if(profile_parms[i].start_bit>32)
    	    profile_parms[i].start_bit =0;
    	if(profile_parms[i].bit_cnt>32)
    	    profile_parms[i].bit_cnt =0;
    }
    if(ip_config==NULL || total_ip_configs==0)
    {
    	if (ip_config) free(ip_config);
    	ip_config = (ip_config_t *)malloc(1 * sizeof(ip_config_t));
    	strcpy_j((char *)&ip_config[0].key_name, KEY_NAME_MX_LEN, "ipc0", 6);
    	ip_config[0].st_ip_info.ip.addr = 0;
    	ip_config[0].st_ip_info.netmask.addr = 0;
    	ip_config[0].st_ip_info.gw.addr = 0;
    	ip_config[0].port = 502;
    	ip_config[0].St_ip_En_dis = 0;
    	ip_config[0].server_timeout = 60;
    	ip_config[0].pass_through_port = 503;
    	ip_config[0].http_port = 80;
    	ip_config[0].Passthr_retry_cnt = 1;
    	total_ip_configs = 1;
    	ESP_LOGI("CONFIG", "Default IP config setting loaded");
    }
   for(int i=0;i<total_configs;i++)
	{
	  printf("Start Add:%d profile_count:%d\n",system_configs[i].profile_start_idx,system_configs[i].profile_count);
	}
}


void debug_print_configs() {
    printf("\n--- SYSTEM CONFIGURATIONS REPORT ---\n");

    // 1. Print URT Settings
    printf("[URT SETTINGS] Total: %d\n", total_urts);
    if (system_urts != NULL) {
        printf("ID   | Name       | Baud    | P | D | S | RETRY_CNT | RTY_PRD | SWAPCRC \n");
        printf("--------------------------------------------\n");
        for (int i = 0; i < total_urts; i++) {
            printf("%-4d | %-10s | %-7ld | %d | %d | %d | %-9d | %-7d|%-9d\n",
                   system_urts[i].urt_id,
                   system_urts[i].key_name,
                   system_urts[i].baud,
                   system_urts[i].parity,
                   system_urts[i].data_bits,
                   system_urts[i].stop_bits,
                   system_urts[i].retry_cnt,
                   system_urts[i].retry_period,
                   system_urts[i].swapcrc);
        }
    } else {
        printf("No URT configurations loaded.\n");
    }

    printf("\n");
    // 1. Print HLT Settings
    printf("[HLT SETTINGS] Total: %d\n", total_hlt_configs);
    if (hlt_configs != NULL) {
        printf("Name       | Pull  | PUSH \n");
        printf("--------------------------------------------\n");
        for (int i = 0; i < total_hlt_configs; i++) {
            printf("%-10s | %-4d | %-4d \n",
                   hlt_configs[i].key_name,
                   hlt_configs[i].poll_frq,
                   hlt_configs[i].push_frq);
        }
    } else {
        printf("No HLT configurations loaded.\n");
    }

    printf("\n");


        // 1. Print IP CONFIG Settings
    printf("[IP CONFIG SETTINGS] Total: %d\n", total_ip_configs);
    if (ip_config != NULL) {
        printf("Name       | IP Address   | Netmask   | Gateway   | Port | PassThrough_Port| HTTP_Port |Server_Timeout | Passthrough_Retry |  Enable_Disable | \n");
        printf("--------------------------------------------------\n");
        for (int i = 0; i < total_ip_configs; i++) {
            printf("%-10s |" IPSTR "|" IPSTR "|" IPSTR "| %-4d |%-4d | %-4d| %-4d| %-4d| %-4d\n",
                   ip_config[i].key_name,
                   IP2STR(&ip_config[i].st_ip_info.ip),
                   IP2STR(&ip_config[i].st_ip_info.netmask),
                   IP2STR(&ip_config[i].st_ip_info.gw),
                   ip_config[i].port,
                   ip_config[i].pass_through_port,
                   ip_config[i].http_port,
                   ip_config[i].server_timeout,
                   ip_config[i].Passthr_retry_cnt,
                   ip_config[i].St_ip_En_dis);
        }
    } else {
        printf("No IP configurations loaded.\n");
    }

    printf("\n");

    // 2. Print MBR Settings
    printf("[MBR SETTINGS] Total: %d\n", total_configs);
    if (system_configs != NULL) {
        printf("DEVICE ID |SLAV_ID | Key Name   | poll_frq | push_frq_m | push_frq_b |Model No | Poll IP Address |   port   | System Config Start| Count\n");
        printf("--------------------------------------------------\n");
        for (int i = 0; i < total_configs; i++) {
            printf("%-10s|%-7d | %-10s | %-8d | %-10d | %-10d |%-15s | %-12s | %-10d |%-10d | %-5d\n",
            	   system_configs[i].device_id,
            	   system_configs[i].slave_id,
                   system_configs[i].key_name,
                   system_configs[i].poll_frq,
                   system_configs[i].push_frq_m,
                   system_configs[i].push_frq_b,
				   system_configs[i].Model_no,
                   inet_ntoa(system_configs[i].poll_ip.addr),
                   system_configs[i].modbus_port,
                   system_configs[i].profile_start_idx,
                   system_configs[i].profile_count
                );
        }
    } else {
        printf("No MBR configurations loaded.\n");
    }
    printf("\n[PARAMS SETTINGS] Total: %d\n", total_profiles);
        if (profile_parms != NULL) {
            printf("|Key        | reg_type | access | mod_add | Raw group | bytes | dwnlnk | uplnk | Endian | multi | start_bit | bit_cnt \n");
            printf("---------------------------------------------------\n");
            for (int idx = 0; idx < total_profiles; idx++) {
                printf("%-10s | %-8d | %-6d | %-7d |%-9d |%-5d | %-6d | %-5d | %-5d | %-4.1f | %-9d | %-7d\n",
                        profile_parms[idx].key_name,
						profile_parms[idx].reg_type,
                        profile_parms[idx].access_type,
                        profile_parms[idx].mod_add,
                        profile_parms[idx].group,
		                profile_parms[idx].bytes,
		                profile_parms[idx].dta_dwnlnk_type,
		                profile_parms[idx].dta_uplnk_type,
		                profile_parms[idx].Endian_type,
						profile_parms[idx].multiplier,
						profile_parms[idx].start_bit,
						profile_parms[idx].bit_cnt);
            }
        }
    // Print HTTP Settings
    printf("\n[HTTP/OTA SETTINGS]\n");
        printf("        url      | port | filepath    | is_ota \n");
        printf("--------------------------------------------\n");
        for (int i = 0; i < total_urts; i++) {
            printf("%-17s | %-10d | %-13s | %-4d\n",
                   httpserver_params.url,
                   httpserver_params.port,
                   httpserver_params.filepath,
                   httpserver_params.is_ota);
        }
    printf("--- END OF REPORT ---\n");
}

void update_config_tag(const char *filename, const char *tag_name, const char *new_value) {
    // 1. Open and Read the file
    FILE *f = fopen(filename, "r");
    char *buffer = NULL;
    if (f) {
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        fseek(f, 0, SEEK_SET);

        // Fix: Only proceed if the file has content
        if (fsize > 0) {
            buffer = malloc(fsize + 1);
            if (buffer) {
                size_t read_size = fread(buffer, 1, fsize, f);
                if (read_size == (size_t)fsize) {
                    buffer[fsize] = '\0'; // Properly null-terminate
                } else {
                    free(buffer);
                    buffer = NULL;
                    ESP_LOGE(TAG, "Failed to read full file");
                }
            }
        } else {
            ESP_LOGW(TAG, "File is empty: %s", filename);
        }
        fclose(f);
    }

    cJSON *root = NULL;
    if (buffer) {
        root = cJSON_Parse(buffer);
        free(buffer);
    }

    if (!root) {
        root = cJSON_CreateObject();
    }

    // 2. Prepare the new item (Determine if it's an Object or a String)
    cJSON *new_item = cJSON_Parse(new_value);
    if (new_item == NULL) {
        // If it's not valid JSON, treat it as a standard string
        new_item = cJSON_CreateString(new_value);
    }

    // 3. Search and Replace or Insert
    if (cJSON_HasObjectItem(root, tag_name)) {
        // Replacing with the new object/string
        cJSON_ReplaceItemInObject(root, tag_name, new_item);
        ESP_LOGI(TAG, "Tag '%s' found and replaced.", tag_name);
    } else {
        // Adding new entry
        cJSON_AddItemToObject(root, tag_name, new_item);
        ESP_LOGI(TAG, "Tag '%s' not found, inserted new tag.", tag_name);
    }

    // 4. Write back to file
    char *rendered = cJSON_PrintUnformatted(root);
	f = fopen(filename, "w");
    if (f) { // If the file opened, we MUST eventually close it
        if (rendered) {
            /* BUG-FIX #15: previously logged "File updated successfully" even
               when fputs failed (SPIFFS full / partition corrupt) — cloud saw
               the success and the device ran with stale config after reboot. */
            int fputs_rc = fputs(rendered, f);
            int fclose_rc = fclose(f);
            f = NULL; /* prevent double-close in the trailing fclose path */
            if (fputs_rc < 0 || fclose_rc != 0) {
                ESP_LOGE(TAG, "File write FAILED (fputs=%d fclose=%d)",
                         fputs_rc, fclose_rc);
            } else {
                ESP_LOGI(TAG, "File updated successfully.");
            }
        } else {
            ESP_LOGE(TAG, "Failed to render JSON string.");
        }
        if (f) fclose(f); // Always close if f is still open
    } else {
        ESP_LOGE(TAG, "Failed to open file for writing.");
    }

    // Always clean up heap memory
    if (rendered) {
        free(rendered);
    }
    cJSON_Delete(root);
}
