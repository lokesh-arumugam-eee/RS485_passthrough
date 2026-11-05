#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#define KEY_NAME_MX_LEN 16
#define MODEL_NO_MAX_LEN 25
#define DEVICE_ID_MAX_LEN 25

#include "esp_netif.h"

#define MAX_DATA_BYTES 200

typedef struct {
    char key_name[KEY_NAME_MX_LEN]; // Stores
    int poll_frq;       //0-65535 sec
    int push_frq;     //0-65535 sec
} hlt_config_t;

typedef struct {
	char device_id[DEVICE_ID_MAX_LEN];  // To store "1", "2", etc.
    char key_name[KEY_NAME_MX_LEN]; // Stores "dmbr1", "dmbr2", etc.
    int slave_id;       //0-255
    int poll_frq;       //0-65535 sec
    int push_frq_m;     //0-65535 sec
    int push_frq_b;     //0-65535 sec
    char Future_use[MODEL_NO_MAX_LEN];
    char Model_no[MODEL_NO_MAX_LEN];
    esp_ip4_addr_t poll_ip;
    int modbus_port;
    // --- Linking Attributes ---
    int profile_start_idx; // The index in profile_parms where this device starts
    int profile_count;     // How many parameters belong to this device
} mbr_config_t;

typedef struct {
	char key_name[KEY_NAME_MX_LEN];
    int urt_id;        //not used
    uint32_t baud;
    int parity;
    int data_bits;
    int stop_bits;
    int retry_cnt;         //from here not used
    int retry_period;
    int DisconnectonNoA;
    int DefConnStatus;
    int InterwriteDelay;
    int InterReadDelay;
    int AutoConnect;
} urt_config_t;

typedef struct {
    char key_name[KEY_NAME_MX_LEN]; // "vlt1", "vlt2"
    int reg_type;
    int access_type;
    int mod_add;       //0 to 2592000  Need to check
    int group;         // 0 to 255
    int bytes;         // 0 to 255 
    int dta_dwnlnk_type;
    int dta_uplnk_type;
    int Endian_type;
    float multiplier;  // -10000.0 to 10000.0
    int start_bit;
    int bit_cnt;
    float data;
    char Isdata_available;
} profile_config_t;

typedef struct {
    char key_name[KEY_NAME_MX_LEN]; // "vlt1", "vlt2"
    int slave_id;
    int reg_type;
    int access_type;
    int mod_add;       //0 to 2592000  Need to check
    int bytes;         // 0 to 255
    char data[MAX_DATA_BYTES*2];
    char Isrdndata_available;
} profile_rdn_t;

typedef struct {
    char key_name[KEY_NAME_MX_LEN];
    esp_netif_ip_info_t st_ip_info;
    int port;                // Modbus TCP listen port (default 502)
    int pass_through_port;   // Raw TCP<->RTU bridge listen port (default 503)
    int http_port;           // HTTP config-server listen port (default 80)
    int server_timeout;
    int Passthr_retry_cnt;
    int St_ip_En_dis;
} ip_config_t;

extern ip_config_t *ip_config;
extern int total_ip_configs;

extern hlt_config_t *hlt_configs;
extern int total_hlt_configs;

extern mbr_config_t *system_configs;
extern int total_configs;

extern urt_config_t *system_urts;
extern int total_urts;

extern profile_config_t *profile_parms;
extern int total_profiles;

extern profile_rdn_t *profile_rdn_parms;
extern int total_rdn_profiles;
extern int rdn_data_ready;


extern uint32_t *last_poll_times;

extern uint32_t *last_push_min;

extern bool *mbr_due_flags;

extern char *config_path;

void save_config_json();
bool file_exists(const char *path);
char* read_config_to_buffer(const char* path);
void parse_mbr_settings(const char* json_buffer);
void init_system_settings(const char* json_buffer);
void parse_profile_settings(const char* json_buffer);
void debug_print_configs();
esp_err_t delete_config_file(const char* path);
void Check_config();
void update_config_tag(const char *filename,const char *tag_name, const char *new_value);

#endif
