#include "controller.h"
#include "TimeManagement.h"
#include "cJSON.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "ble_spp_server_demo.h"
#include "esp_rom_sys.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "SafeFunctions/Jio_EA_Safe_Functions.h"
#include "uart/uart_app.h"
#include "wifiConnect.h"
#include "ota.h"
#include "mqtt_manager.h"
#include "xmodem/xmodem.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "configuration/configuration.h"
#include "wifiConnect.h"
#include "controller.h"
#include "http.h"
extern MQTT_CONFIG_s Mqtt_Config_s;
extern esp_err_t ReadWriteNVS_MqttConfig(uint8_t Type,
                                         MQTT_CONFIG_s *Mqtt_Config_t);


uint8_t IR_Transmit_blink;
// STATUS_S *ptr_status_s=&status_s;
uint8_t g_u8_cmd_val;
uint8_t g_u8_asts_value;
uint8_t g_u8_slvid_value;
uint32_t g_u32_tid_value;
uint8_t g_u8_schd_enbl_dsbl;
uint8_t CMD_Recieved;

char g_i8_root_ver[10];
bool IR_TEST_Completed=0;
uint8_t Test_Mode=0;
uint8_t DIR_BUT=0;
uint8_t l_u8_ack_nak;
uint8_t log_endis=0;
uint8_t excutelog=0;

uint8_t weekday_enable_p[7] = {5, 3, 3, 3, 3, 3, 5};
//holiday_t holidays_p[10] = {{1, 26}, {8, 15}, {10, 2}, {12, 25}};
uint8_t IR_ON_seq=0;
bool reboot=0;
uint8_t rebootcnter=0;
/* BUG-FIX #14: widened from uint16_t to uint32_t. The 16-bit counter wrapped
   every 65535 s (~18 hr), silently breaking any monotonic-increment logic
   that compared against a stored timestamp. */
uint32_t SecCouter = 0, CompresserONTimer_sec = 0;
static const char *TAG = "EVENT_EXAMPLE";
ESP_EVENT_DEFINE_BASE(AC_CNTRL_EVENT_BASE);
// void SetAC_Mode(AC_MODES_e Mode, uint8_t Temparature);
// void SetAsset_Mode(AC_MODES_e Mode, uint8_t Temparature, uint8_t Assert_Mask);
/* BUG-FIX #7: these FSM flags are written from the BLE/MQTT event handler
   context and read from Controller_task / timer callbacks. Without `volatile`
   the compiler is free to cache reads in registers across the loop bodies,
   producing torn-update races that manifest as one-in-thousands stuck OTAs.
   `volatile` is a prerequisite; full correctness still requires
   taskENTER_CRITICAL or a queue around the multi-byte transitions. */
volatile uint8_t g_u8_acceptFileRx;
volatile uint8_t g_u8_addtlDisblFlgfrlg;
volatile uint8_t g_u8_acceptfwRx;
volatile uint8_t g_u8_SendFileXMDMTx;
void Controller_init(void);
//()static const char *TAG = "OneShotTimer";
esp_timer_handle_t one_shot_timer1;
esp_timer_handle_t cmtimeout_timer1;
 uint8_t g_u8_expr_cnt = 0;
 uint8_t g_u8_rx_data[150];
 uint8_t g_u8_commtimeout = 0;



// Timer callback function
void timer_callback_BLE_CMTOUT(void *arg)
{
	g_u8_commtimeout += 1;
	ESP_LOGI(TAG, "BLE Comm timout\n");
	if(g_u8_commtimeout < 3)
	{
		uint8_t l_u8_array[150];
		memset_j(&l_u8_array[0], sizeof(l_u8_array), 0x00, sizeof(l_u8_array));
		
		memcpy_j(&l_u8_array[0], sizeof(l_u8_array), &g_u8_rx_data[0],132);
		Jio_ESP_010104_LIRFile_Tx_FSM(l_u8_array);
	}
	else
	{
		g_u8_commtimeout = 0;
		ESP_LOGE(TAG, "Comm Fail BLE APP\n");
		fclose(g_file);
		g_file = NULL;
    if(g_u8_acceptfwRx==1)
    {
      g_u8_acceptfwRx=0;
      esp_ota_abort(ota_handle);  // Cancel the OTA write
      ota_handle = 0;              // Reset handle
      ota_write_offset = 0;
      ESP_LOGW(TAG, "OTA aborted due to error");
    }
		g_u8_Xmdm_Tx_FSM_STATE = 0;
		g_u16_nxt_blknm = 1;
		ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
	}
}
void timer_callback_BLE_XMDM(void *arg) {
	
	g_u8_expr_cnt += 1;
	
	if(g_u8_expr_cnt > 1)
	{	
		g_u8_expr_cnt = 0;
		ESP_LOGI(TAG, "Timer expired!.");
		
		//ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
		// ESP_ERROR_CHECK(esp_timer_start_once(one_shot_timer, 5000000)); // Timer period in microseconds
		g_u8_acceptFileRx = 0;
    if(g_u8_acceptfwRx==1)
    {
      g_u8_acceptfwRx = 0;
      esp_ota_abort(ota_handle);  // Cancel the OTA write
      ota_handle = 0;              // Reset handle
      ota_write_offset = 0;
      ESP_LOGW(TAG, "OTA aborted due to error");
    }
		g_u16_expctd_blknm = 1;
		g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
		
		g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
		g_u8_SendFileXMDMTx = 0;
		g_u16_nxt_blknm = 1;
	}
	else
	{
		//()90 seconds timeout
		ESP_ERROR_CHECK(esp_timer_start_once(one_shot_timer1, 1800*1000*1000)); 
	}
}

void parse_WifiConfigCmd(const char *json_string) {

  char Resp[150];
  memset_j(Resp, sizeof(Resp), 0, sizeof(Resp));
  cJSON *root = cJSON_Parse(json_string);
  if (root == NULL) {
    acm_log("json err\n");
    return;
  }
  cJSON *J_wifiConfig = cJSON_GetObjectItem(root, "wifiConfig");
  cJSON *mqttConfig = cJSON_GetObjectItem(root, "MqttConfig");
  cJSON *DLIR = cJSON_GetObjectItem(root, "DLIR");
  cJSON *ipconfig = cJSON_GetObjectItem(root, "ipc");
  cJSON *urtconfig = cJSON_GetObjectItem(root, "urt");
  cJSON *dmbrconfig = cJSON_GetObjectItem(root, "dmbr");
  cJSON *mbrconfig = cJSON_GetObjectItem(root, "mbr");
  cJSON *hltconfig = cJSON_GetObjectItem(root, "hlt");
  cJSON *otaconfig = cJSON_GetObjectItem(root, "ota");

  if (cJSON_IsObject(mqttConfig)) {
    MQTT_CONFIG_s Mqtt_Config_s = {0};
    //cJSON *url = cJSON_GetObjectItem(mqttConfig, "BROKER_URL");
    cJSON *psw = cJSON_GetObjectItem(mqttConfig, "Device_Psw");
    cJSON *typ = cJSON_GetObjectItem(mqttConfig, "Typ");
    //ACM_LOG(" Mqtt %s,%s", url->valuestring, psw->valuestring);
    int8_t return_value=0;
    esp_err_t ret = ESP_OK;
    if (cJSON_IsString(psw)){
      /* BUG-FIX #16: previously silently truncated long passwords, persisting
         a corrupt value to NVS that broke MQTT auth permanently. Reject
         oversized inputs so the cloud sees "{\"MqttConfig\":0}" and a human
         can fix the input. */
      size_t psw_len = strlen(psw->valuestring);
      if (psw_len >= sizeof(Mqtt_Config_s.Device_Psw)) {
        ESP_LOGE("ACM", "MQTT password too long (%zu >= %zu); rejecting",
                 psw_len, sizeof(Mqtt_Config_s.Device_Psw));
        Custom_printf("{\"MqttConfig\":0}");
        cJSON_Delete(root); /* BUG-FIX #11 */
        return;
      }
      return_value = strcpy_j((char *)Mqtt_Config_s.Device_Psw,
                               sizeof(Mqtt_Config_s.Device_Psw),
                               (const char *)psw->valuestring,
                               psw_len);
    (void)return_value;
    }
    uint8_t Typ = cJSON_IsNumber(typ) ? ((typ->valueint == 2) ? 2 : 1) : 1;
    ret = ReadWriteNVS_MqttConfig(Typ, &Mqtt_Config_s);
    acm_log("mqtt cfg ret=%s\n", esp_err_to_name(ret));
     memset_j(Resp, sizeof(Resp), 0, sizeof(Resp));
     if(ret==ESP_OK)
     {
		 Custom_printf("{\"MqttConfig\":1}");
	 }
	 else {
		Custom_printf("{\"MqttConfig\":0}");
	}
  } else if (cJSON_IsObject(J_wifiConfig)) { // Navigate to the "wifiConfig" array

    //WIFICONFIG_S wifiConfig_s = {0};
    cJSON *ssid = cJSON_GetObjectItem(J_wifiConfig, "SSID");
    cJSON *pass = cJSON_GetObjectItem(J_wifiConfig, "PASS");
    cJSON *retry = cJSON_GetObjectItem(J_wifiConfig, "RTRYC");
    cJSON *type = cJSON_GetObjectItem(J_wifiConfig, "TYPE");
    /* BUG-FIX #6: missing keys made cJSON_GetObjectItem return NULL; the
       subsequent ssid->valuestring / pass->valuestring / retry->valueint
       derefs would SIGSEGV the entire device on a single malformed cloud
       payload. Validate every required field before use. */
    /* cJSON_IsString implies valuestring != NULL — keep the guard tight. */
    if (!cJSON_IsString(ssid) || !cJSON_IsString(pass) ||
        !cJSON_IsNumber(retry)) {
      ESP_LOGE("ACM", "wifiConfig payload missing/invalid SSID, PASS, or RTRYC");
      Custom_printf("{\"wifiConfig\":0}");
      cJSON_Delete(root); /* BUG-FIX #11 */
      return;
    }
    int8_t return_value = strcpy_j((char *)wifiConfig.ssid,
                               sizeof(wifiConfig.ssid),
                               (const char *)ssid->valuestring,
                               strlen(ssid->valuestring));
    (void)return_value;

    return_value = strcpy_j((char *)wifiConfig.password,
                        sizeof(wifiConfig.password),
                        (const char *)pass->valuestring,
                        strlen(pass->valuestring));
    (void)return_value;

    // Navigate to "TYPE"
    uint8_t Type = cJSON_IsNumber(type) ? ((type->valueint == 2) ? 2 : 1) : 1;
    // Navigate to "RTRYC"
    if (retry->valueint >= 0 && retry->valueint <= 15) { // asset ON/OFF
        wifiConfig.RtryC = retry->valueint;
        esp_err_t ret = ReadWriteNVS_wifiConfig(Type, &wifiConfig);
        ESP_LOGE("wifi cfg", "retry=%d type=%d ret=%s\n", wifiConfig.RtryC, Type, esp_err_to_name(ret));

        if(ret==ESP_OK){
            Custom_printf("{\"wifiConfig\":1}");
            change_wifi_credentials((const char *)wifiConfig.ssid,(const char *)wifiConfig.password);
          }
        else {
            Custom_printf("{\"wifiConfig\":0}");
          }
      }
      else{
        ESP_LOGE("wifi cfg", "RTRYC %d invalid\n", retry->valueint);
        Custom_printf("{\"wifiConfig\":0}");
      }
  }
	 else if (cJSON_IsObject(DLIR))
	 {

      cJSON *mode = cJSON_GetObjectItem(DLIR, "MODE");
      cJSON *but = cJSON_GetObjectItem(DLIR, "BUT");
      /* BUG-FIX #6: guard NULL deref on missing MODE/BUT keys. */
      if (!cJSON_IsNumber(mode) || !cJSON_IsNumber(but)) {
        ESP_LOGE("ACM", "DLIR payload missing/invalid MODE or BUT");
        cJSON_Delete(root); /* BUG-FIX #11 */
        return;
      }
       acm_log("DLIR mode=%d but=%d\n", mode->valueint, but->valueint);
      /* IsNumber checks already done in the guard above. */
      if (mode->valueint>=2 && mode->valueint <= 3){
        Test_Mode=mode->valueint;
        if (but->valueint >=1 && but->valueint <=16)
        {
          DIR_BUT=but->valueint-1;
        }
      }
	 }
   else if (cJSON_IsObject(ipconfig)||cJSON_IsObject(urtconfig)||cJSON_IsObject(dmbrconfig)||cJSON_IsObject(mbrconfig)||cJSON_IsObject(hltconfig))
   {
     //Convert the VALUE (the object or string) to a JSON string
     char *data_to_save =NULL;
     cJSON *configuration =NULL;
     if(cJSON_IsObject(ipconfig)){
      data_to_save = cJSON_PrintUnformatted(ipconfig);
      configuration = ipconfig;
     }
     else if(cJSON_IsObject(urtconfig)){
      data_to_save = cJSON_PrintUnformatted(urtconfig);
      configuration = urtconfig;
     }
     else if(cJSON_IsObject(dmbrconfig)){
      data_to_save = cJSON_PrintUnformatted(dmbrconfig);
      configuration = dmbrconfig;
      }
      else if(cJSON_IsObject(mbrconfig)){
      data_to_save = cJSON_PrintUnformatted(mbrconfig);
      configuration = mbrconfig;
      }
      else if(cJSON_IsObject(hltconfig)){
      data_to_save = cJSON_PrintUnformatted(hltconfig);
      configuration = hltconfig;
      }
      if (data_to_save != NULL) {
          acm_log("save %s\n", configuration->string);
          update_config_tag(config_path, configuration->string, data_to_save);
          free(data_to_save);
          Custom_printf("{\"%s\":1}",configuration->string);
      }

   }
   else if (cJSON_IsObject(otaconfig))
   {
    char port[6];
    char *data_to_save =NULL;
    cJSON *ota = cJSON_GetObjectItemCaseSensitive(root, "ota");
    if (cJSON_IsObject(ota)) {
        cJSON *zero_obj = cJSON_GetObjectItemCaseSensitive(ota, "0");
        if (cJSON_IsObject(zero_obj)) {
            cJSON *ota0 = cJSON_GetObjectItemCaseSensitive(zero_obj, "ota0");

            if (cJSON_IsString(ota0) && (ota0->valuestring != NULL)) {
                /* Field widths bound writes to url[128] and filepath[64]. */
                sscanf(ota0->valuestring, "%127[^:]:%hu:%63[^:]:%hu", httpserver_params.url, &httpserver_params.port, httpserver_params.filepath, &httpserver_params.is_ota);
                acm_log("ota %s:%d/%s en=%d\n", httpserver_params.url, httpserver_params.port, httpserver_params.filepath, httpserver_params.is_ota);
                data_to_save = cJSON_PrintUnformatted(ota0);
                update_config_tag(config_path, otaconfig->string, data_to_save);
                Custom_printf("{\"%s\":1}",ota0->string);
                free(data_to_save);
            }
        }
    }
   }
  // Free memory
  cJSON_Delete(root);
}

void parse_rdn_string(const char* rdn_str, profile_rdn_t* profile) {
    // We expect 5 values from "3:0:94:4:1"
    int parsed = sscanf_j(rdn_str, "%d:%d:%d:%d:%d", 
                         &profile->slave_id, 
                          &profile->reg_type, 
                          &profile->access_type, 
                          &profile->mod_add, 
                          &profile->bytes);
    if (parsed == 5) {
        acm_log("rdn s=%d r=%d a=%d m=%d b=%d\n",
                profile->slave_id, profile->reg_type, profile->access_type,
                profile->mod_add, profile->bytes);
        response_s.res = 2;
    } else {
        profile->Isrdndata_available = 0;
        acm_log("rdn parse err\n");
    }
}

void parseMQTT_revCmd(const char *json_string) {
  acm_log("mqtt rx\n");
  CMD_Recieved=1;
  response_s.res =3;
  cJSON *root = cJSON_Parse(json_string);
      if (root == NULL) return;

      // 1. Enter "ext" array (Index 0)
      cJSON *ext_array = cJSON_GetObjectItem(root, "ext");
      if (cJSON_IsArray(ext_array)) {
          cJSON *ext_item = cJSON_GetArrayItem(ext_array, 0);

          // 2. Enter "sii" object
          cJSON *sii = cJSON_GetObjectItem(ext_item, "sii");
          if (sii) {

              // 3. Enter the dynamic ID object (e.g., "1")
              // If the ID "1" changes, use sii->child to get the first available ID
              cJSON *id_obj = cJSON_GetObjectItem(sii, "1");
              if (id_obj) {
				          //Retrieve tid and cmd
                  cJSON *tid = cJSON_GetObjectItem(id_obj, "tid");
                  cJSON *cmd = cJSON_GetObjectItem(id_obj, "cmd");

                  if (cJSON_IsNumber(tid)) {
                      response_s.tid = tid->valueint;
                  }

                  switch (cmd->valueint){

                    case SET_ATT:
                    break;
                    case GET_ATT:
                    
                    cJSON *rdn_obj = cJSON_GetObjectItem(id_obj, "rdn");
                    if (cJSON_IsObject(rdn_obj)) {
                        // 1. Count how many profiles are in this specific JSON packet
                          rdn_data_ready=0;
                           total_rdn_profiles = 0;

                        cJSON *temp = NULL;
                        cJSON_ArrayForEach(temp, rdn_obj) { total_rdn_profiles++; }

                        if (total_rdn_profiles > 0) {
                            // 2. Dynamically allocate the array
                            profile_rdn_parms = (profile_rdn_t *)malloc(total_rdn_profiles * sizeof(profile_rdn_t));
                            
                            if (profile_rdn_parms == NULL) {
                                ESP_LOGE("MEM", "Failed to allocate %d profiles", total_rdn_profiles);
                                return; 
                            }
                            if (profile_rdn_parms) {
                                memset_j(profile_rdn_parms,sizeof(profile_rdn_t),0, total_rdn_profiles * sizeof(profile_rdn_t)); // Clear all 250 ints per struct
                            }
                            // 3. Fill the array
                            int i = 0;
                            cJSON *element = NULL;
                            cJSON_ArrayForEach(element, rdn_obj) {
                                // Store the dynamic key (e.g., "1", "2", "abc")
                                Jio_EA_strncpy_s(profile_rdn_parms[i].key_name, KEY_NAME_MX_LEN, 
                                                element->string, strlen(element->string));

                                // Parse the "3:0:94:4" string into the struct members
                                parse_rdn_string(element->valuestring, &profile_rdn_parms[i]);
                                i++;
                            }  
                      }
                    }
                   
                    break;
                    case SET_CONF:
                      cJSON *param = NULL;
                      cJSON_ArrayForEach(param, id_obj) {

                    	  int32_t diff = -1; // Default to 'not equal'

                          // 1. Check if the KEY name contains "dmbr"
                          if ((param->string != NULL && memcmp_j(param->string,4, "dmbr",4,&diff) == 0)||
                              (param->string != NULL && memcmp_j(param->string,3, "mbr",3,&diff) == 0)||
                              (param->string != NULL && memcmp_j(param->string,3, "hlt",3,&diff)==0)||
                              (param->string != NULL && memcmp_j(param->string,3, "ipc",3,&diff)==0)||
							                (param->string != NULL && memcmp_j(param->string,3, "urt",3,&diff)==0)) {

                              // 2. Convert the VALUE (the object or string) to a JSON string
                              char *data_to_save = cJSON_PrintUnformatted(param);

                              if (data_to_save != NULL) {
                                  acm_log("save %s\n", param->string);
                                  update_config_tag(config_path, param->string, data_to_save);
                                  free(data_to_save);
                                  response_s.res=2;
                                  rebootcnter=60;
                              }
                          }
                      }
                      // cJSON *dmbr1 = cJSON_GetObjectItem(id_obj, "dmbr1");
                      // if (cJSON_IsObject(dmbr1)) {
                      //   char *dmbr1_str = cJSON_PrintUnformatted(dmbr1);
                      //   if (dmbr1_str != NULL) {
                      //       printf("dmbr1 as string: %s\n", dmbr1_str);
                      //       update_config_tag(config_path,"dmbr1",dmbr1_str);
                      //       response_s.res=2;
                      //       free(dmbr1_str);
                      //   }
                      // }
                  break;
                  case GET_CONF:
                  break;
                  case DEL_CONF:
                      delete_config_file(config_path);
                      response_s.res=2;
                  break;
                  case REBOOT:
                	  /* BUG-FIX #17: previously every REBOOT command was honored,
                	     allowing a stuck cloud automation or replay attack to
                	     hold the device in a continuous reboot loop. If a reboot
                	     is already pending, ignore further requests. response_s.res
                	     is left at the function-entry value of 3 so the caller
                	     can distinguish "rate-limited" from "accepted (=2)". */
                	  if (reboot) {
                	      ESP_LOGW(TAG, "REBOOT already pending; ignoring duplicate");
                	      break;
                	  }
                	  reboot=1;
                	  response_s.res=2;
                	  break;
            		} 
            	}                   
            }
				  }
      cJSON_Delete(root);
      response_s.rev_cmd = 1;
}

char* generate_config_json(void) {
	char tms_buf[32] ;
	char MAC_buf[20];
  int8_t rssi=0;
  wifi_ap_record_t ap_info;
  if(esp_wifi_sta_get_ap_info(&ap_info)==ESP_OK && wifi_is_connected ){
	  rssi=ap_info.rssi;
  }
	else{  
		rssi=127;
  }
	get_MAC_string(MAC_buf,sizeof(MAC_buf));
    cJSON *root = cJSON_CreateObject();
  
    cJSON_AddStringToObject(root, "eafwv", version);
    cJSON_AddStringToObject(root, "reldt", release_date);

    char ip_str[16], gw_str[16], netmask_str[16];
    sprintf_j(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    sprintf_j(gw_str, sizeof(gw_str), IPSTR, IP2STR(&ip_info.gw));
    sprintf_j(netmask_str, sizeof(netmask_str), IPSTR, IP2STR(&ip_info.netmask));
    
    cJSON_AddStringToObject(root, "ssid", (char *)ap_info.ssid);
    cJSON_AddNumberToObject(root, "rssi", rssi);
    cJSON_AddNumberToObject(root, "blest", ble_is_connected); 
    cJSON_AddNumberToObject(root, "tmsyn", time_is_synced); 
    cJSON_AddStringToObject(root, "ip", ip_str);
    cJSON_AddStringToObject(root, "gw", gw_str);
    cJSON_AddStringToObject(root, "netmsk", netmask_str);
    static_ip_enabled? cJSON_AddStringToObject(root, "ip_conf", "static") : cJSON_AddStringToObject(root, "ip_conf", "dhcp");
    // 2. HLT (Health logic strings)
    cJSON *hlt = cJSON_CreateObject();
    char health[16];
    sprintf_j(health, sizeof(health), "%d:%d",hlt_configs[0].poll_frq,hlt_configs[0].push_frq);
    cJSON_AddItemToObject(root, "hlt", hlt);
    cJSON_AddStringToObject(hlt, "dhlt1", health);

    

    // 3. URT (UART Settings)
    char urtsetting[100];
    cJSON *urt = cJSON_CreateObject();
    sprintf_j(urtsetting, sizeof(urtsetting), "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d",
                            system_urts[0].urt_id, system_urts[0].baud,
                            system_urts[0].parity, system_urts[0].data_bits,
                            system_urts[0].stop_bits, system_urts[0].retry_cnt,
                            system_urts[0].retry_period,system_urts[0].DisconnectonNoA,
						    system_urts[0].DefConnStatus,system_urts[0].InterwriteDelay,
						    system_urts[0].InterReadDelay,system_urts[0].AutoConnect);
    cJSON_AddItemToObject(root, "urt", urt);
    cJSON_AddStringToObject(urt, "curt0", urtsetting);

    char *json_out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_out;
}


void ParseRxData(char *Buff, uint16_t BuffLen) {   
	char resp[150];
	uint16_t len=0;
  //Custom_printf("%s", Buff);
  char Trnsfer=0;
  wifi_ap_record_t ap_info;
  char *cmdstring = "SendLIRCmdByID"; // format should be "SendLIRCmdByID,00,00"
  char *sigPtr = strstr((char *)(Buff), cmdstring);
  if (sigPtr != NULL) {
    sigPtr += strlen((const char *)cmdstring) + 1; //+1 to skip comma','
    if (strlen((const char *)sigPtr) <
        5) // Discard if length"00,00"=5 is not valid
      return;
    return;
  }
 
  cmdstring = "REBOOT";
  sigPtr = strstr((const char *)(Buff), cmdstring);
  if (sigPtr != NULL) {
    ESP_LOGI(TAG, "Reboot command received!");
    esp_restart();
    return;
   }
   cmdstring = "PREFLAG";
    if (strstr((const char *)(Buff), cmdstring)){
      g_u8_addtlDisblFlgfrlg=1;
      acm_log("preflag\n");
      return;
    }
   cmdstring = "DLOG";
    if (strstr((const char *)(Buff), cmdstring)){
     log_endis=0;
     excutelog=1;
     acm_log("log=%d\n", log_endis);
      return;
    }
     cmdstring = "ENLOG";
    if (strstr((const char *)(Buff), cmdstring)){
     log_endis=1;
     excutelog=1;
     acm_log("log=%d\n", log_endis);
      return;
    }
      cmdstring = "DELIR";
    if (strstr((const char *)(Buff), cmdstring)){
     acm_log("del file\n");
      if(delete_file_example()==0)
        Custom_printf("\n Deleted");

    }
      cmdstring = "CONFIG";
    if (strstr((const char *)(Buff), cmdstring)){
     acm_log("gen cfg\n");
      char *payload = generate_config_json();
      Custom_printf("%s\n",payload);
      free(payload);
    }

  //      cmdstring = "PWR_ON_T";
  //   if (strstr((const char *)(Buff), cmdstring)){
  //      Test_Mode=2;     

  //   }
  //    cmdstring = "IRINFO";
  //   if (strstr((const char *)(Buff), cmdstring)){
	//    memset_j(resp, sizeof(resp), 0, sizeof(resp));
  //   sprintf_j(resp, sizeof(resp), "{\"IRINFO\":{\"IRTXFRQ\":%d,\"IRCMDINT\":1,\"IRLENLIM\":%d}}",frequency,irlenlim);
  //   Custom_printf("\n %s",resp);
  //    return;
  //   }
  //   cmdstring = "LRNSTS";
  //   if (strstr((const char *)(Buff), cmdstring)){
	//  uint8_t i=0;
	//  for(i=0;i<16;i++)
	//  {
  //    // if (Assert[0].buttons[i].length != 0) {
  //   //   btn_status[i] = 1;
  //   // } else {
  //     btn_status[i] = 0;
  //   //}
  //   }
  //   memset_j(resp, sizeof(resp), 0, sizeof(resp));
  //   sprintf_j(resp, sizeof(resp), "{\"LRNSTS\":[AC_01,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]}",btn_status[0],btn_status[1],btn_status[2],btn_status[3],btn_status[4],btn_status[5],btn_status[6],btn_status[7],btn_status[8],btn_status[9],btn_status[10],btn_status[11],btn_status[12],btn_status[13],btn_status[14],btn_status[15]);
  //   Custom_printf("\n %s",resp);
  //     return;
  //   }
     cmdstring = "IRSTS";
    if (strstr((const char *)(Buff), cmdstring)){
    int8_t rssi=0;
	if(esp_wifi_sta_get_ap_info(&ap_info)==ESP_OK && wifi_is_connected ){
	  rssi=ap_info.rssi;
  }
	else{  
		rssi=127;
  }
    memset_j(resp, sizeof(resp), 0, sizeof(resp));
    sprintf_j(resp, sizeof(resp), "{\"IRSTS\":%d,\"WIFI\":%d,\"RSSI\":%d,\"MQTT\":%d,\"BLE\":%d}",IR_TEST_Completed,wifi_is_connected,rssi,Mqtt_is_connected,ble_is_connected);
    Custom_printf("%s\n",resp);
      return;
    }
     cmdstring = "MAC";
    if (strstr((const char *)(Buff), cmdstring)){
    memset_j(resp, sizeof(resp), 0, sizeof(resp));
	  len = sprintf_j(resp, sizeof(resp), "{\"BLE_MAC\":\"%02X:%02X:%02X:%02X:%02X:%02X\",",mac_ble[0],mac_ble[1],mac_ble[2],mac_ble[3],mac_ble[4],mac_ble[5]);
  	len += sprintf_j(resp+len, sizeof(resp)-len, "\"WIFI_MAC\":\"%02X:%02X:%02X:%02X:%02X:%02X\",",mac_Wifi[0],mac_Wifi[1],mac_Wifi[2],mac_Wifi[3],mac_Wifi[4],mac_Wifi[5]);
    sprintf_j(resp+len, sizeof(resp)-len, "\"fwv\":\"%s\",\"reldt\":\"%s\"}",version,release_date);
    Custom_printf("%s\n",resp);
      return;
    }
  
  cmdstring = "LIRCmd";
  sigPtr = strstr((const char *)(Buff), (const char *)cmdstring);
  if (sigPtr != NULL) {
    sigPtr += strlen((const char *)cmdstring) + 1; //+1 to skip comma','

    if (strstr((const char *)(sigPtr), "FW_UPLOAD_START") != NULL)
    {
      Trnsfer=2;
     // g_u16_expctd_blknm=1;
    }
    else if (strstr((const char *)(sigPtr), "UPLOAD_START") != NULL)
    {
      Trnsfer=1;
    }
	if (Trnsfer ==1 ||Trnsfer==2) {
	  ESP_LOGI("BLE","%s",Buff);
  
     uint8_t ACKsend;
    if(Trnsfer==2)
    {
      g_u8_acceptfwRx =1;
	  g_u8_acceptFileRx=0;
    		if(ota_write_offset==0)
				{
					if(ota_start()==ESP_OK)
					{
						ESP_LOGI("FWUP", "OTA_START SUCCESSFUL");
            change_log_tag("FWUP");
					}
					else
					{
						ESP_LOGI("FWUP", "OTA_START NOT SUCCESSFUL");
						g_u8_acceptfwRx =0;
						//g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
						//ESP_LOGI("FWUP", "Total number of bytes buffered so far is %ld\n",g_u32_file_buffIndx);
						//ESP_LOGI("FWUP", "g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX\n");
					}
				}
	  uint16_t l_u16_ble_sndreslt=0;
      uint16_t Temp = (ota_write_offset/500)+1;
      memcpy_j((uint8_t *)&l_u16_ble_sndreslt, sizeof(l_u16_ble_sndreslt), (uint8_t *)&Temp, 2);
      g_u16_expctd_blknm = (ota_write_offset/500)+1;
      ESP_LOGI("BLE","Resume from %d",g_u16_expctd_blknm);
      ACKsend = ble_spp_SendData(2, (uint8_t *)&l_u16_ble_sndreslt);
      if(ACKsend == 0)
      {
        ESP_LOGI(TAG, "Start byte not send");
      }
      else
      {
        ESP_LOGI(TAG, "Start byte send Successfully");
      }
    }
    else
    {
      g_u16_expctd_blknm = 1;
      g_u8_acceptFileRx = 1;
      change_log_tag("XMODEM");
      l_u8_ack_nak = JIO_ESP_X_NAK;

      ACKsend = ble_spp_SendData(1, &l_u8_ack_nak);
    if(ACKsend == 0)
	  {
		  ESP_LOGI(TAG, "Failure in sending NAK to BLE App");
	  }
	  else
	  {
		  ESP_LOGI(TAG, "Succeeded in sending NAK to BLE App");
	  }
    }
  	g_u8_SendFileXMDMTx = 0;
	  g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
      g_u8_expr_cnt = 0;  
	  //()Send NAK packet to BLE App
	  //()ble_spp_SendData(strlen(printf_buffer), (uint8_t *)printf_buffer);
	  g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
	  // Start the timer (one-shot, 5 seconds = 5000000 us)
    ESP_LOGI(TAG, "Starting single-shot timer for 600 seconds.");
    ESP_ERROR_CHECK(esp_timer_start_once(one_shot_timer1, 1800*1000*1000)); // Timer period in microseconds

      return;
    }
	if (strstr((const char *)(sigPtr), "GET_LIRCMD_FILE") != NULL) {
      g_u8_SendFileXMDMTx = 1;
	  g_u8_acceptFileRx = 0;
    g_u8_acceptfwRx=0;
	  g_u8_expr_cnt = 0;
	  g_u16_nxt_blknm = 1;
	  l_u8_ack_nak = JIO_ESP_X_ACK;
	  uint8_t l_u8_ack_nak_arr[10];
	  memset_j(&l_u8_ack_nak_arr[0], sizeof(l_u8_ack_nak_arr), 0, sizeof(l_u8_ack_nak_arr));
	  //()Send NAK packet to BLE App
	  //()ble_spp_SendData(strlen(printf_buffer), (uint8_t *)printf_buffer);
	  for(int i = 0; i < 1; i++)
	  {
		  uint8_t l_u8_ble_sndreslt;
		  //l_u8_ble_sndreslt = ble_spp_SendData(1, &l_u8_ack_nak);
		  l_u8_ble_sndreslt = ble_spp_SendData(sizeof(l_u8_ack_nak_arr), l_u8_ack_nak_arr);
		  if(l_u8_ble_sndreslt == 0)
		  {
			  ESP_LOGI(TAG, "Failure in sending ACK to BLE App");
		  }
		  else
		  {
			  ESP_LOGI(TAG, "Succeeded in sending ACK to BLE App");
		  }
		  esp_rom_delay_us(2000000);
	  }
	  g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
	  g_u8_commtimeout = 0;
	  // Start the timer (one-shot, 5 seconds = 5000000 us)
	  ESP_LOGI(TAG, " single-shot timer Started for 90sec.");
	  ESP_ERROR_CHECK(esp_timer_start_once(one_shot_timer1, 90000000)); // Timer period in microseconds

	  //()Start a timer for 1 minute. Stop this timer on getting EOT or CAN.
	  //()if timer expires, set g_u8_acceptFileRx = 0
	  //()and g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
      return;
    }
  }

}
// Event handler function
static void AC_Cntrl_event_handler(void *handler_arg,
                                   esp_event_base_t event_base,
                                   int32_t event_id, void *event_data) {
  if (event_base == AC_CNTRL_EVENT_BASE) {
    ESP_LOGI(TAG, "Event received: base=%s, id=%d", event_base, (int)event_id);
    switch (event_id) {
    case EVENT_ID_MQTT_DATA_RX:
      ESP_LOGI(TAG, "Event received EVENT_ID_MQTT_DATA_RX");
      parseMQTT_revCmd((const char *)event_data);
      break;
    case EVENT_ID_SCHEDULE_ON:
      ESP_LOGI(TAG, "SCHEDULE_ON");
      //SendAssertCmdByID(PWR_ON, 01);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      //SendAssertCmdByID(PWR_ON, 01);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      IR_ON_seq=1;
      // SetAsset_Mode(POWER_ON_MODE, System_Config.PD_OnSetTemp_C, 7);
      break;
    case EVENT_ID_SCHEDULE_OFF:
      ESP_LOGI(TAG, "SCHEDULE_OFF");
      //SendAssertCmdByID(PWR_OFF, 01);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      //SendAssertCmdByID(PWR_OFF, 01);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      // SetAsset_Mode(POWER_OFF_MODE, System_Config.PD_OnSetTemp_C, 7);
      break;
    case EVENT_ID_MODE:
    //ESP_LOGI(TAG, "MODE set: %d OFFSET: %d",status_s.md,Modeofset(status_s.md));
    //SendAssertCmdByID(Modeofset(status_s.md), 01);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    break;
    case EVENT_ID_CSPT:
    //ESP_LOGI(TAG, "Temperature SET POINT: %d,OFFSET:%d ",status_s.csp,TEMPofset(status_s.csp));
    //SendAssertCmdByID(TEMPofset(status_s.csp), 01);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    break;
   
     case EVENT_ID_UART_DATA_RX :
     parse_WifiConfigCmd((char *)event_data);
	 ParseRxData((char *)event_data, strlen(event_data));
    break;
    
 case EVENT_ID_BLE_DATA_RX:

    parse_WifiConfigCmd((char *)event_data);
      // ESP_LOGI(TAG, "Received EVENT_ID_BLE_DATA_RX. Data: %s", (char
	  //()Call a function that shall Check if XModem packet is received, if so...
	  ESP_LOGI(TAG, "Value of g_u8_acceptFileRx = %d\n",g_u8_acceptFileRx);
	  
	  ESP_LOGI(TAG, "Value of g_u8_SendFileXMDMTx = %d\n",g_u8_SendFileXMDMTx);
    ESP_LOGI(TAG, "Value of g_u8_acceptfwRx = %d\n",g_u8_acceptfwRx);
	  if(g_u8_acceptFileRx == 1||g_u8_acceptfwRx==1)
	  {
		  
		  //()Call a function that shall commit received chunk of data in to a file. 
		  //()write operation in to a file shall include open a file, write in to a file and
		  //()close a file.
		  //()Send an ack / nack back to BLE device that initiated file transfer
		  //()Let this function shall maintain FSM to track packet exchanges
		  if(Jio_ESP_010102_LIRFile_Rx_FSM((uint8_t*)event_data) == JIO_ESP_SUCCESS)
		  {
			  ESP_LOGI(TAG, "Received LIR file chunk successfully from BLE APP");
			  //()Send ACK to BLE Device
			  l_u8_ack_nak = JIO_ESP_X_ACK;
			  uint8_t l_u8_ble_sndreslt;
			  l_u8_ble_sndreslt =  ble_spp_SendData(1, &l_u8_ack_nak);
			  if(l_u8_ble_sndreslt == 0)
			  {
				  ESP_LOGI(TAG, "Fail ACK to BLE App post receiving file chunk");
			  }
			  else
			  {
				  ESP_LOGI(TAG, "ACK Sent to BLE App post receiving file chunk");
			  }
				
		  }
		  else
		  {
			  ESP_LOGI(TAG, "Failed in Receiving LIR file chunk from BLE APP");
			  //()Send NAK to BLE Device
			  l_u8_ack_nak = JIO_ESP_X_NAK;
			  uint8_t l_u8_ble_sndreslt;
			  l_u8_ble_sndreslt =  ble_spp_SendData(1, &l_u8_ack_nak);
			  if(l_u8_ble_sndreslt == 0)
			  {
				  ESP_LOGI(TAG, "Fail in sending NAK to BLE App post receiving file chunk");
			  }
			  else
			  {
				  ESP_LOGI(TAG, " NAK sent to BLE App post receiving file chunk");
			  }
		  }
		  esp_rom_delay_us(100000);	  
		  
	  }
	  else if(g_u8_SendFileXMDMTx == 1)
	  {
		  ESP_LOGI(TAG, "Entering to Jio_ESP_010104_LIRFile_Tx_FSM\n");
		  memset_j(&g_u8_rx_data[0], sizeof(g_u8_rx_data), 0x00, sizeof(g_u8_rx_data));
		  memcpy_j(&g_u8_rx_data[0], sizeof(g_u8_rx_data), (uint8_t*)&event_data[0], 132);
		  
		  Jio_ESP_010104_LIRFile_Tx_FSM((uint8_t*)event_data);
	  }
	  else
	  {
     ESP_LOGI(TAG, "parse Event Data\n");
		 ParseRxData((char *)event_data, strlen(event_data));
	  }
      break;
    default:
      ESP_LOGI(TAG, "Unknown event ID: %ld", event_id);
      break;
    }
  }
}

void Controller_task(void *arg) {
  //LoadSystemConfig();
  g_u8_acceptFileRx = 0;
  g_u8_acceptfwRx=0;
  // Create the default event loop
  esp_event_loop_create_default();

  // Register the event handler for the custom event base
  esp_event_handler_instance_t instance;
  esp_event_handler_instance_register(AC_CNTRL_EVENT_BASE, ESP_EVENT_ANY_ID,
                                      AC_Cntrl_event_handler, NULL, &instance);

  while (1) {

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(rebootcnter>1){
        rebootcnter--;
        ESP_LOGI(TAG, "Reboot counter %d",rebootcnter);
    }
    if(rebootcnter==1)
      {
       ESP_LOGI(TAG, "Reboot command received!");
       esp_restart();
    }
  }
  vTaskDelete(NULL);
}
void Controller_init(void) {
  // Create a one-shot timer
    const esp_timer_create_args_t one_shot_timer_args = {
        .callback = &timer_callback_BLE_XMDM, // Set the callback function
        .name = "one_shot_timer1"     // Timer name (optional, for logging/debugging)
    };
  ESP_ERROR_CHECK(esp_timer_create(&one_shot_timer_args, &one_shot_timer1)); // Create the timer
  
  const esp_timer_create_args_t comtimeout_timer_args = {
        .callback = &timer_callback_BLE_CMTOUT, // Set the callback function
        .name = "cmtimeout_timer1"     // Timer name (optional, for logging/debugging)
    };
  ESP_ERROR_CHECK(esp_timer_create(&comtimeout_timer_args, &cmtimeout_timer1)); // Create the timer

  xTaskCreate(Controller_task, "Controller_task", 1024 * 8, NULL, 5, NULL);
}
