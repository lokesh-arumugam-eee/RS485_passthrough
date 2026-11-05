#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "main.h"
#include "esp_log.h"
#include "nanomodbus.h"
#include "uart/uart_app.h"
#include "modbus/data.h"
#include "configuration/configuration.h"
#include <inttypes.h>
#include "freertos/semphr.h"
#include "modbus_tcp.h"
#include "xmodem.h"
#include "nanomodbus.h"
#include "modbus_rtu.h"
#include "esp_task_wdt.h"
#include "wifiConnect.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "http.h"
#include "log_capture.h"
#include "controller.h"


#define TAG "MODBUS"

 #define UART1_PORT      UART_NUM_1
// #define UART1_TX_PIN    4
// #define UART1_RX_PIN    5
// #define UART1_BAUDRATE  9600
int run_rdn_cycle(void);

int run_polling_cycle(void);

/* Gate flag owned by chunked_ota.c; non-zero while an OTA download is active.
   We pause Modbus polling while it's set so the Wi-Fi/LWIP stack is free for
   the firmware transfer. */
extern int otataskstarted;
bool is_time_to_poll(int mbr_index);
nmbs_error safe_modbus_read(tcp_request_type_t request_type, uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf);
nmbs_error populate_modbus_write(uint8_t slave_id, uint16_t addr, uint16_t qty, const uint16_t* in_buf);
int current_slave_id =1;
//QueueHandle_t uart1_queue;
/* -------------------------------------------------------
   UART context structure (IMPORTANT)
------------------------------------------------------- */
typedef struct {
    uart_port_t uart_num;
} uart_ctx_t;

static uart_ctx_t uart_ctx = {
    .uart_num = UART1_PORT
};

nmbs_t nmbs_rtu_master;
nmbs_platform_conf rtu_conf;

SemaphoreHandle_t xModbus_Mutex = NULL;

/* -------------------------------------------------------
   NanoModbus callbacks
------------------------------------------------------- */
int32_t uart_read_cb(uint8_t* buf, uint16_t count, int32_t timeout_ms, void* arg)
{
    if (buf == NULL || count == 0) return -1;

    size_t total_read = 0;
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    // Keep reading until we get EXACTLY the number of bytes requested
    while (total_read < count) {
        TickType_t now = xTaskGetTickCount();
        if ((now - start_tick) >= timeout_ticks) {
            break; // Global timeout reached
        }

        int len = uart_read_bytes(UART_NUM_1,
                                  buf + total_read,
                                  count - total_read,
                                  1); // Wait only 1 tick at a time inside loop

        if (len > 0) {
            total_read += len;
        } else if (len < 0) {
            return -1; // Hardware error
        }
    }

    // Now, instead of seeing len:1 in your logs,
    // you should see the total count requested by the library.
    if (total_read > 0) {
        printf("Read %d/%d bytes\n", (int)total_read, count);
    }

    return (int32_t)total_read;
}

int32_t uart_write_cb(const uint8_t* buf, uint16_t count,int32_t timeout_ms, void* arg)
{
    uart_ctx_t *ctx = (uart_ctx_t*)arg;

		ESP_LOGI("MODBUS", "TX %d bytes", count);

		for (int i = 0; i < count; i++) {
			printf("%02X ", buf[i]);
		}
		printf("\n");

		int len = uart_write_bytes(ctx->uart_num, (const char*)buf, count);
		uart_wait_tx_done(ctx->uart_num, pdMS_TO_TICKS(timeout_ms));

	return len;
}
 void uart_flush_cb(nmbs_t* nmbs_rtu_master, void* arg)
{
    uart_ctx_t *ctx = (uart_ctx_t*)arg;
    uart_flush_input(ctx->uart_num);
}

 uint16_t modbus_crc16(const uint8_t* data, uint32_t length, void* arg)
 {
     uint16_t crc = 0xFFFF;

     for (uint32_t pos = 0; pos < length; pos++) {
         crc ^= (uint16_t)data[pos];

         for (int i = 0; i < 8; i++) {
             if (crc & 0x0001) {
                 crc >>= 1;
                 crc ^= 0xA001;
             } else {
                 crc >>= 1;
             }
         }
     }
     printf("crc argument: %p", arg);
     crc = (crc << 8) | (crc >> 8);
     return crc;
 }



/* -------------------------------------------------------
   Modbus Master Init
------------------------------------------------------- */


void modbus_master_init(void)
{
	rtu_conf.transport = NMBS_TRANSPORT_RTU;
	rtu_conf.read = uart_read_cb;
	rtu_conf.write = uart_write_cb;
	rtu_conf.flush = uart_flush_cb;   // ← ADD THIS
	rtu_conf.crc_calc = modbus_crc16;         // use internal CRC
	rtu_conf.arg = &uart_ctx;
	rtu_conf.initialized = 0xFFFFDEBE;

    ESP_LOGI("MODBUS", "read=%p write=%p arg=%p flush=%p",
             rtu_conf.read,
             rtu_conf.write,
             rtu_conf.arg,
			 rtu_conf.flush);

    ESP_LOGI(TAG, "Creating NanoModbus...");

    nmbs_error err = nmbs_client_create(&nmbs_rtu_master, &rtu_conf);
    ESP_LOGI("MODBUS", "Create result: %d", err);

    if (err != NMBS_ERROR_NONE) {
        ESP_LOGE(TAG, "NanoModbus create failed");
        return;
    }

    /* Set slave ID */
    nmbs_set_destination_rtu_address(&nmbs_rtu_master, 1);

    nmbs_set_read_timeout(&nmbs_rtu_master, 2000);
    nmbs_set_byte_timeout(&nmbs_rtu_master, 500);
    ESP_LOGI(TAG, "Modbus Master Ready");
    ESP_LOGI(TAG, "read=%p write=%p arg=%p",
         rtu_conf.read,
         rtu_conf.write,
         rtu_conf.arg);
    ESP_LOGI(TAG, "modbus_master_init called");
}


/* -------------------------------------------------------
   Example Read Task
------------------------------------------------------- */
void modbus_read_task(void* arg)
{
    static int ota_paused_logged = 0;

    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();
        //ESP_LOGI(TAG, "Stack free: %u",uxTaskGetStackHighWaterMark(NULL));
        if (otataskstarted) {
            if (!ota_paused_logged) {
                ESP_LOGI(TAG, "OTA in progress, pausing Modbus polling");
                ota_paused_logged = 1;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (ota_paused_logged) {
            ESP_LOGI(TAG, "OTA finished, resuming Modbus polling");
            ota_paused_logged = 0;
        }
        run_rdn_cycle();
        run_polling_cycle();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
int run_rdn_cycle(void) {
    uint16_t group_regs[MAX_DATA_BYTES];
    static int Previous_return=0;

    if (total_rdn_profiles == 0)
    {
        if (Previous_return != -1)
            ESP_LOGI(TAG, "total_rdn_profiles is NULL\n");
        Previous_return=-1;
    	return -1;
    }

    if (profile_rdn_parms == NULL)
    {
        if (Previous_return != -1)
            ESP_LOGI(TAG, "profile_rdn_parms is NULL\n");
        Previous_return=-1;
    	return -1;
    }

    if (rdn_data_ready == 1) {
        if (Previous_return != -1)
            ESP_LOGI(TAG, "RDN Data is already available\n");
        Previous_return=-1;
        return -1; 
    }
    for (int p_idx = 0; p_idx < total_rdn_profiles; p_idx++) {
        
		 profile_rdn_t *p = &profile_rdn_parms[p_idx];
		
		 printf("  -> Reading %s at Modbus Addr: %d\n", p->key_name, p->mod_add);
		 
		 if( p->mod_add == 0)
		  {
			  printf("No Modbus Address for p->key_name:%s\n",p->key_name);
              p->Isrdndata_available = 2;
			  continue;
		  }
		
		  if( p->slave_id == 0)
		   {
			  printf("No Slave Id for p->key_name:%s\n",p->key_name);
              p->Isrdndata_available = 3;
			  continue;
		   }
		
		  if( p->bytes == 0)
		   {
			  printf("No of bytes is invalid for p->key_name:%s\n",p->key_name);
              p->Isrdndata_available = 4;
			  continue;
		   }
			
			int current_span_regs = p->bytes / 2;

			nmbs_error err = safe_modbus_read(READ_HOLDING_REGISTERS,p->slave_id, p->mod_add - 1, current_span_regs, group_regs);
			char A=0,B=0;	
			if (err == NMBS_ERROR_NONE) {

                for (int l = 0; l < current_span_regs; l++){
                    A = (char)(group_regs[l] >> 8) & 0xFF;
                    B = (char)group_regs[l] & 0xFF;
                    p->data[l*2] = A;
                    p->data[l*2+1] = B;
                }
                p->Isrdndata_available = 1;
			  } 
			  else {
				ESP_LOGE(TAG, "Gap-Group Read Error: %d at Addr %d", err, p->mod_add);
                p->Isrdndata_available = 5; // Mark as error
			 }
       }
       Previous_return=1;
       rdn_data_ready=1;
	    return 1;
}

int run_polling_cycle(void){ 
    uint16_t group_regs[MAX_DATA_BYTES];
    uint8_t intermediate[10];
    static int Previous_return_value=0;

    if (total_configs == 0)
    {
    	if (Previous_return_value != -1)
            ESP_LOGI(TAG, "total Configs is NULL\n");
        Previous_return_value =-1;
    	return -1;
    }

    if (system_configs == NULL)
    {
         if (Previous_return_value != -1)
            ESP_LOGI(TAG, "total Configs is NULL\n");
    	ESP_LOGI(TAG, "No System config present\n");
    	return -1;
    }

    for (int i = 0; i < total_configs; i++) {
         int printmessage=0;
         mbr_config_t *dev = &system_configs[i];
		 if (is_time_to_poll(i)) {
            
            for (int j = 0; j < dev->profile_count; j++) {
				
				 int p_idx = dev->profile_start_idx + j;
				 profile_config_t *p = &profile_parms[p_idx];
				
				 int current_group = p->group;
				 if( p->mod_add == 0)
				  {
					  printf("No Modbus Address,Skip Reading Poll for profile_parms at %d\n for p->key_name:%s\n", j,p->key_name);
					  continue;
				  }
				
				  if( dev->slave_id == 0)
				   {
					  printf("No Slave Id,Skip Reading Poll for profile_parms at %d\n for p->key_name:%s\n", j,p->key_name);
					  continue;
				   }
				
				  if( p->bytes == 0)
				   {
					  printf("No of bytes is invalid ,Skip Reading Poll for profile_parms at %d\n for p->key_name:%s\n", j,p->key_name);
					  continue;
				   }

                    if (p->reg_type ==5 ) // Holding Register
                    {
                        ESP_LOGE(TAG, "Skip Reading: %d", p->mod_add);
                        continue; // Skip read for this profile as reg_type 5 is used to disable read
                    }
                    if (printmessage == 0)
                    {
                        printf("--- Polling Device: %s ---\n", dev->key_name);
                        printmessage=1;
                    }
                    printf("  -> Reading %s at Modbus Addr: %d\n", p->key_name, p->mod_add);
					memset_j(group_regs, sizeof(group_regs), 0, sizeof(group_regs));
				    int group_count = 1;
				    int current_span_regs = p->bytes / 2;
				    int max_gap_regs = 16 / 2; // 8 registers limit
                    printf("Initial Span: %d regs\n", current_span_regs);
				    // 1. Look ahead to find parameters within the gap limit
				    for (int k = j + 1; k < dev->profile_count; k++) {
				        profile_config_t *next_p = &profile_parms[dev->profile_start_idx + k];
				        
				        // Calculate the gap between current end and next start
				        int current_end_addr = p->mod_add + current_span_regs;
				        int gap = next_p->mod_add - current_end_addr;
				
				        // If the gap is positive and within our limit (8 regs), include it
				        if (next_p->group == current_group && gap >= 0 && gap <= max_gap_regs) {
				            group_count++;
				            current_span_regs += gap + (next_p->bytes / 2);
				        } else {
				            break; // Gap too large or addresses out of order
				        }
				    }

				    nmbs_error err; 
				    // 2. Execute Bulk Read including the "invisible" gap registers
                    if (dev->poll_ip.addr != 0 && ip_config[0].St_ip_En_dis==0) {
                        err = modbus_multi_server_read(dev->slave_id, dev->poll_ip, dev->modbus_port, p->mod_add - 1, current_span_regs, group_regs);
                    }
                    else
                    {
                        err = safe_modbus_read(READ_HOLDING_REGISTERS, dev->slave_id, p->mod_add - 1, current_span_regs, group_regs);
                    }
				    if (err == NMBS_ERROR_NONE) {
				        for (int k = 0; k < group_count; k++) {
				            profile_config_t *curr_p = &profile_parms[p_idx + k];
				            
				            // Calculate the specific offset for this parameter inside the gap-filled buffer
				            int offset = curr_p->mod_add - p->mod_add;
				            int invalid_data=0;
				            for (int l = offset; l < curr_p->bytes; l++){
				            	 if (group_regs[l]==32768)
				            	 {
				            		 invalid_data=1;
				            		 ESP_LOGI(TAG, "Invalid Data detected");
				            		 break;
				            	 }
				            }
				            if (invalid_data) break;

				            apply_endianness(&group_regs[offset], intermediate, curr_p->bytes, curr_p->Endian_type);
				            float val = convert_to_value(intermediate, curr_p);
				            curr_p->data = val > 0.000001 ? val : 0;
				            curr_p->Isdata_available = 1;
					    	ESP_LOGI(TAG, "%s= %02f\n", curr_p->key_name, curr_p->data);
				        }
				        
				        // Skip processed parameters
				      j += (group_count - 1); 
				      } 
				      else {
				    	ESP_LOGE(TAG, "Gap-Group Read Error: %d at Addr %d", err, p->mod_add);
				     }
				}
       }
    }
    Previous_return_value=1;
    return 1;
}

bool is_time_to_poll(int mbr_index) {
    // Get current time in seconds
    uint32_t now = (uint32_t)time(NULL);

    // Calculate elapsed time
    uint32_t elapsed = now - last_poll_times[mbr_index];
    if(mbr_index ==0){
        if(elapsed%5==0){
    	    printf("elapsed time :%"PRIu32"\n",elapsed);
        }
    }
    if (elapsed >= system_configs[mbr_index].poll_frq) {
    	last_poll_times[mbr_index] = now;
        return true;
    }

    return false;
}

nmbs_error safe_modbus_read(tcp_request_type_t request_type, uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf) {
    nmbs_error err = NMBS_ERROR_NONE;

    // 1. Try to take the Mutex (Wait up to 1 second)
    if (xSemaphoreTake(xModbus_Mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {

        if(current_slave_id !=slave_id){
            current_slave_id = slave_id;
            nmbs_set_destination_rtu_address(&nmbs_rtu_master, current_slave_id);
            printf("slave id :%d",current_slave_id);
		}
        // 2. Perform the actual RS485 physical communication
        switch (request_type) {
            case READ_HOLDING_REGISTERS:
                err = nmbs_read_holding_registers(&nmbs_rtu_master, addr, qty, out_buf);
                break;
            case READ_INPUT_REGISTERS:
                err = nmbs_read_input_registers(&nmbs_rtu_master, addr, qty, out_buf);
                break;
            case WRITE_MULTIPLE_REGISTERS:
                 err = nmbs_write_multiple_registers(&nmbs_rtu_master, addr, qty, out_buf);
                break;
            default:
                err = NMBS_ERROR_INVALID_ARGUMENT;
        }

        // 3. IMPORTANT: Release the Mutex so other tasks can use it
        xSemaphoreGive(xModbus_Mutex);
    } else {
        // We couldn't get the bus (it's busy with another task)
        ESP_LOGW("RS485", "Bus Busy! Could not acquire Mutex.");
        err = NMBS_ERROR_TIMEOUT;
    }
    
    return err;
}

/**
 * @brief Safely populates a buffer with device settings based on Modbus address
 * * @param slave_id The ID of the device (used for logging/filtering)
 * @param addr     The starting register address (1-based per your table)
 * @param qty      Number of registers to read
 * @param out_buf  The destination buffer provided by the Modbus stack
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ADDR on failure
 */
nmbs_error populate_modbus_read_holding(uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf) {
    uint32_t ip;
    uint16_t Temp;
    int len=0;
    // 1. Boundary Check: Min address is 0, Max is 5
    if ((addr + qty - 1) > CONFIG_MAX_MODBUS_ADDRESS) {
        ESP_LOGW("MODBUS", "Illegal Address Read: Addr %d, Qty %d", addr, qty);
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
    }

    // 2. Map Addresses to Data
    // We use a loop to handle 'qty' in case the master requests multiple registers
    for (uint16_t i = 0; i < qty;) {
        uint16_t current_addr = addr;
        switch (current_addr) {
            case ADD_BAUDRATE: 
             out_buf[i] = (uint16_t)(system_urts[0].baud>>16); 
             i++;
             out_buf[i] = (uint16_t)(system_urts[0].baud & 0xFFFF); 
             i++;
             break;
            case ADD_PARITY: out_buf[i] = system_urts[0].parity;  i++; break;
            case ADD_DATABITS: out_buf[i] = system_urts[0].data_bits; i++;   break;
            case ADD_STOP_BITS: out_buf[i] = system_urts[0].stop_bits; i++;   break;
            case ADD_RETRY_CNT: out_buf[i] = system_urts[0].retry_cnt; i++;   break;
            case ADD_RETRY_PRD: out_buf[i] = system_urts[0].retry_period; i++; break;
            case ADD_RESERVED_17:
                out_buf[i] = 0; i++;
                break;

            case ADD_STATIC_IP:   //IP ADDRESS
                ip = ip_info.ip.addr;
                // Register 7: Bytes 0 and 1 (192.168)
                printf("Static IP: %ld\n",ip);
                printf("Static IP: " IPSTR "\n",IP2STR(&ip_info.ip));
                Temp = (uint16_t)(ip & 0xFFFF);
                Temp = (Temp << 8) | (Temp >> 8);
                out_buf[i] = Temp;
                i++;
                if (qty > i) {
                    // Register 8: Bytes 2 and 3 (1.50)
                    Temp= (uint16_t)((ip >> 16) & 0xFFFF);
                    Temp = (Temp << 8) | (Temp >> 8);
                    out_buf[i] = Temp;
                    i++;
                }
                break;
            case ADD_GW_IP: //GATEWAY IP ADDRESS
                ip = ip_info.gw.addr;
                // Register 7: Bytes 0 and 1 (192.168)
                Temp = (uint16_t)(ip & 0xFFFF);
                Temp = (Temp << 8) | (Temp >> 8);
                out_buf[i] = Temp;
                i++;
                if (qty >i) {
                    // Register 8: Bytes 2 and 3 (1.50)
                    Temp= (uint16_t)((ip >> 16) & 0xFFFF);
                    Temp = (Temp << 8) | (Temp >> 8);
                    out_buf[i] = Temp;
                    i++;
                }
                break;
            case ADD_NW_MASK: //NETMASK IP ADDRESS
                ip = ip_info.netmask.addr;
                // Register 7: Bytes 0 and 1 (192.168)
                Temp = (uint16_t)(ip & 0xFFFF);
                Temp = (Temp << 8) | (Temp >> 8);
                out_buf[i] = Temp;
                i++;
                if (qty >i) {
                    // Register 8: Bytes 2 and 3 (1.50)
                    Temp= (uint16_t)((ip >> 16) & 0xFFFF);
                    Temp = (Temp << 8) | (Temp >> 8);
                    out_buf[i] = Temp;
                    i++;
                }
                break;
            case ADD_TCP_MB_PORT: out_buf[i] = ip_config[0].port; i++; break;
            case ADD_TCP_PT_PORT: out_buf[i] = ip_config[0].pass_through_port; i++; break;
            case ADD_HTTP_LISTEN_PORT: out_buf[i] = ip_config[0].http_port; i++; break;
            case ADD_EN_STATIC_IP: out_buf[i] = ip_config[0].St_ip_En_dis; i++; break;
            case ADD_SRVR_TIMEOUT: out_buf[i] = ip_config[0].server_timeout; i++; break;
            case ADD_PASSTHR_RETRY_CNT: out_buf[i] = ip_config[0].Passthr_retry_cnt; i++; break;
            case ADD_OTA_HTTP_URL: //HTTP_IP Address
            len = sizeof(httpserver_params.url);
                if(qty>len/2) {
                    ESP_LOGW("MODBUS", "Requested quantity exceeds URL length. Adjusting to fit.");
                    qty = len / 2; // Each register holds 2 chars
                }
                for (int j = 0; j < qty; j++) {
                // Pack two chars into one 16-bit register
                uint8_t char1 = httpserver_params.url[(j * 2)];
                uint8_t char2 = httpserver_params.url[(j * 2) + 1];
                out_buf[i] = (char1 << 8) | char2;
                i++;
            }
            break;
            case ADD_OTA_HTTP_PORT: out_buf[i] = httpserver_params.port; i++; break;
            case ADD_OTA_HTTP_FILE_NAME: //HTTP File name
            len = sizeof(httpserver_params.filepath);
                if(qty>len/2) {
                    ESP_LOGW("MODBUS", "Requested quantity exceeds URL length. Adjusting to fit.");
                    qty = len / 2; // Each register holds 2 chars
                }
                for (int j = 0; j < qty; j++) {
                // Pack two chars into one 16-bit register
                uint8_t char1 = httpserver_params.filepath[(j * 2)];
                uint8_t char2 = httpserver_params.filepath[(j * 2) + 1];
                out_buf[i] = (char1 << 8) | char2;
                i++;
            }
            break;
            case ADD_TRIGGER_FOTA:
                uint16_t trigger = httpserver_params.is_ota;
                if (trigger == 100) {
                   trigger =1;
                }
                out_buf[i] = trigger; i++;
            break;
            case ADD_LOG_TRIGGER:  out_buf[i] = log_params.trigger;      i++; break;
            case ADD_LOG_DURATION: out_buf[i] = log_params.duration_sec; i++; break;
            case ADD_LOG_STATUS:   out_buf[i] = log_params.status;       i++; break;
            case ADD_LOG_PATH: //log upload URL path prefix (string, packed 2 chars/reg)
                len = sizeof(log_params.path);
                if (qty > len / 2) {
                    ESP_LOGW("MODBUS", "Requested quantity exceeds path length. Adjusting.");
                    qty = len / 2;
                }
                for (int j = 0; j < qty; j++) {
                    uint8_t c1 = log_params.path[(j * 2)];
                    uint8_t c2 = log_params.path[(j * 2) + 1];
                    out_buf[i] = (c1 << 8) | c2;
                    i++;
                }
                break;
            default:
                return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    printf("value filled in buffer %d\n", out_buf[0]);
    ESP_LOGI("MODBUS", "Slave %d: Successful read from Addr %d", slave_id, addr);
    return NMBS_ERROR_NONE;
}
/**
 * @brief Safely populates a buffer with device settings based on Modbus address
 * * @param slave_id The ID of the device (used for logging/filtering)
 * @param addr     The starting register address (1-based per your table)
 * @param qty      Number of registers to read
 * @param out_buf  The destination buffer provided by the Modbus stack
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ADDR on failure
 */
nmbs_error populate_modbus_read_input(uint8_t slave_id, uint16_t addr, uint16_t qty, uint16_t* out_buf) {
    uint32_t ip;
    uint16_t Temp;
    int len=0;
    // 1. Boundary Check: Min address is 0, Max is 5
    if ((addr + qty - 1) > CONFIG_MAX_MODBUS_ADDRESS) {
        ESP_LOGW("MODBUS", "Illegal Address Read: Addr %d, Qty %d", addr, qty);
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
    }

    // 2. Map Addresses to Data
    // We use a loop to handle 'qty' in case the master requests multiple registers
    for (uint16_t i = 0; i < qty;) {
        uint16_t current_addr = addr;
        switch (current_addr) {
            case ADD_FW_VER: 
            uint32_t fw_ver = (FW_VERHIGH * 1000000 + FW_VERLOW*1000 + FW_VERPATCH);
            printf("FW Version:%ld",fw_ver);
            out_buf[i] = (uint16_t)(fw_ver >> 16); i+=1;
            out_buf[i] = (uint16_t)(fw_ver & 0xFFFF); i+=1;
            break; 
            case ADD_REL_DATE:
            uint32_t rel_date = (REL_DA  * 1000000 + REL_MO * 10000 + REL_YR);
            printf("Release date:%ld",rel_date);
            out_buf[i] = (uint16_t)(rel_date >> 16); i+=1;
            out_buf[i] = (uint16_t)(rel_date & 0xFFFF); i+=1;
            break;
            case ADD_WIFI_MAC:
            out_buf[i] = (uint16_t)(mac_Wifi[0] << 8 | mac_Wifi[1]); i+=1;
            out_buf[i] = (uint16_t)(mac_Wifi[2] << 8 | mac_Wifi[3]); i+=1;
            out_buf[i] = (uint16_t)(mac_Wifi[4] << 8 | mac_Wifi[5]); i+=1;
            break;
            case ADD_BLE_MAC:
            out_buf[i] = (uint16_t)(mac_ble[0] << 8 | mac_ble[1]); i+=1;
            out_buf[i] = (uint16_t)(mac_ble[2] << 8 | mac_ble[3]); i+=1;
            out_buf[i] = (uint16_t)(mac_ble[4] << 8 | mac_ble[5]); i+=1;
            break;
            case ADD_WIFI_RSSI: {
            wifi_ap_record_t ap_info;
            int16_t rssi = 127;
            if (wifi_is_connected && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                rssi = ap_info.rssi;
            }
            out_buf[i] = (uint16_t)rssi; i++;
            break;
            }
            case ADD_FOTA_STS:out_buf[i] = httpserver_params.ota_status; i++;break;
            case ADD_FOTA_PROGRESS: out_buf[i] = httpserver_params.ota_percentage; i++;break;
            case ADD_FOTA_FAILURE_REASON: out_buf[i] = httpserver_params.ota_failure_reason; i++;break;

            default: 
                return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    printf("value filled in buffer %d\n", out_buf[0]);
    ESP_LOGI("MODBUS", "Slave %d: Successful read from Addr %d", slave_id, addr);
    return NMBS_ERROR_NONE;
}


/**
 * @brief Safely populates a buffer with device settings based on Modbus address
 * * @param slave_id The ID of treturn NMBS_ERROR_NONE;he device (used for logging/filtering)
 * @param addr     The starting register address (1-based per your table)
 * @param qty      Number of registers to read
 * @param out_buf  The destination buffer provided by the Modbus stack
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ADDR on failure
 */
nmbs_error populate_modbus_write(uint8_t slave_id, uint16_t addr, uint16_t qty, const uint16_t* in_buf) {
    uint32_t ip;
    int len=0;
    //----------------------------------------------------------------
    urt_config_t temp_urts= system_urts[0];
    ip_config_t temp_ip_config = ip_config[0];
    int uart_update_needed=0;
    int ip_update_needed=0;
    int http_update_needed=0;
    uint32_t Temp=0;
    uint16_t u16Temp=0;
    printf("buffer values : %04X %04X\n", in_buf[0], in_buf[1]);
    if(ADD_DELETE_FILE == addr) {
        // Handle delete file logic
        if (in_buf[0] == 0x0A0A && rebootcnter==0) {
            ESP_LOGW("MODBUS", "Recieved Delete File command:");
            rebootcnter=10;
            return NMBS_ERROR_NONE;
        }
        else
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
            ESP_LOGW("MODBUS", "Invalid value for Delete File command: %04X. Expected 0x0A0A.", in_buf[0]);
        }
    }
    if (ADD_REBOOT_DEVICE == addr)
    {
         if (in_buf[0] == 0x0A0A)
         {
            acm_log("del file\n");
            if(delete_file_example()==0)
                Custom_printf("\n Deleted");
            return NMBS_ERROR_NONE;
         }
         else
         {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
         }
        /* code */
    }
    
    //----------------------------------------------------------------
    // 1. Boundary Check: Min address is 0, Max is 5
    if ((addr + qty - 1) > CONFIG_MAX_MODBUS_ADDRESS) {
        ESP_LOGW("MODBUS", "Illegal Address Read: Addr %d, Qty %d", addr, qty);
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
    }

    // 2. Map Addresses to Data
    // We use a loop to handle 'qty' in case the master requests multiple registers
    for (uint16_t i = 0; i < qty;) {
        uint16_t current_addr = addr;
        switch (current_addr) {
            case ADD_FW_VER:
                 return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
                i+=2;break;
            case ADD_REL_DATE:
                 return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
                i+=2;break;
            case ADD_WIFI_MAC:
                 return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
                i+=2;break;
            case ADD_BLE_MAC:
                 return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; 
                i+=2;break;
            case ADD_BAUDRATE: 
                printf("buffer values %d: %04X %04X\n",i, in_buf[i], in_buf[i + 1]);
                uint32_t baudrate = ((uint32_t)in_buf[i] << 16) | in_buf[i + 1];
                
                printf("Baud rate received: %ld\n", baudrate);
			     if (baudrate>=4800 && baudrate<=115200) {
			         temp_urts.baud = baudrate;
                     system_urts[0].baud = temp_urts.baud;
                     uart_update_needed=1;
                     ESP_LOGI("MODBUS","baud rate recived:%ld\n",temp_urts.baud);
			     }
                 else
                 {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                 }
                 i+=2;
			     break;
            case ADD_PARITY:
                if (in_buf[i] <= 2) {
                    temp_urts.parity = in_buf[i];
                    system_urts[0].parity = temp_urts.parity;
                    uart_update_needed=1;
                    ESP_LOGI("MODBUS","parity recived:%d\n",temp_urts.parity);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid parity value: %d. Must be 0 (None), 1 (Even), or 2 (Odd).", in_buf[i]);
                }
                i++; 
                break;
            case ADD_DATABITS: 
                if (in_buf[i] >= 7 && in_buf[i] <= 8) {
                    temp_urts.data_bits = in_buf[i];
                    system_urts[0].data_bits = temp_urts.data_bits;
                    uart_update_needed=1;
                    ESP_LOGI("MODBUS","Data bits recived:%d\n",temp_urts.parity);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid data bits value: %d. Must be between 5 and 8.", in_buf[i]);
                }
                i++;  
                break;
            case ADD_STOP_BITS: 
                if (in_buf[i] >= 1 && in_buf[i] <= 2) {
                    temp_urts.stop_bits = in_buf[i];
                    system_urts[0].stop_bits = temp_urts.stop_bits;
                    uart_update_needed=1;
                    ESP_LOGI("MODBUS","stop bits recived:%d\n",temp_urts.stop_bits);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid stop bits value: %d. Must be 1 or 2.", in_buf[i]);
                }
                i++;   
                break;
            case ADD_RETRY_CNT: 
                if (in_buf[i] >= 0 && in_buf[i] <= 255) {
                    temp_urts.retry_cnt = in_buf[i];
                    system_urts[0].retry_cnt = temp_urts.retry_cnt;
                    uart_update_needed=1;
                     ESP_LOGI("MODBUS","retry recived:%d\n",temp_urts.retry_cnt);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid retry count: %d. Must be between 0 and 255.", in_buf[i]);
                }
                i++;   
                break;
            case ADD_RETRY_PRD: 
                if (in_buf[i] >= 0 && in_buf[i] <= 65535) {
                    temp_urts.retry_period = in_buf[i];
                    system_urts[0].retry_period = temp_urts.retry_period;
                    uart_update_needed=1;
                    ESP_LOGI("MODBUS","ADD_RETRY_PRD recived:%d\n",temp_urts.retry_period);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid retry period: %d. Must be between 0 and 65535 ms.", in_buf[i]);
                }
                i++; 
                break;
            case ADD_RESERVED_17:
                i++;
                break;
            case ADD_STATIC_IP:   //IP ADDRESS
                Temp = (uint32_t)((in_buf[i+1]<<8|in_buf[i+1]>>8) & 0xFFFF);
                Temp<<= 16;
                u16Temp = (uint16_t)((in_buf[i]<<8|in_buf[i]>>8) & 0xFFFF);
                Temp|=(uint32_t)(u16Temp);
                temp_ip_config.st_ip_info.ip.addr = Temp;
                ip_config[0].st_ip_info.ip.addr = temp_ip_config.st_ip_info.ip.addr;
                ip_update_needed=1;
                i+=2;
                ESP_LOGI("MODBUS","ADD_STATIC_IP recived:" IPSTR "\n",IP2STR(&temp_ip_config.st_ip_info.ip));
                break;
            case ADD_GW_IP: //GATEWAY IP ADDRESS
                Temp = (uint32_t)((in_buf[i+1]<<8|in_buf[i+1]>>8) & 0xFFFF);
                Temp<<= 16;
                u16Temp = (uint16_t)((in_buf[i]<<8|in_buf[i]>>8) & 0xFFFF);
                Temp|=(uint32_t)(u16Temp);
                temp_ip_config.st_ip_info.gw.addr = Temp;
                ip_config[0].st_ip_info.gw.addr = temp_ip_config.st_ip_info.gw.addr;
                ip_update_needed=1;
                ESP_LOGI("MODBUS","ADD_GW_IP recived:" IPSTR "\n",IP2STR(&temp_ip_config.st_ip_info.gw));
                i+=2;
                break;
            case ADD_NW_MASK: //NETMASK IP ADDRESS
                Temp = (uint32_t)((in_buf[i+1]<<8|in_buf[i+1]>>8) & 0xFFFF);
                Temp<<= 16;
                u16Temp = (uint16_t)((in_buf[i]<<8|in_buf[i]>>8) & 0xFFFF);
                Temp|=(uint32_t)(u16Temp);
                temp_ip_config.st_ip_info.netmask.addr = Temp;
                ip_config[0].st_ip_info.netmask.addr = temp_ip_config.st_ip_info.netmask.addr;
                ip_update_needed=1;
                ESP_LOGI("MODBUS","ADD_NW_MASK recived:" IPSTR "\n",IP2STR(&temp_ip_config.st_ip_info.netmask));
                i+=2;
                break;
            case ADD_TCP_MB_PORT: 
                if(in_buf[i]>0 && in_buf[i]<=65535){
                    temp_ip_config.port = in_buf[i]; 
                    ip_config[0].port = temp_ip_config.port;
                    ip_update_needed=1;
                    ESP_LOGI("MODBUS","ADD_TCP_PORT recived:%d\n",temp_ip_config.port);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid TCP port: %d. Must be between 1 and 65535.", in_buf[i]);
                }
                i++; break;
            
                case ADD_TCP_PT_PORT: 
                if(in_buf[i]>0 && in_buf[i]<=65535){
                    temp_ip_config.pass_through_port = in_buf[i]; 
                    ip_config[0].pass_through_port = temp_ip_config.pass_through_port;
                    ip_update_needed=1;
                    ESP_LOGI("MODBUS","ADD_TCP_PT_PORT recived:%d\n",temp_ip_config.pass_through_port);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid TCP port: %d. Must be between 1 and 65535.", in_buf[i]);
                }
                i++; break;
            
                case ADD_HTTP_LISTEN_PORT: 
                if(in_buf[i]>0 && in_buf[i]<=65535){
                    temp_ip_config.http_port = in_buf[i]; 
                    ip_config[0].http_port = temp_ip_config.http_port;
                    ip_update_needed=1;
                    ESP_LOGI("MODBUS","ADD_HTTP_LISTEN_PORT recived:%d\n",temp_ip_config.http_port);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid TCP port: %d. Must be between 1 and 65535.", in_buf[i]);
                }
                i++; break;
            case ADD_EN_STATIC_IP: 
            if(in_buf[i] == 0 || in_buf[i] == 1) {
                temp_ip_config.St_ip_En_dis = in_buf[i];
                ip_config[0].St_ip_En_dis = temp_ip_config.St_ip_En_dis;
                ip_update_needed=1;
                ESP_LOGI("MODBUS","ADD_EN_STATIC_IP recived:%d\n",temp_ip_config.St_ip_En_dis);
            }
            else
            {
                return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                ESP_LOGW("MODBUS", "Invalid Static IP Enable value: %d. Must be 0 (Disable) or 1 (Enable).", in_buf[i]);
            }
            i++; break;
            case ADD_SRVR_TIMEOUT:
            if(in_buf[i] > 0 && in_buf[i] <= 65535) {
                temp_ip_config.server_timeout = in_buf[i];
                ip_config[0].server_timeout =  temp_ip_config.server_timeout;
                ip_update_needed=1;
            }
            else
            {
                return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
                ESP_LOGW("MODBUS", "Invalid Server Timeout: %d. Must be between 1 and 65535 ms.", in_buf[i]);
            }
            i++; break;
            case ADD_PASSTHR_RETRY_CNT:
                if (in_buf[i] <= 255) {
                    temp_ip_config.Passthr_retry_cnt = in_buf[i];
                    ip_config[0].Passthr_retry_cnt   = temp_ip_config.Passthr_retry_cnt;
                    ip_update_needed = 1;
                    ESP_LOGI("MODBUS", "ADD_PASSTHR_RETRY_CNT received: %d",
                             temp_ip_config.Passthr_retry_cnt);
                } else {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
                    ESP_LOGW("MODBUS", "Invalid pass-through retry count: %d. Must be 0..255.",
                             in_buf[i]);
                }
                i++; break;
            case ADD_OTA_HTTP_URL: //HTTP_IP Address
            len = sizeof(httpserver_params.url);
            if ((qty - i) < len/2) {
                ESP_LOGW("MODBUS", "Illegal Address Read: Addr %d, Qty %d", addr, qty);
                return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
            }
            memset_j(httpserver_params.url, sizeof(httpserver_params.url), 0, sizeof(httpserver_params.url));
            for (int j = 0; j < qty; j++) {
                // Unpack two chars from one 16-bit register
                uint8_t char1 = (in_buf[i] >> 8) & 0xFF;
                uint8_t char2 = in_buf[i] & 0xFF;
                httpserver_params.url[j * 2] = char1;
                httpserver_params.url[(j * 2) + 1] = char2;
                i++;
            }
                ESP_LOGI("MODBUS","ADD_OTA_HTTP_URL recived:%s\n",httpserver_params.url);
                http_update_needed=1;
                break;
            case ADD_OTA_HTTP_PORT: 
                if(in_buf[i] <= 65535) {
                    httpserver_params.port = in_buf[i];
                    http_update_needed=1;
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid HTTP port: %d. Must be between 1 and 65535.", in_buf[i]);
                }
                i++; break;
            case ADD_OTA_HTTP_FILE_NAME: //HTTP File name
                len = sizeof(httpserver_params.filepath);
                        if ((qty - i) < len/2) {
                    ESP_LOGW("MODBUS", "Illegal Address Read: Addr %d, Qty %d", addr, qty);
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                }
            
                memset_j(httpserver_params.filepath, sizeof(httpserver_params.filepath), 0, sizeof(httpserver_params.filepath));
                for (int j = 0; j < qty; j++) {
                    // Unpack two chars from one 16-bit register
                    uint8_t char1 = (in_buf[i] >> 8) & 0xFF;
                    uint8_t char2 = in_buf[i] & 0xFF;
                    httpserver_params.filepath[j * 2] = char1;
                    httpserver_params.filepath[(j * 2) + 1] = char2;
                    i++;
                }
                ESP_LOGI("MODBUS","ADD_OTA_HTTP_FILE_NAME recived:%s\n",httpserver_params.filepath);
                http_update_needed=1;
                break;
            case ADD_TRIGGER_FOTA:
                if(in_buf[i] == 0 || in_buf[i] == 1||in_buf[i] == 2) {
                    httpserver_params.is_ota = in_buf[i];  
                    http_update_needed=1;
                    ESP_LOGI("MODBUS","ADD_TRIGGER_FOTA recived:%d\n",httpserver_params.is_ota);
                }
                else
                {
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; 
                    ESP_LOGW("MODBUS", "Invalid FOTA Trigger value: %d. Must be 0 (No Trigger) or 1/2 (Trigger).", in_buf[i]);
                }
                
                i++;break;
            case ADD_FOTA_STS:
                 return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
                i++;break;
            case ADD_LOG_TRIGGER:
                if (in_buf[i] == 0 || in_buf[i] == 1) {
                    log_params.trigger = (uint8_t) in_buf[i];
                    ESP_LOGI("MODBUS", "log trigger = %d", log_params.trigger);
                } else {
                    ESP_LOGW("MODBUS", "Invalid log trigger value: %d (expect 0/1)", in_buf[i]);
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
                }
                i++; break;
            case ADD_LOG_DURATION:
                log_params.duration_sec = in_buf[i];
                ESP_LOGI("MODBUS", "log duration_sec = %u", log_params.duration_sec);
                i++; break;
            case ADD_LOG_STATUS:        /* read-only — externally observed lifecycle */
                return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
            case ADD_LOG_PATH: //log upload URL path prefix (packed 2 chars/reg)
                len = sizeof(log_params.path);
                if ((qty - i) < len / 2) {
                    ESP_LOGW("MODBUS", "Illegal write to LOG_PATH: qty=%d", qty);
                    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
                }
                for (int j = 0; j < len / 2; j++) {
                    log_params.path[(j * 2)]     = (in_buf[i] >> 8) & 0xFF;
                    log_params.path[(j * 2) + 1] =  in_buf[i]       & 0xFF;
                    i++;
                }
                log_params.path[len - 1] = '\0';  // belt-and-suspenders terminator
                ESP_LOGI("MODBUS", "log path = %s", log_params.path);
                break;
            default:
                return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    // 3. If we reach here, all data is valid. Now apply updates atomically.
    if (uart_update_needed) {
        //{"0":{"curt0":"0:9600:0:8:1:1:600:0:1:0:0:1"}}
        printf("Updating UART settings: ID=%d, Baud=%ld, Parity=%d, DataBits=%d, StopBits=%d, RetryCnt=%d, RetryPrd=%d, DisconnectonNoA=%d, DefConnStatus=%d, InterwriteDelay=%d, InterReadDelay=%d, AutoConnect=%d\n",
               temp_urts.urt_id, temp_urts.baud,
               temp_urts.parity, temp_urts.data_bits,
               temp_urts.stop_bits, temp_urts.retry_cnt,
               temp_urts.retry_period,temp_urts.DisconnectonNoA,
               temp_urts.DefConnStatus,temp_urts.InterwriteDelay,
               temp_urts.InterReadDelay,temp_urts.AutoConnect);
           char urtsetting[100];
            sprintf_j(urtsetting, sizeof(urtsetting), "{\"0\":{\"curt0\":\"%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\"}}",
                                    temp_urts.urt_id, temp_urts.baud,
                                    temp_urts.parity, temp_urts.data_bits,
                                    temp_urts.stop_bits, temp_urts.retry_cnt,
                                    temp_urts.retry_period,temp_urts.DisconnectonNoA,
                                    temp_urts.DefConnStatus,temp_urts.InterwriteDelay,
                                    temp_urts.InterReadDelay,temp_urts.AutoConnect);
            printf("Found %s, saving content: %s\n", "urt", urtsetting);
            //Update the file using the dynamic key name (dmbr1, dmbr2, etc)
            update_config_tag(config_path, "urt", urtsetting);

    }
     if (ip_update_needed) {
        // Named fields — see BUG-FIX #18 in configuration.c: the old
        // positional "ipc0" colon string silently mis-parsed on a
        // missing/reordered field.
            char ipsetting[200];
            sprintf_j(ipsetting, sizeof(ipsetting),
                                    "{\"0\":{\"static_ip\":\"" IPSTR "\",\"gw\":\"" IPSTR "\",\"nm\":\"" IPSTR "\","
                                    "\"mb_port\":%d,\"pt_port\":%d,\"http_port\":%d,\"timeout\":%d,\"pt_retry\":%d,\"static_en\":%d}}",
                                    IP2STR(&temp_ip_config.st_ip_info.ip),
                                    IP2STR(&temp_ip_config.st_ip_info.gw),
                                    IP2STR(&temp_ip_config.st_ip_info.netmask),
                                    temp_ip_config.port,
                                    temp_ip_config.pass_through_port,
                                    temp_ip_config.http_port,
                                    temp_ip_config.server_timeout,
                                    temp_ip_config.Passthr_retry_cnt,
                                    temp_ip_config.St_ip_En_dis);

            printf("Found %s, saving content: %s\n", "ipc", ipsetting);
            //Update the file using the dynamic key name (dmbr1, dmbr2, etc)
            update_config_tag(config_path, "ipc", ipsetting); 
        }
        if (http_update_needed) {
            //{\"0\":{\"ota0\":\"10.63.107.102:8000:Version2.bin:1\"}}
            char httpsetting[200];
            sprintf_j(httpsetting, sizeof(httpsetting), "{\"0\":{\"ota0\":\"%s:%d:%s:%d\"}}",
                                    httpserver_params.url,
                                    httpserver_params.port,
                                    httpserver_params.filepath,
                                    httpserver_params.is_ota);

                printf("Found %s, saving content: %s\n", "ota", httpsetting);
                //Update the file using the dynamic key name (dmbr1, dmbr2, etc)
                update_config_tag(config_path, "ota", httpsetting); 

        }
    ESP_LOGI("MODBUS", "Slave %d: Successful read from Addr %d", slave_id, addr);
    return NMBS_ERROR_NONE;

}

void modbus_mutex_init(void) {
    if (xModbus_Mutex == NULL) {
        xModbus_Mutex = xSemaphoreCreateMutex();
    }
}


void modbus_app_init(void)
{
    modbus_mutex_init();
	uart1_init();
    modbus_master_init();
    xTaskCreate(modbus_read_task, "modbus_task", 8192, NULL, 5, NULL);
}
