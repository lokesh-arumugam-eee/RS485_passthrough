#include "main.h"
#include "esp_rom_sys.h"
#include <stdio.h>   // For vsnprintf
#include <stdarg.h>
#include <stdint.h>
#include <errno.h>
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "xmodem.h"
#include "controller.h"
#include "wifiConnect.h"
#include "ble_spp_server_demo.h"
#include "ota.h"
#include "configuration/configuration.h"
const char *XMODEM = "MODEM";
const char *TAG = "LIR_MAN";
const char *tmp_file_path = "/spiffs/tmpfile.txt";
const char *configpath = "/spiffs/config.txt";
uint16_t g_u16_expctd_blknm;
uint16_t g_u16_Recieved_blknm;
uint16_t g_u16_nxt_blknm;
uint8_t g_u8_Xmdm_Rx_FSM_STATE;
uint8_t g_u8_Xmdm_Tx_FSM_STATE;
uint8_t g_u8_FileSystemErr=0;
FILE *g_file = NULL;
FILE *g_file1 = NULL;
uint8_t g_u8_file_buffer[FILE_BUFFER_SIZE];
uint32_t g_u32_file_buffIndx;
uint8_t reboot_after_10Sec;
 
void change_log_tag(const char *new_tag) {
    XMODEM = new_tag;
}
#ifndef Custom_printf

// Typical BLE buffer limit (Check your specific BLE stack/MTU)
#define BLE_MAX_CHUNK 250 
void Custom_printf(const char *format, ...) {
  static char buf[MAX_LOG_LENGTH];
  va_list args;

  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len <= 0) return;
  if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;

  printf("%s", buf);

  int sent = 0;
  while (sent < len) {
    int chunk = len - sent;
    if (chunk > BLE_MAX_CHUNK) chunk = BLE_MAX_CHUNK;
    ble_spp_SendData(chunk, (uint8_t *)(buf + sent));
    sent += chunk;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
void acm_log(const char *format, ...) {
  static char buf[MAX_LOG_LENGTH];
  va_list args;

  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len <= 0) return;
  if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;

  if (log_endis == 1) printf("%s", buf);

  if (g_u8_acceptFileRx == 0 && g_u8_SendFileXMDMTx == 0 &&
      g_u8_acceptfwRx == 0 && g_u8_addtlDisblFlgfrlg == 0) {
    int sent = 0;
    while (sent < len) {
      int chunk = len - sent;
      if (chunk > BLE_MAX_CHUNK) chunk = BLE_MAX_CHUNK;
      ble_spp_SendData(chunk, (uint8_t *)(buf + sent));
      sent += chunk;
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}
#endif
esp_err_t delete_file_example(void) {

  esp_err_t errno;
  errno = unlink(config_path);
  if (errno == 0) {
    ESP_LOGI(TAG, "Deleted file: %s", config_path);
  } else {
    ESP_LOGE(TAG, "Failed to delete file: %s (errno=%d)", config_path, errno);
  }
  return errno;
}



int8_t Jio_ESP_010101_Chk4Xmdm_pkt(uint8_t* p_u8_rx_data)
{

	const xmodem_packet_t *pkt = (const xmodem_packet_t *)p_u8_rx_data;

	// validate header
    if (pkt->header != JIO_ESP_X_SOH)
	{
		acm_log("xm: bad SOH\n");
		return JIO_ESP_FAILURE;
	}

   /* g_u16_Recieved_blknm = pkt->block_num_neg;
    g_u16_Recieved_blknm <<=8;
    g_u16_Recieved_blknm|= pkt->block_num;*/

	if(pkt->block_num == 1)
	{
		g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
	}
    // checksum
    uint8_t sum = 0;
    for (int i = 0; i < JIO_ESP_X_BLOCK_SIZE; i++)
	{
		sum += pkt->data[i];
	}
    if (sum != pkt->checksum)
	{
		acm_log("xm: bad cksum\n");
		return JIO_ESP_FAILURE;
	}
	return JIO_ESP_SUCCESS;
}
long get_file_size_1(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return st.st_size;   // size in bytes
    } else {
        perror("stat failed");
        return -1;
    }
}
esp_err_t spiffs_repair_if_needed(void) {
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE("XMODEM", "esp_spiffs_info failed: %s", esp_err_to_name(ret));
        return ret;
    }

    size_t free_bytes = total - used;

    // Example condition: if free > 10 KB but writes are still failing
    if (free_bytes > FILE_COMMIT_SIZE) {
        ESP_LOGI("XMODEM", "Possible fragmentation detected, running check...");
        ret = esp_spiffs_check(NULL);
        if (ret != ESP_OK) {
            ESP_LOGE("XMODEM", "SPIFFS check failed (%s), reformatting...", esp_err_to_name(ret));
			//()Set a flag to put in a health packet to indicate reformatting is required
            #if 0
			ret = esp_spiffs_format(NULL);
            if (ret == ESP_OK) {
                ESP_LOGI("XMODEM", "SPIFFS formatted successfully.");
            } else {
                ESP_LOGE("XMODEM", "SPIFFS format failed: %s", esp_err_to_name(ret));
            }
			
            
			#endif
			return ret;
			
        } else {
            ESP_LOGI("XMODEM", "SPIFFS check OK, filesystem is consistent.");
            ESP_LOGI("XMODEM", "free_bytes - %u > FILE_COMMIT_SIZE - %u",free_bytes,FILE_COMMIT_SIZE);
        }
    }
    else
	{
		ESP_LOGE("XMODEM", "free_bytes - %u < FILE_COMMIT_SIZE - %u",free_bytes,FILE_COMMIT_SIZE);
	}

    return ESP_OK;
}
const xmodem_packet_t *pkt = NULL;//(const xmodem_packet_t *)p_u8_rx_data;
int8_t Jio_ESP_010102_LIRFile_Rx_FSM(uint8_t* p_u8_rx_data)
{
	//()const xmodem_packet_t 

	pkt = (const xmodem_packet_t *)p_u8_rx_data;
	ESP_LOGI(XMODEM, "g_u8_Xmdm_Rx_FSM_STATE  = %02d\n", g_u8_Xmdm_Rx_FSM_STATE);
	if(g_u8_Xmdm_Rx_FSM_STATE == JIO_ESP_XMDM_RX_FILE_COMPLETE)
	{
		if(pkt->header == JIO_ESP_X_EOT)
		{
			ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
			g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_RX_FILE_COMPLETE;
			ESP_LOGI(XMODEM, "pkt->header  = %02X\n", pkt->header);
			ESP_LOGI(XMODEM, "EOT is Received\n");
		}
		else
		{
			ESP_LOGI(XMODEM, "pkt->header  = %02X\n", pkt->header);
			ESP_LOGI(XMODEM, "EOT is not received\n");
			g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_RX_FILE_COMPLETE;
			return JIO_ESP_FAILURE;
		}
		
	}
	else
	{
		if(pkt->header == JIO_ESP_X_CAN)
		{
			g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
			ESP_LOGI(XMODEM, "pkt->header  = %02X\n", pkt->header);
			ESP_LOGI(XMODEM, "Received Cancel Packet\n");
			ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
		}
		else
		{
			#if 0
			// validate header
			if(g_u8_Xmdm_Rx_FSM_STATE == JIO_ESP_XMDM_FSM_INIT)
			{
				if (pkt->header == JIO_ESP_X_NAK) 
				{
					l_u8_ack_nak = JIO_ESP_X_NAK;
					//()Send NAK packet to BLE App
					//()ble_spp_SendData(strlen(printf_buffer), (uint8_t *)printf_buffer);
					ble_spp_SendData(1, &l_u8_ack_nak);
				}
				else
				{
				}
			}
			else
			#endif
			{


				if (pkt->header != JIO_ESP_X_SOH)
				{
					acm_log("xm: bad SOH\n");
					return JIO_ESP_FAILURE;
				}

				if(pkt->block_num == 1 && g_u16_expctd_blknm==2)
				{
					g_u16_expctd_blknm =1;
					g_u32_file_buffIndx = 0;
					memset_j(&g_u8_file_buffer[0], FILE_BUFFER_SIZE, 0, FILE_BUFFER_SIZE);
					acm_log("xm: adj blk 2->1\n");
				}
				if(pkt->block_num != g_u16_expctd_blknm)
				{
					acm_log("xm: blk exp=%d got=%d\n", g_u16_expctd_blknm, pkt->block_num);
					return JIO_ESP_FAILURE;
				}
				// checksum
				uint8_t sum = 0;
				ESP_LOGI(XMODEM, "Received File chunk: \n");
				for (int i = 0; i < JIO_ESP_X_BLOCK_SIZE; i++)
				{
					sum += pkt->data[i];
					//printf("%02X ",pkt->data[i]);
				}



				ESP_LOGI(XMODEM, "\ncalculated checksum = %02X\n", sum);
				ESP_LOGI(XMODEM, "Received checksum = %02X\n", pkt->checksum);
				if (sum != pkt->checksum) 

				{
					ESP_LOGI(XMODEM,"Received checksum is not matching with its calculated checksum, hence received packet is not compliance");
					return JIO_ESP_FAILURE;
				}
			}
		}
	}
	
	switch(g_u8_Xmdm_Rx_FSM_STATE)
	{
		case JIO_ESP_XMDM_FSM_INIT:
			//()Create a file LIR_CMD.txt
			//(()get file handler ready to use

			if(g_u8_acceptfwRx==1)
			{
				print_ota_status();
				g_u32_file_buffIndx = 0;
            	memset_j(&g_u8_file_buffer[0], FILE_BUFFER_SIZE, 0, FILE_BUFFER_SIZE);
				memcpy_j(&g_u8_file_buffer[g_u32_file_buffIndx], FILE_BUFFER_SIZE - g_u32_file_buffIndx, &pkt->data[0], JIO_ESP_X_BLOCK_SIZE);
				g_u32_file_buffIndx += JIO_ESP_X_BLOCK_SIZE;
				
				g_u16_expctd_blknm += 1;
				ESP_LOGI("FWUP", "Total number of bytes buffered so far is %ld\n",g_u32_file_buffIndx);
				g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_COMMIT_FILE;
			    ESP_LOGI("FWUP", "g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_COMMIT_FILE\n");
			}
			else
			{
				if(remove(tmp_file_path) == JIO_ESP_SUCCESS)
				{
					//ACM_LOG("File %s is successfully removed\n",tmp_file_path);
					ESP_LOGI("XMODEM", "Temporary file  = %s is successfully removed\n", tmp_file_path);
				}
				else
				{
					//ACM_LOG("File %s is Failed to be removed\n",tmp_file_path);
					ESP_LOGE("XMODEM", "Temporary file  = %s is Failed to be removed\n", tmp_file_path);
				}
				
				g_file = fopen(tmp_file_path, "w");  // Adjust path for actual file system
				if (g_file == NULL) 
				{
						ESP_LOGE("XMODEM", "Failed to open file for writing");
						ESP_LOGE("XMODEM", "Failed to open temp_file.txt for writing: %s\n", strerror(errno));
						if(spiffs_repair_if_needed() != ESP_OK)
						{
							ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned failure");
						}
						else
						{
							ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned Success");
							return JIO_ESP_FAILURE;
						}
						// Retry once
						g_file = fopen("/spiffs/temp_file.txt", "w");
						if (!g_file) {
							ESP_LOGE("XMODEM", "STILL cannot open temp file: %s\n", strerror(errno));
							g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
							//Set Flag for health packet
							g_u8_FileSystemErr=1;
							break;
						}
						else
						{
							//Clear Flag
							g_u8_FileSystemErr=0;
							ESP_LOGI("XMODEM", "temp File is created successfully: %s\n", strerror(errno));
						}
						
				}
				else
				{
					g_u8_FileSystemErr=0;
					ESP_LOGI("XMODEM", "File %s is successfully created\n",tmp_file_path);
					fsync(fileno(g_file));
					fclose(g_file);
				}
				g_u32_file_buffIndx = 0;
				memset_j(&g_u8_file_buffer[0], FILE_BUFFER_SIZE, 0, FILE_BUFFER_SIZE);
				memcpy_j(&g_u8_file_buffer[g_u32_file_buffIndx], FILE_BUFFER_SIZE - g_u32_file_buffIndx, &pkt->data[0], JIO_ESP_X_BLOCK_SIZE);
				g_u32_file_buffIndx += JIO_ESP_X_BLOCK_SIZE;
				
				g_u16_expctd_blknm += 1;
				ESP_LOGI("XMODEM", "Total number of bytes buffered so far is %ld\n",g_u32_file_buffIndx);
				g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_COMMIT_FILE;
				ESP_LOGI("XMODEM", "g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_COMMIT_FILE\n");
			}
			break;
			
		case JIO_ESP_XMDM_FSM_COMMIT_FILE:

			uint16_t l_u16_bytcnt = 0;
			uint16_t l_u16_1A_cnt = 0;
			while(l_u16_bytcnt < JIO_ESP_X_BLOCK_SIZE)
			{
				if(pkt->data[l_u16_bytcnt] == 0x1A)
				{
					l_u16_1A_cnt += 1;
				}
				else
				{
					l_u16_1A_cnt = 0;
				}
				l_u16_bytcnt += 1;
			}
			if(l_u16_1A_cnt==1)
			   l_u16_1A_cnt=0;
			if(l_u16_1A_cnt == 0)
			{
				memcpy_j(&g_u8_file_buffer[g_u32_file_buffIndx], FILE_BUFFER_SIZE - g_u32_file_buffIndx, &pkt->data[0], JIO_ESP_X_BLOCK_SIZE);
				g_u32_file_buffIndx += JIO_ESP_X_BLOCK_SIZE;
				if(g_u32_file_buffIndx >= FILE_COMMIT_SIZE)
					{
						if(g_u8_acceptfwRx==1)
						{
							if(ota_write_chunk(&g_u8_file_buffer[0], g_u32_file_buffIndx)==ESP_OK)
							{
								ESP_LOGE("FWUP", "Succesfully Written %ld",g_u32_file_buffIndx);
							}
							else
							{
								ESP_LOGE("FWUP", "Failed to Write %ld",g_u32_file_buffIndx);
								g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
							}
						}
						else
						{
							g_file = fopen(tmp_file_path, "a+");  // Adjust path for actual file system
							if (g_file == NULL) 
							{
								ESP_LOGE("XMODEM", "Failed to open file for writing");
								ESP_LOGE("XMODEM", "Failed to open temp_file.txt for writing: %s\n", strerror(errno));
								if(spiffs_repair_if_needed() != ESP_OK)
								{
									ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned failure");
								}
								else
								{
									ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned Success");
								}
								fflush(g_file);
								fsync(fileno(g_file));
								fclose(g_file);
								g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
								break;				
							}
							ESP_LOGI("XMODEM", "Buff index %ld has filled up to >= FILE_COMMIT_SIZE %d \n",g_u32_file_buffIndx,FILE_COMMIT_SIZE);
							
							//--------------------------------------------------------------------------
							size_t written = fwrite(&g_u8_file_buffer[0], 1, g_u32_file_buffIndx, g_file);
							if (written == g_u32_file_buffIndx) {
								g_u8_FileSystemErr=0;
							ESP_LOGI("XMODEM", "File chunk with size %d written successfully\n",written);
							//g_u16_expctd_blknm += 1;
							//Write Success
							//g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_RX_FILE_COMPLETE;
							} else {
								g_u8_FileSystemErr=2;
								ESP_LOGE("XMODEM", "File chunk with size %d Failed in writing\n",written);
								size_t err_info = errno;
								ESP_LOGE("XMODEM", "File chunk with block number Failed in writing : %s\n", strerror(errno));
								//g_u16_expctd_blknm -= 1;
								if((written == 0) && (ENOSPC == err_info))
								{
									if(spiffs_repair_if_needed() != ESP_OK)
									{
										ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned failure");
									}
									else
									{
										ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned Success");
									}
									fflush(g_file);
									fsync(fileno(g_file));
									fclose(g_file);
									g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
									break;
								}
								
							}
							fflush(g_file);
							fsync(fileno(g_file));
							esp_rom_delay_us(2000000);
							fclose(g_file);
							ESP_LOGI("XMODEM", "Total number of bytes buffered so far is %ld\n",g_u32_file_buffIndx);
						}
						g_u32_file_buffIndx = 0;
						memset_j(&g_u8_file_buffer[0], FILE_BUFFER_SIZE, 0, FILE_BUFFER_SIZE);
					}
					g_u16_expctd_blknm += 1;
			}
			else
			{
				memcpy_j(&g_u8_file_buffer[g_u32_file_buffIndx], FILE_BUFFER_SIZE - g_u32_file_buffIndx, &pkt->data[0], (JIO_ESP_X_BLOCK_SIZE - l_u16_1A_cnt));
				g_u32_file_buffIndx += JIO_ESP_X_BLOCK_SIZE - l_u16_1A_cnt;
				if(g_u8_acceptfwRx==1)
				{
					if(ota_write_chunk(&g_u8_file_buffer[0], g_u32_file_buffIndx)==ESP_OK)
					{
						ESP_LOGE("FWUP", "Succesfully Written %ld",g_u32_file_buffIndx);
						g_u16_expctd_blknm += 1;
						
						g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_RX_FILE_COMPLETE;
						//Clear Flag
					}
					else
					{
						ESP_LOGE("FWUP", "Failed to Write %ld",g_u32_file_buffIndx);
						g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
					}
				}
				else
				{
					spiffs_repair_if_needed();
					g_file = fopen(tmp_file_path, "a+");  // Adjust path for actual file system
					if (g_file == NULL) 
					{
						ESP_LOGE("XMODEM", "Failed to open file for writing");
						ESP_LOGE("XMODEM", "Failed to open temp_file.txt for writing: %s\n", strerror(errno));
						if(spiffs_repair_if_needed() != ESP_OK)
						{
							ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned failure");
						}
						else
						{
							ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned Success");
						}
						fflush(g_file);
						fsync(fileno(g_file));
						fclose(g_file);
						g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
						break;
					}
				
					ESP_LOGI("XMODEM", "Total number of bytes buffered so far is %ld\n",g_u32_file_buffIndx);
					size_t written = fwrite(&g_u8_file_buffer[0], 1, g_u32_file_buffIndx, g_file);
					if (written == g_u32_file_buffIndx) 
					{	
						ESP_LOGI("XMODEM", "File chunk with size %d written successfully\n",written);
						g_u16_expctd_blknm += 1;
						
						g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_RX_FILE_COMPLETE;
						//Clear Flag
						g_u8_FileSystemErr=0;
					} else 
					{
						//Set the Flag for not
						g_u8_FileSystemErr=2; 
						ESP_LOGE("XMODEM", "File chunk with size %d Failed in writing\n",written);
						size_t err_info = errno;
						ESP_LOGE("XMODEM", "File chunk with block number Failed in writing : %s\n", strerror(errno));
						//g_u16_expctd_blknm -= 1;
						fflush(g_file);
						fsync(fileno(g_file));
						esp_rom_delay_us(2000000);
						if((written == 0) && (ENOSPC == err_info))
						{
							if(spiffs_repair_if_needed() != ESP_OK)
							{
								ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned failure");
							}
							else
							{
								ESP_LOGE("XMODEM", "spiffs_repair_if_needed() returned Success");
							}
							fflush(g_file);
							fsync(fileno(g_file));
							fclose(g_file);
							g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
							break;
						}
					}			
				}
			}
			break;
			
		case JIO_ESP_XMDM_RX_FILE_COMPLETE:
			//()delete the file "LIR_CMD.txt" .
			//()Rename current file name from "tmpfile.txt" to "LIR_CMD.txt"
			if(g_u8_acceptfwRx==1)
			{
				if(ota_end_and_set_boot()==ESP_OK)
				{
					ESP_LOGI("FWUP", "OTAEND Sucsessful");
				}
				else
				{
					ESP_LOGI("FWUP", "OTAEND Failed");
				}
				g_u8_acceptfwRx=0;
				 ESP_LOGI("FWUP", "Rebooting.......");
				 esp_rom_delay_us(2000000);
                 esp_restart(); 
			}
			else
			{
				fflush(g_file);
				if (fclose(g_file) == 0) 
				{
					ESP_LOGI("FILE", "File closed successfully");
				} else 
				{
					ESP_LOGE("FILE", "fclose failed: %s", strerror(errno));
				}
				fsync(fileno(g_file));
				esp_rom_delay_us(2000000);
				long int l_sz_file_size = get_file_size_1(tmp_file_path);
				
				ESP_LOGI("XMODEM", "File size: %ld bytes\n", l_sz_file_size);
				if(remove(configpath) == JIO_ESP_SUCCESS)
				{
					ESP_LOGI("XMODEM", "File %s is successfully removed\n",configpath);
				}
				else
				{
					ESP_LOGE("XMODEM", "File %s is Failed to be removed\n",configpath);
				}
				//fsync(fileno(g_file));
				if(rename(tmp_file_path,configpath) == JIO_ESP_SUCCESS)
				{
					ESP_LOGI("XMODEM", "File %s is successfully renamed to file name %s\n",tmp_file_path, configpath);
					ESP_LOGI("XMODEM", "Rebooting....");
					esp_rom_delay_us(2000000);
					//esp_restart();
					//reboot_after_10Sec =60;
					g_file1 = fopen(tmp_file_path, "r");
					if(g_file1==NULL)
					{
						ESP_LOGE("XMODEM", "Temp File got renamed");
						
					}
					else {
						ESP_LOGI("XMODEM", "Temp File did not renamed");
						fclose(g_file1);
					}
				}
				else
				{
					ESP_LOGI("XMODEM", "File %s is failed to be renamed to file name %s\n",tmp_file_path, configpath);
				}
				l_sz_file_size = get_file_size_1(configpath);
				
				ESP_LOGI("XMODEM", "Size of File %s : %ld bytes\n", configpath,l_sz_file_size);
				g_u8_acceptFileRx = 0;
			}
			g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
			break;
			
		case JIO_ESP_XMDM_STATE_MAX:
			//()delete the file "tmpfile.txt"
			if(g_u8_acceptfwRx==1)
			{
				esp_ota_abort(ota_handle);   // Discard failed OTA
				ota_handle = 0;
				ota_write_offset = 0;
				g_u8_acceptfwRx=0;
				ESP_LOGI("FWUP", "Firmware flag cleared and abhorted");
			}
			else
			{
				fflush(g_file);
				fsync(fileno(g_file));
				fclose(g_file);
				if(remove(tmp_file_path) == JIO_ESP_SUCCESS)
				{
					//ACM_LOG("File %s is successfully removed\n",tmp_file_path);
					ESP_LOGI("XMODEM", "Temporary file  = %s is successfully removed\n", tmp_file_path);
					
				}
				else
				{
					//ACM_LOG("File %s is Failed to be removed\n",tmp_file_path);
					ESP_LOGI("XMODEM", "Temporary file  = %s is Failed to be removed\n", tmp_file_path);
				}
				g_u8_acceptFileRx = 0;
			}
			g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
			break;
			
		default:
			g_u8_Xmdm_Rx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
			break;
	}
	if(g_u8_Xmdm_Rx_FSM_STATE < JIO_ESP_XMDM_STATE_MAX)
	{
		ESP_LOGI("XMODEM", "Return success from Rx FSM\n");
		return JIO_ESP_SUCCESS;
	}
	else
	{
		#ifdef UNIT_CHUNK_COMMIT
		forcible_err:
		#endif
		return JIO_ESP_FAILURE;
	}
}

uint16_t Jio_ESP_010105_xmodem_build_packet(uint8_t *data, xmodem_packet_t *packet) {
    uint8_t checksum = 0;

	packet->header = JIO_ESP_X_SOH;
    packet->block_num = g_u16_nxt_blknm;
   // packet->block_num_neg = 255 - g_u8_nxt_blknm;
    memcpy_j(packet->data, sizeof(packet->data), data, JIO_ESP_X_BLOCK_SIZE);

    // checksum
    checksum = 0;
    for (int i = 0; i < JIO_ESP_X_BLOCK_SIZE; i++) 
	{
		checksum += packet->data[i];
	}
    packet->checksum = checksum;

    return (JIO_ESP_X_BLOCK_SIZE + 4); // total packet length
}
uint8_t l_u8_buffer[JIO_ESP_X_BLOCK_SIZE];
uint8_t l_u8_packet[JIO_ESP_X_BLOCK_SIZE + 5];
void Jio_ESP_010104_LIRFile_Tx_FSM(uint8_t *p_u8_rx_data)
{
	
    int16_t l_i16_bytes_read;

	uint8_t l_u8_Cancel;
	xmodem_packet_t *g_tx_fsm_pkt = (xmodem_packet_t *)p_u8_rx_data;
	
	if(g_tx_fsm_pkt->header == JIO_ESP_X_CAN)
	{
		g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_STATE_MAX;
		ESP_LOGI("XMODEMTx", "JIO_ESP_X_CAN is Received, going for graceful bail out\n");
	}
	switch(g_u8_Xmdm_Tx_FSM_STATE)
	{
		case JIO_ESP_XMDM_FSM_INIT:
			ESP_LOGI("XMODEMTx", "Current state is JIO_ESP_XMDM_FSM_READ_SEND_FILE\n");
			if(g_tx_fsm_pkt->header != JIO_ESP_X_NAK)
			{
				ESP_LOGE("XMODEMTx", "Initial NAK is not received, hence bailing out with out any File Transfer\n");
				l_u8_Cancel = JIO_ESP_X_CAN;
				uint8_t l_u8_ble_sndreslt;
				l_u8_ble_sndreslt = ble_spp_SendData(1, &l_u8_Cancel);
				if(l_u8_ble_sndreslt == 0)
				{
				  ESP_LOGE("XMODEMTx", "Failure in sending Packet with CANCEL to BLE App");
				}
				else
				{
				  ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with CANCEL to BLE App");
				  g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
					ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_FSM_INIT\n");
					g_u8_SendFileXMDMTx = 0;
					g_u16_nxt_blknm = 1;
					ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
				}
			}
			else
			{
				ESP_LOGI("XMODEMTx", "Initial NAK is received, hence initiating File Transfer\n");
				g_file = fopen(configpath, "r+");
				if (g_file == NULL)
				{
					ESP_LOGI("XMODEMTx", "File %s doesnt exist to send to BLE App\n",configpath);
					l_u8_Cancel = JIO_ESP_X_CAN;
					uint8_t l_u8_ble_sndreslt;
					l_u8_ble_sndreslt = ble_spp_SendData(1, &l_u8_Cancel);
					if(l_u8_ble_sndreslt == 0)
					{
					  ESP_LOGE("XMODEMTx", "Failure in sending Packet with CANCEL to BLE App");
					}
					else
					{
					  ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with CANCEL to BLE App");
					}
					g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
					ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_FSM_INIT\n");
					g_u8_SendFileXMDMTx = 0;
					g_u16_nxt_blknm = 1;
					ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
				}
				else
				{
					memset_j(&l_u8_buffer[0], sizeof(l_u8_buffer), 0, sizeof(l_u8_buffer));
					
					#if 1
					l_i16_bytes_read = fread(l_u8_buffer, 1, JIO_ESP_X_BLOCK_SIZE, g_file);
					if(l_i16_bytes_read < JIO_ESP_X_BLOCK_SIZE)
					{
						//()What to do now????
						memset_j(l_u8_buffer + l_i16_bytes_read, (JIO_ESP_X_BLOCK_SIZE - l_i16_bytes_read), 0x1A, (JIO_ESP_X_BLOCK_SIZE - l_i16_bytes_read)); // pad with SUB
					}
					#endif
					
					ESP_LOGI("XMODEMTx", "Number of bytes read from File is %d\n",l_i16_bytes_read);
					ESP_LOGI("XMODEMTx", "Buffer contents to be sent to BLE\n");
					for(int i = 0; i < l_i16_bytes_read; i++)
					{
						printf("%02X  ", l_u8_buffer[i]);
					}
					memset_j(&l_u8_packet[0], sizeof(l_u8_packet), 0, sizeof(l_u8_packet));
					xmodem_packet_t *g_tx_fsm_pkt1 = (xmodem_packet_t *)l_u8_packet;
					uint16_t pkt_len = Jio_ESP_010105_xmodem_build_packet(l_u8_buffer, g_tx_fsm_pkt1);
					ESP_LOGI("XMODEMTx", "\nLength of packet to be sent to BLE is %d\n",pkt_len);
					
					memcpy_j(&l_u8_packet[0], sizeof(l_u8_packet), (uint8_t *)g_tx_fsm_pkt1, pkt_len);
					ESP_LOGI("XMODEMTx", "Packet contents to be sent to BLE\n");
					for(uint16_t i = 0; i < pkt_len; i++)
					{
						printf("%02X  ", l_u8_packet[i]);
					}
					printf("\n");

					
					uint8_t l_u8_ble_sndreslt;
					
					l_u8_ble_sndreslt = ble_spp_SendData(pkt_len, l_u8_packet);
					if(l_u8_ble_sndreslt == 0)
					{
					  ESP_LOGE("XMODEMTx", "Failure in sending Packet with Block number %d to BLE App",g_u16_nxt_blknm);
					  
					}
					else
					{
					  ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with Block number %d to BLE App",g_u16_nxt_blknm);
					  g_u8_commtimeout = 0;
					  ESP_LOGI("XMODEMTx", "Starting single-shot timer for 20 seconds.");
					  ESP_ERROR_CHECK(esp_timer_start_once(cmtimeout_timer1, 20000000)); // Timer period in microseconds
					}
				}
				g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_READ_SEND_FILE;
				ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_FSM_READ_SEND_FILE\n");
			}
			break;
			
		case JIO_ESP_XMDM_FSM_READ_SEND_FILE:
			ESP_LOGI("XMODEMTx", "Current state is JIO_ESP_XMDM_FSM_READ_SEND_FILE\n");
			if(g_tx_fsm_pkt->header == JIO_ESP_X_NAK)
			{
				ESP_LOGI("XMODEMTx", "g_tx_fsm_pkt->header = JIO_ESP_X_NAK\n");
			}
			else if(g_tx_fsm_pkt->header == JIO_ESP_X_ACK)
			{
				ESP_LOGI("XMODEMTx", "g_tx_fsm_pkt->header = JIO_ESP_X_ACK\n");
				ESP_ERROR_CHECK(esp_timer_stop(cmtimeout_timer1));
				g_u16_nxt_blknm += 1;
				g_u8_commtimeout = 0;
			}

			{
				memset_j(&l_u8_buffer[0], sizeof(l_u8_buffer), 0, sizeof(l_u8_buffer));
				fseek(g_file,(g_u16_nxt_blknm - 1) * JIO_ESP_X_BLOCK_SIZE,SEEK_SET);
				l_i16_bytes_read = fread(l_u8_buffer, 1, JIO_ESP_X_BLOCK_SIZE, g_file);
				if(l_i16_bytes_read < JIO_ESP_X_BLOCK_SIZE)
				{
					//()What to do now????
					memset_j(l_u8_buffer + l_i16_bytes_read, (JIO_ESP_X_BLOCK_SIZE - l_i16_bytes_read), 0x1A, (JIO_ESP_X_BLOCK_SIZE - l_i16_bytes_read)); // pad with SUB
					g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_TX_FILE_COMPLETE;
					ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_TX_FILE_COMPLETE\n");
				}
				else
				{
					ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_FSM_READ_SEND_FILE\n");
				}
				ESP_LOGI("XMODEMTx", "Number of bytes read from File is %d\n",l_i16_bytes_read);
				ESP_LOGI("XMODEMTx", "Buffer contents to be sent to BLE\n");
				for(int i = 0; i < l_i16_bytes_read; i++)
				{
					printf("%02X  ", l_u8_buffer[i]);
				}
				memset_j(&l_u8_packet[0], sizeof(l_u8_packet), 0, sizeof(l_u8_packet));
				xmodem_packet_t *g_tx_fsm_pkt1 = (xmodem_packet_t *)l_u8_packet;
				uint16_t pkt_len = Jio_ESP_010105_xmodem_build_packet(l_u8_buffer, g_tx_fsm_pkt1);
				ESP_LOGI("XMODEMTx", "\nLength of packet to be sent to BLE is %d\n",pkt_len);
				
				memcpy_j(&l_u8_packet[0], sizeof(l_u8_packet), (uint8_t *)g_tx_fsm_pkt1, pkt_len);
				ESP_LOGI("XMODEMTx", "Packet contents to be sent to BLE\n");
				for(uint16_t i = 0; i < pkt_len; i++)
				{
					printf("%02X  ", l_u8_packet[i]);
				}
				printf("\n");
				

				uint8_t l_u8_ble_sndreslt;
				l_u8_ble_sndreslt = ble_spp_SendData(pkt_len, l_u8_packet);
				if(l_u8_ble_sndreslt == 0)
				{
				  ESP_LOGE("XMODEMTx", "Failure in sending Packet with Block number %d to BLE App",g_u16_nxt_blknm);
				}
				else
				{
				  ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with Block number %d to BLE App",g_u16_nxt_blknm);
				  ESP_LOGI("XMODEMTx", "Starting single-shot timer for 20 seconds.");
				  ESP_ERROR_CHECK(esp_timer_start_once(cmtimeout_timer1, 20000000)); // Timer period in microseconds
				}
				
			}
			break;
		
		case JIO_ESP_XMDM_TX_FILE_COMPLETE:
			ESP_LOGI("XMODEMTx", "Current state is JIO_ESP_XMDM_TX_FILE_COMPLETE\n");
			if(g_tx_fsm_pkt->header == JIO_ESP_X_NAK)
			{
				ESP_ERROR_CHECK(esp_timer_stop(cmtimeout_timer1));
				ESP_LOGI("XMODEMTx", "g_tx_fsm_pkt->header = JIO_ESP_X_NAK\n");

				{
					memset_j(&l_u8_buffer[0], sizeof(l_u8_buffer), 0, sizeof(l_u8_buffer));
					fseek(g_file,(g_u16_nxt_blknm - 1) * JIO_ESP_X_BLOCK_SIZE,SEEK_SET);
					l_i16_bytes_read = fread(l_u8_buffer, 1, JIO_ESP_X_BLOCK_SIZE, g_file);
					if(l_i16_bytes_read < JIO_ESP_X_BLOCK_SIZE)
					{
						//()What to do now????
						memset_j(l_u8_buffer + l_i16_bytes_read, (JIO_ESP_X_BLOCK_SIZE - l_i16_bytes_read), 0x1A, (JIO_ESP_X_BLOCK_SIZE - l_i16_bytes_read)); // pad with SUB
						g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_TX_FILE_COMPLETE;
					}
					ESP_LOGI("XMODEMTx", "Number of bytes read from File is %d\n",l_i16_bytes_read);
					ESP_LOGI("XMODEMTx", "Buffer contents to be sent to BLE\n");
					for(int i = 0; i < l_i16_bytes_read; i++)
					{
						printf("%02X  ", l_u8_buffer[i]);
					}
					memset_j(&l_u8_packet[0], sizeof(l_u8_packet), 0, sizeof(l_u8_packet));
					xmodem_packet_t *g_tx_fsm_pkt1 = (xmodem_packet_t *)l_u8_packet;
					uint16_t pkt_len = Jio_ESP_010105_xmodem_build_packet(l_u8_buffer, g_tx_fsm_pkt1);
					ESP_LOGI("XMODEMTx", "\nLength of packet to be sent to BLE is %d\n",pkt_len);
					
					memcpy_j(&l_u8_packet[0], sizeof(l_u8_packet), (uint8_t *)g_tx_fsm_pkt1, pkt_len);
					ESP_LOGI("XMODEMTx", "Packet contents to be sent to BLE\n");
					for(uint16_t i = 0; i < pkt_len; i++)
					{
						printf("%02X  ", l_u8_packet[i]);
					}
					printf("\n");

					uint8_t l_u8_ble_sndreslt;
					l_u8_ble_sndreslt = ble_spp_SendData(pkt_len, l_u8_packet);
					if(l_u8_ble_sndreslt == 0)
					{
					  ESP_LOGE("XMODEMTx", "Failure in sending Packet with Block number %d to BLE App",g_u16_nxt_blknm);
					}
					else
					{
					  ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with Block number %d to BLE App",g_u16_nxt_blknm);
					  ESP_LOGI("XMODEMTx", "Starting single-shot timer for 20 seconds.");
					  ESP_ERROR_CHECK(esp_timer_start_once(cmtimeout_timer1, 20000000)); // Timer period in microseconds
					}
				}
			}
			else if(g_tx_fsm_pkt->header == JIO_ESP_X_ACK)
			{
				g_u8_commtimeout = 0;
				ESP_ERROR_CHECK(esp_timer_stop(cmtimeout_timer1));
				ESP_LOGI("XMODEMTx", "g_tx_fsm_pkt->header = JIO_ESP_X_ACK\n");

				uint8_t l_u8_ble_sndreslt;
				uint8_t l_u8_eot_arr[10];
				memset_j(&l_u8_eot_arr[0], sizeof(l_u8_eot_arr), 0, sizeof(l_u8_eot_arr));
				//l_u8_ble_sndreslt = ble_spp_SendData(1, &l_u8_eot);
				for(int i=0;i<5;i++)
				{
					l_u8_ble_sndreslt = ble_spp_SendData(sizeof(l_u8_eot_arr), l_u8_eot_arr);
					if(l_u8_ble_sndreslt == 0)
					{
					ESP_LOGE("XMODEMTx", "Failure in sending Packet with EOT to BLE App");
					}
					else
					{
					ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with EOT to BLE App");
					}
				}
				fclose(g_file);
				g_file = NULL;
				g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
				ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_FSM_INIT\n");
				g_u8_SendFileXMDMTx = 0;
				g_u16_nxt_blknm = 1;
				ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
			}
			break;
			
		
		case JIO_ESP_XMDM_STATE_MAX:
			ESP_LOGI("XMODEMTx", "Current state is JIO_ESP_XMDM_STATE_MAX\n");
			l_u8_ack_nak = JIO_ESP_X_ACK;
			uint8_t l_u8_ble_sndreslt;
			uint8_t l_u8_ack_nak_arr[10];
			memset_j(&l_u8_ack_nak_arr[0], sizeof(l_u8_ack_nak_arr), 0, sizeof(l_u8_ack_nak_arr));
			//()l_u8_ble_sndreslt = ble_spp_SendData(1, &l_u8_ack_nak);
			l_u8_ble_sndreslt = ble_spp_SendData(sizeof(l_u8_ack_nak_arr),l_u8_ack_nak_arr);
			if(l_u8_ble_sndreslt == 0)
			{
			  ESP_LOGE("XMODEMTx", "Failure in sending Packet with ACK to BLE App");
			}
			else
			{
			  ESP_LOGI("XMODEMTx", "Succeeded in sending Packet with ACK to BLE App");
			}
			if(g_file != NULL)
			{
				fclose(g_file);
				g_file = NULL;
			}
			g_u8_Xmdm_Tx_FSM_STATE = JIO_ESP_XMDM_FSM_INIT;
			ESP_LOGI("XMODEMTx", "Next state is JIO_ESP_XMDM_FSM_INIT\n");
			g_u8_SendFileXMDMTx = 0;
			g_u16_nxt_blknm = 1;
			ESP_ERROR_CHECK(esp_timer_stop(one_shot_timer1));
			break;
			
		default:
			break;
	}
	
}

