#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "main.h"
#include "esp_log.h"
#include "nanomodbus.h"
#include "modbus_rtu.h"
#include "modbus/data.h"
#include "configuration/configuration.h"
#include "uart/uart_app.h"
#include "driver/uart.h"
#include <errno.h>
#include <inttypes.h>
#include <sys/socket.h>
#include "modbus_tcp.h"
#include "xmodem.h"
#include "http_server.h"
#include "status_led.h"
static const char *TAG = "modbustcp";

#define PASSTHROUGH_PORT_DEFAULT  503
#define PASSTHROUGH_BUF_SIZE      260      // RTU max ADU = 256, plus a small margin
#define PASSTHROUGH_TCP_GAP_MS    50       // idle gap that marks end of TCP-side frame
#define PASSTHROUGH_RTU_GAP_MS    50       // idle gap that marks end of RTU response

extern SemaphoreHandle_t xModbus_Mutex;
static nmbs_t nmbs_tcp_server;
static nmbs_t nmbs_tcp_client;
static nmbs_platform_conf tcp_conf;
static nmbs_callbacks server_callbacks;

int connfd;
uint8_t tcp_slave_id = 0; // Initialize to a default slave ID

void modbus_Tcp_server_init(void);
int32_t my_tcp_read(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg);
int32_t my_tcp_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg);


nmbs_error on_tcp_read_Holding_request(uint16_t address, uint16_t quantity, uint16_t* registers_out, uint8_t unit_id, void* arg) {
    
    
    if(unit_id<1 || unit_id>SLAVE_ID){
        unit_id=1;
    }

    if (unit_id == SLAVE_ID) {
        // Route to your filename/IP/Baudrate logic
        printf("Read holding reg \n");
        return populate_modbus_read_holding(unit_id, address, quantity, registers_out);
    }
    else
    {
          printf("Read holding reg not 249\n");
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; // Don't allow reading holding registers from RS485 slave
    }
        //  else {
    //     // Bridge to RS485 Slave
    //     return safe_modbus_read(READ_HOLDING_REGISTERS, unit_id, address, quantity, registers_out);
    // }
}

nmbs_error on_tcp_read_input_registers(uint16_t address, uint16_t quantity, uint16_t* registers_out, uint8_t unit_id, void* arg) {
    
    if(unit_id<1 || unit_id>SLAVE_ID){
        unit_id=1;
    }
    
    if (unit_id == SLAVE_ID) {
        // Route to your filename/IP/Baudrate logic
        printf("Read input reg \n");
        return populate_modbus_read_input(unit_id, address, quantity, registers_out);
    } 
    else 
    {
        printf("Read input reg invalid input\n");
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; // Don't allow reading input registers from RS485 slave
    }    // else {
    //     // Bridge to RS485 Slave
    //     return safe_modbus_read(READ_INPUT_REGISTERS, unit_id, address, quantity, registers_out);
    // }
}

nmbs_error on_tcp_write_registers(uint16_t address, uint16_t quantity, const uint16_t* registers_in, uint8_t unit_id, void* arg) {

    if(unit_id<1 || unit_id>SLAVE_ID){
        unit_id=1;
    }

    if (unit_id == SLAVE_ID) {
        // Route to your filename/IP/Baudrate logic

         printf("Read write reg \n");
        return populate_modbus_write(unit_id, address, quantity, registers_in);
    }
    else {
         printf("Read write reg slave id not 249\n");
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; // Don't allow writing registers to RS485 slave
    }
    // else {
    //     // Bridge to RS485 Slave
    //     return safe_modbus_read(WRITE_MULTIPLE_REGISTERS, unit_id, address, quantity, registers_in);
    // }
}

/* nanomodbus's write_single_register callback takes a scalar value, not a
   registers array — it can't share on_tcp_write_registers's signature (that
   one previously got assigned here too, which the compiler rejects). Wrap
   the single value in a 1-element buffer and reuse the same write path. */
nmbs_error on_tcp_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void* arg) {
    uint16_t reg = value;

    if (unit_id < 1 || unit_id > SLAVE_ID) {
        unit_id = 1;
    }

    if (unit_id == SLAVE_ID) {
        printf("Write single reg \n");
        return populate_modbus_write(unit_id, address, 1, &reg);
    } else {
        printf("Write single reg slave id not 249\n");
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; // Don't allow writing registers to RS485 slave
    }
}

int32_t my_tcp_read(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    int fd = *(int*)arg;
    
    struct timeval tv = { .tv_sec = 0, .tv_usec = byte_timeout_ms * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ssize_t received = recv(fd, buf, count, 0); 
    
    if (received == 0) return -1; // Client disconnected
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    } 
    return (int32_t)received;
}

// The WRITE signature nanoMODBUS wants:
int32_t my_tcp_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    // 1. Get the socket from the argument (Don't use the global directly)
    int fd = *(int*)arg;
    if (fd < 0) return -1;

    // 2. Log the bytes (You should see the 2-byte CRC at the end)
    ESP_LOG_BUFFER_HEX("RTU_OVER_TCP_OUT", buf, count);

    // 3. Standard send
    ssize_t sent = send(fd, buf, count, 0);

    if (sent < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return (int32_t)sent;
}

void modbus_Tcp_server_init(void) {
    nmbs_platform_conf_create(&tcp_conf);
    tcp_conf.transport = NMBS_TRANSPORT_RTU; // Change to TCP
    tcp_conf.read = my_tcp_read;
    tcp_conf.write = my_tcp_write;
    tcp_conf.arg = &connfd; 

    memset(&server_callbacks, 0, sizeof(nmbs_callbacks));
    server_callbacks.read_holding_registers = on_tcp_read_Holding_request;
    server_callbacks.read_input_registers = on_tcp_read_input_registers;
    server_callbacks.write_single_register = on_tcp_write_single_register;
    server_callbacks.write_multiple_registers = on_tcp_write_registers;

    server_callbacks.initialized = 0xFFFFDEBE;

    // Use 0 to accept all Slave IDs (Gateway Mode)
    nmbs_error err = nmbs_server_create(&nmbs_tcp_server, 1, &tcp_conf, &server_callbacks);

    printf("Callback Pointer: %p\n", server_callbacks.read_holding_registers);
    if (err != NMBS_ERROR_NONE) {
        ESP_LOGE("TCP_SERVER", "Failed to create: %d", err);
    }
}

void modbus_gateway_task(void *pvParameters) {
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    int mb_port = (ip_config && ip_config[0].port > 0) ? ip_config[0].port : 502;
    struct sockaddr_in servaddr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(mb_port)
    };
	int timeout_in_ms = 1000 * ip_config[0].server_timeout;
    bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
    listen(listenfd, 1);

    while (1) {
        printf("Waiting for TCP Client on port %d...\n", mb_port);
        connfd = accept(listenfd, NULL, NULL);
        
        if (connfd >= 0) {
            printf("TCP Client Connected!\n");

            // --- INACTIVITY TIMEOUT CONFIGURATION ---
            // Set a 5-second timeout for receiving data
            struct timeval timeout;
            timeout.tv_sec = ip_config[0].server_timeout;  // 10 seconds
            timeout.tv_usec = 0;

            if (setsockopt(connfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
                printf("Failed to set timeout socket option\n");
            }
            // -----------------------------------------

            // Update the platform argument to the NEW connection file descriptor
            tcp_conf.arg = &connfd;

            while (1) {
                // nmbs_server_poll will now return an error if 5 seconds pass with no data
                nmbs_error err = nmbs_server_poll(&nmbs_tcp_server);
                
                if (err < 0) {
                    // If it's a timeout, the error code usually relates to transport/timeout
                    printf("TCP Inactivity (5s) or Error: %d. Closing connection.\n", err);
                    break; 
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            close(connfd); 
            printf("Socket closed, ready for next client.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}
nmbs_error modbus_multi_server_read(uint8_t slave_id, esp_ip4_addr_t poll_ip,int port, uint16_t address, int quantity, uint16_t* registers_out){

    int timeout_in_ms = 1000 * ip_config[0].server_timeout; 
    nmbs_error return_value = NMBS_ERROR_NONE;
	ESP_LOGI("CLIENT", "Step 1: Connecting to Server" IPSTR "\n",IP2STR(&poll_ip));
    
	// 1. Create Socket
	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (sock < 0) {
		vTaskDelay(pdMS_TO_TICKS(2000));
		return NMBS_ERROR_INVALID_RESPONSE; // Socket creation failed
	}
    

	// Set a connection timeout (e.g., 3 seconds) so we don't hang if a server is offline
	struct timeval timeout = { .tv_sec = ip_config[0].server_timeout, .tv_usec = 0 };
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

	struct sockaddr_in dest_addr;
	dest_addr.sin_addr.s_addr = poll_ip.addr;
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(port);

	// 2. Connect
	if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) == 0) {
		ESP_LOGI("CLIENT", "Step 1: Connected to " IPSTR "\n", IP2STR(&poll_ip));

		// 3. Initialize nanoMODBUS for this specific session
		nmbs_platform_conf platform_conf;
		nmbs_platform_conf_create(&platform_conf);
		platform_conf.transport = NMBS_TRANSPORT_RTU;//NMBS_TRANSPORT_TCP;
		platform_conf.read = my_tcp_read; 
		platform_conf.write = my_tcp_write;
		platform_conf.arg = (void *)&sock;

        printf("Creating client...\n");
        memset(&nmbs_tcp_client, 0, sizeof(nmbs_t));
		nmbs_client_create(&nmbs_tcp_client, &platform_conf);
		nmbs_set_read_timeout(&nmbs_tcp_client, timeout_in_ms); // Convert seconds to milliseconds
        
      //  if(tcp_slave_id !=slave_id){
            tcp_slave_id = slave_id;
            nmbs_set_destination_rtu_address(&nmbs_tcp_client, tcp_slave_id);
            printf("slave id :%d\n",tcp_slave_id);
	//	}
        
        printf("Address: %d,Reg Count :%d\n",address, quantity);
		// 4. Read Data
		nmbs_error err = nmbs_read_holding_registers(&nmbs_tcp_client,address, (uint16_t)quantity, registers_out);
		
		if (err == NMBS_ERROR_NONE) {
			ESP_LOGI("CLIENT", "Data read successfuly:" IPSTR "\n", IP2STR(&poll_ip));
		} else {
			ESP_LOGE("CLIENT", "Read Error:" IPSTR "\n", IP2STR(&poll_ip));
            return_value = NMBS_ERROR_INVALID_RESPONSE;
		}
	} else {
		ESP_LOGW("CLIENT", "Could not reach " IPSTR "\n", IP2STR(&poll_ip));
        return_value = NMBS_ERROR_TRANSPORT;
	}

	// 5. Disconnect immediately
	ESP_LOGI("CLIENT", "Step 5: Closing connection to " IPSTR "\n", IP2STR(&poll_ip));
	close(sock);
    return return_value;
}

/* -------------------------------------------------------
   Pass-through TCP <-> RTU bridge on port 503
   Forwards raw bytes between TCP and UART1 (RS485) without
   any Modbus parsing. Shares xModbus_Mutex with the RTU
   master so the bus stays serialized.
------------------------------------------------------- */
static int passthrough_read_tcp_frame(int fd, uint8_t *buf, size_t buf_size,
                                      int first_byte_timeout_s)
{
    // Wait for the first byte using the caller-supplied idle timeout.
    struct timeval first_to = { .tv_sec = first_byte_timeout_s, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &first_to, sizeof(first_to));

    int n = recv(fd, buf, 1, 0);
    if (n <= 0) return n;          // 0 = peer closed, <0 = timeout/error
    size_t total = 1;

    // After the first byte, switch to a short inter-byte gap to assemble
    // the rest of the frame (most clients send the whole ADU in one write).
    struct timeval gap_to = { .tv_sec = 0,
                              .tv_usec = PASSTHROUGH_TCP_GAP_MS * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &gap_to, sizeof(gap_to));

    while (total < buf_size) {
        n = recv(fd, buf + total, buf_size - total, 0);
        if (n > 0) {
            total += n;
        } else if (n == 0) {
            return -1;             // peer closed mid-frame
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;  // gap => done
            return -1;
        }
    }
    return (int)total;
}

static int passthrough_read_rtu_response(uint8_t *buf, size_t buf_size,
                                         int first_byte_timeout_ms)
{
    // Wait for the first response byte.
    int len = uart_read_bytes(UART_NUM_1, buf, 1,
                              pdMS_TO_TICKS(first_byte_timeout_ms));
    if (len <= 0) return len;

    // Drain remaining bytes until an inter-byte gap >= PASSTHROUGH_RTU_GAP_MS.
    while (len < (int)buf_size) {
        int r = uart_read_bytes(UART_NUM_1, buf + len, buf_size - len,
                                pdMS_TO_TICKS(PASSTHROUGH_RTU_GAP_MS));
        if (r > 0) len += r;
        else break;
    }
    return len;
}

void modbus_passthrough_task(void *pvParameters)
{
    int listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd < 0) {
        ESP_LOGE("PASSTHRU", "socket() failed");
        vTaskDelete(NULL);
        return;
    }
    int reuse = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    int pt_port = (ip_config && ip_config[0].pass_through_port > 0)
                      ? ip_config[0].pass_through_port
                      : PASSTHROUGH_PORT_DEFAULT;

    struct sockaddr_in servaddr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(pt_port),
    };
    if (bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        ESP_LOGE("PASSTHRU", "bind() failed on port %d", pt_port);
        close(listenfd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listenfd, 1) < 0) {
        ESP_LOGE("PASSTHRU", "listen() failed");
        close(listenfd);
        vTaskDelete(NULL);
        return;
    }
    status_led_set_listening(true);

    uint8_t tx_buf[PASSTHROUGH_BUF_SIZE];
    uint8_t rx_buf[PASSTHROUGH_BUF_SIZE];

    while (1) {
        printf("Waiting for TCP client on port %d (pass-through)...\n", pt_port);
        int cfd = accept(listenfd, NULL, NULL);
        if (cfd < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        printf("Pass-through client connected on port %d\n", pt_port);

        int idle_timeout_s = ip_config[0].server_timeout;
        if (idle_timeout_s <= 0) idle_timeout_s = 10;

        while (1) {
            int req_len = passthrough_read_tcp_frame(cfd, tx_buf, sizeof(tx_buf),
                                                    idle_timeout_s);
            if (req_len <= 0) {
                printf("Pass-through: TCP idle/closed (n=%d), closing.\n", req_len);
                break;
            }
            status_led_tcp_activity();
            ESP_LOG_BUFFER_HEX("PT_TCP_IN", tx_buf, req_len);

            int rsp_len = 0;
            /* Total attempts = 1 initial + Passthr_retry_cnt retries.
               Holding the mutex across retries keeps the bus reserved so
               another task can't insert a frame between our retransmits. */
            int retries = (ip_config && ip_config[0].Passthr_retry_cnt > 0)
                              ? ip_config[0].Passthr_retry_cnt : 0;
            if (xSemaphoreTake(xModbus_Mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
                for (int attempt = 0; attempt <= retries; attempt++) {
                    uart_flush_input(UART_NUM_1);
                    uart_write_bytes(UART_NUM_1, (const char *)tx_buf, req_len);
                    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(1000));

                    rsp_len = passthrough_read_rtu_response(rx_buf, sizeof(rx_buf),
                                                           idle_timeout_s * 1000);
                    if (rsp_len > 0) break;

                    if (attempt < retries) {
                        ESP_LOGW("PASSTHRU",
                                 "no RTU response (attempt %d/%d), retrying",
                                 attempt + 1, retries + 1);
                        vTaskDelay(pdMS_TO_TICKS(50));   // short inter-frame pause
                    }
                }
                xSemaphoreGive(xModbus_Mutex);
            } else {
                ESP_LOGW("PASSTHRU", "Bus busy, dropping request");
            }

            if (rsp_len > 0) {
                ESP_LOG_BUFFER_HEX("PT_TCP_OUT", rx_buf, rsp_len);
                int sent = send(cfd, rx_buf, rsp_len, 0);
                if (sent < 0) {
                    printf("Pass-through: send() failed, closing.\n");
                    break;
                }
                status_led_tcp_response();
            } else {
                ESP_LOGW("PASSTHRU", "No RTU response after %d attempt(s)",
                         retries + 1);
            }
        }
        close(cfd);
        printf("Pass-through socket closed, ready for next client.\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void tcp_app_init(void)
{
    modbus_Tcp_server_init();
    xTaskCreate(modbus_gateway_task,     "modbus_tcp_task", 8192, NULL, 5, NULL);
    xTaskCreate(modbus_passthrough_task, "modbus_pt_task",  8192, NULL, 5, NULL);
    http_config_server_init();
}