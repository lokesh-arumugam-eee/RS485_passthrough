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
#include "esp_task_wdt.h"
#include "status_led.h"
static const char *TAG = "modbustcp";

#define PASSTHROUGH_PORT_DEFAULT  503
#define PASSTHROUGH_BUF_SIZE      260      // RTU max ADU = 256, plus a small margin
#define PASSTHROUGH_TCP_GAP_MS    50       // idle gap that marks end of TCP-side frame
#define PASSTHROUGH_RTU_GAP_MS    50       // idle gap that marks end of RTU response
/* Wait for the FIRST byte of the RTU reply. This used to reuse the TCP idle
   timeout (server_timeout, in seconds) as a millisecond value, giving a 60 s
   wait. It must stay well UNDER the eth91-daemon's frame_timeout (1000 ms):
   if the bridge is more patient than its client, a late reply arrives after
   the daemon has already given up and desyncs the request/response stream. */
#define PASSTHROUGH_RTU_FIRST_BYTE_MS  300
/* Longest single blocking wait before we come up for air to feed the task
   watchdog. The TCP idle timeout is legitimately tens of seconds, so it is
   served as a sequence of these slices rather than one long block. */
#define PASSTHROUGH_WDT_SLICE_S   2
/* Cap on the port-502 gateway's socket idle timeout, so its blocking poll
   always returns well inside the task watchdog period. */
#define GATEWAY_MAX_IDLE_S        20
/* E-2: how many consecutive PASSTHROUGH_TCP_GAP_MS timeouts to tolerate while
   waiting for the rest of a partially-arrived ADU before declaring it truncated
   and discarding it. 10 x 50 ms = 500 ms, comfortably over the measured p95
   inter-segment gap (~250 ms) yet well under the daemon's 2000 ms deadline. */
#define PASSTHROUGH_ASSEMBLE_TRIES 10

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
static int32_t my_tcp_server_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg);


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

// Port 502 server replies count as TCP activity for the status LED.
static int32_t my_tcp_server_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    int32_t sent = my_tcp_write(buf, count, byte_timeout_ms, arg);
    if (sent > 0) {
        status_led_tcp_activity();
        status_led_tcp_response();
    }
    return sent;
}

void modbus_Tcp_server_init(void) {
    nmbs_platform_conf_create(&tcp_conf);
    tcp_conf.transport = NMBS_TRANSPORT_RTU; // Change to TCP
    tcp_conf.read = my_tcp_read;
    tcp_conf.write = my_tcp_server_write;
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

    /* Bound accept() so an idle listener still feeds the watchdog. */
    struct timeval accept_to = { .tv_sec = PASSTHROUGH_WDT_SLICE_S, .tv_usec = 0 };
    setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, &accept_to, sizeof(accept_to));
    esp_task_wdt_add(NULL);

    while (1) {
        printf("Waiting for TCP Client on port %d...\n", mb_port);
        esp_task_wdt_reset();
        connfd = accept(listenfd, NULL, NULL);
        
        if (connfd >= 0) {
            printf("TCP Client Connected!\n");

            // --- INACTIVITY TIMEOUT CONFIGURATION ---
            /* Idle timeout for receiving data, clamped to stay under the task
               watchdog period — nmbs_server_poll() blocks for this long, and a
               block longer than the WDT would look like a hang. */
            struct timeval timeout;
            timeout.tv_sec = ip_config[0].server_timeout;
            if (timeout.tv_sec > GATEWAY_MAX_IDLE_S) timeout.tv_sec = GATEWAY_MAX_IDLE_S;
            if (timeout.tv_sec <= 0) timeout.tv_sec = GATEWAY_MAX_IDLE_S;
            timeout.tv_usec = 0;

            if (setsockopt(connfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
                printf("Failed to set timeout socket option\n");
            }
            // -----------------------------------------

            // Update the platform argument to the NEW connection file descriptor
            tcp_conf.arg = &connfd;

            while (1) {
                esp_task_wdt_reset();
                // nmbs_server_poll returns an error once the idle timeout passes with no data
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
/* E-2: expected total length of a Modbus RTU *request* ADU (including the two
   CRC bytes), derived from the function code. This is what lets us read exactly
   one frame instead of "whatever arrived in the last 50 ms".
     >0  definite total length
      0  need more bytes before the length can be decided (variable-length FCs)
     -1  function code not recognised - caller falls back to gap framing
   Requests only: this is a master->slave passthrough, so buf[1] is always a
   request function code. */
static int rtu_request_expected_len(const uint8_t *b, size_t have)
{
    if (have < 2) return 0;
    switch (b[1]) {
        case 0x01: case 0x02: case 0x03: case 0x04:   /* read coils/inputs/regs */
        case 0x05: case 0x06:                         /* write single          */
            return 8;      /* id fc addr(2) val(2) crc(2)                      */
        case 0x07: case 0x0B: case 0x0C: case 0x11:   /* no-payload diagnostics */
            return 4;      /* id fc crc(2)                                     */
        case 0x0F: case 0x10:                         /* write multiple        */
            if (have < 7) return 0;                   /* need the byte count   */
            return 9 + b[6];
        case 0x16:                                    /* mask write register   */
            return 10;
        case 0x17:                                    /* read/write multiple   */
            if (have < 11) return 0;
            return 13 + b[10];
        default:
            return -1;
    }
}

static int passthrough_read_tcp_frame(int fd, uint8_t *buf, size_t buf_size,
                                      int first_byte_timeout_s)
{
    /* Wait for the first byte using the caller-supplied idle timeout, but in
       PASSTHROUGH_WDT_SLICE_S slices so the task watchdog keeps being fed
       while the client is simply idle between sweeps. */
    struct timeval first_to = { .tv_sec = PASSTHROUGH_WDT_SLICE_S, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &first_to, sizeof(first_to));

    int n = -1;
    int waited_s = 0;
    while (waited_s < first_byte_timeout_s) {
        esp_task_wdt_reset();
        n = recv(fd, buf, 1, 0);
        if (n > 0) break;
        if (n == 0) return 0;                                   // peer closed
        if (errno != EAGAIN && errno != EWOULDBLOCK) return -1;  // real error
        waited_s += PASSTHROUGH_WDT_SLICE_S;
    }
    if (n <= 0) return -1;         // idle timeout expired with no data
    size_t total = 1;

    /* E-2: assemble EXACTLY one ADU, length-driven rather than gap-driven.
       We never request more bytes than the current frame needs, so a queued
       second request stays in the socket for the next call instead of being
       glued on (merging), and a frame split across segments more than
       PASSTHROUGH_TCP_GAP_MS apart is still completed rather than truncated. */
    struct timeval gap_to = { .tv_sec = 0,
                              .tv_usec = PASSTHROUGH_TCP_GAP_MS * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &gap_to, sizeof(gap_to));

    int want  = 0;      /* 0 = length not yet known */
    int idle  = 0;      /* consecutive gap timeouts while mid-frame */

    for (;;) {
        if (want == 0) {
            want = rtu_request_expected_len(buf, total);
            if (want < 0) {
                /* Unrecognised function code: fall back to the old gap framing
                   so unusual requests still pass through rather than stalling. */
                ESP_LOGW("PASSTHRU", "unknown fc 0x%02x, gap-framing this one",
                         total >= 2 ? buf[1] : 0);
                break;
            }
            if (want > 0 && (size_t)want > buf_size) return -1;   /* absurd */
        }

        if (want > 0 && total >= (size_t)want) return want;       /* exactly one ADU */

        /* Ask for only the bytes this frame still needs (1 while probing). */
        size_t need = (want > 0) ? ((size_t)want - total) : 1;
        if (total + need > buf_size) return -1;

        esp_task_wdt_reset();
        n = recv(fd, buf + total, need, 0);
        if (n > 0) {
            total += n;
            idle = 0;
        } else if (n == 0) {
            return -1;                                   /* peer closed mid-frame */
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            if (++idle >= PASSTHROUGH_ASSEMBLE_TRIES) {
                ESP_LOGW("PASSTHRU", "incomplete ADU: got %u of %d bytes, discarding",
                         (unsigned)total, want);
                return -1;                               /* truncated - drop, don't forward */
            }
        } else {
            return -1;
        }
    }

    /* Fallback path (unknown function code only): original gap framing. */
    while (total < buf_size) {
        esp_task_wdt_reset();
        n = recv(fd, buf + total, buf_size - total, 0);
        if (n > 0) {
            total += n;
        } else if (n == 0) {
            return -1;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
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

    /* Bound accept() so an idle listener still comes back to feed the
       watchdog. Subscribe only past the early-return paths above, so we
       never leave a deleted task registered. */
    struct timeval accept_to = { .tv_sec = PASSTHROUGH_WDT_SLICE_S, .tv_usec = 0 };
    setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, &accept_to, sizeof(accept_to));
    esp_task_wdt_add(NULL);

    uint8_t tx_buf[PASSTHROUGH_BUF_SIZE];
    uint8_t rx_buf[PASSTHROUGH_BUF_SIZE];

    while (1) {
        printf("Waiting for TCP client on port %d (pass-through)...\n", pt_port);
        esp_task_wdt_reset();
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
                                                           PASSTHROUGH_RTU_FIRST_BYTE_MS);
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