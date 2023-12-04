#include <csp/csp_debug.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <dirent.h>
#include <csp/arch/csp_time.h>
#include <termios.h>

#include <csp/csp.h>
#include <csp/drivers/usart.h>
#include <csp/drivers/can_socketcan.h>
#include <csp/interfaces/csp_if_zmqhub.h>

#define SOF_ERR_READ (1U)

#define I2C_DEV_NO            (1U)
#define I2C_TMP175_SLAVE_ADDR (0x4E)
#define I2C_TMP175_TEMP_REG   (0x00)

#define RASPI_UART1_DEV "/dev/serial0"
#define RASPI_UART1_BAUDRATE (115200U)
#define SOF_BUF_SIZE (20U)
#define SOF_OK       (0U)

/* These three functions must be provided in arch specific way */
int router_start(void);
int server_start(void);
int sensor_start(void);

uint16_t g_temp = 0;
uint16_t g_count = 0;
uint8_t g_sof_err = 0;
uint16_t g_sof_fn = 0;
uint8_t g_sof_drop = 0;

/* Server port, the port the server listens on for incoming connections from the client. */
#define SERVER_PORT		10

#define JPEG_IMAG_PATH "/home/pi/heat_cycle_test/scsat1-rpi/zero/rpios/ImageJPG"
#define JPEG_IMAGE_PREFIX "image"

/* Commandline options */
static uint8_t server_address = 0;

/* Test mode, check that server & client can exchange packets */
static bool test_mode = false;
static unsigned int server_received = 0;

extern csp_conf_t csp_conf;

static int get_tmp175_temp(uint16_t *temp)
{
	int fd;
	unsigned char reg = I2C_TMP175_TEMP_REG;
	unsigned char dat[2];
	char i2c_dev_fn[64];
	float tempfl;
	time_t tm;
	char *timestr;

	time(&tm);
	timestr = ctime(&tm);
	timestr[strlen(timestr)-1] = '\0';

	sprintf(i2c_dev_fn, "/dev/i2c-%d", I2C_DEV_NO);
	if ((fd = open(i2c_dev_fn, O_RDWR)) < 0) {
		printf("[%s] Faild to open i2c port\n", timestr);
		*temp = 0;
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, I2C_TMP175_SLAVE_ADDR) < 0) {
		printf("[%s] Unable to get bus access to talk to slave\n", timestr);
		*temp = 0;
		return -1;
	}

	if ((write(fd, &reg, 1)) != 1) {
		printf("[%s] Error writing to i2c slave\n", timestr);
		*temp = 0;
		return -1;
	}

	if (read(fd, dat, 2) != 2) {
		printf("[%s] Error reading from i2c slave\n", timestr);
		*temp = 0;
		return -1;
	}

	dat[1] = dat[1] >> 4;
	*temp = (dat[0] << 8) | dat[1];
	tempfl = (int8_t)dat[0] + (float)dat[1] * 0.0625;

	close(fd);

	printf("[%s] TMP175 Temperature: %.4f\n", timestr, tempfl);

	return 0;
}

static int get_jpeg_file_count(uint16_t *count)
{
	DIR *dir;
	struct dirent *entry;
	time_t tm;
	char *timestr;
	uint16_t fcount = 0;

	time(&tm);
	timestr = ctime(&tm);
	timestr[strlen(timestr)-1] = '\0';

	dir = opendir(JPEG_IMAG_PATH);
	if (dir == NULL) {
		printf("[%s] Failed to open dir %s\n", timestr, JPEG_IMAG_PATH);
		*count = 0;
		return -1;
	}

	while (1) {
		entry = readdir(dir);
		if (entry == NULL) {
			break;
		} else if (strncmp(entry->d_name, JPEG_IMAGE_PREFIX, strlen(JPEG_IMAGE_PREFIX)) == 0) {
			fcount++;
		}
	}

	closedir(dir);

	*count = fcount;
	printf("[%s] JPEG Count: %d\n", timestr, *count);

	return 0;
}

static void get_sof(int fd)
{
	int ret;
	char buf[SOF_BUF_SIZE];
	uint8_t sof_err;
	uint16_t sof_fn;
	uint8_t sof_drop;
	time_t tm;
	char *timestr;

	time(&tm);
	timestr = ctime(&tm);
	timestr[strlen(timestr)-1] = '\0';

	ret = read(fd, buf, SOF_BUF_SIZE);
	if (ret < 0) {
		printf("[%s] Faild to read UART: %d\n", timestr, ret);
		g_sof_err = SOF_ERR_READ;
		g_sof_fn = 0;
		g_sof_drop = 0;
		goto end;
	} else if (ret == 0) {
		goto end;
	}

	ret = sscanf(buf, "%hhd,%hx,%hhd", &sof_err, &sof_fn, &sof_drop);
	if (ret == 3) {
		printf("[%s] SOF ERR: %d, FN, 0x%3x, DROP: %d\n", timestr, sof_err, sof_fn, sof_drop);
		g_sof_err = sof_err;
		g_sof_fn = sof_fn;
		g_sof_drop = sof_drop;
	}

end:
	return;
}

static int init_uart(void)
{
	struct termios tio;
	int ret;
	int fd;

	fd = open(RASPI_UART1_DEV, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		goto end;
	}

	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = IGNBRK | IGNPAR;
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 0;
	ret = cfsetspeed(&tio, RASPI_UART1_BAUDRATE);
	if (ret < 0) {
		fd = ret;
		goto end;
	}

	ret = tcflush(fd, TCIFLUSH);
	if (ret < 0) {
		fd = ret;
		goto end;
	}

	ret = tcsetattr(fd, TCSANOW, &tio);
	if (ret < 0) {
		fd = ret;
		goto end;
	}

end:
	return fd;
}

/* Sensor task */
void sensor(void) {

	int uart_fd;

	uart_fd = init_uart();
	if (uart_fd < 0) {
		printf("Failed to initialize UART %s\n", strerror(uart_fd));
		goto end;
	}

	while (1) {
		get_tmp175_temp(&g_temp);
		get_jpeg_file_count(&g_count);
		get_sof(uart_fd);
		sleep(1);
	}

end:
	return;
}

/* Server task - handles requests from clients */
void server(void) {

	uint8_t *data_addr;

	csp_print("Server task started\n");

	/* Create socket with no specific socket options, e.g. accepts CRC32, HMAC, etc. if enabled during compilation */
	csp_socket_t sock = {0};

	/* Bind socket to all ports, e.g. all incoming connections will be handled here */
	csp_bind(&sock, CSP_ANY);

	/* Create a backlog of 10 connections, i.e. up to 10 new connections can be queued */
	csp_listen(&sock, 10);

	/* Wait for connections and then process packets on the connection */
	while (1) {

		/* Wait for a new connection, 10000 mS timeout */
		csp_conn_t *conn;
		if ((conn = csp_accept(&sock, 10000)) == NULL) {
			/* timeout */
			continue;
		}

		/* Read packets on connection, timout is 100 mS */
		csp_packet_t *packet;
		while ((packet = csp_read(conn, 50)) != NULL) {
			switch (csp_conn_dport(conn)) {
			case SERVER_PORT:
				csp_print("Get temperature command received\n");
				csp_packet_t * send_packet = csp_buffer_get(0);
				data_addr = send_packet->data;
				memcpy(data_addr, &g_temp, sizeof(g_temp));
				data_addr += sizeof(g_temp);
				memcpy(data_addr, &g_count, sizeof(g_count));
				data_addr += sizeof(g_count);
				memcpy(data_addr, &g_sof_err, sizeof(g_sof_err));
				data_addr += sizeof(g_sof_err);
				memcpy(data_addr, &g_sof_fn, sizeof(g_sof_fn));
				data_addr += sizeof(g_sof_fn);
				memcpy(data_addr, &g_sof_drop, sizeof(g_sof_drop));
				data_addr += sizeof(g_sof_drop);
				send_packet->length = data_addr - send_packet->data;
				csp_send(conn, send_packet);
				csp_buffer_free(send_packet);
				csp_buffer_free(packet);
				++server_received;
				break;

			default:
				/* Call the default CSP service handler, handle pings, buffer use, etc. */
				csp_service_handler(packet);
				break;
			}
		}

		/* Close current connection */
		csp_close(conn);

	}

	return;

}
/* End of server task */

static struct option long_options[] = {
#if (CSP_HAVE_LIBSOCKETCAN)
    {"can-device", required_argument, 0, 'c'},
#endif
    {"kiss-device", required_argument, 0, 'k'},
#if (CSP_HAVE_LIBZMQ)
    {"zmq-device", required_argument, 0, 'z'},
#endif
#if (CSP_USE_RTABLE)
    {"rtable", required_argument, 0, 'R'},
#endif
    {"interface-address", required_argument, 0, 'a'},
    {"connect-to", required_argument, 0, 'C'},
    {"test-mode", no_argument, 0, 't'},
    {"help", no_argument, 0, 'h'},
    {0, 0, 0, 0}
};

void print_help() {
    csp_print("Usage: csp_client [options]\n"
#if (CSP_HAVE_LIBSOCKETCAN)
           " -c <can-device>  set CAN device\n"
#endif
           " -k <kiss-device> set KISS device\n"
#if (CSP_HAVE_LIBZMQ)
           " -z <zmq-device>  set ZeroMQ device\n"
#endif
#if (CSP_USE_RTABLE)
           " -R <rtable>      set routing table\n"
#endif
           " -a <address>     set interface address\n"
           " -t               enable test mode\n"
           " -h               print help\n");
}

/* main - initialization of CSP and start of server/client tasks */
int main(int argc, char * argv[]) {

#if (CSP_HAVE_LIBSOCKETCAN)
    const char * can_device = NULL;
#endif
    const char * kiss_device = NULL;
#if (CSP_HAVE_LIBZMQ)
    const char * zmq_device = NULL;
#endif
#if (CSP_USE_RTABLE)
    const char * rtable = NULL;
#endif
    int opt;
    while ((opt = getopt_long(argc, argv, "c:k:z:R:a:th", long_options, NULL)) != -1) {
        switch (opt) {
#if (CSP_HAVE_LIBSOCKETCAN)
            case 'c':
                can_device = optarg;
                break;
#endif
            case 'k':
                kiss_device = optarg;
                break;
#if (CSP_HAVE_LIBZMQ)
            case 'z':
                zmq_device = optarg;
                break;
#endif
#if (CSP_USE_RTABLE)
            case 'R':
                rtable = optarg;
                break;
#endif
            case 'a':
                server_address = atoi(optarg);
                break;
            case 't':
                test_mode = true;
                break;
            case 'h':
				print_help();
				exit(EXIT_SUCCESS);
            case '?':
                // Invalid option or missing argument
				print_help();
                exit(EXIT_FAILURE);
        }
    }

    // If more than one of the interfaces are set, print a message and exit
    if ((kiss_device && can_device) || (kiss_device && zmq_device) || (can_device && zmq_device)) {
        csp_print("Only one of the interfaces can be set.\n");
        print_help();
        exit(EXIT_FAILURE);
    }

    csp_print("Initialising CSP\n");

	csp_conf.version = 1;

    /* Init CSP */
    csp_init();

    /* Start router */
    router_start();

    /* Add interface(s) */
    csp_iface_t * default_iface = NULL;
    if (kiss_device) {
        csp_usart_conf_t conf = {
            .device = kiss_device,
            .baudrate = 115200, /* supported on all platforms */
            .databits = 8,
            .stopbits = 1,
            .paritysetting = 0,
            .checkparity = 0};
        int error = csp_usart_open_and_add_kiss_interface(&conf, CSP_IF_KISS_DEFAULT_NAME,  &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add KISS interface [%s], error: %d\n", kiss_device, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }
#if (CSP_HAVE_LIBSOCKETCAN)
    if (can_device) {
        int error = csp_can_socketcan_open_and_add_interface(can_device, CSP_IF_CAN_DEFAULT_NAME, server_address, 1000000, true, &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add CAN interface [%s], error: %d\n", can_device, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }
#endif
#if (CSP_HAVE_LIBZMQ)
    if (zmq_device) {
        int error = csp_zmqhub_init(server_address, zmq_device, 0, &default_iface);
        if (error != CSP_ERR_NONE) {
            csp_print("failed to add ZMQ interface [%s], error: %d\n", zmq_device, error);
            exit(1);
        }
        default_iface->is_default = 1;
    }
#endif

#if (CSP_USE_RTABLE)
    if (rtable) {
        int error = csp_rtable_load(rtable);
        if (error < 1) {
            csp_print("csp_rtable_load(%s) failed, error: %d\n", rtable, error);
            exit(1);
        }
    } else if (default_iface) {
        csp_rtable_set(0, 0, default_iface, CSP_NO_VIA_ADDRESS);
    }
#endif

    csp_print("Connection table\r\n");
    csp_conn_print_table();

    csp_print("Interfaces\r\n");
    csp_iflist_print();

#if (CSP_USE_RTABLE)
    csp_print("Route table\r\n");
    csp_rtable_print();
#endif

    /* Start server thread */
    server_start();

    /* Start sensor thread */
    sensor_start();

    /* Wait for execution to end (ctrl+c) */
    while(1) {
        sleep(1);

        if (test_mode) {
            /* Test mode, check that server & client can exchange packets */
            if (server_received < 5) {
                csp_print("Server received %u packets\n", server_received);
                exit(EXIT_FAILURE);
            }
            csp_print("Server received %u packets\n", server_received);
            exit(EXIT_SUCCESS);
        }
    }

    return 0;
}
