#include <csp/drivers/can_socketcan.h>

#include <zephyr/kernel/thread.h>
#include <stdlib.h>
#include <csp/csp_debug.h>
#include <zephyr/net/socket.h>
#include <unistd.h>
#include <errno.h>
#include <unistd.h>

#include <csp/csp.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socketcan.h>
#include <zephyr/net/socketcan_utils.h>

/* RX Thread parameters */
#define PRIORITY  k_thread_priority_get(k_current_get())
#define STACKSIZE (512)

static K_THREAD_STACK_DEFINE(rx_stack, STACKSIZE);

/* CAN interface data, state, etc. */
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	struct device * can_dev;
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
	struct k_thread rx_data;
	k_tid_t rx_tid;
	int socket;
} can_context_t;

static can_context_t ctx;

static void socketcan_free(can_context_t *ctx) {

	if (ctx) {
		if (ctx->socket >= 0) {
			zsock_close(ctx->socket);
		}
	}
}

static void socketcan_rx_thread(can_context_t * pctx) {

	int ret;
	struct can_frame zframe;
	struct socketcan_frame sframe;

	while (1) {
		/* Read CAN frame */
		ret = zsock_recvfrom(pctx->socket, &sframe, sizeof(sframe), 0, NULL, NULL);
		if (ret < 0) {
			csp_print("%s[%s]: recvfrom() failed, errno %d\n", __FUNCTION__, pctx->name, ret);
			continue;
		}

		socketcan_to_can_frame(&sframe, &zframe);

		/* Drop frames with standard id (CSP uses extended) */
		if (!(zframe.flags & CAN_FRAME_IDE)) {
			continue;
		}

		/* Drop remote frames */
		if (zframe.flags & (CAN_FRAME_RTR)) {
			csp_print("%s[%s]: discarding RTR frame\n", __FUNCTION__, pctx->name);
			continue;
		}

		/* Call RX callbacsp_can_rx_frameck */
		csp_can_rx(&pctx->iface, zframe.id, zframe.data, zframe.dlc, NULL);
	}
}

static int csp_can_tx_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc) {

	int ret;
	struct can_frame zframe = {0};
	struct socketcan_frame sframe = {0};
	can_context_t * pctx = (can_context_t * )driver_data;

	if (dlc > 8) {
		return CSP_ERR_INVAL;
	}

	zframe.id = id;
	zframe.dlc = dlc;
	zframe.flags = CAN_FRAME_IDE;
	memcpy(zframe.data, data, dlc);

	socketcan_from_can_frame(&zframe, &sframe);

	ret = zsock_send(pctx->socket, &sframe, sizeof(sframe), 0);
	if (ret < 0) {
		csp_print("%s[%s]: send() failed, errno %d\n", __FUNCTION__, pctx->name, ret);
		return CSP_ERR_TX;
	}

	return CSP_ERR_NONE;
}

int csp_can_socketcan_set_promisc(const bool promisc, can_context_t *pctx) {

	int ret;
	struct can_filter zfilter = {
		.flags = CAN_FILTER_DATA | CAN_FILTER_IDE,
		.id = CFP_MAKE_DST(pctx->iface.addr),
		.mask = 0x0000, /* receive anything */
	};
	struct socketcan_filter sfilter;

	if (pctx->socket < 0) {
		return CSP_ERR_INVAL;
	}

	if (!promisc) {
		if (csp_conf.version == 1) {
			zfilter.id = CFP_MAKE_DST(pctx->iface.addr);
			zfilter.mask = CFP_MAKE_DST((1 << CFP_HOST_SIZE) - 1);
		} else {
			zfilter.id = pctx->iface.addr << CFP2_DST_OFFSET;
			zfilter.mask = CFP2_DST_MASK << CFP2_DST_OFFSET;
		}
	}

	socketcan_from_can_filter(&zfilter, &sfilter);

	ret = zsock_setsockopt(pctx->socket, SOL_CAN_RAW, CAN_RAW_FILTER, &sfilter, sizeof(sfilter));
	if (ret < 0) {
		csp_print("%s: setsockopt() failed, error: %d\n", __FUNCTION__, ret);
		return ret;
	}

	return CSP_ERR_NONE;
}

int csp_can_socketcan_open_and_add_interface(const char * device, const char * ifname, unsigned int node_id,
											 int bitrate, bool promisc, csp_iface_t ** return_iface) {

	int ret;
	struct sockaddr_can can_addr;

	if (ifname == NULL) {
		ifname = CSP_IF_CAN_DEFAULT_NAME;
	}

	csp_print("INIT %s: device: [%s], bitrate: %d, promisc: %d\n", ifname, device, bitrate, promisc);

	ctx.can_dev = DEVICE_DT_GET(DT_NODELABEL(can0));

	/* Set Bit rate */
	ret = can_set_bitrate(ctx.can_dev, bitrate);
	if (ret != 0) {
		csp_print("%s[%s]: can_set_bitrate() failed, error: %s\n", __FUNCTION__, ctx.name, ret);
		return CSP_ERR_INVAL;
	}

	/* Enable CAN */
	ret = can_start(ctx.can_dev);
	if (ret != 0) {
		csp_print("%s[%s]: can_start() failed, error: %s\n", __FUNCTION__, ctx.name, ret);
		return ret;
	}

	/* Set earch parameter */
	strncpy(ctx.name, ifname, sizeof(ctx.name) - 1);
	ctx.iface.name = ctx.name;
	ctx.iface.addr = node_id;
	ctx.iface.interface_data = &ctx.ifdata;
	ctx.iface.driver_data = &ctx;
	ctx.ifdata.tx_func = csp_can_tx_frame;
	ctx.ifdata.pbufs = NULL;

	/* Create socket */
	ctx.socket = zsock_socket(AF_CAN, SOCK_RAW, CAN_RAW);
	if (ctx.socket < 0) {
		csp_print("%s[%s]: socket() failed, error: %s\n", __FUNCTION__, ctx.name, ctx.socket);
		return ctx.socket;
	}

	/* Bind the socket to CAN interface */
	can_addr.can_ifindex = net_if_get_by_iface(net_if_get_first_by_type(&NET_L2_GET_NAME(CANBUS_RAW)));
	printk("can_addr.can_ifindex %d\n", can_addr.can_ifindex);
	can_addr.can_family = PF_CAN;
	ret = zsock_bind(ctx.socket, (struct sockaddr *)&can_addr, sizeof(can_addr));
	if (ret < 0) {
		csp_print("%s[%s]: bind() failed, error: %s\n", __FUNCTION__, ctx.name, ctx.socket);
		socketcan_free(&ctx);
		return ret;
	}

	/* Set filter mode */
	ret = csp_can_socketcan_set_promisc(promisc, &ctx);
	if (ret != CSP_ERR_NONE) {
		csp_print("%s[%s]: csp_can_socketcan_set_promisc() failed, error: %d\n", __FUNCTION__, ctx.name, ret);
		socketcan_free(&ctx);
		return ret;
	}

	/* Add interface to CSP */
	ret = csp_can_add_interface(&ctx.iface);
	if (ret != CSP_ERR_NONE) {
		csp_print("%s[%s]: csp_can_add_interface() failed, error: %d\n", __FUNCTION__, ctx.name, ret);
		socketcan_free(&ctx);
		return ret;
	}

	/* Create receive thread */
	ctx.rx_tid = k_thread_create(&ctx.rx_data, rx_stack,
					 K_THREAD_STACK_SIZEOF(rx_stack),
					 (k_thread_entry_t)socketcan_rx_thread,
					 &ctx, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
	if (!ctx.rx_tid) {
		csp_print("%s[%s]: k_thread_create() failed\n", __FUNCTION__, ctx.name);
		socketcan_free(&ctx);
		return ret;
	}

	if (return_iface) {
		*return_iface = &ctx.iface;
	}

	return CSP_ERR_NONE;
}

csp_iface_t *csp_can_socketcan_init(const char * device, unsigned int node_id, int bitrate, bool promisc) {

	csp_iface_t * return_iface;

	int res = csp_can_socketcan_open_and_add_interface(device, CSP_IF_CAN_DEFAULT_NAME, node_id,
											bitrate, promisc, &return_iface);

	return (res == CSP_ERR_NONE) ? return_iface : NULL;
}

int csp_can_socketcan_stop(csp_iface_t * iface) {

	can_context_t * pctx = iface->driver_data;

	k_thread_abort(pctx->rx_tid);
	socketcan_free(pctx);
	can_stop(pctx->can_dev);

	return CSP_ERR_NONE;
}
