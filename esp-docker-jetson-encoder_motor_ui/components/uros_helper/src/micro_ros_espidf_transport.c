#include "micro_ros_espidf_transport.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

#include <lwip/sockets.h>
#include <lwip/inet.h>

// (Opcional) si tu tag no exporta el prototipo:
#ifndef RMW_UROS_SET_CUSTOM_TRANSPORT_DECLARED
#define RMW_UROS_SET_CUSTOM_TRANSPORT_DECLARED
// Declaración forward usando firmas genéricas (void*)
void rmw_uros_set_custom_transport(
    bool session_on_client,
    void * args,
    bool   (*open_cb )(void * transport),
    bool   (*close_cb)(void * transport),
    size_t (*write_cb)(void * transport, const uint8_t * buf, size_t len, uint8_t * err),
    size_t (*read_cb )(void * transport, uint8_t * buf, size_t len, int timeout_ms, uint8_t * err)
);
#endif

typedef struct {
    int sock;
    struct sockaddr_in agent;
} uros_udp_ctx_t;

static uros_udp_ctx_t g_ctx = { .sock = -1 };

/* ───────── Callbacks con firma void* (agnósticos) ───────── */
static bool udp_open(void * transport)
{
    (void)transport;
    g_ctx.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    return (g_ctx.sock >= 0);
}

static bool udp_close(void * transport)
{
    (void)transport;
    if (g_ctx.sock >= 0) {
        close(g_ctx.sock);
        g_ctx.sock = -1;
    }
    return true;
}

static size_t udp_write(void * transport,
                        const uint8_t * buf, size_t len, uint8_t * err)
{
    (void)transport; (void)err;
    ssize_t s = sendto(g_ctx.sock, buf, len, 0,
                       (struct sockaddr*)&g_ctx.agent, sizeof(g_ctx.agent));
    return (s < 0) ? 0 : (size_t)s;
}

static size_t udp_read(void * transport,
                       uint8_t * buf, size_t len, int timeout_ms, uint8_t * err)
{
    (void)transport; (void)err;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(g_ctx.sock, &rfds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rv = select(g_ctx.sock + 1, &rfds, NULL, NULL, &tv);
    if (rv <= 0) return 0;

    ssize_t r = recvfrom(g_ctx.sock, buf, len, 0, NULL, NULL);
    return (r < 0) ? 0 : (size_t)r;
}

/* ───────── API pública ───────── */
void set_microros_wifi_transports(const char* ssid,
                                  const char* passwd,
                                  const char* agent_ip,
                                  uint16_t agent_port)
{
    (void)ssid;   // Asumimos Wi-Fi ya arriba (trix_wifi_init, etc.)
    (void)passwd;

    memset(&g_ctx.agent, 0, sizeof(g_ctx.agent));
    g_ctx.agent.sin_family = AF_INET;
    g_ctx.agent.sin_port   = htons(agent_port);
    g_ctx.agent.sin_addr.s_addr = inet_addr(agent_ip);

    rmw_uros_set_custom_transport(
        true,       // session on client
        NULL,       // user args (no usamos)
        udp_open,
        udp_close,
        udp_write,
        udp_read
    );
}

