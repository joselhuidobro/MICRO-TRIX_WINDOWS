#include "trix_udp_telemetria.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "TRIX_UDP";

/* Estado interno */
static int                udp_sock = -1;
static struct sockaddr_in dest     = {0};

/* ───────── Helpers internos ───────── */
static esp_err_t resolve_address(const char *host_or_ip, uint16_t port)
{
    /* Intento 1: interpretar como IP numérica (v4) */
    uint32_t ip = inet_addr(host_or_ip);         // ← devuelve IP en orden de red o IPADDR_NONE
    if (ip != IPADDR_NONE) {
        dest.sin_addr.s_addr = ip;
    } else {
        /* Intento 2: DNS / mDNS */
        struct addrinfo hints = {
            .ai_family   = AF_INET,
            .ai_socktype = SOCK_DGRAM
        };
        struct addrinfo *res = NULL;
        if (getaddrinfo(host_or_ip, NULL, &hints, &res) != 0 || !res) {
            ESP_LOGE(TAG, "DNS lookup failed for %s", host_or_ip);
            return ESP_FAIL;
        }
        dest.sin_addr = ((struct sockaddr_in *)res->ai_addr)->sin_addr;
        freeaddrinfo(res);
    }

    dest.sin_family = AF_INET;
    dest.sin_port   = htons(port);
    return ESP_OK;
}


/* ───────── API pública ───────── */
esp_err_t trix_udp_telemetria_init(const char *jetson_ip_or_host, uint16_t port)
{
    if (!jetson_ip_or_host) return ESP_ERR_INVALID_ARG;

    /* Crea socket */
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        return ESP_FAIL;
    }

    /* Resuelve dirección destino */
    ESP_ERROR_CHECK_WITHOUT_ABORT(resolve_address(jetson_ip_or_host, port));

    char buf[16];
    inet_ntop(AF_INET, &dest.sin_addr, buf, sizeof(buf));
    ESP_LOGI(TAG, "UDP ready → %s:%u", buf, port);
    return ESP_OK;
}

esp_err_t trix_udp_telemetria_send(const drone_telemetry_t *pkt)
{
    if (!pkt || udp_sock < 0) return ESP_ERR_INVALID_STATE;

    ssize_t sent = sendto(udp_sock, pkt, sizeof(*pkt), 0,
                          (struct sockaddr *)&dest, sizeof(dest));
    return (sent == sizeof(*pkt)) ? ESP_OK : ESP_FAIL;
}

esp_err_t trix_udp_telemetria_deinit(void)
{
    if (udp_sock < 0) return ESP_ERR_INVALID_STATE;
    close(udp_sock);
    udp_sock = -1;
    return ESP_OK;
}

