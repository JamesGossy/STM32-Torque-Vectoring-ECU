/* Blocking sockets with Nagle turned off, because the lock-step link sends many
   tiny messages and waits for each reply. */
#include "tcp.h"
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
static void start(void)
{
    static int started;
    if (!started) {
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
        started = 1;
    }
}
void tcp_close(int sock)
{
    closesocket((SOCKET)sock);
}
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
static void start(void)
{
    signal(SIGPIPE, SIG_IGN);
} // a closed client must not kill us
void tcp_close(int sock)
{
    close(sock);
}
#endif

int tcp_listen(int port)
{
    start();
    int sock = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    int on = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof on);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // local connections only
    addr.sin_port        = htons((unsigned short)port);
    if (bind(sock, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(sock, 1) < 0) {
        tcp_close(sock);
        return -1;
    }
    return sock;
}

int tcp_accept(int listener)
{
    int sock = (int)accept(listener, 0, 0);
    if (sock < 0) return -1;
    int on = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&on, sizeof on);
    return sock;
}

int tcp_connect(int port)
{
    start();
    int sock = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons((unsigned short)port);
    if (connect(sock, (struct sockaddr *)&addr, sizeof addr) < 0) {
        tcp_close(sock);
        return -1;
    }
    int on = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&on, sizeof on);
    return sock;
}

int tcp_read(int sock, void *buf, int len)
{
    char *p = buf;
    while (len > 0) {
        int n = (int)recv(sock, p, len, 0);
        if (n <= 0) return -1;
        p += n;
        len -= n;
    }
    return 0;
}

int tcp_write(int sock, const void *buf, int len)
{
    const char *p = buf;
    while (len > 0) {
        int n = (int)send(sock, p, len, 0);
        if (n <= 0) return -1;
        p += n;
        len -= n;
    }
    return 0;
}
