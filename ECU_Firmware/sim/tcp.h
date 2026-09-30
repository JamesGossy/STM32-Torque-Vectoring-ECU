/* Tiny blocking TCP helpers for Windows and POSIX. */
#ifndef TCP_H
#define TCP_H

int tcp_listen(int port);                          // on 127.0.0.1, returns a socket or -1
int tcp_accept(int listener);                      // waits for a client
int tcp_connect(int port);                         // to 127.0.0.1, returns a socket or -1
int tcp_read(int sock, void *buf, int len);        // reads exactly len bytes, 0 ok, -1 closed
int tcp_write(int sock, const void *buf, int len); // 0 ok, -1 error
void tcp_close(int sock);

#endif
