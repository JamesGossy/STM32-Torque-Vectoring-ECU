/*
 * syscalls.c - minimal newlib system call stubs.
 *
 * Only the heap is real (printf's float formatting allocates); there is no
 * file system or console.
 */
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>

extern char end, _estack, _Min_Stack_Size;   /* from the linker script */

/* Grow the heap upwards from the end of .bss, stopping short of the stack. */
void *_sbrk(int incr)
{
    static char *brk = &end;
    char *limit = &_estack - (uint32_t)&_Min_Stack_Size;
    if (brk + incr > limit) {
        errno = ENOMEM;
        return (void *)-1;
    }
    char *prev = brk;
    brk += incr;
    return prev;
}

int _close(int f) { (void)f; return -1; }
int _read(int f, char *p, int n) { (void)f; (void)p; (void)n; return 0; }
int _write(int f, char *p, int n) { (void)f; (void)p; return n; }
int _lseek(int f, int o, int w) { (void)f; (void)o; (void)w; return 0; }
int _isatty(int f) { (void)f; return 1; }
int _fstat(int f, struct stat *s) { (void)f; s->st_mode = S_IFCHR; return 0; }
int _getpid(void) { return 1; }
int _kill(int p, int s) { (void)p; (void)s; errno = EINVAL; return -1; }
void _exit(int c) { (void)c; for (;;) {} }
