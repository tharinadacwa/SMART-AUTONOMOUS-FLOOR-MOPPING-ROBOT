/* syscalls.c -- minimal newlib stubs. We never use printf-to-console, files or
 * dynamic allocation, but the linker still demands these symbols exist. */
#include <sys/stat.h>
#include <errno.h>
#include <stddef.h>

int  _close(int f)                  { (void)f; return -1; }
int  _fstat(int f, struct stat *st) { (void)f; st->st_mode = S_IFCHR; return 0; }
int  _isatty(int f)                 { (void)f; return 1; }
int  _lseek(int f, int p, int d)    { (void)f; (void)p; (void)d; return 0; }
int  _read(int f, char *b, int l)   { (void)f; (void)b; (void)l; return 0; }
int  _write(int f, char *b, int l)  { (void)f; (void)b; return l; }
void _exit(int s)                   { (void)s; while (1) { } }
int  _kill(int p, int s)            { (void)p; (void)s; errno = EINVAL; return -1; }
int  _getpid(void)                  { return 1; }
