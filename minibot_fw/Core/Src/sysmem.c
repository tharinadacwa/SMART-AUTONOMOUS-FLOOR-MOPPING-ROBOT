/* sysmem.c -- _sbrk. The heap is capped by _Min_Stack_Size in the linker script,
 * so a runaway allocation returns NULL instead of quietly eating the stack and
 * corrupting the step-engine state. On a robot, silent corruption is worse than
 * a clean failure. */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

extern uint8_t _end;
extern uint8_t _estack;
extern uint8_t _Min_Stack_Size;

void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *heap = NULL;
    uint8_t *prev;
    const uint8_t *limit = (uint8_t *)((uintptr_t)&_estack -
                                       (uintptr_t)&_Min_Stack_Size);
    if (heap == NULL) {
        heap = &_end;
    }
    if (heap + incr > limit) {
        errno = ENOMEM;
        return (void *)-1;
    }
    prev = heap;
    heap += incr;
    return (void *)prev;
}
