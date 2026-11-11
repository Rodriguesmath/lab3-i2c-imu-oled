#include <errno.h>
#include <stddef.h>
#include <stdint.h>

/* Heap usado pelo printf float da newlib-nano; reserva 4 KiB para a pilha. */
void *_sbrk(ptrdiff_t increment)
{
    extern char _end;
    extern char _estack;
    static char *heapEnd;
    if(heapEnd == NULL) heapEnd = &_end;
    uintptr_t current = (uintptr_t)heapEnd;
    uintptr_t limit = (uintptr_t)&_estack - 0x1000U;
    if((increment >= 0 && (uintptr_t)increment > limit - current) ||
       (increment < 0 && (uintptr_t)(-(increment + 1)) + 1U > current - (uintptr_t)&_end))
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    heapEnd += increment;
    return (void *)current;
}
