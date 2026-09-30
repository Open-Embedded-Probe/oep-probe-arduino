#pragma once
#include <stdlib.h>
#define MALLOC_CAP_DMA 1
#define MALLOC_CAP_INTERNAL 2
inline void *heap_caps_aligned_alloc(size_t align, size_t size, int) { return aligned_alloc(align, (size + align - 1) / align * align); }
