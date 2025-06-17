#define CHUNK_SAMPLES           32
#define CHUNK_BYTES            (CHUNK_SAMPLES * sizeof(uint32_t))

/* double buffer: 2 × 32 samples = 64 samples total                       // ⇦ */
#define DMA_RING_SAMPLES        (CHUNK_SAMPLES * 2u)