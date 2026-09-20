#ifndef CJSON_SHARED_H
#define CJSON_SHARED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void cjson_shared_init(void);
void cjson_shared_set_buffer(uint8_t *buffer, size_t size);
bool cjson_shared_acquire(void);
void cjson_shared_release(void);
size_t cjson_shared_used(void);
size_t cjson_shared_peak(void);

#ifdef __cplusplus
}
#endif

#endif /* CJSON_SHARED_H */
