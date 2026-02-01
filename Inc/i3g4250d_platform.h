#ifndef I3G4250D_PLATFORM_H
#define I3G4250D_PLATFORM_H

#include "i3g4250d_reg.h"


// Core platform functions
int32_t i3g4250d_platform_init(stmdev_ctx_t *ctx, void *spi_handle);


int32_t i3g4250d_platform_write(void *handle, uint8_t reg,
                                const uint8_t *data, uint16_t len);

int32_t i3g4250d_platform_read(void *handle, uint8_t reg,
                               uint8_t *data, uint16_t len);

void i3g4250d_platform_delay(uint32_t ms);

#endif
