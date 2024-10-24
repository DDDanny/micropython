/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Mike Teachman
 * Copyright (c) 2023 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//#include "py/runtime.h" //TODO: DF check if include .h. this is required in this .h file
//#include "py/stream.h" //TODO: DF check if include .h. this is required in this .h file

#if MICROPY_PY_MACHINE_I2S //|| MICROPY_PY_MACHINE_PDM

//#include "extmod/modmachine.h" //TODO: DF check if include .h. this is required in this .h file

#if MICROPY_PY_MACHINE_I2S_RING_BUF

typedef struct _ring_buf_t {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
} ring_buf_t;

typedef struct _non_blocking_descriptor_t {
    mp_buffer_info_t appbuf;
    uint32_t index;
    bool copy_in_progress;
} non_blocking_descriptor_t;

static void ringbuf_init(ring_buf_t *rbuf, uint8_t *buffer, size_t size);
static bool ringbuf_push(ring_buf_t *rbuf, uint8_t data);
static bool ringbuf_pop(ring_buf_t *rbuf, uint8_t *data);
static size_t ringbuf_available_data(ring_buf_t *rbuf);
static size_t ringbuf_available_space(ring_buf_t *rbuf);
static void fill_appbuf_from_ringbuf_non_blocking(machine_i2s_obj_t *self);
static void copy_appbuf_to_ringbuf_non_blocking(machine_i2s_obj_t *self);

#endif // MICROPY_PY_MACHINE_I2S_RING_BUF

#endif // MICROPY_PY_MACHINE_I2S || MICROPY_PY_MACHINE_PDM
