/*
* ringbuf.c - C ring buffer (FIFO) implementation.
*
* Written in 2011 by Drew Hess <dhess-src@bothan.net>.
*
* To the extent possible under law, the author(s) have dedicated all
* copyright and related and neighboring rights to this software to
* the public domain worldwide. This software is distributed without
* any warranty.
*
* You should have received a copy of the CC0 Public Domain Dedication
* along with this software. If not, see
* <http://creativecommons.org/publicdomain/zero/1.0/>.
*/

#include "ring_buffer.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "ISConstants.h"

ringbuf_t* ringbuf_new(size_t capacity)
{
    ringbuf_t* rb = (ringbuf_t*)MALLOC(sizeof(ringbuf_t));
	if (rb)
	{
		/* One byte is used for detecting the full condition. */
		rb->size = capacity + 1;
        rb->buf = (uint8_t*)MALLOC(rb->size);
		if (rb->buf)
		{
			ringbuf_reset(rb);
		}
		else
		{
            FREE(rb);
			return 0;
		}
	}
	return rb;
}

size_t ringbuf_buffer_size(const ringbuf_t *rb)
{
	return rb->size;
}

void ringbuf_reset(ringbuf_t* rb)
{
	rb->head = rb->tail = rb->buf;
}

void ringbuf_free(ringbuf_t** rb)
{
    if (rb != 0 && *rb != 0)
    {
        FREE((*rb)->buf);
        FREE(*rb);
        *rb = 0;
    }
}

size_t ringbuf_capacity(const ringbuf_t* rb)
{
	return ringbuf_buffer_size(rb) - 1;
}

/*
* Return a pointer to one-past-the-end of the ring buffer's
* contiguous buffer. You shouldn't normally need to use this function
* unless you're writing a new ringbuf_* function.
*/
static const uint8_t* ringbuf_end(const ringbuf_t* rb)
{
	return rb->buf + ringbuf_buffer_size(rb);
}

size_t ringbuf_bytes_free(const ringbuf_t* rb)
{
	if (rb->head >= rb->tail)
	{
		return ringbuf_capacity(rb) - (rb->head - rb->tail);
	}
	else
	{
		return rb->tail - rb->head - 1;
	}
}

size_t ringbuf_bytes_used(const ringbuf_t *rb)
{
	return ringbuf_capacity(rb) - ringbuf_bytes_free(rb);
}

int ringbuf_is_full(const ringbuf_t *rb)
{
	return ringbuf_bytes_free(rb) == 0;
}

int ringbuf_is_empty(const ringbuf_t *rb)
{
	return ringbuf_bytes_free(rb) == ringbuf_capacity(rb);
}

const void* ringbuf_tail(const ringbuf_t *rb)
{
	return rb->tail;
}

const void* ringbuf_head(const ringbuf_t *rb)
{
	return rb->head;
}

/*
* Given a ring buffer rb and a pointer to a location within its
* contiguous buffer, return the a pointer to the next logical
* location in the ring buffer.
*/
static uint8_t* ringbuf_nextp(ringbuf_t* rb, const uint8_t *p)
{
	/*
	* The assert guarantees the expression (++p - rb->buf) is
	* non-negative; therefore, the modulus operation is safe and
	* portable.
	*/
	assert((p >= rb->buf) && (p < ringbuf_end(rb)));
	return rb->buf + ((++p - rb->buf) % ringbuf_buffer_size(rb));
}

size_t ringbuf_findchr(const ringbuf_t* rb, int c, size_t offset)
{
	const uint8_t *bufend = ringbuf_end(rb);
	size_t bytes_used = ringbuf_bytes_used(rb);
	if (offset >= bytes_used)
		return bytes_used;

	const uint8_t *start = rb->buf +
		(((rb->tail - rb->buf) + offset) % ringbuf_buffer_size(rb));
	assert(bufend > start);
	size_t n = _MIN((size_t)(bufend - start), bytes_used - offset);
	const uint8_t *found = memchr(start, c, n);
	if (found)
		return offset + (found - start);
	else
		return ringbuf_findchr(rb, c, offset + n);
}

size_t ringbuf_memset(ringbuf_t* dst, int c, size_t len)
{
	const uint8_t *bufend = ringbuf_end(dst);
	size_t nwritten = 0;
	size_t count = _MIN(len, ringbuf_buffer_size(dst));
	int overflow = count > ringbuf_bytes_free(dst);

	while (nwritten != count) {

		/* don't copy beyond the end of the buffer */
		assert(bufend > dst->head);
		size_t n = _MIN((size_t)(bufend - dst->head), count - nwritten);
		memset(dst->head, c, n);
		dst->head += n;
		nwritten += n;

        // wrap?
		if (dst->head == bufend)
			dst->head = dst->buf;
	}

	if (overflow) {
		dst->tail = ringbuf_nextp(dst, dst->head);
		assert(ringbuf_is_full(dst));
	}

	return nwritten;
}

void* ringbuf_memcpy_into(ringbuf_t* dst, const void *src, size_t count, size_t* countCopied)
{
	const uint8_t *u8src = src;
	const uint8_t *bufend = ringbuf_end(dst);
	int overflow = count > ringbuf_bytes_free(dst);
	size_t nread = 0;

	while (nread != count) {
		/* don't copy beyond the end of the buffer */
		assert(bufend > dst->head);
		size_t n = _MIN((size_t)(bufend - dst->head), count - nread);
		memcpy(dst->head, u8src + nread, n);
		dst->head += n;
		nread += n;

        // wrap?
		if (dst->head == bufend)
			dst->head = dst->buf;
	}

	if (overflow) {
		dst->tail = ringbuf_nextp(dst, dst->head);
		assert(ringbuf_is_full(dst));
	}

    if (countCopied != 0)
    {
        *countCopied = nread;
    }

	return dst->head;
}

/*
size_t ringbuf_read(int fd, ringbuf_t rb, size_t count)
{
	const uint8_t *bufend = ringbuf_end(rb);
	size_t nfree = ringbuf_bytes_free(rb);

	// don't write beyond the end of the buffer
	assert(bufend > rb->head);
	count = MIN(bufend - rb->head, count);
	size_t n = read(fd, rb->head, count);
	if (n > 0) {
		assert(rb->head + n <= bufend);
		rb->head += n;

		// wrap?
		if (rb->head == bufend)
			rb->head = rb->buf;

		// fix up the tail pointer if an overflow occurred
		if (n > nfree) {
			rb->tail = ringbuf_nextp(rb, rb->head);
			assert(ringbuf_is_full(rb));
		}
	}

	return n;
}
*/

void* ringbuf_memcpy_from(void* dst, ringbuf_t* src, size_t count, size_t* countCopied)
{
	size_t bytes_used = ringbuf_bytes_used(src);
    if (count <= bytes_used)
    {
        uint8_t *u8dst = dst;
        const uint8_t *bufend = ringbuf_end(src);
        size_t nwritten = 0;
        while (nwritten != count) {
            assert(bufend > src->tail);
            size_t n = _MIN((size_t)(bufend - src->tail), count - nwritten);
            memcpy(u8dst + nwritten, src->tail, n);
            src->tail += n;
            nwritten += n;

            // wrap?
            if (src->tail == bufend)
                src->tail = src->buf;
        }

        if (*countCopied != 0)
        {
            *countCopied = nwritten;
        }
    }
	assert(count + ringbuf_bytes_used(src) == bytes_used);
	return src->tail;
}

/*
size_t ringbuf_write(int fd, ringbuf_t rb, size_t count)
{
	size_t bytes_used = ringbuf_bytes_used(rb);
	if (count > bytes_used)
		return 0;

	const uint8_t *bufend = ringbuf_end(rb);
	assert(bufend > rb->head);
	count = MIN(bufend - rb->tail, count);
	size_t n = write(fd, rb->tail, count);
	if (n > 0) {
		assert(rb->tail + n <= bufend);
		rb->tail += n;

		// wrap?
		if (rb->tail == bufend)
			rb->tail = rb->buf;

		assert(n + ringbuf_bytes_used(rb) == bytes_used);
	}

	return n;
}
*/

void* ringbuf_copy(ringbuf_t* dst, ringbuf_t* src, size_t count)
{
	size_t src_bytes_used = ringbuf_bytes_used(src);
	if (count > src_bytes_used)
		return 0;
	int overflow = count > ringbuf_bytes_free(dst);

	const uint8_t *src_bufend = ringbuf_end(src);
	const uint8_t *dst_bufend = ringbuf_end(dst);
	size_t ncopied = 0;
	while (ncopied != count) {
		assert(src_bufend > src->tail);
		size_t nsrc = _MIN((size_t)(src_bufend - src->tail), count - ncopied);
		assert(dst_bufend > dst->head);
		size_t n = _MIN((size_t)(dst_bufend - dst->head), nsrc);
		memcpy(dst->head, src->tail, n);
		src->tail += n;
		dst->head += n;
		ncopied += n;

        // wrap?
		if (src->tail == src_bufend)
			src->tail = src->buf;
		if (dst->head == dst_bufend)
			dst->head = dst->buf;
	}

	assert(count + ringbuf_bytes_used(src) == src_bytes_used);

	if (overflow) {
		dst->tail = ringbuf_nextp(dst, dst->head);
		assert(ringbuf_is_full(dst));
	}

	return dst->head;
}
