#include "ShowStreamReader.h"


#include <string.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

ShowStreamReader::ShowStreamReader()
{
    _src = nullptr;
    _playing = 0;
    _refilling = 1;
    _refill_pending = false;
    _started = false;
    _eof = false;
    _loaded = false;
    _have_anchor = false;
    _pending_light_count = 0;
    _block_count[0] = _block_count[1] = 0;
    _light_count[0] = _light_count[1] = 0;
    _last_refill_ms = 0;
    _io_len = 0;
    _io_pos = 0;
    _threaded = false;
    _io_stop = false;
    _io_busy = false;
    _fill_first = false;
    _io_thread_alive = false;
    _last_success_ms = 0;
    _failure = ShowFileParser::Failure::NONE;
}

// skip_header - discard magic + header + segments so the source is
// positioned at the first event frame.
bool ShowStreamReader::skip_header()
{
    uint32_t to_skip = 4 + ShowFile::HEADER_SIZE + (uint32_t)segment_count() * 12;
    uint8_t skip_buf[32];
    while (to_skip > 0) {
        const uint32_t n = to_skip < sizeof(skip_buf) ? to_skip : sizeof(skip_buf);
        const int32_t skip_got = _src->read(skip_buf, n);
        if (skip_got <= 0) {
            return false;
        }
        to_skip -= (uint32_t)skip_got;
    }
    return true;
}

// load_and_verify - verify the file in two passes, as one operation:
//   pass 1: parse the header and CRC the whole file (sequential read);
//   pass 2: rewind to the head, skip the header and fill block 0 by
//           parsing real frames into the buffer.
// A parse failure while filling fails the verification, so a successful
// load means the file is valid AND the first playback block is ready.
bool ShowStreamReader::load_and_verify(BlockSource &src)
{
    _src = &src;
    if (!src.open()) {
        _failure = ShowFileParser::Failure::TRUNCATED;
        return false;
    }

    uint8_t hdr[32];
    int32_t got = src.read(hdr, sizeof(hdr));
    if (got < 32) {
        _failure = ShowFileParser::Failure::TRUNCATED;
        src.close();
        return false;
    }
    if (!_parser.parse_header(hdr, got)) {
        _failure = _parser.failure();
        src.close();
        return false;
    }

    // ---- pass 1: CRC the whole file in chunks ----
    uint8_t buf[SHOW_STREAM_IO_BUF];
    uint32_t crc = ShowFileParser::crc_accumulate(hdr, got, 0, 0);
    uint32_t offset = got;
    for (;;) {
        got = src.read(buf, sizeof(buf));
        if (got < 0) {
            _failure = ShowFileParser::Failure::TRUNCATED;
            src.close();
            return false;
        }
        if (got == 0) {
            break;
        }
        crc = ShowFileParser::crc_accumulate(buf, got, offset, crc);
        offset += got;
    }

    if (offset < 4 + ShowFile::CRC_OFFSET + 4) {
        _failure = ShowFileParser::Failure::TRUNCATED;
        src.close();
        return false;
    }
    uint32_t stored_crc;
    memcpy(&stored_crc, hdr + 4 + ShowFile::CRC_OFFSET, 4);
    if (stored_crc != crc) {
        _failure = ShowFileParser::Failure::BAD_CRC;
        src.close();
        return false;
    }

    // ---- pass 2: rewind to the head and fill block 0 ----
    if (!src.rewind()) {
        _failure = ShowFileParser::Failure::TRUNCATED;
        src.close();
        return false;
    }
    if (!skip_header()) {
        _failure = ShowFileParser::Failure::TRUNCATED;
        src.close();
        return false;
    }
    _io_len = 0;
    _io_pos = 0;
    const int8_t r = refill_block(0);
    if (r < 0 || _block_count[0] == 0) {
        // a frame could not be parsed: the file is not usable
        _failure = ShowFileParser::Failure::UNKNOWN_EVENT;
        src.close();
        return false;
    }
    _loaded = true;
    return true;
}

// start - begin playback.  In threaded mode the IO thread fills block 0
// and all later blocks; in synchronous mode (unit tests) block 0 is
// refilled here.
bool ShowStreamReader::start()
{
    if (!_loaded) {
        return false;
    }
    _have_anchor = false;
    _pending_light_count = 0;
    _block_count[0] = _block_count[1] = 0;
    _light_count[0] = _light_count[1] = 0;
    _eof = false;
    _refill_pending = true;
    _io_len = 0;
    _io_pos = 0;

    if (_threaded) {
        _io_stop = false;
        _io_busy = false;
        _fill_first = true;
        _last_success_ms = AP_HAL::millis();   // startup grace period
        _started = true;
        if (!hal.scheduler->thread_create(
                FUNCTOR_BIND_MEMBER(&ShowStreamReader::io_thread_main, void),
                "show-io", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
            _started = false;
            return false;
        }
        return true;
    }

    if (!_src->rewind() || !skip_header()) {
        return false;
    }
    _io_len = 0;
    _io_pos = 0;
    const int8_t r = refill_block(0);
    if (r < 0) {
        return false;
    }
    _started = true;
    return true;
}

// io_thread_main - the IO thread loop: poll for refill requests and read
// the next block off the file.  File IO happens here, off the main thread,
// which is the only place ArduPilot permits file access while armed.
void ShowStreamReader::io_thread_main()
{
    {
        WITH_SEMAPHORE(_sem);
        _io_thread_alive = true;
    }
    // all file operations live on this thread: reposition the file to
    // the event stream start before the first refill
    _src->rewind();
    skip_header();
    while (!_io_stop) {
        hal.scheduler->delay(5);
        uint8_t target = 255;
        {
            WITH_SEMAPHORE(_sem);
            if (_io_stop) {
                break;
            }
            const uint8_t want = _fill_first ? _playing : _refilling;
            if (_refill_pending && !_io_busy &&
                _block_count[want] == 0 && !_eof) {
                _io_busy = true;
                target = want;
            }
        }
        if (target == 255) {
            continue;
        }
        const uint32_t now_ms = AP_HAL::millis();
        if (_last_refill_ms != 0 && now_ms - _last_refill_ms < 100) {
            // rate limit retries after failures
            hal.scheduler->delay(100 - (now_ms - _last_refill_ms));
            if (_io_stop) {
                break;
            }
        }
        _last_refill_ms = AP_HAL::millis();
        const int8_t r = refill_block(target);   // file IO, off the lock
        {
            WITH_SEMAPHORE(_sem);
            _io_busy = false;
            _refill_pending = false;
            if (r < 0) {
                _block_count[target] = 0;   // retried on the next request
            } else if (_block_count[target] > 0) {
                _last_success_ms = AP_HAL::millis();
            }
            if (_fill_first && target == _playing) {
                _fill_first = false;
            }
        }
    }
    {
        WITH_SEMAPHORE(_sem);
        _io_thread_alive = false;
    }
}

// refill_block - parse events into 'buf' until it holds
// SHOW_STREAM_WINDOW_FRAMES position frames, EOF or error.
// Any unconsumed bytes already read from the source stay in the
// persistent _io buffer for the next refill.
int8_t ShowStreamReader::refill_block(uint8_t buf)
{
    _block_count[buf] = 0;
    _light_count[buf] = 0;

    for (;;) {
        // parse complete frames out of the buffered bytes
        bool any = true;
        while (any && _block_count[buf] < SHOW_STREAM_WINDOW_FRAMES) {
            any = false;
            const uint8_t *p = _io + _io_pos;
            const uint8_t *end = _io + _io_len;
            uint8_t type;
            uint32_t t_ms;
            ShowFile::Keyframe kf;
            ShowFile::LightEvent le;
            if (_parser.parse_event(p, end, type, t_ms, kf, le)) {
                _io_pos = p - _io;
                any = true;
                if (type == ShowFile::EVENT_POSITION) {
                    _block[buf][_block_count[buf]] = kf;
                    _block_count[buf]++;
                } else if (type == ShowFile::EVENT_LIGHT) {
                    if (_light_count[buf] < SHOW_STREAM_WINDOW_FRAMES) {
                        _light[buf][_light_count[buf]] = le;
                        _light_count[buf]++;
                    }
                }
            } else if (_io_len - _io_pos >= 25) {
                // more than a full (largest) frame is buffered but the
                // frame could not be parsed: unknown event type or
                // corrupt frame (a shorter tail is a partial frame that
                // spans the read boundary and will continue next read)
                return -2;
            }
        }
        if (_block_count[buf] >= SHOW_STREAM_WINDOW_FRAMES) {
            return 1;
        }
        // consumed everything buffered and still need more frames: read
        if (_io_pos >= _io_len) {
            _io_len = 0;
            _io_pos = 0;
        } else {
            // partial frame at the tail: move it to the front
            const uint32_t leftover = _io_len - _io_pos;
            memmove(_io, _io + _io_pos, leftover);
            _io_len = leftover;
            _io_pos = 0;
        }
        const int32_t got = _src->read(_io + _io_len, sizeof(_io) - _io_len);
        if (got < 0) {
            return -1;
        }
        if (got == 0) {
            _eof = true;
            return _block_count[buf] > 0 ? 1 : 0;   // partial final block
        }
        _io_len += got;
    }
}

// update - advance the window and keep the standby buffer full
void ShowStreamReader::update(uint32_t t_ms)
{
    if (!_started) {
        return;
    }
    if (_threaded) {
        WITH_SEMAPHORE(_sem);
        if (_block_count[_refilling] == 0 && !_eof) {
            _refill_pending = true;
        }
        switch_buffers_locked(t_ms);
        return;
    }

    // synchronous path (unit tests): refill inline
    if (_refill_pending && _block_count[_refilling] == 0 && !_eof) {
        if (_last_refill_ms == 0 || t_ms - _last_refill_ms >= 100) {
            _last_refill_ms = t_ms;
            const int8_t r = refill_block(_refilling);
            if (r < 0) {
                _block_count[_refilling] = 0;
            } else if (_block_count[_refilling] > 0) {
                _last_success_ms = t_ms;
            }
        }
    }
    switch_buffers_locked(t_ms);
}

// switch_buffers_locked - swap windows when the playing window has been
// consumed and the standby block is ready.  Caller holds _sem.
void ShowStreamReader::switch_buffers_locked(uint32_t t_ms)
{
    const uint16_t pc = _block_count[_playing];
    if (pc > 0 && t_ms >= _block[_playing][pc - 1].t_ms &&
        _block_count[_refilling] > 0 && !_io_busy) {
        // keep the last frame of the old window as interpolation anchor
        _anchor = _block[_playing][pc - 1];
        _have_anchor = true;
        // carry light frames that have not fired yet across the switch
        _pending_light_count = 0;
        for (uint16_t i = 0; i < _light_count[_playing]; i++) {
            if (_light[_playing][i].t_ms > t_ms) {
                if (_pending_light_count < SHOW_STREAM_WINDOW_FRAMES) {
                    _pending_light[_pending_light_count++] = _light[_playing][i];
                }
            }
        }
        _playing = _refilling;
        _refilling = 1 - _playing;
        _block_count[_refilling] = 0;
        _light_count[_refilling] = 0;
        _refill_pending = true;
    }
}

// position_view - playing window as a contiguous array; when an anchor
// from the previous block is present it is prepended.
bool ShowStreamReader::position_view(const ShowFile::Keyframe *&frames,
                                     uint16_t &count) const
{
    if (!_started || _block_count[_playing] == 0) {
        return false;
    }
    uint16_t n = 0;
    if (_have_anchor) {
        _view[n++] = _anchor;
    }
    for (uint16_t i = 0; i < _block_count[_playing]; i++) {
        _view[n++] = _block[_playing][i];
    }
    frames = _view;
    count = n;
    return true;
}

bool ShowStreamReader::light_view(const ShowFile::LightEvent *&frames,
                                  uint16_t &count) const
{
    if (!_started) {
        return false;
    }
    uint16_t n = 0;
    for (uint16_t i = 0; i < _pending_light_count; i++) {
        _light_view[n++] = _pending_light[i];
    }
    for (uint16_t i = 0; i < _light_count[_playing]; i++) {
        _light_view[n++] = _light[_playing][i];
    }
    frames = _light_view;
    count = n;
    return count > 0;
}

// ready_to_play - first block available?
bool ShowStreamReader::ready_to_play() const
{
    WITH_SEMAPHORE(_sem);
    if (!_started) {
        return false;
    }
    return _block_count[_playing] > 0 || _block_count[_refilling] > 0 || _eof;
}

// can_evaluate - can the reader supply a position frame at time t?
bool ShowStreamReader::can_evaluate(uint32_t t_ms) const
{
    WITH_SEMAPHORE(_sem);
    if (!_started) {
        return false;
    }
    if (_eof) {
        // no more data; the show end is judged by duration upstream and
        // the last frame is clampable
        return true;
    }
    const uint16_t pc = _block_count[_playing];
    if (pc > 0 && t_ms <= _block[_playing][pc - 1].t_ms) {
        return true;   // covered by the playing window
    }
    if (_block_count[_refilling] > 0) {
        return true;   // standby block ready
    }
    // nothing playable right now: give a grace period of 2s since the
    // last successful refill (covers the startup fill and transient IO
    // failures); past that the stream is considered exhausted.  The
    // synchronous path (unit tests) clocks on the simulated show time,
    // the threaded path on wall time.
    const uint32_t now_ms = _threaded ? AP_HAL::millis() : t_ms;
    return (now_ms - _last_success_ms) <= 2000;
}

uint32_t ShowStreamReader::window_end_ms() const
{
    if (_block_count[_playing] == 0) {
        return 0;
    }
    return _block[_playing][_block_count[_playing] - 1].t_ms;
}

void ShowStreamReader::close()
{
    {
        WITH_SEMAPHORE(_sem);
        _io_stop = true;
    }
    if (_threaded) {
        // wait for the IO thread to observe the stop flag and exit
        for (uint8_t i = 0; i < 100; i++) {
            bool alive;
            {
                WITH_SEMAPHORE(_sem);
                alive = _io_thread_alive;
            }
            if (!alive) {
                break;
            }
            hal.scheduler->delay(5);
        }
    }
    if (_src != nullptr) {
        _src->close();
    }
    _src = nullptr;
    _started = false;
    _loaded = false;
    _io_stop = false;
}

// test_wait_idle - block until a pending refill finishes (unit tests)
void ShowStreamReader::test_wait_idle()
{
    for (uint8_t i = 0; i < 200; i++) {
        bool busy;
        bool pending;
        {
            WITH_SEMAPHORE(_sem);
            busy = _io_busy;
            pending = _refill_pending;
        }
        if (!busy && !pending) {
            return;
        }
        hal.scheduler->delay(5);
    }
}
