#pragma once

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>

#include "ShowFile.h"
#include "ShowFileParser.h"

/*
  ShowStreamReader - windowed, dual-buffered reader for the v2
  event-stream show file.

  The whole file is verified (CRC) before flight, then played by
  reading one block at a time sequentially from storage.  A block is up
  to SHOW_STREAM_WINDOW_FRAMES position frames plus the light frames
  that fall inside that span.  Two buffers alternate in a pipeline:
  while one buffer is being played the other is being refilled, so
  playback never waits on IO unless both buffers are exhausted.

  Failure model:
    - a failed refill is retried (rate limited) while the playing
      window still covers the show time; playback is unaffected.
    - can_evaluate(t) reports whether the next frame at time t is
      available; the caller (ModeShow) aborts the show when it is not
      (i.e. the position window is truly exhausted and not at EOF).
    - light frames are best effort: if unreadable the last colour
      simply holds (no safety impact).

  Pure logic apart from the injected BlockSource, so it is unit-tested
  with gtest using an in-memory source that can inject read failures.
*/

#ifndef SHOW_STREAM_WINDOW_FRAMES
#define SHOW_STREAM_WINDOW_FRAMES 128
#endif

// size of the byte staging buffer used while parsing frames from storage
#ifndef SHOW_STREAM_IO_BUF
#define SHOW_STREAM_IO_BUF 512
#endif

class ShowStreamReader {
public:

    // sequential byte source abstraction (real FS or test double)
    class BlockSource {
    public:
        virtual ~BlockSource() {}
        // open the file and position at byte 0; false on error
        virtual bool open() = 0;
        // read up to len bytes at the current position; returns bytes
        // read (>0), 0 on EOF, -1 on error
        virtual int32_t read(void *buf, uint32_t len) = 0;
        // reposition to byte 0 (after verification, before playback)
        virtual bool rewind() = 0;
        virtual void close() = 0;
    };

    ShowStreamReader();

    // run the refills on a dedicated IO thread (required in flight:
    // ArduPilot forbids file IO on the main thread while armed).  When
    // disabled (default) refills happen synchronously inside update(),
    // which keeps the logic unit-testable without threads.
    void enable_io_thread(bool enable) { _threaded = enable; }
    bool io_thread_enabled() const { return _threaded; }

    // verify the whole file (header + CRC).  Leaves the source open and
    // rewound to byte 0 on success; closes it on failure.
    bool load_and_verify(BlockSource &src);

    // start streaming: read block 0; the standby refill starts on the
    // first update().
    bool start();

    // 50Hz: switch windows when consumed and keep the standby full
    // (rate-limited retries on IO failure).
    void update(uint32_t t_ms);

    // assemble the playing window (previous block's last position frame
    // as anchor, then this block's frames) into a contiguous view.
    bool position_view(const ShowFile::Keyframe *&frames, uint16_t &count) const;
    // light frames of the playing block (best effort, no anchor)
    bool light_view(const ShowFile::LightEvent *&frames, uint16_t &count) const;

    // true once the first block is playable (started and at least one
    // position frame is buffered, or EOF).  ModeShow requires this before
    // beginning the performance.
    bool ready_to_play() const;

    // true while a position frame at time t is (or will be) available:
    // the playing window covers t, or the standby block is ready, or the
    // source is at EOF (end of show is judged by duration upstream).
    bool can_evaluate(uint32_t t_ms) const;

    bool started() const { return _started; }
    bool loaded() const { return _loaded; }
    bool eof() const { return _eof; }
    uint32_t window_end_ms() const;

    ShowFileParser::Failure failure() const { return _failure; }
    uint16_t drone_id() const { return _parser.drone_id(); }
    uint32_t duration_ms() const { return _parser.duration_ms(); }
    uint32_t keyframe_count() const { return _parser.keyframe_count(); }
    uint32_t light_count() const { return _parser.light_count(); }
    uint8_t segment_count() const { return _parser.segment_count(); }
    uint32_t event_count() const { return _parser.event_count(); }

    void close();

    // unit tests: wait until the IO thread has finished a pending refill
    void test_wait_idle();

    // unit-test accessors
    uint8_t test_playing() const { return _playing; }
    uint16_t test_block_count(uint8_t buf) const { return _block_count[buf]; }
    uint16_t test_light_count(uint8_t buf) const { return _light_count[buf]; }
    bool test_anchor() const { return _have_anchor; }
    bool test_busy() const { return _io_busy; }
    bool test_fill_first() const { return _fill_first; }
    bool test_refill_pending() const { return _refill_pending; }

private:

    // parse events from the source into buffer 'buf' until it holds
    // SHOW_STREAM_WINDOW_FRAMES position frames, EOF or error.
    // returns 1 block complete, 0 EOF, -1 IO error.
    int8_t refill_block(uint8_t buf);

    void io_thread_main();
    void maybe_refill_locked();      // caller holds _sem
    void switch_buffers_locked(uint32_t t_ms);   // caller holds _sem
    // read past magic + header + segments to the event stream start
    bool skip_header();

    ShowFileParser _parser;
    BlockSource *_src;
    mutable HAL_Semaphore _sem;
    bool _threaded;          // refill on an IO thread
    bool _io_stop;           // stop request for the IO thread
    bool _io_busy;           // IO thread is currently refilling
    bool _fill_first;        // the first refill fills the playing slot
    bool _io_thread_alive;   // IO thread has not exited yet

    // per-buffer block data
    ShowFile::Keyframe _block[2][SHOW_STREAM_WINDOW_FRAMES];
    uint16_t _block_count[2];
    ShowFile::LightEvent _light[2][SHOW_STREAM_WINDOW_FRAMES];
    uint16_t _light_count[2];

    // interpolation anchor: last position frame of the previous block
    // (kept across a window switch so cross-block interpolation works)
    ShowFile::Keyframe _anchor;
    bool _have_anchor;

    // assembled contiguous views for position_view()/light_view()
    mutable ShowFile::Keyframe _view[SHOW_STREAM_WINDOW_FRAMES + 1];
    mutable ShowFile::LightEvent _light_view[2 * SHOW_STREAM_WINDOW_FRAMES];
    // light frames from the previous window that had not fired yet when
    // the window switched (t > current show time); carried over
    ShowFile::LightEvent _pending_light[SHOW_STREAM_WINDOW_FRAMES];
    uint16_t _pending_light_count;

    uint8_t _playing;
    uint8_t _refilling;
    bool _refill_pending;      // standby buffer needs (re)filling
    bool _started;
    bool _eof;
    bool _loaded;

    uint32_t _last_refill_ms;
    uint32_t _last_success_ms;    // last successful refill (main-loop ms)
    ShowFileParser::Failure _failure;
    // _playing/_refilling/_eof etc are read/written under _sem when
    // threaded; the IO thread only touches the standby buffer slot.

    // persistent IO staging buffer: bytes read from the source that have
    // not been consumed by the current block are carried across refills
    uint8_t _io[SHOW_STREAM_IO_BUF];
    uint32_t _io_len;
    uint32_t _io_pos;
};
