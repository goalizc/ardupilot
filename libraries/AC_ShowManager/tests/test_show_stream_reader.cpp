#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AC_ShowManager/ShowStreamReader.h>

#include <vector>
#include <string.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ------------------------------------------------------------------ //
// byte writers (v2 file construction helpers, mirrored from the parser
// test so this file stays self-contained)
static void put_u8(std::vector<uint8_t> &b, uint8_t v) { b.push_back(v); }
static void put_u16(std::vector<uint8_t> &b, uint16_t v)
{
    b.push_back(v & 0xff);
    b.push_back((v >> 8) & 0xff);
}
static void put_u32(std::vector<uint8_t> &b, uint32_t v)
{
    for (int i = 0; i < 4; i++) {
        b.push_back((v >> (8 * i)) & 0xff);
    }
}
static void put_i16(std::vector<uint8_t> &b, int16_t v) { put_u16(b, (uint16_t)v); }
static void put_i32(std::vector<uint8_t> &b, int32_t v) { put_u32(b, (uint32_t)v); }

static std::vector<uint8_t> build_v2(uint32_t duration_ms)
{
    std::vector<uint8_t> b;
    b.insert(b.end(), {'S', 'H', 'O', 'W'});
    put_u8(b, ShowFile::FORMAT_VERSION);
    put_u8(b, 0);
    put_u16(b, 7);
    put_u32(b, duration_ms);
    put_u32(b, 0);
    put_u32(b, 0);
    put_u32(b, 0);
    put_u8(b, 0);
    b.insert(b.end(), 3, 0);
    b.insert(b.end(), 4, 0);
    return b;
}

static void put_position(std::vector<uint8_t> &b, uint32_t t_ms, int32_t x)
{
    put_u8(b, ShowFile::EVENT_POSITION);
    put_u32(b, t_ms);
    put_i32(b, x);
    put_i32(b, 0);
    put_i32(b, -5000);
    put_i16(b, 0);
    put_i16(b, 0);
    put_i16(b, 0);
    put_i16(b, 0);
}

static void put_light(std::vector<uint8_t> &b, uint32_t t_ms, uint8_t r)
{
    put_u8(b, ShowFile::EVENT_LIGHT);
    put_u32(b, t_ms);
    put_u8(b, 0);
    put_u8(b, r);
    put_u8(b, 0);
    put_u8(b, 0);
}

static void finalise(std::vector<uint8_t> &b, uint32_t n_pos, uint32_t n_light)
{
    const uint32_t n_events = n_pos + n_light;
    for (int i = 0; i < 4; i++) {
        b[4 + 8 + i] = (n_events >> (8 * i)) & 0xff;
        b[4 + 12 + i] = (n_pos >> (8 * i)) & 0xff;
        b[4 + 16 + i] = (n_light >> (8 * i)) & 0xff;
    }
    uint32_t crc = ShowFileParser::crc_accumulate(b.data(), b.size(), 0, 0);
    for (int i = 0; i < 4; i++) {
        b[4 + ShowFile::CRC_OFFSET + i] = (crc >> (8 * i)) & 0xff;
    }
}

// n position frames at 10ms spacing plus one light frame near the end
static std::vector<uint8_t> make_file(uint32_t n_pos, uint32_t duration_ms)
{
    std::vector<uint8_t> b = build_v2(duration_ms);
    for (uint32_t i = 0; i < n_pos; i++) {
        put_position(b, i * 10, (int32_t)(i * 100));
    }
    put_light(b, duration_ms - 1, 255);
    finalise(b, n_pos, 1);
    return b;
}

// ------------------------------------------------------------------ //
// in-memory BlockSource with failure injection
class MemSource : public ShowStreamReader::BlockSource {
public:
    MemSource(const std::vector<uint8_t> &data) : _data(data) {}

    // fail the next 'n' read() calls, then recover
    void fail_next_reads(uint32_t n) { _fail_remaining = n; }
    // fail every read() from now on
    void fail_forever() { _fail_forever = true; }
    // simulate the storage being cut short mid-playback: reads past
    // 'limit' return EOF
    void truncate(uint32_t limit) { _limit = limit; }
    uint32_t read_calls() const { return _read_calls; }
    uint32_t pos() const { return _pos; }

    bool open() override { _pos = 0; return true; }
    int32_t read(void *buf, uint32_t len) override
    {
        _read_calls++;
        if (_fail_forever || _fail_remaining > 0) {
            if (_fail_remaining > 0) {
                _fail_remaining--;
            }
            return -1;
        }
        const uint32_t end = _limit < _data.size() ? _limit : _data.size();
        const uint32_t avail = end - _pos;
        const uint32_t n = avail < len ? avail : len;
        if (n == 0) {
            return 0;    // EOF
        }
        memcpy(buf, _data.data() + _pos, n);
        _pos += n;
        return (int32_t)n;
    }
    bool rewind() override { _pos = 0; return true; }
    void close() override {}

private:
    const std::vector<uint8_t> &_data;
    uint32_t _pos = 0;
    uint32_t _limit = 0xffffffffU;
    uint32_t _fail_remaining = 0;
    bool _fail_forever = false;
    uint32_t _read_calls = 0;
};

// ------------------------------------------------------------------ //

TEST(ShowStreamReader, VerifyGoodFile)
{
    std::vector<uint8_t> f = make_file(10, 1000);
    MemSource src(f);
    ShowStreamReader r;
    EXPECT_TRUE(r.load_and_verify(src));
    EXPECT_EQ(r.keyframe_count(), 10U);
    EXPECT_EQ(r.light_count(), 1U);
    EXPECT_TRUE(r.loaded());
    r.close();
}

TEST(ShowStreamReader, VerifyBadCrc)
{
    std::vector<uint8_t> f = make_file(10, 1000);
    f[f.size() - 8] ^= 0x55;    // corrupt a payload byte
    MemSource src(f);
    ShowStreamReader r;
    EXPECT_FALSE(r.load_and_verify(src));
    EXPECT_EQ(r.failure(), ShowFileParser::Failure::BAD_CRC);
    r.close();
}

// multi-block playback: window must advance and stay evaluable
TEST(ShowStreamReader, PlaysAcrossBlocks)
{
    // ~3 blocks of position frames
    const uint32_t n = 3 * SHOW_STREAM_WINDOW_FRAMES + 10;
    const uint32_t dur = n * 10;
    std::vector<uint8_t> f = make_file(n, dur);
    MemSource src(f);
    ShowStreamReader r;
    ASSERT_TRUE(r.load_and_verify(src));
    ASSERT_TRUE(r.start());
    const ShowFile::Keyframe *frames = nullptr;
    uint16_t count = 0;
    uint8_t last_playing = r.test_playing();
    bool saw_anchor = false;
    uint32_t t = 0;
    while (t < dur + 20) {
        r.update(t);
        ASSERT_TRUE(r.can_evaluate(t));
        ASSERT_TRUE(r.position_view(frames, count));
        ASSERT_GT(count, 0U);
        // window must bracket t (modulo EOF clamp)
        ASSERT_LE(frames[0].t_ms, t);
        if (r.test_playing() != last_playing) {
            saw_anchor = true;
            last_playing = r.test_playing();
        }
        t += 100;   // 10Hz control steps
    }
    EXPECT_TRUE(saw_anchor);
    EXPECT_TRUE(r.eof());
    r.close();
}

// light frames near a block boundary survive a window switch
TEST(ShowStreamReader, CarriesLightAcrossBlocks)
{
    const uint32_t n = SHOW_STREAM_WINDOW_FRAMES + 5;
    const uint32_t dur = n * 10;
    std::vector<uint8_t> f = make_file(n, dur);   // light at dur-1 ms
    MemSource src(f);
    ShowStreamReader r;
    ASSERT_TRUE(r.load_and_verify(src));
    ASSERT_TRUE(r.start());

    const ShowFile::LightEvent *lights = nullptr;
    uint16_t count = 0;
    // push past the first block so a switch happens
    for (uint32_t t = 0; t <= (uint32_t)SHOW_STREAM_WINDOW_FRAMES * 10 + 100; t += 50) {
        r.update(t);
    }
    ASSERT_TRUE(r.light_view(lights, count));
    // the carried-over light (t = dur-1) must still be visible
    bool found = false;
    for (uint16_t i = 0; i < count; i++) {
        if (lights[i].t_ms == dur - 1) {
            found = true;
        }
    }
    EXPECT_TRUE(found);
    r.close();
}

// transient read failures must not stop playback (retry recovers)
TEST(ShowStreamReader, RecoversFromTransientReadFailure)
{
    const uint32_t n = 2 * SHOW_STREAM_WINDOW_FRAMES;
    const uint32_t dur = n * 10;
    std::vector<uint8_t> f = make_file(n, dur);
    MemSource src(f);
    ShowStreamReader r;
    ASSERT_TRUE(r.load_and_verify(src));
    ASSERT_TRUE(r.start());

    // after start, fail the refill reads a few times
    const uint32_t calls_before = src.read_calls();
    src.fail_next_reads(3);
    const ShowFile::Keyframe *frames = nullptr;
    uint16_t count = 0;
    uint32_t t = 0;
    bool ever_exhausted = false;
    while (t < dur + 10) {
        r.update(t);
        if (!r.can_evaluate(t)) {
            ever_exhausted = true;
            break;
        }
        ASSERT_TRUE(r.position_view(frames, count));
        t += 100;
    }
    EXPECT_FALSE(ever_exhausted);
    EXPECT_GT(src.read_calls(), calls_before + 3);
    EXPECT_TRUE(r.eof());
    r.close();
}

// persistent read failure after the first block must exhaust
TEST(ShowStreamReader, ExhaustsOnPersistentFailure)
{
    const uint32_t n = 2 * SHOW_STREAM_WINDOW_FRAMES;
    const uint32_t dur = n * 10;
    std::vector<uint8_t> f = make_file(n, dur);
    MemSource src(f);
    ShowStreamReader r;
    ASSERT_TRUE(r.load_and_verify(src));
    ASSERT_TRUE(r.start());
    src.fail_forever();

    const ShowFile::Keyframe *frames = nullptr;
    uint16_t count = 0;
    bool exhausted = false;
    // walk past the first window; the refill never succeeds
    for (uint32_t t = 0; t < 5 * SHOW_STREAM_WINDOW_FRAMES * 10; t += 100) {
        r.update(t);
        if (!r.can_evaluate(t)) {
            exhausted = true;
            break;
        }
        ASSERT_TRUE(r.position_view(frames, count));
    }
    EXPECT_TRUE(exhausted);
    r.close();
}

// EOF that arrives before the declared frame count (e.g. the storage was
// truncated mid-show on a frame boundary) must exhaust after the grace
// period, not clamp: the vehicle must not keep hovering on the last
// frame until the choreography duration has run out.
TEST(ShowStreamReader, ExhaustsOnPrematureEof)
{
    const uint32_t n = 3 * SHOW_STREAM_WINDOW_FRAMES + 10;   // ~3 blocks
    const uint32_t dur = n * 10;
    std::vector<uint8_t> f = make_file(n, dur);
    MemSource src(f);
    ShowStreamReader r;
    ASSERT_TRUE(r.load_and_verify(src));
    ASSERT_TRUE(r.start());
    // cut the file cleanly after 250 position frames (each 25 bytes, after
    // the 32-byte magic+header+crc prefix): reads past that return EOF on
    // a frame boundary, so the reader sees a clean early EOF
    const uint32_t kept = 250;
    src.truncate(32U + kept * 25U);

    const ShowFile::Keyframe *frames = nullptr;
    uint16_t count = 0;
    bool exhausted = false;
    // walk well past the truncated data + the 2s grace period
    for (uint32_t t = 0; t < dur + 6000; t += 100) {
        r.update(t);
        if (!r.can_evaluate(t)) {
            exhausted = true;
            break;
        }
        ASSERT_TRUE(r.position_view(frames, count));
    }
    EXPECT_TRUE(exhausted);
    r.close();
}


// threaded mode: refills happen on the IO thread; playback must start as
// soon as the first block is ready and run to EOF without exhaustion
TEST(ShowStreamReader, ThreadedPlaysToEof)
{
    const uint32_t n = 3 * SHOW_STREAM_WINDOW_FRAMES;
    const uint32_t dur = n * 10;
    std::vector<uint8_t> f = make_file(n, dur);
    MemSource src(f);
    ShowStreamReader r;
    r.enable_io_thread(true);
    ASSERT_TRUE(r.load_and_verify(src));
    ASSERT_TRUE(r.start());

    // wait for the first block (IO thread) then play through
    const ShowFile::Keyframe *frames = nullptr;
    uint16_t count = 0;
    uint32_t t = 0;
    bool saw_data = false;
    bool exhausted = false;
    while (t < dur + 50) {
        r.update(t);
        r.test_wait_idle();
        if (r.ready_to_play()) {
            saw_data = true;
        }
        if (!r.can_evaluate(t)) {
            exhausted = true;
            break;
        }
        if (r.position_view(frames, count)) {
            ASSERT_GT(count, 0U);
        }
        t += 100;
    }
    EXPECT_TRUE(saw_data);
    EXPECT_FALSE(exhausted);
    EXPECT_TRUE(r.eof());
    r.close();
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
