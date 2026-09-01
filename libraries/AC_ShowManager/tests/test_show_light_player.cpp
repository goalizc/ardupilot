#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AC_ShowManager/ShowFile.h>
#include <AC_ShowManager/ShowLightPlayer.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ShowLightPlayer, NoTrackReturnsOff)
{
    ShowLightPlayer p;
    uint8_t r, g, b;
    EXPECT_FALSE(p.evaluate(0, r, g, b));
}

TEST(ShowLightPlayer, BeforeFirstEventIsOff)
{
    ShowLightPlayer p;
    ShowFile::LightEvent ev{1000, 0, 255, 0, 0};
    p.set_track(&ev, 1);
    uint8_t r, g, b;
    EXPECT_FALSE(p.evaluate(0, r, g, b));
    EXPECT_FALSE(p.evaluate(999, r, g, b));
}

TEST(ShowLightPlayer, HoldsColorBetweenEvents)
{
    ShowLightPlayer p;
    ShowFile::LightEvent ev[] = {
        {1000, 0, 255, 0, 0},
        {5000, 0, 0, 255, 0},
    };
    p.set_track(ev, 2);
    uint8_t r, g, b;
    // at the first event: red
    EXPECT_TRUE(p.evaluate(1000, r, g, b));
    EXPECT_EQ(r, 255); EXPECT_EQ(g, 0); EXPECT_EQ(b, 0);
    // between events: still red (step hold)
    EXPECT_TRUE(p.evaluate(3000, r, g, b));
    EXPECT_EQ(r, 255); EXPECT_EQ(g, 0); EXPECT_EQ(b, 0);
    // at the second event: green
    EXPECT_TRUE(p.evaluate(5000, r, g, b));
    EXPECT_EQ(r, 0); EXPECT_EQ(g, 255); EXPECT_EQ(b, 0);
    // after the last event: still green (no black-out after end)
    EXPECT_TRUE(p.evaluate(9000, r, g, b));
    EXPECT_EQ(r, 0); EXPECT_EQ(g, 255); EXPECT_EQ(b, 0);
}

TEST(ShowLightPlayer, IgnoresPerPixelEvents)
{
    ShowLightPlayer p;
    ShowFile::LightEvent ev[] = {
        {1000, 0, 255, 0, 0},
        {2000, 1, 0, 0, 255},   // per-pixel: ignored by the overall-colour player
        {3000, 0, 0, 255, 0},
    };
    p.set_track(ev, 3);
    uint8_t r, g, b;
    EXPECT_TRUE(p.evaluate(1000, r, g, b));
    EXPECT_EQ(r, 255);
    // 2000..2999: the per-pixel event must not change the overall colour
    EXPECT_TRUE(p.evaluate(2500, r, g, b));
    EXPECT_EQ(r, 255); EXPECT_EQ(g, 0); EXPECT_EQ(b, 0);
    EXPECT_TRUE(p.evaluate(3000, r, g, b));
    EXPECT_EQ(g, 255);
}

TEST(ShowLightPlayer, MultipleOverallEvents)
{
    ShowLightPlayer p;
    ShowFile::LightEvent ev[] = {
        {0, 0, 255, 0, 0},
        {100, 0, 0, 255, 0},
        {200, 0, 0, 0, 255},
        {300, 0, 255, 255, 255},
    };
    p.set_track(ev, 4);
    uint8_t r, g, b;
    EXPECT_TRUE(p.evaluate(0, r, g, b));
    EXPECT_EQ(r, 255); EXPECT_EQ(g, 0); EXPECT_EQ(b, 0);
    EXPECT_TRUE(p.evaluate(199, r, g, b));
    EXPECT_EQ(r, 0); EXPECT_EQ(g, 255); EXPECT_EQ(b, 0);
    EXPECT_TRUE(p.evaluate(200, r, g, b));
    EXPECT_EQ(r, 0); EXPECT_EQ(g, 0); EXPECT_EQ(b, 255);
    EXPECT_TRUE(p.evaluate(400, r, g, b));
    EXPECT_EQ(r, 255); EXPECT_EQ(g, 255); EXPECT_EQ(b, 255);
}

// 注：预期行为"末事件后保持末色"（不黑掉）与 ShowPlayer 末帧钳制一致。
// 若实现决策改为"末事件后黑"，此处断言需同步修改（见任务 2 步骤 3 注释）。

AP_GTEST_PANIC()
AP_GTEST_MAIN()
