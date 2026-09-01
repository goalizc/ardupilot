#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AC_ShowManager/ShowPlayer.h>

#include <AC_ShowManager/AC_ShowManager.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// track: t=0 pos(0,0,0) vel(0,0,0) yaw 0;
//        t=10000 pos(10,0,-5) vel(1,0,0) yaw 0;
//        t=20000 pos(20,10,-10) vel(1,1,0) yaw 9000
static ShowFile::Keyframe track[3];

static void init_track()
{
    track[0] = {0,     0, 0, 0,     0, 0, 0, 0};
    track[1] = {10000, 10000, 0, -5000, 1000, 0, 0, 0};
    track[2] = {20000, 20000, 10000, -10000, 1000, 1000, 0, 9000};
}

TEST(ShowPlayer, ClampsBeforeFirstKeyframe)
{
    init_track();
    ShowPlayer player;
    player.set_track(track, 3);
    ShowFile::Keyframe out;
    ASSERT_TRUE(player.evaluate(0, out));
    EXPECT_EQ(out.pos_x_mm, 0);
    EXPECT_EQ(out.pos_y_mm, 0);
}

TEST(ShowPlayer, InterpolatesBetweenKeyframes)
{
    init_track();
    ShowPlayer player;
    player.set_track(track, 3);
    ShowFile::Keyframe out;
    ASSERT_TRUE(player.evaluate(5000, out));  // halfway between kf0 and kf1
    EXPECT_EQ(out.pos_x_mm, 5000);
    EXPECT_EQ(out.vel_x_mms, 500);           // kf0 vel 0, kf1 vel 1000
    EXPECT_EQ(out.pos_y_mm, 0);
}

TEST(ShowPlayer, InterpolatesVerticalAndYaw)
{
    init_track();
    ShowPlayer player;
    player.set_track(track, 3);
    ShowFile::Keyframe out;
    ASSERT_TRUE(player.evaluate(15000, out));  // halfway between kf1 and kf2
    EXPECT_EQ(out.pos_x_mm, 15000);
    EXPECT_EQ(out.pos_y_mm, 5000);
    EXPECT_EQ(out.pos_z_mm, -7500);
    EXPECT_EQ(out.vel_y_mms, 500);
    EXPECT_EQ(out.yaw_cd, 4500);
}

TEST(ShowPlayer, ClampsAfterLastKeyframe)
{
    init_track();
    ShowPlayer player;
    player.set_track(track, 3);
    ShowFile::Keyframe out;
    ASSERT_TRUE(player.evaluate(30000, out));
    EXPECT_EQ(out.pos_x_mm, 20000);
    EXPECT_EQ(out.pos_y_mm, 10000);
    EXPECT_EQ(out.pos_z_mm, -10000);
}

TEST(ShowPlayer, EmptyTrackFails)
{
    ShowPlayer player;
    ShowFile::Keyframe out;
    EXPECT_FALSE(player.evaluate(0, out));
}

TEST(ShowPlayer, YawWrapInterpolation)
{
    // yaw crossing +17000 -> -17000 must interpolate the short way
    ShowFile::Keyframe kf[2] = {
        {0,     0, 0, 0, 0, 0, 0, 17000},
        {10000, 0, 0, 0, 0, 0, 0, -17000},
    };
    ShowPlayer player;
    player.set_track(kf, 2);
    ShowFile::Keyframe out;
    ASSERT_TRUE(player.evaluate(5000, out));
    // short way: +17000 -> +19000 (wraps to -17000), halfway is +18000
    EXPECT_EQ(out.yaw_cd, 18000);
}

TEST(ShowCoordinate, RotateZeroOrientation)
{
    float north, east;
    AC_ShowManager::rotate_show_NE_mm(0.0f, 1000, 0, north, east);
    EXPECT_NEAR(north, 1.0f, 0.001f);
    EXPECT_NEAR(east, 0.0f, 0.001f);
}

TEST(ShowCoordinate, RotateNinetyDegrees)
{
    float north, east;
    AC_ShowManager::rotate_show_NE_mm(90.0f, 1000, 0, north, east);
    EXPECT_NEAR(north, 0.0f, 0.001f);
    EXPECT_NEAR(east, 1.0f, 0.001f);
}

TEST(ShowCoordinate, YAxisPointsEast)
{
    float north, east;
    AC_ShowManager::rotate_show_NE_mm(0.0f, 0, 1000, north, east);
    EXPECT_NEAR(north, 0.0f, 0.001f);
    EXPECT_NEAR(east, 1.0f, 0.001f);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
