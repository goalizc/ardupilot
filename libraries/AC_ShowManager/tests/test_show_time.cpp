#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>

#include <AC_ShowManager/AC_ShowManager.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// constants mirrored exactly from AP_GPS.h (AP_MSEC_PER_WEEK, UNIX_OFFSET_MSEC)
static const uint32_t WEEK_MS = 604800000UL;
static const uint64_t UNIX_OFFSET_MSEC = 17000ULL * 86400ULL + 520ULL * 604800000ULL - 18000ULL;

TEST(ShowTime, FutureInCurrentWeek)
{
    // start is later this week -> use the current week
    const uint64_t epoch = AC_ShowManager::compute_start_epoch_ms(100, 5000000UL, 6000000UL);
    EXPECT_EQ(epoch, UNIX_OFFSET_MSEC + 100ULL * WEEK_MS + 6000000ULL);
}

TEST(ShowTime, PastInCurrentWeekRollsToNextWeek)
{
    // start is earlier this week -> use the next week
    const uint64_t epoch = AC_ShowManager::compute_start_epoch_ms(100, 5000000UL, 4000000UL);
    EXPECT_EQ(epoch, UNIX_OFFSET_MSEC + 101ULL * WEEK_MS + 4000000ULL);
}

TEST(ShowTime, BoundaryEqualUsesNextWeek)
{
    // gps_week_ms == start_ms is treated as "in the past"
    const uint64_t epoch = AC_ShowManager::compute_start_epoch_ms(100, 6000000UL, 6000000UL);
    EXPECT_EQ(epoch, UNIX_OFFSET_MSEC + 101ULL * WEEK_MS + 6000000UL);
}

TEST(ShowTime, WeekBoundary)
{
    // near the end of the week: start 100ms after now still lands in this week
    const uint64_t epoch = AC_ShowManager::compute_start_epoch_ms(100, 604799990UL, 100UL);
    EXPECT_EQ(epoch, UNIX_OFFSET_MSEC + 101ULL * WEEK_MS + 100ULL);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
