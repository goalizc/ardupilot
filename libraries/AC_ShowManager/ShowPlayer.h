#pragma once

#include <stdint.h>
#include "ShowFile.h"

/*
  Player for a drone show trajectory track. Evaluates the choreography
  keyframes with linear interpolation; pure logic, unit-tested with gtest.
*/
class ShowPlayer {
public:

    // Constructor
    ShowPlayer(void) : _keyframes(nullptr), _count(0) {}

    // set the keyframe track to play
    void set_track(const ShowFile::Keyframe *keyframes, uint16_t count);

    // evaluate the track at time t_ms, clamping before the first and after
    // the last keyframe. returns false when no track is set.
    bool evaluate(uint32_t t_ms, ShowFile::Keyframe &out) const;

private:

    const ShowFile::Keyframe *_keyframes;
    uint16_t _count;
};
