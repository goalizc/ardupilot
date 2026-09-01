#pragma once

#include <stdint.h>

#include "ShowFile.h"

/*
  ShowLightPlayer - evaluates the overall-colour light track of a
  choreography at a given show-clock position.

  The light track is a list of discrete events (ShowFile::LightEvent).
  Only events with index 0 (overall colour) are considered; per-pixel
  events (index >= 1) are ignored here and belong to the per-pixel LED
  layer (D7-A).  Between events the colour is held (step hold): the
  colour changes only when the show clock reaches the next event.  Before
  the first event the light is off (evaluate returns false).
*/
class ShowLightPlayer {
public:
    // constructor: no track loaded yet
    ShowLightPlayer(void) : _lights(nullptr), _light_count(0) {}

    // load the overall-colour light track; the caller must keep the
    // buffer alive while the player is used (same contract as ShowPlayer)
    void set_track(const ShowFile::LightEvent *lights, uint16_t count);

    // evaluate the overall colour at show-clock time t_ms.
    // returns false (light off) before the first overall-colour event;
    // otherwise fills r/g/b and returns true.
    bool evaluate(uint32_t t_ms, uint8_t &r, uint8_t &g, uint8_t &b) const;

private:
    const ShowFile::LightEvent *_lights;
    uint16_t _light_count;
};
