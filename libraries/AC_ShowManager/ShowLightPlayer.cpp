#include "ShowLightPlayer.h"

void ShowLightPlayer::set_track(const ShowFile::LightEvent *lights, uint16_t count)
{
    _lights = lights;
    _light_count = count;
}

bool ShowLightPlayer::evaluate(uint32_t t_ms, uint8_t &r, uint8_t &g, uint8_t &b) const
{
    // find the latest overall-colour event at or before t_ms.  the events
    // are sorted by time (parser guarantees monotonic t_ms), so scan in
    // order and keep the last matching one.
    const ShowFile::LightEvent *current = nullptr;
    for (uint16_t i = 0; i < _light_count; i++) {
        const ShowFile::LightEvent &ev = _lights[i];
        if (ev.index != 0) {
            continue;   // per-pixel events do not change the overall colour
        }
        if (ev.t_ms > t_ms) {
            break;      // events are time-sorted; later events are in the future
        }
        current = &ev;
    }
    if (current == nullptr) {
        // before the first overall-colour event: light off
        return false;
    }
    r = current->r;
    g = current->g;
    b = current->b;
    return true;
}
