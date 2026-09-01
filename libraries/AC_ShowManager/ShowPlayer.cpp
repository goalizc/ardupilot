#include "ShowPlayer.h"

#include <AP_Math/AP_Math.h>

// set_track - set the keyframe track to play
void ShowPlayer::set_track(const ShowFile::Keyframe *keyframes, uint16_t count)
{
    _keyframes = keyframes;
    _count = count;
}

// evaluate - linearly interpolate the track at t_ms
bool ShowPlayer::evaluate(uint32_t t_ms, ShowFile::Keyframe &out) const
{
    if (_count == 0 || _keyframes == nullptr) {
        return false;
    }
    if (t_ms <= _keyframes[0].t_ms) {
        out = _keyframes[0];
        return true;
    }
    if (t_ms >= _keyframes[_count - 1].t_ms) {
        out = _keyframes[_count - 1];
        return true;
    }

    // find the bracketing keyframes
    uint16_t i = 0;
    while (i + 1 < _count && _keyframes[i + 1].t_ms < t_ms) {
        i++;
    }
    const ShowFile::Keyframe &a = _keyframes[i];
    const ShowFile::Keyframe &b = _keyframes[i + 1];
    const float frac = (float)(t_ms - a.t_ms) / (float)(b.t_ms - a.t_ms);

    out.t_ms = t_ms;
    out.pos_x_mm = a.pos_x_mm + (int32_t)((b.pos_x_mm - a.pos_x_mm) * frac);
    out.pos_y_mm = a.pos_y_mm + (int32_t)((b.pos_y_mm - a.pos_y_mm) * frac);
    out.pos_z_mm = a.pos_z_mm + (int32_t)((b.pos_z_mm - a.pos_z_mm) * frac);
    out.vel_x_mms = a.vel_x_mms + (int16_t)((b.vel_x_mms - a.vel_x_mms) * frac);
    out.vel_y_mms = a.vel_y_mms + (int16_t)((b.vel_y_mms - a.vel_y_mms) * frac);
    out.vel_z_mms = a.vel_z_mms + (int16_t)((b.vel_z_mms - a.vel_z_mms) * frac);
    // interpolate yaw the short way around the circle
    const int32_t yaw_delta = wrap_180_cd((int32_t)b.yaw_cd - (int32_t)a.yaw_cd);
    out.yaw_cd = wrap_180_cd((int32_t)a.yaw_cd + (int16_t)(yaw_delta * frac));
    return true;
}
