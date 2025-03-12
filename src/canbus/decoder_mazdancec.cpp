// MIT License
//
// Copyright (c) 2022 Joe Roback <joe.roback@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "../racechrono-canbus.hpp"

#include "decoder.hpp"

namespace canbus
{

/**
 * decode for MAZDA mx-5 MK3 MK3.5 MK3.75 vehicles.
 * 
 * Document of mx-5 CAN-bus signals, data formats, units, formulas (factors + offsets)
 * https://github.com/timurrrr/RaceChronoDiyBleDevice/blob/master/can_db/mazda_mx5_nc.md
 */
class decoder_mazdancec
    : public decoder
{
    CPP_NOCOPY(decoder_mazdancec);
    CPP_NOMOVE(decoder_mazdancec);

public:
    static decoder_mazdancec& get() noexcept
    {
        static decoder_mazdancec instance;
        return instance;
    }

    ~decoder_mazdancec() noexcept override
    {
    }

    twai_timing_config_t timing() const noexcept override
    {
        return TWAI_TIMING_CONFIG_500KBITS();
    }

    twai_filter_config_t filter() const noexcept override
    {
        return { .acceptance_code = (0x081 << 21) | (0x4b0 << 5),
                 .acceptance_mask = (0x2c5 << 21) | (0x000 << 5) | 0xf000f,
                 .single_filter = false };
    }

    uint16_t rate(uint32_t pid) const noexcept override
    {
        switch (pid)
        {
            // STEERING ANGLE - 100hz
            case 0x081:
                return 3;
            // BRAKE PRESSURE - 100hz
            case 0x085:
                return 3;
            // THROTTLE POSITION (%) - 100hz
            case 0x200:
                return 3;
            // ENGINE RPM / SPEED / ACCELERATOR POSITION (%) - 100hz
            case 0x201:
                return 3;
            // CALCULATED LOAD (%) / COOLANT TEMPERATURE - 10hz
            case 0x240:
                return 1;
            // VEHICLE SPEED FL / FR / RL / RR - 100hz
            case 0x4B0:
                return 3;
            default:
                return rate_default;
        }
    }

private:
    explicit decoder_mazdancec() noexcept
        : decoder(6)
    {
        // pre-sorted list of pids
        // list must be sorted by ID, as binary search is used
        // easy enough to pre-sort this list here
        size_t idx = 0;
        
        _ids[idx++] = { 0x081, rate_disabled, 0 }; // STEERING ANGLE
        _ids[idx++] = { 0x085, rate_disabled, 0 }; // BRAKE PRESSURE
        _ids[idx++] = { 0x200, rate_disabled, 0 }; // THROTTLE POSITION (%)
        _ids[idx++] = { 0x201, rate_disabled, 0 }; // ENGINE RPM / SPEED / ACCELERATOR POSITION (%)
        _ids[idx++] = { 0x240, rate_disabled, 0 }; // CALCULATED LOAD (%) / COOLANT TEMPERATURE
        _ids[idx++] = { 0x4B0, rate_disabled, 0 }; // VEHICLE SPEED FL / FR / RL / RR

        // make sure pids are in sorted order
        uint32_t id = 0;
        for (size_t i = 0; i < idx; i++)
        {
            RCASSERT(id < _ids[i].id);
            id = _ids[i].id;
        }
    }
};

} // namespace canbus

#if defined(CONFIG_CANBUS_DECODER_MAZDANCEC)
canbus::decoder& CANDEC = canbus::decoder_mazdancec::get();
#endif
