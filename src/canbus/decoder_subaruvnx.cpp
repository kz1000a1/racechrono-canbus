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
 * decode for SUBARU LEVORG VNx vehicles.
 * 
 * Based on the document of 2nd gen FT86 CAN-bus signals, data formats, units, formulas (factors + offsets)
 * https://github.com/ukmook/ft86/blob/main/can_bus/gen2.md
 */
class decoder_subaruvnx
    : public decoder
{
    CPP_NOCOPY(decoder_subaruvnx);
    CPP_NOMOVE(decoder_subaruvnx);

public:
    static decoder_subaruvnx& get() noexcept
    {
        static decoder_subaruvnx instance;
        return instance;
    }

    ~decoder_subaruvnx() noexcept override
    {
    }

    twai_timing_config_t timing() const noexcept override
    {
        return TWAI_TIMING_CONFIG_500KBITS();
    }

    twai_filter_config_t filter() const noexcept override
    {
        return { .acceptance_code = (0x040 << 21) | (0x345 << 5),
                 .acceptance_mask = (0x17b << 21) | (0x000 << 5) | 0xf000f,
                 .single_filter = false };
    }

    uint16_t rate(uint32_t pid) const noexcept override
    {
        switch (pid)
        {
            // ENGINE RPM / ACCELERATOR POSITION (%) - 100hz
            case 0x040:
                return 3;
            // STEERING ANGLE / YAW RATE - 50hz
            case 0x138:
                return 2;
            // SPEED / BRAKE PRESSURE - 50hz
            case 0x139:
                return 2;
            // VEHICLE SPEED FL / FR / RL / RR - 50hz
            case 0x13A:
                return 2;
            // LATERAL ACCELERATION / LONGITUDINAL ACCELERATION / COMBINED ACCELERATION - 50hz
            case 0x13B:
                return 2;
            // ENGINE OIL TEMPERATURE / COOLANT TEMPERATURE - 10hz
            case 0x345:
                return 1;
            // AIR TEMPERATURE - 10hz
            case 0x390:
                return 1;
            // FUEL LEVEL (%) - 10hz
            case 0x393:
                return 1;
            default:
                return rate_default;
        }
    }

private:
    explicit decoder_subaruvnx() noexcept
        : decoder(8)
    {
        // pre-sorted list of pids
        // list must be sorted by ID, as binary search is used
        // easy enough to pre-sort this list here
        size_t idx = 0;
        
        _ids[idx++] = { 0x040, rate_disabled, 0 }; // ENGINE RPM / ACCELERATOR POSITION (%) - 100hz
        _ids[idx++] = { 0x138, rate_disabled, 0 }; // STEERING ANGLE / YAW RATE - 50hz
        _ids[idx++] = { 0x139, rate_disabled, 0 }; // SPEED / BRAKE PRESSURE (%) - 50hz
        _ids[idx++] = { 0x13A, rate_disabled, 0 }; // VEHICLE SPEED FL / FR / RL / RR - 50hz
        _ids[idx++] = { 0x13B, rate_disabled, 0 }; // LATERAL ACCELERATION / LONGITUDINAL ACCELERATION / COMBINED ACCELERATION - 50hz
        _ids[idx++] = { 0x345, rate_disabled, 0 }; // ENGINE OIL TEMPERATURE / COOLANT TEMPERATURE - 10hz
        _ids[idx++] = { 0x390, rate_disabled, 0 }; // AIR TEMPERATURE - 10hz
        _ids[idx++] = { 0x393, rate_disabled, 0 }; // FUEL LEVEL (%) - 10hz
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

#if defined(CONFIG_CANBUS_DECODER_SUBARUVNX)
canbus::decoder& CANDEC = canbus::decoder_subaruvnx::get();
#endif
