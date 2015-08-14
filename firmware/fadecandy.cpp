/*
 * Fadecandy Firmware
 * 
 * Copyright (c) 2013 Micah Elizabeth Scott
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <math.h>
#include <algorithm>
#include "OctoWS2811z.h"
#include "arm_math.h"
#include "fc_usb.h"
#include "fc_defs.h"
#include "HardwareSerial.h"
#define FIXMATH_NO_ROUNDING
#include "noise.h"

// USB data buffers
static fcBuffers buffers;
fcLinearLUT fcBuffers::lutCurrent;

const uint16_t defaultLUT[LUT_TOTAL_SIZE] = { 0, 0, 0, 1, 2, 3, 5, 8, 11, 14, 19, 24, 30, 36, 44, 52, 61, 71, 82, 94, 106, 120, 135, 151, 168, 186, 205, 225, 247, 269, 293, 318, 344, 372, 401, 431, 462, 495, 529, 564, 601, 640, 679, 720, 763, 807, 853, 900, 949, 999, 1050, 1104, 1159, 1215, 1273, 1333, 1395, 1458, 1522, 1589, 1657, 1727, 1799, 1872, 1947, 2024, 2103, 2183, 2266, 2350, 2436, 2524, 2614, 2706, 2799, 2895, 2992, 3092, 3193, 3296, 3402, 3509, 3618, 3729, 3843, 3958, 4076, 4195, 4317, 4440, 4566, 4694, 4824, 4956, 5091, 5227, 5366, 5507, 5650, 5795, 5942, 6092, 6244, 6398, 6554, 6713, 6874, 7037, 7203, 7371, 7541, 7714, 7889, 8066, 8246, 8428, 8612, 8799, 8988, 9180, 9374, 9570, 9769, 9971, 10174, 10381, 10590, 10801, 11015, 11231, 11450, 11672, 11896, 12122, 12351, 12583, 12817, 13054, 13294, 13536, 13781, 14028, 14278, 14531, 14786, 15044, 15305, 15569, 15835, 16104, 16375, 16649, 16926, 17206, 17489, 17774, 18062, 18353, 18647, 18943, 19242, 19544, 19849, 20157, 20467, 20781, 21097, 21416, 21738, 22063, 22391, 22722, 23056, 23392, 23732, 24074, 24420, 24768, 25119, 25473, 25831, 26191, 26554, 26920, 27290, 27662, 28037, 28416, 28797, 29182, 29569, 29960, 30353, 30750, 31150, 31553, 31959, 32368, 32781, 33196, 33615, 34037, 34461, 34890, 35321, 35755, 36193, 36634, 37078, 37525, 37975, 38429, 38886, 39346, 39810, 40276, 40746, 41220, 41696, 42176, 42659, 43146, 43635, 44128, 44625, 45124, 45627, 46134, 46644, 47157, 47673, 48193, 48716, 49243, 49773, 50307, 50843, 51384, 51928, 52475, 53025, 53579, 54137, 54698, 55262, 55830, 56402, 56977, 57555, 58137, 58723, 59312, 59904, 60500, 61100, 61703, 62310, 0, 0, 0, 1, 2, 3, 6, 8, 11, 15, 20, 25, 31, 38, 46, 54, 64, 74, 86, 98, 112, 126, 142, 159, 176, 195, 215, 237, 259, 283, 308, 334, 362, 391, 421, 453, 486, 520, 556, 594, 632, 673, 715, 758, 803, 849, 897, 947, 998, 1050, 1105, 1161, 1219, 1278, 1339, 1402, 1467, 1533, 1601, 1671, 1743, 1816, 1892, 1969, 2048, 2129, 2212, 2297, 2383, 2472, 2562, 2655, 2749, 2846, 2944, 3045, 3147, 3252, 3358, 3467, 3578, 3691, 3806, 3923, 4042, 4163, 4287, 4413, 4540, 4671, 4803, 4937, 5074, 5213, 5354, 5498, 5644, 5792, 5942, 6095, 6250, 6408, 6567, 6730, 6894, 7061, 7230, 7402, 7576, 7753, 7932, 8113, 8297, 8484, 8673, 8864, 9058, 9255, 9454, 9655, 9859, 10066, 10275, 10487, 10701, 10919, 11138, 11361, 11586, 11813, 12043, 12276, 12512, 12750, 12991, 13235, 13481, 13731, 13983, 14237, 14495, 14755, 15018, 15284, 15552, 15824, 16098, 16375, 16655, 16938, 17223, 17512, 17803, 18097, 18395, 18695, 18998, 19304, 19612, 19924, 20239, 20557, 20877, 21201, 21528, 21857, 22190, 22526, 22865, 23206, 23551, 23899, 24250, 24604, 24961, 25321, 25685, 26051, 26420, 26793, 27169, 27548, 27930, 28315, 28703, 29095, 29490, 29888, 30289, 30693, 31101, 31512, 31926, 32343, 32764, 33188, 33615, 34045, 34479, 34916, 35356, 35800, 36247, 36697, 37151, 37608, 38068, 38531, 38999, 39469, 39943, 40420, 40901, 41385, 41872, 42363, 42857, 43355, 43856, 44361, 44869, 45381, 45896, 46414, 46936, 47462, 47991, 48524, 49060, 49600, 50143, 50690, 51240, 51794, 52352, 52913, 53477, 54046, 54618, 55193, 55772, 56355, 56941, 57531, 58125, 58723, 59324, 59928, 60537, 61149, 61765, 62384, 63007, 63634, 64265, 64899, 65535, 0, 0, 0, 1, 2, 3, 6, 8, 11, 15, 20, 25, 31, 38, 46, 54, 64, 74, 86, 98, 112, 126, 142, 159, 176, 195, 215, 237, 259, 283, 308, 334, 362, 391, 421, 453, 486, 520, 556, 594, 632, 673, 715, 758, 803, 849, 897, 947, 998, 1050, 1105, 1161, 1219, 1278, 1339, 1402, 1467, 1533, 1601, 1671, 1743, 1816, 1892, 1969, 2048, 2129, 2212, 2297, 2383, 2472, 2562, 2655, 2749, 2846, 2944, 3045, 3147, 3252, 3358, 3467, 3578, 3691, 3806, 3923, 4042, 4163, 4287, 4413, 4540, 4671, 4803, 4937, 5074, 5213, 5354, 5498, 5644, 5792, 5942, 6095, 6250, 6408, 6567, 6730, 6894, 7061, 7230, 7402, 7576, 7753, 7932, 8113, 8297, 8484, 8673, 8864, 9058, 9255, 9454, 9655, 9859, 10066, 10275, 10487, 10701, 10919, 11138, 11361, 11586, 11813, 12043, 12276, 12512, 12750, 12991, 13235, 13481, 13731, 13983, 14237, 14495, 14755, 15018, 15284, 15552, 15824, 16098, 16375, 16655, 16938, 17223, 17512, 17803, 18097, 18395, 18695, 18998, 19304, 19612, 19924, 20239, 20557, 20877, 21201, 21528, 21857, 22190, 22526, 22865, 23206, 23551, 23899, 24250, 24604, 24961, 25321, 25685, 26051, 26420, 26793, 27169, 27548, 27930, 28315, 28703, 29095, 29490, 29888, 30289, 30693, 31101, 31512, 31926, 32343, 32764, 33188, 33615, 34045, 34479, 34916, 35356, 35800, 36247, 36697, 37151, 37608, 38068, 38531, 38999, 39469, 39943, 40420, 40901, 41385, 41872, 42363, 42857, 43355, 43856, 44361, 44869, 45381, 45896, 46414, 46936, 47462, 47991, 48524, 49060, 49600, 50143, 50690, 51240, 51794, 52352, 52913, 53477, 54046, 54618, 55193, 55772, 56355, 56941, 57531, 58125, 58723, 59324, 59928, 60537, 61149, 61765, 62384, 63007, 63634, 64265, 64899, 65535};

// Double-buffered DMA memory for raw bit planes of output
static DMAMEM int ledBuffer[LEDS_PER_STRIP * 12];
static OctoWS2811z leds(LEDS_PER_STRIP, ledBuffer, WS2811_800kHz);

/*
 * Residuals for temporal dithering. Usually 8 bits is enough, but
 * there are edge cases when it isn't, and we don't have the spare CPU cycles
 * to saturate values before storing. So, 16-bit it is.
 */
typedef int16_t residual_t;
static residual_t residual[CHANNELS_TOTAL];

// Reserved RAM area for signalling entry to bootloader
extern uint32_t boot_token;

//#define PARASOL
//#define RAINBOW_CIRCLE
#define SMOOTH_RAINBOW
//#define RAINBOW_TOPHAT
//#define RAINBOW_LANTERNS

#ifdef RAINBOW_LANTERNS
int16_t directions[] = { 5, 1, 3, -4, 2, -3, -1, 4, 3, -2, -5, -1, -5, 4, -1, -2, 3, -3, -4, 2 };
int16_t steps[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int16_t indexes[] = { 3, 5, 1, 6, 2, 4, 2, 0, 2, 0, 4, 5, 3, 5, 6, 1, 3, 0, 2, 1 };
#endif

/*
 * Low-level drawing code, which we want to compile in the same unit as the main loop.
 * We compile this multiple times, with different config flags.
 */

#define FCP_INTERPOLATION   0
#define FCP_DITHERING       0
#define FCP_FN(name)        name##_I0_D0
#include "fc_pixel_lut.cpp"
#include "fc_pixel.cpp"
#include "fc_draw.cpp"
#undef FCP_INTERPOLATION
#undef FCP_DITHERING
#undef FCP_FN

#define FCP_INTERPOLATION   1
#define FCP_DITHERING       0
#define FCP_FN(name)        name##_I1_D0
#include "fc_pixel_lut.cpp"
#include "fc_pixel.cpp"
#include "fc_draw.cpp"
#undef FCP_INTERPOLATION
#undef FCP_DITHERING
#undef FCP_FN

#define FCP_INTERPOLATION   0
#define FCP_DITHERING       1
#define FCP_FN(name)        name##_I0_D1
#include "fc_pixel_lut.cpp"
#include "fc_pixel.cpp"
#include "fc_draw.cpp"
#undef FCP_INTERPOLATION
#undef FCP_DITHERING
#undef FCP_FN

#define FCP_INTERPOLATION   1
#define FCP_DITHERING       1
#define FCP_FN(name)        name##_I1_D1
#include "fc_pixel_lut.cpp"
#include "fc_pixel.cpp"
#include "fc_draw.cpp"
#undef FCP_INTERPOLATION
#undef FCP_DITHERING
#undef FCP_FN


static inline uint32_t calculateInterpCoefficient()
{
    /*
     * Calculate our interpolation coefficient. This is a value between
     * 0x0000 and 0x10000, representing some point in between fbPrev and fbNext.
     *
     * We timestamp each frame at the moment its final packet has been received.
     * In other words, fbNew has no valid timestamp yet, and fbPrev/fbNext both
     * have timestamps in the recent past.
     *
     * fbNext's timestamp indicates when both fbPrev and fbNext entered their current
     * position in the keyframe queue. The difference between fbPrev and fbNext indicate
     * how long the interpolation between those keyframes should take.
     */

    uint32_t now = millis();
    uint32_t tsPrev = buffers.fbPrev->timestamp;
    uint32_t tsNext = buffers.fbNext->timestamp;
    uint32_t tsDiff = tsNext - tsPrev;
    uint32_t tsElapsed = now - tsNext;

    // Careful to avoid overflows if the frames stop coming...
    return (std::min<uint32_t>(tsElapsed, tsDiff) << 16) / tsDiff;
}

static void dfu_reboot()
{
    // Reboot to the Fadecandy Bootloader
    boot_token = 0x74624346;

    // Short delay to allow the host to receive the response to DFU_DETACH.
    uint32_t deadline = millis() + 10;
    while (millis() < deadline) {
        watchdog_refresh();
    }

    // Detach from USB, and use the watchdog to time out a 10ms USB disconnect.
    __disable_irq();
    USB0_CONTROL = 0;
    while (1);
}

extern "C" int usb_rx_handler(usb_packet_t *packet)
{
    // USB packet interrupt handler. Invoked by the ISR dispatch code in usb_dev.c
    return buffers.handleUSB(packet);
}

uint32_t rainbow7[7];
uint32_t rainbow10[10];
const float timestep = .005f;
float time = 0;

uint32_t color(uint8_t red, uint8_t green, uint8_t blue)
{
  return (red << 16) | (green << 8) | blue;
}

#define RED(color) ((uint8_t)(((color) & 0x00FF0000) >> 16))
#define GREEN(color) ((uint8_t)(((color) & 0x0000FF00) >> 8))
#define BLUE(color) ((uint8_t)((color) & 0x000000FF))

extern "C" int main()
{
    uint32_t last = systick_millis_count;
    uint32_t lastRender = 0;
    uint16_t i = 0;
    uint16_t j = 0;
    uint8_t index = 0;
    uint8_t startIndex = 0;
    pinMode(LED_BUILTIN, OUTPUT);
    for (i = 0; i < LUT_TOTAL_SIZE; i++)
    {
      buffers.lutCurrent.entries[i] = defaultLUT[i] >> 1;// (i % LUT_CH_SIZE) << 7;
    }

#ifdef RAINBOW_LANTERNS
    int16_t numSteps = 64;
#endif

    leds.begin();

    rainbow10[0] = rainbow7[0] = color(0xFF, 0, 0);
    rainbow10[1] = rainbow7[1] = color(0xFF, 0xA5, 0);
    rainbow10[2] = rainbow7[2] = color(0xFF, 0xFF, 0);
    rainbow10[3] = rainbow7[3] = color(0, 0x80, 0);
    rainbow10[4] = color(0, 0xFF, 0);
    rainbow10[5] = color(0, 0xA5, 0x80);
    rainbow10[6] = rainbow7[4] = color(0, 0, 0xFF);
    rainbow10[7] = rainbow7[5] = color(0x4B, 0, 0x82);
    rainbow10[8] = rainbow7[6] = color(0xFF, 0, 0xFF);
    rainbow10[9] = color(0xEE, 0x82, 0xEE);

    buffers.flags = CFLAG_NO_ACTIVITY_LED;
    //buffers.flags |= CFLAG_LED_CONTROL;

    // Announce firmware version
    serial_begin(BAUD2DIV(115200));
    serial_print("Fadecandy v" DEVICE_VER_STRING "\r\n");

    //usb_serial_printf("test\n");

    // Application main loop
    while (usb_dfu_state == DFU_appIDLE) {
        watchdog_refresh();

        // Select a different drawing loop based on our firmware config flags
        switch (buffers.flags & (CFLAG_NO_INTERPOLATION | CFLAG_NO_DITHERING)) {
            case 0:
            default:
                updateDrawBuffer_I1_D1(calculateInterpCoefficient());
                break;
            case CFLAG_NO_INTERPOLATION:
                updateDrawBuffer_I0_D1(0x10000);
                break;
            case CFLAG_NO_DITHERING:
                updateDrawBuffer_I1_D0(calculateInterpCoefficient());
                break;
            case CFLAG_NO_INTERPOLATION | CFLAG_NO_DITHERING:
                updateDrawBuffer_I0_D0(0x10000);
                break;
        }

        //buffers.flags |= CFLAG_NO_ACTIVITY_LED;
        // Start sending the next frame over DMA
        leds.show();

        if ((systick_millis_count - last) > 500)
        {
            //buffers.flags ^= CFLAG_LED_CONTROL;

            last = systick_millis_count;
        }

        #ifdef RAINBOW_CIRCLE
        if ((systick_millis_count - lastRender) > 500)
        {
          buffers.flags ^= CFLAG_LED_CONTROL;
          lastRender = systick_millis_count;
          index = startIndex;

          for (i = 0; i < 52/*32*/; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 7 + i);
            uint32_t color = rainbow7[index];

            pixel[0] = RED(color);
            pixel[1] = GREEN(color);
            pixel[2] = BLUE(color);

            index = (index + 1) % 7;
          }

          for (i = 0; i < 20; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 6 + i);
            uint32_t color = rainbow7[index];

            pixel[0] = RED(color);
            pixel[1] = GREEN(color);
            pixel[2] = BLUE(color);

            index = (index + 1) % 7;
          }

          startIndex = (startIndex + 1) % 7;

          buffers.finalizeFramebuffer();
        }
        #endif

#ifdef RAINBOW_TOPHAT
        (void)j;
        if ((systick_millis_count - lastRender) > 500)
        {
          buffers.flags ^= CFLAG_LED_CONTROL;
          lastRender = systick_millis_count;
          index = startIndex;

          for (i = 0; i < 52/*32*/; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 7 + i);
            uint32_t color = rainbow7[index];

            pixel[0] = RED(color);
            pixel[1] = GREEN(color);
            pixel[2] = BLUE(color);
            if (i>0 && i % 5 == 0)
            {
              index = (index + 1) % 7;
            }
          }

          for (i = 0; i < 20; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 6 + i);
            uint32_t color = rainbow7[index];

            pixel[0] = RED(color);
            pixel[1] = GREEN(color);
            pixel[2] = BLUE(color);

            index = (index + 1) % 7;
          }

          startIndex = (startIndex + 1) % 7;

          buffers.finalizeFramebuffer();
        }
#endif

#ifdef RAINBOW_LANTERNS
        /*(void)j;
        (void)index;
        (void)startIndex;
        if ((systick_millis_count - lastRender) > 500)
        {
          buffers.flags ^= CFLAG_LED_CONTROL;
          lastRender = systick_millis_count;

          for (i = 0; i < 20; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 7 + i);
            int16_t direction = directions[i] < 0 ? -1 : 1;
            uint32_t color1 = rainbow7[indexes[i]];
            int16_t index2 = indexes[i] + direction;
            if (index2 < 0)
            {
              index2 = 6;
            }
            else if (index2 >= 7)
            {
              index2 = 0;
            }
            uint32_t color2 = rainbow7[index2];

            int32_t reddiff = RED(color2) - RED(color1);
            int32_t bluediff = BLUE(color2) - BLUE(color1);
            int32_t greendiff = GREEN(color2) - BLUE(color1);

            reddiff *= steps[i];
            reddiff /= numSteps;
            bluediff *= steps[i];
            bluediff /= numSteps;
            greendiff *= steps[i];
            greendiff /= numSteps;

            pixel[0] = RED(color1) + reddiff;
            pixel[1] = GREEN(color1) + greendiff;
            pixel[2] = BLUE(color1) + bluediff;

            steps[i] += abs(directions[i]);

            if (steps[i] >= numSteps)
            {
              steps[i] = 0;
              indexes[i] = index2;
            }
          }

          buffers.finalizeFramebuffer();
        }*/
        (void)j;
        (void)numSteps;
        if ((systick_millis_count - lastRender) > 5000)
        {
          buffers.flags ^= CFLAG_LED_CONTROL;
          lastRender = systick_millis_count;
          index = startIndex;

          for (i = 0; i < 20/*32*/; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 7 + i);
            uint32_t color = rainbow7[index];

            pixel[0] = RED(color);
            pixel[1] = GREEN(color);
            pixel[2] = BLUE(color);
            if (i>0 && i % 2 == 0)
            {
              index = (index + 1) % 7;
            }
          }

          for (i = 0; i < 20; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 6 + i);
            uint32_t color = rainbow7[index];

            pixel[0] = RED(color);
            pixel[1] = GREEN(color);
            pixel[2] = BLUE(color);

            index = (index + 1) % 7;
          }

          startIndex = (startIndex + 1) % 7;

          buffers.finalizeFramebuffer();
        }
#endif

#ifdef PARASOL
        if ((systick_millis_count - lastRender) > 500)
        {
          buffers.flags ^= CFLAG_LED_CONTROL;
          lastRender = systick_millis_count;
          index = startIndex;

          for (i = 0; i < 8; i++)
          {
            for (j = 0; j < 3; j++)
            {
              uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * i + j);
              uint32_t color = rainbow7[index];

              pixel[0] = RED(color);
              pixel[1] = GREEN(color);
              pixel[2] = BLUE(color);
            }

            index = (index + 1) % 7;
          }

          startIndex = (startIndex + 1) % 7;

          buffers.finalizeFramebuffer();
        }
#endif

        #ifdef SMOOTH_RAINBOW
        (void)startIndex;
        (void)j;
        if ((systick_millis_count - lastRender) > 30)
        {
          lastRender = systick_millis_count;
          uint32_t ledIndex = 0;

          if (index >= 16)
          {
            buffers.flags ^= CFLAG_LED_CONTROL;
            index = 0;
          }

          index++;

          float color1[3];
          float color2[3];
          
          for (i = 0; i < 32; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 7 + i);

            pixel[0] = 0xFF;
            pixel[1] = 0;
            pixel[2] = 0;
            float scaled = ledIndex / 44.0f;
            //fix16_t noise = 32768; (void)scaled;
            float noise = .5f + noise2(scaled, time);
            //float noise = .5f + fbm_noise3(scaled, time, .5f, 4, .25f, 2.0f);
            noise = fmax(fmin(1, noise), 0);
            float mult = 9 * noise;

            float index1 = floor(mult);
            float index2 = ceil(mult);
            float percent = mult - index1;
            uint32_t color = rainbow10[(int)index1];
            color1[0] = RED(color);
            color1[1] = GREEN(color);
            color1[2] = BLUE(color);

            color = rainbow10[(int)index2];
            color2[0] = RED(color);
            color2[1] = GREEN(color);
            color2[2] = BLUE(color);

            pixel[0] = (uint8_t)((int)(color1[0] + ((color2[0] - color1[0]) * percent)) & 0xFF);
            pixel[1] = (uint8_t)((int)(color1[1] + ((color2[1] - color1[1]) * percent)) & 0xFF);
            pixel[2] = (uint8_t)((int)(color1[2] + ((color2[2] - color1[2]) * percent)) & 0xFF);

            ledIndex++;
          }
          /*
          for (i = 0; i < 12; i++)
          {
            uint8_t* pixel = buffers.fbNew->pixel(LEDS_PER_STRIP * 6 + i);

            pixel[0] = 0xFF;
            pixel[1] = 0;
            pixel[2] = 0;
            float scaled = ledIndex / 44.0f;
            (void)scaled;
            float noise = 32768;
              //noise2(scaled, time);
            //fix16_t noise = 32768 + noise2(scaled, time);
            //fix16_t noise = 32768 + fbm_noise3(scaled, time, 0, 4, 16384, 131072);
            noise = fmax(fmin(noise, 1), 0);
            float mult = 9 * noise;
            float index1 = floor(mult);
            float index2 = ceil(mult);
            float percent = mult - index1;
            (void)index2;
            (void)percent;
            uint32_t color = rainbow10[1];// fix16_to_int(index1)];
            color1[0] = RED(color);
            color1[1] = GREEN(color);
            color1[2] = BLUE(color);

            color = rainbow10[2];// fix16_to_int(index2)];
            color2[0] = RED(color);
            color2[1] = GREEN(color);
            color2[2] = BLUE(color);

            pixel[0] = (uint8_t)((int)(color1[0] + ((color2[0] - color1[0]) * percent)) & 0xFF);
            pixel[1] = (uint8_t)((int)(color1[1] + ((color2[1] - color1[1]) * percent)) & 0xFF);
            pixel[2] = (uint8_t)((int)(color1[2] + ((color2[2] - color1[2]) * percent)) & 0xFF);

            ledIndex++;
          } */

          time = (time + timestep);

          buffers.finalizeFramebuffer();
        }
        #endif

        // We can switch to the next frame's buffer now.
        buffers.finalizeFrame();

        // Performance counter, for monitoring frame rate externally
        perf_frameCounter++;
    }

    // Reboot into DFU bootloader
    dfu_reboot();
}
