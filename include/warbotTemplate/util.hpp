#pragma once
#include "main.h"
#include "warbotTemplate/logo_data.hpp"

namespace warbots {

    // Shared controller handle so a running auton (called synchronously from
    // opcontrol for bench testing) can poll for a second button press to abort
    // itself, without a second task/Controller instance.
    inline pros::Controller master(pros::E_CONTROLLER_MASTER);

    // Set once a fresh Y press is seen while an auton is running; checked by
    // Drive's blocking PID loops and armGoTo() so they can exit cleanly via
    // their own existing stop-motors code, instead of force-killing a task.
    inline volatile bool autonAbortRequested = false;

    inline bool checkAutonAbort() {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) autonAbortRequested = true;
        return autonAbortRequested;
    }

template <typename Condition>
void waitUntil(Condition condition) {
  while (!condition()) {
    pros::delay(10);
  }
}

    inline void screenPrint(const std::string text, int line = 0,
                            pros::text_format_e_t fmt = pros::E_TEXT_MEDIUM,
                            uint32_t bg_color = 0x78b839,
                            uint32_t text_color = 0x000000) {
        pros::screen::set_eraser(bg_color);
        pros::screen::set_pen(text_color);
        pros::screen::print(fmt, line, "%s", text.c_str());
    }

    inline void drawBackground(uint32_t color = 0x78b839) {
        pros::screen::set_eraser(color);
        pros::screen::erase();
    }

    inline std::string doubleToString(double value, int decimals = 2) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", decimals, value);
        return std::string(buf);
    }

    inline void drawLogo(int x_offset = 350, int y_offset = 50, uint32_t bg_color = 0x78b839) {
        static uint32_t modified_pixels[LOGO_HEIGHT][LOGO_WIDTH];
        static bool initialized = false;
        if (!initialized) {
            for (int y = 0; y < LOGO_HEIGHT; y++) {
                for (int x = 0; x < LOGO_WIDTH; x++) {
                    modified_pixels[y][x] = (logo_pixels[y][x] == 0x00FFFFFF) ? bg_color : logo_pixels[y][x];
                }
            }
            initialized = true;
        }
        pros::screen::copy_area(
            x_offset,
            y_offset,
            x_offset + LOGO_WIDTH,
            y_offset + LOGO_HEIGHT,
            (uint32_t*)modified_pixels,
            LOGO_WIDTH
        );
    }
}
