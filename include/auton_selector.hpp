#pragma once
#include <functional>
#include <string>
#include <vector>
#include "api.h"

struct Auton {
    std::string name;
    std::function<void()> run;
};

class AutonSelector {
public:
    std::vector<Auton> autons;
    int auton_page_current = 0;

    void autons_add(std::vector<Auton> list) {
        autons = list;
    }

    void selected_auton_print() {
        pros::lcd::set_text(1, autons[auton_page_current].name.c_str());
    }

    void page_left() {
        auton_page_current = (auton_page_current - 1 + (int)autons.size()) % (int)autons.size();
        selected_auton_print();
        
    }

    void page_right() {
        auton_page_current = (auton_page_current + 1) % (int)autons.size();
        selected_auton_print();
    }

    // Declared here, defined below after `selector` is declared.
    void init();

    void selected_auton_call() {
        if (!autons.empty()) autons[auton_page_current].run();
    }
};

inline AutonSelector selector;

// Defined after `selector` so the lambdas can reference it.
inline void AutonSelector::init() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "< Auton Selector >");
    selected_auton_print();
    pros::lcd::register_btn0_cb([]() { selector.page_left(); });
    pros::lcd::register_btn2_cb([]() { selector.page_right(); });
}
