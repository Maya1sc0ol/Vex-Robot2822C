#include "main.h"



// To add a new auto: declare it in autons.h, add one line below, implement it.
void register_autons() {
    selector.autons_add({
        //The first part in brackets that is in quotes is what is shown on the brain
        //The second part of the brackets is the function name so whatever you named it in autos.cpp
        //{"Example Auto", exampleAuto},
        {"Red Auto",  redAuto},
        {"Blue Auto", blueAuto},
        {"Skills",    skills},
        {"Red Left", redLeft}

    });
}

void redAuto() {
    // red alliance autonomous routine
}

void redLeft(){
    //Use this line to code to mirror left auto to right auto or reversed!
    drive.setMirrored(true);
    drive.PID_driveInches(12);

}

void blueAuto() {
    // blue alliance autonomous routine
}

void skills() {
    // skills autonomous routine
}
