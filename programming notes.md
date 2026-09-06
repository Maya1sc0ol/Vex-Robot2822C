# Programming Notes

## >>> START HERE NEXT SESSION <<<
Phase 2 (gear ratio, drive PID, turn PID, IMU_HEADING, CSV logging/telemetry) is
done - see Feature Verification Status below. Next: run `redAuto()` on hardware
(no separate test auton - work in `redAuto` directly) and review what comes back:
arm ramp/settle during each `armGoTo()` call, claw timing around
`openclaw()`/`closeclaw()`/`CLAW_SETTLE_MS`, and that the drive/turn legs exit via
`SMALL` like the tuning runs did. That closes out Phase 3.

## To Do
- Motion chaining for auton PID movements (queue drive/turn calls instead of one-at-a-time blocking calls)
- Drive-to-point / boomerang movement (odometry-based) - do this AFTER verifying odom pod accuracy on the field
- (general, not yet implemented) Add speedScale (or actual left/right velocity) to the teleop screen printout, so Phase 1 speed-curve testing has a number to check instead of just feel

## Feature Verification Status (as of merging autons-and-gear-ratio-wip into main)
Nothing below is field-verified unless marked done - check items off as they're bench/field tested.
Sequence: Phase 1 -> Phase 2 -> Phase 3, then write the real scoring auto.

### Phase 1 - Teleop drive-speed controls (testing now)
- [ ] A button precision mode - confirmed as design intent: hold A = flat constant 30% speed, full override, does not stack/combine with the arm-height curve. Code already matches this (no change needed) - still needs hardware verification.
- [ ] Arm-height drive speed curve (100% at/below arm=50, ramping 80% -> 70% from arm=50 to arm=100) - brand new, never run
- [x] Bench-test Y-button auton trigger - removed from teleop testing for now (commented out in main.cpp's opcontrol(), not deleted - re-enable later if we want it back)

### Phase 2 - Auton tuning + sensor validation
Big phase - tackling one sensor/item at a time across sessions rather than all at once.
1. [x] Drive encoders / gear ratio calibration - field-confirmed (0.75 ratio, 36T motor-side/48T wheel-side)
2. [x] Drive PID tuning (PID_driveInches) - validated via pidTuneTest (pid_tune_12/13.csv); DRIVE_KP=6.0/KI=0.5/KD=0.2/startI=2.5 promoted to include/autonConfig.hpp
3. [x] Turn PID tuning (PID_turnDegrees) - validated via pidTurnTuneTest (pid_turn_7.csv, after fixing a sign bug); TURN_KP=3.0/KI=0.2/KD=0.3/startI=5.0 promoted to include/autonConfig.hpp
4. [x] HeadingHoldMode::IMU_HEADING - exercised across odom_square_1 through _8.csv with no regression; also extended with an optional absolute headingTarget (PID_driveInches) + new PID_turnToHeading() to stop per-turn overshoot from compounding across multiple turns - validated on odom_square_8.csv (turn error stayed bounded near +-1deg instead of growing to +7.9deg)
5. [ ] HeadingHoldMode::ODOM_LATERAL (drive straight via the horizontal tracker instead of IMU) - correction sign unverified, may need flipping - still not exercised
6. [x] Per-run CSV logging (CsvLogger/nextLogPath) - confirmed uniquely-numbered files across many odom_square_N/pid_tune_N/pid_turn_N.csv runs this session
7. [x] Per-motor/IMU/tracker telemetry columns - confirmed sane, real values (motor current/voltage, IMU heading/gyro/accel, tracker inches, pose x/y/angle) in reviewed CSVs
8. [x] Exercise odomSquareTest, pidTuneTest, pidTurnTuneTest - all run and their CSVs reviewed this session (driveOdomPodTest/driveEncoderOnlyTest no longer exist in the codebase, dropped from this list)

Also done this session, not originally on this list: a shared `include/autonConfig.hpp` now holds the drive/turn PID + exit-condition values applied to every match auto in one place (`autonomous()` in main.cpp) - retune by running pidTuneTest/pidTurnTuneTest and copying winning numbers there, not per-auton.

### Phase 3 - Auto building blocks (gate before writing the real scoring auto)
Everything needed to "just write the auto": drive, turn, elevator-by-position, claw open/close.
Not yet run/reviewed in an actual auton context - next step is running redAuto()
itself on hardware (it already implements the "Red Auto" sequence below) and
reviewing the result, not building a separate test auton.
- [ ] Drive (PID_driveInches) - tuned in Phase 2; not yet exercised inside redAuto specifically
- [ ] Turn (PID_turnDegrees / PID_turnToHeading) - tuned in Phase 2; not yet exercised inside redAuto specifically
- [ ] Elevator/arm position control in auto (armGoTo()) - separate from teleop arm control; verify smooth, accurate, no overshoot/oscillation given the recent arm-lifting issues
- [ ] Claw open/close in auto (openclaw()/closeclaw(), CLAW_SETTLE_MS=500) - verify claw positions and settle timing are right
- [ ] Arm PID gains (KP=3.5, KD=0.35, no KI) + wrap-safe angle math, gravity feedforward, rotation sensor zero, brake-mode-first init ordering - all rolled into armGoTo() above, but call out explicitly since this is what broke before

Once Phase 3 is fully checked: redAuto is ready to compete on. blueAuto is still an empty stub - not in scope until redAuto is validated.

Red Auto
drive forwared 12
turn 90
drive forwared and lift to level 1
score (open the claw, lift the arm, then back up)

Red auto
close claw
lift arm to fist level
drive forward 3 inches
score= open claw
raise arm to highest level
put arm all the way down
turn -45 degrees
drive forward 8 inches
drive backwards 13 1/2 inches
finished

Blue auto
clow claw 
lift arm to first level
drive forward 3 inches
score= open claw
raise arm to highest level
put arm all the way down
turn -45 degrees
drive forward 8 inches
drive backwards 13 1/2 inches
finished