# Programming Notes

## >>> START HERE NEXT SESSION <<<
Field-calibrate the drivetrain gear ratio before any further PID tuning. See "Confirm the drivetrain's real gear ratio" below - do this first, it gates the rest of Phase 2.

## To Do
- Motion chaining for auton PID movements (queue drive/turn calls instead of one-at-a-time blocking calls)
- Drive-to-point / boomerang movement (odometry-based) - do this AFTER verifying odom pod accuracy on the field
- Confirm the drivetrain's real gear ratio in the field (main.cpp's Drive constructor now uses 0.75, calculated from 36T motor-side / 48T wheel-side - was 450, a leftover placeholder, until this merge) - drive a known distance and check reported inches match reality
- (general, not yet implemented) Add speedScale (or actual left/right velocity) to the teleop screen printout, so Phase 1 speed-curve testing has a number to check instead of just feel

## Feature Verification Status (as of merging autons-and-gear-ratio-wip into main)
Nothing below is field-verified unless marked done - check items off as they're bench/field tested.
Sequence: Phase 1 -> Phase 2 -> Phase 3, then write the real scoring auto.

### Phase 1 - Teleop drive-speed controls (testing now)
- [ ] A button precision mode - confirmed as design intent: hold A = flat constant 30% speed, full override, does not stack/combine with the arm-height curve. Code already matches this (no change needed) - still needs hardware verification.
- [ ] Arm-height drive speed curve (100% at/below arm=50, ramping 80% -> 70% from arm=50 to arm=100) - brand new, never run
- [x] Bench-test Y-button auton trigger - removed from teleop testing for now (commented out in main.cpp's opcontrol(), not deleted - re-enable later if we want it back)

### Phase 2 - Auton tuning + sensor validation
Big phase - tackling one sensor/item at a time across sessions rather than all at once. Suggested order (adjust as needed), first item is the START HERE marker above:
1. [ ] Drive encoders / gear ratio calibration (see START HERE marker) - gates everything else below
2. [ ] Drive PID tuning (PID_driveInches, DRIVE_KP=1.5) - first real tuning pass now that the gear ratio is correct
3. [ ] Turn PID tuning (PID_turnDegrees)
4. [ ] HeadingHoldMode::IMU_HEADING - re-verify no regression from the old bool holdHeading -> enum change
5. [ ] HeadingHoldMode::ODOM_LATERAL (drive straight via the horizontal tracker instead of IMU) - correction sign unverified, may need flipping
6. [ ] Per-run CSV logging (CsvLogger/nextLogPath) - confirm files actually land uniquely numbered on the SD card instead of overwriting
7. [ ] Per-motor/IMU/tracker telemetry columns - confirm real, sane values show up in a log (not zeros/stale)
8. [ ] Exercise driveOdomPodTest, driveEncoderOnlyTest, odomSquareTest, pidTuneTest, pidTurnTuneTest - none run yet with current code

### Phase 3 - Auto building blocks (gate before writing the real scoring auto)
Everything needed to "just write the auto": drive, turn, elevator-by-position, claw open/close.
- [ ] Drive (PID_driveInches) - carried over from Phase 2 once tuned
- [ ] Turn (PID_turnDegrees) - carried over from Phase 2 once tuned
- [ ] Elevator/arm position control in auto (armGoTo()) - separate from teleop arm control; verify smooth, accurate, no overshoot/oscillation given the recent arm-lifting issues
- [ ] Claw open/close in auto (openclaw()/closeclaw(), CLAW_SETTLE_MS=500) - verify claw positions and settle timing are right
- [ ] Arm PID gains (KP=3.5, KD=0.35, no KI) + wrap-safe angle math, gravity feedforward, rotation sensor zero, brake-mode-first init ordering - all rolled into armGoTo() above, but call out explicitly since this is what broke before

Once Phase 3 is fully checked: write the real scoring auto (redAuto/blueAuto are currently either untuned or empty stubs, not ready to compete on).

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