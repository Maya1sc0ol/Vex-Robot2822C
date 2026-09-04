# Programming Notes

## To Do
- Motion chaining for auton PID movements (queue drive/turn calls instead of one-at-a-time blocking calls)
- Drive-to-point / boomerang movement (odometry-based) - do this AFTER verifying odom pod accuracy on the field
- Find the drivetrain's real gear ratio (main.cpp's Drive constructor currently uses 0.83, an eyeballed estimate) - drive a known distance and calibrate ratio by actual/reported

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