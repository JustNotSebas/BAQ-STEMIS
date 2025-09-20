## Rundown

This repository contains a personal, in-improvement code for a line-follower robot made with the Arduino software and hardware.
Made for the BAQ-STEM+IS SKILL CHALLENGE ROBOTICA 2025, this single-folder code contains the following functionality:

- Calibrating the middle point between the difference of the line and its background
- Reading the current colors below a sensor using analog reading
- Getting the current position of the car relative to the line
- Using a PID controller to determine the motors' speed, altogether with a exponential velocity decrease in curves.
- Moving two motors to move the robot along the line
- Other functionality to support good movement in 90°/45° turns, intersections or close calls with short headed lines

## Program structure

```
Main.ino
   │
   ▼
+----------------+
| Config.h       |  <- constants, variables and definitions
+----------------+
   │       │       │
   ▼       ▼       ▼
Utils.h  Sensors.h Motors.h
(calib.) (reading) (movem.)
                      │
                      ▼
                PIDControl.h
                      │
                      ▼
              Intersections.h
```

## Work(s) in progress

- [X] Modularize the program (This repo!)
- [ ] Reset windup when the car is relocated (PIDControl.cpp)
- [ ] Modify kPIDV through Serial without recompiling (ROM.h and ROM.cpp)
- [ ] Rewrite frenos() function to make slow turns to last known location (Motors.cpp)
- [ ] Remake 90° and 45° turns system (Intersections.cpp)

## Runup

This is one of the projects I've spent the most love (and hate) on it. And even though this is my last year at the Skill Challenge Robotica (at least with my current team- God knows what I'll do in the future), it's also good to be saving this for posperity.
My previous repository, single files, which contains my work for the year 2024 and this one's (up to September 2025) can be found [here](https://github.com/JustNotSebas/SKCHRobotica).




