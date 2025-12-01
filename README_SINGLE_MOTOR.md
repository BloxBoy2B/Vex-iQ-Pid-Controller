# Single Motor RPM PID Controller

A PID controller for spinning a motor to a specific RPM. Good for shooters, catapults, intakes, or any mechanism that needs consistent speed.

## What it does

Instead of just setting motor power and hoping it reaches the right speed, this uses PID control to automatically adjust power until the motor hits your target RPM.

## How to use

```cpp
MotorSpinUp(150, 5000, 4000);  // spin motor to 150 RPM with 5 second timeout and with a condidtion where it needs to spin for 4 secs no matter what then it can exit.
```

The function takes:
- Target RPM (how fast you want the motor)
- Timeout in milliseconds (stops trying after this long)
- Spin for duration (spins the motor for how ever long you input with no exit condidtion)

## Use cases

**Shooters/Flywheels** - Need consistent RPM for accurate shots. PID maintains speed even as battery drains.

**Catapults** - Spin up to exact release speed every time.

**Intakes** - Control speed for different game objects without manually tuning power.

**Conveyors** - Keep constant speed regardless of load.

## Motor groups

To use with motor groups instead of single motor:
1. Change `motor Motor` to `motor_group MotorGroup` in the config
2. Change `Motor.velocity(rpm)` to `MotorGroup.velocity(rpm)` 
3. Change `Motor.spin()` to `MotorGroup.spin()`

Everything else stays the same.

## Tuning

If the motor overshoots or is too slow, change these values in the code:

```cpp
float kp = 0.5;   // main speed adjustment
float ki = 0.1;   // helps reach exact RPM
float kd = 0.2;   // slows down approach
```

Start with these defaults and adjust if needed.

## How it works

Every 20ms the code:
1. Checks current motor RPM
2. Calculates how far off from target
3. Uses PID math to figure out the right power
4. Adjusts motor power
5. Repeats until target reached or timeout

The motor automatically compensates for battery drain, friction, and load changes.

## Hardware

Currently set up for motor on PORT1. Change in the robot config if yours is different.
