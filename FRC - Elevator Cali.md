FRC - Elevator Cali



Steps to configure Motion Magic

○ Download and configure sample code to fit our system
CAN address
Joystick
disable all followers
Selected Feedback Sensor

○ Check motor and sensor polarity
■ These must be in phase meaning positive motor command results in positive
feedback

zero all PIDF, Cruise Velocity, and acceleration
special case setting for limit switches and referencing the encoder:
zero the sensor: (init position to 0)
talon.setSelectedSensorPosition(0, constants.kPIDLoopidx, constants.KtimeoutMs);

special case:
talon.configclearPositionOnLimitR(true, constants.kTimoutMs);
talon.configFeebackNotContinous(true, Constants.kTimeoutMs);

Current Limits, Motor stall
talon.configPeakcurrentLimit(30);
talon.configPeakCurrentDuration(150);
talon.configContinuousCurrentLimit(20);
https://www.youtube.com/watch?v=U4pgviiEwLg

BreakMode:
talon.setNeutralMode(NeutralMode.Brake);

IntegralZone:
talon.confIntegralZone(constans.KSlotIdex, 30);



○ Configure FeedForward this term is used to tell the system how to scale native units per
100ms to 100% output
■ F-gain = (100% X 1023) / (measured units per 100ms @100%)
○ Set Motion Magic Cruise Velocity And Acceleration
■ Cruise-Velocity = (arbitrarily% * measured units per 100ms @100%)
■ Acceleration = (Cruise-Velocity / (seconds * Cruise-Velocity)) * Cruise-Velocity
○ Kp helps the closed-loop react to error
■ P-gain = (% motor output X 1023) / (error)
● Suggested % motor output = 10%
○ Ki closes steady state error when at rest
■ Consider adding starting at .001
● /* Configure Integral Zone */
● _talon.config_IntegralZone(Constants.kSlotIdx, typical_error);

○ kD dampens overshoot
■ Suggested D-gain = kp * 10
○ Finally look at the Arbitrary feedforward to remove gravity





0.1 Selecting the sensor type (see previous Bring-Up sections)

0.2 Confirm motor and sensor health (see previous Bring-Up section on sensor)

0.3 Confirm sensor phase (see previous Bring-Up sections)
positive command with positve motor direction and positive sensor phase


1. Gravity Offset (Elevator)
In the case of a traditional elevator mechanism, there is a constant force due to gravity affecting the mechanism. Because the force is constant, we can determine a constant offset to keep the elevator at position when error is zero.

Use either the Phoenix Tuner Control Tab or Joystick control in your robot code to apply output to the elevator until it stays at a position without moving. Use Phoenix Tuner (plotter or Self-test Snapshot) to measure the output value - this is the Arbitrary Feed Forward value needed to offset gravity.

If we measure a motor output of 7% to keep position, then our java code for Arbitrary Feed Forward with Motion Magic would look like this:

double feedforward = 0.07;
_motorcontroller.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, feedforward);

If your elevator mechanism will change weight while in use (i.e. pick up a heavy game piece), it is helpful to measure gravity offsets at each expected weight and switch between Arbitrary Feed Forward values as needed.


Setting Arbitrary Feed Forward
Arbitrary Feed Forward is passed as an optional parameter in a set() call or VI. The value must be set on every call, just like the primary set value.


2. Calculating Velocity Feed Forward gain (kF)

The velocity feed forward (kF) is different from the Arbitrary Feed Forward in that it is a specialized feed forward designed to approximate the needed motor output to achieve a specified velocity.

Do I need to calculate kF?
If using any of the control modes, we recommend calculating the kF:

Velocity Closed Loop: kF is multiplied by target velocity and added to output.

Current (Draw) Closed Loop: kF is multiplied by the target current-draw and added to output.

MotionMagic/ MotionProfile / MotionProfileArc: kF is multiplied by the runtime-calculated target and added to output.

When using position closed loop, it is generally desired to use a kF of ‘0’. During this mode target position is multiplied by kF and added to motor output. If providing a feedforward is necessary, we recommend using the arbitrary feed forward term (4 param Set) to better implement this.

How to calculate kF
Using Tuner (Self-test Snapshot or Plotter), we’ve measured a peak velocity of 9326 native units per 100ms at 100% output. This can also be retrieved using getSelectedSensorVelocity (routine or VI).

However, many mechanical systems and motors are not perfectly linear (though they are close). To account for this, we should calculate our feed forward using a measured velocity around the percent output we will usually run the motor.

For our mechanism, we will typically be running the motor ~75% output. We then use Tuner (Self-test Snapshot or Plotter) to measure our velocity - in this case, we measure a velocity of 7112 native units per 100ms.

Now let’s calculate a Feed-forward gain so that 75% motor output is calculated when the requested speed is 7112 native units per 100ms.

F-gain = (75% X 1023) / 7112 F-gain = 0.1079

Let’s check our math, if the target speed is 7112 native units per 100ms, Closed-loop output will be (0.1079 X 7112) => 767.38 (75% of full forward).

The output of the PIDF controller in Talon/Victor uses 1023 as the “full output”.


3. Bring up plotting interface so you can visually see sensor position and motor output. This can be done via Tuner Plotter, or through LabVIEW/SmartDash/API plotting.



4. Configure gains and closed-loop centric configs.




5. Checkout the relevant example from CTREs GitHub.

6. Set all of your gains to zero. Use either API or Phoenix Tuner.

7. If not using Position-Closed loop mode, set the kF to your calculated value (see previous section).

If using Motion Magic, set your initial cruise velocity and acceleration (section below).

Setting Motion Magic Cruise Velocity And Acceleration
The recommended way to do this is to take your max sensor velocity (previous section).

Suppose your kMaxSensorVelocity is 9326 units per 100ms. A reasonable initial cruise velocity may be half of this velocity, which is 4663.

Config 4663 to be the cruiseVelocity via configMotionCruiseVelocity routine/VI.

Next lets set the acceleration, which is in velocity units per second (where velocity units = change in sensor per 100ms). This means that if we choose the same value of 4663 for our acceleration, than Motion Magic will ensure it takes one full second to reach peak cruise velocity.

In short set the acceleration to be the same 4663 value via configMotionAcceleration routine/VI.

Later you can increase these values based on the application requirements.


8. Deploy the application and use the joystick to adjust your target. Normally this requires holding down a button on the gamepad (to enter closed loop mode).

9. Plot the sensor-position to assess how well it is tracking. This can be done with WPI plotting features, or with Phoenix Tuner.


10. Dialing kP
Next we will add in P-gain so that the closed-loop can react to error. In the previous section, after running the mechanism with just F-gain, the servo appears to settle with an error or ~1400.

Given an error of (~1400.), suppose we want to respond with another 10% of throttle. Then our starting kP would be….

(10% X 1023) / (1400) = 0.0731 Now let’s check our math, if the Talon SRX sees an error of 1400 the P-term will be 1400 X 0.0731= 102 (which is about 10% of 1023) kP = 0.0731

Apply the P -gain programmatically using your preferred method. Now retest to see how well the closed-loop responds to varying loads.

Retest the maneuver by holding button 1 and sweeping the gamepad stick. At the end of this capture, the wheels were hand-spun to demonstrate how aggressive the position servo responds. Because the wheel still back-drives considerably before motor holds position, the P-gain still needs to be increased.

Once settles, the motor is back-driven to assess how firm the motor holds position.

The wheel is held by the motor firmly.


11. Dialing kD
To resolve the overshoot at the end of the maneuver, D-gain is added. D-gain can start typically at 10 X P-gain.

With this change the visual overshoot of the wheel is gone. The plots also reveal reduced overshoot at the end of the maneuver.


12. Dialing kI
Typically, the final step is to confirm the sensor settles very close to the target position. If the final closed-loop error is not quite close enough to zero, consider adding I-gain and I-zone to ensure the Closed-Loop Error ultimately lands at zero (or close enough).

In testing the closed-loop error settles around 20 units, so we’ll set the Izone to 50 units (large enough to cover the typical error), and start the I-gain at something small (0.001).

Keep doubling I-gain until the error reliably settles to zero.

With some tweaking, we find an I-gain that ensures maneuver settles with an error of 0.



13. Mechanism is Finished Command
Often it is necessary to move a mechanism to a setpoint and ensure that it has properly reached its final position before moving on to the next command. A proper implementation requires the following:

waiting long enough to ensure CAN framing has provided fresh data. See setStatusFramePeriod() to modify update rates.

waiting long enough to ensure mechanism has physically settled. Otherwise closed-loop overshoot (due to inertia) will not be corrected.

Warning

If using Motion Magic control mode, robot code should additionally poll getActiveTrajectoryPosition() routine/VI to determine when final target position has been reached. This is because the closed-loop error corresponds how well the position profile is tracking, not when profiled maneuver is complete.

The general requirements are to periodically monitor the closed-loop error provided the following:

The latest closed-loop error (via API).

The threshold that the closed-loop error must be within to be considered acceptable.

How long the closed-loop error has been acceptable.

The threshold of how long the error must be acceptable before moving on to the next command.

An example of this is shown below in Java, within a class that implements the Command interface

int kErrThreshold = 10; // how many sensor units until its close-enough
int kLoopsToSettle = 10; // how many loops sensor must be close-enough
int _withinThresholdLoops = 0;

// Called repeatedly when this Command is scheduled to run
@Override
protected void execute() {
    /* Check if closed loop error is within the threshld */
    if (talon.getClosedLoopError() < +kErrThreshold &&
        talon.getClosedLoopError() > -kErrThreshold) {

        ++_withinThresholdLoops;
    } else {
        _withinThresholdLoops = 0;
    }
}

// Make this return true when this Command no longer needs to run execute()
@Override
protected boolean isFinished() {
    return (_withinThresholdLoops > kLoopsToSettle);
}
Warning

If using Motion Magic control mode, robot code should additionally poll getActiveTrajectoryPosition() routine/VI to determine when final target position has been reached. This is because the closed-loop error corresponds how well the position profile is tracking, not when profiled maneuver is complete.


14



General Closed-Loop Configs:

PID 0 Primary Feedback Sensor:
Selects the sensor source for PID0 closed loop, soft limits, and
value reporting for the SelectedSensor API.


PID 0 Primary Sensor Coefficient:
Scalar (0,1] to multiply selected sensor value before using.
Note this will reduce resolution of the closed-loop.



Closed-Loop configs per slot (four slots available):

kF:
Feed Fwd gain for Closed loop. See documentation for calculation details. If using velocity, motion magic, or motion profile, use (1023 * duty-cycle / sensor-velocity-sensor-units-per-100ms)

kP:
Proportional gain for closed loop. This is multiplied by closed loop error in sensor units. Note the closed loop output interprets a final value of 1023 as full output. So use a gain of ‘0.25’ to get full output if err is 4096u (Mag Encoder 1 rotation)

kI:
Integral gain for closed loop. This is multiplied by closed loop error in sensor units every PID Loop. Note the closed loop output interprets a final value of 1023 as full output. So use a gain of ‘0.00025’ to get full output if err is 4096u (Mag Encoder 1 rotation) after 1000 loops

kD:
Derivative gain for closed loop. This is multiplied by derivative error (sensor units per PID loop). Note the closed loop output interprets a final value of 1023 as full output. So use a gain of ‘250’ to get full output if derr is 4096u per (Mag Encoder 1 rotation) per 1000 loops (typ 1 sec)

Loop Period Ms:
Number of milliseconds per PID loop. Typically, this is 1ms.

Allowable Error:
If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable. Value is in sensor units.

I Zone:
Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too far from the target. This prevent unstable oscillation if the kI is too large. Value is in sensor units.

Max Integral Accum:
Cap on the integral accumulator in sensor units. Note accumulator is multiplied by kI AFTER this cap takes effect.

Peak Output:
Absolute max motor output during closed-loop control modes only. A value of ‘1’ represents full output in both directions.




Motion Magic Closed-Loop Configs
Acceleration:
Motion Magic target acceleration in (sensor units per 100ms) per second.

Cruise Velocity:
Motion Magic maximum target velocity in sensor units per 100ms.

S-Curve Strength:
Zero to use trapezoidal motion during motion magic. [1,8] for S-Curve, higher value for greater smoothing.






