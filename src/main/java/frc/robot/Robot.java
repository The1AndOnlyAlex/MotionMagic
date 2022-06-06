/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon SRX Software Reference Manual.
 * 
 * Controls:
 * Button 1(Button A): When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2(Button B): When pushed, the selected feedback sensor gets zero'd
 * POV 180(Dpad Down): When pushed, will decrement the smoothing of the motion magic down to 0
 * POV 0(Dpad Up): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon SRX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Magic: Servo Talon SRX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
 /*
 Step A
1. Set the _talon CAN address
2. Set the port for the _joystick
3. Commment out or set any followers that maybe in the System
4. Config selected Feeback sensor
5. Adjust sensor and motor phase
6. Zero all PIDF values
7. Zero Cruise Velocity and acceleration
 */
/*
Step B (optinal)
_talon.configClearPositionOnLmitR(true, Constants.kTimeoutMs);
_talon.configFeedbackNotContinuous(true, Constants,kTimeoutMs);

_talon.configPeakCurrentLimit(30);
_talon.configPeakCurrentDuration(150);
_talon.configContinuousCurrentLimit(20);

_talon.setNeutralMode(NeutralMode.Brake);
_talon.configIntegralZone(Constants.kSlotIdx, 30);
*/
/*
Step C
1. Add position data output/print out
2. Add min and max position limits (estmation)
*/
/* Step D
Deploy Code and Verify parameters (Free run, without Motion Magic Ctrl)
1. Verify sensor and motor phase (Joystick/Motor; ouput/position-Tuner)
2. Adjust min and max position limits
3. Collect the max speed, push button to high speed, read the max Vel value on console
4. Calculate the F-gain / kF on excel form
5. Calculate the acceleration and vcruise velocity on excel form
*/
/*
Step E
With Motion Magic Ctrl (different ctrl buttons)
1. Collect the Error, calculate kP on excel form
2. calculate the kD on excel form
3. kI
*/
/*
Step F
TBD
*/
/* 
Step G
TBD
*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
	double m_targetMin = 10; //C.2 D.2
	double m_targetMax = 700; //C.2 D.2

	/* Hardware */
	WPI_TalonSRX _talon = new WPI_TalonSRX(1); //A.1
	Joystick _joy = new Joystick(0); //A.2

	/* create some followers */
	//A.3 BaseMotorController _follower1 = new WPI_TalonSRX(0);
	//A.3 BaseMotorController _follower2 = new WPI_VictorSPX(0);
	//A.3 BaseMotorController _follower3 = new WPI_VictorSPX(1);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public void simulationInit() {
		PhysicsSim.getInstance().addTalonSRX(_talon, 0.75, 5100, false);
	}
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public void robotInit() {
		/* setup some followers */
		//A.3 _follower1.configFactoryDefault();
		//A.3 _follower2.configFactoryDefault();
		//A.3 _follower3.configFactoryDefault();
		//A.3 _follower1.follow(_talon);
		//A.3 _follower2.follow(_talon);
		//A.3 _follower3.follow(_talon);

		/* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs); //A.4

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(true); //A.5 //D.1
		_talon.setInverted(false); //A.5 //D.1

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		// make sure the setpoints enabled before kI is enabled
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, 0, Constants.kTimeoutMs); //A.6 D.4
		_talon.config_kP(Constants.kSlotIdx, 0, Constants.kTimeoutMs); //A.6
		_talon.config_kI(Constants.kSlotIdx, 0, Constants.kTimeoutMs); //A.6
		_talon.config_kD(Constants.kSlotIdx, 0, Constants.kTimeoutMs); //A.6

		/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(0, Constants.kTimeoutMs); //A.7 D.5
		_talon.configMotionAcceleration(0, Constants.kTimeoutMs); //A.7 D.5

		/* Zero the sensor once on robot boot up */ // set current position as the 0, we don't want it happen
		//comment it out if use the Analog Encoder
		//A.4 _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		//B.1 _talon.configClearPositionOnLimitR(true, Constants.kTimeoutMs);
		// not roll over rom 1096 to 0 ...
		_talon.configFeedbackNotContinuous(true, Constants.kTimeoutMs); //B.1

		_talon.configPeakCurrentLimit(30); //B.1
		_talon.configPeakCurrentDuration(150); //B.1
		_talon.configContinuousCurrentLimit(20); //B.1

		//B.1 _talon.setNeutralMode(NeutralMode.Brake);
		_talon.config_IntegralZone(Constants.kSlotIdx, 3); //B.1
	}

	/*
	@Override
	public void teleopInit()
	{
		// for MotionMagic reenable if the prevous position was in the middle
		_talon.set(ControlMode.PercentOutput, 0);
	}*/ //G

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY(); /* left-side Y for Xbox360Gamepad */
		double rghtYstick = -1.0 * _joy.getRawAxis(5); /* right-side Y for Xbox360Gamepad */
		if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
		if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */

		/* Get current Talon SRX motor output */
		double motorOutput = _talon.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		// use it to get m_targetMin and m_targetMax postions from console
		_sb.append("\tPosition:"); //C.1
		_sb.append(_talon.getSelectedSensorPosition()); //C.1
		SmartDashboard.putNumber("Lift Position", _talon.getSelectedSensorPosition()); //C.1

		/* Arbitrary Feed Forward */
		// how much power is needed to hold the motor
		// use joystick to run slow speed to get the estmated value
		double horizontalHoldOutput = 0.1; //F
		double arbFeedFwdTerm = getFeedForward(horizontalHoldOutput); // for arm //F

		/**
		 * Perform Motion Magic when Button 1 is held, else run Percent Output, which can
		 * be used to confirm hardware setup.
		 */
		if (_joy.getRawButton(1)) {
			/* Motion Magic */

			/* 4096 ticks/rev * 10 Rotations in either direction */
			//C.2 double targetPos = rghtYstick * 4096 * 10.0;
			double targetPos = m_targetMin; //C.2
			_talon.set(ControlMode.MotionMagic, targetPos); //F TBD
			//DemandType.ArbitraryFeedForward, horizontalHouldOupt);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		}
		else if(_joy.getRawButton(4)) //C.2
		{
			double targetPos = m_targetMax; //C.2
			_talon.set(ControlMode.MotionMagic, targetPos); //F TBD
			//DemandType.ArbitraryFeedForward, horizontalHouldOupt);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} 
		else { // free run
			/* Percent Output */

			_talon.set(ControlMode.PercentOutput, leftYstick);
		}
		if (_joy.getRawButton(2)) {
			/* Zero sensor positions */
			_talon.setSelectedSensorPosition(0);
		}

		int pov = _joy.getPOV();
		if (_pov == pov) {
			/* no change */
		} else if (_pov == 180) { // D-Pad down
			/* Decrease smoothing */
			_smoothing--;
			if (_smoothing < 0)
				_smoothing = 0;
			_talon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		} else if (_pov == 0) { // D-Pad up
			/* Increase smoothing */
			_smoothing++;
			if (_smoothing > 8)
				_smoothing = 8;
			_talon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		_pov = pov; /* save the pov value for next time */

		/* Instrumentation */
		Instrum.Process(_talon, _sb);
	}

	private double getFeedForward(double horizontalHoldoutput)
	{
		double theta = Math.toRadians(90 - my_getCurrentArmAngle());
		double gravityCompensation = Math.cos(theta);
		double arb_feedForward = gravityCompensation * horizontalHoldoutput;
		return arb_feedForward;
	}

	private double my_getCurrentArmAngle()
	{
		double m_fullRange = 150;
		double m_offset = -2.3;
		double m_sensorPos = _talon.getSelectedSensorPosition(0);
		double m_angle = (m_sensorPos / 1024) * m_fullRange + m_offset;
		return m_angle;
	}
}