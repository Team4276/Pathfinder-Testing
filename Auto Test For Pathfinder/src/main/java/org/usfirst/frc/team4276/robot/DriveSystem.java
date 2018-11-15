
package org.usfirst.frc.team4276.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.followers.EncoderFollower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSystem {

	private EncoderFollower leftFollower;
	private EncoderFollower rightFollower;

	private Encoder leftBaseEncoder;
	private Encoder rightBaseEncoder;

	private VictorSPX leftBaseSp1;
	private VictorSPX leftBaseSp2;
	private VictorSPX rightBaseSp1;
	private VictorSPX rightBaseSp2;

	private final double MAX_ACCELERATION = 24.130538058; // ft/s/s
	private final double MAX_VELOCITY = 7.2289156627297; // ft/s
	private final double MAX_JERK = 60.0; // ft/s/s/s
	private final double WHEEL_BASE = 2.132546; // ft
	private double kP = 0.0;
	private double kI = 0.0;
	private double kD = 0.0;
	private double kV = 1 / MAX_VELOCITY;
	private double kA = 0.002;
	private final double dT = (1 / Robot.operatingFrequency);
	private final int TickPRev = 1907;
	private final double wheelDiam = 0.5249344; // ft

	public Trajectory testTrajectory;
	public Trajectory left;
	public Trajectory right;

	public DriveSystem(int R_MOT_1, int L_MOT_1, int R_MOT_2, int L_MOT_2, int L_ENC_A, int L_ENC_B, int R_ENC_A,
			int R_ENC_B) {

		leftBaseSp1 = new VictorSPX(L_MOT_1);
		leftBaseSp2 = new VictorSPX(L_MOT_2);
		rightBaseSp1 = new VictorSPX(R_MOT_1);
		rightBaseSp2 = new VictorSPX(R_MOT_2);

		leftBaseEncoder = new Encoder(L_ENC_A, L_ENC_B);
		rightBaseEncoder = new Encoder(R_ENC_A, R_ENC_B);

	}

	public void executeManualDrive() {
		setLeftMotors(Robot.lJoy.getY());
		setRightMotors(Robot.rJoy.getY());
		tuneControlGains();
	}

	private void setRightMotors(double power) {
		rightBaseSp1.set(ControlMode.PercentOutput, power);
		rightBaseSp2.set(ControlMode.PercentOutput, power);
	}

	private void setLeftMotors(double power) {
		leftBaseSp1.set(ControlMode.PercentOutput, power);
		leftBaseSp2.set(ControlMode.PercentOutput, power);
	}

	public void tuneControlGains() {
		// feed forward constants on Right Joystick
		if (Robot.rJoy.getRawButton(7) == true) {
			kA = kA + 10e-3;
		}
		if (Robot.rJoy.getRawButton(8) == true) {
			kA = kA - 10e-3;
		}
		if (Robot.rJoy.getRawButton(9) == true) {
			kV = kV + 1e-3;
		}
		if (Robot.rJoy.getRawButton(10) == true) {
			kV = kV - 1e-3;
		}
		// feed back constants on Left Joystick
		if (Robot.lJoy.getRawButton(7) == true) {
			kP = kP + 10e-3;
		}
		if (Robot.lJoy.getRawButton(8) == true) {
			kP = kP - 10e-3;
		}
		if (Robot.lJoy.getRawButton(9) == true) {
			kI = kI + 1e-3;
		}
		if (Robot.lJoy.getRawButton(10) == true) {
			kI = kI - 1e-3;
		}
		if (Robot.lJoy.getRawButton(11) == true) {
			kD = kD + 1e-3;
		}
		if (Robot.lJoy.getRawButton(12) == true) {
			kD = kD - 1e-3;
		}
		SmartDashboard.putNumber("Elevator kA", kA);
		SmartDashboard.putNumber("Elevator kV", kV);
		SmartDashboard.putNumber("Elevator Kp*1e-3", kP * 1e3);
		SmartDashboard.putNumber("Elevator Ki*1e-3", kI * 1e3);
		SmartDashboard.putNumber("Elevator Kd*1e-3", kD * 1e3);
	}

	public void initializeTrajectory(Waypoint[] argRoute) {

		// initializes default configuration for trajectories
		Trajectory.Config defaultConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, dT, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);

		// creates trajectory to follow
		testTrajectory = Pathfinder.generate(argRoute, defaultConfig);

		// modifies trajectories for driveBases
		TankModifier modifier = new TankModifier(testTrajectory).modify(WHEEL_BASE);
		left = modifier.getLeftTrajectory();
		right = modifier.getRightTrajectory();

		// sets up encoderFollower objects to follow trajectories
		leftFollower = new EncoderFollower(left);
		rightFollower = new EncoderFollower(right);

		// configures Gains
		leftFollower.configurePIDVA(kP, kI, kD, kV, kA);
		rightFollower.configurePIDVA(kP, kI, kD, kV, kA);

		// calibrates for accurate encoder readings
		leftFollower.configureEncoder(leftBaseEncoder.getRaw(), TickPRev, wheelDiam);
		rightFollower.configureEncoder(rightBaseEncoder.getRaw(), TickPRev, wheelDiam);
	}

	public void executeAutoDrive() {

		double LPower = leftFollower.calculate(leftBaseEncoder.getRaw());
		double RPower = rightFollower.calculate(rightBaseEncoder.getRaw());

		double gyro_heading = 0; // Assuming the gyro is giving a value in degrees
		double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

		setLeftMotors(LPower + turn);
		setRightMotors(RPower - turn);
	}
}