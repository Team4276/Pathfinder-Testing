/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * <p>
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 *
 * <p>
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {

	public static Joystick lJoy, rJoy;
	DriveSystem mDriveSystem;

	public static Timer systemTimer;

	public static final double operatingFrequency = 50; // Hz
	private static double currentTimeStamp = 0, lastTimeStamp = 0;
	private boolean autoInit = true;

	public Robot() {
		systemTimer = new Timer();
		lJoy = new Joystick(0);
		rJoy = new Joystick(1);

		mDriveSystem = new DriveSystem(RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_R2,
				RoboRioPorts.CAN_DRIVE_L2, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B,
				RoboRioPorts.DIO_DRIVE_RIGHT_A, RoboRioPorts.DIO_DRIVE_RIGHT_B);
	}

	@Override
	public void robotInit() {
		mDriveSystem.tuneControlGains();
	}

	@Override
	public void autonomous() {

		if (autoInit) {
			mDriveSystem.initializeTrajectory(WaypointList.route1);
			autoInit = false;
		}

		currentTimeStamp = systemTimer.get();
		if ((currentTimeStamp - lastTimeStamp) >= (1 / operatingFrequency)) {
			mDriveSystem.executeAutoDrive();
		}
	}

	@Override
	public void operatorControl() {
		mDriveSystem.executeManualDrive();
		Timer.delay(0.005);

	}

	@Override
	public void test() {
	}
}
