/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5826.robot;

import java.util.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	WPI_TalonSRX leftfront = new WPI_TalonSRX(4);
	WPI_TalonSRX leftback = new WPI_TalonSRX(3);
	WPI_TalonSRX rightfront = new WPI_TalonSRX(2);
	WPI_TalonSRX rightback = new WPI_TalonSRX(1);
	SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftfront, leftback);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightfront, rightback);
	DifferentialDrive myRobot = new DifferentialDrive(leftDrive, rightDrive);
	Joystick stick = new Joystick(0);
	WPI_TalonSRX hwheel = new WPI_TalonSRX(5);
	NetworkTable table;
	double speed = 0;
	

	
	public void robotInit() {
		leftDrive.setInverted(true);
		rightDrive.setInverted(true);
		CameraServer.getInstance().startAutomaticCapture();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {

		
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		
	}

	public void teleopInit() {
		
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		myRobot.arcadeDrive(stick.getY(), -stick.getX());
		if(stick.getRawButton(3)) {
			if(speed < .75) {
				speed += .05;
			}
			
			hwheel.set(speed);
		}
		else if(stick.getRawButton(4)) {
			if(speed > -.75) {
				speed -= .05;
			}
			hwheel.set(speed);
		}
		else {
			speed = 0;
			hwheel.set(0);
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
