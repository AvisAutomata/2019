/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5826.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
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
	Encoder leftEncoder = new Encoder(0, 1);
	WPI_TalonSRX rightfront = new WPI_TalonSRX(2);
	WPI_TalonSRX rightback = new WPI_TalonSRX(1);
	Encoder rightEncoder = new Encoder(2, 3);
	SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftfront, leftback);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightfront, rightback);
	DifferentialDrive myRobot = new DifferentialDrive(leftDrive, rightDrive);
	NetworkTable table;
	private LidarLitePWM lidar = new LidarLitePWM(new DigitalInput(6));
	Talon hwheel = new Talon(1);
	Joystick stick = new Joystick(0);
	Spark led = new Spark(8);
	boolean drive = true;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
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

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopInit() {
		
		
	}
	
	@Override
	public void teleopPeriodic() {

		
		if (stick.getRawButtonPressed(5)) {
			drive = !drive;
		}
		if (stick.getRawButton(2)) {
			approach();
		}		
		else if (stick.getRawButton(6)) {
			approachArea();
		}
		else if(stick.getTrigger()){
			approachNew();
		}
		else if(drive) {
			if (stick.getRawButton(3)) {
				hwheel.set(.4);
				myRobot.arcadeDrive(-stick.getY(), stick.getX());
			} 
			else if (stick.getRawButton(4)) {
				hwheel.set(-.4);
				myRobot.arcadeDrive(-stick.getY(), stick.getX());
			}
			else {
				myRobot.arcadeDrive(-stick.getY(), stick.getX());
				hwheel.set(0);
			}
		}
		
		else {
			hwheel.set(-stick.getX());
			myRobot.arcadeDrive(-stick.getY(), stick.getZ());
		}
//		System.out.println(-stick.getZ());

		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
//		NetworkTableEntry tv = table.getEntry("tv");
//		double v = tv.getDouble(0.0);
		
		// if(lidar.getDistance() > 20) {
    	// 	led.set(0.77);
		// }
		// else {
		// 	led.set(0.61);
		// }
		led.set(Math.min(lidar.getDistance()/50+ 0.57,0.99));
	}
	
	
	public void approach() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry ts = table.getEntry("ts");
		NetworkTableEntry tl = table.getEntry("tl");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
//		System.out.println("Starting dumb stuff");
//		for(char i = 'a'; i <= 'z'; i++) {
//			for(char j = 'a'; j <= 'z'; j++) {
//				String s = "" + i + j;
//				NetworkTableEntry entry = table.getEntry(s);
//				if(entry != null && entry.getDouble(-1) != -1) {
//					System.out.println(s + ": " + entry.getName() + " - " + entry.getDouble(-1));
//					
//				}
//			}
//		}
//		System.out.println("Ending dumb stuff");

		pipeline.setDouble(0);
		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double a = ta.getDouble(0.0);
		double v = tv.getDouble(0.0);
		double s = ts.getDouble(0.0);
		double l = tl.getDouble(0.0);

//		System.out.println(x + "Angle Horizontal");
//		System.out.println(y + "Angle Vertical");
//		System.out.println(a + "Area");
//		System.out.println(v + "Target");
		System.out.println(s + "Skew");
		// System.out.println(l + "Lateny");
		System.out.println(lidar.getDistance() + "Distance");
		double forwardSpeed = 0;
		double rotation = 0;
		double speed = 0;
		if(v==1) {
		if (x > 0.5) {
			speed = Math.max((-x)/15, -0.25);
		} else if (x < -0.5) {
			speed = Math.min((-x)/15 , 0.25);
		} else {
			speed = 0;
		}
		if (s < -80 && s > -90) {
				rotation = -0.33;
		} else if (s > -45 && s < 0) {
				rotation = 0.33;
		}
		if(lidar.getDistance() > 20 && v == 1) {
			forwardSpeed = 0;
		}

		myRobot.arcadeDrive(forwardSpeed, rotation);
		hwheel.set(speed);
		}
	}

	public void approachNew() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry ts = table.getEntry("ts");

		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);
		double area = ta.getDouble(0.0);
		double skew = ts.getDouble(0.0);

		if(target == 1 && skew < -45){
			myRobot.arcadeDrive(0, x/Math.abs(x)*Math.min(Math.abs(x/20), 0.5));
			hwheel.set(0.5);
		}
		else if(target==1){
			myRobot.arcadeDrive(0, x/Math.abs(x)*Math.min(Math.abs(x/20), 0.5));
			hwheel.set(-0.5);
		}
		else{
			myRobot.arcadeDrive(0, x/Math.abs(x)*Math.min(Math.abs(x/20), 0.5));
			hwheel.set(0);
		}
	}
	
	public void approachArea() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry ts = table.getEntry("ts");
		NetworkTableEntry tl = table.getEntry("tl");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
//		System.out.println("Starting dumb stuff");
//		for(char i = 'a'; i <= 'z'; i++) {
//			for(char j = 'a'; j <= 'z'; j++) {
//				String s = "" + i + j;
//				NetworkTableEntry entry = table.getEntry(s);
//				if(entry != null && entry.getDouble(-1) != -1) {
//					System.out.println(s + ": " + entry.getName() + " - " + entry.getDouble(-1));
//					
//				}
//			}
//		}
//		System.out.println("Ending dumb stuff");

		pipeline.setDouble(0);
		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double s = ts.getDouble(0.0);
		double v = tv.getDouble(0.0);
		double l = tl.getDouble(0.0);
		pipeline.setDouble(1);
		double aL = ta.getDouble(0.0);
		pipeline.setDouble(2);
		double aR = ta.getDouble(0.0);


//		System.out.println(x + "Angle Horizontal");
//		System.out.println(y + "Angle Vertical");
		System.out.println(aL + " Area Left");
		System.out.println(aR + " Area Right");
//		System.out.println(v + "Target");
		System.out.println(s + " Skew");
		// System.out.println(l + "Lateny");
		System.out.println(lidar.getDistance() + "Distance");
		double forwardSpeed = 0;
		double rotation = 0;
		double speed = 0;
		if(v==1) {
		if (x > 0.5) {
			speed = Math.max((-x)/15, -0.25);
		} else if (x < -0.5) {
			speed = Math.min((-x)/15 , 0.25);
		} else {
			speed = 0;
		}
		if (aR > aL+0.1*aL*aL) {
				rotation = -0.33;
		} else if (aL > aR+0.1*aR*aR) {
				rotation = 0.33;
		}
		if(lidar.getDistance() > 20 && v == 1) {
			forwardSpeed = 0;
		}

		myRobot.arcadeDrive(forwardSpeed, rotation);
		hwheel.set(speed);
	}
	}
}
