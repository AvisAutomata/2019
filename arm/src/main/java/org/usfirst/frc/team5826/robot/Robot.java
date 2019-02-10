/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5826.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	Encoder wristEncoder = new Encoder(0, 1);
	Encoder armEncoder = new Encoder(2, 3);

	VictorSPX vacuumController1 = new VictorSPX(9);
	VictorSPX vacuumController2 = new VictorSPX(8);
	Compressor compressor = new Compressor(6);
	WPI_TalonSRX leftfront = new WPI_TalonSRX(4);
	WPI_TalonSRX leftback = new WPI_TalonSRX(3);
	WPI_TalonSRX rightfront = new WPI_TalonSRX(2);
	WPI_TalonSRX rightback = new WPI_TalonSRX(1);
	PowerDistributionPanel pdp = new PowerDistributionPanel(0);
	SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftfront, leftback);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightfront, rightback);
	DifferentialDrive myRobot = new DifferentialDrive(leftDrive, rightDrive);
	Talon wristController = new Talon(7);
	Talon armController = new Talon(6);
	Talon hwheel = new Talon(5);
	XboxController armOperator = new XboxController(1);
	Joystick driver = new Joystick(0);
	private Actuator wrist;
	private Actuator arm;

	NetworkTable table;
	private Gyro gyro = new ADXRS450_Gyro();
	LidarLitePWM lidarLeft = new LidarLitePWM(new DigitalInput(5));
	LidarLitePWM lidarRight = new LidarLitePWM(new DigitalInput(4));
	DoubleSolenoid backSolenoid = new DoubleSolenoid(6, 0, 1);
	DoubleSolenoid fowardSolenoid = new DoubleSolenoid(6, 2, 3);

	int count = 0;
	double wristAngle = 0;
	boolean drive = true;
	boolean controller = false;
	double d = 0;
	double left = 0;
	double right = 0;
	double dArea = 0;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		gyro.calibrate();
		wrist = new Actuator(wristController, wristEncoder, 71, 7, 0);
		arm = new Actuator(armController, armEncoder, 100, 20, 0);
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		ledMode.setDouble(1);
	}

	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		wristAngle = wrist.getAngle();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		if (driver.getRawButtonPressed(5)) {
			drive = !drive;
		}

		if (driver.getRawButtonPressed(7)) {
			controller = !controller;
		}

		if (driver.getRawButtonReleased(8)) {
			// table = NetworkTableInstance.getDefault().getTable("limelight");
			// NetworkTableEntry ledMode = table.getEntry("ledMode");
			// ledMode.setDouble(1);
		}

		if (driver.getRawButton(8) || armOperator.getTriggerAxis(Hand.kLeft) > 0.5) {
			approachNew();
			if (driver.getRawButton(3)) {
				hwheel.set(0.5);
			} else if (driver.getRawButton(4)) {
				hwheel.set(-0.5);
			} else {
				hwheel.set(0);
			}
		} else if (controller) {
			myRobot.arcadeDrive(-armOperator.getY(Hand.kLeft) / 2, armOperator.getX(Hand.kLeft) / 2);
			if (armOperator.getBumper(Hand.kLeft)) {
				hwheel.set(0.5);
			} else if (armOperator.getBumper(Hand.kRight)) {
				hwheel.set(-0.5);
			} else {
				hwheel.set(0);
			}
		} else if (drive) {
			myRobot.arcadeDrive(driver.getY() * ((driver.getRawAxis(3) - 1) / 2),
					driver.getX() * -((driver.getRawAxis(3) - 1) / 2));
			if (driver.getRawButton(3)) {
				hwheel.set(0.5);
			} else if (driver.getRawButton(4)) {
				hwheel.set(-0.5);
			} else {
				hwheel.set(0);
			}
		} else {
			followTarget();
		}

		// Start Arm/Wrist
		if (armOperator.getBackButton()) {
			arm.setStartPosition();
			wrist.setStartPosition();
			wristAngle = 0;
		}
		System.out.println("arm = " + arm.getAngle() + " wrist = " + wrist.getAngle() + " Gyro = " + gyro.getAngle());
		if (armOperator.getAButton()) {
			arm.armSet(0);
			wrist.spinTo(0);
			wristAngle = 0;
		} else if (armOperator.getBButton()) {
			arm.armSet(25);
			wrist.spinTo(123);
			wristAngle = 123;
		} else if (armOperator.getXButton()) {
			arm.armSet(67);
			wrist.spinTo(160);
			wristAngle = 160;
		} else if (armOperator.getYButton()) {
			arm.armSet(118);
			wrist.spinTo(212);
			wristAngle = 212;
		} else {
			arm.setSpeed(armOperator.getY(Hand.kRight));
			if (Math.abs(armOperator.getX(Hand.kLeft)) > 0.15 && !controller) {
				wrist.setSpeed(armOperator.getX(Hand.kLeft) / 5);
				wristAngle = wrist.getAngle();
			} else if (Math.abs(armOperator.getX(Hand.kRight)) > 0.15) {
				wrist.setSpeed(armOperator.getX(Hand.kRight) / 5);
				wristAngle = wrist.getAngle();
			} else {
				wrist.spinTo(wristAngle);
			}
		}
		// End Arm/Wrist

		if (!(count % 10 == 0)) {
			right += lidarRight.getDistance();
			left += lidarLeft.getDistance();
		} else {
			d = left/10 - right/10;
			System.out.println("Left distance = " + left / 10);
			System.out.println("Right Distance = " + right / 10);
			left = 0;
			right = 0;
		}
		// if(driver.getRawButtonPressed(11)){
		// 	on = false;
		// }
		// else if(driver.getRawButtonPressed(11) && !on){

		// }


		// Start Vacuums
		if (armOperator.getBumper(Hand.kRight) && !controller) {
			vacuumController1.set(ControlMode.PercentOutput, -0.5);
		} else if (armOperator.getTriggerAxis(Hand.kRight) > 0.5 && !controller) {
			vacuumController1.set(ControlMode.PercentOutput, 0.5);
		} else {
			vacuumController1.set(ControlMode.PercentOutput, 0);
		}

		if (armOperator.getBumper(Hand.kLeft) && !controller) {
			vacuumController2.set(ControlMode.PercentOutput, -1);
		} else {
			vacuumController2.set(ControlMode.PercentOutput, 0);
		}
		// End Vacuums

		if (pdp.getCurrent(4) > 8.6 || (pdp.getCurrent(12) < 10 && pdp.getCurrent(12) > 4)) {
			armOperator.setRumble(RumbleType.kRightRumble, 0.2);
		} else {
			armOperator.setRumble(RumbleType.kRightRumble, 0);
		}
		count++;
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	private void followTarget() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		ledMode.setDouble(0);

		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);
		double area = ta.getDouble(0.0);

		if (area < 1 && target == 1) {
			myRobot.arcadeDrive(0.4, x / Math.abs(x) * Math.min(Math.abs(x / 10), 0.5));
		} else if (area > 1.5 && target == 1) {
			myRobot.arcadeDrive(-0.4, x / Math.abs(x) * (Math.min(Math.abs(x / 10), 0.5)));
		} else {
			myRobot.arcadeDrive(0, 0);
		}
	}

	public void approachNew() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry ts = table.getEntry("ts");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
		ledMode.setDouble(0);

		pipeline.setDouble(0);
		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);
		double area = ta.getDouble(0.0);
		double skew = ts.getDouble(0.0);
		// pipeline.setDouble(1);
		// double aL = ta.getDouble(0.0);
		// pipeline.setDouble(2);
		// double aR = ta.getDouble(0.0);
		// pipeline.setDouble(0);
		
		myRobot.arcadeDrive(-driver.getY(), x/Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));

		// System.out.println("Area = " + area);
		// System.out.println("Skew = " + skew);
		// System.out.println(aL + " Area Left");
		// System.out.println(aR + " Area Right");
		// if (target == 1 && Math.abs(x) > 3) {
		// 	myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
		// 	hwheel.set(0);
		// } else if (Math.abs(d) > 2 && Math.abs(d) < 100) {
		// 	hwheel.set(d / Math.abs(d) * Math.min(Math.abs(d), 0.4));
		// 	myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
		// } else {
		// 	hwheel.set(0);
		// 	myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
		// }
	}

	public void approachNewer(){
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry ts = table.getEntry("ts");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
		ledMode.setDouble(0);

		pipeline.setDouble(0);
		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);
		double area = ta.getDouble(0.0);
		double skew = ts.getDouble(0.0);
		double dif = Math.round(d);
		if(count%30 == 0){
			dArea = area;
		}

		if (target == 1 && Math.abs(x) > 3) {
			myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
			hwheel.set(0);
		} else if (area - dArea > 0.2) {
			hwheel.set(0.3);
			myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
		} else if(area - dArea < -0.2){
			hwheel.set(-0.3);
			myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
		}
		else {
			hwheel.set(0);
			myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
			dArea = area;
		}
	}
}