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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Scheduler;
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
	//PowerDistributionPanel pdp = new PowerDistributionPanel(5);
	//This broke the robot sorry victor
	//TODO Make sure pdp is on 5
	WPI_TalonSRX leftfront = new WPI_TalonSRX(4);
	WPI_TalonSRX leftback = new WPI_TalonSRX(3);
	WPI_TalonSRX rightfront = new WPI_TalonSRX(2);
	WPI_TalonSRX rightback = new WPI_TalonSRX(1);
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
	private Arm armSS;

	NetworkTable table;
	private Gyro gyro = new ADXRS450_Gyro();
	LidarLitePWM lidarLeft = new LidarLitePWM(new DigitalInput(8));
	LidarLitePWM lidarRight = new LidarLitePWM(new DigitalInput(9));
	DoubleSolenoid backSolenoid = new DoubleSolenoid(6, 6, 7);
	DoubleSolenoid fowardSolenoid = new DoubleSolenoid(6, 4, 5);

	int count = 0;
	double wristAngle = 0;
	boolean drive = true;
	double d = 0;
	double left = 0;
	double right = 0;
	double dArea = 0;

	public boolean vacuum1 = false;
	public boolean vacuum2 = false;

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
		armSS = new Arm(arm, wrist, armOperator);
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		ledMode.setDouble(1);
	}

	@Override
	public void autonomousInit() {
		teleopInit();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		teleopPeriodic();
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

		if(armOperator.getStartButton()){
			gyro.reset();
		}

		if(Math.abs(gyro.getAngle()) > 360){
			gyro.reset();
		}

		if (driver.getRawButtonReleased(8) || driver.getRawButtonReleased(5)) {
			table = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry ledMode = table.getEntry("ledMode");
			ledMode.setDouble(1);
		}

		if (driver.getRawButton(8)) {
			approach();
		}
		else if (drive) {
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
		//System.out.println("arm = " + arm.getAngle() + " wrist = " + wrist.getAngle() + " Gyro = " + gyro.getAngle());
		if (armOperator.getAButton()) {
			wristAngle = armSS.moveStation(vacuum1, vacuum2);
		} else if (armOperator.getBButton()) {
			wristAngle = armSS.moveBottom(vacuum1, vacuum2);
		} else if (armOperator.getYButton()) {
			wristAngle = armSS.moveMid(vacuum1, vacuum2);
		} else if (armOperator.getYButton()){
			wristAngle = armSS.moveTop(vacuum1, vacuum2);
		}
		else {
			if(arm.getAngle() < 120){
				arm.setSpeed(armOperator.getY(Hand.kRight));
			} else{
				arm.setSpeed(Math.max(armOperator.getY(Hand.kRight),0));
			} 
			if (Math.abs(armOperator.getY(Hand.kLeft)) > 0.15){
				wrist.setSpeed(armOperator.getY(Hand.kLeft)/3);
				wristAngle = wrist.getAngle();
			} else {
			
				wrist.spinTo(wristAngle);
			}
		}
		// End Arm/Wrist

		// if (!(count % 10 == 0)) {
		// 	right += lidarRight.getDistance();
		// 	left += lidarLeft.getDistance();
		// } else {
		// 	d = left/10 - right/10;
		// 	System.out.println("Left distance = " + left / 10);
		// 	System.out.println("Right Distance = " + right / 10);
		// 	left = 0;
		// 	right = 0;
		// }
		// if(driver.getRawButtonPressed(11)){
		// 	on = false;
		// }
		// else if(driver.getRawButtonPressed(11) && !on){

		// }

		// Start Vacuums
	/* 	if (armOperator.getBumper(Hand.kRight)) {
			vacuumController1.set(ControlMode.PercentOutput, -0.50);
		} else if (armOperator.getTriggerAxis(Hand.kRight) > 0.5) {
			vacuumController1.set(ControlMode.PercentOutput, 0.5);
		} else {
			vacuumController1.set(ControlMode.PercentOutput, 0);
		} */

		//Start Disk Vacuum
		if(armOperator.getBumper(Hand.kRight)) {
			
			vacuum1 = true;

		} 

		if(armOperator.getTriggerAxis(Hand.kRight) > 0.5) {

			vacuum1 = false;

		}

		if(vacuum1 == true) {
			vacuumController1.set(ControlMode.PercentOutput, -0.75);
		} else {
			vacuumController1.set(ControlMode.PercentOutput, 0);
		}
		//End Disk Vacuum

		// Start Ball Vacuum
		if(armOperator.getBumper(Hand.kLeft)) {
			
			vacuum2 = true;

		} 

		if(armOperator.getTriggerAxis(Hand.kLeft) > 0.5) {

			vacuum2 = false;

		}
		
		if(vacuum2) {
			vacuumController2.set(ControlMode.PercentOutput, -1);
		} else {
			vacuumController2.set(ControlMode.PercentOutput, 0);
		}
		//End Ball Vacuum

		// if (pdp.getCurrent(4) > 8 || (pdp.getCurrent(12) < 10 && pdp.getCurrent(12) > 4)) {
		// 	armOperator.setRumble(RumbleType.kRightRumble, 0.2);
		// } else {
		// 	armOperator.setRumble(RumbleType.kRightRumble, 0);
		// }
		// End Vacuums
		//Start Solenoids
		int pov = armOperator.getPOV();
		if (pov >= 270 || (pov <= 90 && pov != -1)) {
			fowardSolenoid.set(Value.kForward);//DOWN 
		}
		else{
			fowardSolenoid.set(Value.kReverse);//UP
		}
		if (pov < 270 && pov > 90) {
			backSolenoid.set(Value.kForward);//DOWN
		}
		else{
			backSolenoid.set(Value.kReverse);//UP
		}
		
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

	public void approach (){
	//	System.out.println(lidarRight.getDistance());
		//space = dist between robot and board
		double space = 80;
		//Limelight mount offset
		double offset = 5;
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		ledMode.setDouble(0);
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
		pipeline.setDouble(0);
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry tv = table.getEntry("tv");
		double x = tx.getDouble(0.0) - offset;
		double target = tv.getDouble(0.0);
		double leftDist = lidarLeft.getDistance();
		double rightDist = lidarRight.getDistance();
		double mindist = Math.min(leftDist, rightDist);
		double maxdist = Math.max(leftDist, rightDist);
		double curve = (leftDist - rightDist) / (maxdist * 1.25);

		

		if(target == 1){
			hwheel.set(-x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
			if (mindist > 2 * space){
				myRobot.arcadeDrive(.7, curve);
			}
			else if (mindist >= space){
				myRobot.arcadeDrive(.25, curve);
			}
			else if (mindist < 2 + space && mindist > space - 2){
				myRobot.arcadeDrive(0, curve);
			}
			else if (mindist < space - 2 && mindist > 0){
				myRobot.arcadeDrive(-.25, curve);
			}

		}
	}
	// public void approachNew() {
	// 	table = NetworkTableInstance.getDefault().getTable("limelight");
	// 	NetworkTableEntry ledMode = table.getEntry("ledMode");
	// 	NetworkTableEntry tx = table.getEntry("tx");
	// 	NetworkTableEntry ta = table.getEntry("ta");
	// 	NetworkTableEntry tv = table.getEntry("tv");
	// 	NetworkTableEntry ts = table.getEntry("ts");
	// 	NetworkTableEntry pipeline = table.getEntry("pipeline");
	// 	ledMode.setDouble(0);

	// 	pipeline.setDouble(0);
	// 	double target = tv.getDouble(0.0);
	// 	double x = tx.getDouble(0.0);
	// 	double area = ta.getDouble(0.0);
	// 	double skew = ts.getDouble(0.0);

	// 	myRobot.arcadeDrive(-driver.getY(), (Math.abs(gyro.getAngle())-180)/Math.abs(Math.abs(gyro.getAngle())-180)*Math.min(gyro.getAngle()+180, 0.33));

	// 	// if (target == 1 && Math.abs(x) > 0.5) {
	// 	// 	myRobot.arcadeDrive(-driver.getY(), (gyro.getAngle() + 180)/20);
	// 	// 	hwheel.set(x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 	// 	// myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 	// } else {
	// 	// 	hwheel.set(0);
	// 	// 	myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 	// }
	// }

	// public void approachNewer(){
	// 	table = NetworkTableInstance.getDefault().getTable("limelight");
	// 	NetworkTableEntry ledMode = table.getEntry("ledMode");
	// 	NetworkTableEntry tx = table.getEntry("tx");
	// 	NetworkTableEntry ta = table.getEntry("ta");
	// 	NetworkTableEntry tv = table.getEntry("tv");
	// 	NetworkTableEntry ts = table.getEntry("ts");
	// 	NetworkTableEntry pipeline = table.getEntry("pipeline");
	// 	ledMode.setDouble(0);

	// 	pipeline.setDouble(0);
	// 	double target = tv.getDouble(0.0);
	// 	double x = tx.getDouble(0.0);
	// 	double area = ta.getDouble(0.0);
	// 	double skew = ts.getDouble(0.0);
	// 	double dif = Math.round(d);
	// 	if(count%30 == 0){
	// 		dArea = area;
	// 	}

	// 	if (target == 1 && Math.abs(x) > 3) {
	// 		myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 		hwheel.set(0);
	// 	} else if (area - dArea > 0.2) {
	// 		hwheel.set(0.3);
	// 		myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 	} else if(area - dArea < -0.2){
	// 		hwheel.set(-0.3);
	// 		myRobot.arcadeDrive(-driver. getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 	}
	// 	else {
	// 		hwheel.set(0);
	// 		myRobot.arcadeDrive(-driver.getY(), x / Math.abs(x) * Math.min(Math.abs(x / 2), 0.33));
	// 		dArea = area;
	// 	}
	// }
}