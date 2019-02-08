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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	WPI_TalonSRX wristController = new WPI_TalonSRX(5);
	VictorSPX armController = new VictorSPX(7);
	Encoder wristEncoder = new Encoder(0, 1);
	WPI_TalonSRX leftfront = new WPI_TalonSRX(4);
	WPI_TalonSRX leftback = new WPI_TalonSRX(3);
	WPI_TalonSRX rightfront = new WPI_TalonSRX(2);
	WPI_TalonSRX rightback = new WPI_TalonSRX(1);
	SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftfront, leftback);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightfront, rightback);
	DifferentialDrive myRobot = new DifferentialDrive(leftDrive, rightDrive);
	Talon hwheel = new Talon(1);
	Encoder armEncoder = new Encoder(2, 3);
	int phase = 0;
	private Actuator wrist;
	private Actuator arm;
	Joystick driver = new Joystick(0);
	XboxController armOperator = new XboxController(1);
	double wristAngle = 0;
	Talon vacuumController1 = new Talon(0);
	VictorSPX vacuumController2 = new VictorSPX(8);
	boolean drive = true;
	boolean controller = false;
	NetworkTable table;
	PowerDistributionPanel pdp = new PowerDistributionPanel(6);


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
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

		if(driver.getRawButtonReleased(8)){
			table = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry ledMode = table.getEntry("ledMode");
			ledMode.setDouble(1);
		}

		if(driver.getRawButton(8)){
			approachNew();
		}
		else if(controller){
			myRobot.arcadeDrive(-armOperator.getY(Hand.kLeft)/2, armOperator.getX(Hand.kLeft)/2);
			if(armOperator.getBumper(Hand.kLeft)) {
				hwheel.set(0.5);
			}
			else if(armOperator.getBumper(Hand.kRight)) {
				hwheel.set(-0.5);
			}
			else {
				hwheel.set(0);
			}
		}
		else if(drive){
			myRobot.arcadeDrive(driver.getY()*((driver.getRawAxis(3)-1)/2), driver.getX()*-((driver.getRawAxis(3)-1)/2));
			if(driver.getRawButton(3)) {
				hwheel.set(0.5);
			}
			else if(driver.getRawButton(4)) {
				hwheel.set(-0.5);
			}
			else {
				hwheel.set(0);
			}
		}
		else{
			followTarget();
		}
		
		//Start Arm/Wrist
		if(armOperator.getBackButton()){
			arm.setStartPosition();
			wrist.setStartPosition();
			wristAngle=0;
		}
		// System.out.println("arm = " + arm.getAngle() + " wrist = " + wrist.getAngle());

		if(armOperator.getAButton()){
			arm.armSet(0);
			wrist.spinTo(0);
			wristAngle=0;
		}
		else if(armOperator.getBButton()){
			arm.armSet(25);
			wrist.spinTo(123);
			wristAngle=123;
		}
		else if(armOperator.getXButton()){
			arm.armSet(67);
			wrist.spinTo(160);
			wristAngle=160;
		}
		else if(armOperator.getYButton()){
			arm.armSet(118);
			wrist.spinTo(212);
			wristAngle=212;
		}
		else{
			arm.setSpeed(armOperator.getY(Hand.kRight));
			if(Math.abs(armOperator.getX(Hand.kLeft)) > 0.15 && !controller){
				wrist.setSpeed(armOperator.getX(Hand.kLeft)/5);
				wristAngle = wrist.getAngle();
			}
			else if(Math.abs(armOperator.getX(Hand.kRight)) > 0.15){
				wrist.setSpeed(armOperator.getX(Hand.kRight)/5);
				wristAngle = wrist.getAngle();
			}
			else {
				wrist.spinTo(wristAngle);
			}
		}
		//End Arm/Wrist

		//Start Vacuums
		if(armOperator.getBumper(Hand.kRight) && !controller){
			vacuumController1.set(-1);
		}
		else if(armOperator.getTriggerAxis(Hand.kRight) >0.5 && !controller){
			vacuumController1.set(1);
		}
		else{
			vacuumController1.set(0);
		}

		if(armOperator.getBumper(Hand.kLeft) && !controller){
			vacuumController2.set(ControlMode.PercentOutput, -1);
		}
		else{
			vacuumController2.set(ControlMode.PercentOutput, 0);
		}
		//End Vacuums

		if(driver.getTrigger()){
			armOperator.setRumble(RumbleType.kLeftRumble,1);
			armOperator.setRumble(RumbleType.kRightRumble,1);
		}		
		else{
			armOperator.setRumble(RumbleType.kLeftRumble,0);
			armOperator.setRumble(RumbleType.kRightRumble,0);
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	private void followTarget(){
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tv = table.getEntry("tv");
		ledMode.setDouble(0);

		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);
		double area = ta.getDouble(0.0);

		if(area < 1 && target == 1){
			myRobot.arcadeDrive(0.4, x/Math.abs(x)*Math.min(Math.abs(x/10), 0.5));
		}
		else if(area > 1.5 && target==1){
			myRobot.arcadeDrive(-0.4, x/Math.abs(x)*(Math.min(Math.abs(x/10), 0.5)));
		}
		else{
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
		pipeline.setDouble(1);
		double aL = ta.getDouble(0.0);
		pipeline.setDouble(2);
		double aR = ta.getDouble(0.0);
		pipeline.setDouble(0);

		System.out.println("Area = " + area);
		System.out.println("Skew = " + skew);
		System.out.println(aL + " Area Left");
		System.out.println(aR + " Area Right");

		// if(target == 1 && skew < -45 && skew >-87){
		// 	myRobot.arcadeDrive(0, x/Math.abs(x)*Math.min(Math.abs(x/5), 0.5));
		// 	hwheel.set(0.3);
		// }
		// else if(target==1 & skew < -3){
		// 	myRobot.arcadeDrive(0, x/Math.abs(x)*Math.min(Math.abs(x/5), 0.5));
		// 	hwheel.set(-0.3);
		// }
		// else{
		// 	myRobot.arcadeDrive(0, x/Math.abs(x)*Math.min(Math.abs(x/5), 0.5));
		// 	hwheel.set(0);
		// }
	}
}