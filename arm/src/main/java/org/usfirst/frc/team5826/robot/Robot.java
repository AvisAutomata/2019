
package org.usfirst.frc.team5826.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
// TODO Change Iterative to Timed
public class Robot extends IterativeRobot {
	Encoder wristEncoder = new Encoder(0, 1);
	Encoder armEncoder = new Encoder(2, 3);

	Compressor compressor = new Compressor(11);
	PowerDistributionPanel PDP = new PowerDistributionPanel(10);
	WPI_TalonSRX vacuumController1 = new WPI_TalonSRX(9);
	WPI_TalonSRX vacuumController2 = new WPI_TalonSRX(8);
	WPI_TalonSRX wristController = new WPI_TalonSRX(7);
	WPI_TalonSRX armController = new WPI_TalonSRX(6);
	WPI_TalonSRX hwheel = new WPI_TalonSRX(5);
	WPI_TalonSRX leftfront = new WPI_TalonSRX(4);
	WPI_TalonSRX leftback = new WPI_TalonSRX(3);
	WPI_TalonSRX rightfront = new WPI_TalonSRX(2);
	WPI_TalonSRX rightback = new WPI_TalonSRX(1);
	SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftfront, leftback);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightfront, rightback);
	DifferentialDrive myRobot = new DifferentialDrive(leftDrive, rightDrive);
	XboxController armOperator = new XboxController(1);
	Joystick driver = new Joystick(0);
	private Actuator wrist;
	private Actuator arm;
	private Arm armSS;
	private Dashboard dash;
	NetworkTable table;
	private Gyro gyro = new ADXRS450_Gyro();
	LidarLitePWM lidarLeft = new LidarLitePWM(new DigitalInput(8));
	LidarLitePWM lidarRight = new LidarLitePWM(new DigitalInput(9));
	DoubleSolenoid backSolenoid = new DoubleSolenoid(10, 6, 7);
	DoubleSolenoid fowardSolenoid = new DoubleSolenoid(10, 4, 5);

	double wristAngle = 0;

	boolean drive = true;
	boolean discVac = false;
	boolean ballVac = false;

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
		dash = new Dashboard(arm, wrist, gyro, lidarRight, lidarLeft);
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		ledMode.setDouble(1);
		dash.init();
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
		double gyroValue = (gyro.getAngle() >= 0) ? gyro.getAngle() : gyro.getAngle() + 360;
		if (driver.getRawButtonPressed(5)) {
			drive = !drive;
		}

		if (armOperator.getStartButton() || Math.abs(gyro.getAngle()) > 360) {
			gyro.reset();
		}

		if (driver.getRawButtonReleased(8) || driver.getRawButtonReleased(5)) {
			table = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry ledMode = table.getEntry("ledMode");
			ledMode.setDouble(1);
		}

		if (driver.getRawButton(8)) {
			approach();
		} else if (driver.getRawButton(11)) {
			// TODO Test this.
			if (Math.abs(gyroValue - 180) > 2) {
				myRobot.arcadeDrive(-driver.getY(),
						-gyroValue / Math.abs(((gyroValue - 180) != 0) ? (gyroValue - 180) : 1)
								* Math.min(Math.abs(gyroValue - 180) / 6, 0.4));
			} else {
				myRobot.arcadeDrive(-driver.getY(), 0);
			}

		} else if (driver.getRawButton(10)) {
			approachGyroBased(gyroValue);
		} else if (drive) {
			myRobot.arcadeDrive(driver.getY() * ((driver.getRawAxis(3) - 1) / 2),
					driver.getZ() * -((driver.getRawAxis(3) - 1) / 2));
			// if (driver.getRawButton(3)) {
			// hwheel.set(0.5);
			// } else if (driver.getRawButton(4)) {
			// hwheel.set(-0.5);
			// } else {
			// hwheel.set(0);
			// }
			if (Math.abs(driver.getX()) > 0.2) {
				hwheel.set(driver.getX() / Math.abs((driver.getX() != 0) ? driver.getX() : 1)
						* Math.min(driver.getX(), 0.5));
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
		System.out.println("arm = " + arm.getAngle() + " wrist = " + wrist.getAngle() + " Gyro = " + gyroValue);
		if (armOperator.getAButton()) {
			wristAngle = armSS.moveStation(discVac, ballVac);
		} else if (armOperator.getBButton()) {
			wristAngle = armSS.moveBottom(discVac, ballVac);
		} else if (armOperator.getYButton()) {
			wristAngle = armSS.moveMid(discVac, ballVac);
		} else if (armOperator.getYButton()) {
			wristAngle = armSS.moveTop(discVac, ballVac);
		} else {
			if (arm.getAngle() < 120 - 4/* TODO Change for Competition robot */) {
				arm.setSpeed((armOperator.getY(Hand.kRight) < 0) ? armOperator.getY(Hand.kRight)
						: armOperator.getY(Hand.kRight) / 2);
			} else {
				arm.setSpeed(Math.max(armOperator.getY(Hand.kRight), 0));
			}
			if (Math.abs(armOperator.getY(Hand.kLeft)) > 0.15) {
				wrist.setSpeed(armOperator.getY(Hand.kLeft) / 3);
				wristAngle = wrist.getAngle();
			} else {

				wrist.spinTo(wristAngle);
			}
		}
		// End Arm/Wrist

		// Start Disk Vacuum
		if (armOperator.getBumper(Hand.kRight)) {
			discVac = true;
		} else if (armOperator.getTriggerAxis(Hand.kLeft) > 0.5 || armOperator.getTriggerAxis(Hand.kRight) > 0.5) {
			discVac = false;
		}
		if (discVac == true) {
			// TODO lower power for competition robot
			vacuumController1.set(ControlMode.PercentOutput, -1);
		} else {
			vacuumController1.set(ControlMode.PercentOutput, 0);
		}
		// End Disk Vacuum

		// Start Ball Vacuum
		if (armOperator.getBumper(Hand.kLeft)) {
			ballVac = true;
		} else if (armOperator.getTriggerAxis(Hand.kLeft) > 0.5 || armOperator.getTriggerAxis(Hand.kRight) > 0.5) {
			ballVac = false;
		}

		if (ballVac) {
			vacuumController2.set(ControlMode.PercentOutput, -1);
		} else {
			vacuumController2.set(ControlMode.PercentOutput, 0);
		}
		// End Ball Vacuum
		// PDP CODE
		// if (pdp.getCurrent(4) > 8 || (pdp.getCurrent(12) < 10 && pdp.getCurrent(12) >
		// 4)) {
		// armOperator.setRumble(RumbleType.kRightRumble, 0.2);
		// } else {
		// armOperator.setRumble(RumbleType.kRightRumble, 0);
		// }
		// End Vacuums
		// Start Solenoids
		int pov = driver.getPOV();
		if (pov >= 270 || (pov <= 90 && pov != -1)) {
			fowardSolenoid.set(Value.kForward);// DOWN
		} else {
			fowardSolenoid.set(Value.kReverse);// UP
		}
		if (pov < 270 && pov > 90) {
			backSolenoid.set(Value.kForward);// DOWN
		} else {
			backSolenoid.set(Value.kReverse);// UP
		}
		dash.output();
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

	public void approach() {
		// System.out.println(lidarRight.getDistance());
		// space = dist between robot and board
		double space = 80;
		// Limelight mount offset
		double offset = 5;

		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		ledMode.setDouble(0);
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

		if (target == 1) {
			hwheel.set(-x / Math.abs((x != 0) ? x : 1) * Math.min(Math.abs(x / 2), 0.33));
			if (mindist > 2 * space) {
				myRobot.arcadeDrive(.7, curve);
			} else if (mindist >= space) {
				myRobot.arcadeDrive(.25, curve);
			} else if (mindist < 2 + space && mindist > space - 2) {
				myRobot.arcadeDrive(0, curve);
			} else if (mindist < space - 2 && mindist > 0) {
				myRobot.arcadeDrive(-.25, curve);
			}

		}
	}

	// NEW APPROACH CODE
	public void approachGyroBased(double gyroValue) {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry ledMode = table.getEntry("ledMode");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
		ledMode.setDouble(0);

		pipeline.setDouble(0);
		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);

		// TODO Find Angles, Base angle on where we are.
		if (target == 1) {
			hwheel.set(-x / Math.abs((x != 0) ? x : 1) * Math.min(Math.abs(x / 2), 0.33));
			if (Math.abs(gyroValue - 180) > 2) {
				myRobot.arcadeDrive(-driver.getY(),
						gyroValue / Math.abs((gyroValue != 0) ? gyroValue : 1) * Math.min((gyroValue - 180) / 4, 0.5));
			} else {
				myRobot.arcadeDrive(-driver.getY(), 0);
			}
		} else {
			hwheel.set(0);
			myRobot.arcadeDrive(-driver.getY(), 0);
		}
	}

	public void cardBoxApproach() {
		if (lidarLeft.getDistance() > dash.getDistance()) {
			myRobot.arcadeDrive(0.4, 0);
		} else {
			myRobot.arcadeDrive(0, 0);
		}

	}
}