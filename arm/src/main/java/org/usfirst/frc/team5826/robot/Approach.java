/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5826.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class Approach {
    private NetworkTable table;
    private Actuator arm;
    private WPI_TalonSRX hwheel;
    private Joystick driver;
    private DifferentialDrive myRobot;

    public Approach(NetworkTable table, Actuator arm, WPI_TalonSRX hwheel, Joystick driver, DifferentialDrive myRobot){
        super();
        this.arm = arm;
        this.table = table;
        this.hwheel = hwheel;
        this.driver = driver;
        this.myRobot = myRobot;
    }

    public void lineUp(double gyroValue, double angle) {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry pipeline = table.getEntry("pipeline");
		if (arm.getAngle() > 90) {
			pipeline.setDouble(1);
		} else {
			pipeline.setDouble(0);
		}
		double target = tv.getDouble(0.0);
		double x = tx.getDouble(0.0);

		if (target == 1) {
			hwheel.set(x / Math.abs((x != 0) ? x : 1) * Math.min(Math.abs(x / 14), 0.33));
			if (Math.abs(gyroValue - angle) > 2) {
				myRobot.arcadeDrive(-driver.getY(),
						-(gyroValue - angle) / Math.abs(((gyroValue - angle) != 0) ? (gyroValue - angle) : 1)
								* Math.min(Math.abs(gyroValue - angle) / 8, 0.5));
			} else {
				myRobot.arcadeDrive(-driver.getY(), 0);
			}
		} else {
			hwheel.set(0);
			myRobot.arcadeDrive(-driver.getY(), 0);
		}
    }
    
    public void rocket(double gyroValue, boolean ballVac, boolean discVac){
        if (Math.abs(gyroValue - 330) < 30 && discVac) {
            lineUp(gyroValue, 330);
        } else if (Math.abs(gyroValue - 270) < 30 && ballVac) {
            lineUp(gyroValue, 270);
        } else if (Math.abs(gyroValue - 210) < 30 && discVac) {
            lineUp(gyroValue, 210);
        } else if (Math.abs(gyroValue - 30) < 30 && discVac) {
            lineUp(gyroValue, 30);
        } else if (Math.abs(gyroValue - 90) < 30 && ballVac) {
            lineUp(gyroValue, 90);
        } else if (Math.abs(gyroValue - 150) < 30 && discVac) {
            lineUp(gyroValue, 150);
        }
    }

    public void cargoStation(double gyroValue){
        if (Math.abs(gyroValue - 270) < 45) {
            lineUp(gyroValue, 270);
        } else if (Math.abs(gyroValue - 90) < 45) {
            lineUp(gyroValue, 90);
        } else if (gyroValue > 315 || gyroValue < 45){
            lineUp(gyroValue,(gyroValue>315)?360:0);
        } else if(Math.abs(gyroValue - 180) < 45){
            lineUp(gyroValue, 180);
        }
    }
}
