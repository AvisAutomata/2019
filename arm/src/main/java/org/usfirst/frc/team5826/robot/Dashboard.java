/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5826.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Dashboard {
    private Actuator arm;
    private Actuator wrist;
    private LidarLitePWM lidarRight;
    private LidarLitePWM lidarLeft;

    public Dashboard(Actuator arm, Actuator wrist, LidarLitePWM lidarRight,
    LidarLitePWM lidarLeft) {
        super();
        this.arm = arm;
        this.wrist = wrist;
        this.lidarLeft = lidarLeft;
        this.lidarRight = lidarRight;
    }

    public void init() {
        SmartDashboard.putNumber("ApproachDistance", 30.0);
        SmartDashboard.putNumber("Gyro", 0.0);
        SmartDashboard.putNumber("WristAngle", 0.0);
        SmartDashboard.putNumber("ArmAngle", 0.0);
        SmartDashboard.putNumber("DistanceRight",0.0);
        SmartDashboard.putNumber("DistanceLeft",0.0);
        SmartDashboard.putNumber("ApproachAngle", 0.0);
    }

    public void output(double gyroValue, double debug) {
        SmartDashboard.putNumber("Gyro", (double) Math.round(gyroValue*1000.0) / 1000.0);
        SmartDashboard.putNumber("WristAngle", wrist.getAngle());
        SmartDashboard.putNumber("ArmAngle", arm.getAngle());
        SmartDashboard.putNumber("DistanceRight", (double) Math.round(lidarRight.getDistance()*100.0) / 100.0);
        SmartDashboard.putNumber("DistanceLeft", (double) Math.round(lidarLeft.getDistance()*100.0) / 100.0);
        SmartDashboard.putNumber("debug", debug);
    }

    public double getDistance(){
        return SmartDashboard.getNumber("ApproachDistance", 30.0);
    }
    public double getAngle(){
        return SmartDashboard.getNumber("ApproachAngle", 0.0);
    }
}
