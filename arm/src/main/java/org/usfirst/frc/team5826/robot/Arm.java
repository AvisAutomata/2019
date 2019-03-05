package org.usfirst.frc.team5826.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Arm {
	private Actuator arm;
    private Actuator wrist;
    private XboxController controller;
      
    public Arm (Actuator arm, Actuator wrist, XboxController controller){

        this.wrist = wrist;
        this.arm = arm;
        this.controller = controller;

        

    }
    public double moveZero(boolean discVac, boolean ballVac){
        arm.armSet(0);
        wrist.spinTo(0);
        return 0;
    }
    public double getAngle() {
        return arm.getAngle();

    }
    // Start Arm/Wrist Presets
    public double moveTop(boolean discVac, boolean ballVac) {
        double topAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getYButton()){
            arm.armSet(117);
            wrist.spinTo(192);
            topAngle = 192;
        }
        else if(ballVac && !discVac && controller.getYButton()){
            arm.armSet(117);
            wrist.spinTo(146);
            topAngle = 146;
        }
        // else{
        //     arm.armSet(120);
        //     wrist.spinTo(131);
        //     topAngle = 131;
        // }
        return topAngle;
    }
    public double moveMid(boolean discVac, boolean ballVac) {
        double midAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getBButton()){
            arm.armSet(72);
            wrist.spinTo(162);
            midAngle = 162;
        }
        else if(ballVac && !discVac && controller.getBButton()){
            arm.armSet(86);
            wrist.spinTo(172);
            midAngle = 172;
        }
        // else{
        //     arm.armSet(61);
        //     wrist.spinTo(20);
        //     midAngle = 20;
        // }
        return midAngle;
    }
    public double moveBottom(boolean discVac, boolean ballVac) {
        double bottomAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getAButton()){
            arm.armSet(30);
            wrist.spinTo(118);
            bottomAngle = 118;
        }
        else if(ballVac && !discVac && controller.getAButton()){
            arm.armSet(42);
            wrist.spinTo(127);
            bottomAngle = 127;
        }
        // else{
        //     arm.armSet(32);
        //     wrist.spinTo(122);
        //     bottomAngle = 122;
        // }
        return bottomAngle;
    }
    public double moveStation(boolean discVac, boolean ballVac) {
        double stationAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getXButton()){
            arm.armSet(30);
            wrist.spinTo(118);
            stationAngle = 118;
        }
        else if(ballVac && !discVac && controller.getXButton()){
            arm.armSet(52);
            wrist.spinTo(101);
            stationAngle = 101;
        }
        // else{
        //     arm.armSet(32);
        //     wrist.spinTo(122);
        //     stationAngle = 122;
        // }
        return stationAngle;
    }
    public double moveGround(boolean discVac, boolean ballVac) {
        double groundAngle = wrist.getAngle();
        if(controller.getBumper(Hand.kRight)){
            arm.armSet(19);
            wrist.spinTo(136);
            groundAngle = 136;
        }
        else if(controller.getBumper(Hand.kLeft)){
            arm.armSet(19);
            wrist.spinTo(136);
            groundAngle = 136;
        }
        // else{
        //     arm.armSet(32);
        //     wrist.spinTo(122);
        //     groundAngle = 122;
        // }
        return groundAngle;
    }
    // End Arm/Wrist Presets
}
