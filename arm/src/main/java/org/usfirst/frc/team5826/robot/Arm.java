package org.usfirst.frc.team5826.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Arm {
	private Actuator arm;
    private Actuator wrist;
    private XboxController controller;
    
    // jdgfde  
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
    public double moveTop(boolean discVac, boolean ballVac) {
        double topAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getYButton()){
            arm.armSet(120);
            wrist.spinTo(131);
            topAngle = 131;
        }
        else if(ballVac && !discVac && controller.getYButton()){
            arm.armSet(120);
            wrist.spinTo(131);
            topAngle = 131;
        }
        else{
            arm.armSet(120);
            wrist.spinTo(131);
            topAngle = 131;
        }
        return topAngle;
    }
    public double moveMid(boolean discVac, boolean ballVac) {
        double midAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getBButton()){
            arm.armSet(87);
            wrist.spinTo(160);
            midAngle = 160;
        }
        else if(ballVac && !discVac && controller.getBButton()){
            arm.armSet(85);
            wrist.spinTo(160);
            midAngle = 160;
        }
        else{
            arm.armSet(61);
            wrist.spinTo(20);
            midAngle = 20;
        }
        return midAngle;
    }
    public double moveBottom(boolean discVac, boolean ballVac) {
        double bottomAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getAButton()){
            arm.armSet(113);
            wrist.spinTo(43);
            bottomAngle = 43;
        }
        else if(ballVac && !discVac && controller.getAButton()){
            arm.armSet(117);
            wrist.spinTo(43);
            bottomAngle = 43;
        }
        else{
            arm.armSet(32);
            wrist.spinTo(122);
            bottomAngle = 122;
        }
        return bottomAngle;
    }
    public double moveStation(boolean discVac, boolean ballVac) {
        double stationAngle = wrist.getAngle();
        if(discVac && !ballVac && controller.getXButton()){
            arm.armSet(60);
            wrist.spinTo(104);
            stationAngle = 104;
        }
        else if(ballVac && !discVac && controller.getXButton()){
            arm.armSet(60);
            wrist.spinTo(108);
            stationAngle = 108;
        }
        else{
            arm.armSet(32);
            wrist.spinTo(122);
            stationAngle = 122;
        }
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
        else{
            arm.armSet(32);
            wrist.spinTo(122);
            groundAngle = 122;
        }
        return groundAngle;
    }
}
