package org.usfirst.frc.team5826.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Arm {

    private static final double topAngle = 10;
    private static final double midAngle = 150;
    private static final double bottomAngle = 2;
    private static final double stationAngle = 2;
    private static final double groundAngle = 2;
	private Actuator arm;
    private Actuator wrist;
    private XboxController controller;
    
    // jdgfde  
    public Arm (Actuator arm, Actuator wrist, XboxController controller){

        this.wrist = wrist;
        this.arm = arm;
        this.controller = controller;

        

    }
    public double moveZero(boolean vacuum1, boolean vacuum2){
        arm.armSet(0);
        wrist.spinTo(0);
        return 0;
    }
    public double getAngle() {
        return arm.getAngle();

    }
    public double moveTop(boolean vacuum1, boolean vacuum2) {
        double topAngle = wrist.getAngle();
        if(vacuum1 && !vacuum2 && controller.getYButton()){
            arm.armSet(120);
            wrist.spinTo(131);
            topAngle = 131;
        }
        else if(vacuum2 && !vacuum1 && controller.getYButton()){
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
    public double moveMid(boolean vacuum1, boolean vacuum2) {
        double midAngle = wrist.getAngle();
        if(vacuum1 && !vacuum2 && controller.getBButton()){
            arm.armSet(87);
            wrist.spinTo(160);
            midAngle = 160;
        }
        else if(vacuum2 && !vacuum1 && controller.getBButton()){
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
    public double moveBottom(boolean vacuum1, boolean vacuum2) {
        double bottomAngle = wrist.getAngle();
        if(vacuum1 && !vacuum2 && controller.getAButton()){
            arm.armSet(113);
            wrist.spinTo(43);
            bottomAngle = 43;
        }
        else if(vacuum2 && !vacuum1 && controller.getAButton()){
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
    public double moveStation(boolean vacuum1, boolean vacuum2) {
        double stationAngle = wrist.getAngle();
        if(vacuum1 && !vacuum2 && controller.getXButton()){
            arm.armSet(60);
            wrist.spinTo(104);
            stationAngle = 104;
        }
        else if(vacuum2 && !vacuum1 && controller.getXButton()){
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
    public double moveGround(boolean vacuum1, boolean vacuum2) {
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
