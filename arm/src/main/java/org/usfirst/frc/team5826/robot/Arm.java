package org.usfirst.frc.team5826.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Arm {

    private static final double TopAngle = 10;
    private static final double midAngle = 150;
    private static final double BotAngle = 2;
	private Actuator arm;
    private Actuator wrist;
    private XboxController controller;
    
    // jdgfde  
    public Arm (Actuator arm, Actuator wrist, XboxController controller){

        this.wrist = wrist;
        this.arm = arm;
        this.controller = controller;

    }
    public double moveZero(){
        arm.armSet(0);
        wrist.spinTo(0);
        return 0;
    }
    public double getAngle() {
        return arm.getAngle();

    }
    public double moveTop() {
        double topAngle = wrist.getAngle();
        if(controller.getBumper(Hand.kRight)){
            // arm.armSet();
            // wrist.spinTo();
            // topAngle = ;
        }
        else if(controller.getBumper(Hand.kLeft)){
            // arm.armSet();
            // wrist.spinTo();
            // topAngle = ;
        }
        else{
            // arm.armSet();
            // wrist.spinTo();
            // topAngle = ;
        }
        return topAngle;
    }
    public double moveMid() {
        double midAngle = wrist.getAngle();
        if(controller.getBumper(Hand.kRight)){
            arm.armSet(75);
            wrist.spinTo(150);
            midAngle = 150;
        }
        else if(controller.getBumper(Hand.kLeft)){
            // arm.armSet();
            // wrist.spinTo();
            // midAngle = ;
        }
        else{
            arm.armSet(61);
            wrist.spinTo(122);
            midAngle = 122;
        }
        return midAngle;
    }
    public double moveBot() {
        double botAngle = wrist.getAngle();
        if(controller.getBumper(Hand.kRight)){
            arm.armSet(75);
            wrist.spinTo(150);
            botAngle = 150;
        }
        else if(controller.getBumper(Hand.kLeft)){
            // arm.armSet();
            // wrist.spinTo();
            // midAngle = ;
        }
        else{
            arm.armSet(32);
            wrist.spinTo(122);
            botAngle = 122;
        }
        return botAngle;
    }
}
