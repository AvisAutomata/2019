package org.usfirst.frc.team5826.robot;

public class Arm {

    private static final double TopAngle = 10;
    private static final double MidAngle = 5;
    private static final double BotAngle = 2;
	private Actuator arm;
    private Actuator wrist;
    
    // jdgfde  
    public Arm (Actuator arm, Actuator wrist){

        this.wrist = wrist;
        this.arm = arm;

    }
    public double getAngle() {
        
        return arm.getAngle();

    }
    public void moveTop() {

        arm.spinTo(TopAngle);
        wrist.spinTo(-TopAngle);

    }
    public void moveMid() {

        arm.spinTo(TopAngle);
        wrist.spinTo(-TopAngle);

    }
    public void moveBot() {

        arm.spinTo(TopAngle);
        wrist.spinTo(-TopAngle);

    }
}
