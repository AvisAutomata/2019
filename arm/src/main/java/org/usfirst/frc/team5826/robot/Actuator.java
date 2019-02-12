package org.usfirst.frc.team5826.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;

public class Actuator {
	private Talon controller;
	private Encoder encoder;
	private int gearRatio;
	private int revolutionsPerPulse;
	private double startingPoint;

	public Actuator(Talon controller, Encoder encoder, int gearRatio, int revolutionsPerPulse,
			double startingPoint) {
		super();
		this.controller = controller;
		this.encoder = encoder;
		this.gearRatio = gearRatio;
		this.revolutionsPerPulse = revolutionsPerPulse;
		this.startingPoint = startingPoint;
	}

	public boolean spinTo(double angle) {
		if (getAngle() < angle - 0.5) {
			controller.set(Math.min((angle - 0.5 - getAngle()) / 6, 0.5));
			return false;
		} else if (getAngle() > angle + 0.5) {
			controller.set(Math.max((angle + 0.5 - getAngle()) / 6, -0.5));
			return false;
		} else {
			controller.set(0);
			return true;
		}
	}

	public void hold() {
	}

	public void armSet(double angle) {
		if (getAngle() < angle - 0.5) {
			controller.set(Math.min((getAngle() - angle - 0.5), -0.5));
		} else if (getAngle() > angle + 0.5) {
			controller.set(Math.min((getAngle() - angle + .5), 0.5));
		} else {
			controller.set(0);
		}
	}

	public double getAngle() {
		double angle = startingPoint + encoder.get() * 360 / (revolutionsPerPulse * gearRatio);
		return angle;
	}

	public void setSpeed(double speed) {
		controller.set(speed);
	}

	public void setStartPosition() {
		encoder.reset();
	}

}
