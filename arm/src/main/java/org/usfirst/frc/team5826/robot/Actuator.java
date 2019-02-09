package org.usfirst.frc.team5826.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Encoder;

public class Actuator {
	private BaseMotorController controller;
	private Encoder encoder;
	private int gearRatio;
	private int revolutionsPerPulse;
	private double startingPoint;

	public Actuator(BaseMotorController controller, Encoder encoder, int gearRatio, int revolutionsPerPulse,
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
			controller.set(ControlMode.PercentOutput, Math.min((angle - 0.5 - getAngle()) / 10, 0.5));
			return false;
		} else if (getAngle() > angle + 0.5) {
			controller.set(ControlMode.PercentOutput, Math.max((angle + 0.5 - getAngle()) / 10, -0.5));
			return false;
		} else {
			controller.set(ControlMode.PercentOutput, 0);
			return true;
		}
	}

	public void hold() {
	}

	public void armSet(double angle) {
		if (getAngle() < angle - 0.5) {
			controller.set(ControlMode.PercentOutput, Math.min((getAngle() - angle - 0.5), -0.5));
		} else if (getAngle() > angle + 0.5) {
			controller.set(ControlMode.PercentOutput, Math.min((getAngle() - angle + .5), 0.5));
		} else {
			controller.set(ControlMode.PercentOutput, 0);
		}
	}

	public double getAngle() {
		double angle = startingPoint + encoder.get() * 360 / (revolutionsPerPulse * gearRatio);
		return angle;
	}

	public void setSpeed(double speed) {
		controller.set(ControlMode.PercentOutput, speed);
	}

	public void setStartPosition() {
		encoder.reset();
	}

}
