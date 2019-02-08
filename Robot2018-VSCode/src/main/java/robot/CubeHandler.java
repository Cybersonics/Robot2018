package org.usfirst.frc.team103.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team103.robot.RobotMap.*;

import org.usfirst.frc.team103.command.TeleopCubeHandler;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class CubeHandler extends Subsystem {
	public static final double CONVEYOR_SPEED = 0.5;
	
	public void intake(double direction, double speed) {
		move(direction, speed, true);
	}
	
	public void outtake(double direction, double speed) {
		move(direction, speed, false);
	}
	
	private void move(double angle, double speed, boolean isIntake) {
		SmartDashboard.putNumber("MoveAngle", angle);
		SmartDashboard.putNumber("MoveSpeed", speed);
		SmartDashboard.putBoolean("MoveIsIntake", isIntake);
		double angleDifference = angle - positioning.getHeading();
		if (Math.abs(angleDifference) > 180.0) {
			angleDifference -= 360.0 * Math.signum(angleDifference);
		}
		boolean useFront = Math.abs(angleDifference) < 90.0;
		SmartDashboard.putBoolean("MoveUseFront", useFront);
		conveyor.set(ControlMode.PercentOutput, (useFront ^ isIntake ? -CONVEYOR_SPEED : CONVEYOR_SPEED));
		(useFront ? armFront : armRear).set(ControlMode.PercentOutput, (isIntake ? speed : -speed));
		(useFront ? armRear : armFront).set(ControlMode.PercentOutput, 0.0);
	}
	
	public void hold() {
		conveyor.set(ControlMode.PercentOutput, 0.0);
		armFront.set(ControlMode.PercentOutput, 0.0);
		armRear.set(ControlMode.PercentOutput, 0.0);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new TeleopCubeHandler());
	}
}
