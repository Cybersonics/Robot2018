package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StopDrive extends InstantCommand {
	public StopDrive() {
		requires(RobotMap.drive);
	}
	
	@Override
	protected void initialize() {
		RobotMap.driveLeftFront.set(ControlMode.Disabled, 0.0);
		RobotMap.driveLeftRear.set(ControlMode.Disabled, 0.0);
		RobotMap.driveRightFront.set(ControlMode.Disabled, 0.0);
		RobotMap.driveRightRear.set(ControlMode.Disabled, 0.0);
	}
}
