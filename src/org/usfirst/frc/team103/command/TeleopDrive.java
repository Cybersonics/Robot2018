package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class TeleopDrive extends Command {
	public static boolean usePID = false;
	
	private double zeroHeading = 0.0;

	public TeleopDrive() {
		requires(RobotMap.drive);
	}
	
	@Override
	protected void execute() {
		double strafe = RobotMap.leftJoy.getX();
		double forward = -RobotMap.leftJoy.getY();
		double omega = RobotMap.rightJoy.getX();
		
		if (RobotMap.leftJoy.getRawButton(7)) {
			zeroHeading = RobotMap.positioning.getHeading();
		}
		
		if (RobotMap.leftJoy.getTrigger()) {
			double originCorrection = Math.toRadians(zeroHeading - RobotMap.positioning.getHeading());
    		double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
    		strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
    		forward = temp;
		}
		
		RobotMap.drive.swerveDrive(strafe, forward, omega, true, usePID);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
