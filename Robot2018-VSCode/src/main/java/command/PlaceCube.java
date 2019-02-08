package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class PlaceCube extends Command {
	public static final int ALLOWABLE_HEIGHT_ERROR = 500;
	public static final double PLACEMENT_TIMEOUT = 3.0;
	
	private final int height;
	private final double direction, speed;
	
	private boolean isPlacing;
	private double placementEndTime;
	
	public PlaceCube(int height, double direction, double speed) {
		this.height = height;
		this.direction = direction;
		this.speed = speed;

		requires(RobotMap.elevator);
		requires(RobotMap.cubeHandler);
	}
	
	@Override
	protected void initialize() {
		isPlacing = false;
		RobotMap.elevator.setHeight(height);
	}
	
	@Override
	protected void execute() {
		if (!isPlacing && Math.abs(RobotMap.elevator.getHeight() - height) < ALLOWABLE_HEIGHT_ERROR) {
			isPlacing = true;
			placementEndTime = Timer.getFPGATimestamp() + PLACEMENT_TIMEOUT;
			RobotMap.cubeHandler.outtake(direction, speed);
		}
	}

	@Override
	protected boolean isFinished() {
		return isPlacing && Timer.getFPGATimestamp() > placementEndTime;
	}
	
	@Override
	protected void end() {
		RobotMap.cubeHandler.hold();
	}
}
