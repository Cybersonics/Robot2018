package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TrevorCommand extends CommandGroup {
	public TrevorCommand() {
		requires(RobotMap.drive);
		addSequential(new DriveTo(0.0, 50.0, 0.0, 0.8, 0.8));
	}
}
