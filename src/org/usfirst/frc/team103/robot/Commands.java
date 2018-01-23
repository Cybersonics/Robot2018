package org.usfirst.frc.team103.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class Commands {
	public static Command instantCommand(Runnable initialize, boolean runWhenDisabled) {
		return new InstantCommand() {
			{
				setRunWhenDisabled(runWhenDisabled);
			}
			@Override
			protected void initialize() {
				initialize.run();
			}
		};
	}
}
