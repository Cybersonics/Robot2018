package org.usfirst.frc.team103.robot;

import java.util.LinkedList;
import java.util.List;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.command.PathTo;
import org.usfirst.frc.team103.command.PlaceCube;
import org.usfirst.frc.team103.command.StopDrive;
import org.usfirst.frc.team103.util.Commands;
import org.usfirst.frc.team103.util.DynamicCommandGroup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
	public static final String START_KEY = "Starting Position",
			DEPLOY_KEY = "Deploy Action",
			DELAY_1_KEY = "Delay 1", ACTION_1_KEY = "Action 1",
			DELAY_2_KEY = "Delay 2", ACTION_2_KEY = "Action 2",
			END_KEY = "Ending Position";
	
	public static final double DRIVE_SPEED = 0.75;
	public static final double START_X = 103.0;
	
	private SendableChooser<StartingPosition> startChooser;
	private SendableChooser<DeployAction> deployChooser;
	private SendableChooser<Action> action1Chooser, action2Chooser;
	private SendableChooser<EndingPosition> endChooser;
	
	private StartingPosition startPosition;
	private FieldSide switchSide, scaleSide;
	
	public void initializeOptions() {
		startChooser = new SendableChooser<>();
		fillSendableChooser(startChooser, StartingPosition.LEFT, StartingPosition.RIGHT);
		SmartDashboard.putData(START_KEY, startChooser);
		
		deployChooser = new SendableChooser<>();
		fillSendableChooser(deployChooser, DeployAction.SPIN, DeployAction.NOTHING);
		SmartDashboard.putData(DEPLOY_KEY, deployChooser);
		
		//SmartDashboard.putNumber(DELAY_1_KEY, 0.0);
		
		action1Chooser = new SendableChooser<>();
		fillSendableChooser(
			action1Chooser,
			Action.NOTHING,
			Action.CROSS, Action.SWITCH_SIDE, Action.SWITCH_MIDDLE, Action.SCALE_MIDDLE, Action.SCALE_SIDE
		);
		SmartDashboard.putData(ACTION_1_KEY, action1Chooser);
		
		//SmartDashboard.putNumber(DELAY_2_KEY, 0.0);
		
		action2Chooser = new SendableChooser<>();
		fillSendableChooser(
			action2Chooser,
			Action.NOTHING,
			Action.SWITCH_SIDE, Action.SWITCH_MIDDLE, Action.SCALE_MIDDLE, Action.SCALE_SIDE
		);
		SmartDashboard.putData(ACTION_2_KEY, action2Chooser);
		
		endChooser = new SendableChooser<>();
		fillSendableChooser(endChooser, EndingPosition.STAY, EndingPosition.SWITCH_FENCE, EndingPosition.NULL);
		SmartDashboard.putData(END_KEY, endChooser);
	}
	
	public Command generateAutonomous() {
		String message;
		while ((message = DriverStation.getInstance().getGameSpecificMessage()).isEmpty()) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		startPosition = startChooser.getSelected();
		switchSide = FieldSide.sideFromCharacter(message.charAt(0));
		scaleSide = FieldSide.sideFromCharacter(message.charAt(1));
		System.out.println("Starting position: " + startPosition);
		System.out.println("Switch side: " + switchSide);
		System.out.println("Scale side: " + scaleSide);

		RobotMap.positioning.setFieldZeroHeading((RobotMap.navX.getYaw() + 360.0) % 360.0);
		RobotMap.positioning.setPosition(startPosition.multiplier * START_X, 15.0);
		
		List<Command> commands = new LinkedList<>();
		commands.add(new WaitCommand(SmartDashboard.getNumber(DELAY_1_KEY, 0.0)));
		commands.add(new PrintCommand("Delay 1 done"));
		commands.add(generateDeploy(deployChooser.getSelected()));
		commands.add(new PrintCommand("Deploy done"));
		commands.add(generateAction(action1Chooser.getSelected()));
		commands.add(new PrintCommand("Action 1 done"));
		commands.add(new WaitCommand(SmartDashboard.getNumber(DELAY_2_KEY, 0.0)));
		commands.add(new PrintCommand("Delay 2 done"));
		commands.add(generateAction(action2Chooser.getSelected()));
		commands.add(new PrintCommand("Action 2 done"));
		commands.add(generateEnd(endChooser.getSelected()));
		commands.add(new PrintCommand("End done"));
		commands.add(new StopDrive());
		
		return new DynamicCommandGroup() {
			{
				requires(RobotMap.drive);
				requires(RobotMap.elevator);
				requires(RobotMap.cubeHandler);
			}
			@Override
			protected void dynamicInitialize() {
				for (Command command : commands) {
					addSequential(command);
				}
			}
		};
	}
	
	@SafeVarargs
	private static <T> void fillSendableChooser(SendableChooser<T> chooser, T defaultOption, T... options) {
		chooser.addDefault(defaultOption.toString(), defaultOption);
		for (T option : options) {
			chooser.addObject(option.toString(), option);
		}
	}
	
	private Command generateDeploy(DeployAction deploy) {
		switch (deploy) {
		case NOTHING:
			return new InstantCommand();
		case SPIN:
			return new DriveTo(startPosition.multiplier * START_X, 80.0, 180.0, 1.0, 0.0, 30.0, 20.0);
		default:
			return null;
		}
	}
	
	private Command generateAction(Action action) {
		System.out.println("Generating " + action);
		switch (action) {
		case NOTHING:
			return new InstantCommand();
		case CROSS:
			return new PathTo(switchSide.multiplier * 100.0, 110.0, 0.0, DRIVE_SPEED, 0.0);
		case SWITCH_SIDE:
			return Commands.sequentialCommand(
				new PathTo(switchSide.multiplier * 102.0, 170.0, 90.0, DRIVE_SPEED, 0.0),
				new PlaceCube(9000, 180.0 + switchSide.multiplier * 90.0, 1.0)
			);
		case SWITCH_MIDDLE:
			return Commands.sequentialCommand(
				new PathTo(switchSide.multiplier * 70.0, 225.0, 0.0, DRIVE_SPEED, 0.0),
				new PlaceCube(9000, 180.0, 1.0)
			);
		case SCALE_MIDDLE:
			return Commands.sequentialCommand(
				new PathTo(scaleSide.multiplier * 85.0, 265.0, 0.0, DRIVE_SPEED, 0.0),
				new PlaceCube(28000, 0.0, 1.0)
			);
		case SCALE_SIDE:
			return Commands.sequentialCommand(
				new PathTo(scaleSide.multiplier * 121.0, 320.0, 90.0, DRIVE_SPEED, 0.0),
				new PlaceCube(28000, 108.0 + scaleSide.multiplier * 90.0, 1.0)
			);
		default:
			return null;
		}
	}
	
	private Command generateEnd(EndingPosition end) {
		switch (end) {
		case STAY:
			return new InstantCommand();
		case SWITCH_FENCE:
			return new PathTo(switchSide.multiplier * 70.0, 225.0, 0.0, DRIVE_SPEED, 0.0);
		case NULL:
			return new PathTo(scaleSide.multiplier * 121.0, 320.0, 0.0, DRIVE_SPEED, 0.0);
		default:
			return null;
		}
	}
	
	private static enum StartingPosition {
		LEFT("Left", -1.0), RIGHT("Right", 1.0), MIDDLE("Middle", 0.0);
		
		public final String label;
		public final double multiplier;
		private StartingPosition(String label, double multiplier) {
			this.label = label;
			this.multiplier = multiplier;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private static enum DeployAction {
		NOTHING("Nothing"), SPIN("Spin");
		
		public final String label;
		private DeployAction(String label) {
			this.label = label;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private static enum EndingPosition {
		STAY("Stay in Place"), SWITCH_FENCE("Switch Fence"), NULL("Null Territory");
		
		public final String label;
		private EndingPosition(String label) {
			this.label = label;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private static enum Action {
		NOTHING("Nothing"), CROSS("Cross Line"), SWITCH_SIDE("Switch from Side"), SWITCH_MIDDLE("Switch from Middle"), SCALE_MIDDLE("Scale from Middle"), SCALE_SIDE("Scale from Side");

		public final String label;
		private Action(String label) {
			this.label = label;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private static enum FieldSide {
		LEFT(-1.0), RIGHT(1.0);
		
		public final double multiplier;
		private FieldSide(double multiplier) {
			this.multiplier = multiplier;
		}
		
		public static FieldSide sideFromCharacter(char c) {
			switch (c) {
			case 'L':
				return LEFT;
			case 'R':
				return RIGHT;
			default:
				return null;
			}
		}
	}
}
