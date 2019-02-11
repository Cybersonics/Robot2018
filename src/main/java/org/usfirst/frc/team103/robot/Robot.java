package org.usfirst.frc.team103.robot;

import static org.usfirst.frc.team103.robot.RobotMap.driveLeftFront;
import static org.usfirst.frc.team103.robot.RobotMap.driveLeftRear;
import static org.usfirst.frc.team103.robot.RobotMap.driveRightFront;
import static org.usfirst.frc.team103.robot.RobotMap.driveRightRear;
import static org.usfirst.frc.team103.robot.RobotMap.leftJoy;
import static org.usfirst.frc.team103.robot.RobotMap.navX;
import static org.usfirst.frc.team103.robot.RobotMap.positioning;
import static org.usfirst.frc.team103.util.Commands.instantCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	@Override
	public void robotInit() {
		//Jacob was here
		//Trevor wasn't here
		
		RobotMap.init();

        new JoystickButton(leftJoy, 6).whenPressed(instantCommand(() -> {
        	positioning.setFieldZeroHeading((navX.getYaw() + 360.0) % 360.0);
        	positioning.setPosition(-103.0, 15.0);
        }, true));
        
        /*new JoystickButton(rightJoy, 8).whenPressed(instantCommand(() -> {
        	driveLeftFront.setNeutralMode(NeutralMode.Brake);
        	driveLeftRear.setNeutralMode(NeutralMode.Brake);
        	driveRightFront.setNeutralMode(NeutralMode.Brake);
        	driveRightRear.setNeutralMode(NeutralMode.Brake);
        }, true));
        new JoystickButton(rightJoy, 9).whenPressed(instantCommand(() -> {
        	driveLeftFront.setNeutralMode(NeutralMode.Coast);
        	driveLeftRear.setNeutralMode(NeutralMode.Coast);
        	driveRightFront.setNeutralMode(NeutralMode.Coast);
        	driveRightRear.setNeutralMode(NeutralMode.Coast);
        }, true));
        new JoystickButton(rightJoy, 10).whenPressed(instantCommand(() -> TeleopDrive.usePID = false, true));
        new JoystickButton(rightJoy, 11).whenPressed(instantCommand(() -> TeleopDrive.usePID = true, true));*/
	}
	
	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run();
		
		SmartDashboard.putNumber("SteerLeftFront", RobotMap.steerLeftFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerLeftRear", RobotMap.steerLeftRear.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerRightFront", RobotMap.steerRightFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerRightRear", RobotMap.steerRightRear.getSelectedSensorPosition(0));

		SmartDashboard.putNumber("DriveLeftFront", RobotMap.driveLeftFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveLeftRear", RobotMap.driveLeftRear.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveRightFront", RobotMap.driveRightFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveRightRear", RobotMap.driveRightRear.getSelectedSensorPosition(0));

		SmartDashboard.putNumber("SpeedLeftFront", RobotMap.driveLeftFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("SpeedLeftRear", RobotMap.driveLeftRear.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("SpeedRightFront", RobotMap.driveRightFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("SpeedRightRear", RobotMap.driveRightRear.getSelectedSensorVelocity(0));
		
		SmartDashboard.putNumber("FusedHeading", RobotMap.navX.getFusedHeading());
		SmartDashboard.putNumber("Yaw", (navX.getYaw() + 360.0) % 360.0);
		
		SmartDashboard.putNumber("DriveLeftFrontError", RobotMap.driveLeftFront.getClosedLoopError(0));
		SmartDashboard.putNumber("DriveLeftRearError", RobotMap.driveLeftRear.getClosedLoopError(0));
		SmartDashboard.putNumber("DriveRightFrontError", RobotMap.driveRightFront.getClosedLoopError(0));
		SmartDashboard.putNumber("DriveRightRearError", RobotMap.driveRightRear.getClosedLoopError(0));

		SmartDashboard.putNumber("ElevatorFront", RobotMap.elevatorFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("ElevatorRear", RobotMap.elevatorRear.getSelectedSensorPosition(0));
		
		SmartDashboard.putNumber("ElevatorFrontSpeed", RobotMap.elevatorFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("ElevatorTrajectoryPosition", RobotMap.elevatorFront.getActiveTrajectoryPosition());
		SmartDashboard.putNumber("ElevatorOutput", RobotMap.elevatorFront.getMotorOutputPercent());
	}
	
	@Override
	public void disabledInit() {
		RobotMap.elevatorFront.set(ControlMode.Disabled, 0.0);
	}
	
	@Override
	public void autonomousInit() {
		RobotMap.elevatorFront.setSelectedSensorPosition(0, 0, 0);
		driveLeftFront.setNeutralMode(NeutralMode.Brake);
		driveLeftRear.setNeutralMode(NeutralMode.Brake);
		driveRightFront.setNeutralMode(NeutralMode.Brake);
		driveRightRear.setNeutralMode(NeutralMode.Brake);
		
		autonomousCommand = RobotMap.autonomous.generateAutonomous();
		autonomousCommand.start();
	}
	
	@Override
	public void autonomousPeriodic() {
		
	}
	
	@Override
	public void teleopInit() {
		driveLeftFront.setNeutralMode(NeutralMode.Coast);
		driveLeftRear.setNeutralMode(NeutralMode.Coast);
		driveRightFront.setNeutralMode(NeutralMode.Coast);
		driveRightRear.setNeutralMode(NeutralMode.Coast);
		
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}
	
	@Override
	public void teleopPeriodic() {
		
	}
}

