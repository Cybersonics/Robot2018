package org.usfirst.frc.team103.robot;

import static org.usfirst.frc.team103.robot.Commands.*;
import static org.usfirst.frc.team103.robot.RobotMap.*;

import org.usfirst.frc.team103.command.DoubleRightSwitch;
import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.command.MiddleCubes;
import org.usfirst.frc.team103.command.OppositeScale;
import org.usfirst.frc.team103.command.RightSwitchLeftScale;
import org.usfirst.frc.team103.command.TeleopDrive;
import org.usfirst.frc.team103.command.RightSwitchRightScale;
import org.usfirst.frc.team103.pixy.Pixy;
import org.usfirst.frc.team103.pixy.Pixy.ExposureSetting;
import org.usfirst.frc.team103.pixy.Pixy.WhiteBalanceSetting;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	@Override
	public void robotInit() {
		//Jacob was here
		//Trevor wasn't here
		
		RobotMap.init();
        
        SmartDashboard.putData("Enumerate", instantCommand(() -> Pixy.enumerate(), true));
        SmartDashboard.putData("GetParameters", instantCommand(() -> {
    		try {
        		pixy.stopBlockProgram();
        		pixy.stopFrameGrabber();
        		Thread.sleep(200);
        		getParameters();
        		Thread.sleep(200);
        		pixy.startBlockProgram();
        		pixy.startFrameGrabber();
    		} catch (InterruptedException e) {
    			e.printStackTrace();
    		}
    	}, true));
        SmartDashboard.putData("SetParameters", instantCommand(() -> {
    		try {
        		pixy.stopBlockProgram();
        		pixy.stopFrameGrabber();
        		Thread.sleep(200);
        		setParameters();
        		Thread.sleep(200);
        		pixy.startBlockProgram();
        		pixy.startFrameGrabber();
    		} catch (InterruptedException e) {
    			e.printStackTrace();
    		}
    	}, true));

        new JoystickButton(leftJoy, 6).whenPressed(instantCommand(() -> {
        	fieldZeroHeading = (navX.getYaw() + 360.0) % 360.0;
        	positioning.setOrigin();
        }, true));
        new JoystickButton(leftJoy, 8).whenPressed(new TeleopDrive());
        new JoystickButton(leftJoy, 9).whenPressed(new MiddleCubes());
        new JoystickButton(leftJoy, 10).whenReleased(new OppositeScale());
        new JoystickButton(leftJoy, 11).whenReleased(new RightSwitchRightScale());
        new JoystickButton(rightJoy, 6).whenReleased(new DoubleRightSwitch());
        new JoystickButton(rightJoy, 7).whenReleased(new RightSwitchLeftScale());
        
        
        new JoystickButton(rightJoy, 8).whenPressed(instantCommand(() -> {
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
        new JoystickButton(rightJoy, 11).whenPressed(instantCommand(() -> TeleopDrive.usePID = true, true));
	}
	
	@Override
	public void robotPeriodic() {
		/*SmartDashboard.putNumber("Left Front Speed", RobotMap.driveLeftFront.getSpeed());
		SmartDashboard.putNumber("Left Rear Speed", RobotMap.driveLeftRear.getSpeed());
		SmartDashboard.putNumber("Right Front Speed", RobotMap.driveRightFront.getSpeed());
		SmartDashboard.putNumber("Right Rear Speed", RobotMap.driveRightRear.getSpeed()); */
		
		Scheduler.getInstance().run();
		Vision.findCube();
		SmartDashboard.putNumber("SteerLeftFront", RobotMap.steerLeftFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerLeftRear", RobotMap.steerLeftRear.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerRightFront", RobotMap.steerRightFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerRightRear", RobotMap.steerRightRear.getSelectedSensorPosition(0));

		SmartDashboard.putNumber("DriveLeftFront", RobotMap.driveLeftFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveLeftRear", RobotMap.driveLeftRear.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveRightFront", RobotMap.driveRightFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveRightRear", RobotMap.driveRightRear.getSelectedSensorPosition(0));
		
		SmartDashboard.putNumber("FusedHeading", RobotMap.navX.getFusedHeading());
		SmartDashboard.putNumber("Yaw", (navX.getYaw() + 360.0) % 360.0);
		
		SmartDashboard.putNumber("Ultrasonic1", ultrasonic1.getRangeInches());
	}

	@Override
	public void autonomousInit() {
	}

	
	@Override
	public void autonomousPeriodic() {
		
		
	}


	@Override
	public void teleopPeriodic() {
		/*if (RobotMap.rightJoy.getTrigger()) {
			CubeTarget target = Vision.findCube();
			if (target != null) {
				double omega = (double) (target.x - 160) / 160.0 * 0.5;
				RobotMap.drive.swerveDrive(0.0, 1.0, omega, true, true);
			} else {
				RobotMap.drive.swerveDrive(0, 0, 0, true, true);
			}
		} else if (RobotMap.leftJoy.getRawButton(10)) {
			double distance = (Math.abs(RobotMap.driveLeftFront.getSelectedSensorPosition(0))
					+ Math.abs(RobotMap.driveLeftRear.getSelectedSensorPosition(0))
					+ Math.abs(RobotMap.driveRightFront.getSelectedSensorPosition(0))
					+ Math.abs(RobotMap.driveRightRear.getSelectedSensorPosition(0))) / 4.0;
			SmartDashboard.putNumber("DriveDistance", distance);
			if (distance < 2000.0) {
				RobotMap.drive.swerveDrive(0.0, 0.5, 0.0, true, true);
			} else {
				RobotMap.drive.swerveDrive(0.0, 0.0, 0.0, true, true);
			}
		}*/
	}

	
	@Override
	public void testPeriodic() {
	}
	
	private void getParameters() {
		boolean aec = pixy.getAutoExposure();
		boolean awb = pixy.getAutoWhiteBalance();
		ExposureSetting exp = pixy.getExposureCompensation();
		WhiteBalanceSetting wbv = pixy.getWhiteBalanceValue();
		SmartDashboard.putBoolean("AutoExposure", aec);
		SmartDashboard.putBoolean("AutoWhiteBalance", awb);
		SmartDashboard.putNumber("ExposureGain", exp.gain);
		SmartDashboard.putNumber("ExposureCompensation", exp.compensation);
		SmartDashboard.putNumber("WhiteBalanceRed", wbv.red);
		SmartDashboard.putNumber("WhiteBalanceGreen", wbv.green);
		SmartDashboard.putNumber("WhiteBalanceBlue", wbv.blue);
	}

	private void setParameters() {
		boolean aec = SmartDashboard.getBoolean("AutoExposure", false);
		boolean awb = SmartDashboard.getBoolean("AutoWhiteBalance", false);
		int gain = (int) SmartDashboard.getNumber("ExposureGain", 20);
		int comp = (int) SmartDashboard.getNumber("ExposureCompensation", 100);
		int r = (int) SmartDashboard.getNumber("WhiteBalanceRed", 64);
		int g = (int) SmartDashboard.getNumber("WhiteBalanceGreen", 64);
		int b = (int) SmartDashboard.getNumber("WhiteBalanceBlue", 64);
		pixy.setAutoExposure(aec);
		if (!aec) {
			pixy.setExposureCompensation(new ExposureSetting(gain, comp));
		}
		pixy.setAutoWhiteBalance(awb);
		if (!awb) {
			pixy.setWhiteBalanceValue(new WhiteBalanceSetting(r, g, b));
		}
	}
}

