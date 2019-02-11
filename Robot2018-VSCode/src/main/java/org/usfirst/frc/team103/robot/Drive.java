package org.usfirst.frc.team103.robot;

import org.usfirst.frc.team103.command.TeleopDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Drive extends Subsystem {
	public static final double WHEEL_BASE_LENGTH = 22.0; //double unit in inches
	public static final double WHEEL_BASE_WIDTH = 27.0;
	public static final double WHEEL_SPEED_MAXIMUM = 132.0; //11 * 12 (INCHES PER SECOND)
	public static final double DEADZONE = 0.08;
	//Robot goes 11 feet per second, converting it to inches per second ^
	
	public void swerveDrive(double strafe, double forward, double omega, boolean useDeadzone, boolean usePID, boolean useReverse) {
		if (useDeadzone) {
			strafe = (Math.abs(strafe) < DEADZONE ? 0.0 : strafe);
			forward = (Math.abs(forward) < DEADZONE ? 0.0 : forward);
			omega = (Math.abs(omega) < DEADZONE ? 0.0 : omega);
			
			if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
				if (usePID) {
					RobotMap.driveLeftFront.set(ControlMode.Velocity, 0.0);
					RobotMap.driveLeftRear.set(ControlMode.Velocity, 0.0);
					RobotMap.driveRightFront.set(ControlMode.Velocity, 0.0);
					RobotMap.driveRightRear.set(ControlMode.Velocity, 0.0);
				} else {
					RobotMap.driveLeftFront.set(ControlMode.PercentOutput, 0.0);
					RobotMap.driveLeftRear.set(ControlMode.PercentOutput, 0.0);
					RobotMap.driveRightFront.set(ControlMode.PercentOutput, 0.0);
					RobotMap.driveRightRear.set(ControlMode.PercentOutput, 0.0);
				}
				return;
			}
			
			if (usePID) {
				strafe = (strafe - Math.signum(strafe) * DEADZONE) / (1.0 - DEADZONE);
				forward = (forward - Math.signum(forward) * DEADZONE) / (1.0 - DEADZONE);
				omega = (omega - Math.signum(omega) * DEADZONE) / (1.0 - DEADZONE);
			}
		}
		
		strafe *= WHEEL_SPEED_MAXIMUM;   //Converting velocity to inches per second.
		forward *= WHEEL_SPEED_MAXIMUM;  // ^ 
		omega *= 2.0 * Math.PI;
		//strafe = x | forward = y
		double A = strafe - omega * WHEEL_BASE_LENGTH / 2;
		double B = strafe + omega * WHEEL_BASE_LENGTH / 2;
		double C = forward - omega * WHEEL_BASE_WIDTH / 2;
		double D = forward + omega * WHEEL_BASE_WIDTH / 2;
		
		//Wheel 2
		double leftFrontSpeed = Math.hypot(B, D);// / WHEEL_SPEED_MAXIMUM;
		double leftFrontAngle = (Math.atan2(B, D) * 180.0 / Math.PI + 360.0) % 360.0;
		
		//Wheel 1
		double rightFrontSpeed = Math.hypot(B, C);// / WHEEL_SPEED_MAXIMUM;
		double rightFrontAngle = (Math.atan2(B, C) * 180.0 / Math.PI + 360.0) % 360.0;
		
		//Wheel 3
		double leftRearSpeed = Math.hypot(A, D);// / WHEEL_SPEED_MAXIMUM;
		double leftRearAngle = (Math.atan2(A, D) * 180.0 / Math.PI + 360.0) % 360.0;
		
		//Wheel 4
		double rightRearSpeed = Math.hypot(A, C);// / WHEEL_SPEED_MAXIMUM; //Take all speeds
		double rightRearAngle = (Math.atan2(A, C) * 180.0 / Math.PI + 360.0) % 360.0;
		
		double leftFrontPosition = RobotMap.steerLeftFront.getSelectedSensorPosition(0) * 360.0 / 1024.0;
		double leftFrontDelta = leftFrontAngle - (leftFrontPosition % 360.0);
		if (Math.abs(leftFrontDelta) > 180.0) {
			leftFrontDelta -= 360.0 * Math.signum(leftFrontDelta);
		}
		if (useReverse && Math.abs(leftFrontDelta) > 90.0) {
			leftFrontDelta -= 180.0 * Math.signum(leftFrontDelta);
			leftFrontSpeed = -leftFrontSpeed;
		}
		leftFrontAngle = (leftFrontPosition + leftFrontDelta) * 1024.0 / 360.0;
		
		double leftRearPosition = RobotMap.steerLeftRear.getSelectedSensorPosition(0) * 360.0 / 1024.0;
		double leftRearDelta = leftRearAngle - (leftRearPosition % 360.0);
		if (Math.abs(leftRearDelta) > 180.0) {
			leftRearDelta -= 360.0 * Math.signum(leftRearDelta);
		}
		if (useReverse && Math.abs(leftRearDelta) > 90.0) {
			leftRearDelta -= 180.0 * Math.signum(leftRearDelta);
			leftRearSpeed = -leftRearSpeed;
		}
		leftRearAngle = (leftRearPosition + leftRearDelta) * 1024.0 / 360.0;
		
		double rightFrontPosition = RobotMap.steerRightFront.getSelectedSensorPosition(0) * 360.0 / 1024.0;
		double rightFrontDelta = rightFrontAngle - (rightFrontPosition % 360.0);
		if (Math.abs(rightFrontDelta) > 180.0) {
			rightFrontDelta -= 360.0 * Math.signum(rightFrontDelta);
		}
		if (useReverse && Math.abs(rightFrontDelta) > 90.0) {
			rightFrontDelta -= 180.0 * Math.signum(rightFrontDelta);
			rightFrontSpeed = -rightFrontSpeed;
		}
		rightFrontAngle = (rightFrontPosition + rightFrontDelta) * 1024.0 / 360.0;
		
		double rightRearPosition = RobotMap.steerRightRear.getSelectedSensorPosition(0) * 360.0 / 1024.0;
		double rightRearDelta = rightRearAngle - (rightRearPosition % 360.0);
		if (Math.abs(rightRearDelta) > 180.0) {
			rightRearDelta -= 360.0 * Math.signum(rightRearDelta);
		}
		if (useReverse && Math.abs(rightRearDelta) > 90.0) {
			rightRearDelta -= 180.0 * Math.signum(rightRearDelta);
			rightRearSpeed = -rightRearSpeed;
		}
		rightRearAngle = (rightRearPosition + rightRearDelta) * 1024.0 / 360.0;
		
		RobotMap.steerLeftFront.set(ControlMode.Position, leftFrontAngle);
		RobotMap.steerLeftRear.set(ControlMode.Position, leftRearAngle);
		RobotMap.steerRightFront.set(ControlMode.Position, rightFrontAngle);
		RobotMap.steerRightRear.set(ControlMode.Position, rightRearAngle);
		
		if (usePID) {
			RobotMap.driveLeftFront.set(ControlMode.Velocity, leftFrontSpeed * 600.0 / WHEEL_SPEED_MAXIMUM);
			RobotMap.driveLeftRear.set(ControlMode.Velocity, leftRearSpeed * 600.0 / WHEEL_SPEED_MAXIMUM);
			RobotMap.driveRightFront.set(ControlMode.Velocity, rightFrontSpeed * 600.0 / WHEEL_SPEED_MAXIMUM);
			RobotMap.driveRightRear.set(ControlMode.Velocity, rightRearSpeed * 600.0 / WHEEL_SPEED_MAXIMUM);
		} else {
			RobotMap.driveLeftFront.set(ControlMode.PercentOutput, leftFrontSpeed / WHEEL_SPEED_MAXIMUM);
			RobotMap.driveLeftRear.set(ControlMode.PercentOutput, leftRearSpeed / WHEEL_SPEED_MAXIMUM);
			RobotMap.driveRightFront.set(ControlMode.PercentOutput, rightFrontSpeed / WHEEL_SPEED_MAXIMUM);
			RobotMap.driveRightRear.set(ControlMode.PercentOutput, rightRearSpeed / WHEEL_SPEED_MAXIMUM);
		}
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}
}

