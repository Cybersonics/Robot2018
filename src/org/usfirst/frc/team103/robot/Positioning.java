package org.usfirst.frc.team103.robot;

import static org.usfirst.frc.team103.robot.RobotMap.*;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Positioning {
	
	public static final double WHEEL_BASE_LENGTH = 27.0, WHEEL_BASE_WIDTH = 22.0;
	public static final double DIAGONAL_LENGTH = Math.sqrt(WHEEL_BASE_WIDTH * WHEEL_BASE_WIDTH + WHEEL_BASE_LENGTH * WHEEL_BASE_LENGTH) / 2.0;
	public static final double DIAGONAL_ANGLE = Math.atan(WHEEL_BASE_WIDTH / WHEEL_BASE_LENGTH);
	public static final double ENCODER_SCALE = 1.0 / ((80.0 * 6.67) / (4.0 * Math.PI)), STEERING_SCALE = 2.0 * Math.PI / 1024.0;
	
	private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor(new ThreadFactory() {
		@Override
		public Thread newThread(Runnable r) {
			Thread t = Executors.defaultThreadFactory().newThread(r);
			t.setDaemon(true);
			return t;
		}
	});

	private volatile double prevRobotX, prevRobotY, prevRobotHeading;
	private double prevEncoderA, prevEncoderB, prevEncoderC, prevEncoderD;
	private double prevAngleA, prevAngleB, prevAngleC, prevAngleD;
	
	public Positioning() {
		executor.scheduleAtFixedRate(() -> update(), 0, 10, TimeUnit.MILLISECONDS);
	}
	
	private void update() {
		//TODO: Figure out why this isn't rotated by 90 degrees
		//double robotHeading = -Math.toRadians(navX.getFusedHeading() - fieldZeroHeading);
		double robotHeading = -Math.toRadians((navX.getYaw() + 360.0) % 360.0 - fieldZeroHeading);
		
		//SmartDashboard.putNumber("RobotHeading", Math.toDegrees(robotHeading) % 360.0);
		
		double encoderA = driveLeftFront.getSelectedSensorPosition(0) * ENCODER_SCALE;
		double encoderB = driveRightFront.getSelectedSensorPosition(0) * ENCODER_SCALE;
		double encoderC = driveLeftRear.getSelectedSensorPosition(0) * ENCODER_SCALE;
		double encoderD = driveRightRear.getSelectedSensorPosition(0) * ENCODER_SCALE;
		
		double distanceA = encoderA - prevEncoderA;
		double distanceB = encoderB - prevEncoderB;
		double distanceC = encoderC - prevEncoderC;
		double distanceD = encoderD - prevEncoderD;

		SmartDashboard.putNumber("RobotDistanceA", distanceA);
		SmartDashboard.putNumber("RobotDistanceB", distanceB);
		SmartDashboard.putNumber("RobotDistanceC", distanceC);
		SmartDashboard.putNumber("RobotDistanceD", distanceD);
		
		prevEncoderA = encoderA;
		prevEncoderB = encoderB;
		prevEncoderC = encoderC;
		prevEncoderD = encoderD;
		
		double currentAngleA = robotHeading + steerLeftFront.getSelectedSensorPosition(0) * STEERING_SCALE + Math.PI / 2.0;
		double currentAngleB = robotHeading + steerRightFront.getSelectedSensorPosition(0) * STEERING_SCALE + Math.PI / 2.0;
		double currentAngleC = robotHeading + steerLeftRear.getSelectedSensorPosition(0) * STEERING_SCALE + Math.PI / 2.0;
		double currentAngleD = robotHeading + steerRightRear.getSelectedSensorPosition(0) * STEERING_SCALE + Math.PI / 2.0;

		double angleA = (prevAngleA + currentAngleA) / 2.0;
		double angleB = (prevAngleB + currentAngleB) / 2.0;
		double angleC = (prevAngleC + currentAngleC) / 2.0;
		double angleD = (prevAngleD + currentAngleD) / 2.0;
		
		SmartDashboard.putNumber("RobotAngleA", Math.toDegrees(angleA) % 360.0);
		SmartDashboard.putNumber("RobotAngleB", Math.toDegrees(angleB) % 360.0);
		SmartDashboard.putNumber("RobotAngleC", Math.toDegrees(angleC) % 360.0);
		SmartDashboard.putNumber("RobotAngleD", Math.toDegrees(angleD) % 360.0);

		prevAngleA = currentAngleA;
		prevAngleB = currentAngleB;
		prevAngleC = currentAngleC;
		prevAngleD = currentAngleD;
		
		double prevAx = prevRobotX + DIAGONAL_LENGTH * Math.cos(prevRobotHeading + DIAGONAL_ANGLE);
		double prevAy = prevRobotY + DIAGONAL_LENGTH * Math.sin(prevRobotHeading + DIAGONAL_ANGLE);
		double prevBx = prevRobotX + DIAGONAL_LENGTH * Math.cos(prevRobotHeading - DIAGONAL_ANGLE);
		double prevBy = prevRobotY + DIAGONAL_LENGTH * Math.sin(prevRobotHeading - DIAGONAL_ANGLE);
		double prevCx = prevRobotX - DIAGONAL_LENGTH * Math.cos(prevRobotHeading - DIAGONAL_ANGLE);
		double prevCy = prevRobotY - DIAGONAL_LENGTH * Math.sin(prevRobotHeading - DIAGONAL_ANGLE);
		double prevDx = prevRobotX - DIAGONAL_LENGTH * Math.cos(prevRobotHeading + DIAGONAL_ANGLE);
		double prevDy = prevRobotY - DIAGONAL_LENGTH * Math.sin(prevRobotHeading + DIAGONAL_ANGLE);
		
		double projAx = prevAx + distanceA * Math.cos(angleA);
		double projAy = prevAy + distanceA * Math.sin(angleA);
		double projBx = prevBx + distanceB * Math.cos(angleB);
		double projBy = prevBy + distanceB * Math.sin(angleB);
		double projCx = prevCx + distanceC * Math.cos(angleC);
		double projCy = prevCy + distanceC * Math.sin(angleC);
		double projDx = prevDx + distanceD * Math.cos(angleD);
		double projDy = prevDy + distanceD * Math.sin(angleD);

		double robotX = (projAx + projBx + projCx + projDx) / 4.0;
		double robotY = (projAy + projBy + projCy + projDy) / 4.0;

		//SmartDashboard.putNumber("RobotX", robotX);
		//SmartDashboard.putNumber("RobotY", robotY);
		
		prevRobotX = robotX;
		prevRobotY = robotY;
		prevRobotHeading = robotHeading;
		
		SmartDashboard.putNumber("RobotX", getX());
		SmartDashboard.putNumber("RobotY", getY());
		SmartDashboard.putNumber("RobotHeading", getHeading());
	}
	
	public void setOrigin() {
		prevRobotX = 0.0;
		prevRobotY = 0.0;
	}
	
	public double getX() {
		return prevRobotX;
	}
	
	public double getY() {
		return prevRobotY;
	}
	
	public double getHeading() {
		double heading = -Math.toDegrees(prevRobotHeading) % 360.0;
		if (heading < 0) heading += 360.0;
		return heading;
	}

}
