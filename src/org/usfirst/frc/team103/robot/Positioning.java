package org.usfirst.frc.team103.robot;

import static org.usfirst.frc.team103.robot.RobotMap.*;

import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Positioning {
	
	private static final double WHEEL_BASE_LENGTH = 27.0, WHEEL_BASE_WIDTH = 22.0;
	private static final double DIAGONAL_LENGTH = Math.sqrt(Math.pow(WHEEL_BASE_WIDTH, 2.0) + Math.pow(WHEEL_BASE_LENGTH, 2.0)) / 2.0;
	private static final double DIAGONAL_ANGLE = Math.atan(WHEEL_BASE_WIDTH / WHEEL_BASE_LENGTH);
	private static final double ENCODER_SCALE = 1.0 / ((80.0 * 6.67) / (4.0 * Math.PI)), STEERING_SCALE = 2.0 * Math.PI / 1024.0;
	private static final int UPDATE_PERIOD = 5;
	private static final String[] MSE_LABELS = { "KeepAll", "DropLeftFront", "DropRightFront", "DropLeftRear", "DropRightRear" };
	private static final double OUTLIER_K = 1.5;
	
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
		executor.scheduleAtFixedRate(() -> update(), 0, UPDATE_PERIOD, TimeUnit.MILLISECONDS);
	}
	
	private void update() {
		double robotHeading = -Math.toRadians((navX.getYaw() + 360.0) % 360.0 - fieldZeroHeading);
		
		double encoderA = driveLeftFront.getSelectedSensorPosition(0) * ENCODER_SCALE;
		double encoderB = driveRightFront.getSelectedSensorPosition(0) * ENCODER_SCALE;
		double encoderC = driveLeftRear.getSelectedSensorPosition(0) * ENCODER_SCALE;
		double encoderD = driveRightRear.getSelectedSensorPosition(0) * ENCODER_SCALE;
		
		double distanceA = encoderA - prevEncoderA;
		double distanceB = encoderB - prevEncoderB;
		double distanceC = encoderC - prevEncoderC;
		double distanceD = encoderD - prevEncoderD;

		/*SmartDashboard.putNumber("RobotDistanceA", distanceA);
		SmartDashboard.putNumber("RobotDistanceB", distanceB);
		SmartDashboard.putNumber("RobotDistanceC", distanceC);
		SmartDashboard.putNumber("RobotDistanceD", distanceD);*/
		
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
		
		/*SmartDashboard.putNumber("RobotAngleA", Math.toDegrees(angleA) % 360.0);
		SmartDashboard.putNumber("RobotAngleB", Math.toDegrees(angleB) % 360.0);
		SmartDashboard.putNumber("RobotAngleC", Math.toDegrees(angleC) % 360.0);
		SmartDashboard.putNumber("RobotAngleD", Math.toDegrees(angleD) % 360.0);*/

		prevAngleA = currentAngleA;
		prevAngleB = currentAngleB;
		prevAngleC = currentAngleC;
		prevAngleD = currentAngleD;

		double prevCosPlus = Math.cos(prevRobotHeading + DIAGONAL_ANGLE);
		double prevCosMinus = Math.cos(prevRobotHeading - DIAGONAL_ANGLE);
		double prevSinPlus = Math.sin(prevRobotHeading + DIAGONAL_ANGLE);
		double prevSinMinus = Math.sin(prevRobotHeading - DIAGONAL_ANGLE);
		
		double prevAx = prevRobotX + DIAGONAL_LENGTH * prevCosPlus;
		double prevAy = prevRobotY + DIAGONAL_LENGTH * prevSinPlus;
		double prevBx = prevRobotX + DIAGONAL_LENGTH * prevCosMinus;
		double prevBy = prevRobotY + DIAGONAL_LENGTH * prevSinMinus;
		double prevCx = prevRobotX - DIAGONAL_LENGTH * prevCosMinus;
		double prevCy = prevRobotY - DIAGONAL_LENGTH * prevSinMinus;
		double prevDx = prevRobotX - DIAGONAL_LENGTH * prevCosPlus;
		double prevDy = prevRobotY - DIAGONAL_LENGTH * prevSinPlus;
		
		double projAx = prevAx + distanceA * Math.cos(angleA);
		double projAy = prevAy + distanceA * Math.sin(angleA);
		double projBx = prevBx + distanceB * Math.cos(angleB);
		double projBy = prevBy + distanceB * Math.sin(angleB);
		double projCx = prevCx + distanceC * Math.cos(angleC);
		double projCy = prevCy + distanceC * Math.sin(angleC);
		double projDx = prevDx + distanceD * Math.cos(angleD);
		double projDy = prevDy + distanceD * Math.sin(angleD);
		
		double cosPlus = Math.cos(robotHeading + DIAGONAL_ANGLE);
		double cosMinus = Math.cos(robotHeading - DIAGONAL_ANGLE);
		double sinPlus = Math.sin(robotHeading + DIAGONAL_ANGLE);
		double sinMinus = Math.sin(robotHeading - DIAGONAL_ANGLE);
		
		double robotXAll = (projAx + projBx + projCx + projDx) / 4.0;
		double robotXDropA = (projBx + projCx + projDx + DIAGONAL_LENGTH * cosPlus) / 3.0;
		double robotXDropB = (projAx + projCx + projDx + DIAGONAL_LENGTH * cosMinus) / 3.0;
		double robotXDropC = (projAx + projBx + projDx - DIAGONAL_LENGTH * cosMinus) / 3.0;
		double robotXDropD = (projAx + projBx + projCx - DIAGONAL_LENGTH * cosPlus) / 3.0;
		
		double robotYAll = (projAy + projBy + projCy + projDy) / 4.0;
		double robotYDropA = (projBy + projCy + projDy + DIAGONAL_LENGTH * sinPlus) / 3.0;
		double robotYDropB = (projAy + projCy + projDy + DIAGONAL_LENGTH * sinMinus) / 3.0;
		double robotYDropC = (projAy + projBy + projDy - DIAGONAL_LENGTH * sinMinus) / 3.0;
		double robotYDropD = (projAy + projBy + projCy - DIAGONAL_LENGTH * sinPlus) / 3.0;
		
		double mseAll = (Math.pow(robotXAll + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYAll + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXAll + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYAll + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXAll - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYAll - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)
				+ Math.pow(robotXAll - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYAll - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 4.0;
		double mseDropA = (Math.pow(robotXDropA + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYDropA + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXDropA - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYDropA - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)
				+ Math.pow(robotXDropA - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYDropA - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 3.0;
		double mseDropB = (Math.pow(robotXDropB + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYDropB + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXDropB - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYDropB - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)
				+ Math.pow(robotXDropB - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYDropB - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 3.0;
		double mseDropC = (Math.pow(robotXDropC + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYDropC + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXDropC + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYDropC + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXDropC - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYDropC - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 3.0;
		double mseDropD = (Math.pow(robotXDropD + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYDropD + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXDropD + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYDropD + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXDropD - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYDropD - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)) / 3.0;

		/*SmartDashboard.putNumber("MSEAll", mseAll);
		SmartDashboard.putNumber("MSEDropA", mseDropA);
		SmartDashboard.putNumber("MSEDropB", mseDropB);
		SmartDashboard.putNumber("MSEDropC", mseDropC);
		SmartDashboard.putNumber("MSEDropD", mseDropD);*/

		double robotXs[] = { robotXAll, robotXDropA, robotXDropB, robotXDropC, robotXDropD };
		double robotYs[] = { robotYAll, robotYDropA, robotYDropB, robotYDropC, robotYDropD };
		double mses[] = { mseAll, mseDropA, mseDropB, mseDropC, mseDropD };
		
		/*double mseMean = 0.0;
		for (int i = 0; i < mses.length; i++) mseMean += mses[i];
		mseMean /= mses.length;
		double mseVariance = 0.0;
		for (int i = 0; i < mses.length; i++) mseVariance += Math.pow(mses[i] - mseMean, 2.0);
		mseVariance /= mses.length;

		SmartDashboard.putNumber("MSEMean", mseMean);
		SmartDashboard.putNumber("MSEVariance", mseVariance);*/
		
		int mseMinIndex = 0;
		double mseMin = Double.MAX_VALUE;
		for (int i = 0; i < mses.length; i++) {
			if (mses[i] < mseMin) {
				mseMinIndex = i;
				mseMin = mses[i];
			}
		}
		Arrays.sort(mses);
		int positionIndex = (mseMin < mses[1] - OUTLIER_K * (mses[3] - mses[1]) ? mseMinIndex : 0);
		
		/*SmartDashboard.putNumber("MSEMinimum", mseMin);
		SmartDashboard.putNumber("MSEThreshold", mses[1] - OUTLIER_K * (mses[3] - mses[1]));*/
		/*SmartDashboard.putNumber("PositionChoiceIndex", positionIndex);
		SmartDashboard.putString("PositionChoice", MSE_LABELS[positionIndex]);*/
		
		double robotX = robotXs[positionIndex];
		double robotY = robotYs[positionIndex];
		
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
