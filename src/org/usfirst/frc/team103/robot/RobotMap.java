package org.usfirst.frc.team103.robot;

import org.usfirst.frc.team103.pixy.Pixy;
import org.usfirst.frc.team103.pixy.Pixy.ExposureSetting;
import org.usfirst.frc.team103.pixy.Pixy.WhiteBalanceSetting;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

public class RobotMap {
    public static TalonSRX driveLeftFront;
    public static TalonSRX driveLeftRear;
    public static TalonSRX driveRightFront;
    public static TalonSRX driveRightRear;
    public static TalonSRX steerLeftFront;
    public static TalonSRX steerLeftRear;
    public static TalonSRX steerRightFront;
    public static TalonSRX steerRightRear;
    public static Drive drive;
	
    public static Joystick leftJoy; 
    public static Joystick rightJoy;
    
    private static final double DRIVE_P = 6.0, DRIVE_I = 0.005, DRIVE_D = 1.0, DRIVE_F = 0.0, DRIVE_RAMP_RATE = 0.2;
    private static final int DRIVE_I_ZONE = 300, DRIVE_ALLOWABLE_ERROR = 15, DRIVE_MEASUREMENT_WINDOW = 1;
    private static final VelocityMeasPeriod DRIVE_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_10Ms;
    
    public static Pixy pixy;
    
    public static AHRS navX;
    public static double fieldZeroHeading;
    public static Positioning positioning;
	
	public static void init(){
        driveLeftFront = new TalonSRX(10);
        /*driveLeftFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveLeftFront.reverseSensor(true);*/
        driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveLeftFront.config_kP(0, DRIVE_P, 0);
        driveLeftFront.config_kI(0, DRIVE_I, 0);
        driveLeftFront.config_kD(0, DRIVE_D, 0);
        driveLeftFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveLeftFront.config_kF(0, DRIVE_F, 0);
        driveLeftFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveLeftFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveLeftFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveLeftFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*driveLeftFront.setIZone(DRIVE_I_ZONE);
        driveLeftFront.setF(DRIVE_F);
        driveLeftFront.setAllowableClosedLoopErr(DRIVE_ALLOWABLE_ERROR);
        driveLeftFront.setCloseLoopRampRate(DRIVE_RAMP_RATE);
        driveLeftFront.SetVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD);
        driveLeftFront.SetVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW);
        driveLeftFront.set(TalonControlMode.Speed);
        LiveWindow.addActuator("Drive", "LeftFrontDrive", driveLeftFront);*/
        
        
        driveLeftRear = new TalonSRX(11);
        driveLeftRear.setSensorPhase(true);
        /*driveLeftRear.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveLeftRear.reverseSensor(true);*/
        driveLeftRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveLeftRear.config_kP(0, DRIVE_P, 0);
        driveLeftRear.config_kI(0, DRIVE_I, 0);
        driveLeftRear.config_kD(0, DRIVE_D, 0);
        driveLeftRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveLeftRear.config_kF(0, DRIVE_F, 0);
        driveLeftRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveLeftRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveLeftRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveLeftRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*driveLeftRear.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveLeftRear.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveLeftRear.reverseSensor(true);
        driveLeftRear.setPID(DRIVE_P, DRIVE_I, DRIVE_D);
        driveLeftRear.setIZone(DRIVE_I_ZONE);
        driveLeftRear.setF(DRIVE_F);
        driveLeftRear.setAllowableClosedLoopErr(DRIVE_ALLOWABLE_ERROR);
        driveLeftRear.setCloseLoopRampRate(DRIVE_RAMP_RATE);
        driveLeftRear.changeControlMode(TalonControlMode.Speed);
        driveLeftRear.SetVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD);
        driveLeftRear.SetVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW);
        LiveWindow.addActuator("Drive", "LeftRearDrive", driveLeftRear);
        */
        
        driveRightFront = new TalonSRX(12);
        driveRightFront.setInverted(true);
        /*driveRightFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveRightFront.reverseSensor(true);*/
        driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveRightFront.config_kP(0, DRIVE_P, 0);
        driveRightFront.config_kI(0, DRIVE_I, 0);
        driveRightFront.config_kD(0, DRIVE_D, 0);
        driveRightFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveRightFront.config_kF(0, DRIVE_F, 0);
        driveRightFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveRightFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveRightFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveRightFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*driveRightFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveRightFront.setInverted(true);
        driveRightFront.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveRightFront.reverseSensor(true);
        driveRightFront.reverseOutput(true);
        driveRightFront.setPID(DRIVE_P, DRIVE_I, DRIVE_D);
        driveRightFront.setIZone(DRIVE_I_ZONE);
        driveRightFront.setF(DRIVE_F);
        driveRightFront.setAllowableClosedLoopErr(DRIVE_ALLOWABLE_ERROR);
        driveRightFront.setCloseLoopRampRate(DRIVE_RAMP_RATE);
        driveRightFront.changeControlMode(TalonControlMode.Speed);
        driveRightFront.SetVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD);
        driveRightFront.SetVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW);
        LiveWindow.addActuator("Drive", "RightFrontDrive", driveRightFront); */
        
        driveRightRear = new TalonSRX(13);
        driveRightRear.setInverted(true);
        /*driveRightRear.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveRightRear.reverseSensor(true);*/
        driveRightRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveRightRear.config_kP(0, DRIVE_P, 0);
        driveRightRear.config_kI(0, DRIVE_I, 0);
        driveRightRear.config_kD(0, DRIVE_D, 0);
        driveRightRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveRightRear.config_kF(0, DRIVE_F, 0);
        driveRightRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveRightRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveRightRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveRightRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*driveRightRear.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveRightRear.setInverted(true);
        driveRightRear.reverseSensor(true);
        driveRightRear.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        driveRightRear.reverseSensor(true);
        driveRightRear.reverseOutput(true);
        driveRightRear.setPID(DRIVE_P, DRIVE_I, DRIVE_D);
        driveRightRear.setIZone(DRIVE_I_ZONE);
        driveRightRear.setF(DRIVE_F);
        driveRightRear.setAllowableClosedLoopErr(DRIVE_ALLOWABLE_ERROR);
        driveRightRear.setCloseLoopRampRate(DRIVE_RAMP_RATE);
        driveRightRear.changeControlMode(TalonControlMode.Speed);
        driveRightRear.SetVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD);
        driveRightRear.SetVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW);
        LiveWindow.addActuator("Drive", "RightRearDrive", driveRightRear); */
        
        steerLeftFront = new TalonSRX(16);
        
        steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftFront.config_kP(0, 10.0, 0);
        steerLeftFront.config_kI(0, 0.02, 0);
        steerLeftFront.config_kD(0, 0.0, 0);
        steerLeftFront.config_IntegralZone(0, 100, 0);
        steerLeftFront.configAllowableClosedloopError(0, 5, 0);
        steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*steerLeftFront.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
        steerLeftFront.setPID(10.0, 0.02, 0.0);
        steerLeftFront.setIZone(100);
        steerLeftFront.setAllowableClosedLoopErr(5);
        steerLeftFront.changeControlMode(TalonControlMode.Position);
        LiveWindow.addActuator("Steer", "LeftFrontSteer", steerLeftFront); */
        
        steerLeftRear = new TalonSRX(17);
        
        steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftRear.config_kP(0, 10.0, 0);
        steerLeftRear.config_kI(0, 0.02, 0);
        steerLeftRear.config_kD(0, 0.0, 0);
        steerLeftRear.config_IntegralZone(0, 100, 0);
        steerLeftRear.configAllowableClosedloopError(0, 5, 0);
        steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*steerLeftRear.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
        steerLeftRear.setPID(10.0, 0.02, 0.0);
        steerLeftRear.setIZone(100);
        steerLeftRear.setAllowableClosedLoopErr(5);
        steerLeftRear.changeControlMode(TalonControlMode.Position);
        LiveWindow.addActuator("Steer", "LeftRearSteer", steerLeftRear); */
        
        steerRightFront = new TalonSRX(18);
        
        steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightFront.config_kP(0, 10.0, 0);
        steerRightFront.config_kI(0, 0.02, 0);
        steerRightFront.config_kD(0, 0.0, 0);
        steerRightFront.config_IntegralZone(0, 100, 0);
        steerRightFront.configAllowableClosedloopError(0, 5, 0);
        steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*steerRightFront.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
        steerRightFront.setPID(10.0, 0.02, 0.0);
        steerRightFront.setIZone(100);
        steerRightFront.setAllowableClosedLoopErr(5);
        steerRightFront.changeControlMode(TalonControlMode.Position);
        //LiveWindow.addActuator("Steer", "RightFrontSteer", steerRightFront); */
        
        steerRightRear = new TalonSRX(19);
        
        steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightRear.config_kP(0, 10.0, 0);
        steerRightRear.config_kI(0, 0.02, 0);
        steerRightRear.config_kD(0, 0.0, 0);
        steerRightRear.config_IntegralZone(0, 100, 0);
        steerRightRear.configAllowableClosedloopError(0, 5, 0);
        steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 0);
        
        /*steerRightRear.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
        steerRightRear.setPID(10.0, 0.02, 0.0);
        steerRightRear.setIZone(100);
        steerRightRear.setAllowableClosedLoopErr(5);
        steerRightRear.changeControlMode(TalonControlMode.Position);
        //LiveWindow.addActuator("Steer", "RightRearSteer", steerRightRear);*/
        
        drive = new Drive();
        
        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
		
		Pixy.ensureAvailable(0xF8F5B551);
		pixy = new Pixy(0xF8F5B551);
		try {
			Thread.sleep(200);
			pixy.setAutoExposure(false);
			pixy.setExposureCompensation(new ExposureSetting(20, 150));
			pixy.setAutoWhiteBalance(false);
			pixy.setWhiteBalanceValue(new WhiteBalanceSetting(72, 64, 122));
			Thread.sleep(200);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		pixy.startBlockProgram();
		pixy.startFrameGrabber();
		
		navX = new AHRS(SPI.Port.kMXP);
		
		positioning = new Positioning();
		
		/*for (StatusFrameEnhanced frame : StatusFrameEnhanced.values()) {
			System.out.println(frame.name() + ": " + driveLeftFront.getStatusFramePeriod(frame, 0));
		}*/
	}
	
	
}
