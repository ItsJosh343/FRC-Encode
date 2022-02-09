// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Lift;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    
    private CANSparkMax liftMaster;
    private CANSparkMax liftSlave;

    private SparkMaxPIDController liftMasterController;
    private SparkMaxPIDController liftSlaveController;
    
    public Climber() {
        //initalize Spark Max motors
        liftMaster = new CANSparkMax(RobotMap.kLiftMaster, MotorType.kBrushless);
        liftSlave = new CANSparkMax(RobotMap.kLiftSlave, MotorType.kBrushless);
        liftMaster.restoreFactoryDefaults();
        liftSlave.restoreFactoryDefaults();

        //assign PID and Motor contoller values
        liftMaster.setInverted(false);
        liftSlave.setInverted(true);

        liftMasterController = liftMaster.getPIDController();
        liftSlaveController = liftSlave.getPIDController();

        liftMasterController.setOutputRange(-1, 1);
        liftMasterController.setSmartMotionMaxVelocity(Lift.kLiftSpeed, 0);
        liftMasterController.setSmartMotionMaxAccel(10, 0);

        liftSlaveController.setOutputRange(-1, 1);
        liftSlaveController.setSmartMotionMaxVelocity(Lift.kLiftSpeed, 0);
        liftSlaveController.setSmartMotionMaxAccel(10, 0);

        //Limit the motion of the lift to only turn within a certain range.
        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        liftMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,Lift.kUpwardLimit);

        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        liftMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,Lift.kDownwardLimit);

        liftSlave.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        liftSlave.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,Lift.kUpwardLimit);

        liftSlave.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        liftSlave.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,Lift.kDownwardLimit);
    }

    //Run PID to targetLeft with given values for P, I, and D.
    public void PIDSetPoint(double targetVelocity,double P, double I, double D){
        liftMasterController.setP(P);
        liftMasterController.setI(I);
        liftMasterController.setD(D);

        liftSlaveController.setP(P);
        liftSlaveController.setI(I);
        liftSlaveController.setD(D);

        liftMasterController.setReference(targetVelocity, CANSparkMax.ControlType.kSmartVelocity);
        liftSlaveController.setReference(targetVelocity, CANSparkMax.ControlType.kSmartVelocity);
    }

    //Enable/disable softLimits for testing.
    public void enableLimits(boolean enable) {
        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enable);
        liftSlave.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enable);
        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enable);
        liftSlave.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enable);
    }

    public void runPID(double targetVelocity,double P, double I, double D,boolean enable)
    {
        PIDSetPoint(targetVelocity,P,I,D);
        enableLimits(enable);
    }

    //manually set the speed of lift
    public void liftDrive(double speed) {
        liftMaster.set(speed);
        liftSlave.set(speed);
    }

    //stop all lift movement
    public void stop() {
        liftMaster.set(0);
        liftSlave.set(0);
    }

}
