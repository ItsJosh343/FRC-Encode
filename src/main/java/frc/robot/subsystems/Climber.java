// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Lift;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    
    private CANSparkMax liftMaster;
    private CANSparkMax liftSlave;

    private RelativeEncoder LMEncoder;

    private SparkMaxPIDController liftMasterCanController;
    
    public Climber() {
        liftMaster = new CANSparkMax(RobotMap.kLiftMaster, MotorType.kBrushless);
        liftSlave = new CANSparkMax(RobotMap.kLiftSlave, MotorType.kBrushless);
        liftSlave.follow(liftMaster,true);
        
        LMEncoder = liftMaster.getEncoder();

        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        liftMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,Lift.kUpwardLimit);

    }

    public double getLiftMasterPosition() {
        return LMEncoder.getPosition();
    }

    public void PIDSetPoint() {
        
    }

}
