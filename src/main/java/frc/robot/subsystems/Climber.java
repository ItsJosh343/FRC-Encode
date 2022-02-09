// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Lift;

/** Add your docs here. */
public class Climber extends SubsystemBase{
    
    private CANSparkMax liftMaster;
    private CANSparkMax liftSlave;
    
    public Climber() {
        //assign CANSparkMax to appropriate values
        liftMaster = new CANSparkMax(RobotMap.kLiftMaster, MotorType.kBrushless);
        liftSlave = new CANSparkMax(RobotMap.kLiftSlave, MotorType.kBrushless);
        liftMaster.setInverted(false);
        liftSlave.follow(liftMaster,true);

        //Limit the motion of the lift to only turn within a certain range.
        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,true);
        liftMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,Lift.kUpwardLimit);

        liftMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,true);
        liftMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,Lift.kDownwardLimit);

    }

    public void liftDrive(double speed) {
        liftMaster.set(speed);
    }

    public void stop() {
        liftMaster.set(0);
    }

}
