// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class Drivetrain extends SubsystemBase {

     // declare PID Controllers
    private CANSparkMax leftMaster;
    private RelativeEncoder LMEncoder;
    private SparkMaxPIDController leftMasterCanController;
    
    private double kPDrive, kIDrive, kIZoneDrive, kDDrive, kFFDrive, kForwardRR, kStrafeRR, kIsMoreStrafe;

    // public DifferentialDrive drive;
    public Drivetrain() {

        leftMaster = new CANSparkMax(RobotMap.kLeftMaster, MotorType.kBrushless);
        LMEncoder = leftMaster.getEncoder();
        leftMasterCanController = leftMaster.getPIDController();
        
        //Define PID Values
        kPDrive = 5e-20;
        kIDrive = 1e-6 + 1e-3;
        kIZoneDrive = 0;
        kDDrive = 0;
        kFFDrive = 0.000156;
        kForwardRR = 0;
        kStrafeRR = 0;
        kIsMoreStrafe = .5;

        //assign PID Values for PID Controllers
        leftMasterCanController.setP(kPDrive);
        leftMasterCanController.setI(kIDrive);
        leftMasterCanController.setIZone(kIZoneDrive);
        leftMasterCanController.setD(kDDrive);
        leftMasterCanController.setFF(kFFDrive);
        leftMasterCanController.setOutputRange(-1, 1);
        leftMasterCanController.setSmartMotionMaxVelocity(10000, 0);
        leftMasterCanController.setSmartMotionMaxAccel(2000, 0);

    }

    // Sets new PID Values according to inputs
    public void PIDUpdate(double P, double I, double D) {

        leftMasterCanController.setP(P);
        leftMasterCanController.setI(I);
        leftMasterCanController.setD(D);
    }

    // Returns the LeftMaster encoder Position
    public double getLeftMasterPosition() {

        return LMEncoder.getPosition();
    }

    // Enables motor to drive to PID Target location relative to where it is now
    public void PIDSetPoint(double targetLeft){

        leftMasterCanController.setReference(LMEncoder.getPosition() + targetLeft, CANSparkMax.ControlType.kSmartMotion);
    }

    // Enables motor to dirve to PID absolute encoder position of target
    public void PIDSetPointAbsolute(double targetLeft) {

        leftMasterCanController.setReference(targetLeft, CANSparkMax.ControlType.kSmartMotion);
    }
}


//Deprecated (?) code:


// THE FOLLOWING NEEDS TO BE DECLARED IN CONSTANTS (PID DRIVE RELATED)
// CANSparkMax(device number, type of motor (NEO brushless in this case))
/*
 * leftMaster = new CANSparkMax(RobotMap.kLeftMaster, MotorType.kBrushless);
 * //CAN 3
 * leftSlave = new CANSparkMax(RobotMap.kLeftSlave, MotorType.kBrushless); //CAN
 * 1
 * rightMaster = new CANSparkMax(RobotMap.kRightMaster, MotorType.kBrushless);
 * //CAN 2
 * rightSlave = new CANSparkMax(RobotMap.kRightSlave, MotorType.kBrushless);
 * //CAN 4
 * drive = new DifferentialDrive(leftMaster, rightMaster);
 * 
 * //Set Motor Polarities (is it going to be inverted?)
 * leftMaster.setInverted(false);
 * leftSlave.setInverted(false);
 * rightMaster.setInverted(false);
 * rightSlave.setInverted(false);
 * 
 * //Slave motors to follow Master motors
 * leftSlave.follow(leftMaster);
 * rightSlave.follow(rightMaster);
 * 
 * /* The RestoreFactoryDefaults method can be used to reset the configuration
 * parameters
 * in the SPARK MAX to their factory default state. If no argument is passed,
 * these
 * parameters will not persist between power cycles
 *//*
    * leftMaster.restoreFactoryDefaults();
    * rightMaster.restoreFactoryDefaults();
    */