// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */


public class Drivetrain extends SubsystemBase{

    /* First, we declare our four motors for the drivetrain
    We are going to use NEO motors and have a leader (master) and follower (slave) on each side
    */
        // private CANSparkMax leftMaster;
        // private CANSparkMax leftSlave;
        // private CANSparkMax rightMaster;
        // private CANSparkMax rightSlave;

        private CANSparkMax leftMaster = new CANSparkMax(RobotMap.kLeftMaster, MotorType.kBrushless); //CAN 3
    
        //declare PID Controllers
        private SparkMaxPIDController _leftMasterCanController;
        // private SparkMaxPIDController _rightMasterCanController;
    
    
        //declarations for PID
        private  double _kPDrive =  5e-20;
        private  double _kIDrive = 1e-6 + 1e-3;
        private  double _kIZoneDrive = 0;
        private  double _kDDrive = 0;
        private  double _kFFDrive = 0.000156;
        // private  double _kForwardRR = 0;
        // private  double _kStrafeRR = 0;
        // private  double _kIsMoreStrafe = .5;
    
        public DifferentialDrive drive;
    
    /* DifferentialDrive is when the robot is controlled by wheels on each side. 
    Steering is controlled by motor speed and direction */
    
    //initalize all neccesary Drive objects
    public void init() {
    
        _leftMasterCanController = leftMaster.getPIDController();
        // _rightMasterCanController = rightMaster.getPIDController();
    
        _leftMasterCanController.setP(_kPDrive);
        _leftMasterCanController.setI(_kIDrive);
        _leftMasterCanController.setIZone(_kIZoneDrive);
        _leftMasterCanController.setD(_kDDrive);
        _leftMasterCanController.setFF(_kFFDrive);
        _leftMasterCanController.setOutputRange(-1, 1);
        _leftMasterCanController.setSmartMotionMaxVelocity(10000, 0);
        _leftMasterCanController.setSmartMotionMaxAccel(2000, 0);
    
        // _rightMasterCanController.setP(_kPDrive);
        // _rightMasterCanController.setI(_kIDrive);
        // _rightMasterCanController.setIZone(_kIZoneDrive);
        // _rightMasterCanController.setD(_kDDrive);
        // _rightMasterCanController.setFF(_kFFDrive);
        // _rightMasterCanController.setOutputRange(-1, 1);
        // _rightMasterCanController.setSmartMotionMaxVelocity(2000, 0);
        // _rightMasterCanController.setSmartMotionMaxAccel(1500, 0);
    }
    
    //Drive to position using smart motion profiling
    public void setSetPoint(double targetLeft){

        // double setPoint = SmartDashboard.getNumber("Set Position", 0);
        /**
         * As with other PID modes, Smart Motion is set by calling the
         * setReference method on an existing pid object and setting
         * the control type to kSmartMotion
         */
        _leftMasterCanController.setReference(targetLeft, CANSparkMax.ControlType.kSmartMotion);
        // _leftMasterCanController.setReference(targetFrontLeft, ControlType.kSmartMotion);
        // _rightMasterCanController.setReference(targetFrontRight, ControlType.kSmartMotion);
    
      }
    
        //public Drivetrain(){
    
    //THE FOLLOWING NEEDS TO BE DECLARED IN CONSTANTS (PID DRIVE RELATED)
    // CANSparkMax(device number, type of motor (NEO brushless in this case))
    /*
            leftMaster = new CANSparkMax(RobotMap.kLeftMaster, MotorType.kBrushless); //CAN 3
            leftSlave = new CANSparkMax(RobotMap.kLeftSlave, MotorType.kBrushless); //CAN 1
            rightMaster = new CANSparkMax(RobotMap.kRightMaster, MotorType.kBrushless); //CAN 2
            rightSlave = new CANSparkMax(RobotMap.kRightSlave, MotorType.kBrushless); //CAN 4
            drive = new DifferentialDrive(leftMaster, rightMaster);
     
            //Set Motor Polarities (is it going to be inverted?)
            leftMaster.setInverted(false);
            leftSlave.setInverted(false);
            rightMaster.setInverted(false);
            rightSlave.setInverted(false);
    
            //Slave motors to follow Master motors
            leftSlave.follow(leftMaster);
            rightSlave.follow(rightMaster);
    
            /* The RestoreFactoryDefaults method can be used to reset the configuration parameters
            in the SPARK MAX to their factory default state. If no argument is passed, these
            parameters will not persist between power cycles
            *//*
            leftMaster.restoreFactoryDefaults();
            rightMaster.restoreFactoryDefaults();
    */
        //} 
    
        
    
        
        
        
    
    }