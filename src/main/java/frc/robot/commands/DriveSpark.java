// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PID;
import frc.robot.subsystems.Drivetrain;

public class DriveSpark extends CommandBase {
  /** Creates a new DriveSpark. */
  private Timer PIDThresholdCheck;
  private double leftMasterSetPoint;
  private double setTarget;
  private double tolerance;
  private boolean wasOnTarget;

  private final Drivetrain drivetrain;

  //add interupt requirements and assign refrence subsystems/parameters
  public DriveSpark(double target,Drivetrain subDrivetrain,double subTolerance) {

    setTarget = target;
    drivetrain = subDrivetrain;
    tolerance = subTolerance;
    wasOnTarget = false;
    PIDThresholdCheck = new Timer();

    addRequirements(subDrivetrain);
  }

  private boolean isOnTarget(){
    return tolerance >= Math.abs(leftMasterSetPoint - drivetrain.getLeftMasterPosition());
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftMasterSetPoint = drivetrain.getLeftMasterPosition() + setTarget;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //enable motors to run to PID position
    drivetrain.PIDSetPointAbsolute(leftMasterSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when motor stays within threshold for longer than a second
  @Override
  public boolean isFinished() {
    if(!isOnTarget())
    {
      if(wasOnTarget)
      {
        PIDThresholdCheck.stop();
        PIDThresholdCheck.reset();
        wasOnTarget = false;
      }
    } 

    else if(!wasOnTarget)
    {
      PIDThresholdCheck.start();
      wasOnTarget = true;
    }

    else if(PIDThresholdCheck.hasElapsed(1))
    {
      return true;
    }
    return false;
  }
}
