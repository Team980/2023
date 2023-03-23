// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveCommand extends CommandBase {
  private Drivetrain drivetrain;
  private double distance;//feet
  private double speed;//from -1 to 1
  private int cyclesToEStop;
  private int timer;

  /** Creates a new AutoDriveCommand. */
  public AutoDriveCommand(Drivetrain drivetrain, double distance, double speed, double eStopTime) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.speed = speed;
    cyclesToEStop = (int)(eStopTime / .02); //convert time in seconds to 20ms cycles
    timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public AutoDriveCommand(Drivetrain drivetrain, double distance, double speed) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.speed = speed;
    cyclesToEStop = 0; //convert time in seconds to 20ms cycles
    timer = -1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetYaw(0);
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveRobot(speed, -drivetrain.getYPR()[0]/30);
    if(timer >= 0){
      timer++;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(drivetrain.getLeftDistance() <= distance || drivetrain.getRightDistance() <= distance || timer >= cyclesToEStop) {
      return true;
    }
    return false;
  }

  
}
