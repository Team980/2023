// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

public class AutoBalanceCommand extends CommandBase {
  private Drivetrain drivetrain;
  private Shifter shifter;
  private int timer;
  double lPos;
  double rPos;


  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(Drivetrain drivetrain , Shifter shifter) {
    this.drivetrain = drivetrain;
    this.shifter = shifter;
    timer = 25;//half second determining if truly balanced
    lPos = drivetrain.getLeftDistance();
    rPos = drivetrain.getRightDistance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain , shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shifter.setLowGear();
    drivetrain.resetYaw(0);
    drivetrain.setFrontRio();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(drivetrain.getYPR()[1]) > 2){
      drivetrain.driveRobot(-1 * .25 * (drivetrain.getYPR()[1] / 15), -drivetrain.getYPR()[0]/30);
      lPos = drivetrain.getLeftDistance();
      rPos = drivetrain.getRightDistance();
      timer = 25;
    }
    else{
      drivetrain.setLeftDrive(-(drivetrain.getLeftDistance() - lPos));
      drivetrain.setRightDrive(-(drivetrain.getRightDistance() - rPos));
      timer--;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer <= 0;
  }
}
