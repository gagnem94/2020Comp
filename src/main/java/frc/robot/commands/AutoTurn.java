/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurn extends CommandBase {

  private DriveSubsystem driveSub;
  private double angleTo;
  private double rotateKp = 0.1;
  private double headingError;

  /**
   * Creates a new AutoTurn.
   */
  public AutoTurn(DriveSubsystem drive, double angle) {
    driveSub  = drive;
    angleTo = angle;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.drive(0.0, 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    headingError = angleTo - driveSub.getAngle();
    driveSub.drive(0.0, headingError*rotateKp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(headingError) < 0.5) {
      return true;
    } else {
      return false;
    }
  }
}
