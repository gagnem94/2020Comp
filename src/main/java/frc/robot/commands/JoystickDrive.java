/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class JoystickDrive extends CommandBase {

  private DriveSubsystem driveSub;
  private XboxController driveController;
  private LEDSubsystem ledSub;

  public JoystickDrive(DriveSubsystem drive, LEDSubsystem led, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSub = drive;
    ledSub = led;
    driveController = controller;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSub.teleop();
    double throttle = -driveController.getRawAxis(1);
    double yaw = driveController.getRawAxis(4);

    if (Math.abs(throttle) < Constants.DRIVER_CONTROLLER_DEADBAND) {
      throttle = 0.0;
    }

    if (Math.abs(yaw) < Constants.DRIVER_CONTROLLER_DEADBAND) {
      yaw = 0.0;
    }

    if (throttle > 1) {
      throttle = 1;
    }

    if (throttle < -1) {
      throttle = -1;
    }

    if (yaw > 1) {
      yaw = 1;
    }

    if (yaw < -1) {
      yaw = -1;
    }

    driveSub.drive(throttle, 0.6*yaw);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
