/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class ShootOff extends CommandBase {
  private final PneumaticSubsystem pneumaticSub;
  private final IntakeSubsystem intakeSub;

  public ShootOff(PneumaticSubsystem pneumatics, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumaticSub = pneumatics;
    intakeSub = intake;
    addRequirements(pneumaticSub);
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    pneumaticSub.tensionerDown();
    intakeSub.elevatorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
