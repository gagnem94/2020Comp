/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootDriveBack extends SequentialCommandGroup {
  /**
   * Creates a new AutoShootDriveBack.
   */
  public AutoShootDriveBack(PneumaticSubsystem pneumatics, ShooterSubsystem shooter, DriveSubsystem drive, LEDSubsystem led, IntakeSubsystem intake)  {
    //new IntakeDown(pneumatics);
    new SpinUpShooter(shooter, led, 1.0);
    new WaitCommand(500);
    new Shoot(pneumatics, intake);
    new WaitCommand(2000);
    new ShooterOff(shooter);
    //new AutoTurn(drive, 5.0);


    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
  //   super(new SpinUpShooter(shooter, 3200), 
  //         new ShooterOff(shooter)),
  //         new AutoDrive(drive, 1.0));
  // 
  }
}
