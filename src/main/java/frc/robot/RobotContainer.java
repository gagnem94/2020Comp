/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoShootDriveBack;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootOff;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.VisionAlign;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem drive = new DriveSubsystem();
  private PneumaticSubsystem pneumatics = new PneumaticSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private VisionSubsystem vision = new VisionSubsystem();
  public static LEDSubsystem leds = new LEDSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();


  private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private double shooterRPM = SmartDashboard.getNumber("Target RPM", 0.0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drive.setDefaultCommand(new JoystickDrive(drive, leds, driverController));
    shooter.setDefaultCommand(new ShooterDefaultCommand(shooter));

    autoChooser.addOption("Basic Auto", new AutoShootDriveBack(pneumatics, shooter, drive, leds, intake));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, XboxController.Button.kBumperLeft.value).whenPressed(() -> drive.toggleReversed());
    new JoystickButton(driverController, XboxController.Button.kB.value).whileHeld(new VisionAlign(drive, vision, leds, 1.0));
    new JoystickButton(driverController, XboxController.Button.kY.value).whileHeld(new VisionAlign(drive, vision, leds, 0.0));
    new JoystickButton(driverController, XboxController.Button.kA.value).whileHeld(new VisionAlign(drive, vision, leds, 2.0));
    new JoystickButton(operatorController, XboxController.Button.kA.value).whileHeld(() -> intake.intakeAndElevate());
    new JoystickButton(operatorController, XboxController.Button.kA.value).whenReleased(() -> intake.intakeAndElevateOff());
    new JoystickButton(operatorController, XboxController.Button.kB.value).whenPressed(() -> pneumatics.intakeDown());
    new JoystickButton(operatorController, XboxController.Button.kX.value).whenPressed(() -> pneumatics.intakeUp());
    new JoystickButton(operatorController, XboxController.Button.kBumperRight.value).whileHeld(new SpinUpShooter(shooter, leds, shooterRPM));
    new JoystickButton(operatorController, XboxController.Button.kBumperRight.value).whenReleased(() -> shooter.shooterOff());
    new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value).whileHeld(new Shoot(pneumatics, intake));
    new JoystickButton(operatorController, XboxController.Button.kBumperLeft.value).whenReleased(new ShootOff(pneumatics, intake)); 
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
