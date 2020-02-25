/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticSubsystem extends SubsystemBase {

  private Compressor compressor = new Compressor();
  private DoubleSolenoid intakeCyls = new DoubleSolenoid(Constants.INTAKE_CYLINDER_FORWARD, Constants.INTAKE_CYLINDER_REVERSE);
  //private DoubleSolenoid shooterCover1 = new DoubleSolenoid(Constants.SHOOTER_COVER_FORWARD_1, Constants.SHOOTER_COVER_REVERSE_1);
  //private DoubleSolenoid shooterCover2 = new DoubleSolenoid(Constants.SHOOTER_COVER_FORWARD_2, Constants.SHOOTER_COVER_REVERSE_2);



  private String intakeState = "Up";
  private String shooterCoverState = "Closed";


  public PneumaticSubsystem() {

  }

  public void intakeUp() {
    if (intakeState.equals("Down")) {
      intakeCyls.set(Value.kReverse);
      intakeState = "Up";
    }
  }

  public void intakeDown() {
    if (intakeState.equals("Up")) {
      intakeCyls.set(Value.kForward);
      intakeState = "Down";
    }
  }

  public void closeShooter() {
    if (shooterCoverState.equals("Open")) {
      //shooterCover1.set(Value.kForward);
      //shooterCover2.set(Value.kForward);
    }
  }

  public void openShooter() {
    if (shooterCoverState.equals("Closed")) {
      //  shooterCover1.set(Value.kReverse);
      //shooterCover2.set(Value.kReverse);
    }
  }

  public void startCompressor() {
    compressor.start();
  }

  public void stopCompressor() {
    compressor.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
