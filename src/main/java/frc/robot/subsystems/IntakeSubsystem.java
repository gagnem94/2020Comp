/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_VictorSPX sideRollers = new WPI_VictorSPX(Constants.SIDE_ROLLER_INTAKE_CAN_ID);
  private WPI_VictorSPX elevatorMotor = new WPI_VictorSPX(Constants.ELEVATOR_MOTOR_CAN_ID);

  public IntakeSubsystem() {
  }

  public void intakeOn() {
    sideRollers.set(1.0);
  }

  public void intakeOff() {
    sideRollers.set(0.0);
  }

  public void elevatorOn() {
    elevatorMotor.set(1.0);
  }

  public void elevatorOff() {
    elevatorMotor.set(0.0);
  }

  public void intakeAndElevate() {
    sideRollers.set(1.0);
    elevatorMotor.set(0.5);
  }

  public void intakeAndElevateOff() {
    sideRollers.set(0.0);
    elevatorMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
