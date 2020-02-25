/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax leftShooter = new CANSparkMax(Constants.LEFT_SHOOTER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightShooter = new CANSparkMax(Constants.RIGHT_SHOOTER_CAN_ID, MotorType.kBrushless);

  private CANEncoder leftEnc = leftShooter.getEncoder();
  private CANEncoder rightEnc = rightShooter.getEncoder();

  private double leftkP = 0.0002;
  private double rightkP = 0.0002;

  private PIDController leftPID = new PIDController(0.00015, 0.0002, 0.000025);
  private PIDController rightPID = new PIDController(0.00015, 0.0002, 0.000025);

    // 3000 rpm at initiation line

  public ShooterSubsystem() {
    rightShooter.restoreFactoryDefaults();
    leftShooter.restoreFactoryDefaults();
    leftShooter.setInverted(true);

    spinManual(0.05);
    SmartDashboard.putNumber("Left Shooter kP", leftkP);
    SmartDashboard.putNumber("Right Shooter kP",  rightkP);
    SmartDashboard.putNumber("Target RPM", 3300);
    

    leftPID.setTolerance(10, 30);
    rightPID.setTolerance(10, 30);

  }

  public double getLeftRPM() {
    return leftEnc.getVelocity();
  }

  public double getRightRPM() {
    return rightEnc.getVelocity();
  }

  public void spinManual(double speed) {
    leftShooter.set(speed);
    rightShooter.set(speed);
  }

  public void spinAuto(double targetRPM) {
    double left = leftPID.calculate(getLeftRPM(), targetRPM);
    double right = rightPID.calculate(getRightRPM(), targetRPM);

    if (left < 0) {
      left = 0.0;
    }
    leftShooter.set(left);
    rightShooter.set(right);
    if (leftPID.atSetpoint() && rightPID.atSetpoint()) {
      SmartDashboard.putBoolean("Shooter Ready:", true);
    } else {
      SmartDashboard.putBoolean("Shooter Ready:", false);
    }
  }


  public void shooterOff() {
    leftShooter.set(0.0);
    rightShooter.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Shooter RPM:", getLeftRPM());
    SmartDashboard.putNumber("Right Shooter RPM:", getRightRPM());
  }
}
