/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax leftMaster = new CANSparkMax(Constants.LEFT_MASTER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax leftSlave = new CANSparkMax(Constants.LEFT_SLAVE_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightMaster = new CANSparkMax(Constants.RIGHT_MASTER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightSlave = new CANSparkMax(Constants.RIGHT_SLAVE_CAN_ID, MotorType.kBrushless);

  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  private CANEncoder leftEnc = leftMaster.getEncoder();
  private CANEncoder rightEnc = rightMaster.getEncoder();

  private final DifferentialDriveOdometry odometry;

  private boolean isReversed = false;
  private double maxOutput = 1.0;

  public DriveSubsystem() {
    leftMaster.restoreFactoryDefaults();
    leftSlave.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    rightSlave.restoreFactoryDefaults();

    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftEnc.setPositionConversionFactor(0.4); 
    rightEnc.setPositionConversionFactor(0.4);

    resetEncoders();
    resetGyro();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  public double getTurnRate() {
    return navx.getRate();
  }

  public double getAvgDistance() {
    return (leftEnc.getPosition() + rightEnc.getPosition()) / 2;
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public void drive(double throttle, double yaw) {
    if (isReversed()) {
      leftMaster.set(maxOutput * (throttle+yaw));
      rightMaster.set(maxOutput * (throttle-yaw));

    } else {
      leftMaster.set(maxOutput * (-throttle+yaw)); //-throttle+yaw
      rightMaster.set(maxOutput * (-throttle-yaw)); //-throttle-yaw
    }
    
  }

  public void driveVoltage(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
  }

  public void resetEncoders() {
    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);
  }

  public void resetGyro() {
    navx.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void stop() {
    leftMaster.set(0.0);
    rightMaster.set(0.0);
  }

  public boolean isReversed() {
    return isReversed;
  }

  public void toggleReversed() {
    isReversed = !isReversed;
  }

  public void setMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEnc.getPosition(), rightEnc.getPosition());
    SmartDashboard.putBoolean("Drive Reversed", isReversed());
  }

}
