/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private double tx, ty, tv;

  public VisionSubsystem() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);
    setLed(3.0);
    setCamMode(0.0);
  }

  public void setLed(double state) {
    // 1 = force off, 2 = force blink, 3 = force on
    limelight.getEntry("ledMode").setNumber(state);
  }

  public void setCamMode(double state) {
    // 0 = vision processor, 1 = Driver Camera
    limelight.getEntry("camMode").setNumber(state);
  }

  public void setPipeline(double pipeline) {
    // 0 = initiation line, 1 = trench, 2 = salsa shot
    limelight.getEntry("pipeline").setNumber(pipeline);
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public boolean hasTarget() {
    return (tv==1.0);
  }

  public boolean isXAlligned() {
    return (Math.abs(tx) <= Constants.LIMELIGHT_YAW_THRESHOLD);
  }

  public boolean isYAlligned() {
    return (Math.abs(ty) <= Constants.LIMELIGHT_YAW_THRESHOLD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);
  }
}
