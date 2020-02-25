/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlign extends CommandBase {

  private DriveSubsystem driveSub;
  private VisionSubsystem visionSub;
  private LEDSubsystem ledSub;

  private double steer_kp = 0.04;
  private double drive_kp = 0.15;

  private double yawFeedForward = 0.51;
  private double driveFeedForward = 0.51;

  private double maxYaw = 0.75;
  private double maxThrottle = 0.75;
  
  public VisionAlign(DriveSubsystem drive, VisionSubsystem vision, LEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies
    driveSub = drive;
    visionSub = vision;
    ledSub = led;
    // visionSub.setLed(3.0);
    visionSub.setCamMode(0.0);
    addRequirements(driveSub);
    addRequirements(visionSub);
    addRequirements(ledSub);

    SmartDashboard.putNumber("Steering kP", steer_kp);
    SmartDashboard.putNumber("Drive kP", drive_kp);
    SmartDashboard.putNumber("Drive FF", driveFeedForward);
    SmartDashboard.putNumber("Yaw FF", yawFeedForward);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // steer_kp = SmartDashboard.getNumber("Steering kP", 0.0);
    // drive_kp = SmartDashboard.getNumber("Drive kP", 0.0);
    // yawFeedForward = SmartDashboard.getNumber("Yaw FF", 0.0);
    // driveFeedForward = SmartDashboard.getNumber("Drive FF", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = visionSub.getTy()*drive_kp;
    double throttle = visionSub.getTx()*steer_kp;

    

    if (throttle > -driveFeedForward){
      //DriverStation.reportError("Guy I'm Here", false);
      if(throttle < 0) {
        //DriverStation.reportError("Bruh", false);
        throttle = -driveFeedForward;
      }
    }

    if (throttle < driveFeedForward && throttle > 0) {
      throttle = driveFeedForward;
    }

    

    if (throttle > maxThrottle) {
      throttle = maxThrottle;
    }
    if (throttle < -maxThrottle) {
      throttle = -maxThrottle;
    }

    if (yaw < yawFeedForward && yaw > 0) {
      yaw = yawFeedForward;
    }
    if (yaw > -yawFeedForward && yaw < 0) {
      yaw = -yawFeedForward;
    }
    if (yaw > maxYaw) {
      yaw = maxYaw;
    }
    if (yaw < -maxYaw) {
      yaw = -maxYaw;
    }

    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Yaw", yaw);

    if (visionSub.hasTarget()) {
      if (visionSub.isXAlligned() && visionSub.isYAlligned()) {
        throttle = 0.0;
        yaw = 0.0;
        ledSub.shooterAligned();
      } else if (visionSub.isXAlligned()) {
        yaw = 0.0;
      } else if (visionSub.isYAlligned()) {
        throttle = 0.0;
      }
    } else {
      driveSub.drive(0.0, 0.0);
    }
    driveSub.drive(0.75*throttle, 0.75*yaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.drive(0.0, 0.0);
    //visionSub.setLed(1.0);
    //visionSub.setCamMode(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
