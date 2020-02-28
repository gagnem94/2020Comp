/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drive constants
    public static final int LEFT_MASTER_CAN_ID = 1;
    public static final int LEFT_SLAVE_CAN_ID = 2;
    public static final int RIGHT_MASTER_CAN_ID = 3;
    public static final int RIGHT_SLAVE_CAN_ID = 4;

    // Intake/Elevator constants
    public static final int SIDE_ROLLER_INTAKE_CAN_ID = 5;
    public static final int ELEVATOR_MOTOR_CAN_ID = 6;
    public static final int DINGUS_MOTOR_CAN_ID = 10;

    // Pneumatic constants
    public static final int INTAKE_CYLINDER_FORWARD = 0;
    public static final int INTAKE_CYLINDER_REVERSE = 1;
    public static final int TENSIONER_CYLS_FORWARD = 2;
    public static final int TENIONER_CYLS_REVERSE= 3;

    // Shooter constants
    public static final int LEFT_SHOOTER_CAN_ID = 7;
    public static final int RIGHT_SHOOTER_CAN_ID = 8;

    // Controller constants
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVER_CONTROLLER_DEADBAND = 0.2;
    public static final double OPERATOR_CONTROLLER_DEADBAND = 0.2;

    // LED constants
    public static final int LED_PWM_PORT = 0;
    public static final int LED_COUNT = 80;

    // Limelight constants
    public static final double LIMELIGHT_YAW_THRESHOLD = 1.0;
    public static final double LIMELIGHT_THROTTLE_THRESHOLD = 1.0;

    // Path Planning constants
    public static final double ksVolts = 0.0;
    public static final double kvVoltSecondsPerMeter = 0.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0;

    public static final double trackWidth = 0.0; // take distance between seets of wheels and add a few inches, keep testing to see what the actual track width is
    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(trackWidth);
    public static final double maxSpeedMetersPerSecond = 0.0;
    public static final double maxAccelMetersPerSecondSquare = 0.0;

    public static final double ramseteB = 0.7;
    public static final double ramseteZeta = 2.0;
}
