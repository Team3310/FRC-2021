// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // USB Port IDs
    public static final int DRIVER_JOYSTICK_1_USB_ID = 0;
    public static final int OPERATOR_JOYSTICK_1_USB_ID = 1;

    public static final int TOP_LEFT_DRIVE_CAN_ID = 0;
    public static final int TOP_RIGHT_DRIVE_CAN_ID = 0;
    public static final int BOTTOM_LEFT_DRIVE_CAN_ID = 0;
    public static final int BOTTOM_RIGHT_DRIVE_CAN_ID = 0;

    public static final int TOP_LEFT_STEER_CAN_ID = 0;
    public static final int TOP_RIGHT_STEER_CAN_ID = 0;
    public static final int BOTTOM_LEFT_STEER_CAN_ID = 0;
    public static final int BOTTOM_RIGHT_STEER_CAN_ID = 0;

    // Gyro
    public static final int GYRO_CAN_ID = 0;

    public static final Translation2d FRONT_LEFT_WHEEL_LOCATION = new Translation2d(0.186055,0.186055);
    public static final Translation2d FRONT_RIGHT_WHEEL_LOCATION = new Translation2d(0.186055, -0.186055);
    public static final Translation2d BACK_LEFT_WHEEL_LOCATION = new Translation2d(-0.186055, 0.186055);
    public static final Translation2d BACK_RIGHT_WHEEL_LOCATION = new Translation2d(-0.186055, -0.186055);

    // Drive
    public static final double DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES = 0.0;
    public static final double kMaxSpeedMetersPerSecond = 5;



}
