package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controller.GameController;

import java.util.ArrayList;
import java.util.Arrays;

public class Drive extends SubsystemBase {

    public static enum DriveControlMode {
        JOYSTICK, PATH_FOLLOWING
    }

    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

    private TalonFX topLeftDrive;
    private TalonFX topRightDrive;
    private TalonFX bottomLeftDrive;
    private TalonFX bottomRightDrive;

    private CANCoder topLeftTurn;
    private CANCoder topRightTurn;
    private CANCoder bottomLeftTurn;
    private CANCoder bottomRightTurn;

    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;

    // Speed Control
    private static final double STEER_NON_LINEARITY = 0.5;
    private static final double MOVE_NON_LINEARITY = 1.0;

    private static final int MOVE_NON_LINEAR = 0;
    private static final int STEER_NON_LINEAR = -3;

    private static final double MOVE_SCALE = 1.0;
    private static final double STEER_SCALE = 0.75;

    private static final double MOVE_TRIM = 0.0;
    private static final double STEER_TRIM = 0.0;

    private static final double STICK_DEADBAND = 0.02;

    public static final double OPEN_LOOP_PERCENT_OUTPUT_LO = 0.6;
    public static final double OPEN_LOOP_PERCENT_OUTPUT_HI = 1.0;

    public static final double OPEN_LOOP_VOLTAGE_RAMP_HI = 0.3;
    public static final double OPEN_LOOP_VOLTAGE_RAMP_LO = 0.3;

    private double m_moveInput = 0.0;
    private double m_steerInput = 0.0;

    private double m_moveOutput = 0.0;
    private double m_steerOutput = 0.0;

    private boolean isHighGear = false;

    private PigeonIMU m_gyro;
    private double[] yprPigeon = new double[3];
    private short[] xyzPigeon = new short[3];
    private boolean isCalibrating = false;
    private double gyroYawOffsetAngleDeg = Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES;

    // Differential Drive
    private DifferentialDrive m_drive;
    private GameController m_driverController;
    private SwerveDriveOdometry m_odometry;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            Constants.FRONT_LEFT_WHEEL_LOCATION, Constants.FRONT_RIGHT_WHEEL_LOCATION, Constants.BACK_LEFT_WHEEL_LOCATION, Constants.BACK_RIGHT_WHEEL_LOCATION);

    private final static Drive INSTANCE = new Drive();

    /** Creates a new ExampleSubsystem. */
    public Drive() {
        frontLeft = new SwerveModule(topLeftDrive, topLeftTurn, "Top_Left_Module");
        frontRight = new SwerveModule(topRightDrive, topRightTurn, "Top_Right_Module");
        backLeft = new SwerveModule(bottomLeftDrive, bottomLeftTurn, "Bottom_Left_Module");
        backRight = new SwerveModule(bottomRightDrive, bottomRightTurn, "bottom_Right_Module");

        m_gyro = new PigeonIMU(Constants.GYRO_CAN_ID);
        m_odometry = new SwerveDriveOdometry(m_kinematics,Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));
    }

    public static Drive getInstance() {
        return INSTANCE;
    }

    public void setDriverController(GameController driverController) {
        m_driverController = driverController;
    }

    public synchronized void driveWithJoystick() {
        if (m_drive == null) {
            return;
        }

        boolean isHighGearPrevious = isHighGear;
        //       isHighGear = m_driverController.getRightBumper().get();
        isHighGear = m_driverController.getRightTrigger().get();
        if (isHighGearPrevious != isHighGear) {
            updateOpenLoopVoltageRamp();
        }

        double shiftScaleFactor = OPEN_LOOP_PERCENT_OUTPUT_LO;
        if (isHighGear == true) {
            shiftScaleFactor = OPEN_LOOP_PERCENT_OUTPUT_HI;
        }

        m_moveInput = -m_driverController.getLeftYAxis();
        m_steerInput = m_driverController.getRightXAxis();

        m_moveOutput = adjustForSensitivity(MOVE_SCALE * shiftScaleFactor, MOVE_TRIM, m_moveInput, MOVE_NON_LINEAR, MOVE_NON_LINEARITY);
        m_steerOutput = adjustForSensitivity(STEER_SCALE, STEER_TRIM, m_steerInput, STEER_NON_LINEAR, STEER_NON_LINEARITY);

        m_drive.arcadeDrive(m_moveOutput, m_steerOutput);
    }

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Convert to module states
    private SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeftModuleState = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRightModuleState = moduleStates[0];

    // Back left module state
    SwerveModuleState bottomLeftModuleState = moduleStates[0];

    // Back right module state
    SwerveModuleState backRightModuleState = moduleStates[0];


    private ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0, 0.0, Math.PI / 0.0, Rotation2d.fromDegrees(0.0));


    private SwerveModuleState frontLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    private SwerveModuleState frontRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    private SwerveModuleState backLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    private SwerveModuleState backRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    // Convert to chassis speeds
    private ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);

    // Getting individual speeds
    private double forward = chassisSpeeds.vxMetersPerSecond;
    private double sideways = chassisSpeeds.vyMetersPerSecond;
    private double angular = chassisSpeeds.omegaRadiansPerSecond;

    // Gyro Set Up
    public void calibrateGyro() {
        m_gyro.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature);
    }

    public void endGyroCalibration() {
        if (isCalibrating == true) {
            isCalibrating = false;
        }
    }

    public void setGyroYawOffset(double offsetDeg) {
        gyroYawOffsetAngleDeg = offsetDeg;
    }

    public synchronized double getGyroYawAngleDeg() {
        m_gyro.getYawPitchRoll(yprPigeon);
        return yprPigeon[0] + gyroYawOffsetAngleDeg;
    }

    public synchronized double getGyroFusedHeadingAngleDeg() {
        return m_gyro.getFusedHeading() + gyroYawOffsetAngleDeg;
    }

    public synchronized double getGyroPitchAngle() {
        m_gyro.getYawPitchRoll(yprPigeon);
        return yprPigeon[2];
    }

    public synchronized void resetGyroYawAngle() {
        m_gyro.setYaw(0);
        m_gyro.setFusedHeading(0);
    }

    public synchronized void resetGyroYawAngle(double homeAngle) {
        resetGyroYawAngle();
        setGyroYawOffset(homeAngle);
    }

    public double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
                                       double wheelNonLinearity) {
        if (inDeadZone(steer))
            return 0;

        steer += trim;
        steer *= scale;
        steer = limitValue(steer);

        int iterations = Math.abs(nonLinearFactor);
        for (int i = 0; i < iterations; i++) {
            if (nonLinearFactor > 0) {
                steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
            } else {
                steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
            }
        }
        return steer;
    }

    private boolean inDeadZone(double input) {
        boolean inDeadZone;
        if (Math.abs(input) < STICK_DEADBAND) {
            inDeadZone = true;
        } else {
            inDeadZone = false;
        }
        return inDeadZone;
    }

    private double limitValue(double value) {
        if (value > 1.0) {
            value = 1.0;
        } else if (value < -1.0) {
            value = -1.0;
        }
        return value;
    }

    private double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
        return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
    }

    private double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
        return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Get my gyro angle. We are negating the value because gyros return positive
        // values as the robot turns clockwise. This is not standard convention that is
        // used by the WPILib classes.
        var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getFusedHeading());

        // Update the pose
        Pose2d m_pose = m_odometry.update(gyroAngle, frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backLeft.getState());
    }
}
