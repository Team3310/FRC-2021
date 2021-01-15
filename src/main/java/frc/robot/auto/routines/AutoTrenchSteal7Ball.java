package frc.robot.auto.routines;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.*;
import frc.robot.commands.IntakeExtendAllAuto;
import frc.robot.commands.ShooterReset;
import frc.robot.subsystems.*;

public class AutoTrenchSteal7Ball extends SequentialCommandGroup {
    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Magazine mMagazine = Magazine.getInstance();
    Turret mTurret = Turret.getInstance();
    Intake mIntake = Intake.getInstance();
    Limelight mLimelight = Limelight.getInstance();

    public AutoTrenchSteal7Ball() {
        addCommands(
                new ResetOdometryAuto(new Pose2d(Units.inchesToMeters(147), Units.inchesToMeters(-245), new Rotation2d(0))),
                new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                new RamseteCommand(
                        mTrajectories.getStealStartToStealBallV2(),
                        mDrive::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        mDrive::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        // RamseteCommand passes volts to the callback
                        mDrive::tankDriveVolts,
                        mDrive),
                new StopTrajectory(),
                new ParallelCommandGroup(
                        new RamseteCommand(
                                mTrajectories.getStealBallToCenterShotV2(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),
                       new ShooterAutoLegShotTrack(mShooter, mTurret)
                ),
                new StopTrajectory(),
                new ShooterAutoLegShoot(mShooter,mMagazine,mTurret,
                        Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
                new IntakeExtendAllAuto(mIntake, mTurret, mMagazine),
                new RamseteCommand(
                            mTrajectories.getStealFarSideRendezvousPoint2Balls(),
                            mDrive::getPose,
                            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                            new SimpleMotorFeedforward(Constants.ksVolts,
                                    Constants.kvVoltSecondsPerMeter,
                                    Constants.kaVoltSecondsSquaredPerMeter),
                            Constants.kDriveKinematics,
                            mDrive::getWheelSpeeds,
                            new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                            new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                            // RamseteCommand passes volts to the callback
                            mDrive::tankDriveVolts,
                            mDrive),
                new StopTrajectory(),
                new ParallelCommandGroup(
                        new RamseteCommand(
                                mTrajectories.getStealFarSideRendezvousPoint2BallsReverse(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),
                        new ShooterAutoLegShotTrack(mShooter, mTurret)
                ),
                new StopTrajectory(),
                new ShooterAutoLegShoot(mShooter,mMagazine,mTurret,
                        Constants.MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES_5_BALL),
                new ShooterReset(mShooter, mMagazine, mTurret, Limelight.getInstance())
        );
    }
}