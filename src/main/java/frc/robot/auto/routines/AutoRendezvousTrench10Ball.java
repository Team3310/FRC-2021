/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.routines;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.commands.IntakeExtendAll;
import frc.robot.subsystems.*;

public class AutoRendezvousTrench10Ball extends ParallelCommandGroup {
        TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
        Drive mDrive = Drive.getInstance();
        Shooter mShooter = Shooter.getInstance();
        Magazine mMagazine = Magazine.getInstance();
        Turret mTurret = Turret.getInstance();
        Intake mIntake = Intake.getInstance();
        Limelight mLimelight = Limelight.getInstance();        /**
         * Add your docs here.
         */
        public AutoRendezvousTrench10Ball() {
                addCommands(new SequentialCommandGroup(
                        new ResetOdometryAuto(),
                        new ParallelCommandGroup(
                                new IntakeExtendAll(mIntake,mMagazine),
                                new RamseteCommand(
                                        mTrajectories.getCenterStartToRendezvous2ball(),
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
                                        mDrive)),
                        new StopTrajectory(),
                        new RamseteCommand(
                                mTrajectories.getRendezvous2BallToStartOfTrench(),
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
                        //Intake in Parallel
                        new RamseteCommand(
                                mTrajectories.getStartOfTrenchToEndOfTrench(),
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
                        new RamseteCommand(
                                mTrajectories.getEndOfTrenchToStartOfTrench(),
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
                        new StopTrajectory()
                        //Shoot
                ));
        }
}
