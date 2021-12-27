// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.util.PIDFGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static class SwerveModuleConstants {
            public static final double freeSpeedMetersPerSecond = 3.6576;
            public static final double driveRatio = 1.0 / 8.14; 
            public static final double steeringRatio = 1.0 / 12.8; 
            public static final double wheelRadiusMeters = 0.0508; // 2 inches (in meters)
            public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI; 
            public static final double driveDPRMeters = wheelCircumferenceMeters * driveRatio;
            
            public final Translation2d position;
            public final int idDrive;
            public final PIDFGains driveGains;
            public final int idSteering;
            public final PIDFGains steeringGains;

            public SwerveModuleConstants(Translation2d position, int idDrive, int idSteering) {
                this(position, 
                     idDrive, 
                     idSteering, 
                     new PIDFGains(0.0002, 0, 0.01, 1.0/6/902.0, 50, 0), 
                     new PIDFGains(0.35, 0, 0, 0, 1, 0));
            }
            public SwerveModuleConstants(Translation2d position, 
                                         int idDrive, 
                                         int idSteering, 
                                         PIDFGains driveGains, 
                                         PIDFGains steeringGains) {
                this.position = position;
                this.idDrive = idDrive;
                this.driveGains = driveGains;
                this.idSteering = idSteering;
                this.steeringGains = steeringGains;
            }
        }
        
        public static final double trackWidth = 0.43;
        // Distance between centers of right and left wheels on robot
        public static final double wheelBase = trackWidth;
        // Distance between front and back wheels on robot

        public static final SwerveModuleConstants TRModule = new SwerveModuleConstants(new Translation2d(wheelBase / 2, trackWidth / 2), 1, 2);
        public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(new Translation2d(wheelBase / 2, -trackWidth / 2), 7, 8);
        public static final SwerveModuleConstants TLModule = new SwerveModuleConstants(new Translation2d(-wheelBase / 2, trackWidth / 2), 3, 4);
        public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(new Translation2d(-wheelBase / 2, -trackWidth / 2), 5, 6);
    }
}
