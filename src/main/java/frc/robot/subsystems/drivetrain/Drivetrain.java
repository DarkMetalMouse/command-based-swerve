package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Drivetrain.BLModule;
import static frc.robot.Constants.Drivetrain.BRModule;
import static frc.robot.Constants.Drivetrain.TLModule;
import static frc.robot.Constants.Drivetrain.TRModule;
import static frc.robot.Constants.Drivetrain.pigeonTalonId;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;

/**
 * Drivetrain
 */
public class Drivetrain extends SubsystemBase {

    private SwerveModule[] _modules;


    private TalonSRX _pigeonTalon;
    private PigeonIMU _pigeon;

    private SwerveDriveKinematics _kinematics;

    public Drivetrain() {
        _modules = new SwerveModule[] {
            new SwerveModule(TRModule),
            new SwerveModule(TLModule),
            new SwerveModule(BRModule),
            new SwerveModule(BLModule)
        };


        _kinematics = new SwerveDriveKinematics(TRModule.position, 
                                                TLModule.position,
                                                BRModule.position,
                                                BLModule.position);
        
        _pigeonTalon = new TalonSRX(pigeonTalonId);
        _pigeon = new PigeonIMU(_pigeonTalon);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
        rot *= 2;

        SwerveModuleState[] moduleStates =
            _kinematics.toSwerveModuleStates(
                fieldRelative && _pigeon.getState() == PigeonState.Ready
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getHeading()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, SwerveModuleConstants.freeSpeedMetersPerSecond);


        for (int i = 0; i < _modules.length; i++) {
            _modules[i].setDesiredState(moduleStates[i]);
        }
    }

    private double getHeading() {
        FusionStatus status = new FusionStatus();
        _pigeon.getFusedHeading(status);
        
        return status.heading;
    }

    public void resetYaw() {
        _pigeon.setFusedHeading(0);
    }

    public void printSetpoints() {
        for (int i = 0; i < _modules.length; i++) {
            SmartDashboard.putNumber("steer " + i   , _modules[i].getSteeringSetpoint());
            SmartDashboard.putNumber("drive " + i, _modules[i].getDriveSetpoint());
            SmartDashboard.putNumber("speed " + i, _modules[i].getVelocity());
        }
    }
}