package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.util.PIDFGains;

public class SwerveModule {

    private static final int DEGREES_IN_ROATION = 360;
    private static final int SECONDS_IN_MINUTE = 60;
    private final CANSparkMax m_driveSparkMax;
    private final CANPIDController m_drivePID;
    private final CANEncoder m_driveEncoder;
    private final CANSparkMax m_steeringSparkMax;
    private final CANPIDController m_steeringPID;
    private final CANEncoder m_steeringEncoder;

    private double m_steerSetpoint;
    private double m_driveSetpoint;

    /**
     * Constructs a SwerveModule
     * @param constants the constants for the module
     */
    public SwerveModule(SwerveModuleConstants constants) {
        this.m_driveSparkMax = new CANSparkMax(constants.idDrive, MotorType.kBrushless);
        this.m_steeringSparkMax = new CANSparkMax(constants.idSteering, MotorType.kBrushless);
        
        this.m_drivePID = this.m_driveSparkMax.getPIDController();
        this.m_driveEncoder = this.m_driveSparkMax.getEncoder();
        this.m_driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.driveDPRMeters / SECONDS_IN_MINUTE); // convert RPM to m/s

        this.m_steeringPID = this.m_steeringSparkMax.getPIDController();
        this.m_steeringEncoder = this.m_steeringSparkMax.getEncoder();
        this.m_steeringEncoder.setPositionConversionFactor(SwerveModuleConstants.steeringRatio * DEGREES_IN_ROATION); // convert motor rotations to wheel angle

        setPIDGains(this.m_drivePID, constants.driveGains);
        setPIDGains(this.m_steeringPID, constants.steeringGains);
                
        this.m_steerSetpoint = 0;
        this.m_driveSetpoint = 0;
    }

    private static void setPIDGains(CANPIDController pidController, PIDFGains gains) {
        pidController.setI(gains.getI());
        pidController.setP(gains.getP());
        pidController.setD(gains.getD());
        pidController.setFF(gains.getF());
        pidController.setIZone(gains.getIZone());
        pidController.setOutputRange(-1.0,1.0);
    }

    /**
     * Stop the drive motor
     */
    public void stopDrive() {
        this.m_driveSparkMax.set(0);
    } 

    /**
     * start a PID loop to the desired wheel angle and velocity
     * @param desiredState the desired wheel angle and velocity
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(this.getWheelAngle()));

        this.m_steerSetpoint = this.m_steeringEncoder.getPosition() + state.angle.getDegrees();
        this.m_driveSetpoint = state.speedMetersPerSecond;

        if (state.speedMetersPerSecond != 0) { 
            this.m_steeringPID.setReference(this.m_steerSetpoint, ControlType.kPosition);
        }
        // _driveSparkMax.set(1 * Math.signum(state.speedMetersPerSecond));
        if (state.speedMetersPerSecond == 0){
            this.stopDrive();
        }else{
            this.m_drivePID.setReference(this.m_driveSetpoint, ControlType.kVelocity);
        }
    }

    /**
     * Get the current angle of the wheel (in degrees)
     * @return The current angle of the wheel
     */
    public double getWheelAngle() {
        double pos = this.m_steeringEncoder.getPosition() / DEGREES_IN_ROATION;
        pos = pos - Math.floor(pos);
        return pos * DEGREES_IN_ROATION;
    }

    /**
     * Get the current velocity of the wheel (in meters per second)
     * @return The current velocity of the wheel
     */
    public double getVelocity() {
        return this.m_driveEncoder.getVelocity();
    }

    /**
     * Get the current steering setpoint (in degrees)
     * @return The current steering setpoint
     */
    public double getSteeringSetpoint() {
        return this.m_steerSetpoint;
    }

    /**
     * Get the current wheel velocity setpoint (in meters per second)
     * @return The current wheel velocity setpoint
     */
    public double getDriveSetpoint() {
        return this.m_driveSetpoint;
    }
}
