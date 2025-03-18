package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class SwerveModule extends SubsystemBase {
    
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;

    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingController;
    private final SparkClosedLoopController m_turningController;

    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private double normalizeAngle(double rawAngle) {
        return rawAngle % (2 * Math.PI);
    }

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        // ✅ Apply preconfigured settings from Configs.java
        m_drivingSpark.configure(Configs.SwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_turningSpark.configure(Configs.SwerveModule.turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // ✅ Get Integrated NEO Encoder for driving motor
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_drivingEncoder.setPosition(0);  // Reset driving encoder

        // ✅ Get Alternate Encoder (Throughbore Encoder) for turning motor
        m_turningEncoder = m_turningSpark.getAlternateEncoder();

        // ✅ Get controllers
        m_drivingController = m_drivingSpark.getClosedLoopController();
        m_turningController = m_turningSpark.getClosedLoopController();

        m_chassisAngularOffset = normalizeAngle(m_turningEncoder.getPosition() * 2 * Math.PI);
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    }

    /** Returns the current state of the module. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset)
        );
    }

    /** Returns the current position of the module. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),   // ✅ Get driving motor position
            new Rotation2d(m_turningEncoder.getPosition())  // ✅ Get turning motor angle
        );
    }

    /** Sets the desired state for the module. */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))
        );

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

        m_drivingController.setReference(
            correctedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity
        );
        m_turningController.setReference(
            correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition
        );

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
        //m_turningEncoder.setPosition(0);  // Disabled because it's an absolute encoder
    }

    /** Logs important values to the SmartDashboard. */
    public void logToSmartDashboard() {
        SmartDashboard.putNumber("Swerve/" + m_drivingSpark.getDeviceId() + "/Drive Encoder", m_drivingEncoder.getPosition());
        SmartDashboard.putNumber("Swerve/" + m_turningSpark.getDeviceId() + "/Turn Encoder", m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Swerve/" + m_turningSpark.getDeviceId() + "/Turn Angle (deg)", Math.toDegrees(m_turningEncoder.getPosition()));
    }

    /** This method will be called once per scheduler run */
    @Override
    public void periodic() {
        logToSmartDashboard();
        // Debug print statements if needed
        // System.out.println("Module " + m_turningSpark.getDeviceId() + " Encoder: " + m_turningEncoder.getPosition());
    }
}







/*package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SwerveModule {

    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;

    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingController;
    private final SparkClosedLoopController m_turningController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingController = m_drivingSpark.getClosedLoopController();
        m_turningController = m_turningSpark.getClosedLoopController();

        m_drivingSpark.configure(
            Configs.SwerveModule.drivingConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        m_turningSpark.configure(
            Configs.SwerveModule.turningConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingEncoder.getVelocity(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))
        );

        // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        m_drivingController.setReference(
            correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity
        );
        m_turningController.setReference(
            correctedDesiredState.angle.getRadians(), ControlType.kPosition
        );

        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
*/