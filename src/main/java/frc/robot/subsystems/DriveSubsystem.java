package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule m_frontLeft = new SwerveModule(
        Constants.DriveConstants.kFrontLeftDrivingCanId,
        Constants.DriveConstants.kFrontLeftTurningCanId,
        Constants.DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
        Constants.DriveConstants.kFrontRightDrivingCanId,
        Constants.DriveConstants.kFrontRightTurningCanId,
        Constants.DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
        Constants.DriveConstants.kRearLeftDrivingCanId,
        Constants.DriveConstants.kRearLeftTurningCanId,
        Constants.DriveConstants.kBackLeftChassisAngularOffset);
    
    private final SwerveModule m_rearRight = new SwerveModule(
        Constants.DriveConstants.kRearRightDrivingCanId,
        Constants.DriveConstants.kRearRightTurningCanId,
        Constants.DriveConstants.kBackRightChassisAngularOffset);    
    

    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    /** Get gyro angle in radians for internal calculations */
    private Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_gyro.getAngle());
    }

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getGyroRotation(),
        new SwerveModulePosition[]{
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            
            
        });

    public DriveSubsystem() {}

    @Override
    public void periodic() {
        m_odometry.update(
            getGyroRotation(),
            new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
                
                
            });

        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
        
        
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
        
        
    }
    

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            getGyroRotation(),
            new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
                
                
            },
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                    ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                    rot * DriveConstants.kMaxAngularSpeed,
                    getGyroRotation())
                : new ChassisSpeeds(
                    xSpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                    ySpeed * DriveConstants.kMaxSpeedMetersPerSecond,
                    rot * DriveConstants.kMaxAngularSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
        
        
    }

    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearLeft.resetEncoders();
        m_rearRight.resetEncoders();
        
        

        // ✅ Ensure odometry is updated after resetting encoders
        m_odometry.resetPosition(
            getGyroRotation(),
            new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()   
            },
            new Pose2d(0, 0, Rotation2d.fromRadians(0)) // ✅ Reset to known starting pose
        );
    }

    public Command setXCommand() {
        return this.run(() -> {
            m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
            m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
            m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
            m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
            
            
        });
    }

    public Command zeroHeadingCommand() {
        return this.runOnce(() -> m_gyro.reset());
    }

    public double getHeading() {
        return getGyroRotation().getDegrees();
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
