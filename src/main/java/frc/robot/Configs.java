package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {

    public static final class SwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            drivingConfig.inverted(false);  // ✅ Flip drive motor direction
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

            turningConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
            turningConfig.alternateEncoder
                //.inverted(turningCANId == 5 || turningCANId == 7 ? false : true)
                .countsPerRevolution(8192)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60.0);
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .pid(.1, 0, .0001)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class CoralSubsystem {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();
        public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        static {
            armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            armConfig.inverted(true);
            armConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);

            elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
            elevatorConfig.limitSwitch
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen);
            elevatorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(4200)
                .maxAcceleration(6000)
                .allowedClosedLoopError(0.5);

            intakeConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        }
    }

    public static final class AlgaeSubsystem {
        public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static {
            armConfig.smartCurrentLimit(40);
            armConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-0.5, 0.5);

            intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        }
    }
}
