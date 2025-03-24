package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import frc.robot.Configs;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;

    public ArmSubsystem(int motorID) {
        armMotor = new SparkMax(motorID, MotorType.kBrushless);
        armMotor.configure(Configs.CoralSubsystem.armConfig, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
        armEncoder = armMotor.getEncoder();
    }

    public void setMotorSpeed(double speed) {
        armMotor.set(speed);
    }

    public void stopMotor() {
        armMotor.stopMotor();
    }

    public void setPosition(double position) {
        armMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }

    public double getPosition() {
        return armEncoder.getPosition();
    }
}
