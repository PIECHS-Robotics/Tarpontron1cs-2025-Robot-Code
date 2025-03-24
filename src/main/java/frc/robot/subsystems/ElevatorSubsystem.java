package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final DigitalInput bottomLimitSwitch;

    public ElevatorSubsystem(int motorID, int limitSwitchChannel) {
        elevatorMotor = new SparkFlex(motorID, MotorType.kBrushless);
        elevatorMotor.configure(Configs.CoralSubsystem.elevatorConfig, SparkFlex.ResetMode.kNoResetSafeParameters, SparkFlex.PersistMode.kNoPersistParameters);
        elevatorEncoder = elevatorMotor.getEncoder();
        bottomLimitSwitch = new DigitalInput(9);
    }

    public void setMotorSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    public void stopMotor() {
        elevatorMotor.stopMotor();
    }

    public void setPosition(double position) {
        elevatorMotor.getClosedLoopController().setReference(position, SparkFlex.ControlType.kPosition);
    }

    public double getPosition() {
        return elevatorEncoder.getPosition();
    }

    public boolean isAtBottom() {
        return !bottomLimitSwitch.get();
    }

    @Override
    public void periodic() {
        if (isAtBottom()) {
            elevatorEncoder.setPosition(0);
        }
    }
}



/* 
public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex elevatorMotor;
    private final RelativeEncoder elevatorEncoder;
    private final DigitalInput bottomLimitSwitch;

    private final DigitalInput elevatorBottomSwitch = new DigitalInput(9);

    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private double elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel1;


    public ElevatorSubsystem(int motorID) {
        elevatorMotor.configure(Configs.CoralSubsystem.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //SparkFlexConfig config = new SparkFlexConfig();
        //config.inverted(false).idleMode(SparkMaxConfig.IdleMode.kBrake);
        //elevatorMotor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        elevatorEncoder.setPosition(0);
    }

    public void setMotorSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    public void stopMotor() {
        elevatorMotor.stopMotor();
    }

    private void zeroElevatorOnLimitSwitch() {
        if (!wasResetByLimit && !elevatorBottomSwitch.get()) {
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
        } else if (elevatorBottomSwitch.get()) {
            wasResetByLimit = false;
        }
    }

    private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            wasResetByButton = true;
            //armEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    @Override
    public void periodic() {
        //moveToSetpoint();
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

}
*/
