package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.SimulationRobotConstants;

public class CoralSubsystem extends SubsystemBase {


    public enum Setpoint {
        kFeederStation, kLevel1, kLevel2, kLevel3, kLevel4
    }

    //private final SparkMax m_armMotor = new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);

    private final SparkMax armMotor = new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
    private final SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    private boolean isManualControl = false;

    private final SparkFlex elevatorMotor = new SparkFlex(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
    private final SparkClosedLoopController elevatorController = elevatorMotor.getClosedLoopController();
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private final SparkMax intakeMotor = new SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

    private final DigitalInput elevatorBottomSwitch = new DigitalInput(9);

    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private double armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kFeederStation;
    private double elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation;

    private final DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
    private final SparkFlexSim elevatorMotorSim;
    private final SparkLimitSwitchSim elevatorLimitSwitchSim;
    private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
        elevatorMotorModel,
        SimulationRobotConstants.kElevatorGearing,
        SimulationRobotConstants.kCarriageMass,
        SimulationRobotConstants.kElevatorDrumRadius,
        SimulationRobotConstants.kMinElevatorHeightMeters,
        SimulationRobotConstants.kMaxElevatorHeightMeters,
        true,
        SimulationRobotConstants.kMinElevatorHeightMeters,
        0.0,
        0.0);

    private final DCMotor armMotorModel = DCMotor.getNEO(1);
    private final SparkMaxSim armMotorSim;
    private final SingleJointedArmSim m_armSim =
        new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
            SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
          SimulationRobotConstants.kArmLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

    public CoralSubsystem() {
        armMotor.configure(Configs.CoralSubsystem.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor.configure(Configs.CoralSubsystem.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(Configs.CoralSubsystem.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder.setPosition(0);
        elevatorEncoder.setPosition(0);

        elevatorMotorSim = new SparkFlexSim(elevatorMotor, elevatorMotorModel);
        elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
        armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
    }

    public void resetArmEncoder() {
        armEncoder.setPosition(0);
    }

 /** Moves the arm manually using joystick input */
    public Command manualArmControlCommand(DoubleSupplier joystickInput) {
        return new RunCommand(() -> {
            double power = joystickInput.getAsDouble() * 0.5; // Limit power to 50%
            if (Math.abs(power) > 0.1) { // Apply deadband
                isManualControl = true;
                armMotor.set(power);
            } else {
                applyBrakeMode();
            }
        }, this);
    }

    /** Moves the arm to a preset setpoint */
    public Command setArmToPositionCommand(double position) {
        return new InstantCommand(() -> {
            isManualControl = false;
            armController.setReference(position, ControlType.kPosition);
        }, this);
    }

    /** Apply brake mode when manual control is idle */
    private void applyBrakeMode() {
        if (isManualControl) {
            armMotor.set(0);
        }
    }

    private void moveToSetpoint() {
        armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
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
            armEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(() -> {
            switch (setpoint) {
                case kFeederStation:
                    armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kFeederStation;
                    elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation;
                    break;
                case kLevel1:
                    armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel1;
                    elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel1;
                    break;
                case kLevel2:
                    armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel2;
                    elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel2;
                    break;
                case kLevel3:
                    armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel3;
                    elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel3;
                    break;
                case kLevel4:
                    armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel4;
                    elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel4;
                    break;
            }
        });
    }

    public Command runIntakeCommand() {
        return this.startEnd(() -> setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kForward), () -> setIntakePower(0.0));
    }

    public Command reverseIntakeCommand() {
        return this.startEnd(() -> setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kReverse), () -> setIntakePower(0.0));
    }

    public void moveArmManually(double joystickInput) {
        double speed = joystickInput * 0.3; //Scale joystick input to control speed
        if (Math.abs(speed) > 0.1) { //deadband to prevent small acciddental movment
            armMotor.set(speed);
        } else {
            armMotor.set(0); //stop motor if joystick input is idle
        }
            
    }
    
    @Override
    public void periodic() {
        moveToSetpoint();
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        SmartDashboard.putNumber("Coral/Arm/Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Elevator/Position", elevatorEncoder.getPosition());
        SmartDashboard.putBoolean("Coral/Elevator/Bottom Switch", !elevatorBottomSwitch.get());
    }

    public double getSimulationCurrentDraw() {
        return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() <= 0.01);

        m_elevatorSim.update(0.020);
        m_armSim.update(0.020);

        elevatorMotorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond() * 60.0, RobotController.getBatteryVoltage(), 0.02);
        armMotorSim.iterate(m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction, RobotController.getBatteryVoltage(), 0.02);
    }
}
