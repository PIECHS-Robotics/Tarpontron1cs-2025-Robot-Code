package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
//import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class RobotContainer {

    public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

    private final PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
    private final PS4Controller m_operatorController = new PS4Controller(1); // Assuming Controller 2 is in USB port 1


    public RobotContainer() {
        configureButtonBindings();

        // Default drive command (joystick control)
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband),  // Forward/Back
                    MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),  // Strafe
                    -MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband), // FIXED: Right Stick X (Rotation)
                    true),
                m_robotDrive
            )
            
        );

        m_coralSubsystem.setDefaultCommand(
            new RunCommand(() -> 
                m_coralSubsystem.moveArmManually(m_operatorController.getRawAxis(4)), 
                m_coralSubsystem
            )
        );


        // Default idle behavior for Algae subsystem
        m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
    }

    private void configureButtonBindings() {

        //m_coralSubsystem.setDefaultCommand(m_coralSubsystem.manualArmControlCommand(() -> m_operatorController.getRawAxis(4)));
        
        // Set swerve to X (L1)
        new JoystickButton(m_driverController, 4)
            .whileTrue(m_robotDrive.setXCommand());

        // Coral intake (L2)
        //new JoystickButton(m_driverController, 7)
        //     .whileTrue(m_coralSubsystem.runIntakeCommand());

        new Trigger(() -> m_driverController.getRawAxis(2) > 0.2)
            .whileTrue(m_coralSubsystem.runIntakeCommand());

        new Trigger(() -> m_operatorController.getRawAxis(2) > 0.2)
            .whileTrue(m_coralSubsystem.runIntakeCommand());    

        // Reverse Coral intake (R2)
        //new JoystickButton(m_driverController, 8)
        //    .whileTrue(m_coralSubsystem.reverseIntakeCommand());

        // R2 -> Reverse Coral Intake
        new Trigger(() -> m_driverController.getRawAxis(3) > 0.2)
            .whileTrue(m_coralSubsystem.reverseIntakeCommand());

        new Trigger(() -> m_operatorController.getRawAxis(3) > 0.2)
            .whileTrue(m_coralSubsystem.reverseIntakeCommand());    

        // Feeder station setpoint (Circle) & stow algae intake
        new JoystickButton(m_driverController, 2)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));

        new JoystickButton(m_operatorController, 2)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));            

        // Level 2 setpoint (Cross)
        new JoystickButton(m_driverController, 1)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel2));

        new JoystickButton(m_operatorController, 1)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel2));    

        // Level 3 setpoint (Square)
        new JoystickButton(m_driverController, 3)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel3));

        new JoystickButton(m_operatorController, 3)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel3));    

        // Level 4 setpoint (Triangle)
        new JoystickButton(m_driverController, 10)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel4));

        new JoystickButton(m_operatorController, 4)
            .onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.kLevel4));    

        // Run Algae intake (R1)
        new JoystickButton(m_driverController, 6)
            .whileTrue(m_algaeSubsystem.runIntakeCommand());

        new JoystickButton(m_operatorController, 6)
            .whileTrue(m_algaeSubsystem.runIntakeCommand());    

        // Reverse Algae intake (L1)
        new JoystickButton(m_driverController, 5)
            .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

        new JoystickButton(m_operatorController, 5)
            .whileTrue(m_algaeSubsystem.reverseIntakeCommand());    

        // Zero heading (Options)
        //new JoystickButton(m_driverController, 4)
            //.onTrue(m_robotDrive.zeroHeadingCommand());

        /*new JoystickButton(m_operatorController, 10)
            .onTrue(new InstantCommand(() -> {
            m_coralSubsystem.resetArmEncoder();
            System.out.println("✅ Coral Arm Encoder Reset to 0!");
            }, m_coralSubsystem)); */   
    }

    

    public double getSimulationTotalCurrentDraw() {
        return m_coralSubsystem.getSimulationCurrentDraw() + m_algaeSubsystem.getSimulationCurrentDraw();
    }

    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);
    
        // Define an example trajectory
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
    
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // ✅ **Corrected `SwerveControllerCommand` constructor**
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Pose supplier
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0), // X controller
            new PIDController(AutoConstants.kPYController, 0, 0), // Y controller
            thetaController, // Theta controller
            (states) -> m_robotDrive.setModuleStates(states), // Set module states
            m_robotDrive);
    
        // Reset odometry to starting pose of trajectory
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
        // Return command sequence
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
    
}
