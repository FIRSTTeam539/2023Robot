package frc.robot;



import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Commands.TurnToAngleProfiled;
import frc.robot.Commands.TurnToAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
    public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ArmSubsystem m_robotArm = new ArmSubsystem();
    private final Command m_autoCommand = new Autos(m_robotDrive, m_robotArm);
    // The driver's controller
    CommandXboxController m_driverController0 =
        new CommandXboxController(OIConstants.kDriverControllerPort0);
    CommandXboxController m_driverController1 =
        new CommandXboxController(OIConstants.kDriverControllerPort1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        Commands.run(
            () ->
                m_robotDrive.tankDrive(
                    m_driverController0.getRightY(), m_driverController0.getLeftY()),
            m_robotDrive)); // controls were reversed (- instead of +)
    //sets defalt arm commands
    m_robotArm.setDefaultCommand(
        Commands.run(
            () ->
                m_robotArm.moveArm(m_driverController1.getLeftY(), (m_driverController1.getRightTriggerAxis()*ArmConstants.kshootRate
                -m_driverController1.getLeftTriggerAxis()*ArmConstants.kintakeRate)), m_robotArm));
       
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */

     private void configureButtonBindings() {
    // configures button bindings
    //drives at half speed when left bumper is held
    m_driverController0
    .rightBumper()
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1))); 
    //should reverse direction of drive when y button is pressed
    m_driverController0
    .y()
    .toggleOnTrue(Commands.run(() -> m_robotDrive.tankDrive(-m_driverController0.getRightY(), -m_driverController0.getLeftY())))
    .toggleOnFalse(Commands.run (() -> m_robotDrive.tankDrive(m_driverController0.getRightY(), m_driverController0.getLeftY())));
            //direction was revereed (True +, False -)
    // Stabilize robot to drive straight with gyro when left bumper is held
    m_driverController0
        .leftBumper()
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizationP,
                    DriveConstants.kStabilizationI,
                    DriveConstants.kStabilizationD),
                // Close the loop on the turn rate
                m_robotDrive::getTurnRate,
                // Setpoint is 0
                0,
                // Pipe the output to the turning controls
                output -> m_robotDrive.arcadeDrive(-m_driverController0.getLeftY(), output),
                // Require the robot drive
                m_robotDrive));
        
    }
    /**
     * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
     * disable to prevent integral windup.
     */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoCommand;
    }
}