package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int up = XboxController.Axis.kRightY.value;

    private final int down = XboxController.Axis.kLeftTrigger.value;
    private final int up2 = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final PenomaticSubsystem  m_Penomatic = new PenomaticSubsystem(); 
    private final VisionSubsystem m_Vision = new VisionSubsystem(s_Swerve);
    private final Arm m_Arm = new Arm();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(m_Penomatic);
        CommandScheduler.getInstance().registerSubsystem(m_Vision);
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        //uncomint for testing 
        m_Arm.setDefaultCommand(
            new armControl(
                m_Arm, 
                () -> -driver.getRawAxis(up), 
                () -> -driver.getRawAxis(up2), 
                () -> -driver.getRawAxis(down)
            )
        );
        

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // xButton.onTrue(new InstantCommand(() -> m_Arm.position_control(80000)));
        // aButton.onTrue(new InstantCommand(() -> m_Arm.position_control(0)));
        
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }

    public Swerve getSwerve(){
        return s_Swerve;
    }
}
