package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    
    private final int translationAxis   = XboxController.Axis.kLeftY.value;
    private final int strafeAxis        = XboxController.Axis.kLeftX.value;
    private final int rotationAxis      = XboxController.Axis.kRightX.value;
    
    private final int firstStageAxis   = XboxController.Axis.kLeftY.value;
    private final int secondStageAxis  = XboxController.Axis.kRightY.value;
    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); //
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); //XboxController.Button.kLeftBumper.value
    private final JoystickButton bDriveButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowDrive = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton lift = new JoystickButton(driver, XboxController.Button.kA.value);
    
    private final JoystickButton xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton aButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton yButton = new JoystickButton(operator, XboxController.Button.kY.value);
    
    private final JoystickButton open = new JoystickButton(operator, XboxController.Button.kLeftBumper.value); //XboxController.Button.kLeftBumper.value
    private final JoystickButton close = new JoystickButton(operator, XboxController.Button.kRightBumper.value); //XboxController.Button.kLeftBumper.value
    
    SendableChooser<Command> m_chooser = new SendableChooser<>();


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final PenomaticSubsystem  m_Penomatic = new PenomaticSubsystem(); 
    private final VisionSubsystem m_Vision = new VisionSubsystem(s_Swerve);
    private final Arm m_Arm = new Arm();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        m_chooser.setDefaultOption("Deafult", new autoTest(s_Swerve, m_Arm, m_Penomatic));
        m_chooser.addOption("ScoreHigh Only", new putCube(m_Arm, m_Penomatic));
        m_chooser.addOption("HighAndLine", new highAndLine(s_Swerve, m_Arm, m_Penomatic));
        
        // CommandScheduler.getInstance().registerSubsystem(m_Penomatic);
        // CommandScheduler.getInstance().registerSubsystem(m_Vision);
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                m_Vision,
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> bDriveButton.getAsBoolean(),
                () -> slowDrive.getAsBoolean()

            )
        );

        //uncomint for testing 
        m_Arm.setDefaultCommand(
            new armControl(
                m_Arm, 
                () -> -operator.getRawAxis(firstStageAxis), 
                () -> operator.getRawAxis(secondStageAxis)
            )
        );
        
        SmartDashboard.putData(m_chooser);

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
        
        bButton.whileTrue(new Lowput(m_Arm, m_Penomatic));
        xButton.whileTrue(new closeArm(m_Arm, m_Penomatic));
        aButton.whileTrue(new takeFormGround(m_Arm, m_Penomatic));
        yButton.whileTrue(new holdStright(m_Arm, m_Penomatic));

        bDriveButton.onTrue(new InstantCommand( ()->m_Vision.TurnOnVisionProcessor()));
        bDriveButton.onFalse(new InstantCommand( ()->m_Vision.TurnOffVisionProcessor()));
        
        lift.onTrue(new InstantCommand(() -> m_Penomatic.up()));

        open.onTrue(new InstantCommand(()-> m_Penomatic.open()));
        close.onTrue(new InstantCommand(()-> m_Penomatic.close()));
        
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_chooser.getSelected();
    }

    public Swerve getSwerve(){
        return s_Swerve;
    }
}
