package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the    
* project.
*/
public class Robot extends TimedRobot {
 private Command m_autonomousCommand;


 private RobotContainer m_robotContainer;


 // Declaration of motors
 private TalonFX leftMotorOne;
 private TalonFX leftMotorTwo;
 private TalonFX armMotor;
 private Joystick controller;
 private int timer = 0;




 /**
  * This function is run when the robot is first started up and should be used for any
  * initialization code.
  */
 @Override
 public void robotInit() {
   // In this method, we instantiate the RobotContainer, which sets up button bindings and autonomous chooser on the dashboard.
   // We also initialize motor controllers (TalonFX) for driving motors and a joystick for controlling the robot.
   // Finally, we set the initial speed of the motors to 0 to ensure they start at a stopped state.
   m_robotContainer = new RobotContainer();
   leftMotorOne = new TalonFX(13);
   leftMotorTwo = new TalonFX(14);
   armMotor = new TalonFX(15);
   controller = new Joystick(0);
   leftMotorOne.set(0);
   leftMotorTwo.set(0);
   armMotor.set(0);
}


 /**
  * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
  * that you want ran during disabled, autonomous, teleoperated and test.
  *
  * <p>This runs after the mode specific periodic functions, but before LiveWindow and
  * SmartDashboard integrated updating.
  */
 @Override
 public void robotPeriodic() {
   // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
   // commands, running already-scheduled commands, removing finished or interrupted commands,
   // and running subsystem periodic() methods.  This must be called from the robot's periodic
   // block in order for anything in the Command-based framework to work.
   CommandScheduler.getInstance().run();
 }


 /** This function is called once each time the robot enters Disabled mode. */
 @Override
 public void disabledInit() {}


 @Override
 public void disabledPeriodic() {}


 /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
 @Override
 public void autonomousInit() {


   // schedule the autonomous command (example)
   if (m_autonomousCommand != null) {
     m_autonomousCommand.schedule();
   }
 }


 /** This function is called periodically during autonomous. */
 @Override
 public void autonomousPeriodic() {}


 @Override
 public void teleopInit() {
   // This makes sure that the autonomous stops running when
   // teleop starts running. If you want the autonomous to
   // continue until interrupted by another command, remove
   // this line or comment it out.
   if (m_autonomousCommand != null) {
     m_autonomousCommand.cancel();
   }
 }


 /** This function is called periodically during operator control. */
 @Override
 public void teleopPeriodic() {
   // In this method, we read the values of the left and right triggers on the controller, which control the intake and shooter motors respectively.
   // We calculate the motor speeds based on the trigger values, where the left trigger controls the intake and the right trigger controls the shooter.
   // Then, we set the motor speeds accordingly to achieve the desired movement.
   // Needs to be moved out of robot.java and organized into the subsystems
   double leftTrigger = controller.getRawAxis(2); // Reads the value of the left trigger
   double rightTrigger = controller.getRawAxis(3); // Reads the value of the right trigger
   boolean topButton = controller.getRawButton(4);
  boolean bottomButton = controller.getRawButton(1);


   double intakeSpeed = leftTrigger/2; // Calculates intake motor speed 
   double shooterSpeed = rightTrigger; // Shooter motor speed directly controlled by the right trigger


   // Set motor speeds based on trigger values
  if (intakeSpeed == 0 && shooterSpeed > 0) { // If only the shooter is active/''
       leftMotorTwo.set(shooterSpeed);
      timer++;
       if (timer>50) {
           leftMotorOne.set(shooterSpeed);
       }
    
   } else if (intakeSpeed > 0 && shooterSpeed == 0) { // If only the intake is active
       leftMotorOne.set(intakeSpeed*-1); // Assuming intake needs to spin in reverse direction
       leftMotorTwo.set(intakeSpeed*-1); // Assuming intake needs to spin in reverse direction
   } else { // If neither or both triggers are active
       leftMotorOne.set(0); // Stop both motors
       leftMotorTwo.set(0); // Stop both motors
       timer = 0;
   }


   if (!topButton && !bottomButton){
    armMotor.set(0);
   } else if (topButton && !bottomButton){
    armMotor.set(.4);
   } else if (bottomButton && !topButton){
    armMotor.set(-.4);
   }
}


 @Override
 public void testInit() {
   // Cancels all running commands at the start of test mode.
   CommandScheduler.getInstance().cancelAll();
 }


 /** This function is called periodically during test mode. */
 @Override
 public void testPeriodic() {}


 /** This function is called once when the robot is first started up. */
 @Override
 public void simulationInit() {}


 /** This function is called periodically whilst in simulation. */
 @Override
 public void simulationPeriodic() {}
}
