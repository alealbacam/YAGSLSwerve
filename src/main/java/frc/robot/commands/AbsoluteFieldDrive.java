package frc.robot.commands;


import java.util.List;
import java.util.function.DoubleSupplier;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import swervelib.SwerveController;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;


/**
* An example command that uses an example subsystem.
*/
public class AbsoluteFieldDrive extends Command
{


   public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
   public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
   public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag 


 private final DrivetrainSubsystem swerve;
 private final DoubleSupplier  vX, vY, heading;


 /**
  * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
  * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
  * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
  * will rotate to.
  *
  * @param swerve  The swerve drivebase subsystem.
  * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
  *                deadband already accounted for.  Positive X is away from the alliance wall.
  * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
  *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
  *                station glass.
  * @param heading DoubleSupplier that supplies the robot's heading angle.
  */
 public AbsoluteFieldDrive(DrivetrainSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                           DoubleSupplier heading)
 {
   this.swerve = swerve;
   this.vX = vX;
   this.vY = vY;
   this.heading = heading;


   addRequirements(swerve);
 }


 @Override
 public void initialize()
 {
 }


 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute()
 {


   // Get the desired chassis speeds based on a 2 joystick module.


   ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                        new Rotation2d(heading.getAsDouble() * Math.PI));


   // Limit velocity to prevent tippy
   Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
   translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                          LOOP_TIME, ROBOT_MASS, List.of(CHASSIS),
                                          swerve.getSwerveDriveConfiguration());
   SmartDashboard.putNumber("LimitedTranslation", translation.getX());
   SmartDashboard.putString("Translation", translation.toString());


   // Make the robot move
   swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);


 }


 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted)
 {
 }


 // Returns true when the command should end.
 @Override
 public boolean isFinished()
 {
   return false;
 }




}
