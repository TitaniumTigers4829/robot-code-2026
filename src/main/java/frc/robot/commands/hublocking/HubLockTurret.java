// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.hublocking;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.turret.TurretConstants;
// import frc.robot.subsystems.turret.TurretSubsystem;
// import java.util.Optional;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class HubLockTurret extends Command {
//   SwerveDrive swerveDrive;
//   TurretSubsystem turretSubsystem;
//   public double desiredHeading;
//   public Rotation2d heading;
//   public double turretAngleToHub;
//   public Translation2d hubPos;
//   public double turretToHubYDist;
//   public double turretToHubXDist;

//   /** Creates a new HubLockTurret. */
//   public HubLockTurret(SwerveDrive swerveDrive, TurretSubsystem turretSubsystem) {
//     this.swerveDrive = swerveDrive;
//     this.turretSubsystem = turretSubsystem;
//     addRequirements(swerveDrive, turretSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Optional<Alliance> alliance = DriverStation.getAlliance();
//     // Sets hub position based on the alliance
//     if (alliance.isPresent() && alliance.get() == Alliance.Red) {
//       hubPos = FieldConstants.RED_HUB_CENTER;
//     } else {
//       hubPos = FieldConstants.BLUE_HUB_CENTER;
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // Gets the heading of the robot as a Rotation2d
//     heading = swerveDrive.getOdometryRotation2d();

//     // Gets the position of the turret
//     Translation2d turretPos =
//         swerveDrive
//             .getEstimatedPose()
//             .getTranslation()
//             .plus(TurretConstants.TURRET_OFFSET.rotateBy(heading));

//     // Our turret angling math works as follows. Assuming the 0 rotations on the turret is
//     // facing the current heading of the robot and the turret rotates positively
// counterclockwise,
//     // we can approximate
//     // the angle it needs to turn in rotations from 0 to the target angle. This is the desired
//     // heading.
//     // With arctan we can calulate the angle the turret makes with the hub relative to the y
// axis,
//     // otherwise known as
//     // the field relative angle. The y axis is horizontal and the x axis is vertical from the
// driver
//     // station pov.
//     // We can subtract the heading (and therefore the zero angle) of the robot from the field
//     // relative angle. This
//     // will get the radians needed to turn to face the hub and when converted to rotations
// becomes
//     // the desired heading.

//     // Gets y and x distances of the turret to the hub
//     turretToHubYDist = hubPos.getY() - turretPos.getY();
//     turretToHubXDist = hubPos.getX() - turretPos.getX();

//     // Gets the needed angle for the turret to turn to face the hub in radians
//     turretAngleToHub = (Math.atan2(turretToHubYDist, turretToHubXDist) - heading.getRadians());

//     // Converts radians to rotations
//     desiredHeading = turretAngleToHub / (2 * Math.PI);

//     // Sets the turret to that angle
//     turretSubsystem.setTurretAngle(desiredHeading);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
