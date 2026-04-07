package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.DriveNoVision;
import frc.robot.commands.drive.FollowSwerveSampleCommand;
import frc.robot.commands.hublocking.ShootWhileMove;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakePivotBounceLower;
import frc.robot.commands.intake.IntakePivotUpCommand;
import frc.robot.extras.util.AllianceFlipper;
import frc.robot.subsystems.adjustableHood.AdjustableHoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** This class is where all the auto routines are created. It also contains the auto chooser */
public class Autos {
  private final LoggedDashboardChooser<String> chooser;
  private final AutoFactory autoFactory;
  private SwerveDrive swerveDrive;
  private VisionSubsystem visionSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private AdjustableHoodSubsystem hoodSubsystem;
  private final String NONE_NAME = "Do Nothing";

  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();

  private String selectedCommandName = NONE_NAME;
  private Command selectedCommand = Commands.none();
  private boolean selectedOnRed = false;

  public Autos(
      SwerveDrive swerveDrive,
      VisionSubsystem visionSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      AdjustableHoodSubsystem hoodSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.swerveDrive = swerveDrive;
    this.visionSubsystem = visionSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);
    // this sets up the auto factory
    this.autoFactory =
        new AutoFactory(
            swerveDrive::getEstimatedPose, // A function that returns the current robot pose
            swerveDrive::resetEstimatedPose, // A function that resets the current robot pose to the
            (SwerveSample sample) -> {
              FollowSwerveSampleCommand followSwerveSampleCommand =
                  new FollowSwerveSampleCommand(this.swerveDrive, this.visionSubsystem, sample);
              followSwerveSampleCommand.execute();
              Logger.recordOutput("Trajectory/sample", sample.getPose());
            }, // A function that follows a choreo trajectory
            false, // If alliance flipping should be enabled
            this.swerveDrive);

    addRoutine("y one meter", () -> yOneMeterAuto());

    addRoutine("test", () -> test_auto());

    addRoutine("blue_new_left_neutral_auto", () -> blueNewLeftNeutralAuto());

    // addRoutine("back and shoot", () -> backAndShootAuto());
  }

  public AutoRoutine yOneMeterAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.Y_ONE_METER_AUTO);
    AutoTrajectory yOneMeterTrajectory = routine.trajectory(AutoConstants.Y_ONE_METER_TRAJECTORY);
    routine
        .active()
        .onTrue(
            new SequentialCommandGroup(
                autoFactory.resetOdometry(AutoConstants.Y_ONE_METER_TRAJECTORY),
                yOneMeterTrajectory.cmd(),
                new DriveNoVision(
                        swerveDrive, () -> 0, () -> 0, () -> 0, () -> false, () -> false, null)
                    .withTimeout(1),
                yOneMeterTrajectory.cmd(),
                new DriveNoVision(
                        swerveDrive, () -> 0, () -> 0, () -> 0, () -> false, () -> false, null)
                    .withTimeout(1)
                // new PassFuelCommand(shooterSubsystem, hoodSubsystem).withTimeout(5)));
                ));

    return routine;
  }

  public AutoRoutine test_auto() {
    AutoRoutine routine = autoFactory.newRoutine("test");
    AutoTrajectory yOneMeterTrajectory = routine.trajectory("MiscTrajectories/test_red_path_right");
    routine
        .active()
        .onTrue(
            new SequentialCommandGroup(
                autoFactory.resetOdometry("MiscTrajectoritest_red_path_right"),
                yOneMeterTrajectory.cmd(),
                new DriveNoVision(
                        swerveDrive, () -> 0, () -> 0, () -> 0, () -> false, () -> false, null)
                    .withTimeout(1)));

    return routine;
  }

  public AutoRoutine blueNewLeftNeutralAuto() {
    AutoRoutine routine = autoFactory.newRoutine(AutoConstants.BLUE_LEFT_NEUTRAL_AUTO_NEW);
    AutoTrajectory blueNewLeftNeutralTrajectory =
        routine.trajectory(AutoConstants.LEFT_NEW_NEUTRAL_TRAJECTORY);
    AutoTrajectory secondSweep =
        routine.trajectory(AutoConstants.BLUE_LEFT_SECOND_SWEEP_TRAJECTORY);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    Commands.sequence(
                        autoFactory.resetOdometry(AutoConstants.LEFT_NEW_NEUTRAL_TRAJECTORY),
                        blueNewLeftNeutralTrajectory.cmd()),
                    Commands.sequence(
                        new IntakePivotUpCommand(intakeSubsystem).withTimeout(1.2),
                        new IntakeCommand(intakeSubsystem).withTimeout(2.1),
                        new IntakePivotUpCommand(intakeSubsystem).withTimeout(1.7))),
                new DriveNoVision(
                        swerveDrive, () -> 0, () -> 0, () -> 0, () -> false, () -> false, null)
                    .withTimeout(.1),
                Commands.deadline(
                    new ShootWhileMove(
                            swerveDrive, turretSubsystem, shooterSubsystem, hoodSubsystem)
                        .withTimeout(5),
                    new IntakePivotBounceLower(intakeSubsystem)),
                Commands.deadline(
                    secondSweep.cmd(),
                    Commands.sequence(
                        new IntakePivotUpCommand(intakeSubsystem).withTimeout(1),
                        new IntakeCommand(intakeSubsystem).withTimeout(1.8),
                        new IntakePivotUpCommand(intakeSubsystem).withTimeout(1.86))),
                new DriveNoVision(
                        swerveDrive, () -> 0, () -> 0, () -> 0, () -> false, () -> false, null)
                    .withTimeout(.1),
                Commands.deadline(
                    new ShootWhileMove(
                            swerveDrive, turretSubsystem, shooterSubsystem, hoodSubsystem)
                        .withTimeout(5),
                    new IntakePivotBounceLower(intakeSubsystem))));
    // Commands.sequence(
    // new WaitCommand(3),
    // new IntakeCommand(intakeSubsystem).withTimeout(6),
    // new IntakePivotUpCommand(intakeSubsystem).withTimeout(10))),
    // Commands.parallel(
    // new IntakePivotBounceLower(intakeSubsystem).withTimeout(5),
    // new ShootWhileMove(
    // swerveDrive, turretSubsystem, shooterSubsystem, hoodSubsystem)
    // .withTimeout(10)),
    // new InstantCommand(
    // () -> swerveDrive.resetEstimatedPose(visionSubsystem.getLastSeenPose()))));

    return routine;
  }

  /*
  private AutoRoutine createRoutine(AutoFactory factory, SwerveDrive swerve, Source source) {
    AutoRoutine routine = factory.newRoutine("Autogenerated Routine");
    return routine;
  }
  */

  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);
  private final Alert loadedAutoAlert = new Alert("", Alert.AlertType.kInfo);

  public void update() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      String selected = chooser.get();
      if (selected.equals(selectedCommandName) && selectedOnRed == AllianceFlipper.isRed()) {
        return;
      }
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get().withName(selectedCommandName);
      selectedOnRed = AllianceFlipper.isRed();
      loadedAutoAlert.setText("Loaded Auto: " + selectedCommandName);
      loadedAutoAlert.set(true);
    }
  }

  public void clear() {
    selectedCommandName = NONE_NAME;
    selectedCommand = Commands.none();
    selectedOnRed = false;
  }

  public Command getSelectedCommand() {
    // AutoRoutine test = yOneMeterAuto();
    // return test.cmd();
    return selectedCommand;
  }

  private void addRoutine(String name, Supplier<AutoRoutine> generator) {
    chooser.addOption(name, name);
    routines.put(name, () -> generator.get().cmd());
  }

  // private enum Source {
  //   L,
  //   R
  // }
}
