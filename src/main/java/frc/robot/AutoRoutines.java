package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
  Drive m_drive;
  AutoFactory m_factory;

  public AutoRoutines(Drive drive, AutoFactory factory) {
    m_drive = drive;
    m_factory = factory;
  }

  public Command autoPathCommandFactory(String name) {
    AutoLoop loop = m_factory.newLoop(name);
    AutoTrajectory trajectory = m_factory.trajectory(name, loop);

    loop.enabled()
        .onTrue(
            m_drive
                .runOnce(
                    () ->
                        trajectory
                            .getInitialPose()
                            .ifPresentOrElse(pose -> m_drive.setPose(pose), loop::kill))
                .andThen(trajectory.cmd()));

    return loop.cmd().withName(name);
  }

  public Command figureEightTest(AutoFactory factory) {
    return autoPathCommandFactory("figureEightTest");
  }

  public Command longFigureEightTest(AutoFactory factory) {
    return autoPathCommandFactory("longFigureEightTest");
  }

  public Command loopTest(AutoFactory factory) {
    return autoPathCommandFactory("loopTest");
  }

  public Command longLoopTest(AutoFactory factory) {
    return autoPathCommandFactory("longLoopTest");
  }
}
;
