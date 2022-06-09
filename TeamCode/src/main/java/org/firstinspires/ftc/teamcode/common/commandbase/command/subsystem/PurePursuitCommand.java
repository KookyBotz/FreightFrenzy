package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;

import org.firstinspires.ftc.teamcode.common.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;

import java.util.List;

public class PurePursuitCommand extends CommandBase {
    private final DifferentialDrive drive;
    private final DifferentialDriveOdometry odometry;
    private final PurePursuitPath path;

    public PurePursuitCommand(DifferentialDrive d, DifferentialDriveOdometry o, Waypoint... w) {
        drive = d;
        odometry = o;
        path = new PurePursuitPath(w);
    }

    @Override
    public void initialize() {
        path.init();
    }

    @Override
    public void execute() {
        List<Double> powers = path.update(new Pose(odometry.getPoseMeters()));

        drive.tankDrive(powers.get(0), powers.get(1));
    }

    @Override
    public void end(boolean interrupted){
        drive.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return path.isFinished();
    }
}
