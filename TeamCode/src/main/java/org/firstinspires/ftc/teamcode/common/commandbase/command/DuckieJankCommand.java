package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.ff.DuckPipeline2;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class DuckieJankCommand extends CommandBase {
    private final Robot robot;
    private DifferentialDriveOdometry odometry;
    private Telemetry telemetry;
    private DuckPipeline2 pipeline;
    private Alliance alliance;

    private final double time;
    private ElapsedTime timer;

    public static double pixels_to_inches = 320 / 24.;

    private final SequentialCommandGroup after;

    public DuckieJankCommand(Robot robot, DuckPipeline2 pipeline, Alliance alliance, DifferentialDriveOdometry odometry, Telemetry telemetry, double time, SequentialCommandGroup after) {
        this.robot = robot;
        this.time = time;
        this.after = after;
        this.odometry = odometry;
        this.telemetry = telemetry;
        this.pipeline = pipeline;
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > time;
    }

    @Override
    public void end(boolean interrupted) {
        double pos = pipeline.getDuckie();
        robot.webcam2.closeCameraDeviceAsync(() -> System.out.println("closed 2"));

        double inches = (pos - 160) / pixels_to_inches;

        if (pos == 0) {
            inches = 0;
        }

        System.out.println("inches " + inches);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.start()),
                        alliance == Alliance.BLUE ?
                                new DrivetrainCommand(new Pose(-1, -15 + inches, 0), robot, odometry, telemetry, 1000).alongWith(new WaitCommand(2000)) :
                                new DrivetrainCommand(new Pose(-1, -(-15 + inches), 0), robot, odometry, telemetry, 1000).alongWith(new WaitCommand(2000)),
                        after
                )
        );
    }
}
