package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.DrivetrainCommand;
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

    private final double time;
    private ElapsedTime timer;

    public static double pixels_to_inches = 320 / 24.;

    private final SequentialCommandGroup after;

    public DuckieJankCommand(Robot robot, DifferentialDriveOdometry odometry, Telemetry telemetry, double time, SequentialCommandGroup after) {
        this.robot = robot;
        this.time = time;
        this.after = after;
        this.odometry = odometry;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        robot.webcam2.setPipeline(pipeline = new DuckPipeline2());        robot.webcam.setMillisecondsPermissionTimeout(2500);
        robot.webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        FtcDashboard.getInstance().stopCameraStream();
        FtcDashboard.getInstance().startCameraStream(robot.webcam2, 30);
        timer = new ElapsedTime();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > time;
    }

    @Override
    public void end(boolean interrupted) {
        double pos = pipeline.getDuckie();

        double inches = (pos - 160) / pixels_to_inches;

        if (pos == 0) {
            inches = 0;
        }

        System.out.println("inches " + inches);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new DrivetrainCommand(new Pose(-3, -15 + inches, 0), robot, odometry, telemetry, 1000),
                        after
                )
        );
    }
}
