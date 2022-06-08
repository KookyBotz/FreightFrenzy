package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.command.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.ff.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.nio.file.DirectoryIteratorException;

@Autonomous
public class BlueDuckAuto extends OpMode {
    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private BarcodePipeline pipeline;
    private BarcodePipeline.BarcodePosition analysis;
    private double loop;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometry = new DifferentialDriveOdometry(new Rotation2d(0));

        robot.webcam.setPipeline(pipeline = new BarcodePipeline());

        robot.webcam.setMillisecondsPermissionTimeout(2500);
        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard.getInstance().startCameraStream(robot.webcam, 30);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new DrivetrainCommand(new Pose(-20, 5, -40), robot, odometry, telemetry),
                        new WaitCommand(2000),
                        new DrivetrainCommand(new Pose(-4, -21, -50), robot, odometry, telemetry, 500),
                        new WaitCommand(2000),
                        new DrivetrainCommand(new Pose(-20, -15, -185), robot, odometry, telemetry, 1500)
                )
        );

        robot.turret.middle();
        robot.arm.linkageIn();
        robot.bucket.in();
        robot.arm.armIn();
    }

    @Override
    public void init_loop() {
        telemetry.addData("analysis: ", pipeline.getAnalysis());
    }

    public void start() {
        analysis = pipeline.getAnalysis();
        robot.webcam.closeCameraDevice();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        Rotation2d imu = new Rotation2d(-robot.imu.getAngularOrientation().firstAngle);
        double right_position = robot.right_encoder.getPosition() / 383.6 * 11.873743682;
        double left_position = robot.left_encoder.getPosition() / 383.6 * 11.873743682;

        odometry.update(
                imu, right_position, left_position
        );

        telemetry.addLine(odometry.getPoseMeters().toString());

        double time = System.currentTimeMillis();
        telemetry.addData("loop ", time - loop);
        telemetry.addData("arm ", robot.arm.getCachePos());
        loop = time;

        telemetry.update();
    }
}
