package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.command.AllianceHubPreCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.AllianceHubAutoCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadDumpCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.ff.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedWarehouseAuto extends OpMode {
    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private BarcodePipeline pipeline;

    private double loop;


    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
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

        BarcodePipeline.BarcodePosition analysis = pipeline.getAnalysis();
        robot.webcam.closeCameraDeviceAsync(() -> System.out.println("closed 1"));

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PreloadExtendCommand(robot, analysis, Alliance.BLUE, odometry, telemetry),
                        new PreloadDumpCommand(robot),
                        new DrivetrainCommand(new Pose(-15, 0, -90), robot, odometry, telemetry, 0)
                                .alongWith(new PreloadRetractCommand(robot)),
                        new DrivetrainCommand(new Pose(-15, -34, -60), robot, odometry, telemetry, 500)
                                .alongWith(new WaitCommand(1000)
                                        .andThen(new InstantCommand(() -> robot.intake.start()))),
                        new DrivetrainCommand(new Pose(-15, 10, -37.5), robot, odometry, telemetry, 500)
                                .alongWith(new AllianceHubPreCommand(robot)
                                        .andThen(new AllianceHubAutoCommand(robot))),
                        new PreloadDumpCommand(robot),
                        new DrivetrainCommand(new Pose(-13, 5, -90), robot, odometry, telemetry, 500)
                                .alongWith(new PreloadRetractCommand(robot)),
                        new DrivetrainCommand(new Pose(-13, -36, -60), robot, odometry, telemetry, 500)
                                .alongWith(new WaitCommand(1000)
                                        .andThen(new InstantCommand(() -> robot.intake.start()))),
                        new DrivetrainCommand(new Pose(-13, 15, -37.5), robot, odometry, telemetry, 500)
                                .alongWith(new AllianceHubPreCommand(robot)
                                        .andThen(new AllianceHubAutoCommand(robot))),
                        new PreloadDumpCommand(robot),
                        new DrivetrainCommand(new Pose(-13, 8, -90), robot, odometry, telemetry, 500)
                                .alongWith(new PreloadRetractCommand(robot)),
                        new DrivetrainCommand(new Pose(-13, -30, -90), robot, odometry, telemetry, 500)
                )
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.arm.loop();

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

    @Override
    public void stop() {
        robot.webcam.closeCameraDeviceAsync(() -> System.out.print("cam 1 closed"));
        robot.webcam2.closeCameraDeviceAsync(() -> System.out.print("cam 2 closed"));
        CommandScheduler.getInstance().reset();
    }
}
