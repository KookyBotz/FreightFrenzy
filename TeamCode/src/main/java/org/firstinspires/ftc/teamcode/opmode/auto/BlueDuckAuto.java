package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.CycleDuckCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.DuckArmExtend;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.DuckArmRetract;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.DuckCycleExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.DuckCycleOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadDumpCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.autocommand.PreloadRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem.DrivetrainCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.DuckieJankCommand;
import org.firstinspires.ftc.teamcode.common.ff.Alliance;
import org.firstinspires.ftc.teamcode.common.ff.BarcodePipeline;
import org.firstinspires.ftc.teamcode.common.ff.DuckPipeline2;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueDuckAuto extends OpMode {
    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private BarcodePipeline pipeline;
    private BarcodePipeline.BarcodePosition analysis;

    private DuckPipeline2 pipeline2;
    private double loop;


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
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
        analysis = pipeline.getAnalysis();
        robot.webcam.closeCameraDeviceAsync(() -> System.out.println("closed 1"));

        robot.webcam2.setPipeline(pipeline2 = new DuckPipeline2());
        robot.webcam2.setMillisecondsPermissionTimeout(2500);
        robot.webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PreloadExtendCommand(robot, analysis, odometry, telemetry),
                        new PreloadDumpCommand(robot),
                        new WaitCommand(500).andThen(new DrivetrainCommand(new Pose(-3, -20.5, -50), robot, odometry, telemetry, 750))
                                .alongWith(new PreloadRetractCommand(robot)),
                        new CycleDuckCommand(robot).alongWith(new DuckArmExtend(robot, Alliance.BLUE)),
                        new DrivetrainCommand(new Pose(-25, -15, 0), robot, odometry, telemetry, 750).alongWith(new DuckArmRetract(robot)),
                        new DuckieJankCommand(robot, pipeline2, odometry, telemetry, 1500,
                                new SequentialCommandGroup(
                                        ((analysis == BarcodePipeline.BarcodePosition.RIGHT) ?
                                                new DrivetrainCommand(new Pose(-53, -15, -90), robot, odometry, telemetry, 1000).alongWith(new DuckCycleExtendCommand(robot, Alliance.BLUE))
                                                : new DrivetrainCommand(new Pose(-53, 5, -90), robot, odometry, telemetry, 1000).alongWith(new DuckCycleExtendCommand(robot, Alliance.BLUE))),
                                        new DrivetrainCommand(new Pose(-53, 17, -90), robot, odometry, telemetry, 1000).andThen(new DuckCycleOuttakeCommand(robot)),
                                        new DrivetrainCommand(new Pose(-53, 44, 0), robot, odometry, telemetry, 1000),
                                        new DrivetrainCommand(new Pose(-27, 46, 90), robot, odometry, telemetry, 2000),
                                        new DrivetrainCommand(new Pose(-27, 100, 0), robot, odometry, telemetry, 2000)
                                )
                        )
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
        robot.webcam2.closeCameraDevice();
    }
}
