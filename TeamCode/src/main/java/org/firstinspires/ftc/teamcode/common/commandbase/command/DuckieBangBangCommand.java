package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;

import org.firstinspires.ftc.teamcode.common.ff.DuckPipeline2;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class DuckieBangBangCommand extends CommandBase {
    private OpenCvWebcam camera;
    private Robot robot;
    private DuckPipeline2 pipeline;

    public static double turn_power = 0.195;

    public static double min_threshold = 145;
    public static double max_threshold = 175;

    private boolean finished = false;

    public DuckieBangBangCommand(OpenCvWebcam camera, Robot robot) {
        this.camera = camera;
        this.robot = robot;
        turn_power *= 12.0 / robot.batteryVoltageSensor.getVoltage();
    }

    @Override
    public void initialize() {
        camera.setPipeline(pipeline = new DuckPipeline2());
    }

    @Override
    public void execute() {
        double pos = pipeline.getDuckie();
        if (pos != 0) {
            if (pos < min_threshold) {
                robot.drive.tankDrive(turn_power, -turn_power);
            } else if (pos > max_threshold) {
                robot.drive.tankDrive(-turn_power, turn_power);
            } else {
                robot.drive.tankDrive(0, 0);
                finished = true;
            }
        } else {
            robot.drive.tankDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted){
        robot.drive.tankDrive(0, 0);
    }
}
