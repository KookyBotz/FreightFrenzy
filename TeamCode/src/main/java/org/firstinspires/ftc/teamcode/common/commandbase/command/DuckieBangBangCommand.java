package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;

import org.firstinspires.ftc.teamcode.common.ff.DuckPipeline2;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class DuckieBangBangCommand extends CommandBase {
    private OpenCvWebcam camera;
    private DifferentialDrive drive;
    private DuckPipeline2 pipeline;

    public static double turn_power = 0.15;

    public static double min_threshold = 150;
    public static double max_threshold = 170;

    public DuckieBangBangCommand(OpenCvWebcam camera, DifferentialDrive drive) {
        this.camera = camera;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        camera.setPipeline(pipeline = new DuckPipeline2());
    }

    @Override
    public void execute() {
        double pos = pipeline.getDuckie();
        if (pos < min_threshold) {
            drive.tankDrive(turn_power, -turn_power);
        } else if (pos > max_threshold) {
            drive.tankDrive(-turn_power, turn_power);
        } else {
            drive.tankDrive(0, 0);
        }
    }
}
