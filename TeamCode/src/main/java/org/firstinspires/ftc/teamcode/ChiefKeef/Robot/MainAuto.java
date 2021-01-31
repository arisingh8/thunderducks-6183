package org.firstinspires.ftc.teamcode.ChiefKeef.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ChiefKeef.AutoFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class MainAuto extends AutoFunctions {
    @Override
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d(-62, -5, Math.toRadians(0));

        drive.setPoseEstimate(start);

        Trajectory beginning = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-3.25, 23.5), Math.toRadians(0))
                .build();

        Trajectory pshot2 = drive.trajectoryBuilder(beginning.end())
                .strafeTo(new Vector2d(-3.25, 16))
                .build();

        Trajectory pshot3 = drive.trajectoryBuilder(pshot2.end())
                .strafeTo(new Vector2d(-3.25, 8.5))
                .build();

        Trajectory wobblea = drive.trajectoryBuilder(pshot3.end())
                .splineTo(new Vector2d(5.5, -22.5), Math.toRadians(270))
                .build();

        Trajectory wobbleb = drive.trajectoryBuilder(pshot3.end())
                .splineTo(new Vector2d(29, 1.5), Math.toRadians(270))
                .build();

        Trajectory wobblec = drive.trajectoryBuilder(pshot3.end())
                .splineTo(new Vector2d(54.5, -23), Math.toRadians(270))
                .build();

        Trajectory end = drive.trajectoryBuilder(pshot3.end())
                .splineTo(new Vector2d(5.5, 0), Math.toRadians(90))
                .build();

        drive.followTrajectory(beginning);
        drive.followTrajectory(pshot2);
        drive.followTrajectory(pshot3);
        drive.followTrajectory(end);
    }
}
