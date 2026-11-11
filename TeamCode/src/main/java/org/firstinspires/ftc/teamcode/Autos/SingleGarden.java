package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.LM1Robot;

@Autonomous(name = "Single Garden", group = "Autonomous")
public class SingleGarden extends LinearOpMode {

    private Follower f;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 7.75, 180);
    private final Pose startGarden = poseFactory.of(10, 9, 180);
    private final Pose gardenHive1 = poseFactory.of(57, 9, 180);
    private final Pose hive1Park = poseFactory.of(14.75, 94.75, 0);
    private final Pose hive1ParkControl1 = poseFactory.of(14.75, 9.75, 0);
    private final Pose hive1ParkControl2 = poseFactory.of(12.75, 66.75, 0);
    private final Pose hive1ParkControl3 = poseFactory.of(14.75, 56.75, 0);
    private final Pose hive1ParkSegment1Heading = poseFactory.of(14.75, 94.75, 180);
    private final Pose hive1ParkSegment2Start = poseFactory.of(14.75, 94.75, 180);
    private final Pose hive1ParkSegment2End = poseFactory.of(14.75, 94.75, 120);
    private final Pose hive1ParkSegment4Start = poseFactory.of(14.75, 94.75, 98.0273);
    private final Pose hive1ParkSegment4End = poseFactory.of(14.75, 94.75, 0);
    LM1Robot LM1Robot;
    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                LM1Robot.cmdBlockerDown,
                waitMs(250),
                LM1Robot.cmdIntake,
                LM1Robot.cmdShoot,
                follow(f, startGarden()),
                follow(f, gardenHive1()),
                LM1Robot.cmdShoot,
                LM1Robot.cmdNulltake,
                follow(f, hive1Park())
        );
    }

    @Override
    public void runOpMode() {
        LM1Robot = new LM1Robot(hardwareMap,gamepad1,gamepad2,telemetry);
        LM1Robot.Commands();
        Scheduler.reset();
        f = LM1Robot.follower;
        f.setPose(start);
        f.update();

        waitForStart();
        LM1Robot.start();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            LM1Robot.positionUpdate();
            f.update();
            Scheduler.execute();

            telemetry.addData("x", f.pose().x());
            telemetry.addData("y", f.pose().y());
            telemetry.addData("heading", f.pose().heading());

            if (f.currentPath() != null) {
                telemetry.addData("Current path distance remaining", f.distanceToEndpoint());
                telemetry.addData("Path number", f.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path startGarden() {
        return Paths.line(start, startGarden).constant(startGarden);
    }

    public Path gardenHive1() {
        return Paths.line(startGarden, gardenHive1).constant(gardenHive1);
    }

    public Path hive1Park() {
        return Paths.curve(gardenHive1, hive1ParkControl1, hive1ParkControl2, hive1ParkControl3, hive1Park).heading(Interpolator.piecewise().until(0.1277, Interpolator.constant(hive1ParkSegment1Heading)).until(0.3257, Interpolator.linear(hive1ParkSegment2Start, hive1ParkSegment2End)).until(0.6297, Interpolator.tangent).until(1, Interpolator.linear(hive1ParkSegment4Start, hive1ParkSegment4End)));
    }
}
