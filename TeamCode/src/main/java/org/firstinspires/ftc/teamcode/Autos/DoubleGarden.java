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
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.LM1Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Double Garden Auto", group = "Autonomous")
public class DoubleGarden extends LinearOpMode {

    private Follower f;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.75, 6.75, 180);
    private final Pose startGarden = poseFactory.of(6.75, 9, 270);
    private final Pose startGardenControl1 = poseFactory.of(54.75, 10.75, 0);
    private final Pose startGardenControl2 = poseFactory.of(7.2143, 38.3007, 0);
    private final Pose startGardenSegment1Start = poseFactory.of(7.75, 9, 180);
    private final Pose startGardenSegment1End = poseFactory.of(7.75, 9, 270);
    private final Pose startGardenSegment2Heading = poseFactory.of(7.75, 9, -90);
    private final Pose gardenAP = poseFactory.of(10.75, 70.75, -90);
    private final Pose gardenAPControl1 = poseFactory.of(26.75, 57.75, 0);
    private final Pose gardenAPControl2 = poseFactory.of(9.75, 60.75, 0);
    private final Pose gardenAPSegment1Heading = poseFactory.of(10.75, 70.75, 270);
    private final Pose aPPush = poseFactory.of(13.75, 91.75, 270);
    private final Pose pushHive2 = poseFactory.of(59.75, 125.75, -200);
    private final Pose pushHive2Control1 = poseFactory.of(14.75, 64.75, 0);
    private final Pose pushHive2Control2 = poseFactory.of(45.75, 89.75, 0);
    private final Pose pushHive2Control3 = poseFactory.of(23.75, 118.75, 0);
    private final Pose pushHive2Control4 = poseFactory.of(59.75, 109.75, 0);
    private final Pose pushHive2Segment1Heading = poseFactory.of(59.75, 125.75, 270);
    private final Pose pushHive2Segment2Start = poseFactory.of(59.75, 125.75, -90);
    private final Pose pushHive2Segment2End = poseFactory.of(59.75, 125.75, -120);
    private final Pose pushHive2Segment4Start = poseFactory.of(59.75, 125.75, -121.1201);
    private final Pose pushHive2Segment4End = poseFactory.of(59.75, 125.75, 170);
    private final Pose hive2Flower2 = poseFactory.of(46.75, 125.75, 90);
    private final Pose hive2Flower2Control1 = poseFactory.of(52.75, 111.75, 0);
    private final Pose hive2Flower2Segment1Start = poseFactory.of(46.75, 125.75, 170);
    private final Pose hive2Flower2Segment1End = poseFactory.of(46.75, 125.75, 90);
    private final Pose flower2Hive2 = poseFactory.of(59.75, 125.75, 270);
    private final Pose flower2Hive2Control1 = poseFactory.of(52.75, 111.75, 0);
    private final Pose flower2Hive2Segment1Heading = poseFactory.of(59.75, 125.75, 90);
    private final Pose flower2Hive2Segment2Start = poseFactory.of(59.75, 125.75, 90);
    private final Pose flower2Hive2Segment2End = poseFactory.of(59.75, 125.75, 270);
    private final Pose hive2Flower1 = poseFactory.of(14.75, 46.75, 180);
    private final Pose hive2Flower1Control1 = poseFactory.of(58.4965, 26.5003, 0);
    private final Pose hive2Flower1Control2 = poseFactory.of(58.6753, 23.7532, 0);
    private final Pose hive2Flower1Control3 = poseFactory.of(58.6408, 19.8234, 0);
    private final Pose hive2Flower1Control4 = poseFactory.of(30.9584, 47.5873, 0);
    private final Pose hive2Flower1Segment2Start = poseFactory.of(14.75, 46.75, -91.5638);
    private final Pose hive2Flower1Segment2End = poseFactory.of(14.75, 46.75, 180);
    private final Pose hive2Flower1Segment3Heading = poseFactory.of(14.75, 46.75, 180);
    private final Pose flower1Hive2 = poseFactory.of(59.75, 125.75, 20);
    private final Pose flower1Hive2Control1 = poseFactory.of(64.0623, 28.8592, 0);
    private final Pose flower1Hive2Control2 = poseFactory.of(59.3981, 8.031, 0);
    private final Pose flower1Hive2Control3 = poseFactory.of(57.2315, 60.4748, 0);
    private final Pose flower1Hive2Segment1Start = poseFactory.of(59.75, 125.75, 180);
    private final Pose flower1Hive2Segment1End = poseFactory.of(59.75, 125.75, 90);
    private final Pose flower1Hive2Segment3Start = poseFactory.of(59.75, 125.75, 89.1559);
    private final Pose flower1Hive2Segment3End = poseFactory.of(59.75, 125.75, 10);
    private final Pose hive2Base = poseFactory.of(9.75, 118.75, 0);
    private final Pose hive2BaseControl1 = poseFactory.of(33.75, 123.75, 0);
    private final Pose hive2BaseControl2 = poseFactory.of(9.75, 141.5, 0);
    private final Pose hive2BaseSegment1Start = poseFactory.of(9.75, 118.75, 10);
    private final Pose hive2BaseSegment1End = poseFactory.of(9.75, 118.75, 0);
    private final Pose hive2BaseSegment2Heading = poseFactory.of(9.75, 118.75, 0);
    LM1Robot bot;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                bot.cmdStart,
                bot.cmdBlockerDown,
                waitMs(250),
                bot.cmdIdleShooter,
                bot.cmdIntake,
                bot.cmdActivateShooter,
                waitMs(200),
                bot.cmdShoot,
                deadline(waitMs(2800),
                        follow(f, startGarden())),
                follow(f, gardenAP()),

                follow(f, aPPush()),
                follow(f, pushHive2()),
                bot.cmdNulltake,
                bot.cmdDepollinatorDown,
                bot.cmdShoot,
                bot.cmdIntake,
                follow(f, hive2Flower2()),
                waitMs(2000),
                parallel
                        (follow(f, flower2Hive2()),
                                sequential
                                        (waitMs(200),
                                                bot.cmdDepollinatorUp)),
                bot.cmdShoot,
                bot.cmdDepollinatorDown,
                follow(f, hive2Flower1()),
                waitMs(2000),
                parallel
                        (follow(f, flower1Hive2()),
                        sequential
                                (waitMs(350),
                                bot.cmdDepollinatorUp)),
                bot.cmdNulltake,
                bot.cmdShoot,
                follow(f, hive2Base()),
                bot.cmdStop

        );
    }

    @Override
    public void runOpMode() {
        bot = new LM1Robot(hardwareMap,gamepad1,gamepad2,telemetry);
        bot.commands();
        Scheduler.reset();
        f = bot.follower;
        f.setPose(start);
        f.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            bot.positionUpdate();
            f.update();
            Scheduler.execute();

            bot.depollinatorJiggle();
            telemetry.addData("x", f.pose().x());
            telemetry.addData("y", f.pose().y());
            telemetry.addData("heading", f.pose().heading());

            if (f.currentPath() != null) {
                telemetry.addData("Current path distance remaining", f.distanceToEndpoint());
                telemetry.addData("Path number", f.pathIndex());
            }

            telemetry.update();
        }
        bot.stop();
    }

    public Path startGarden() {
        return Paths.curve(start, startGardenControl1, startGardenControl2, startGarden).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(startGardenSegment1Start, startGardenSegment1End)).until(1, Interpolator.constant(startGardenSegment2Heading)));
    }

    public Path gardenAP() {
        return Paths.curve(startGarden, gardenAPControl1, gardenAPControl2, gardenAP).heading(Interpolator.piecewise().until(1, Interpolator.constant(gardenAPSegment1Heading)));
    }

    public Path aPPush() {
        return Paths.line(gardenAP, aPPush).constant(aPPush);
    }

    public Path pushHive2() {
        return Paths.curve(aPPush, pushHive2Control1, pushHive2Control2, pushHive2Control3, pushHive2Control4, pushHive2).heading(Interpolator.piecewise().until(0.1561, Interpolator.constant(pushHive2Segment1Heading)).until(0.3303, Interpolator.linear(pushHive2Segment2Start, pushHive2Segment2End)).until(0.5064, Interpolator.tangent.reverse()).until(1, Interpolator.linear(pushHive2Segment4Start, pushHive2Segment4End)));
    }

    public Path hive2Flower2() {
        return Paths.curve(pushHive2, hive2Flower2Control1, hive2Flower2).heading(Interpolator.piecewise().until(1, Interpolator.linear(hive2Flower2Segment1Start, hive2Flower2Segment1End)));
    }

    public Path flower2Hive2() {
        return Paths.curve(hive2Flower2, flower2Hive2Control1, flower2Hive2).heading(Interpolator.piecewise().until(0.3222, Interpolator.constant(flower2Hive2Segment1Heading)).until(1, Interpolator.linear(flower2Hive2Segment2Start, flower2Hive2Segment2End)));
    }

    public Path hive2Flower1() {
        return Paths.curve(flower2Hive2, hive2Flower1Control1, hive2Flower1Control2, hive2Flower1Control3, hive2Flower1Control4, hive2Flower1).heading(Interpolator.piecewise().until(0.4806, Interpolator.tangent).until(0.882, Interpolator.linear(hive2Flower1Segment2Start, hive2Flower1Segment2End)).until(1, Interpolator.constant(hive2Flower1Segment3Heading)));
    }

    public Path flower1Hive2() {
        return Paths.curve(hive2Flower1, flower1Hive2Control1, flower1Hive2Control2, flower1Hive2Control3, flower1Hive2).heading(Interpolator.piecewise().until(0.3482, Interpolator.linear(flower1Hive2Segment1Start, flower1Hive2Segment1End)).until(0.7536, Interpolator.tangent).until(1, Interpolator.linear(flower1Hive2Segment3Start, flower1Hive2Segment3End)));
    }

    public Path hive2Base() {
        return Paths.curve(flower1Hive2, hive2BaseControl1, hive2BaseControl2, hive2Base).heading(Interpolator.piecewise().until(0.2051, Interpolator.linear(hive2BaseSegment1Start, hive2BaseSegment1End)).until(1, Interpolator.constant(hive2BaseSegment2Heading)));
    }
}
