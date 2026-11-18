package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * PEDROV2: TempAuton's route (same poses, same start heading, same headings on every path)
 * plus the mechanisms. Each of TempAuton's waitMs(...) placeholders is now a real step:
 *   waitMs(2000) -> shootFor(SHOOT_MS)     waitMs(750) -> lowerFlower()
 *   waitMs(1500) -> intakeFor(INTAKE_MS), then liftFlower()
 * Shooter code (setShooterVelocity, shooterSpeed, shooterReady) is the same as HamzaTeli2.
 */
@Autonomous(name = "PedroV2", group = "Autonomous")
public class PedroV2 extends LinearOpMode {

    // ===== Settings (TUNE on the robot) =====
    private static final double TRANSFER_POWER = -1.0;   // pulls pollen in (same as HamzaTeli's L2)
    private static final double INTAKE_MS = 1500;        // TUNE: how long intakeFor() runs at a flower
    private static final double SHOOT_MS  = 1500;        // TUNE: how long shootFor() feeds the shooter

    private static final double TARGET_SHOOTER_VELOCITY = 1800.0; // encoder ticks per second
    private static final double VELOCITY_TOLERANCE = 100;         // same as HamzaTeli2
    private static final double STEADY_MS = 150;                  // same as HamzaTeli2
    private static final double SPINUP_TIMEOUT_MS = 2000;         // shoot anyway if never steady

    private static final double FLOWER_UP_POS   = 1.0;
    private static final double FLOWER_DOWN_POS = 0.6;   // TUNE with ServoTuner (HamzaTeli2 uses 0.6)
    private static final double FLOWER_MOVE_MS  = 750;   // time for the bar to get there

    // Same as HamzaTeli2 (swapped: 0.5 blocks, 0.0 lets pollen through). TUNE with ServoTuner.
    private static final double STOPPER_CLOSED_POS = 0.5;
    private static final double STOPPER_OPEN_POS   = 0.0;

    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo flowerServo1, flowerServo2;
    private Servo stopperServo;
    private final ElapsedTime steadyTimer = new ElapsedTime();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose sTART = poseFactory.of(60, 9, -90);
    private final Pose path1 = poseFactory.of(59.7203, 13.9835, 270);
    private final Pose point2Start = poseFactory.of(59.7203, 13.9835, 270);
    private final Pose point2 = poseFactory.of(21.8519, 46.8937, 180);
    private final Pose point3Start = poseFactory.of(21.8519, 46.8937, 180);
    private final Pose point3 = poseFactory.of(15.5925, 46.8966, 180);

    private final Pose flower1toshotBStart = poseFactory.of(15.5925, 46.8966, 180);
    private final Pose flower1toshotB = poseFactory.of(63.621, 126.3525, 80);
    private final Pose flower1toshotBControl1 = poseFactory.of(14.4827, 107.2396, 0);
    private final Pose point5Start = poseFactory.of(63.621, 126.3525, 80);
    private final Pose point5 = poseFactory.of(46.6685, 118.7013, 90);
    private final Pose point6Start = poseFactory.of(46.6685, 118.7013, 90);
        private final Pose point6 = poseFactory.of(50.6287, 125.5548, 90); // 46.6287

    private final Pose point7Start = poseFactory.of(46.6287, 125.5548, 90);

private final Pose point7 = poseFactory.of(63.8507, 126.3962, 80);

    private final Pose point8Start = poseFactory.of(63.8507, 126.3962, 80);
    private final Pose point8 = poseFactory.of(23.9616, 107.4503, 90);
    private final Pose point9Start = poseFactory.of(23.9616, 107.4503, 90);
    private final Pose point9 = poseFactory.of(10.8261, 107.2791, 90);

    // Autonomous routine: TempAuton's paths, with the waits replaced by mechanism steps.
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                shootFor(SHOOT_MS),                 // was waitMs(2000)
                follow(follower, path2()),
                lowerFlower(),                      // was waitMs(750)
                follow(follower, path3()),
                intakeFor(INTAKE_MS),               // was waitMs(1500)
                liftFlower(),
                follow(follower, flower1toshotB()),
                shootFor(SHOOT_MS),                 // was waitMs(2000)
                follow(follower, path5()),
                lowerFlower(),                      // was waitMs(750)
                follow(follower, path6()),
                intakeFor(INTAKE_MS),               // was waitMs(1500)
                liftFlower(),
                follow(follower, path7()),
                shootFor(SHOOT_MS),                 // was waitMs(2000)
                follow(follower, path8()),
                follow(follower, path9())
        );
    }

    // ===== Mechanism commands =====
    // Flower bar down (HamzaTeli's X button), then wait for it to get there.
    Command lowerFlower() {
        return sequential(
                instant(() -> setFlower(FLOWER_DOWN_POS)),
                waitMs(FLOWER_MOVE_MS)
        );
    }

    // Flower bar up (HamzaTeli's triangle button), then wait for it to get there.
    Command liftFlower() {
        return sequential(
                instant(() -> setFlower(FLOWER_UP_POS)),
                waitMs(FLOWER_MOVE_MS)
        );
    }

    // Intake on for ms milliseconds, then off (HamzaTeli's L2 held for that long).
    Command intakeFor(double ms) {
        return sequential(
                instant(() -> setTransfer(TRANSFER_POWER)),
                waitMs(ms),
                instant(() -> setTransfer(0))
        );
    }

    // Spin up until the speed is steady (or SPINUP_TIMEOUT_MS passes), open the stopper,
    // feed for ms milliseconds, close the stopper, let the flywheel coast. (HamzaTeli2's R2.)
    Command shootFor(double ms) {
        return sequential(
                instant(() -> setShooterVelocity(TARGET_SHOOTER_VELOCITY)),
                instant(() -> steadyTimer.reset()),
                race(waitUntil(() -> shooterReady()), waitMs(SPINUP_TIMEOUT_MS)),
                instant(() -> stopperServo.setPosition(STOPPER_OPEN_POS)),
                intakeFor(ms),
                instant(() -> stopperServo.setPosition(STOPPER_CLOSED_POS)),
                instant(() -> setShooterPower(0))
        );
    }

    // ===== Helpers (same as HamzaTeli2) =====
    private void setTransfer(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setShooterVelocity(double ticksPerSecond) {
        shooterLeft.setVelocity(ticksPerSecond);
        shooterRight.setVelocity(ticksPerSecond);
    }

    private double shooterSpeed() {
        return (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
    }

    // True only once the speed has stayed close to the target for STEADY_MS.
    private boolean shooterReady() {
        double error = Math.abs(shooterSpeed() - TARGET_SHOOTER_VELOCITY);
        if (error > VELOCITY_TOLERANCE) {
            steadyTimer.reset();
        }
        return steadyTimer.milliseconds() >= STEADY_MS;
    }

    private void setFlower(double position) {
        flowerServo1.setPosition(position);
        flowerServo2.setPosition(position);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);   // sets up the 4 drive motors and the Pinpoint
        follower.setPose(sTART);
        follower.update();

        transferLeft  = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight = hardwareMap.get(DcMotorEx.class, "sr");
        flowerServo1 = hardwareMap.get(Servo.class, "f1");
        flowerServo2 = hardwareMap.get(Servo.class, "f2");
        stopperServo = hardwareMap.get(Servo.class, "ss");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);

        flowerServo1.setDirection(Servo.Direction.FORWARD);
        flowerServo2.setDirection(Servo.Direction.REVERSE);
        stopperServo.setDirection(Servo.Direction.FORWARD);

        // FLOAT: the flywheel coasts down instead of braking hard. RUN_USING_ENCODER: needed for setVelocity.
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopperServo.setPosition(STOPPER_CLOSED_POS);
        flowerServo1.setPosition(FLOWER_UP_POS);
        flowerServo2.setPosition(FLOWER_UP_POS);

        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("shooter speed", shooterSpeed());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        setTransfer(0);
        setShooterPower(0);
    }

    public Path path1() {
        return Paths.line(sTART, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path flower1toshotB() {
        return Paths.curve(flower1toshotBStart, flower1toshotBControl1, flower1toshotB).linear(flower1toshotBStart, flower1toshotB);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }

    public Path path9() {
        return Paths.line(point9Start, point9).linear(point9Start, point9);
    }
}
