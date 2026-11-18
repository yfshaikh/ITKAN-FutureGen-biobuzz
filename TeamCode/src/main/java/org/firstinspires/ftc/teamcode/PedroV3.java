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
 * PEDROV3: PedroV2 with named points and paths (same route, same mechanisms).
 * Each point is a place on the field; each path goes from one point to the next.
 * A path's heading goes from its start point's heading to its end point's heading.
 */
@Autonomous(name = "PedroV3", group = "Autonomous")
public class PedroV3 extends LinearOpMode {

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
    private static final double BACKUP_INCHES   = 6;     // TUNE: how far to back out of a flower after intaking

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

    // ===== Points: poseFactory.of(x, y, heading in degrees) =====
    private final Pose startPose       = poseFactory.of(60, 9, -90);
    private final Pose shoot1Pose      = poseFactory.of(59.7203, 13.9835, 270);
    private final Pose flowerDown1Pose = poseFactory.of(21.8519, 46.8937, 180);  // in front of flower 1: lower the bar here
    private final Pose flower1Pose     = poseFactory.of(15.5925, 46.8966, 180);  // pushed into flower 1: intake here (was 15.5925, moved 2 in left)
    private final Pose flower1BackPose = poseFactory.of(15.5925 + BACKUP_INCHES, 46.8966, 180);  // backed out: right (+x)
    private final Pose shoot2Control   = poseFactory.of(14.4827, 107.2396, 0);   // curve control point (heading unused)
    private final Pose shoot2Pose      = poseFactory.of(63.621, 126.3525, 80);
    private final Pose flowerDown2Pose = poseFactory.of(46.6685, 118.7013, 90);  // in front of flower 2
    private final Pose flower2Pose     = poseFactory.of(50.6287, 124.5548, 90);  // pushed into flower 2 (was y 125.5548, moved 2 in up)
    private final Pose flower2BackPose = poseFactory.of(50.6287, 124.5548 - BACKUP_INCHES, 90);  // backed out: down (-y)
    private final Pose shoot3Pose      = poseFactory.of(63.8507, 126.3962, 80);
    private final Pose prepParkPose    = poseFactory.of(23.9616, 107.4503, 90);
    private final Pose parkPose        = poseFactory.of(10.8261, 107.2791, 90);

    // Autonomous routine: drive, then do a mechanism step at each stop.
    public Command autoRoutine() {
        return sequential(
                follow(follower, startToShoot1()),
                shootFor(SHOOT_MS),
                follow(follower, shoot1ToFlowerDown1()),
                lowerFlower(),
                follow(follower, flowerDown1ToFlower1()),
                intakeFor(INTAKE_MS),
                follow(follower, flower1ToFlower1Back()),   // back out of flower 1
                liftFlower(),
                follow(follower, flower1ToShoot2()),
                shootFor(SHOOT_MS),
                follow(follower, shoot2ToFlowerDown2()),
                lowerFlower(),
                follow(follower, flowerDown2ToFlower2()),
                intakeFor(INTAKE_MS),
                follow(follower, flower2ToFlower2Back()),   // back out of flower 2
                liftFlower(),
                follow(follower, flower2ToShoot3()),
                shootFor(SHOOT_MS),
                follow(follower, shoot3ToPrepPark()),
                follow(follower, prepParkToPark())
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
        follower.setPose(startPose);
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

    // ===== Paths =====
    public Path startToShoot1() {
        return Paths.line(startPose, shoot1Pose).constant(shoot1Pose);
    }

    public Path shoot1ToFlowerDown1() {
        return Paths.line(shoot1Pose, flowerDown1Pose).linear(shoot1Pose, flowerDown1Pose);
    }

    public Path flowerDown1ToFlower1() {
        return Paths.line(flowerDown1Pose, flower1Pose).linear(flowerDown1Pose, flower1Pose);
    }

    // Same heading at both ends, so the robot keeps facing the flower and drives backwards.
    public Path flower1ToFlower1Back() {
        return Paths.line(flower1Pose, flower1BackPose).linear(flower1Pose, flower1BackPose);
    }

    public Path flower1ToShoot2() {
        return Paths.curve(flower1BackPose, shoot2Control, shoot2Pose).linear(flower1BackPose, shoot2Pose);
    }

    public Path shoot2ToFlowerDown2() {
        return Paths.line(shoot2Pose, flowerDown2Pose).linear(shoot2Pose, flowerDown2Pose);
    }

    public Path flowerDown2ToFlower2() {
        return Paths.line(flowerDown2Pose, flower2Pose).linear(flowerDown2Pose, flower2Pose);
    }

    public Path flower2ToFlower2Back() {
        return Paths.line(flower2Pose, flower2BackPose).linear(flower2Pose, flower2BackPose);
    }

    public Path flower2ToShoot3() {
        return Paths.line(flower2BackPose, shoot3Pose).linear(flower2BackPose, shoot3Pose);
    }

    public Path shoot3ToPrepPark() {
        return Paths.line(shoot3Pose, prepParkPose).linear(shoot3Pose, prepParkPose);
    }

    public Path prepParkToPark() {
        return Paths.line(prepParkPose, parkPose).linear(prepParkPose, parkPose);
    }
}
