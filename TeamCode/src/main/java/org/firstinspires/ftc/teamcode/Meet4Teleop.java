package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*
 * Meet4Teleop = Meet3Teleop + LIMELIGHT AUTO-AIM.
 *   Hold R2:     the robot turns to face the hive tag AND shoots (one ball per pulse) once it is
 *                aimed and the flywheel is at speed. No tag in view -> it shoots without aiming.
 *   Hold CIRCLE: aim only (line up before you start shooting).
 *   INIT: D-pad LEFT = RED alliance (tags 30-37), D-pad RIGHT = BLUE (tags 38-45).
 *
 * Meet3Teleop = Meet2Teleop + ONE BALL PER PULSE (see shooter block). Meet2 = Meet1 + a fix for the ball jamming the stopper.
 * Problem: the transfer pushes the next ball hard against the closed stopper, and the friction
 * stops the servo from swinging open. Fix: every time the stopper opens, the transfer first runs
 * BACKWARDS gently for BACKOFF_MS to pull the ball off the stopper, then feeds forward.
 * (A Thread.sleep in loop() can't do this: it freezes the whole robot, and the transfer
 * keeps pushing during the sleep.)
 *
 * FIELD-CENTRIC drive; OPTIONS re-zeroes heading. Flywheel always spins; hold R2 to shoot:
 * the stopper opens only while R2 is held AND the flywheel is steady at target speed.
 */
@TeleOp(name = "Meet4Teleop", group = "Competition")
public class Meet4Teleop extends OpMode {

    private static final double TRANSFER_POWER = -1;
    private static final double REVERSE_POWER  = 1.0;


    private static final double TARGET_SHOOTER_VELOCITY = 1600.0; // Ticks per second


    private static final double FLOWER_UP_POS   = 1.0;
    private static final double FLOWER_DOWN_POS = 0.6;


    // Swapped: on the robot, 0.0 sat outside (not blocking) and 0.5 swung inside (blocking).
    // TUNE with ServoTuner: CLOSED must block the ball, OPEN must let it through.
    private static final double STOPPER_CLOSED_POS = 0.7;
    private static final double STOPPER_OPEN_POS   = 0.0;
    // TUNE: how close to the target counts as "at speed"
    private static final double VELOCITY_TOLERANCE = 100;
    // TUNE: how long it must stay at speed before we shoot
    private static final double STEADY_MS = 150;
    // TUNE: when the stopper opens, run the transfer backwards this long to free the ball, then feed.
    // Too short: stopper still sticks. Too long: ball drops back too far and the shot is slow.
    private static final double BACKOFF_MS    = 150;
    private static final double BACKOFF_POWER = 0.3;   // gentle reverse (REVERSE_POWER direction)
    // TUNE: after the back-off, feed forward this long = ONE ball, then the stopper closes no matter what.
    // 2+ balls per shot: lower it. Ball doesn't leave: raise it.
    private static final double FEED_MS = 200;
    // TUNE: stay closed at least this long between shots (servo needs time to swing back in front of the next ball)
    private static final double MIN_CLOSED_MS = 300;

    private static final boolean SWAP_BACK_MOTORS = false;
    // TUNE: the Pedro heading (degrees) that points AWAY from the drivers. Used only when auton handed
    // over its heading (PoseHandoff). Test: run auton, start this, push forward: robot must go away from you.
    private static final double DRIVER_FORWARD_DEG = 0;
    private String headingSource = "";


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo flowerServo1, flowerServo2;
    private Servo stopperServo;
    private GoBildaPinpointDriver pinpoint;   // only its gyro (heading) is used here

    // ===== Auto-aim (values from LimelightAlignTest, which worked) =====
    private static final int    PIPELINE    = 0;     // AprilTag pipeline in the Limelight web UI
    private static final double KP          = 0.02;  // turn power per degree of tx
    private static final double MIN_TURN    = 0.06;  // so it doesn't stall just short of the tag
    private static final double MAX_TURN    = 0.4;
    private static final double ALIGNED_DEG = 1.0;   // TUNE: |tx| below this counts as aimed
    private static final double TURN_SIGN   = 1;     // flip to -1 if it turns away from the tag
    private Limelight3A limelight;
    private boolean redAlliance = true;              // picked in INIT with the D-pad

    private boolean flowerIsDown = false;


    // timer to track how long the speed has been steady for
    private ElapsedTime steadyTimer = new ElapsedTime();
    // keep track of if the shooter was ready (for the rumble on the controller)
    private boolean wasReady = false;

    // stopper: open/closed state and how long it has been open
    private boolean stopperOpen = false;
    private ElapsedTime stopperTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private String lastFlowerLog = "", lastStopperLog = "";

    double currentVelocity;

    @Override
    public void init() {

        frontLeft    = hardwareMap.get(DcMotor.class, "fl");
        frontRight   = hardwareMap.get(DcMotor.class, "fr");
        backLeft     = hardwareMap.get(DcMotor.class, SWAP_BACK_MOTORS ? "br" : "bl");
        backRight    = hardwareMap.get(DcMotor.class, SWAP_BACK_MOTORS ? "bl" : "br");
        transferLeft  = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight = hardwareMap.get(DcMotorEx.class, "sr");
        flowerServo1 = hardwareMap.get(Servo.class, "f1");
        flowerServo2 = hardwareMap.get(Servo.class, "f2");
        stopperServo = hardwareMap.get(Servo.class, "ss");
        pinpoint     = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        if (Double.isNaN(PoseHandoff.headingRad)) {
            pinpoint.resetPosAndIMU();   // no auton: robot must be still, heading 0 = the way it faces now
            headingSource = "Heading: zeroed (point robot away from you, or OPTIONS + D-pad)";
        } else {
            // auton just ran: keep its heading so field-centric is right without pressing OPTIONS
            pinpoint.setHeading(PoseHandoff.headingRad - Math.toRadians(DRIVER_FORWARD_DEG), AngleUnit.RADIANS);
            PoseHandoff.headingRad = Double.NaN;   // use it once; a second teleop run re-zeroes
            headingSource = "Heading: from auton";
        }

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);

        flowerServo1.setDirection(Servo.Direction.FORWARD);
        flowerServo2.setDirection(Servo.Direction.REVERSE);
        stopperServo.setDirection(Servo.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        setFlower(FLOWER_UP_POS);
        stopperServo.setPosition(STOPPER_CLOSED_POS);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left)  redAlliance = true;
        if (gamepad1.dpad_right) redAlliance = false;
        LLResultTypes.FiducialResult tag = findHiveTag();
        telemetry.addData("Alliance", "%s   (D-pad LEFT = red, RIGHT = blue)", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Hive tag", tag == null ? "not seen" : String.format("ID %d  tx %.1f", tag.getFiducialId(), tag.getTargetXDegrees()));
        telemetry.addLine("Ready. Press START.");
        telemetry.addLine(headingSource);
        telemetry.update();
    }

    @Override
    public void loop() {

        double y  = deadband(gamepad1.left_stick_y);
        double x  = deadband(-gamepad1.left_stick_x);
        double rx = deadband(gamepad1.right_stick_x);

        // OPTIONS = re-zero: wherever the robot points is now "forward" (away from the driver).
        // Hold OPTIONS + D-pad = tell it which way the robot faces RIGHT NOW, from where you stand,
        // so you don't have to turn it first: UP = away from you, DOWN = toward you, LEFT / RIGHT = your left / right.
        pinpoint.update();
        if (gamepad1.options) {
            double facing = 0;                       // UP, or OPTIONS alone
            if (gamepad1.dpad_left)  facing = 90;    // TUNE: if LEFT/RIGHT come out swapped, swap 90 and -90
            if (gamepad1.dpad_right) facing = -90;
            if (gamepad1.dpad_down)  facing = 180;
            pinpoint.setHeading(facing, AngleUnit.DEGREES);
        }
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        double fieldX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double fieldY = x * Math.sin(-heading) + y * Math.cos(-heading);
        x = fieldX * 1.1;   // strafe correction goes AFTER the rotation (it fixes the robot's sideways motion)
        y = fieldY;

        // AUTO-AIM: while R2 or CIRCLE is held and our hive tag is in view, the Limelight takes over
        // turning (the left stick still drives). aimed = the tag is centered.
        boolean r2 = gamepad1.right_trigger > 0.1;
        LLResultTypes.FiducialResult tag = findHiveTag();
        double tx = tag == null ? 0 : tag.getTargetXDegrees();
        boolean aimed = tag == null || Math.abs(tx) < ALIGNED_DEG;   // no tag: don't block shooting
        if ((r2 || gamepad1.circle) && tag != null) {
            rx = aimed ? 0 : TURN_SIGN * Math.max(-MAX_TURN, Math.min(MAX_TURN, KP * tx + Math.copySign(MIN_TURN, tx)));
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);

        // Flywheel always spins at target (R1 = reverse to clear a jam).
        // Holding R2 shoots ONE BALL PER PULSE: open -> back off -> feed FEED_MS -> close.
        // It reopens only after MIN_CLOSED_MS AND once the flywheel is steady at speed again.
        // (Waiting for the speed dip alone let all 4 through: the dip shows up too late.)
        if (gamepad1.right_bumper) {
            setShooterPower(-1.0);
        } else {
            setShooterVelocity(TARGET_SHOOTER_VELOCITY);
        }
        boolean ready = !gamepad1.right_bumper && shooterReady();   // checked every loop so the steady timer stays accurate
        if (stopperOpen) {
            if (!r2 || stopperTimer.milliseconds() >= BACKOFF_MS + FEED_MS) {
                stopperOpen = false;          // pulse over (or R2 released): close
                stopperTimer.reset();         // now times how long it has been closed
                steadyTimer.reset();          // must be steady at speed again before the next ball
            }
        } else if (r2 && ready && stopperTimer.milliseconds() >= MIN_CLOSED_MS) {
            stopperOpen = true;               // start a pulse
            stopperTimer.reset();             // now times the back-off + feed
        }
        // DEBUG: hold SHARE = stopper forced OPEN (transfer not touched). If it doesn't move even now,
        // it's the servo / wiring / config name / something jammed, not the code.
        boolean forceOpen = gamepad1.share;
        stopperServo.setPosition(stopperOpen || forceOpen ? STOPPER_OPEN_POS : STOPPER_CLOSED_POS);
        boolean backingOff = stopperOpen && stopperTimer.milliseconds() < BACKOFF_MS;

        // DEBUG: the reason the stopper is in its current state (first thing that's blocking it)
        String stopperWhy = forceOpen ? "FORCED OPEN (SHARE held)"
                : stopperOpen ? (backingOff ? "OPEN: backing off" : "OPEN: feeding")
                : !r2 ? "closed: R2 not held"
                : gamepad1.right_bumper ? "closed: R1 held (shooter reversed)"
                : !ready ? String.format("closed: waiting for speed %.0f / %.0f (must stay within %.0f for %.0f ms)",
                        shooterSpeed(), TARGET_SHOOTER_VELOCITY, VELOCITY_TOLERANCE, STEADY_MS)
                : !aimed ? String.format("closed: waiting for aim, tx %.1f (needs under %.1f)", tx, ALIGNED_DEG)
                : stopperTimer.milliseconds() < MIN_CLOSED_MS ? "closed: waiting between balls"
                : "closed: ?";


        // L1 = reverse; while the stopper is opening, back the ball off it; then feed (R2 or L2).
        if (gamepad1.left_bumper) {
            setTransfer(REVERSE_POWER);
        } else if (backingOff) {
            setTransfer(BACKOFF_POWER);
        } else if (gamepad1.left_trigger > 0.1 || stopperOpen) {
            setTransfer(TRANSFER_POWER);
        } else {
            setTransfer(0);
        }


        if (gamepad1.cross) {
            flowerIsDown = true;
        } else if (gamepad1.triangle) {
            flowerIsDown = false;
        }
        setFlower(flowerIsDown ? FLOWER_DOWN_POS : FLOWER_UP_POS);


        // driver feedback: rumble once when the shooter becomes ready; show READY on telemetry
        if (ready && !wasReady) {
            gamepad1.rumble(200);   // one short buzz the moment it's ready
        }


        wasReady = ready;

        // LOGGING. getPosition() is the position we last SENT, not where the servo really is:
        // if the numbers change but the part doesn't move, it's wiring/config/mechanical, not code.
        String flowerLog = String.format("cross=%b triangle=%b -> %s (sent %.2f)",
                gamepad1.cross, gamepad1.triangle, flowerIsDown ? "DOWN" : "UP", flowerServo1.getPosition());
        String stopperLog = String.format("%s  (sent %.2f)", stopperWhy, stopperServo.getPosition());
        // every change also goes to the robot log with a timestamp, so brief flickers aren't missed:
        //   adb logcat | grep Meet4Teleop
        if (!flowerLog.equals(lastFlowerLog)) {
            RobotLog.ii("Meet4Teleop", "%.2fs FLOWER %s", runtime.seconds(), flowerLog);
            lastFlowerLog = flowerLog;
        }
        if (!stopperLog.equals(lastStopperLog)) {
            RobotLog.ii("Meet4Teleop", "%.2fs STOPPER %s speed=%.0f", runtime.seconds(), stopperLog, shooterSpeed());
            lastStopperLog = stopperLog;
        }

        telemetry.addData("Aim", tag == null ? "no tag" : String.format("ID %d  tx %.1f  %s", tag.getFiducialId(), tx, aimed ? "AIMED" : "turning"));
        telemetry.addData("Heading (deg)", "%.1f   (OPTIONS = re-zero, +D-pad = facing)", Math.toDegrees(heading));
        telemetry.addData("Shooter", ready ? "READY" : "spinning up");
        telemetry.addData("Shooter speed", shooterSpeed());
        telemetry.addData("Shooter L Vel", shooterLeft.getVelocity());
        telemetry.addData("Shooter R Vel", shooterRight.getVelocity());
        telemetry.addData("Flower", flowerLog);
        telemetry.addData("Flower Pos 1 / 2 (sent)", "%.2f / %.2f", flowerServo1.getPosition(), flowerServo2.getPosition());
        telemetry.addData("Stopper", stopperLog);
        telemetry.addData("Stopper open for", stopperOpen ? String.format("%.0f ms", stopperTimer.milliseconds()) : "-");
        telemetry.update();
    }



    private void setShooterVelocity(double ticksPerSecond) {
        shooterLeft.setVelocity(ticksPerSecond);
        shooterRight.setVelocity(ticksPerSecond);
    }

    // calculates shooter speed
    private double shooterSpeed() {
        return (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
    }

    // This function returns true once the speed has stayed close to the target for STEADY_MS.
    private boolean shooterReady() {
        double error = Math.abs(shooterSpeed() - TARGET_SHOOTER_VELOCITY);
        if (error > VELOCITY_TOLERANCE) {
            steadyTimer.reset();   // not steady yet: start counting again
        }
        if (steadyTimer.milliseconds() >= STEADY_MS) {
            return true;
        } else {
            return false;
        }
    }

    private double deadband(double v) {
        return Math.abs(v) < 0.08 ? 0 : v;
    }

    private void setTransfer(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setFlower(double position) {
        flowerServo1.setPosition(position);
        flowerServo2.setPosition(position);
    }

    // Our alliance's hive tag (red 30-37, blue 38-45), or null if none is in view.
    private LLResultTypes.FiducialResult findHiveTag() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        int lo = redAlliance ? 30 : 38;
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            if (f.getFiducialId() >= lo && f.getFiducialId() <= lo + 7) return f;
        }
        return null;
    }

    @Override
    public void stop() {
        limelight.stop();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);}
}
