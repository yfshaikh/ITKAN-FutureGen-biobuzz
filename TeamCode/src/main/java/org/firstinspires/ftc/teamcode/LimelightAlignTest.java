package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * LIMELIGHT ALIGN TEST: no field needed, a printed AprilTag (36h11 family) is enough.
 *
 * Step 1 (sees the tag?): INIT, point the camera at the tag. Telemetry lists every tag ID it sees
 *         and tx (degrees left/right of center). Move the tag left/right: tx should change sign.
 * Step 2 (aligns?): START. Drive normally. HOLD CROSS: the robot turns until the tag is centered
 *         (|tx| < ALIGNED_DEG). You can still drive/strafe with the left stick while holding it.
 *
 * BIOBUZZ hive tags (from the SDK tag library, 3.25 in tags in clusters of 4):
 *   30-33 RED SCORING   34-37 RED AUDIENCE   38-41 BLUE AUDIENCE   42-45 BLUE SCORING
 * TARGET_IDS = which IDs to aim at. Empty = any tag (handy with a random printed tag).
 */
@TeleOp(name = "LimelightAlignTest", group = "Test")
public class LimelightAlignTest extends OpMode {

    private static final int PIPELINE = 0;                 // AprilTag pipeline slot set up in the Limelight web UI
    private static final int[] TARGET_IDS = {};            // e.g. {30, 31, 32, 33} for red scoring
    // TUNE: turn power = KP * tx, plus at least MIN_TURN so it doesn't stall near the target
    private static final double KP = 0.02;
    private static final double MIN_TURN = 0.06;
    private static final double MAX_TURN = 0.4;
    private static final double ALIGNED_DEG = 1.0;
    // TUNE: if holding cross turns the robot AWAY from the tag, flip this to -1
    private static final double TURN_SIGN = 1;

    private Limelight3A limelight;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();   // start in INIT so step 1 works before pressing START

        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void init_loop() {
        showVision(findTarget());
    }

    @Override
    public void loop() {
        LLResultTypes.FiducialResult tag = findTarget();
        showVision(tag);

        double y  = deadband(gamepad1.left_stick_y);
        double x  = deadband(-gamepad1.left_stick_x) * 1.1;
        double rx = deadband(gamepad1.right_stick_x);

        if (gamepad1.cross) {
            if (tag == null) {
                rx = 0;   // no tag: don't spin blindly
                telemetry.addData("Align", "NO TAG");
            } else {
                double tx = tag.getTargetXDegrees();
                if (Math.abs(tx) < ALIGNED_DEG) {
                    rx = 0;
                    telemetry.addData("Align", "ALIGNED");
                } else {
                    double turn = KP * tx + Math.copySign(MIN_TURN, tx);
                    rx = TURN_SIGN * Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));
                    telemetry.addData("Align", "turning %.2f", rx);
                }
            }
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    // The tag to aim at: the first one seen whose ID is in TARGET_IDS (or any, if TARGET_IDS is empty).
    private LLResultTypes.FiducialResult findTarget() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;
        for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
            if (TARGET_IDS.length == 0) return f;
            for (int id : TARGET_IDS) if (f.getFiducialId() == id) return f;
        }
        return null;
    }

    private void showVision(LLResultTypes.FiducialResult tag) {
        LLResult r = limelight.getLatestResult();
        telemetry.addData("Limelight connected", limelight.isConnected());
        if (r == null) {
            telemetry.addLine("No result yet (check config name 'limelight' and the USB cable)");
        } else {
            telemetry.addData("Pipeline", r.getPipelineIndex());
            telemetry.addData("Result age (ms)", r.getStaleness());
            StringBuilder ids = new StringBuilder();
            if (r.isValid()) for (LLResultTypes.FiducialResult f : r.getFiducialResults()) ids.append(f.getFiducialId()).append(' ');
            telemetry.addData("Tags seen", ids.length() == 0 ? "none" : ids.toString());
        }
        telemetry.addData("Target", tag == null ? "-" : String.format("ID %d  tx %.1f deg", tag.getFiducialId(), tag.getTargetXDegrees()));
        telemetry.addLine("Hold CROSS to auto-align");
        telemetry.update();
    }

    private double deadband(double v) {
        return Math.abs(v) < 0.08 ? 0 : v;
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
