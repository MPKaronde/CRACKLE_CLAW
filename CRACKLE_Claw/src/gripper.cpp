#include "gripper.h"

static const int MIN_ANGLE       = 0;
static const int MAX_ANGLE       = 180;
static const int TRAVEL_MS       = 700;
static const int GRIP_STEP_MS    = 10;    // delay between steps when closing
static const int GRIP_STEP_DEG  = 1.5;     // degrees moved per step when closing
static const int ZERO_SAMPLES    = 30;    // ~3s at HX711 default 10Hz
static const float FORCE_THRESH  = 500.0; // tune to your load cell units
static const float SLIP_RATIO    = 0.80;  // re-grip if reading drops below 80% of peak
static const float EMA_ALPHA     = 0.2f;  // smoothing factor (lower = smoother, slower)
static const int   CONFIRM_COUNT = 1;     // consecutive smoothed readings above threshold to confirm contact

Gripper::Gripper(int left_servo, int right_servo,
                 int left_dt, int left_scl,
                 int right_dt, int right_scl)
    : LEFT_SERVO_PIN(left_servo), RIGHT_SERVO_PIN(right_servo),
      DT_LEFT(left_dt), SCL_LEFT(left_scl),
      DT_RIGHT(right_dt), SCL_RIGHT(right_scl),
      left_cell_offset(0.0f), right_cell_offset(0.0f),
      left_angle(MIN_ANGLE), right_angle(MIN_ANGLE)
{
    this->left_servo.attach(LEFT_SERVO_PIN, 500, 2500);
    this->right_servo.attach(RIGHT_SERVO_PIN, 500, 2500);
    left_cell.begin(DT_LEFT, SCL_LEFT);
    right_cell.begin(DT_RIGHT, SCL_RIGHT);
}

void Gripper::zero_servos()
{
    left_servo.write(MIN_ANGLE);
    right_servo.write(MIN_ANGLE);
    left_angle = MIN_ANGLE;
    right_angle = MIN_ANGLE;
    delay(TRAVEL_MS);
}

void Gripper::zero_loadcells()
{
    float left_sum = 0.0f, right_sum = 0.0f;
    int count = 0;

    while (count < ZERO_SAMPLES) {
        if (left_cell.is_ready() && right_cell.is_ready()) {
            left_sum  += left_cell.get_units();
            right_sum += right_cell.get_units();
            count++;
        }
    }

    left_cell_offset  = left_sum  / ZERO_SAMPLES;
    right_cell_offset = right_sum / ZERO_SAMPLES;
}

float Gripper::get_loadcell_reading(Side side)
{
    if (side == LEFT) {
        return left_cell.is_ready()
            ? left_cell.get_units() - left_cell_offset
            : 0.0f;
    } else {
        return right_cell.is_ready()
            ? right_cell.get_units() - right_cell_offset
            : 0.0f;
    }
}

void Gripper::move_servo(Side side, int angle)
{
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
    if (side == LEFT) {
        left_servo.write(angle);
        left_angle = angle;
    } else {
        right_servo.write(angle);
        right_angle = angle;
    }
}

void Gripper::grip_object()
{
    float ema             = get_loadcell_reading(RIGHT); // seed with first reading
    float peak_right      = 0.0f;                        // tracks most negative reading seen
    bool  contacted       = false;
    int   confirms        = 0;
    int   raw_contact_angle = -1; // angle where raw reading first spiked negative

    // Close incrementally from current position — caller should call release() first if starting fresh
    for (int angle = left_angle; angle <= MAX_ANGLE && !contacted; angle += GRIP_STEP_DEG) {
        move_servo(LEFT,  angle);
        move_servo(RIGHT, angle);
        delay(GRIP_STEP_MS);

        float raw = get_loadcell_reading(RIGHT);
        ema = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema;

        // Contact = reading goes sufficiently negative; track earliest raw crossing
        if (raw < -FORCE_THRESH && raw_contact_angle < 0)
            raw_contact_angle = angle;
        else if (raw >= -FORCE_THRESH)
            raw_contact_angle = -1; // dropped back — was a spike, reset

        // EMA confirms contact is real, not transient noise
        if (ema < -FORCE_THRESH) {
            if (++confirms >= CONFIRM_COUNT) {
                contacted = true;
                peak_right = ema;
            }
        } else {
            confirms = 0;
        }
    }

    if (!contacted) return;

    // Return to where raw reading first went negative — actual contact point, not lagged EMA point
    int contact_angle = (raw_contact_angle >= 0) ? raw_contact_angle : left_angle;
    contact_angle = constrain(contact_angle, MIN_ANGLE, MAX_ANGLE);
    move_servo(LEFT,  contact_angle);
    move_servo(RIGHT, contact_angle);
    delay(TRAVEL_MS);

    // Slip monitoring: reading becoming less negative means losing grip
    for (int i = 0; i < 20; i++) {
        ema = EMA_ALPHA * get_loadcell_reading(RIGHT) + (1.0f - EMA_ALPHA) * ema;

        // peak_right is negative; slip if ema rises above SLIP_RATIO of peak (less force)
        if (ema > peak_right * SLIP_RATIO) {
            move_servo(LEFT,  left_angle  + 1);
            move_servo(RIGHT, right_angle + 1);
        }

        if (ema < peak_right) peak_right = ema; // track most negative

        delay(50);
    }
}

void Gripper::release()
{
    left_servo.write(MIN_ANGLE);
    right_servo.write(MIN_ANGLE);
    left_angle  = MIN_ANGLE;
    right_angle = MIN_ANGLE;
    delay(TRAVEL_MS);
}
