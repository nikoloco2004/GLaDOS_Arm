# Physical calibration checklist

Use this after the software stack is running. **Do not skip mechanical range checks** — servo limits in code are validated for *your* build but wiring and assembly can differ.

## Already fixed in software (measurements you provided)

- Link lengths: shoulder→elbow **130 mm**, elbow→wrist **120 mm**.
- Servo limits and neutral pose (see `glados_arm/config.py` and Arduino sketch).
- Joint order: Servo 1 wrist, 2 elbow, 3 base, 4 shoulder.
- Elbow inversion handled in **`mapping.py`** (`ELBOW_INVERT`, signs, `ELBOW_RAD_TO_SERVO_DEG`).

## Must calibrate on the bench

### 1. I2C and PCA9685 wiring

- Uno R4 **SDA/SCL** to PCA9685; **GND** common; servos powered from the PCA9685 **V+** rail (adequate current).
- Default address **0x40**; if the board uses another address, set `PCA9685_I2C_ADDR` in the sketch.
- Use `I2C_SCAN` over serial to confirm the device appears.
- **PWM tick calibration:** firmware maps each clamped degree with `map(deg, 0, 270, PWM_TICK_MIN, PWM_TICK_MAX)` (defaults 102–512). Adjust `PWM_TICK_MIN` / `PWM_TICK_MAX` in the sketch if needed. Use `SET_PWM <ch> <ticks>` for raw tests. Degree limits must stay aligned with `glados_arm/config.py`.

### 2. Mapping direction and scale

For each joint, from **neutral**:

- **Base**: increasing servo command → confirm left vs right matches `BASE_YAW_SIGN` and `BASE_RAD_TO_SERVO_DEG`.
- **Shoulder**: confirm positive `q_shoulder` matches expected “up/down” in the vertical plane; flip `SHOULDER_SIGN` if needed.
- **Elbow**: confirm “physical up” decreases servo from 270 as expected; tune `ELBOW_SIGN` / `ELBOW_RAD_TO_SERVO_DEG` (linear model is a first pass).
- **Wrist**: trim only in v1; confirm `WRIST_SIGN` and scale.

### 3. FK frame references (`THETA1_REF_NEUTRAL_RAD`, `THETA2_REF_NEUTRAL_RAD`)

At **neutral** pose, measure tip position in the vertical plane (roughly: forward and up from shoulder pivot). Adjust `THETA1_REF_NEUTRAL_RAD` / `THETA2_REF_NEUTRAL_RAD` so `python -m glados_arm.main fk --shoulder-deg 0 --elbow-deg 0` matches your measurement **or** document the offset as a constant error — the IK uses the same references, so consistency matters more than absolute perfection.

### 4. Elbow-up vs elbow-down branch

`prefer=elbow_up` vs `elbow_down` chooses the second IK solution. Pick the branch that avoids singularities and matches your mechanical stops.

### 5. Vision → angles (not included in v1)

You still need a **camera–base** calibration:

- Horizontal pixel error → **angular error on base** (not raw servo ticks).
- Vertical pixel error → **elevation or plane target** for the vertical chain.

Keep that in a separate `vision` module so the arm stack stays geometric.

### 6. Potato / end effector

Link lengths are pivot-to-pivot. The potato’s center of mass is offset from the wrist pivot; for “believable” pointing you may later add a fixed offset in the plane or extend FK with a short third link.
