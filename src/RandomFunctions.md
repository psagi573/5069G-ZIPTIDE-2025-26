void longdrive(double distInches, double headingDeg)
{
    distPID.reset();
    headingPID.reset();

    Profile profile(maxVelDefault, accelDefault);

    double startX = getPose().x;
    double startY = getPose().y;
    double target = distInches;

    double lastError = 0;
    int elapsed = 0;
    const int timeout = 3000;

    while (true)
    {
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        double error = target - traveled;

        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        double direction = (error >= 0) ? 1.0 : -1.0;

        if ((fabs(error) < 0.5 && fabs(error - lastError) < 0.1) || elapsed > timeout)
            break;

        // If using motion profile, apply velocity limit and clamp output
        // Motion profiling clamp only
        double velLimit = profile.getTargetVelocity(fabs(error), traveled, direction);
        linearOut = clamp(linearOut, -velLimit, velLimit);

        // Clamp linear output to ±1 before scaling volts for smooth control
        double turnOut = 0.0;
        double headingError = headingDeg - pose.theta;

        // Normalize heading error to [-180, 180]
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;

        if (fabs(headingError) > 5)
        {
            turnOut = headingPID.compute(headingError, 0);

            double headingScale = 0.02; // Tunable
            turnOut *= fabs(headingError) * headingScale;

            // Soft cap to avoid overcorrection
            turnOut = clamp(turnOut, -0.3, 0.3); // ~3.3V max correction
        }

        // Scale outputs from PID range [-1, 1] to volts [-11, 11]
        linearOut = linearOut * 11.0;
        turnOut = turnOut * 11.0;

        double leftVolt = linearOut - turnOut;
        double rightVolt = linearOut + turnOut;

        setDrive(leftVolt, rightVolt);

        // Exit conditions: close enough or timeout
        lastError = error;
        elapsed += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}

void moveTo(double targetX, double targetY, double targetTheta, bool turnAtEnd)
{
    distPID.reset();
    headingPID.reset();

    Pose start = getPose();
    double totalDist = hypot(targetX - start.x, targetY - start.y);

    // Choose profile based on distance
    double maxVelLocal = (totalDist < 12.0) ? maxVelShort : maxVelDefault;
    double accelLocal = (totalDist < 12.0) ? accelShort : accelDefault;
    Profile profile(maxVelLocal, accelLocal);

    double prevError = totalDist;
    double elapsedTime = 0;
    const double timeout = 2000;        // ms
    const double settleThreshold = 0.5; // inches
    const double slopeThreshold = 0.2;  // inches

    while (true)
    {
        Pose pose = getPose();

        double dx = targetX - pose.x;
        double dy = targetY - pose.y;

        double remainingDist = hypot(dx, dy);
        double traveled = totalDist - remainingDist;
        double error = totalDist - traveled;

        double direction = (error >= 0) ? 1.0 : -1.0;

        // Break condition: close and stable, or timeout
        double errorDelta = fabs(prevError - error);
        if ((remainingDist < settleThreshold && errorDelta < slopeThreshold) || elapsedTime > timeout)
        {
            break;
        }

        // Calculate linear PID and velocity profile limit
        double linearOut = distPID.compute(totalDist, traveled);
        double velLimit = profile.getTargetVelocity(fabs(error), traveled, direction);
        if (velLimit < 1.0)
            velLimit = 1.0;
        linearOut = clamp(linearOut, -velLimit, velLimit);

        // Kickstart: if output is tiny but still far, push minimum voltage
        if (fabs(linearOut) < 1.0 && fabs(error) > 1.0)
        {
            linearOut = direction * 1.0;
        }

        // Calculate heading and heading error normalized [-180,180]
        double desiredHeading = atan2(dy, dx) * (180.0 / M_PI);
        double headingError = desiredHeading - pose.theta;
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;

        // Heading PID — **NO minus sign here**, matches your turn()
        double turnOut = headingPID.compute(0, headingError);

        // Combine linear and turn outputs for left/right voltages
        double left = linearOut - turnOut;
        double right = linearOut + turnOut;

        setDrive(left, right);

        prevError = error;
        wait(10, msec);
        elapsedTime += 10;
    }

    setDrive(0, 0);

    // Optional final turn to targetTheta
    if (turnAtEnd)
    {
        turn(targetTheta);
    }
}

void sturn(double stargetHeading)
{
    fastTurnPID.reset();
    double elapsedTime = 0;
    const double timeout = 1000;

    double error = 0;
    double lastError = 0;
    double turnOutput = 0;

    while (true)
    {
        double heading = getPose().theta;
        error = stargetHeading - heading;

        // Wrap angle error [-180, 180]
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        // Only accumulate integral if error is small enough
        if (fabs(error) < 10)
        {
            fastTurnPID.integral += error;
        }

        // Use your PID class for final output
        fastTurnPID.prevError = lastError; // For derivative
        turnOutput = fastTurnPID.kP * error + fastTurnPID.kI * fastTurnPID.integral + fastTurnPID.kD * (error - lastError);

        // Clamp output between -1 and 1, then scale to volts
        if (turnOutput > 1)
            turnOutput = 1;
        if (turnOutput < -1)
            turnOutput = -1;

        double leftVolt = 11 * turnOutput;
        double rightVolt = -11 * turnOutput;

        setDrive(leftVolt, rightVolt);

        // Exit condition (same as old version)
        if (fabs(error) < 1.0 || elapsedTime >= timeout)
            break;

        lastError = error;
        elapsedTime += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}