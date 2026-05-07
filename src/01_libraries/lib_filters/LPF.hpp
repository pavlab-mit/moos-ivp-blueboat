/*************************************************************
      File: lib_filters/LPF.hpp
     Brief:
        Header-only low-pass filter primitive (LPFSmoother).
        Tau-based first-order LPF with optional 360- and 180-
        degree wraparound update variants. Used by applets and
        any controller needing a simple time-constant smoother.
        (Note: BBNavigatorInterface defines its own simpler
        alpha-based LPF inline; the two are intentionally
        separate classes.)
*************************************************************/

#pragma once

class LPFSmoother
{
public:
  double alpha;
  double tau;
  double previousOutput;
  double nextOutput;

  double nextOutput_dot;
  double prev_dt;

public:
  LPFSmoother()
  {
    alpha = 0;
    tau = 0.5;
    previousOutput = 0;

    nextOutput = 0;
    nextOutput_dot = 0;

    prev_dt = 0.1;
  }

  LPFSmoother(double tau)
  {
    /*
      alpha = dt/(tau + dt)
      if we want a fin to reach 99% of the 1 seconds, consider five time constants, where tau = 0.2; if we sample at 50 hz, then dt = 0.02, and alpha = ~0.67
    */
    alpha = 0;
    this->tau = tau;
    previousOutput = 0;

    nextOutput = 0;
    nextOutput_dot = 0;

    prev_dt = 0.1;
  }

  void update(double desired_state, double dt)
  {
    const double epsilon = 1e-6;
    if (dt < epsilon)
      dt = epsilon;

    alpha = dt / (tau + dt);
    nextOutput = alpha * desired_state + (1 - alpha) * previousOutput;
    nextOutput_dot = (nextOutput - previousOutput) / dt;
    previousOutput = nextOutput;
    prev_dt = dt;
  }

  void update360(double desired_state, double dt)
  {
    double error = desired_state - previousOutput;

    if (error > 180)
    {
      desired_state -= 360;
    }
    else if (error < -180)
    {
      desired_state += 360;
    }

    // Call the core update logic after adjusting the desired state
    update(desired_state, dt);
  }

  void update180(double desired_state, double dt)
  {
    double error = desired_state - previousOutput;

    // Handle wraparound at ±180
    if (error > 180)
    {
      desired_state -= 360;
    }
    else if (error < -180)
    {
      desired_state += 360;
    }

    // Call the core update logic
    update(desired_state, dt);

    // Ensure output remains within [-180, 180]
    if (nextOutput > 180)
    {
      nextOutput -= 360;
    }
    else if (nextOutput < -180)
    {
      nextOutput += 360;
    }
  }

  double getNextState()
  {
    return nextOutput;
  }

  double getNextStateDot()
  {
    return nextOutput_dot;
  }

  void reset(double setState = 0.0)
  {
    previousOutput = setState;
    nextOutput = setState;
    nextOutput_dot = 0.0;
  }
};
