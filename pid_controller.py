import numpy as np

class PIDController:
    """
    A PID (Proportional-Integral-Derivative) controller implementation.
    
    The PID controller computes a control signal based on the error between
    the desired setpoint and the actual output of the system.
    """
    
    def __init__(self, Kp, Ki, Kd, output_limits=None):
        """
        Initialize the PID controller.
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain  
            Kd (float): Derivative gain
            output_limits (tuple): Optional (min, max) output limits
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # Internal state variables
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_limits = output_limits
        
        # For debugging/analysis
        self.proportional_term = 0.0
        self.integral_term = 0.0
        self.derivative_term = 0.0
    
    def update(self, error, dt):
        """
        Update the PID controller with the current error.
        
        Args:
            error (float): Current error (setpoint - actual_output)
            dt (float): Time step since last update
            
        Returns:
            float: Control signal output
        """
        # Proportional term
        self.proportional_term = self.Kp * error
        
        # Integral term (accumulate error over time)
        self.integral += error * dt
        self.integral_term = self.Ki * self.integral
        
        # Derivative term (rate of change of error)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.derivative_term = self.Kd * derivative
        
        # Compute total output
        output = self.proportional_term + self.integral_term + self.derivative_term
        
        # Apply output limits if specified
        if self.output_limits:
            min_output, max_output = self.output_limits
            if output > max_output:
                output = max_output
                # Anti-windup: prevent integral from growing when output is saturated
                self.integral = (max_output - self.proportional_term - self.derivative_term) / self.Ki if self.Ki != 0 else 0
            elif output < min_output:
                output = min_output
                # Anti-windup: prevent integral from shrinking when output is saturated
                self.integral = (min_output - self.proportional_term - self.derivative_term) / self.Ki if self.Ki != 0 else 0
        
        # Store error for next derivative calculation
        self.previous_error = error
        
        return output
    
    def reset(self):
        """Reset the PID controller internal state."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.proportional_term = 0.0
        self.integral_term = 0.0
        self.derivative_term = 0.0
    
    def set_gains(self, Kp, Ki, Kd):
        """
        Update PID gains.
        
        Args:
            Kp (float): New proportional gain
            Ki (float): New integral gain
            Kd (float): New derivative gain
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def get_components(self):
        """
        Get the individual PID components from the last update.
        
        Returns:
            dict: Dictionary containing P, I, D components
        """
        return {
            'proportional': self.proportional_term,
            'integral': self.integral_term,
            'derivative': self.derivative_term
        }
    
    def set_output_limits(self, min_output, max_output):
        """
        Set output limits for the controller.
        
        Args:
            min_output (float): Minimum output value
            max_output (float): Maximum output value
        """
        self.output_limits = (min_output, max_output)
    
    def get_tuning_info(self):
        """
        Get information about current tuning parameters.
        
        Returns:
            dict: Dictionary containing tuning information
        """
        return {
            'Kp': self.Kp,
            'Ki': self.Ki,
            'Kd': self.Kd,
            'integral_state': self.integral,
            'previous_error': self.previous_error
        }
