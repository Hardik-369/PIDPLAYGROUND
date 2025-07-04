import numpy as np

class SystemSimulation:
    """
    Simulates different types of 1D systems for PID control testing.
    
    Supports:
    - First Order System: τ*dy/dt + y = K*u
    - Second Order System: d²y/dt² + 2*ζ*ωn*dy/dt + ωn²*y = ωn²*u  
    - Integrator System: dy/dt = K*u
    """
    
    def __init__(self, system_type, params):
        """
        Initialize the system simulation.
        
        Args:
            system_type (str): Type of system ("First Order", "Second Order", "Integrator")
            params (dict): System parameters based on type
        """
        self.system_type = system_type
        self.params = params
        
        # System state variables
        self.output = 0.0
        self.output_dot = 0.0  # First derivative (velocity)
        self.previous_input = 0.0
        
        # Initialize system based on type
        self._initialize_system()
    
    def _initialize_system(self):
        """Initialize system-specific parameters."""
        if self.system_type == "First Order":
            # τ*dy/dt + y = K*u
            self.time_constant = self.params.get('time_constant', 1.0)
            self.gain = self.params.get('gain', 1.0)
            
        elif self.system_type == "Second Order":
            # d²y/dt² + 2*ζ*ωn*dy/dt + ωn²*y = ωn²*u
            self.damping = self.params.get('damping', 0.5)
            self.natural_freq = self.params.get('natural_freq', 1.0)
            self.gain = self.params.get('gain', 1.0)
            
        elif self.system_type == "Integrator":
            # dy/dt = K*u
            self.gain = self.params.get('gain', 1.0)
            
        else:
            raise ValueError(f"Unknown system type: {self.system_type}")
    
    def update(self, input_signal, dt):
        """
        Update the system with a new input signal.
        
        Args:
            input_signal (float): Control input to the system
            dt (float): Time step
            
        Returns:
            float: System output
        """
        if self.system_type == "First Order":
            return self._update_first_order(input_signal, dt)
        elif self.system_type == "Second Order":
            return self._update_second_order(input_signal, dt)
        elif self.system_type == "Integrator":
            return self._update_integrator(input_signal, dt)
    
    def _update_first_order(self, input_signal, dt):
        """
        Update first-order system: τ*dy/dt + y = K*u
        Using Euler integration: y(k+1) = y(k) + dt * (K*u - y(k))/τ
        """
        # dy/dt = (K*u - y)/τ
        dydt = (self.gain * input_signal - self.output) / self.time_constant
        
        # Euler integration
        self.output += dydt * dt
        
        return self.output
    
    def _update_second_order(self, input_signal, dt):
        """
        Update second-order system: d²y/dt² + 2*ζ*ωn*dy/dt + ωn²*y = ωn²*u
        Using state-space representation with Euler integration
        """
        # State variables: x1 = y, x2 = dy/dt
        # dx1/dt = x2
        # dx2/dt = -ωn²*x1 - 2*ζ*ωn*x2 + ωn²*u
        
        wn = self.natural_freq
        zeta = self.damping
        
        # Calculate derivatives
        dx1dt = self.output_dot
        dx2dt = -wn*wn*self.output - 2*zeta*wn*self.output_dot + wn*wn*self.gain*input_signal
        
        # Euler integration
        self.output += dx1dt * dt
        self.output_dot += dx2dt * dt
        
        return self.output
    
    def _update_integrator(self, input_signal, dt):
        """
        Update integrator system: dy/dt = K*u
        Using Euler integration: y(k+1) = y(k) + dt * K*u
        """
        # dy/dt = K*u
        dydt = self.gain * input_signal
        
        # Euler integration
        self.output += dydt * dt
        
        return self.output
    
    def reset(self):
        """Reset the system to initial conditions."""
        self.output = 0.0
        self.output_dot = 0.0
        self.previous_input = 0.0
    
    def set_initial_conditions(self, initial_output=0.0, initial_output_dot=0.0):
        """
        Set initial conditions for the system.
        
        Args:
            initial_output (float): Initial output value
            initial_output_dot (float): Initial output derivative (for second-order systems)
        """
        self.output = initial_output
        self.output_dot = initial_output_dot
    
    def get_system_info(self):
        """
        Get information about the current system.
        
        Returns:
            dict: System information
        """
        info = {
            'type': self.system_type,
            'current_output': self.output,
            'parameters': self.params
        }
        
        if self.system_type == "Second Order":
            info['current_output_dot'] = self.output_dot
            
        return info
    
    def get_transfer_function_info(self):
        """
        Get transfer function information for the system.
        
        Returns:
            dict: Transfer function coefficients and description
        """
        if self.system_type == "First Order":
            # G(s) = K / (τ*s + 1)
            return {
                'numerator': [self.gain],
                'denominator': [self.time_constant, 1],
                'description': f"G(s) = {self.gain} / ({self.time_constant}*s + 1)"
            }
            
        elif self.system_type == "Second Order":
            # G(s) = K*ωn² / (s² + 2*ζ*ωn*s + ωn²)
            wn = self.natural_freq
            zeta = self.damping
            return {
                'numerator': [self.gain * wn * wn],
                'denominator': [1, 2*zeta*wn, wn*wn],
                'description': f"G(s) = {self.gain*wn*wn:.2f} / (s² + {2*zeta*wn:.2f}*s + {wn*wn:.2f})"
            }
            
        elif self.system_type == "Integrator":
            # G(s) = K / s
            return {
                'numerator': [self.gain],
                'denominator': [1, 0],
                'description': f"G(s) = {self.gain} / s"
            }
    
    def step_response_analytical(self, time_array, step_magnitude=1.0):
        """
        Calculate analytical step response for comparison.
        
        Args:
            time_array (numpy.ndarray): Time points
            step_magnitude (float): Magnitude of step input
            
        Returns:
            numpy.ndarray: Analytical step response
        """
        if self.system_type == "First Order":
            # y(t) = K * step_magnitude * (1 - e^(-t/τ))
            return self.gain * step_magnitude * (1 - np.exp(-time_array / self.time_constant))
            
        elif self.system_type == "Second Order":
            wn = self.natural_freq
            zeta = self.damping
            
            if zeta < 1.0:  # Underdamped
                wd = wn * np.sqrt(1 - zeta*zeta)
                response = self.gain * step_magnitude * (1 - np.exp(-zeta*wn*time_array) * 
                          (np.cos(wd*time_array) + (zeta*wn/wd)*np.sin(wd*time_array)))
            elif zeta == 1.0:  # Critically damped
                response = self.gain * step_magnitude * (1 - np.exp(-wn*time_array) * (1 + wn*time_array))
            else:  # Overdamped
                r1 = -wn * (zeta + np.sqrt(zeta*zeta - 1))
                r2 = -wn * (zeta - np.sqrt(zeta*zeta - 1))
                response = self.gain * step_magnitude * (1 - (r2*np.exp(r1*time_array) - r1*np.exp(r2*time_array))/(r2-r1))
            
            return response
            
        elif self.system_type == "Integrator":
            # y(t) = K * step_magnitude * t
            return self.gain * step_magnitude * time_array
            
        return np.zeros_like(time_array)
