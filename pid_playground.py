import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from pid_controller import PIDController
from system_simulation import SystemSimulation
from plotting_utils import create_plots

# Configure Streamlit page
st.set_page_config(
    page_title="PID Playground",
    page_icon="🎛️",
    layout="wide"
)

# Custom CSS for better styling
st.markdown("""
<style>
    .stApp > header {
        background-color: transparent;
    }
    
    .stApp {
        margin-top: -80px;
    }
    
    .main-header {
        font-size: 3rem;
        font-weight: bold;
        text-align: center;
        color: #1f77b4;
        margin-bottom: 2rem;
    }
    
    .section-header {
        font-size: 1.5rem;
        font-weight: bold;
        color: #2c3e50;
        margin-top: 2rem;
        margin-bottom: 1rem;
    }
    
    .tooltip {
        background-color: #f0f2f6;
        padding: 10px;
        border-radius: 5px;
        margin-bottom: 10px;
        border-left: 4px solid #1f77b4;
    }
</style>
""", unsafe_allow_html=True)

def main():
    # Main header
    st.markdown('<h1 class="main-header">🎛️ PID Playground</h1>', unsafe_allow_html=True)
    st.markdown("**Interactive PID Controller Simulation for 1D Systems**")
    
    # Sidebar for controls
    st.sidebar.header("🎮 Control Parameters")
    
    # Preset configurations
    st.sidebar.subheader("📋 Preset Configurations")
    preset = st.sidebar.selectbox(
        "Choose a preset:",
        ["Custom", "Conservative", "Aggressive", "Oscillatory", "Sluggish"]
    )
    
    # Define presets
    presets = {
        "Conservative": {"Kp": 1.0, "Ki": 0.1, "Kd": 0.05, "setpoint": 10.0, "sim_time": 20.0},
        "Aggressive": {"Kp": 3.0, "Ki": 1.0, "Kd": 0.2, "setpoint": 10.0, "sim_time": 20.0},
        "Oscillatory": {"Kp": 4.0, "Ki": 2.0, "Kd": 0.0, "setpoint": 10.0, "sim_time": 20.0},
        "Sluggish": {"Kp": 0.5, "Ki": 0.02, "Kd": 0.1, "setpoint": 10.0, "sim_time": 30.0}
    }
    
    # Set default values based on preset
    if preset != "Custom":
        default_values = presets[preset]
    else:
        default_values = {"Kp": 1.0, "Ki": 0.1, "Kd": 0.05, "setpoint": 10.0, "sim_time": 20.0}
    
    st.sidebar.subheader("⚙️ PID Parameters")
    
    # PID parameter inputs with tooltips
    st.sidebar.markdown("""
    <div class="tooltip">
    <b>Kp (Proportional Gain)</b><br>
    Controls how aggressively the controller responds to current error. 
    Higher values = faster response but may cause overshoot.
    </div>
    """, unsafe_allow_html=True)
    
    Kp = st.sidebar.slider("Kp - Proportional Gain", 0.0, 10.0, default_values["Kp"], 0.1)
    
    st.sidebar.markdown("""
    <div class="tooltip">
    <b>Ki (Integral Gain)</b><br>
    Eliminates steady-state error by accumulating past errors. 
    Too high may cause oscillations and instability.
    </div>
    """, unsafe_allow_html=True)
    
    Ki = st.sidebar.slider("Ki - Integral Gain", 0.0, 5.0, default_values["Ki"], 0.01)
    
    st.sidebar.markdown("""
    <div class="tooltip">
    <b>Kd (Derivative Gain)</b><br>
    Predicts future error based on rate of change. 
    Reduces overshoot and improves stability.
    </div>
    """, unsafe_allow_html=True)
    
    Kd = st.sidebar.slider("Kd - Derivative Gain", 0.0, 2.0, default_values["Kd"], 0.01)
    
    st.sidebar.subheader("🎯 Simulation Settings")
    
    setpoint = st.sidebar.number_input("Target Setpoint", value=default_values["setpoint"], min_value=0.0, max_value=100.0)
    sim_time = st.sidebar.number_input("Simulation Time (seconds)", value=default_values["sim_time"], min_value=1.0, max_value=100.0)
    
    # System parameters
    st.sidebar.subheader("🔧 System Parameters")
    system_type = st.sidebar.selectbox("System Type", ["Second Order", "First Order", "Integrator"])
    
    if system_type == "Second Order":
        damping = st.sidebar.slider("Damping Ratio", 0.1, 2.0, 0.5, 0.1)
        natural_freq = st.sidebar.slider("Natural Frequency (rad/s)", 0.1, 10.0, 1.0, 0.1)
        system_params = {"damping": damping, "natural_freq": natural_freq}
    elif system_type == "First Order":
        time_constant = st.sidebar.slider("Time Constant", 0.1, 10.0, 1.0, 0.1)
        system_params = {"time_constant": time_constant}
    else:  # Integrator
        gain = st.sidebar.slider("System Gain", 0.1, 5.0, 1.0, 0.1)
        system_params = {"gain": gain}
    
    # Run simulation button
    if st.sidebar.button("🚀 Run Simulation", type="primary"):
        with st.spinner("Running simulation..."):
            # Create PID controller
            pid = PIDController(Kp, Ki, Kd)
            
            # Create system simulation
            system_sim = SystemSimulation(system_type, system_params)
            
            # Run simulation
            dt = 0.01  # Time step
            time_steps = int(sim_time / dt)
            
            time_array = np.linspace(0, sim_time, time_steps)
            setpoint_array = np.full(time_steps, setpoint)
            
            # Initialize arrays
            output = np.zeros(time_steps)
            control_signal = np.zeros(time_steps)
            error = np.zeros(time_steps)
            
            # Initial conditions
            output[0] = 0.0
            
            # Simulation loop
            for i in range(1, time_steps):
                # Calculate error
                error[i-1] = setpoint - output[i-1]
                
                # Calculate control signal
                control_signal[i-1] = pid.update(error[i-1], dt)
                
                # Update system
                output[i] = system_sim.update(control_signal[i-1], dt)
            
            # Calculate final error
            error[-1] = setpoint - output[-1]
            control_signal[-1] = control_signal[-2]  # Hold last control signal
            
            # Store results in session state
            st.session_state['simulation_data'] = {
                'time': time_array,
                'setpoint': setpoint_array,
                'output': output,
                'control_signal': control_signal,
                'error': error,
                'pid_params': {'Kp': Kp, 'Ki': Ki, 'Kd': Kd},
                'system_type': system_type,
                'system_params': system_params
            }
        
        st.success("Simulation completed successfully!")
    
    # Display results if simulation has been run
    if 'simulation_data' in st.session_state:
        data = st.session_state['simulation_data']
        
        # Display current parameters
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("Kp", f"{data['pid_params']['Kp']:.2f}")
        with col2:
            st.metric("Ki", f"{data['pid_params']['Ki']:.2f}")
        with col3:
            st.metric("Kd", f"{data['pid_params']['Kd']:.2f}")
        
        # Calculate performance metrics
        steady_state_error = abs(data['error'][-1])
        overshoot = max(0, (np.max(data['output']) - data['setpoint'][0]) / data['setpoint'][0] * 100)
        settling_time = calculate_settling_time(data['output'], data['setpoint'][0], data['time'])
        
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("Steady State Error", f"{steady_state_error:.3f}")
        with col2:
            st.metric("Overshoot (%)", f"{overshoot:.1f}")
        with col3:
            st.metric("Settling Time (s)", f"{settling_time:.1f}")
        
        # Create and display plots
        fig = create_plots(data)
        st.pyplot(fig)
        
        # Display system information
        st.subheader("📊 System Information")
        st.write(f"**System Type:** {data['system_type']}")
        st.write(f"**System Parameters:** {data['system_params']}")

def calculate_settling_time(output, setpoint, time_array, tolerance=0.02):
    """Calculate settling time (time to reach within 2% of setpoint)"""
    target_range = setpoint * tolerance
    
    # Find the last time the output was outside the settling range
    settling_mask = np.abs(output - setpoint) > target_range
    
    if not np.any(settling_mask):
        return 0.0
    
    last_violation_idx = np.where(settling_mask)[0][-1]
    
    if last_violation_idx >= len(time_array) - 1:
        return time_array[-1]
    
    return time_array[last_violation_idx + 1]

if __name__ == "__main__":
    main()
