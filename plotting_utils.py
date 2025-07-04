import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle

def create_plots(simulation_data):
    """
    Create comprehensive plots for PID simulation results.
    
    Args:
        simulation_data (dict): Dictionary containing simulation results
        
    Returns:
        matplotlib.figure.Figure: Figure with all plots
    """
    # Set up the plotting style
    plt.style.use('default')
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 12))
    gs = gridspec.GridSpec(3, 2, hspace=0.3, wspace=0.3, figure=fig)
    
    # Extract data
    time = simulation_data['time']
    setpoint = simulation_data['setpoint']
    output = simulation_data['output']
    control_signal = simulation_data['control_signal']
    error = simulation_data['error']
    
    # Color scheme
    colors = {
        'setpoint': '#e74c3c',
        'output': '#3498db',
        'control': '#2ecc71',
        'error': '#f39c12',
        'background': '#ecf0f1'
    }
    
    # Plot 1: System Response
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(time, setpoint, '--', color=colors['setpoint'], linewidth=2, label='Setpoint', alpha=0.8)
    ax1.plot(time, output, color=colors['output'], linewidth=2, label='System Output')
    ax1.fill_between(time, setpoint, output, alpha=0.2, color=colors['background'])
    ax1.set_title('System Response vs Setpoint', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Output')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Add performance annotations
    steady_state_error = abs(error[-1])
    overshoot = max(0, (np.max(output) - setpoint[0]) / setpoint[0] * 100)
    
    # Add text box with performance metrics
    textstr = f'Steady State Error: {steady_state_error:.3f}\nOvershoot: {overshoot:.1f}%'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax1.text(0.02, 0.98, textstr, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', bbox=props)
    
    # Plot 2: Error vs Time
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(time, error, color=colors['error'], linewidth=2, label='Error')
    ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    ax2.fill_between(time, 0, error, alpha=0.3, color=colors['error'])
    ax2.set_title('Error vs Time', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (Setpoint - Output)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Plot 3: Control Signal
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(time, control_signal, color=colors['control'], linewidth=2, label='Control Signal u(t)')
    ax3.set_title('Control Signal vs Time', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control Signal')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Plot 4: Phase Portrait (for second-order systems)
    ax4 = fig.add_subplot(gs[2, 0])
    if simulation_data['system_type'] == 'Second Order':
        # Calculate derivative numerically
        output_dot = np.gradient(output, time)
        ax4.plot(output, output_dot, color=colors['output'], linewidth=2, alpha=0.7)
        ax4.plot(output[0], output_dot[0], 'go', markersize=8, label='Start')
        ax4.plot(output[-1], output_dot[-1], 'rs', markersize=8, label='End')
        ax4.set_title('Phase Portrait (Output vs Output Rate)', fontsize=14, fontweight='bold')
        ax4.set_xlabel('Output')
        ax4.set_ylabel('Output Rate (dy/dt)')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
    else:
        # For other systems, show error histogram
        ax4.hist(error, bins=30, color=colors['error'], alpha=0.7, edgecolor='black')
        ax4.set_title('Error Distribution', fontsize=14, fontweight='bold')
        ax4.set_xlabel('Error')
        ax4.set_ylabel('Frequency')
        ax4.grid(True, alpha=0.3)
    
    # Plot 5: PID Components Analysis
    ax5 = fig.add_subplot(gs[2, 1])
    
    # Calculate PID components (simplified reconstruction)
    pid_params = simulation_data['pid_params']
    Kp, Ki, Kd = pid_params['Kp'], pid_params['Ki'], pid_params['Kd']
    
    # Proportional component
    P_component = Kp * error
    
    # Integral component (cumulative)
    dt = time[1] - time[0]
    I_component = Ki * np.cumsum(error) * dt
    
    # Derivative component
    D_component = Kd * np.gradient(error, time)
    
    ax5.plot(time, P_component, label=f'P term (Kp={Kp:.2f})', alpha=0.8)
    ax5.plot(time, I_component, label=f'I term (Ki={Ki:.2f})', alpha=0.8)
    ax5.plot(time, D_component, label=f'D term (Kd={Kd:.2f})', alpha=0.8)
    ax5.plot(time, control_signal, '--', color='black', label='Total Control', linewidth=2)
    
    ax5.set_title('PID Components', fontsize=14, fontweight='bold')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Component Value')
    ax5.grid(True, alpha=0.3)
    ax5.legend()
    
    # Overall figure title
    fig.suptitle(f'PID Controller Analysis - {simulation_data["system_type"]} System', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    return fig

def create_comparison_plot(simulation_results_list, labels):
    """
    Create a comparison plot for multiple simulation runs.
    
    Args:
        simulation_results_list (list): List of simulation data dictionaries
        labels (list): List of labels for each simulation
        
    Returns:
        matplotlib.figure.Figure: Comparison figure
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('PID Controller Comparison', fontsize=16, fontweight='bold')
    
    colors = plt.cm.Set1(np.linspace(0, 1, len(simulation_results_list)))
    
    for i, (data, label) in enumerate(zip(simulation_results_list, labels)):
        color = colors[i]
        time = data['time']
        setpoint = data['setpoint']
        output = data['output']
        control_signal = data['control_signal']
        error = data['error']
        
        # System response
        axes[0, 0].plot(time, output, color=color, linewidth=2, label=label)
        
        # Error
        axes[0, 1].plot(time, error, color=color, linewidth=2, label=label)
        
        # Control signal
        axes[1, 0].plot(time, control_signal, color=color, linewidth=2, label=label)
        
        # Performance metrics
        steady_state_error = abs(error[-1])
        overshoot = max(0, (np.max(output) - setpoint[0]) / setpoint[0] * 100)
        axes[1, 1].scatter(steady_state_error, overshoot, color=color, s=100, label=label)
    
    # Add setpoint reference to first plot
    if simulation_results_list:
        axes[0, 0].plot(simulation_results_list[0]['time'], 
                       simulation_results_list[0]['setpoint'], 
                       '--', color='red', linewidth=2, label='Setpoint')
    
    # Configure subplots
    axes[0, 0].set_title('System Response')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Output')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    axes[0, 1].set_title('Error')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Error')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    axes[1, 0].set_title('Control Signal')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Control Signal')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    axes[1, 1].set_title('Performance Comparison')
    axes[1, 1].set_xlabel('Steady State Error')
    axes[1, 1].set_ylabel('Overshoot (%)')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    
    plt.tight_layout()
    return fig

def create_step_response_plot(system_sim, time_array, step_magnitude=1.0):
    """
    Create a step response plot for system analysis.
    
    Args:
        system_sim: SystemSimulation object
        time_array: Time array for simulation
        step_magnitude: Magnitude of step input
        
    Returns:
        matplotlib.figure.Figure: Step response figure
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Calculate analytical step response
    analytical_response = system_sim.step_response_analytical(time_array, step_magnitude)
    
    # Plot
    ax.plot(time_array, analytical_response, 'b-', linewidth=2, label='Step Response')
    ax.axhline(y=step_magnitude, color='r', linestyle='--', alpha=0.7, label='Input Step')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Output')
    ax.set_title(f'Step Response - {system_sim.system_type} System')
    ax.legend()
    
    return fig

def apply_plot_styling():
    """Apply consistent styling to matplotlib plots."""
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 12,
        'axes.titlesize': 14,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.titlesize': 16,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'lines.linewidth': 2,
        'axes.axisbelow': True
    })
