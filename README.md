# 🎛️ PID Playground

An interactive Streamlit web application for simulating and visualizing PID (Proportional-Integral-Derivative) controllers on 1D systems.

## Features

### 🎮 Interactive Controls
- **PID Parameter Tuning**: Adjust Kp, Ki, and Kd gains with intuitive sliders
- **Preset Configurations**: Choose from Conservative, Aggressive, Oscillatory, and Sluggish presets
- **System Selection**: Test on First Order, Second Order, or Integrator systems
- **Real-time Simulation**: Run simulations with customizable setpoints and time duration

### 📊 Comprehensive Visualizations
- **System Response**: Compare actual output vs setpoint with performance metrics
- **Error Analysis**: Track error over time with visual indicators
- **Control Signal**: Monitor the PID controller output u(t)
- **Phase Portrait**: Visualize system dynamics (for second-order systems)
- **PID Components**: Break down Proportional, Integral, and Derivative contributions

### 🎯 Performance Metrics
- Steady-state error calculation
- Overshoot percentage
- Settling time analysis
- Real-time performance feedback

### 📚 Educational Features
- **Tooltips**: Detailed explanations of each PID parameter
- **Multiple System Types**: Learn how different systems respond to PID control
- **Visual Learning**: Understand PID behavior through comprehensive plots

## Installation

1. **Clone or download the project**:
   ```bash
   git clone <repository-url>
   cd PIDplayground
   ```

2. **Install required packages**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application**:
   ```bash
   streamlit run pid_playground.py
   ```

4. **Open your browser** and navigate to the URL shown in the terminal (typically `http://localhost:8501`)

## Usage

1. **Choose a preset** or use custom parameters
2. **Adjust PID gains** using the sliders in the sidebar
3. **Set your target setpoint** and simulation time
4. **Select system type** and configure system parameters
5. **Click "Run Simulation"** to see results
6. **Analyze the plots** to understand PID behavior

### PID Parameter Guidelines

- **Kp (Proportional)**: Controls response speed. Higher values = faster response but may cause overshoot
- **Ki (Integral)**: Eliminates steady-state error. Too high may cause oscillations
- **Kd (Derivative)**: Reduces overshoot and improves stability. Helps predict future error

### System Types

- **First Order**: Simple lag system (e.g., temperature control)
- **Second Order**: Mass-spring-damper system (e.g., motor position control)
- **Integrator**: Pure integrator (e.g., tank level control)

## Project Structure

```
PIDplayground/
│
├── pid_playground.py      # Main Streamlit application
├── pid_controller.py      # PID controller implementation
├── system_simulation.py   # System dynamics simulation
├── plotting_utils.py      # Visualization utilities
├── requirements.txt       # Python dependencies
└── README.md             # This file
```

## Technical Details

### PID Controller Implementation
- Standard PID algorithm with anti-windup protection
- Configurable output limits
- Component analysis for educational purposes

### System Simulation
- Numerical integration using Euler method
- Support for multiple system types
- Analytical step response calculation for comparison

### Plotting
- Professional matplotlib visualizations
- Interactive performance metrics
- Comprehensive analysis plots

## Educational Applications

This tool is perfect for:
- **Control Systems Education**: Visual understanding of PID behavior
- **Engineering Students**: Hands-on experience with control theory
- **Process Control Training**: Understanding real-world applications
- **Parameter Tuning Practice**: Learn optimal PID tuning strategies

## Contributing

Feel free to contribute improvements, additional system types, or educational features!

## Requirements

- Python 3.7+
- Streamlit 1.28.0+
- NumPy 1.24.0+
- Matplotlib 3.7.0+

## License

This project is open source and available under the MIT License.
