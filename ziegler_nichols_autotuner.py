#!/usr/bin/env python3
"""
Ziegler-Nichols PID Auto-Tuning Library

This library implements the Ziegler-Nichols closed-loop method for PID tuning:
1. Increase Kp until system oscillates with constant amplitude (finding ultimate gain Ku)
2. Measure the oscillation period (Pu)
3. Calculate PID parameters based on Ziegler-Nichols rules

Compatible with simple-pid and other PID libraries.
"""

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID


def _clear_screen():
    """Clear the terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')


def _move_cursor_home():
    """Move cursor to home position without clearing"""
    if os.name == 'nt':
        os.system('cls')
    else:
        sys.stdout.write('\033[H')
        sys.stdout.flush()


class ZieglerNicholsAutotuner:
    """
    Implements the Ziegler-Nichols closed-loop method for PID tuning using simple-pid
    
    This class provides automated tuning of PID controllers by finding the ultimate
    gain (Ku) and oscillation period (Pu), then calculating optimal PID parameters
    using various Ziegler-Nichols tuning rules.
    """
    
    def __init__(self, sensor_func, actuator_func, setpoint, 
                 min_output=0, max_output=4095,
                 start_kp=0.1, kp_step=0.1, kp_change_interval_pct=0.25,
                 sample_time=0.5, window_size=100,
                 oscillation_threshold=0.05, min_oscillations=4,
                 max_test_time=300, safety_limits=None,
                 dual_actuator=False, set_remote_kp_func=None):
        """
        Initialize the Ziegler-Nichols autotuner
        
        Args:
            sensor_func (callable): Function that reads the process variable.
                                   Should return a float value.
                                   Example: lambda: sensor.read_temperature()
            
            actuator_func (callable): Function that controls the actuator.
                                     Takes one argument (output value).
                                     Example: lambda x: heater.set_power(x)
            
            setpoint (float): Desired process variable value
            
            min_output (int|float): Minimum actuator output value
                                   For unidirectional control: typically 0
                                   For bidirectional control: typically -max_output
            
            max_output (int|float): Maximum actuator output value
                                   Example: 4095 for 12-bit DAC, 255 for 8-bit PWM
            
            start_kp (float): Initial proportional gain to start testing
                             Default: 0.1
            
            kp_step (float): Step size for incrementally increasing Kp
                            Default: 0.1
            
            kp_change_interval_pct (float): Percentage of max_test_time to wait between Kp changes
                                           Example: 0.25 means Kp changes every 0.25% of max_test_time
                                           If max_test_time=300s, Kp changes every 0.75 seconds
                                           Default: 0.25
            
            sample_time (float): Time between control loop iterations in seconds
                                Default: 0.5
            
            window_size (int): Number of samples to analyze for oscillation detection
                              Default: 100
            
            oscillation_threshold (float): Relative threshold for detecting oscillations
                                          (as fraction of setpoint or mean value)
                                          Default: 0.05 (5%)
            
            min_oscillations (int): Minimum number of oscillation cycles required
                                   Default: 4
            
            max_test_time (float): Maximum test duration in seconds
                                  Default: 300 (5 minutes)
            
            safety_limits (tuple): Optional (min, max) safety limits for process variable
                                  If PV goes outside these limits, test stops
                                  Example: (15.0, 35.0) for temperature in °C
                                  Default: None
            
            dual_actuator (bool): Whether using bidirectional control
                                 (e.g., heater + cooler, or motor forward/reverse)
                                 Default: False
            
            set_remote_kp_func (callable): Optional function to set Kp on remote device.
                                          Takes one argument (Kp value).
                                          Use this when the device has internal PID control.
                                          Example: lambda kp: device.set_pid_kp(kp)
                                          Default: None (disabled)
        
        Example:
            >>> # Simple temperature control with heater only
            >>> autotuner = ZieglerNicholsAutotuner(
            ...     sensor_func=lambda: sensor.read_temperature(),
            ...     actuator_func=lambda x: heater.set_power(x),
            ...     setpoint=25.0,
            ...     min_output=0,
            ...     max_output=4095
            ... )
            
            >>> # Bidirectional temperature control with heater and fan
            >>> autotuner = ZieglerNicholsAutotuner(
            ...     sensor_func=lambda: sensor.read_temperature(),
            ...     actuator_func=lambda x: set_dual_actuator(x),
            ...     setpoint=25.0,
            ...     min_output=-4095,
            ...     max_output=4095,
            ...     dual_actuator=True
            ... )
        """
        self.sensor_func = sensor_func
        self.actuator_func = actuator_func
        self.setpoint = setpoint
        self.min_output = min_output
        self.max_output = max_output
        self.start_kp = start_kp
        self.kp_step = kp_step
        self.kp_change_interval_pct = kp_change_interval_pct
        self.sample_time = sample_time
        self.window_size = window_size
        self.oscillation_threshold = oscillation_threshold
        self.min_oscillations = min_oscillations
        self.max_test_time = max_test_time
        self.safety_limits = safety_limits
        self.dual_actuator = dual_actuator
        self.set_remote_kp_func = set_remote_kp_func
        
        # Data storage for analysis and plotting
        self.times = []
        self.pv_values = []
        self.output_values = []
        self.kp_values = []
        
        # Results
        self.ku = None  # Ultimate gain
        self.pu = None  # Oscillation period
        self.pid_params = None  # Calculated PID parameters
    
    def _check_safety(self, pv):
        """
        Check if process variable is within safety limits
        
        Args:
            pv (float): Current process variable value
            
        Returns:
            bool: True if safe or no limits set, False if outside limits
        """
        if self.safety_limits is None:
            return True
        
        min_limit, max_limit = self.safety_limits
        return min_limit <= pv <= max_limit
    
    def _print_status(self, elapsed, pv, output, kp, is_oscillating=False, period=0):
        """Print status information in a fixed position"""
        # Move cursor to home position
        _move_cursor_home()
        
        # Print header
        print("=" * 70)
        print("  ZIEGLER-NICHOLS AUTO-TUNING - LIVE STATUS")
        print("=" * 70)
        print()
        
        # Print main status
        print(f"  Elapsed Time:       {elapsed:8.1f} s  /  {self.max_test_time:.0f} s max")
        print(f"  Setpoint:           {self.setpoint:8.2f}")
        print(f"  Process Variable:   {pv:8.2f}")
        print(f"  Error:              {pv - self.setpoint:8.2f}")
        print(f"  Control Output:     {output:8.2f}")
        print(f"  Current Kp:         {kp:8.6f}")
        print()
        
        # Oscillation status
        if is_oscillating:
            print(f"  Oscillation:        DETECTED ✓")
            print(f"  Period (Pu):        {period:8.4f} s")
            if self.ku:
                print(f"  Ultimate Gain (Ku): {self.ku:8.6f}")
            else:
                print(f"  Ultimate Gain (Ku): Measuring...")
        else:
            print(f"  Oscillation:        Searching...")
            print(f"  Period (Pu):        --")
            print(f"  Ultimate Gain (Ku): --")
        
        print()
        print("=" * 70)
        print("  Press Ctrl+C to stop")
        print("=" * 70)
        
        # Add padding to prevent terminal artifacts
        for _ in range(5):
            print()
        
        sys.stdout.flush()
    
    def _detect_oscillations(self, values):
        """
        Detect sustained oscillations in the process variable
        
        Uses peak and trough detection to identify oscillation cycles.
        Requires consistent oscillation period and sufficient amplitude.
        
        Args:
            values (list): Recent process variable values
            
        Returns:
            tuple: (is_oscillating (bool), period (float))
                  period is in seconds if oscillating, 0 otherwise
        """
        if len(values) < self.window_size:
            return False, 0
        
        # Get the last window_size values
        recent_values = values[-self.window_size:]
        
        # Calculate mean and standard deviation
        mean_value = np.mean(recent_values)
        std_value = np.std(recent_values)
        
        # Check if standard deviation is significant relative to mean or setpoint
        reference_value = max(abs(mean_value), abs(self.setpoint), 1e-6)
        if std_value < self.oscillation_threshold * reference_value:
            return False, 0
        
        # Find peaks and troughs
        peaks = []
        troughs = []
        
        for i in range(1, len(recent_values) - 1):
            if (recent_values[i] > recent_values[i-1] and 
                recent_values[i] > recent_values[i+1]):
                peaks.append(i)
            elif (recent_values[i] < recent_values[i-1] and 
                  recent_values[i] < recent_values[i+1]):
                troughs.append(i)
        
        # Check if we have enough oscillations
        if len(peaks) < self.min_oscillations or len(troughs) < self.min_oscillations:
            return False, 0
        
        # Calculate average period from peaks
        peak_periods = []
        for i in range(1, len(peaks)):
            peak_periods.append(peaks[i] - peaks[i-1])
        
        # Calculate average period from troughs
        trough_periods = []
        for i in range(1, len(troughs)):
            trough_periods.append(troughs[i] - troughs[i-1])
        
        # Average the periods and convert to seconds
        if peak_periods and trough_periods:
            avg_period_samples = (np.mean(peak_periods) + np.mean(trough_periods)) / 2
            period_seconds = avg_period_samples * self.sample_time
            
            # Check if period is reasonably stable (low variation)
            all_periods = peak_periods + trough_periods
            if np.std(all_periods) / avg_period_samples > 0.3:
                return False, 0  # Too much variation in period
                
            return True, period_seconds
        
        return False, 0
    
    def run(self, verbose=True, plot_results=True):
        """
        Run the autotuning process
        
        This method performs the following steps:
        1. Initialize a P-only controller with low gain
        2. Gradually increase Kp until sustained oscillations are detected
        3. Record the ultimate gain (Ku) and oscillation period (Pu)
        4. Calculate PID parameters using Ziegler-Nichols rules
        
        Args:
            verbose (bool): Print progress messages during tuning
                           Default: True
            
            plot_results (bool): Generate and save plots after tuning completes
                               Saves to 'ziegler_nichols_tuning_results.png'
                               Default: True
        
        Returns:
            dict or None: Dictionary of PID parameters for different controller types.
                         Returns None if tuning fails.
                         
                         Structure:
                         {
                             "P": {"Kp": float, "Ki": 0.0, "Kd": 0.0},
                             "PI": {"Kp": float, "Ki": float, "Kd": 0.0},
                             "PID": {"Kp": float, "Ki": float, "Kd": float},
                             "Pessen Integral Rule": {...},
                             "Some Overshoot": {...},
                             "No Overshoot": {...}
                         }
        
        Raises:
            KeyboardInterrupt: Can be used to stop tuning early (handled gracefully)
        
        Example:
            >>> params = autotuner.run(verbose=True, plot_results=True)
            >>> if params:
            ...     print(f"PID: Kp={params['PID']['Kp']:.4f}")
            ...     print(f"     Ki={params['PID']['Ki']:.4f}")
            ...     print(f"     Kd={params['PID']['Kd']:.4f}")
        """
        if verbose:
            _clear_screen()
            print(f"Starting Ziegler-Nichols autotuning")
            print(f"Setpoint: {self.setpoint}")
            print(f"Initial Kp: {self.start_kp}, Step size: {self.kp_step}")
            kp_interval_seconds = self.max_test_time * (self.kp_change_interval_pct / 100.0)
            print(f"Kp change interval: {kp_interval_seconds:.2f}s ({self.kp_change_interval_pct}% of max time)")
            print(f"Output range: {self.min_output} to {self.max_output}")
            if self.dual_actuator:
                print(f"Mode: Bidirectional (dual actuator)")
            else:
                print(f"Mode: Unidirectional")
            if self.safety_limits:
                print(f"Safety limits: {self.safety_limits[0]} to {self.safety_limits[1]}")
            if self.set_remote_kp_func is not None:
                print(f"Remote Kp setting: ENABLED")
            print("\nStarting in 3 seconds...")
            time.sleep(3)
            _clear_screen()
        
        # Initialize P-only controller using simple-pid
        controller = PID(
            Kp=self.start_kp,
            Ki=0.0,
            Kd=0.0,
            setpoint=self.setpoint,
            output_limits=(self.min_output, self.max_output),
            sample_time=self.sample_time,
            auto_mode=True
        )
        
        # Set initial Kp on remote device if function provided
        if self.set_remote_kp_func is not None:
            try:
                self.set_remote_kp_func(self.start_kp)
                if verbose:
                    print(f"Initial Kp ({self.start_kp}) set on remote device")
            except Exception as e:
                if verbose:
                    print(f"Warning: Failed to set initial remote Kp: {e}")
        
        # Initialize data collection
        self.times = []
        self.pv_values = []
        self.output_values = []
        self.kp_values = []
        
        # Calculate Kp change interval in seconds
        kp_change_interval = self.max_test_time * (self.kp_change_interval_pct / 100.0)
        if verbose:
            print(f"Kp will change every {kp_change_interval:.2f} seconds ({self.kp_change_interval_pct}% of max time)")
            time.sleep(2)
        
        # Start time tracking
        start_time = time.time()
        last_sample_time = start_time
        last_kp_change_time = start_time
        
        try:
            # Main tuning loop
            while time.time() - start_time < self.max_test_time:
                current_time = time.time()
                
                # Check if it's time for a new sample
                if current_time - last_sample_time >= self.sample_time:
                    # Read process variable
                    pv = self.sensor_func()
                    
                    # Safety check
                    if not self._check_safety(pv):
                        if verbose:
                            _clear_screen()
                            print("\n\nSAFETY LIMIT EXCEEDED!")
                            print(f"Process variable: {pv}")
                            print(f"Safety limits: {self.safety_limits}")
                        break
                    
                    # Calculate control output
                    output = controller(pv)
                    
                    # Apply control output
                    self.actuator_func(output)
                    
                    # Store data
                    elapsed_time = current_time - start_time
                    self.times.append(elapsed_time)
                    self.pv_values.append(pv)
                    self.output_values.append(output)
                    self.kp_values.append(controller.Kp)
                    
                    # Check for oscillations
                    is_oscillating, period = self._detect_oscillations(self.pv_values)
                    
                    # Display status in fixed position
                    if verbose:
                        self._print_status(elapsed_time, pv, output, controller.Kp,
                                         is_oscillating, period)
                    
                    # If oscillating, we've found Ku and Pu
                    if is_oscillating:
                        self.ku = controller.Kp
                        self.pu = period
                        if verbose:
                            _clear_screen()
                            print("\n\n✓ SUSTAINED OSCILLATIONS DETECTED!")
                            print(f"\nUltimate Gain (Ku): {self.ku:.6f}")
                            print(f"Oscillation Period (Pu): {self.pu:.6f} seconds")
                            print("\nTuning complete!")
                        break
                    
                    # Increase Kp if not oscillating yet and enough time has elapsed
                    if current_time - last_kp_change_time >= kp_change_interval:
                        controller.Kp += self.kp_step
                        last_kp_change_time = current_time
                        
                        # Update Kp on remote device if function provided
                        if self.set_remote_kp_func is not None:
                            try:
                                self.set_remote_kp_func(controller.Kp)
                            except Exception as e:
                                if verbose:
                                    print(f"\nWarning: Failed to set remote Kp: {e}")
                    
                    # Update last sample time
                    last_sample_time = current_time
                
                # Small sleep to prevent CPU spinning
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            if verbose:
                _clear_screen()
                print("\n\nAutotuning interrupted by user!")
        
        finally:
            # Turn off actuator
            self.actuator_func(0)
            if verbose:
                print("\nActuator turned off")
        
        # Calculate PID parameters if we found Ku and Pu
        if self.ku is not None and self.pu is not None and self.pu > 0:
            self.pid_params = self._calculate_pid_parameters()
            
            if verbose and self.pid_params:
                print("\n" + "="*60)
                print("Recommended PID parameters:")
                print("="*60)
                for controller_type, params in self.pid_params.items():
                    print(f"\n{controller_type}:")
                    print(f"  Kp = {params['Kp']:.6f}")
                    print(f"  Ki = {params['Ki']:.6f}")
                    print(f"  Kd = {params['Kd']:.6f}")
        else:
            if verbose:
                print("\n" + "="*60)
                print("Autotuning incomplete or failed")
                print("="*60)
                print("No sustained oscillations detected.")
                print("\nTroubleshooting suggestions:")
                print("  - Increase max_test_time")
                print("  - Adjust start_kp or kp_step")
                print("  - Check oscillation_threshold")
                print("  - Verify sensor_func and actuator_func are working")
                print("="*60)
        
        # Generate plots if requested and data exists
        if plot_results and self.times:
            self._plot_results()
        
        return self.pid_params
    
    def _calculate_pid_parameters(self):
        """
        Calculate PID parameters using Ziegler-Nichols rules
        
        Applies various Ziegler-Nichols tuning formulas to calculate
        PID parameters optimized for different response characteristics.
        
        Returns:
            dict: PID parameters for different controller types, or None if Ku/Pu invalid
                 
                 Keys:
                 - "P": Proportional only
                 - "PI": Proportional-Integral
                 - "PID": Classic PID
                 - "Pessen Integral Rule": More aggressive
                 - "Some Overshoot": Moderate response
                 - "No Overshoot": Conservative, stable response
        """
        if self.ku is None or self.pu is None or self.pu <= 0:
            return None
        
        # Ziegler-Nichols rules for different controller types
        params = {
            "P": {
                "Kp": 0.5 * self.ku,
                "Ki": 0.0,
                "Kd": 0.0
            },
            "PI": {
                "Kp": 0.45 * self.ku,
                "Ki": 0.54 * self.ku / self.pu,
                "Kd": 0.0
            },
            "PID": {
                "Kp": 0.6 * self.ku,
                "Ki": 1.2 * self.ku / self.pu,
                "Kd": 0.075 * self.ku * self.pu
            },
            "Pessen Integral Rule": {
                "Kp": 0.7 * self.ku,
                "Ki": 1.75 * self.ku / self.pu,
                "Kd": 0.105 * self.ku * self.pu
            },
            "Some Overshoot": {
                "Kp": 0.33 * self.ku,
                "Ki": 0.66 * self.ku / self.pu,
                "Kd": 0.11 * self.ku * self.pu
            },
            "No Overshoot": {
                "Kp": 0.2 * self.ku,
                "Ki": 0.4 * self.ku / self.pu,
                "Kd": 0.066 * self.ku * self.pu
            }
        }
        
        return params
    
    def _plot_results(self):
        """
        Generate plots of the autotuning process
        
        Creates a 3-panel plot showing:
        1. Process variable vs. time with setpoint reference
        2. Control output vs. time
        3. Kp value vs. time with ultimate gain marker
        
        Saves plot to 'ziegler_nichols_tuning_results.png'
        """
        try:
            plt.figure(figsize=(12, 10))
            
            # Plot process variable
            plt.subplot(3, 1, 1)
            plt.plot(self.times, self.pv_values, 'b-', linewidth=1.5)
            plt.axhline(y=self.setpoint, color='r', linestyle='--', 
                       linewidth=2, label='Setpoint')
            plt.title('Process Variable Over Time', fontsize=14, fontweight='bold')
            plt.xlabel('Time (s)')
            plt.ylabel('Process Variable')
            plt.grid(True, alpha=0.3)
            plt.legend()
            
            # Plot control output
            plt.subplot(3, 1, 2)
            plt.plot(self.times, self.output_values, 'g-', linewidth=1.5)
            plt.title('Control Output Over Time', fontsize=14, fontweight='bold')
            plt.xlabel('Time (s)')
            if self.dual_actuator:
                plt.ylabel(f'Output ({self.min_output} to {self.max_output})')
                plt.axhline(y=0, color='k', linestyle='--', 
                           linewidth=1, label='Neutral')
                plt.legend()
            else:
                plt.ylabel(f'Output ({self.min_output} to {self.max_output})')
            plt.grid(True, alpha=0.3)
            
            # Plot Kp value
            plt.subplot(3, 1, 3)
            plt.plot(self.times, self.kp_values, 'orange', linewidth=1.5)
            if self.ku is not None:
                plt.axhline(y=self.ku, color='r', linestyle='--', 
                           linewidth=2, label=f'Ultimate Gain (Ku={self.ku:.4f})')
                plt.legend()
            plt.title('Proportional Gain (Kp) Over Time', fontsize=14, fontweight='bold')
            plt.xlabel('Time (s)')
            plt.ylabel('Kp')
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            # Save plot
            plot_filename = 'ziegler_nichols_tuning_results.png'
            plt.savefig(plot_filename, dpi=150)
            print(f"\nPlot saved to {plot_filename}")
            plt.close()
            
        except Exception as e:
            print(f"Error generating plot: {e}")
    
    def get_results(self):
        """
        Get the tuning results
        
        Returns:
            dict: Results containing Ku, Pu, and PID parameters
                 {
                     'ku': float,
                     'pu': float,
                     'pid_params': dict,
                     'success': bool
                 }
        """
        return {
            'ku': self.ku,
            'pu': self.pu,
            'pid_params': self.pid_params,
            'success': self.ku is not None and self.pu is not None
        }
    
    def save_results(self, filename='pid_tuning_results.txt'):
        """
        Save tuning results to a text file
        
        Args:
            filename (str): Output filename
                           Default: 'pid_tuning_results.txt'
        
        Returns:
            bool: True if successful, False if no results to save
        """
        if self.pid_params is None:
            print("No results to save. Run autotuning first.")
            return False
        
        try:
            with open(filename, 'w') as f:
                f.write("="*60 + "\n")
                f.write("Ziegler-Nichols PID Auto-Tuning Results\n")
                f.write("="*60 + "\n\n")
                
                f.write(f"Setpoint: {self.setpoint}\n")
                f.write(f"Ultimate Gain (Ku): {self.ku:.6f}\n")
                f.write(f"Oscillation Period (Pu): {self.pu:.6f} seconds\n\n")
                
                f.write("="*60 + "\n")
                f.write("Recommended PID Parameters\n")
                f.write("="*60 + "\n\n")
                
                for controller_type, params in self.pid_params.items():
                    f.write(f"{controller_type}:\n")
                    f.write(f"  Kp = {params['Kp']:.6f}\n")
                    f.write(f"  Ki = {params['Ki']:.6f}\n")
                    f.write(f"  Kd = {params['Kd']:.6f}\n\n")
            
            print(f"Results saved to {filename}")
            return True
            
        except Exception as e:
            print(f"Error saving results: {e}")
            return False