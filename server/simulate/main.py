"""
Multi-Agent Formation Control Simulation
=========================================
Virtual Structure Control with Event-Triggered Communication
Centroid is COMPUTED from actual robot positions

Usage:
    python main.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import warnings
warnings.filterwarnings('ignore')

# Local imports
from config import (
    DT, SIMULATION_TIME, NUM_ROBOTS, CORNERS,
    ANIMATION_INTERVAL, SPEED_FACTOR, FORMATION_OFFSETS,
    HEADING_BIAS_STD, MEASUREMENT_NOISE_STD, TRIGGER_THRESHOLD,
    VELOCITY_MAX, FORMATION_SPACING, SENSOR_UPDATE_PERIOD,
    LATENCY_MIN, LATENCY_MAX, OUTLIER_PROBABILITY, OUTLIER_MAGNITUDE
)
from robot_model import MecanumRobotETC
from controller import VirtualStructureController
from visualizer import FormationVisualizer


class FormationSimulation:
    """Main simulation class with centroid-based control."""
    
    def __init__(self):
        # Calculate initial centroid position
        initial_centroid = CORNERS[0].copy()
        
        # Initialize robots at formation positions around centroid
        self.robots = []
        for i in range(NUM_ROBOTS):
            initial_pos = initial_centroid + FORMATION_OFFSETS[i]
            robot = MecanumRobotETC(i, initial_pos)
            self.robots.append(robot)
        
        # Print robot info
        print("\n" + "=" * 70)
        print("Virtual Structure Formation Control (Centroid-Based)")
        print("=" * 70)
        print(f"\nRobot Heading Biases (IMU Drift):")
        for robot in self.robots:
            bias_deg = np.degrees(robot.heading_bias)
            print(f"  Robot {robot.id + 1}: {bias_deg:+.3f}°")
        
        # Initialize controller (centroid-based, no VirtualLeader)
        self.controller = VirtualStructureController(self.robots)
        
        # Initialize visualizer
        self.visualizer = FormationVisualizer(
            self.robots, self.controller
        )
        
        # Simulation state
        self.current_step = 0
        self.total_steps = int(SIMULATION_TIME / DT)
        self.simulation_time = 0.0
    
    def physics_step(self):
        """Perform one physics step (100Hz)."""
        if self.current_step >= self.total_steps:
            return False
        
        # Compute robot velocities (controller updates centroid internally)
        velocities = self.controller.compute_velocities(self.simulation_time)
        
        # Apply velocities and update robots
        for i, robot in enumerate(self.robots):
            robot.set_velocity(velocities[i][0], velocities[i][1])
            robot.update(DT, self.simulation_time)
        
        # Check constraints
        satisfied, violations = self.controller.check_constraints()
        if not satisfied:
            for v in violations:
                print(f"  ⚠ Constraint violation: Robot {v['robot']+1} at {v['distance']*100:.1f}cm (limit: {v['limit']*100:.0f}cm)")
        
        # Update timers
        self.simulation_time += DT
        self.current_step += 1
        
        return True
    
    def update_animation(self, frame):
        """Update animation frame."""
        # Get current speed from slider (dynamic)
        speed = self.visualizer.get_speed_factor()
        
        # Multiple physics steps per frame based on slider
        for _ in range(speed):
            if not self.physics_step():
                break
        
        # Update visualizer
        self.visualizer.update_frame(
            self.simulation_time, 
            self.current_step, 
            self.total_steps
        )
    
    def run(self):
        """Run the simulation."""
        print(f"\n" + "-" * 70)
        print("Simulation Parameters:")
        print("-" * 70)
        print(f"Physics Rate:       100 Hz (dt = {DT*1000:.0f}ms)")
        print(f"Sensor Rate:        20 Hz (period = {SENSOR_UPDATE_PERIOD*1000:.0f}ms)")
        print(f"Trigger Threshold:  {TRIGGER_THRESHOLD*100:.0f} cm")
        print(f"Velocity Max:       {VELOCITY_MAX} m/s")
        print(f"Formation:          Triangle (centroid-based)")
        print(f"Error Model:        Heading bias ±{np.degrees(HEADING_BIAS_STD):.2f}° + {MEASUREMENT_NOISE_STD*100:.0f}cm noise")
        print(f"\nFAILURE MODES:")
        print(f"  Latency Jitter:   {LATENCY_MIN*1000:.0f}-{LATENCY_MAX*1000:.0f} ms (variable)")
        print(f"  Sensor Outliers:  {OUTLIER_PROBABILITY*100:.0f}% chance, {OUTLIER_MAGNITUDE*100:.0f}cm jump")
        print(f"\nAnimation Speed:    {SPEED_FACTOR}x")
        print("-" * 70)
        print("\n🔔 Yellow flashes = ETC broadcast events")
        print("   Cyan polygon = Payload (connects robots)")
        print("   X marker = Centroid (computed from robot positions)")
        print("\n⚠ Constraint violations will be printed if formation breaks!")
        print("\nStarting animation... (Close window to exit)")
        
        num_frames = int(self.total_steps / SPEED_FACTOR) + 50
        
        self.anim = FuncAnimation(
            self.visualizer.get_figure(),
            self.update_animation,
            frames=num_frames,
            interval=ANIMATION_INTERVAL,
            blit=False,
            repeat=False
        )
        
        plt.show()
        
        self.print_statistics()
    
    def print_statistics(self):
        """Print final statistics."""
        print("\n" + "=" * 70)
        print("FINAL STATISTICS")
        print("=" * 70)
        
        total_packets = sum(robot.total_broadcasts for robot in self.robots)
        expected_tt = NUM_ROBOTS * (self.simulation_time / SENSOR_UPDATE_PERIOD)
        savings = (1 - total_packets / expected_tt) * 100 if expected_tt > 0 else 0
        
        print(f"\n=== Communication ===")
        print(f"Total ETC Packets:    {total_packets}")
        print(f"Expected Time-Trig:   {int(expected_tt)}")
        print(f"Bandwidth Savings:    {savings:.1f}%")
        print(f"\nPer-Robot Broadcasts:")
        for robot in self.robots:
            print(f"  Robot {robot.id + 1}: {robot.total_broadcasts} packets")
        
        # Formation errors
        formation_errors = self.controller.get_formation_error() * 100
        
        print(f"\n=== Final Formation Errors ===")
        for i, err in enumerate(formation_errors):
            print(f"  Robot {i+1}: {err:.1f} cm from ideal position")
        print(f"  Mean: {np.mean(formation_errors):.1f} cm")
        
        # Sensor outliers
        total_outliers = sum(robot.total_outliers for robot in self.robots)
        print(f"\n=== Failure Mode Statistics ===")
        print(f"Total Sensor Outliers: {total_outliers}")
        for robot in self.robots:
            print(f"  Robot {robot.id + 1}: {robot.total_outliers} outliers")
        
        # Centroid final position
        final_centroid = self.controller.get_current_centroid()
        target = CORNERS[-1]
        print(f"\n=== Path Completion ===")
        print(f"Final Centroid: ({final_centroid[0]:.3f}, {final_centroid[1]:.3f})")
        print(f"Target:         ({target[0]:.3f}, {target[1]:.3f})")
        print(f"Distance Error: {np.linalg.norm(final_centroid - target):.3f} m")
        print("=" * 70)


def main():
    """Entry point."""
    np.random.seed(42)  # For reproducibility
    
    simulation = FormationSimulation()
    simulation.run()


if __name__ == "__main__":
    main()
