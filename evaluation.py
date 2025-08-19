#!/usr/bin/env python3
"""
Quantitative Navigation Evaluation (ROS 2, rosbag2) with enhanced visualizations.

Metrics calculated per execution (one bag ≈ one mission):
- Success rate (goal reached within tolerance and before timeout)
- Mission time (s)
- RMSE between followed trajectory and global plan (m)
- Extra distance traveled (%)
- Blockages: number and total duration (s) of unplanned stops

Generated visualizations:
- Trajectory vs Global Plan (2D)
- Linear and angular velocity vs time
- Distance to goal vs time
- Blockages highlighted in velocity plots

Basic usage:
    python evaluation.py --bag ./nav_eval_bag --use_pose /odom
"""

import argparse
import math
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ROS 2 imports
try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except Exception as e:
    print("[ERROR] This script requires a ROS 2 environment with rosbag2_py and rclpy installed.")
    print(e)
    sys.exit(1)


def path_length(xy: np.ndarray) -> float:
    """
    Calculate the total length of a path.
    
    Args:
        xy: Array of shape (n, 2) containing x,y coordinates
        
    Returns:
        Total path length in meters
    """
    if len(xy) < 2:
        return 0.0
    diffs = np.diff(xy, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def nearest_point_rmse(traj: np.ndarray, plan: np.ndarray) -> float:
    """
    Calculate RMSE between trajectory and plan by finding nearest points.
    
    Args:
        traj: Trajectory points as np.ndarray of shape (n, 2)
        plan: Plan points as np.ndarray of shape (m, 2)
        
    Returns:
        RMSE value in meters or NaN if input is invalid
    """
    if len(traj) == 0 or len(plan) < 2:
        return float('nan')
    
    seg_starts = plan[:-1]
    seg_ends = plan[1:]
    seg_vecs = seg_ends - seg_starts
    seg_lens2 = np.sum(seg_vecs**2, axis=1)
    
    def point_to_polyline_dist2(p):
        ap = p - seg_starts
        t = np.clip(np.sum(ap * seg_vecs, axis=1) / np.maximum(seg_lens2, 1e-12), 0.0, 1.0)
        proj = seg_starts + seg_vecs * t[:, None]
        d2 = np.sum((p - proj)**2, axis=1)
        return float(np.min(d2))
    
    d2s = [point_to_polyline_dist2(p) for p in traj]
    return math.sqrt(float(np.mean(d2s)))


@dataclass
class TopicSpec:
    """Class for storing topic name and type information."""
    name: str
    type_str: str


def build_reader(bag_path: str) -> SequentialReader:
    """
    Build a rosbag2 sequential reader.
    
    Args:
        bag_path: Path to the rosbag2 directory
        
    Returns:
        Configured SequentialReader object
    """
    so = StorageOptions(uri=bag_path, storage_id='sqlite3')
    co = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(so, co)
    return reader


def list_topics(reader: SequentialReader) -> List[TopicSpec]:
    """
    List all topics in a rosbag2.
    
    Args:
        reader: SequentialReader object
        
    Returns:
        List of TopicSpec objects
    """
    infos = reader.get_all_topics_and_types()
    return [TopicSpec(t.name, t.type) for t in infos]


def read_topic_messages(bag_path: str, topic: str) -> List[Tuple[int, object]]:
    """
    Read all messages from a specific topic in a rosbag2.
    
    Args:
        bag_path: Path to the rosbag2 directory
        topic: Topic name to read from
        
    Returns:
        List of tuples (timestamp, message)
    """
    reader = build_reader(bag_path)
    topics = list_topics(reader)
    type_str = None
    
    for t in topics:
        if t.name == topic:
            type_str = t.type_str
            break
            
    if type_str is None:
        return []
        
    msg_type = get_message(type_str)
    reader = build_reader(bag_path)
    out = []
    
    while reader.has_next():
        topic_name, data, t = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, msg_type)
            out.append((t, msg))
            
    return out


def extract_xy_from_odometry(messages):
    """
    Extract position data from odometry messages.
    
    Args:
        messages: List of (timestamp, message) tuples
        
    Returns:
        Tuple of numpy arrays: (timestamps, x_positions, y_positions)
    """
    ts, xs, ys = [], [], []
    
    for t, m in messages:
        ts.append(t * 1e-9)
        xs.append(m.pose.pose.position.x)
        ys.append(m.pose.pose.position.y)
        
    return np.array(ts), np.array(xs), np.array(ys)


def extract_plan_xy(messages):
    """
    Extract the global plan from path messages.
    
    Args:
        messages: List of (timestamp, message) tuples
        
    Returns:
        Numpy array of shape (n, 2) with x,y coordinates or empty array
    """
    best = None
    best_len = -1
    
    for _, m in messages:
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        if len(pts) > best_len:
            best = np.array(pts, dtype=float)
            best_len = len(pts)
            
    return best if best is not None else np.empty((0, 2))


def extract_goal(messages):
    """
    Extract the goal position from goal messages.
    
    Args:
        messages: List of (timestamp, message) tuples
        
    Returns:
        Tuple (x, y) or None if no messages
    """
    if not messages:
        return None
    _, m = messages[-1]
    return (m.pose.position.x, m.pose.position.y)


def extract_cmd_vel(messages):
    """
    Extract velocity commands from cmd_vel messages.
    
    Args:
        messages: List of (timestamp, message) tuples
        
    Returns:
        Tuple of numpy arrays: (timestamps, linear_velocities, angular_velocities)
    """
    ts, v, w = [], [], []
    
    for t, m in messages:
        ts.append(t * 1e-9)
        v.append(m.linear.x)
        w.append(m.angular.z)
        
    return np.array(ts), np.array(v), np.array(w)


def mission_time(t_pose, xy, goal, tol, timeout, t0=None):
    """
    Calculate mission success and completion time.
    
    Args:
        t_pose: Array of timestamps
        xy: Array of positions (n, 2)
        goal: Tuple (x, y) of goal position
        tol: Distance tolerance to consider goal reached
        timeout: Maximum mission time allowed
        t0: Start time (defaults to first timestamp)
        
    Returns:
        Tuple (success, mission_time, reach_time)
    """
    if len(t_pose) == 0:
        return False, float('nan'), float('nan')
        
    if t0 is None:
        t0 = t_pose[0]
        
    gx, gy = goal
    dists = np.linalg.norm(xy - np.array([gx, gy]), axis=1)
    idx = np.where(dists <= tol)[0]
    
    if len(idx) > 0:
        t_reach = t_pose[idx[0]]
        tm = t_reach - t0
        return (tm <= timeout), tm, t_reach
    else:
        return False, t_pose[-1] - t0, float('nan')


def compute_blockages(t_cmd, v, v_thresh, t_thresh, ignore_initial=True):
    """
    Calculate robot blockage periods based on low velocity.
    
    Args:
        t_cmd: Array of command timestamps
        v: Array of linear velocities
        v_thresh: Velocity threshold to consider stopped
        t_thresh: Minimum time threshold to consider a blockage
        ignore_initial: If True, ignore initial period until first movement
        
    Returns:
        Tuple (number_of_blockages, total_blockage_time, blockage_intervals)
    """
    if len(t_cmd) == 0:
        return 0, 0.0, []
        
    is_stop = np.abs(v) < v_thresh
    blocks, in_block, t_start = [], False, None
    
    # Find first significant movement
    first_movement_idx = 0
    if ignore_initial:
        moving_indices = np.where(np.abs(v) >= v_thresh)[0]
        if len(moving_indices) > 0:
            first_movement_idx = moving_indices[0]
    
    # Analyze blockages after first movement
    for i in range(first_movement_idx, len(t_cmd)):
        if is_stop[i] and not in_block:
            in_block, t_start = True, t_cmd[i]
        elif not is_stop[i] and in_block:
            dur = t_cmd[i] - t_start
            if dur >= t_thresh:
                blocks.append((t_start, t_cmd[i]))
            in_block = False
    
    # Check if ending in blockage
    if in_block:
        dur = t_cmd[-1] - t_start
        if dur >= t_thresh:
            blocks.append((t_start, t_cmd[-1]))
            
    return len(blocks), sum(b[1]-b[0] for b in blocks), blocks


def align_plan_to_traj(plan_xy: np.ndarray, traj_xy: np.ndarray) -> np.ndarray:
    """
    Align global plan to start at the same point as trajectory.
    (Only corrects translation, not rotation or scale).
    
    Args:
        plan_xy: Plan points as np.ndarray of shape (n, 2)
        traj_xy: Trajectory points as np.ndarray of shape (m, 2)
        
    Returns:
        Aligned plan as np.ndarray
    """
    if len(plan_xy) == 0 or len(traj_xy) == 0:
        return plan_xy
        
    shift = traj_xy[0] - plan_xy[0]
    return plan_xy + shift


def procrustes_align(plan_xy: np.ndarray, traj_xy: np.ndarray, allow_scaling=False, extend_start=True) -> np.ndarray:
    """
    Align plan with trajectory using Procrustes analysis.
    Finds best translation, rotation, and optionally scale.
    
    Args:
        plan_xy: Plan points as np.ndarray of shape (n, 2)
        traj_xy: Trajectory points as np.ndarray of shape (m, 2)
        allow_scaling: Whether to allow scaling transformation
        extend_start: Whether to extend plan to start before first point
        
    Returns:
        Aligned plan as np.ndarray
    """
    if len(plan_xy) < 2 or len(traj_xy) < 2:
        return align_plan_to_traj(plan_xy, traj_xy)
    
    # Get cumulative lengths to parameterize curves
    def get_cum_lengths(points):
        diffs = np.diff(points, axis=0)
        seg_lens = np.linalg.norm(diffs, axis=1)
        cum_lens = np.concatenate([[0], np.cumsum(seg_lens)])
        return cum_lens, cum_lens[-1]
    
    plan_cum_lens, plan_total_len = get_cum_lengths(plan_xy)
    traj_cum_lens, traj_total_len = get_cum_lengths(traj_xy)
    
    # Select sample points for correspondence 
    num_samples = min(20, min(len(plan_xy), len(traj_xy)))
    
    # Calculate equidistant points in arc length
    plan_samples_idx = []
    traj_samples_idx = []
    
    for i in range(num_samples):
        # Relative position in curve (0 to 1)
        t = i / (num_samples - 1)
        
        # Find corresponding indices in both curves
        plan_pos = t * plan_total_len
        traj_pos = t * traj_total_len
        
        plan_idx = np.argmin(np.abs(plan_cum_lens - plan_pos))
        traj_idx = np.argmin(np.abs(traj_cum_lens - traj_pos))
        
        plan_samples_idx.append(plan_idx)
        traj_samples_idx.append(traj_idx)
    
    # Extract sample points
    plan_samples = plan_xy[plan_samples_idx]
    traj_samples = traj_xy[traj_samples_idx]
    
    # Center both point sets
    plan_centroid = np.mean(plan_samples, axis=0)
    traj_centroid = np.mean(traj_samples, axis=0)
    
    plan_centered = plan_samples - plan_centroid
    traj_centered = traj_samples - traj_centroid
    
    # Calculate covariance matrix
    covariance = np.dot(plan_centered.T, traj_centered)
    
    # SVD decomposition
    U, S, Vt = np.linalg.svd(covariance)
    
    # Calculate rotation matrix
    R = np.dot(Vt.T, U.T)
    
    # Correct if reflection instead of rotation
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)
    
    # Calculate scale if allowed
    scale = 1.0
    if allow_scaling:
        scale = np.sum(S) / np.sum(np.sum(plan_centered**2, axis=1))
    
    # Apply transformation to full original plan
    aligned_plan = scale * np.dot(plan_xy - plan_centroid, R.T) + traj_centroid
    
    # Extend plan backwards if requested
    if extend_start and len(aligned_plan) >= 2:
        # Direction vector from point 1 to point 0
        direction = aligned_plan[0] - aligned_plan[1]
        # Normalize and add as extension
        direction_norm = direction / np.linalg.norm(direction)
        # Extend by 50% of distance between first two points
        extension_distance = np.linalg.norm(direction) * 0.5
        new_start_point = aligned_plan[0] + direction_norm * extension_distance
        # Add new point to beginning
        aligned_plan = np.vstack([new_start_point, aligned_plan])
    
    return aligned_plan


def evaluate_bag(args):
    """
    Evaluate navigation performance from a rosbag2.
    
    Args:
        args: Parsed command-line arguments
        
    Returns:
        Dictionary with calculated metrics
        
    Raises:
        RuntimeError: If required messages are not found
    """
    pose_msgs = read_topic_messages(args.bag, args.use_pose)
    if not pose_msgs:
        raise RuntimeError(f"No messages found in {args.use_pose}")
        
    t_pose, xs, ys = extract_xy_from_odometry(pose_msgs)
    traj_xy = np.column_stack([xs, ys])

    plan_xy = extract_plan_xy(read_topic_messages(args.bag, args.plan_topic))
    goal = extract_goal(read_topic_messages(args.bag, args.goal_topic))
    
    if goal is None and len(plan_xy) > 0:
        goal = tuple(plan_xy[-1])
    if goal is None:
        goal = (traj_xy[-1,0], traj_xy[-1,1])

    t_cmd, v_cmd, w_cmd = extract_cmd_vel(read_topic_messages(args.bag, args.cmd_topic))
    t0 = t_cmd[0] if len(t_cmd) else t_pose[0]

    # Apply alignment before calculating metrics
    aligned_plan_xy = procrustes_align(plan_xy, traj_xy, allow_scaling=False)
    
    # Update goal to be at end of aligned plan
    if len(aligned_plan_xy) > 0:
        goal = (aligned_plan_xy[-1, 0], aligned_plan_xy[-1, 1])
    
    # Calculate metrics
    success, t_mis, t_reach = mission_time(t_pose, traj_xy, goal, args.goal_tolerance, args.timeout, t0)
    traj_len = path_length(traj_xy)
    plan_len = path_length(aligned_plan_xy)
    extra_pct = (traj_len - plan_len) / plan_len * 100.0 if plan_len > 0 else float('nan')
    rmse = nearest_point_rmse(traj_xy, aligned_plan_xy) if len(aligned_plan_xy) > 1 else float('nan')
    n_blocks, t_blocks, blocks = compute_blockages(t_cmd, v_cmd, args.block_v_thresh, 
                                                  args.block_t_thresh, ignore_initial=True)

    os.makedirs(args.out_dir, exist_ok=True)

    # --- Generate plots ---
    # 1) Trayectoria vs Plan
    plt.figure()
    if len(aligned_plan_xy):
        plt.plot(aligned_plan_xy[:,0], aligned_plan_xy[:,1], 'r--', label='Plan Alineado')
    plt.plot(traj_xy[:,0], traj_xy[:,1], 'b-', label='Trayectoria')
    plt.scatter([goal[0]], [goal[1]], c='g', marker='*', s=120, label='Meta')
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.title('Trayectoria vs Plan')
    plt.savefig(os.path.join(args.out_dir, 'trayectoria_vs_plan.png'), dpi=180)
    plt.close()

    # 2) Velocidades lineal y angular
    t_rel_cmd = np.array(t_cmd) - t0
    plt.figure()
    plt.plot(t_rel_cmd, v_cmd, 'g-', label='v [m/s]')
    plt.plot(t_rel_cmd, w_cmd, 'b-', label='ω [rad/s]')
    # Highlight blockages
    for t_ini, t_fin in blocks:
        plt.axvspan(t_ini - t0, t_fin - t0, alpha=0.2, color='r')
    plt.grid()
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Velocidad') 
    plt.legend()
    plt.title('Velocidades')
    plt.savefig(os.path.join(args.out_dir, 'velocidades.png'), dpi=180)
    plt.close()

    # 3) Distancia a la meta
    t_rel_pose = np.array(t_pose) - t0
    gx, gy = goal
    dists = np.linalg.norm(traj_xy - np.array([gx, gy]), axis=1)
    plt.figure()
    plt.plot(t_rel_pose, dists, 'b-', label='Distancia a la meta')
    plt.axhline(y=args.goal_tolerance, color='g', linestyle='--', alpha=0.5, 
                label=f'Tolerancia {args.goal_tolerance}m')
    # Highlight blockages
    for t_ini, t_fin in blocks:
        plt.axvspan(t_ini - t0, t_fin - t0, alpha=0.2, color='r')
    plt.grid()
    plt.xlabel('Tiempo [s]') 
    plt.ylabel('Distancia [m]')
    plt.legend()
    plt.title('Distancia a la Meta')
    plt.savefig(os.path.join(args.out_dir, 'distancia_meta.png'), dpi=180)
    plt.close()

    # --- Save results ---
    # Metrics summary in CSV
    metrics_df = pd.DataFrame({
        'bag': [os.path.basename(args.bag)],
        'success': [success],
        'mission_time_s': [t_mis],
        'traj_length_m': [traj_len],
        'plan_length_m': [plan_len],
        'extra_distance_pct': [extra_pct],
        'rmse_m': [rmse],
        'num_blockages': [n_blocks],
        'blockages_total_s': [t_blocks]
    })
    metrics_df.to_csv(os.path.join(args.out_dir, 'metricas.csv'), index=False)

    # Blockage details
    if n_blocks > 0:
        blockages_df = pd.DataFrame(blocks, columns=['t_ini_s', 't_fin_s'])
        blockages_df['dur_s'] = blockages_df['t_fin_s'] - blockages_df['t_ini_s']
        blockages_df.to_csv(os.path.join(args.out_dir, 'bloqueos.csv'), index=False)

    print(f"Analysis completed: {args.bag}")
    print(f"  Success: {success}, Time: {t_mis:.2f}s, RMSE: {rmse:.3f}m")
    print(f"  Distance: +{extra_pct:.1f}%, Blockages: {n_blocks} ({t_blocks:.1f}s)")
    
    # Return metrics dict for programmatic use
    return {
        'success': success,
        'mission_time_s': t_mis,
        'traj_length_m': traj_len,
        'plan_length_m': plan_len,
        'extra_distance_pct': extra_pct,
        'rmse_m': rmse,
        'num_blockages': n_blocks,
        'blockages_total_s': t_blocks,
        'blocks': blocks
    }


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Quantitative navigation evaluation from rosbag2")
    parser.add_argument('--bag', type=str, required=True, help="Rosbag2 directory path")
    parser.add_argument('--use_pose', type=str, default='/odom', help="Pose topic to use (/odom or /amcl_pose)")
    parser.add_argument('--plan_topic', type=str, default='/plan', help="Global plan topic")
    parser.add_argument('--cmd_topic', type=str, default='/cmd_vel', help="Velocity commands topic")
    parser.add_argument('--goal_topic', type=str, default='/goal_pose', help="Goal topic")
    parser.add_argument('--goal_tolerance', type=float, default=0.3, help="Goal distance tolerance [m]")
    parser.add_argument('--timeout', type=float, default=300.0, help="Mission timeout [s]")
    parser.add_argument('--block_v_thresh', type=float, default=0.05, help="Velocity threshold for blockages [m/s]")
    parser.add_argument('--block_t_thresh', type=float, default=3.0, help="Time threshold for blockages [s]")
    parser.add_argument('--out_dir', type=str, default="./resultados", help="Output directory")
    return parser.parse_args()


def main():
    """Main function."""
    args = parse_args()
    rclpy.init(args=None)
    try:
        evaluate_bag(args)
    except Exception as e:
        print(f"Error processing {args.bag}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()