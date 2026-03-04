"""
Formation Metrics Utilities
===========================
Các hàm tính toán metrics cho phân tích đội hình
"""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum


class FormationShape(Enum):
    """Các loại đội hình"""
    LINE = "line"
    TRIANGLE = "triangle"
    SQUARE = "square"
    DIAMOND = "diamond"
    CUSTOM = "custom"


@dataclass
class RobotState:
    """Trạng thái của robot"""
    x: float
    y: float
    theta: float
    vx: float = 0.0
    vy: float = 0.0
    timestamp: float = 0.0
    robot_id: str = ""


@dataclass
class FormationMetrics:
    """Các metrics đánh giá đội hình"""
    centroid: Tuple[float, float]
    mean_distance_to_centroid: float
    formation_error: float
    compactness: float
    spread: float
    alignment_error: float
    coordination_index: float
    stability_score: float


def calculate_centroid(positions: List[Tuple[float, float]]) -> Tuple[float, float]:
    """
    Tính trọng tâm của đội hình
    
    Args:
        positions: List các vị trí (x, y)
    
    Returns:
        Tuple (x, y) của trọng tâm
    """
    if not positions:
        return (0.0, 0.0)
    
    x_coords = [p[0] for p in positions]
    y_coords = [p[1] for p in positions]
    
    return (np.mean(x_coords), np.mean(y_coords))


def calculate_inter_robot_distances(positions: List[Tuple[float, float]]) -> np.ndarray:
    """
    Tính ma trận khoảng cách giữa các robot
    
    Args:
        positions: List các vị trí (x, y)
    
    Returns:
        Ma trận khoảng cách NxN
    """
    n = len(positions)
    distances = np.zeros((n, n))
    
    for i in range(n):
        for j in range(i + 1, n):
            dx = positions[i][0] - positions[j][0]
            dy = positions[i][1] - positions[j][1]
            dist = np.sqrt(dx**2 + dy**2)
            distances[i, j] = dist
            distances[j, i] = dist
    
    return distances


def calculate_formation_error(
    current_positions: List[Tuple[float, float]],
    desired_positions: List[Tuple[float, float]]
) -> float:
    """
    Tính sai số đội hình so với vị trí mong muốn
    
    Args:
        current_positions: Vị trí hiện tại
        desired_positions: Vị trí mong muốn
    
    Returns:
        RMSE của sai số vị trí
    """
    if len(current_positions) != len(desired_positions):
        raise ValueError("Số lượng vị trí không khớp")
    
    errors = []
    for curr, desired in zip(current_positions, desired_positions):
        error = np.sqrt((curr[0] - desired[0])**2 + (curr[1] - desired[1])**2)
        errors.append(error)
    
    return np.sqrt(np.mean(np.array(errors)**2))


def calculate_distance_formation_error(
    current_positions: List[Tuple[float, float]],
    desired_distances: np.ndarray
) -> float:
    """
    Tính sai số đội hình dựa trên khoảng cách mong muốn giữa các robot
    
    Args:
        current_positions: Vị trí hiện tại
        desired_distances: Ma trận khoảng cách mong muốn
    
    Returns:
        RMSE của sai số khoảng cách
    """
    current_distances = calculate_inter_robot_distances(current_positions)
    n = len(current_positions)
    
    errors = []
    for i in range(n):
        for j in range(i + 1, n):
            error = abs(current_distances[i, j] - desired_distances[i, j])
            errors.append(error)
    
    if not errors:
        return 0.0
    
    return np.sqrt(np.mean(np.array(errors)**2))


def calculate_compactness(positions: List[Tuple[float, float]]) -> float:
    """
    Tính độ gọn (compactness) của đội hình
    Giá trị càng nhỏ càng gọn
    
    Args:
        positions: List các vị trí (x, y)
    
    Returns:
        Chỉ số compactness
    """
    if len(positions) < 2:
        return 0.0
    
    centroid = calculate_centroid(positions)
    distances_to_centroid = [
        np.sqrt((p[0] - centroid[0])**2 + (p[1] - centroid[1])**2)
        for p in positions
    ]
    
    return np.std(distances_to_centroid)


def calculate_spread(positions: List[Tuple[float, float]]) -> float:
    """
    Tính độ trải rộng (spread) của đội hình
    
    Args:
        positions: List các vị trí (x, y)
    
    Returns:
        Bán kính lớn nhất từ centroid
    """
    if len(positions) < 2:
        return 0.0
    
    centroid = calculate_centroid(positions)
    distances_to_centroid = [
        np.sqrt((p[0] - centroid[0])**2 + (p[1] - centroid[1])**2)
        for p in positions
    ]
    
    return np.max(distances_to_centroid)


def calculate_alignment_error(headings: List[float], target_heading: float = None) -> float:
    """
    Tính sai số alignment (hướng đi) của các robot
    
    Args:
        headings: List các góc heading (rad)
        target_heading: Góc mục tiêu (nếu None thì dùng mean)
    
    Returns:
        Độ lệch chuẩn của heading error
    """
    if not headings:
        return 0.0
    
    if target_heading is None:
        # Tính mean heading sử dụng circular mean
        sin_mean = np.mean(np.sin(headings))
        cos_mean = np.mean(np.cos(headings))
        target_heading = np.arctan2(sin_mean, cos_mean)
    
    # Tính angular error
    errors = []
    for h in headings:
        error = np.arctan2(np.sin(h - target_heading), np.cos(h - target_heading))
        errors.append(abs(error))
    
    return np.std(errors)


def calculate_velocity_coherence(velocities: List[Tuple[float, float]]) -> float:
    """
    Tính độ đồng nhất vận tốc (velocity coherence)
    Giá trị 1 = hoàn toàn đồng nhất, 0 = hoàn toàn không đồng nhất
    
    Args:
        velocities: List các vận tốc (vx, vy)
    
    Returns:
        Chỉ số velocity coherence [0, 1]
    """
    if not velocities:
        return 0.0
    
    # Tính mean velocity
    vx_mean = np.mean([v[0] for v in velocities])
    vy_mean = np.mean([v[1] for v in velocities])
    mean_speed = np.sqrt(vx_mean**2 + vy_mean**2)
    
    if mean_speed < 1e-6:
        return 1.0  # Tất cả đang đứng yên = đồng nhất
    
    # Tính dot product với mean velocity
    coherence_values = []
    for vx, vy in velocities:
        speed = np.sqrt(vx**2 + vy**2)
        if speed > 1e-6:
            dot_product = (vx * vx_mean + vy * vy_mean) / (speed * mean_speed)
            coherence_values.append(max(0, dot_product))
        else:
            coherence_values.append(1.0)
    
    return np.mean(coherence_values)


def calculate_coordination_index(
    positions: List[Tuple[float, float]],
    velocities: List[Tuple[float, float]]
) -> float:
    """
    Tính chỉ số phối hợp tổng thể
    
    Args:
        positions: List các vị trí (x, y)
        velocities: List các vận tốc (vx, vy)
    
    Returns:
        Chỉ số coordination [0, 1]
    """
    if len(positions) < 2:
        return 1.0
    
    # Velocity coherence
    vel_coherence = calculate_velocity_coherence(velocities)
    
    # Formation compactness (normalize)
    compactness = calculate_compactness(positions)
    spread = calculate_spread(positions)
    
    if spread > 0:
        relative_compactness = 1 - (compactness / spread)
    else:
        relative_compactness = 1.0
    
    # Combine metrics
    coordination = 0.6 * vel_coherence + 0.4 * relative_compactness
    
    return min(1.0, max(0.0, coordination))


def calculate_stability_score(
    formation_errors: np.ndarray,
    window_size: int = 20
) -> float:
    """
    Tính điểm ổn định của đội hình dựa trên chuỗi formation error
    
    Args:
        formation_errors: Array các giá trị formation error theo thời gian
        window_size: Kích thước cửa sổ tính variance
    
    Returns:
        Điểm ổn định [0, 1]
    """
    if len(formation_errors) < window_size:
        window_size = max(2, len(formation_errors) // 2)
    
    # Tính moving variance
    moving_var = []
    for i in range(window_size, len(formation_errors)):
        window = formation_errors[i-window_size:i]
        moving_var.append(np.var(window))
    
    if not moving_var:
        return 1.0
    
    # Normalize variance
    max_var = np.max(moving_var)
    if max_var > 0:
        stability = 1 - np.mean(moving_var) / max_var
    else:
        stability = 1.0
    
    return min(1.0, max(0.0, stability))


def detect_formation_breakdown(
    formation_errors: np.ndarray,
    threshold: float = 0.1,
    min_duration: int = 10
) -> List[Tuple[int, int]]:
    """
    Phát hiện các khoảng thời gian đội hình bị phá vỡ
    
    Args:
        formation_errors: Array các giá trị formation error
        threshold: Ngưỡng xác định breakdown
        min_duration: Thời gian tối thiểu (số samples)
    
    Returns:
        List các (start_idx, end_idx) của breakdown periods
    """
    breakdown_mask = formation_errors > threshold
    breakdowns = []
    
    start_idx = None
    for i, is_breakdown in enumerate(breakdown_mask):
        if is_breakdown and start_idx is None:
            start_idx = i
        elif not is_breakdown and start_idx is not None:
            if i - start_idx >= min_duration:
                breakdowns.append((start_idx, i))
            start_idx = None
    
    # Check end
    if start_idx is not None and len(formation_errors) - start_idx >= min_duration:
        breakdowns.append((start_idx, len(formation_errors)))
    
    return breakdowns


def calculate_all_metrics(
    positions: List[Tuple[float, float]],
    velocities: List[Tuple[float, float]],
    headings: List[float],
    formation_error_history: np.ndarray = None
) -> FormationMetrics:
    """
    Tính toán tất cả các metrics cho đội hình
    
    Args:
        positions: Vị trí các robot
        velocities: Vận tốc các robot
        headings: Heading các robot
        formation_error_history: Lịch sử formation error
    
    Returns:
        FormationMetrics object
    """
    centroid = calculate_centroid(positions)
    
    # Mean distance to centroid
    distances_to_centroid = [
        np.sqrt((p[0] - centroid[0])**2 + (p[1] - centroid[1])**2)
        for p in positions
    ]
    mean_dist_to_centroid = np.mean(distances_to_centroid)
    
    compactness = calculate_compactness(positions)
    spread = calculate_spread(positions)
    alignment_error = calculate_alignment_error(headings)
    coordination_index = calculate_coordination_index(positions, velocities)
    
    # Formation error (dùng compactness làm proxy)
    formation_error = compactness
    
    # Stability score
    if formation_error_history is not None and len(formation_error_history) > 0:
        stability_score = calculate_stability_score(formation_error_history)
    else:
        stability_score = 1.0
    
    return FormationMetrics(
        centroid=centroid,
        mean_distance_to_centroid=mean_dist_to_centroid,
        formation_error=formation_error,
        compactness=compactness,
        spread=spread,
        alignment_error=alignment_error,
        coordination_index=coordination_index,
        stability_score=stability_score
    )


def calculate_desired_formation_positions(
    centroid: Tuple[float, float],
    formation_shape: FormationShape,
    num_robots: int,
    scale: float = 1.0,
    rotation: float = 0.0
) -> List[Tuple[float, float]]:
    """
    Tính toán vị trí mong muốn cho đội hình chuẩn
    
    Args:
        centroid: Trọng tâm đội hình
        formation_shape: Loại đội hình
        num_robots: Số robot
        scale: Hệ số scale khoảng cách
        rotation: Góc xoay đội hình
    
    Returns:
        List vị trí mong muốn
    """
    cx, cy = centroid
    positions = []
    
    if formation_shape == FormationShape.LINE:
        # Xếp hàng ngang
        spacing = scale
        start_x = cx - (num_robots - 1) * spacing / 2
        for i in range(num_robots):
            x = start_x + i * spacing
            y = cy
            positions.append((x, y))
    
    elif formation_shape == FormationShape.TRIANGLE:
        # Tam giác đều
        if num_robots == 3:
            r = scale
            angles = [np.pi/2, np.pi/2 + 2*np.pi/3, np.pi/2 + 4*np.pi/3]
            for angle in angles:
                x = cx + r * np.cos(angle)
                y = cy + r * np.sin(angle)
                positions.append((x, y))
        else:
            # Fallback to line
            return calculate_desired_formation_positions(
                centroid, FormationShape.LINE, num_robots, scale, rotation
            )
    
    elif formation_shape == FormationShape.SQUARE:
        # Hình vuông
        if num_robots == 4:
            r = scale / np.sqrt(2)
            angles = [np.pi/4, 3*np.pi/4, 5*np.pi/4, 7*np.pi/4]
            for angle in angles:
                x = cx + r * np.cos(angle)
                y = cy + r * np.sin(angle)
                positions.append((x, y))
        else:
            return calculate_desired_formation_positions(
                centroid, FormationShape.LINE, num_robots, scale, rotation
            )
    
    elif formation_shape == FormationShape.DIAMOND:
        # Hình thoi
        if num_robots >= 4:
            r = scale
            angles = [0, np.pi/2, np.pi, 3*np.pi/2]
            for i, angle in enumerate(angles[:num_robots]):
                x = cx + r * np.cos(angle)
                y = cy + r * np.sin(angle)
                positions.append((x, y))
        else:
            return calculate_desired_formation_positions(
                centroid, FormationShape.LINE, num_robots, scale, rotation
            )
    
    else:
        # Custom - xếp vòng tròn
        r = scale
        for i in range(num_robots):
            angle = 2 * np.pi * i / num_robots
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            positions.append((x, y))
    
    # Apply rotation
    if rotation != 0:
        cos_r = np.cos(rotation)
        sin_r = np.sin(rotation)
        rotated_positions = []
        for x, y in positions:
            # Translate to origin
            x_rel = x - cx
            y_rel = y - cy
            # Rotate
            x_new = x_rel * cos_r - y_rel * sin_r + cx
            y_new = x_rel * sin_r + y_rel * cos_r + cy
            rotated_positions.append((x_new, y_new))
        positions = rotated_positions
    
    return positions
