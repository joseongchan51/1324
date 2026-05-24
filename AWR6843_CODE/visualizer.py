import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge

from config import VIEW_X_MIN, VIEW_X_MAX, VIEW_Y_MIN, VIEW_Y_MAX

AWR_AZIMUTH_MIN_DEG = -60.0
AWR_AZIMUTH_MAX_DEG = 60.0
AWR_BORESIGHT_DEG = 90.0
LANE_WIDTH_M = 0.5
LANE_COUNT = 3
EGO_LANE_INDEX = 2


def _get_config_value(name, default):
    try:
        import config  # type: ignore

        return float(getattr(config, name, default))
    except Exception:
        return float(default)


def _resolve_fov_config():
    az_min = _get_config_value("AWR_AZIMUTH_MIN_DEG", AWR_AZIMUTH_MIN_DEG)
    az_max = _get_config_value("AWR_AZIMUTH_MAX_DEG", AWR_AZIMUTH_MAX_DEG)
    boresight = _get_config_value("AWR_BORESIGHT_DEG", AWR_BORESIGHT_DEG)

    default_range_max = max(0.0, VIEW_Y_MAX)
    range_max = _get_config_value("AWR_RANGE_MAX_M", default_range_max)
    return az_min, az_max, boresight, max(0.0, range_max)


def _draw_fov_overlay(ax, az_min_deg, az_max_deg, boresight_deg, range_max_m):
    theta1 = boresight_deg + az_min_deg
    theta2 = boresight_deg + az_max_deg

    fov_patch = Wedge(
        center=(0.0, 0.0),
        r=range_max_m,
        theta1=theta1,
        theta2=theta2,
        facecolor="lightgreen",
        alpha=0.15,
        edgecolor="green",
        linewidth=1.2,
        linestyle="--",
        zorder=0,
        label="AWR6843 FOV",
    )
    ax.add_patch(fov_patch)


def _resolve_lane_config():
    lane_width = max(0.01, _get_config_value("LANE_WIDTH_M", LANE_WIDTH_M))
    lane_count = max(1, int(round(_get_config_value("LANE_COUNT", LANE_COUNT))))
    ego_lane_index = int(round(_get_config_value("EGO_LANE_INDEX", EGO_LANE_INDEX)))
    ego_lane_index = min(max(1, ego_lane_index), lane_count)
    return lane_width, lane_count, ego_lane_index


def _get_lane_edges(lane_width_m, lane_count, ego_lane_index):
    return np.array(
        [
            (lane_index - ego_lane_index - 0.5) * lane_width_m
            for lane_index in range(1, lane_count + 2)
        ],
        dtype=float,
    )


def _draw_lane_overlay(ax, lane_width_m, lane_count, ego_lane_index, lane_edges):
    y_span = VIEW_Y_MAX - VIEW_Y_MIN
    if y_span <= 0:
        return

    road_left = lane_edges[0]
    road_right = lane_edges[-1]
    road_width = road_right - road_left

    ax.add_patch(
        Rectangle(
            (road_left, VIEW_Y_MIN),
            road_width,
            y_span,
            facecolor="#5f6770",
            alpha=0.12,
            edgecolor="none",
            zorder=0.1,
        )
    )

    ego_left = lane_edges[ego_lane_index - 1]
    ax.add_patch(
        Rectangle(
            (ego_left, VIEW_Y_MIN),
            lane_width_m,
            y_span,
            facecolor="#42a5f5",
            alpha=0.12,
            edgecolor="none",
            zorder=0.2,
        )
    )

    for edge_index, edge_x in enumerate(lane_edges):
        is_outer_edge = edge_index == 0 or edge_index == len(lane_edges) - 1
        ax.plot(
            [edge_x, edge_x],
            [VIEW_Y_MIN, VIEW_Y_MAX],
            color="#30343a" if is_outer_edge else "#70757d",
            linewidth=1.4 if is_outer_edge else 1.1,
            linestyle="-" if is_outer_edge else (0, (6, 6)),
            alpha=0.85,
            label="Lane boundary" if edge_index == 0 else "_nolegend_",
            zorder=1.2,
        )

    ego_center_x = (lane_edges[ego_lane_index - 1] + lane_edges[ego_lane_index]) / 2.0
    ax.plot(
        [ego_center_x, ego_center_x],
        [VIEW_Y_MIN, VIEW_Y_MAX],
        color="#1565c0",
        linewidth=1.2,
        linestyle=":",
        label=f"Ego center (Lane {ego_lane_index})",
        zorder=1.3,
    )

    label_y = VIEW_Y_MAX - (0.06 * y_span)
    for lane_index in range(1, lane_count + 1):
        center_x = (lane_edges[lane_index - 1] + lane_edges[lane_index]) / 2.0
        label = f"L{lane_index}" + (" ego" if lane_index == ego_lane_index else "")
        ax.text(
            center_x,
            label_y,
            label,
            ha="center",
            va="top",
            fontsize=8,
            color="#20242a",
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.65, edgecolor="none"),
            zorder=2,
        )


def visualize_points(fig, ax, df, labels, x, y, num_detected_obj, cluster_objects, velocity_obj):
    print("\033c", end="")
    print("===== AWR6843 Detected Objects =====")
    print(f"Detected objects(header): {num_detected_obj}")
    print(df)

    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    labels = np.asarray(labels)
    velocity_obj = np.asarray(velocity_obj, dtype=float)
    if velocity_obj.size and velocity_obj.ndim == 1:
        velocity_obj = velocity_obj.reshape(1, -1)

    lane_width_m, lane_count, ego_lane_index = _resolve_lane_config()
    lane_edges = _get_lane_edges(lane_width_m, lane_count, ego_lane_index)

    unique_labels = sorted(set(labels.tolist())) if labels.size else []
    cluster_count = len([lb for lb in unique_labels if lb != -1])
    noise_count = int(np.sum(labels == -1)) if labels.size else 0

    print(f"Clusters(DBSCAN): {cluster_count}, Noise points: {noise_count}")
    print(f"Velocity filter pass objects: {len(velocity_obj)}")
    print(f"Lane setup: {lane_count} lanes x {lane_width_m:.2f} m, ego lane {ego_lane_index}")

    ax.clear()

    az_min_deg, az_max_deg, boresight_deg, range_max_m = _resolve_fov_config()
    _draw_fov_overlay(ax, az_min_deg, az_max_deg, boresight_deg, range_max_m)
    _draw_lane_overlay(ax, lane_width_m, lane_count, ego_lane_index, lane_edges)

    abs_deg = np.degrees(np.arctan2(y, x))
    rel_deg = ((abs_deg - boresight_deg + 180.0) % 360.0) - 180.0
    distances = np.hypot(x, y)

    in_fov_mask = (
        (rel_deg >= az_min_deg)
        & (rel_deg <= az_max_deg)
        & (distances <= range_max_m)
        & (y >= 0.0)
    )

    if np.any(in_fov_mask):
        sc = ax.scatter(
            x[in_fov_mask],
            y[in_fov_mask],
            s=30,
            c=labels[in_fov_mask],
            cmap="tab20",
            zorder=2,
        )

        if not hasattr(fig, "_awr_colorbar"):
            fig._awr_colorbar = plt.colorbar(sc, ax=ax)
            fig._awr_colorbar.set_label("Cluster ID (-1: noise)")
        else:
            fig._awr_colorbar.update_normal(sc)

    if np.any(~in_fov_mask):
        ax.scatter(
            x[~in_fov_mask],
            y[~in_fov_mask],
            s=35,
            c="gray",
            marker="x",
            alpha=0.85,
            label="Out of FOV",
            zorder=3,
        )

    if cluster_objects:
        ax.scatter(
            [obj["x"] for obj in cluster_objects],
            [obj["y"] for obj in cluster_objects],
            s=80,
            c="red",
            marker="s",
            edgecolors="black",
            linewidths=1.0,
            label="Centroid",
            zorder=5,
        )

    if velocity_obj.size != 0 and velocity_obj.shape[1] >= 3:
        ax.scatter(
            velocity_obj[:, 1],
            velocity_obj[:, 2],
            s=140,
            c="deepskyblue",
            marker="*",
            edgecolors="black",
            linewidths=1.2,
            label="Velocity Filter Pass",
            zorder=8,
        )

    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.set_title(f"AWR6843 Position / DBSCAN Cluster (Ego Lane {ego_lane_index})")
    ax.grid(True)
    ax.set_xlim(VIEW_X_MIN, VIEW_X_MAX)
    ax.set_ylim(VIEW_Y_MIN, VIEW_Y_MAX)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right")

    plt.pause(0.001)
