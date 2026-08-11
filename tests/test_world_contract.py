from collections import deque

import pytest

from conftest import SRC, parse_xml


WORLD = SRC / "sw01_gazebo" / "worlds" / "sw01_maze.world"
CELL_SIZE = 0.1
CLEARANCE = 0.50
GRID_MIN = -15.0
GRID_MAX = 15.0

EXPECTED_WALLS = {
    "wall_b_n": (0.0, 15.0, 30.2, 0.2),
    "wall_b_s": (0.0, -15.0, 30.2, 0.2),
    "wall_b_e": (15.0, 0.0, 0.2, 30.2),
    "wall_b_w": (-15.0, 0.0, 0.2, 30.2),
    "wall_h01": (-8.0, -10.0, 10.0, 0.2),
    "wall_h02": (4.0, -10.0, 8.0, 0.2),
    "wall_h03": (11.5, -10.0, 3.0, 0.2),
    "wall_v01": (-8.0, -6.5, 0.2, 7.0),
    "wall_v02": (-8.0, 5.0, 0.2, 14.0),
    "wall_h04": (-11.0, -2.0, 4.0, 0.2),
    "wall_h05": (-2.5, -2.0, 9.0, 0.2),
    "wall_h06": (9.0, -2.0, 8.0, 0.2),
    "wall_v03": (-2.0, -8.0, 0.2, 4.0),
    "wall_v04": (-2.0, 4.5, 0.2, 11.0),
    "wall_v05": (-2.0, 12.0, 0.2, 2.0),
    "wall_h07": (-7.0, 4.0, 12.0, 0.2),
    "wall_h08": (5.0, 4.0, 10.0, 0.2),
    "wall_v06": (4.0, -6.0, 0.2, 8.0),
    "wall_v07": (4.0, 9.0, 0.2, 8.0),
    "wall_h09": (-9.0, 10.0, 8.0, 0.2),
    "wall_h10": (3.0, 10.0, 12.0, 0.2),
    "wall_h11": (12.0, 10.0, 2.0, 0.2),
    "wall_v08": (10.0, -7.0, 0.2, 6.0),
    "wall_v09": (10.0, 4.0, 0.2, 10.0),
}


def wall_boxes(root):
    result = []
    for model in root.findall(".//model"):
        if not model.attrib.get("name", "").startswith("wall_"):
            continue
        pose = [float(value) for value in model.findtext("pose").split()]
        size = [
            float(value)
            for value in model.findtext("link/collision/geometry/box/size").split()
        ]
        result.append((model.attrib["name"], pose, size))
    return result


def model_box(root, name):
    model = root.find(f".//model[@name='{name}']")
    assert model is not None
    pose = [float(value) for value in model.findtext("pose").split()]
    size = [
        float(value)
        for value in model.findtext("link/collision/geometry/box/size").split()
    ]
    return model, pose, size


def rasterized_obstacles(walls):
    cells = int(round((GRID_MAX - GRID_MIN) / CELL_SIZE)) + 1
    occupied = [[False] * cells for _ in range(cells)]
    for _, pose, size in walls:
        half_x = size[0] / 2 + CLEARANCE
        half_y = size[1] / 2 + CLEARANCE
        for row in range(cells):
            y = GRID_MIN + row * CELL_SIZE
            if abs(y - pose[1]) > half_y + 1e-9:
                continue
            for column in range(cells):
                x = GRID_MIN + column * CELL_SIZE
                if abs(x - pose[0]) <= half_x + 1e-9:
                    occupied[row][column] = True
    return occupied


def grid_cell(x, y):
    return (
        int(round((y - GRID_MIN) / CELL_SIZE)),
        int(round((x - GRID_MIN) / CELL_SIZE)),
    )


def free_neighbors(occupied, cell):
    row, column = cell
    cells = len(occupied)
    for row_delta, column_delta in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        next_row, next_column = row + row_delta, column + column_delta
        if (
            0 <= next_row < cells
            and 0 <= next_column < cells
            and not occupied[next_row][next_column]
        ):
            yield next_row, next_column


def test_world_scale_walls_lights_and_markers():
    root = parse_xml(WORLD)
    walls = wall_boxes(root)

    assert root.attrib["version"] == "1.6"
    assert root.findtext("world/gravity") == "0 0 -9.8"
    assert root.findtext("world/scene/ambient") == "0.8 0.8 0.8 1"
    assert len(walls) == 24
    assert all(abs(size[2] - 1.5) < 1e-9 for _, _, size in walls)
    assert all(min(size[0], size[1]) == pytest.approx(0.2) for _, _, size in walls)
    assert len(root.findall(".//light")) >= 2
    assert root.find(".//light[@name='sun']").attrib["type"] == "directional"
    point_light = root.find(".//light[@name='maze_point_light']")
    assert point_light is not None and point_light.attrib["type"] == "point"
    assert [float(value) for value in point_light.findtext("pose").split()[:3]] == pytest.approx(
        (0.0, 0.0, 12.0)
    )

    ground, ground_pose, ground_size = model_box(root, "ground")
    assert ground.findtext("static") == "true"
    assert ground_pose == pytest.approx((0.0, 0.0, -0.05, 0.0, 0.0, 0.0))
    assert ground_size == pytest.approx((30.4, 30.4, 0.1))

    for name, expected_pose, expected_colour in (
        ("start_zone", (-12.0, -12.0, 0.006), "0 1 0 1"),
        ("goal_zone", (12.0, 12.0, 0.006), "0 0 1 1"),
    ):
        marker, pose, size = model_box(root, name)
        assert marker.findtext("static") == "true"
        assert pose[:3] == pytest.approx(expected_pose)
        assert size == pytest.approx((2.0, 2.0, 0.01))
        assert marker.findtext("link/visual/material/diffuse") == expected_colour


def test_wall_geometry_matches_approved_30_meter_maze_layout():
    root = parse_xml(WORLD)
    actual = {name: (pose[0], pose[1], size[0], size[1]) for name, pose, size in wall_boxes(root)}
    assert set(actual) == set(EXPECTED_WALLS)
    for name, expected in EXPECTED_WALLS.items():
        assert actual[name] == pytest.approx(expected)

    for name in EXPECTED_WALLS:
        model, _, collision_size = model_box(root, name)
        visual_size = [
            float(value)
            for value in model.findtext("link/visual/geometry/box/size").split()
        ]
        assert collision_size == pytest.approx(visual_size)
        assert model.findtext("static") == "true"
        assert model.findtext("link/visual/material/script/name") == "Gazebo/Grey"
        assert model.findtext("link/visual/material/diffuse").split()[-1] == "1"


def test_inflated_wall_geometry_allows_start_to_goal_route():
    occupied = rasterized_obstacles(wall_boxes(parse_xml(WORLD)))
    start, goal = grid_cell(-12.0, -12.0), grid_cell(12.0, 12.0)
    assert not occupied[start[0]][start[1]]
    assert not occupied[goal[0]][goal[1]]

    visited = {start}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        for neighbor in free_neighbors(occupied, cell):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)

    assert goal in visited


def test_inflated_maze_has_three_separated_junction_regions():
    occupied = rasterized_obstacles(wall_boxes(parse_xml(WORLD)))
    cells = len(occupied)
    junction_cells = {
        (row, column)
        for row in range(cells)
        for column in range(cells)
        if not occupied[row][column]
        and len(tuple(free_neighbors(occupied, (row, column)))) >= 3
    }
    regions = 0
    while junction_cells:
        regions += 1
        queue = deque([junction_cells.pop()])
        while queue:
            cell = queue.popleft()
            for neighbor in free_neighbors(occupied, cell):
                if neighbor in junction_cells:
                    junction_cells.remove(neighbor)
                    queue.append(neighbor)

    assert regions >= 3
