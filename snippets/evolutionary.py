import json
import math
import random
import time
from typing import List, Tuple, Optional
from aerialist.px4.aerialist_test import AerialistTest
from aerialist.px4.obstacle import Obstacle
from testcase import TestCase
from shapely.geometry import Polygon
from shapely.ops import unary_union


def find_plan_file(yaml_file: str):
    with open(yaml_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("mission_file:"):
                value = line.split(":", 1)[1].strip()
                value = value.split("#", 1)[0].strip().strip("'\"")
                return value
    raise ValueError(f"mission_file non trovato in {yaml_file}")


def load_trajectory(plan_file: str):
    with open(plan_file, "r") as f:
        plan = json.load(f)
    home_lat, home_lon, _ = plan["mission"]["plannedHomePosition"]
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(home_lat))
    waypoints = [(0.0, 0.0)]  
    for item in plan["mission"]["items"]:
        params = item.get("params", [])
        if len(params) >= 6 and params[4] is not None and params[5] is not None:
            x = (params[4] - home_lat) * m_per_deg_lat
            y = (params[5] - home_lon) * m_per_deg_lon
            waypoints.append((x, y))
    cleaned = [waypoints[0]]
    for p in waypoints[1:]:
        if math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 1.0:
            cleaned.append(p)
    return cleaned

def point_to_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def obstacle_distance_to_trajectory(obs: Obstacle, trajectory: List[Tuple[float, float]]):
    cx, cy = obs.position.x, obs.position.y
    half_diag = math.hypot(obs.size.l, obs.size.w) / 2.0
    min_d = float("inf")
    for i in range(len(trajectory) - 1):
        ax, ay = trajectory[i]
        bx, by = trajectory[i + 1]
        d_center = point_to_segment_distance(cx, cy, ax, ay, bx, by)
        d_edge = max(0.0, d_center - half_diag)
        if d_edge < min_d:
            min_d = d_edge
    return min_d


def trajectory_segment_midpoints(trajectory: List[Tuple[float, float]], n_segments: int):
    total_len = 0.0
    seg_lens = []
    for i in range(len(trajectory) - 1):
        L = math.hypot(trajectory[i + 1][0] - trajectory[i][0],
                       trajectory[i + 1][1] - trajectory[i][1])
        seg_lens.append(L)
        total_len += L
    target_distances = [(k + 0.5) * total_len / n_segments for k in range(n_segments)]
    midpoints = []
    for td in target_distances:
        acc = 0.0
        for i, L in enumerate(seg_lens):
            if acc + L >= td:
                t = (td - acc) / L if L > 0 else 0.0
                ax, ay = trajectory[i]
                bx, by = trajectory[i + 1]
                midpoints.append((ax + t * (bx - ax), ay + t * (by - ay)))
                break
            acc += L
    return midpoints


class ObstacleGenerator:
    # x: [-40, 30]  y: [10, 40]  l/w: [2, 20]  h: [25, 25]  r: [0, 90]
    min_size = Obstacle.Size(l=2, w=2, h=25)
    max_size = Obstacle.Size(l=20, w=20, h=25)
    min_position = Obstacle.Position(x=-40, y=10, z=0, r=0)
    max_position = Obstacle.Position(x=30, y=40, z=0, r=90)

    @staticmethod
    def create_random_obstacle():
        size = Obstacle.Size(
            l=random.uniform(ObstacleGenerator.min_size.l, ObstacleGenerator.max_size.l),
            w=random.uniform(ObstacleGenerator.min_size.w, ObstacleGenerator.max_size.w),
            h=ObstacleGenerator.max_size.h,
        )
        position = Obstacle.Position(
            x=random.uniform(ObstacleGenerator.min_position.x, ObstacleGenerator.max_position.x),
            y=random.uniform(ObstacleGenerator.min_position.y, ObstacleGenerator.max_position.y),
            z=0,
            r=random.uniform(ObstacleGenerator.min_position.r, ObstacleGenerator.max_position.r),
        )
        return Obstacle(size, position)

    @staticmethod
    def check_overlap(obstacles: List[Obstacle]):
        for i, obs1 in enumerate(obstacles):
            for obs2 in obstacles[i + 1:]:
                if ObstacleGenerator.is_overlapping(obs1, obs2):
                    return True
        return False

    @staticmethod
    def is_overlapping(obs1: Obstacle, obs2: Obstacle):
        return _obstacle_polygon(obs1).buffer(0.25).intersects(
               _obstacle_polygon(obs2).buffer(0.25))


def _clip(val, lo, hi):
    return max(lo, min(hi, val))


def mutate_obstacle(obs: Obstacle): 
    bp_min = ObstacleGenerator.min_position
    bp_max = ObstacleGenerator.max_position
    bs_min = ObstacleGenerator.min_size
    bs_max = ObstacleGenerator.max_size

    x, y, r = obs.position.x, obs.position.y, obs.position.r
    l, w, h = obs.size.l, obs.size.w, obs.size.h
    op = random.choice(["pos", "size", "rot"])
    if op == "pos":
        sx = 0.1 * (bp_max.x - bp_min.x)
        sy = 0.1 * (bp_max.y - bp_min.y)
        x = _clip(x + random.gauss(0, sx), bp_min.x, bp_max.x)
        y = _clip(y + random.gauss(0, sy), bp_min.y, bp_max.y)
    elif op == "size":
        sl = 0.1 * (bs_max.l - bs_min.l)
        sw = 0.1 * (bs_max.w - bs_min.w)
        l = _clip(l + random.gauss(0, sl), bs_min.l, bs_max.l)
        w = _clip(w + random.gauss(0, sw), bs_min.w, bs_max.w)
    else:  
        sr = 0.1 * (bp_max.r - bp_min.r)
        r = _clip(r + random.gauss(0, sr), bp_min.r, bp_max.r)

    return Obstacle(
        Obstacle.Size(l=l, w=w, h=h),
        Obstacle.Position(x=x, y=y, z=0, r=r),
    )


def mutate_config(obstacles: List[Obstacle]):
    idx = random.randrange(len(obstacles))
    new_obstacles = list(obstacles)
    new_obstacles[idx] = mutate_obstacle(obstacles[idx])
    return new_obstacles

def calculate_fitness(min_dist: float, n_obst: int, exec_time_min: float):
    if exec_time_min <= 0 or n_obst <= 0:
        return -100.0
    if min_dist < 1.5: 
        return (1.5 - min_dist) / (n_obst ** 2 * exec_time_min)
    else:
        return -(min_dist - 1.5)

def _obstacle_polygon(obs):
    cx, cy = obs.position.x, obs.position.y
    hx, hy = obs.size.l / 2.0, obs.size.w / 2.0
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    theta = math.radians(obs.position.r)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    rotated = [
        (cx + x * cos_t - y * sin_t, cy + x * sin_t + y * cos_t)
        for (x, y) in corners
    ]
    return Polygon(rotated)


def _test_area_polygon(test):
    polys = [_obstacle_polygon(o) for o in test.test.simulation.obstacles]
    if not polys:
        return Polygon()
    return unary_union(polys)


def _similarity(test_a, test_b):
    a = _test_area_polygon(test_a)
    b = _test_area_polygon(test_b)
    if a.is_empty or b.is_empty:
        return 0.0
    union_area = a.union(b).area
    if union_area <= 0:
        return 0.0
    return a.intersection(b).area / union_area


def deduplicate(test_cases, threshold: float = 0.99):
    kept = []
    for t in test_cases:
        if not any(_similarity(t, k) >= threshold for k in kept):
            kept.append(t)
    return kept

class Seed:
    def __init__(self, name: str, n_obstacles: int, init_anchor: Optional[Tuple[float, float]] = None):
        self.name = name
        self.n_obstacles = n_obstacles
        self.init_anchor = init_anchor
        self.parent_obstacles: Optional[List[Obstacle]] = None
        self.parent_fitness: float = -float("inf") 


    def update_if_better(self, child_obstacles: List[Obstacle],child_fitness: float):
        if child_fitness > self.parent_fitness:
            self.parent_obstacles = child_obstacles
            self.parent_fitness = child_fitness
            return True
        return False

class EvolutionaryGenerator:
    SAFETY_THRESHOLD = 1.5
    PREFILTER_THRESHOLD = 1.4 
    MAX_RETRIES = 1000
    INIT_RADIUS = 8.0          
    MAX_INIT_CONFIGURATION = 4 
    TIMEOUT_MIN = 8.0  

    def __init__(self, case_study_file: str, use_anchors: bool = True):
        self.case_study = AerialistTest.from_yaml(case_study_file)
        plan_file = find_plan_file(case_study_file)
        self.trajectory = load_trajectory(plan_file)
        self.use_anchors = use_anchors

        print(f"loaded trajectory from {plan_file}")
        print(f"{len(self.trajectory)} waypoint(s):")
        for i, (x, y) in enumerate(self.trajectory):
            print(f"    wp{i}: x={x:6.2f}  y={y:6.2f}")

        if use_anchors:
            anchors = trajectory_segment_midpoints(self.trajectory, 4)
            self.seeds = [
                Seed("seed_1 (2 obst)", n_obstacles=2, init_anchor=anchors[0]),
                Seed("seed_2 (2 obst)", n_obstacles=2, init_anchor=anchors[1]),
                Seed("seed_3 (2 obst)", n_obstacles=2, init_anchor=anchors[2]),
                Seed("seed_4 (2 obst)", n_obstacles=2, init_anchor=anchors[3])
            ]
            print("MODE: 4 seeds with anchor")
            print(f"Anchor: {[(round(a[0], 1), round(a[1], 1)) for a in anchors]}")
        else:
            self.seeds = [
                Seed("seed_1 (2 obst)", n_obstacles=2, init_anchor=None),
                Seed("seed_2 (2 obst)", n_obstacles=2, init_anchor=None),
                Seed("seed_3 (2 obst)", n_obstacles=2, init_anchor=None),
                Seed("seed_4 (2 obst)", n_obstacles=2, init_anchor=None)
            ]
            print("MODE: 4 seeds, random init (no anchor)")

        self.archive: List[TestCase] = []
        self.discarded_seeds: List[Tuple[List[Obstacle], TestCase, float]] = []


    def _in_anchor_zone(self, obstacle: Obstacle, anchor: Tuple[float, float]):
        ax, ay = anchor
        cx, cy = obstacle.position.x, obstacle.position.y
        return math.hypot(cx - ax, cy - ay) <= self.INIT_RADIUS

    
    def _config_valid(self, obstacles: List[Obstacle], seed: Optional["Seed"] = None):
        if ObstacleGenerator.check_overlap(obstacles):
            return False
        for o in obstacles:
            if obstacle_distance_to_trajectory(o, self.trajectory) >= self.PREFILTER_THRESHOLD:
                return False
        if seed is not None and seed.init_anchor is not None and len(obstacles) > 0:
            if not self._in_anchor_zone(obstacles[0], seed.init_anchor):
                return False
        return True

    def _random_obstacle_near(self, anchor: Tuple[float, float]):
        ax, ay = anchor
        bp_min = ObstacleGenerator.min_position
        bp_max = ObstacleGenerator.max_position

        size = Obstacle.Size(
            l=random.uniform(ObstacleGenerator.min_size.l, ObstacleGenerator.max_size.l),
            w=random.uniform(ObstacleGenerator.min_size.w, ObstacleGenerator.max_size.w),
            h=ObstacleGenerator.max_size.h
        )
        x = _clip(ax + random.uniform(-self.INIT_RADIUS, self.INIT_RADIUS),
                  bp_min.x, bp_max.x)
        y = _clip(ay + random.uniform(-self.INIT_RADIUS, self.INIT_RADIUS),
                  bp_min.y, bp_max.y)
        r = random.uniform(bp_min.r, bp_max.r)
        return Obstacle(size, Obstacle.Position(x=x, y=y, z=0, r=r))

    def _init_config(self, seed: Seed, seed_budget: int):
        best_obstacles = None
        best_test = None
        best_min_dist = float("inf")
        n_sims_used = 0
        max_init = min(self.MAX_INIT_CONFIGURATION, max(0, seed_budget))
        for sim_try in range(max_init):
            obstacles = None
            for _ in range(self.MAX_RETRIES):
                candidate = []
                for k in range(seed.n_obstacles):
                    if k == 0 and seed.init_anchor is not None:
                        candidate.append(self._random_obstacle_near(seed.init_anchor))
                    else:
                        candidate.append(ObstacleGenerator.create_random_obstacle())
                if self._config_valid(candidate, seed=seed):
                    obstacles = candidate
                    break
            
            if obstacles is None:
                print(f"No obstacoles generated")
                continue
            print(f"\nInit try {sim_try+1}/{self.MAX_INIT_CONFIGURATION}] Seed={seed.name}")
            test = self._execute(obstacles)
            n_sims_used += 1
            
            if test is None:
                continue
            
            print(f"init candidate: min_dist={test._min_dist:.3f}m")
            
            if test._min_dist < self.SAFETY_THRESHOLD:
                print(f"CHALLENGING, init accepted")
                return obstacles, test, n_sims_used
            if not self.use_anchors:
                self.discarded_seeds.append((obstacles, test, test._fitness))
            elif test._min_dist < best_min_dist:
                best_min_dist = test._min_dist
                best_obstacles = obstacles
                best_test = test

        if not self.use_anchors and self.discarded_seeds:
            best_idx = max(range(len(self.discarded_seeds)),
                           key=lambda i: self.discarded_seeds[i][2])
            best_obstacles, best_test, _ = self.discarded_seeds.pop(best_idx)
            print(f"fallback from global pool: min_dist={best_test._min_dist:.3f}m  "
                  f"({len(self.discarded_seeds)} left in the pool)")
        
        elif best_obstacles is not None:
            print(f"init: 0 test challenging found in {n_sims_used} sim, "
                f"selecting from pull, best scenario min_dist={best_min_dist:.3f}m")
        return best_obstacles, best_test, n_sims_used

    def _mutate_until_valid(self, parent: List[Obstacle]):
        for _ in range(self.MAX_RETRIES):
            child = mutate_config(parent)
            if self._config_valid(child):
                return child
        return None

    def _execute(self, obstacles: List[Obstacle]):
        test = TestCase(self.case_study, obstacles)
        try:
            t_start = time.time()
            test.execute()
            exec_time_min = (time.time() - t_start) / 60.0
            min_dist = min(test.get_distances())
            test.plot()

            if exec_time_min > self.TIMEOUT_MIN:
                print(f"TIME ({exec_time_min:.2f} > timeout {self.TIMEOUT_MIN} ) INVALID")
                return None
            test._min_dist = min_dist
            test._n_obstacles = len(obstacles)
            test._exec_time = exec_time_min
            test._fitness = calculate_fitness(min_dist, len(obstacles), exec_time_min)
            return test
        except Exception as e:
            print(f"exception: {e}")
            return None



    def generate(self, budget: int):
        n_seed = len(self.seeds)
        budget_per_seed = budget // n_seed
        remainder = budget - budget_per_seed * n_seed
        budgets = [budget_per_seed] * n_seed
        budgets[0] += remainder
        global_test_idx = 0
        for idx, seed in enumerate(self.seeds):
            seed_budget = budgets[idx]
            print(f"\n\nSeed {idx + 1}/{n_seed}: {seed.name}  (budget {seed_budget})")
            init_obstacles, test, init_sims = self._init_config(seed, seed_budget)
            if init_obstacles is None or test is None:
                print("init failed, skipping seed")
                continue

            global_test_idx += init_sims
            print(f"init used {init_sims}/{self.MAX_INIT_CONFIGURATION} sim")
            print(f"min_dist={test._min_dist:.3f}m  fitness={test._fitness:.3f}")

            seed.update_if_better(init_obstacles, test._fitness)
            if test._min_dist < self.SAFETY_THRESHOLD:
                self.archive.append(test)
                print("CHALLENGING, saved")

            n_mutations = seed_budget - init_sims
            for step in range(n_mutations):
                global_test_idx += 1
                child = self._mutate_until_valid(seed.parent_obstacles)
                if child is None:
                    print(f"[test {global_test_idx}/{budget}] mutation: no valid child, skip")
                    continue

                print(f"\n[test {global_test_idx}/{budget}] step {step + 1}"
                f"/{n_mutations}  seed={seed.name}")
                test = self._execute(child)
                if test is None:
                    continue

                replaced = seed.update_if_better(child, test._fitness)
                tag = "BETTER" if replaced else "worse "
                print(f"min_dist={test._min_dist:.3f}m  "
                      f"fitness={test._fitness:.3f}  parent={seed.parent_fitness:.3f}"
                      f"  [{tag}]")

                if test._min_dist < self.SAFETY_THRESHOLD:
                    self.archive.append(test)
                    print("CHALLENGING, saved")
        self.archive.sort(key=lambda t: (t._min_dist, t._n_obstacles))
        n_before = len(self.archive)
        self.archive = deduplicate(self.archive)
        print(f"{n_before-len(self.archive)} removed")
        print(f"\nSUITE FINALE: {len(self.archive)}challenging test ")
        for i, t in enumerate(self.archive):
            print(f"  {i + 1:>3}. min_dist={t._min_dist:.3f}m  "
                  f"#obst={t._n_obstacles}  time={t._exec_time:.2f}min")

        return self.archive

if __name__ == "__main__":
    ANCORE = False
    BUDGET = 60
    generator = EvolutionaryGenerator("case_studies/mission3.yaml", ANCORE)
    test_cases = generator.generate(BUDGET)
    print(f"\n{len(test_cases)} test challenging generated.")

