import sys
from collections import deque, defaultdict

Pos = tuple[int, int]
EMPTY = -1
DO_NOTHING = -1
STATION = 0
RAIL_HORIZONTAL = 1
RAIL_VERTICAL = 2
RAIL_LEFT_DOWN = 3
RAIL_LEFT_UP = 4
RAIL_RIGHT_UP = 5
RAIL_RIGHT_DOWN = 6
COST_STATION = 5000
COST_RAIL = 100
STATION_AREA_DIR = [(-2, 0), (-1, -1), (-1, 0), (-1, 1), (0, -2), (0, -1), (0, 0), (0, 1), (0, 2), (1, -1), (1, 0), (1, 1), (2, 0)]


class UnionFind:
    def __init__(self, n: int):
        self.n = n
        self.parents = [-1 for _ in range(n * n)]

    def _find_root(self, idx: int) -> int:
        if self.parents[idx] < 0:
            return idx
        self.parents[idx] = self._find_root(self.parents[idx])
        return self.parents[idx]

    def is_same(self, p: Pos, q: Pos) -> bool:
        p_idx = p[0] * self.n + p[1]
        q_idx = q[0] * self.n + q[1]
        return self._find_root(p_idx) == self._find_root(q_idx)

    def unite(self, p: Pos, q: Pos) -> None:
        p_idx = p[0] * self.n + p[1]
        q_idx = q[0] * self.n + q[1]
        p_root = self._find_root(p_idx)
        q_root = self._find_root(q_idx)
        if p_root != q_root:
            p_size = -self.parents[p_root]
            q_size = -self.parents[q_root]
            if p_size > q_size:
                p_root, q_root = q_root, p_root
            self.parents[q_root] += self.parents[p_root]
            self.parents[p_root] = q_root


def distance(a: Pos, b: Pos) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class Action:
    def __init__(self, type: int, pos: Pos):
        self.type = type
        self.pos = pos

    def __str__(self):
        if self.type == DO_NOTHING:
            return "-1"
        else:
            return f"{self.type} {self.pos[0]} {self.pos[1]}"


class Result:
    def __init__(self, actions: list[Action], score: int):
        self.actions = actions
        self.score = score

    def __str__(self):
        return "\n".join(map(str, self.actions))


class Field:
    def __init__(self, N: int):
        self.N = N
        self.rail = [[EMPTY] * N for _ in range(N)]
        self.uf = UnionFind(N)
        self.station_list = set()

    def build(self, type: int, r: int, c: int) -> None:
        assert self.rail[r][c] != STATION
        if 1 <= type <= 6:
            assert self.rail[r][c] == EMPTY
        self.rail[r][c] = type
        if type == STATION:
            self.station_list.add((r, c))

        # 隣接する区画と接続
        # 上
        if type in (STATION, RAIL_VERTICAL, RAIL_LEFT_UP, RAIL_RIGHT_UP):
            if r > 0 and self.rail[r - 1][c] in (STATION, RAIL_VERTICAL, RAIL_LEFT_DOWN, RAIL_RIGHT_DOWN):
                self.uf.unite((r, c), (r - 1, c))
        # 下
        if type in (STATION, RAIL_VERTICAL, RAIL_LEFT_DOWN, RAIL_RIGHT_DOWN):
            if r < self.N - 1 and self.rail[r + 1][c] in (STATION, RAIL_VERTICAL, RAIL_LEFT_UP, RAIL_RIGHT_UP):
                self.uf.unite((r, c), (r + 1, c))
        # 左
        if type in (STATION, RAIL_HORIZONTAL, RAIL_LEFT_DOWN, RAIL_LEFT_UP):
            if c > 0 and self.rail[r][c - 1] in (STATION, RAIL_HORIZONTAL, RAIL_RIGHT_DOWN, RAIL_RIGHT_UP):
                self.uf.unite((r, c), (r, c - 1))
        # 右
        if type in (STATION, RAIL_HORIZONTAL, RAIL_RIGHT_DOWN, RAIL_RIGHT_UP):
            if c < self.N - 1 and self.rail[r][c + 1] in (STATION, RAIL_HORIZONTAL, RAIL_LEFT_DOWN, RAIL_LEFT_UP):
                self.uf.unite((r, c), (r, c + 1))

    def is_connected(self, s: Pos, t: Pos) -> bool:
        assert distance(s, t) > 4  # 前提条件
        stations0 = self.collect_stations(s)
        stations1 = self.collect_stations(t)
        for station0 in stations0:
            for station1 in stations1:
                if self.uf.is_same(station0, station1):
                    return True
        return False

    def collect_stations(self, pos: Pos) -> list[Pos]:
        stations = []
        for dr, dc in STATION_AREA_DIR:
            r = pos[0] + dr
            c = pos[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and self.rail[r][c] == STATION:
                stations.append((r, c))
        return stations
    
    def calc_rail_count(self, start: Pos, goal: Pos) -> tuple[int, int]:
        # bfsによって求める
        rail_copy = [[self.rail[i][j] for j in range(self.N)] for i in range(self.N)]
        rail_copy[start[0]][start[1]] = STATION        
        rail_copy[goal[0]][goal[1]] = STATION
        
        # start地点を区画に含む駅が存在するか
        queue = deque()
        used = [[(1e9, -1)] * self.N for _ in range(self.N)]# (建設コスト, 何日目に建設されるか)
        for dr, dc in STATION_AREA_DIR:
            if (dr == 0 and dc == 0):
                continue
            r = start[0] + dr
            c = start[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and rail_copy[r][c] == STATION:
                queue.append((r, c))
                used[r][c] = (0, 0)
        
        # goal地点を区画に含む駅が存在するか
        goal_list = []  
        for dr, dc in STATION_AREA_DIR:
            if (dr == 0 and dc == 0):
                continue
            r = goal[0] + dr
            c = goal[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and rail_copy[r][c] == STATION:
                goal_list.append((r, c))
                    
        queue.append(start)
        used[start[0]][start[1]] = (5000, 0)
        while(queue):
            r, c = queue.popleft()
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if (0 <= nr < self.N and 0 <= nc < self.N):
                    if rail_copy[nr][nc] == EMPTY:
                        if used[nr][nc][0] > used[r][c][0] + COST_RAIL:
                            used[nr][nc] = (used[r][c][0] + COST_RAIL, used[r][c][1] + 1)
                            queue.append((nr, nc))
                        
                    elif rail_copy[nr][nc] == STATION:
                        if (nr, nc) == goal:
                            if used[nr][nc][0] > used[r][c][0] + COST_STATION:
                                used[nr][nc] = (used[r][c][0] + COST_STATION, used[r][c][1] + 1)
                        else:
                            if used[nr][nc][0] > used[r][c][0]:
                                used[nr][nc] = used[r][c]
                                queue.append((nr, nc))
                            now_station_parent = self.uf._find_root(nr * self.N + nc)
                            for station_obj in self.station_list:
                                # 繋がっている駅がある場合
                                if station_obj != (nr, nc) and self.uf._find_root(station_obj[0] * self.N + station_obj[1]) == now_station_parent:
                                    if used[station_obj[0]][station_obj[1]][0] > used[r][c][0]:
                                        used[station_obj[0]][station_obj[1]] = used[r][c]
                                        queue.append(station_obj)
        
        best_val = used[goal[0]][goal[1]]
        for goal_obj in goal_list:
            if used[goal_obj[0]][goal_obj[1]][-1] == -1:
                continue
            if best_val[0] > used[goal_obj[0]][goal_obj[1]][0]:
                best_val = used[goal_obj[0]][goal_obj[1]]
        return best_val
    
    def calc_rail_route(self, start: Pos, goal: Pos) -> list[Pos]:
        # bfsをした後に、逆順に辿る
        rail_copy = [[self.rail[i][j] for j in range(self.N)] for i in range(self.N)]
        
        # start地点を区画に含む駅が存在するか
        # print(f"start = {start}", file=sys.stderr)
        # print(f"goal = {goal}", file=sys.stderr)
        queue = deque()
        start_list = [start]
        parent = {}
        parent[start] = None
        used = [[(1e9, -1)] * self.N for _ in range(self.N)]# (建設コスト, 何回目に訪れたか)
        for dr, dc in STATION_AREA_DIR:
            if (dr == 0 and dc == 0):
                continue
            r = start[0] + dr
            c = start[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and rail_copy[r][c] == STATION:
                queue.append((r, c))
                start_list.append((r, c))
                parent[(r,c)] = None
                used[r][c] = (0, 0)
        
        # goal地点を区画に含む駅が存在するか
        goal_list = []  
        for dr, dc in STATION_AREA_DIR:
            if (dr == 0 and dc == 0):
                continue
            r = goal[0] + dr
            c = goal[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and rail_copy[r][c] == STATION:
                goal_list.append((r, c))
                    
        queue.append(start)
        used[start[0]][start[1]] = (5000, 0)
        while(queue):
            r, c = queue.popleft()
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if (0 <= nr < self.N and 0 <= nc < self.N):
                    
                    if (nr, nc) == goal:
                        if used[nr][nc][0] > used[r][c][0] + COST_RAIL:
                                used[nr][nc] = (used[r][c][0] + COST_RAIL, used[r][c][1] + 1)
                                queue.append((nr, nc))
                                parent[(nr, nc)] = (r, c)
                                
                    elif rail_copy[nr][nc] == EMPTY:
                        if used[nr][nc][0] > used[r][c][0] + COST_RAIL:
                            used[nr][nc] = (used[r][c][0] + COST_RAIL, used[r][c][1] + 1)
                            queue.append((nr, nc))
                            parent[(nr, nc)] = (r, c)
                        
                    elif rail_copy[nr][nc] == STATION:
                        if used[nr][nc][0] > used[r][c][0]:
                            used[nr][nc] = (used[r][c][0], used[r][c][1] + 1)
                            queue.append((nr, nc))
                            parent[(nr, nc)] = (r, c)
                        now_station_parent = self.uf._find_root(nr * self.N + nc)
                        for station_obj in self.station_list:
                            # 繋がっている駅がある場合
                            if station_obj != (nr, nc) and self.uf._find_root(station_obj[0] * self.N + station_obj[1]) == now_station_parent:
                                if used[station_obj[0]][station_obj[1]][0] > used[nr][nc][0]:
                                    used[station_obj[0]][station_obj[1]] = (used[nr][nc][0], used[nr][nc][1] + 1)
                                    queue.append(station_obj)
                                    parent[station_obj] = (nr, nc)
                                    
        used[goal[0]][goal[1]] = (used[goal[0]][goal[1]][0] - COST_RAIL + COST_STATION, used[goal[0]][goal[1]][1])
        best_val = (used[goal[0]][goal[1]], goal)# (評価コスト, 位置)
        for goal_obj in goal_list:
            if used[goal_obj[0]][goal_obj[1]][1] == -1:
                continue
            if best_val[0][0] > used[goal_obj[0]][goal_obj[1]][0]:
                best_val = (used[goal_obj[0]][goal_obj[1]], goal_obj)
        
        parent_route = [best_val[1]]
        now = best_val[1]
        while(parent[now] != None):
            parent_route.append(parent[now])
            now = parent[now]
        return parent_route

class Solver:
    def __init__(self, N: int, M: int, K: int, T: int, home: list[Pos], workplace: list[Pos]):
        self.N = N
        self.M = M
        self.K = K
        self.T = T
        self.home = home
        self.workplace = workplace
        self.used_person = [False] * M
        self.next_person_candidate = [False] * M
        self.person_mapping = [[[] for _ in range(N)] for _ in range(N)]
        for i in range(M):
            self.person_mapping[home[i][0]][home[i][1]].append(i)
            self.person_mapping[workplace[i][0]][workplace[i][1]].append(i)
        self.field = Field(N)
        self.money = K
        self.income = 0
        self.actions = []
        
    def _check_connected(self, r0: int, c0: int, r1: int, c1: int, i:int) -> bool:
        r2, c2 = self.home[i]
        r3, c3 = self.workplace[i]
        if abs(r0-r2) + abs(c0-c2) <= 2:
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r0-r3) + abs(c0-c3) <= 2:
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r1-r2) + abs(c1-c2) <= 2:
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r1-r3) + abs(c1-c3) <= 2:
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        else:
            return False

    def calc_income(self, r0: int, c0: int, r1: int, c1: int) -> None:
        self.income = 0
        for i in range(self.M):
            if self.used_person[i] or self._check_connected(r0, c0, r1, c1, i):
                self.income += distance(self.home[i], self.workplace[i])

    def build_rail(self, type: int, r: int, c: int) -> None:
        while(self.money < COST_RAIL):
            self.build_nothing()
            self.money += self.income
        self.field.build(type, r, c)
        self.money -= COST_RAIL
        self.money += self.income
        self.actions.append(Action(type, (r, c)))

    def build_station(self, r: int, c: int) -> None:
        if self.field.rail[r][c] == STATION:
            return
        # print(f"build_station: {r} {c}", file=sys.stderr)
        while(self.money < COST_STATION):
            self.build_nothing()
            self.money += self.income
        self.field.build(STATION, r, c)
        self.money -= COST_STATION
        self.money += self.income
        self.actions.append(Action(STATION, (r, c)))

    def build_nothing(self) -> None:
        self.actions.append(Action(DO_NOTHING, (0, 0)))
        
    def _calclate_expected_income(self, r0, c0, r1, c1) -> int:
        expected_income = 0
        person_num = 0
        h = defaultdict(lambda: 0)
        for dr, dc in STATION_AREA_DIR:
            rr0 = r0 + dr
            cc0 = c0 + dc
            rr1 = r1 + dr
            cc1 = c1 + dc
            if 0 <= rr0 < self.N and 0 <= cc0 < self.N:
                for person_idx in self.person_mapping[rr0][cc0]:
                    person_num += 1
                    h[person_idx] += 1
                    if h[person_idx] == 2:
                        expected_income += distance(self.home[person_idx], self.workplace[person_idx])
                        person_num -= 1
            if 0 <= rr1 < self.N and 0 <= cc1 < self.N:
                for person_idx in self.person_mapping[rr1][cc1]:
                    person_num += 1
                    h[person_idx] += 1
                    if h[person_idx] == 2:
                        expected_income += distance(self.home[person_idx], self.workplace[person_idx])
                        person_num -= 1
        return expected_income, person_num
    
    def initial_build(self) -> bool:
        """
        初期段階で配置する駅と線路を配置する(home => workで各abs2以下のマス全探索)
        評価値：
            800ターン目に得られるコスト - 建設するコスト
        """
        remaining_rail_count = (self.K - COST_STATION * 2) // COST_RAIL
        best_eval = (-1,-1) # [現時点でのincome, stationに含まれるpersonの数]
        best_place = [] # [(), ()] 駅の座標が2つ
        for person_idx in range(self.M):
            dist = distance(self.home[person_idx], self.workplace[person_idx])
            if dist - 5 > remaining_rail_count:
                continue
            
            for start_dr, start_dc in STATION_AREA_DIR:
                start_r = self.home[person_idx][0] + start_dr
                start_c = self.home[person_idx][1] + start_dc
                if not (0 <= start_r < self.N and 0 <= start_c < self.N):
                    continue
                for goal_dr, goal_dc in STATION_AREA_DIR:
                    goal_r = self.workplace[person_idx][0] + goal_dr
                    goal_c = self.workplace[person_idx][1] + goal_dc
                    if not (0 <= goal_r < self.N and 0 <= goal_c < self.N):
                        continue
                    station_dist = distance((start_r, start_c), (goal_r, goal_c))
                    if station_dist - 1 > remaining_rail_count:
                        continue
                    expected_income, person_num = self._calclate_expected_income(start_r, start_c, goal_r, goal_c)
                    expected_money = ((797 - station_dist) * expected_income) - (COST_STATION * 2 + COST_RAIL * (station_dist-1))
                    if expected_money > 0:
                        if expected_income > best_eval[0]:
                            best_eval = (expected_income, person_num)
                            best_place = [(start_r, start_c), (goal_r, goal_c)]
                        elif expected_income == best_eval[0] and person_num > best_eval[1]:
                            best_eval = (expected_income, person_num)
                            best_place = [(start_r, start_c), (goal_r, goal_c)]
                   
        if best_eval[0] == -1:
            self.build_nothing()
            return False
        
        # 駅の配置
        self.build_station(*best_place[0])
        self.build_station(*best_place[1])
        
        # 線路を配置して駅を接続する
        r0, c0 = best_place[0]
        r1, c1 = best_place[1]
        # r0 -> r1
        if r0 < r1:
            for r in range(r0 + 1, r1):
                self.build_rail(RAIL_VERTICAL, r, c0)
            if c0 < c1:
                self.build_rail(RAIL_RIGHT_UP, r1, c0)
            elif c0 > c1:
                self.build_rail(RAIL_LEFT_UP, r1, c0)
        elif r0 > r1:
            for r in range(r0 - 1, r1, -1):
                self.build_rail(RAIL_VERTICAL, r, c0)
            if c0 < c1:
                self.build_rail(RAIL_RIGHT_DOWN, r1, c0)
            elif c0 > c1:
                self.build_rail(RAIL_LEFT_DOWN, r1, c0)
        # c0 -> c1
        if c0 < c1:
            for c in range(c0 + 1, c1):
                self.build_rail(RAIL_HORIZONTAL, r1, c)
        elif c0 > c1:
            for c in range(c0 - 1, c1, -1):
                self.build_rail(RAIL_HORIZONTAL, r1, c)
        
        self.calc_income(r0, c0, r1, c1)
        self.money += self.income
        return True
        
    def build_station_and_rail(self, home: Pos, workplace: Pos) -> None:
        # 線路を配置して駅を接続する
        route = self.field.calc_rail_route(home, workplace)
        for i in range(1, len(route)-1):
            pre0, pre1 = route[i-1]
            now0, now1 = route[i]
            nex0, nex1 = route[i+1]
            
            if self.field.rail[now0][now1] == STATION:
                continue
            
            if pre0 == now0 and pre1 < now1:#右
                if now1 < nex1:#右
                    self.build_rail(RAIL_HORIZONTAL, now0, now1)
                elif now0-1 == nex0:#上
                    self.build_rail(RAIL_LEFT_UP, now0, now1)
                elif now0+1 == nex0:#下
                    self.build_rail(RAIL_LEFT_DOWN, now0, now1)
            elif pre0 == now0 and pre1 > now1:#左
                if now1 > nex1:#左
                    self.build_rail(RAIL_HORIZONTAL, now0, now1)
                elif now0-1 == nex0:#上
                    self.build_rail(RAIL_RIGHT_UP, now0, now1)
                elif now0+1 == nex0:#下
                    self.build_rail(RAIL_RIGHT_DOWN, now0, now1)
            elif pre1 == now1 and pre0 < now0:#下
                if now0 < nex0:#下
                    self.build_rail(RAIL_VERTICAL, now0, now1)
                elif now1 < nex1:#右
                    self.build_rail(RAIL_RIGHT_UP, now0, now1)
                elif now1 > nex1:#左
                    self.build_rail(RAIL_LEFT_UP, now0, now1)
            elif pre1 == now1 and pre0 > now0:#上
                if now0 > nex0:#上
                    self.build_rail(RAIL_VERTICAL, now0, now1)
                elif now1 < nex1:#右
                    self.build_rail(RAIL_RIGHT_DOWN, now0, now1)
                elif now1 > nex1:#左
                    self.build_rail(RAIL_LEFT_DOWN, now0, now1)
                    
        # 駅の配置
        self.build_station(*route[0])
        self.build_station(*route[-1])
        
    def main_build(self) -> bool:
        """
        通常の建設
        評価値：
            800ターン目に得られるコスト - 建設するコスト
        """
        max_eval = (-1, -1e9, -1)# [最大の評価値, person_idx, 待たなきゃいけない時間]
        for person_idx in range(self.M):
            # 例外処理(既に使われている人 or 配置しても収益が得られない場合)
            if self.used_person[person_idx]:
                continue
            dist = distance(self.home[person_idx], self.workplace[person_idx])            
            build_cost, build_days = self.field.calc_rail_count(self.home[person_idx], self.workplace[person_idx])
            
            build_days = max(build_days, (build_cost - self.money + self.income - 1) // self.income)
            if build_days == -1 or build_days > self.T - len(self.actions):
                continue
            expected_income = (self.T - len(self.actions) - build_days) *  dist
            tmp_val = expected_income - build_cost
            if tmp_val > 0 and max_eval[1] < tmp_val:
                max_eval = (person_idx, tmp_val)
                   
        if max_eval[0] == -1:
            self.build_nothing()
            self.money += self.income
            return False

        # print(max_eval, file=sys.stderr)
                  
        # 配置する
        person_idx = max_eval[0]
        self.used_person[person_idx] = True
        self.build_station_and_rail(self.home[person_idx], self.workplace[person_idx])
        
        self.money -= self.income
        self.calc_income(*self.home[person_idx], *self.workplace[person_idx])
        self.money += self.income
        return True

    def solve(self) -> Result:
        # 初期処理
        if self.initial_build() and False:
            flag = False
            while len(self.actions) < self.T:
                if flag:
                    self.build_nothing()
                    self.money += self.income
                else:
                    while(self.main_build()):
                        pass
                    flag = True
            
        else:
            while len(self.actions) < self.T:
                self.build_nothing()

        return Result(self.actions, self.money)


def main():
    N, M, K, T = map(int, input().split())
    home = []
    workplace = []
    for _ in range(M):
        r0, c0, r1, c1 = map(int, input().split())
        home.append((r0, c0))
        workplace.append((r1, c1))

    solver = Solver(N, M, K, T, home, workplace)
    result = solver.solve()
    print(result)
    print(f"score={result.score}", file=sys.stderr)


if __name__ == "__main__":
    main()
