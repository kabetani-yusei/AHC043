import sys

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

    def build(self, type: int, r: int, c: int) -> None:
        assert self.rail[r][c] != STATION
        if 1 <= type <= 6:
            assert self.rail[r][c] == EMPTY
        self.rail[r][c] = type

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
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                if abs(dr) + abs(dc) > 2:
                    continue
                r = pos[0] + dr
                c = pos[1] + dc
                if 0 <= r < self.N and 0 <= c < self.N and self.rail[r][c] == STATION:
                    stations.append((r, c))
        return stations
    
    def calc_rail_count(self, start: Pos, goal: Pos) -> int:
        # bfsによって求める
        rail_copy = [[self.rail[i][j] for j in range(self.N)] for i in range(self.N)]
        rail_copy[start[0]][start[1]] = STATION
        rail_copy[goal[0]][goal[1]] = STATION
        used = [[-1] * self.N for _ in range(self.N)]
        queue = [start]
        used[start[0]][start[1]] = 0
        while(queue):
            r, c = queue.pop(0)
            if (r, c) == goal:
                return used[r][c] - 1
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if ((0 <= nr < self.N and 0 <= nc < self.N and used[nr][nc] == -1) and 
                    (rail_copy[nr][nc] == EMPTY or rail_copy[nr][nc] == STATION)):
                    used[nr][nc] = used[r][c] + 1
                    queue.append((nr, nc))
        return -1
    
    def _calc_rail_route(self, start: Pos, goal: Pos) -> list[Pos]:
        # bfsをした後に、逆順に辿る
        rail_copy = [[self.rail[i][j] for j in range(self.N)] for i in range(self.N)]
        rail_copy[start[0]][start[1]] = STATION
        rail_copy[goal[0]][goal[1]] = STATION
        used = [[-1] * self.N for _ in range(self.N)]
        queue = [start]
        used[start[0]][start[1]] = 0
        while(queue):
            r, c = queue.pop(0)
            if (r, c) == goal:
                break
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if ((0 <= nr < self.N and 0 <= nc < self.N and used[nr][nc] == -1) and 
                    (rail_copy[nr][nc] == EMPTY or rail_copy[nr][nc] == STATION)):
                    used[nr][nc] = used[r][c] + 1
                    queue.append((nr, nc))
        
        route = [goal]
        now = goal
        while(now != start):
            r, c = now
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if (0 <= nr < self.N and 0 <= nc < self.N and used[nr][nc] == used[r][c] - 1):
                    route.append((nr, nc))
                    now = (nr, nc)
                    break
                
        route.reverse()
        return route
    
    def build_station_and_rail(self, home: Pos, workplace: Pos) -> None:
        # 駅の配置
        if self.rail[home[0]][home[1]] != STATION:
            self.build(STATION, *home)
        if self.rail[workplace[0]][workplace[1]] != STATION:
            self.build(STATION, *workplace)
        
        # 線路を配置して駅を接続する
        route = self._calc_rail_route(home, workplace)
        for i in range(1, len(route)-1):
            pre0, pre1 = route[i-1]
            now0, now1 = route[i]
            nex0, nex1 = route[i+1]
            
            if self.rail[now0][now1] != EMPTY:
                continue
            
            if pre0 == now0 and pre1 < now1:#右
                if now1 < nex1:#右
                    self.build(RAIL_HORIZONTAL, now0, now1)
                elif now0-1 == nex0:#上
                    self.build(RAIL_LEFT_UP, now0, now1)
                elif now0+1 == nex0:#下
                    self.build(RAIL_LEFT_DOWN, now0, now1)
            elif pre0 == now0 and pre1 > now1:#左
                if now1 > nex1:#左
                    self.build(RAIL_HORIZONTAL, now0, now1)
                elif now0-1 == nex0:#上
                    self.build(RAIL_RIGHT_UP, now0, now1)
                elif now0+1 == nex0:#下
                    self.build(RAIL_RIGHT_DOWN, now0, now1)
            elif pre1 == now1 and pre0 < now0:#下
                if now0 < nex0:#下
                    self.build(RAIL_VERTICAL, now0, now1)
                elif now1 < nex1:#右
                    self.build(RAIL_RIGHT_UP, now0, now1)
                elif now1 > nex1:#左
                    self.build(RAIL_LEFT_UP, now0, now1)
            elif pre1 == now1 and pre0 > now0:#上
                if now0 > nex0:#上
                    self.build(RAIL_VERTICAL, now0, now1)
                elif now1 < nex1:#右
                    self.build(RAIL_RIGHT_DOWN, now0, now1)
                elif now1 > nex1:#左
                    self.build(RAIL_LEFT_DOWN, now0, now1)

class Solver:
    def __init__(self, N: int, M: int, K: int, T: int, home: list[Pos], workplace: list[Pos]):
        self.N = N
        self.M = M
        self.K = K
        self.T = T
        self.home = home
        self.workplace = workplace
        self.used_person = [False] * M
        self.field = Field(N)
        self.money = K
        self.income = 0
        self.actions = []

    def calc_income(self) -> int:
        # todo 差分更新できる
        income = 0
        for i in range(self.M):
            if self.field.is_connected(self.home[i], self.workplace[i]):
                income += distance(self.home[i], self.workplace[i])
        return income

    def build_rail(self, type: int, r: int, c: int) -> None:
        self.field.build(type, r, c)
        self.money -= COST_RAIL
        self.actions.append(Action(type, (r, c)))

    def build_station(self, r: int, c: int) -> None:
        self.field.build(STATION, r, c)
        self.money -= COST_STATION
        self.actions.append(Action(STATION, (r, c)))

    def build_nothing(self) -> None:
        self.actions.append(Action(DO_NOTHING, (0, 0)))
    
    def initial_build(self) -> None:
        """
        初期段階で配置する駅と線路を配置する
        評価値：
            800ターン目に得られるコスト - 建設するコスト
        """
        rail_count = (self.K - COST_STATION * 2) // COST_RAIL
        max_eval = [-1, -1e9]# [person_idx, 最大の評価値]
        for person_idx in range(self.M):
            dist = distance(self.home[person_idx], self.workplace[person_idx])
            build_rail_count = dist - 1
            if build_rail_count > rail_count:
                continue
            
            build_days = 2 + build_rail_count
            build_cost = COST_STATION * 2 + COST_RAIL * build_rail_count
            expected_income = (self.T - build_days) *  dist
            
            increase_check = expected_income - build_cost
            if increase_check <= 0:
                continue
            
            tmp_val = expected_income - build_cost
            if max_eval[1] < tmp_val:
                max_eval = [person_idx, tmp_val]
                   
        if max_eval[0] == -1:
            self.build_nothing()
            return
        
        # 配置する
        person_idx = max_eval[0]
        self.used_person[person_idx] = True
        
        # 駅の配置
        self.build_station(*self.home[person_idx])
        self.build_station(*self.workplace[person_idx])
        
        # 線路を配置して駅を接続する
        r0, c0 = self.home[person_idx]
        r1, c1 = self.workplace[person_idx]
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
        
        self.income = self.calc_income()
        self.money += self.income
        
    def main_build(self) -> None:
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
            build_rail_count = dist - 1
            build_cost = COST_STATION * 2 + COST_RAIL * build_rail_count
            build_days = max(2 + build_rail_count, (build_cost - self.money + self.income - 1) // self.income)
            expected_income = (self.T - len(self.actions) - build_days) *  dist
            increase_check = expected_income - build_cost
            if increase_check <= 0:
                continue
            
            build_rail_count = self.field.calc_rail_count(self.home[person_idx], self.workplace[person_idx])
            if build_rail_count == -1:
                continue
            build_cost = COST_STATION * 2 + COST_RAIL * build_rail_count
            build_days = max(2 + build_rail_count, (build_cost - self.money + self.income - 1) // self.income)
            expected_income = (self.T - len(self.actions) - build_days) *  dist
            increase_check = expected_income - build_cost
            if increase_check <= 0:
                continue
            
            tmp_val = expected_income - build_cost
            if max_eval[1] < tmp_val:
                wait_times = max(0, ((build_cost - self.money + self.income - 1) // self.income) - (2 + build_rail_count))
                max_eval = (person_idx, tmp_val, wait_times)
                   
        if max_eval[0] == -1:
            self.build_nothing()
            self.money += self.income
            return
        
        # 待ち時間の消費をする
        for _ in range(max_eval[2]):
            self.build_nothing()
            self.money += self.income
            
        # 配置する
        person_idx = max_eval[0]
        self.used_person[person_idx] = True
        self.field.build_station_and_rail(self.home[person_idx], self.workplace[person_idx])
        
        income = self.calc_income()
        self.money += income

    def solve(self) -> Result:
        # 初期処理
        self.initial_build()

        # メイン処理
        while len(self.actions) < self.T:
            self.main_build()

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
