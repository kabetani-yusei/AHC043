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
        self.station_list = set()

    def build(self, type: int, r: int, c: int) -> None:
        assert self.rail[r][c] != STATION
        if 1 <= type <= 6:
            assert self.rail[r][c] == EMPTY
        self.rail[r][c] = type
        if type == STATION:
            self.station_list.add((r, c))

    def is_connected(self, s: Pos, t: Pos) -> bool:
        return self.collect_stations(s) and self.collect_stations(t)

    def collect_stations(self, pos: Pos) -> list[Pos]:
        for dr, dc in STATION_AREA_DIR:
            r = pos[0] + dr
            c = pos[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and self.rail[r][c] == STATION:
                return True
        return False
    
    def calc_expected_score(self, start: Pos, goal: Pos) -> tuple[int, int]:
        """
        start地点からgoal地点までの最適な経路を求める
        score_list = [((r, c), (build_cost, build_days)), ((r, c), (build_cost, build_days)), ...]
        """
        # bfsによって求める
        rail_copy = [[self.rail[i][j] for j in range(self.N)] for i in range(self.N)]
        
        # start地点を区画に含む駅が存在するか
        queue = deque()
        used = [[(int(1e9), -1) for _ in range(self.N)] for _ in range(self.N)]# (建設コスト, 何日目に訪れたか)
        
        # start地点にはどうせ駅がある->すべての駅を(0,0)にすればいい
        for station_obj in self.station_list:
            queue.append(station_obj)
            used[station_obj[0]][station_obj[1]] = (0, 0)
            
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
                        elif used[nr][nc][0] == used[r][c][0] + COST_RAIL and used[nr][nc][1] > used[r][c][1] + 1:
                            used[nr][nc] = (used[r][c][0] + COST_RAIL, used[r][c][1] + 1)
                            queue.append((nr, nc))
         
        score_list = []                               
        for dr, dc in STATION_AREA_DIR:
            r = goal[0] + dr
            c = goal[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N:
                # railの上に駅を立てる場合
                if 1 <= rail_copy[r][c] <= 6:
                    score_list.append(((r, c), (5000,1)))
                # 何もない場合
                elif used[r][c][1] != -1:
                    score_list.append(((r, c), (used[r][c][0] - COST_RAIL + COST_STATION, used[r][c][1])))
        return score_list
    
    def calc_rail_route(self, start: Pos, goal: Pos) -> list[Pos]:
        # bfsによってgoalからstartに向かって辿る
        parent = {}
        rail_copy = [[self.rail[i][j] for j in range(self.N)] for i in range(self.N)]
        
        # railの上に駅を立てる場合の例外処理
        if 1 <= rail_copy[goal[0]][goal[1]] <= 6:
            return [goal]
                
        queue = deque()
        used = [[int(1e9) for _ in range(self.N)] for _ in range(self.N)]# 何日目に訪れたか
        queue.append(goal)
        used[goal[0]][goal[1]] = 0
        parent[goal] = None
        
        station_score_list = []# [(r,c, 何日目に訪れたか), ...]

        while(queue):
            r, c = queue.popleft()
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if (0 <= nr < self.N and 0 <= nc < self.N):
                    if rail_copy[nr][nc] == EMPTY:
                        if used[nr][nc] > used[r][c] + 1:
                            used[nr][nc] = used[r][c] + 1
                            queue.append((nr, nc))
                            parent[(nr, nc)] = (r, c)
                        
                    elif rail_copy[nr][nc] == STATION:
                        if used[nr][nc] > used[r][c] + 1:
                            used[nr][nc] = used[r][c] + 1
                            queue.append((nr, nc))
                            parent[(nr, nc)] = (r, c)
                            station_score_list.append((nr, nc, used[nr][nc]))
                            
        # start地点を区画に含む駅
        best_score = int(1e9) # 建設コスト、何日目に訪れたか
        best_station = None
        for station_obj in station_score_list:
            if station_obj[2] < best_score:
                best_score = station_obj[2]
                best_station = (station_obj[0], station_obj[1])
        
        parent_route = [best_station]
        now = best_station
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
        self.station_space = [[False] * N for _ in range(N)]
        self.field = Field(N)
        self.money = K
        self.income = 0
        self.actions = []
        
    def _check_connected(self, r0: int, c0: int, r1: int, c1: int, i:int) -> bool:
        r2, c2 = self.home[i]
        r3, c3 = self.workplace[i]
        if abs(r0-r2) + abs(c0-c2) <= 2:
            self.station_space[r2][c2] = True
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r0-r3) + abs(c0-c3) <= 2:
            self.station_space[r3][c3] = True
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r1-r2) + abs(c1-c2) <= 2:
            self.station_space[r2][c2] = True
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r1-r3) + abs(c1-c3) <= 2:
            self.station_space[r3][c3] = True
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        else:
            return False
        
    def _check_connected_main(self, r0: int, c0: int, i:int) -> bool:
        r2, c2 = self.home[i]
        r3, c3 = self.workplace[i]
        if abs(r0-r2) + abs(c0-c2) <= 2:
            self.station_space[r2][c2] = True
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        elif abs(r0-r3) + abs(c0-c3) <= 2:
            self.station_space[r3][c3] = True
            self.next_person_candidate[i] = True
            return self.field.is_connected(self.home[i], self.workplace[i])
        else:
            return False

    def calc_income(self, r0: int, c0: int, r1: int, c1: int) -> None:
        self.income = 0
        for i in range(self.M):
            if self.used_person[i] or self._check_connected(r0, c0, r1, c1, i):
                self.income += distance(self.home[i], self.workplace[i])
                self.used_person[i] = True
                
    def calc_income_main(self, r0: int, c0: int) -> None:
        self.income = 0
        for i in range(self.M):
            if self.used_person[i] or self._check_connected_main(r0, c0, i):
                self.income += distance(self.home[i], self.workplace[i])
                self.used_person[i] = True

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
        self.station_space[r][c] = True
        self.field.build(STATION, r, c)
        self.money -= COST_STATION
        self.money += self.income
        self.actions.append(Action(STATION, (r, c)))

    def build_nothing(self) -> None:
        self.actions.append(Action(DO_NOTHING, (0, 0)))
        
    def _calclate_expected_income_initial(self, r0, c0, r1, c1) -> int:
        expected_income = 0
        person_num = 0
        h = defaultdict(lambda: 0)
        a = set()
        
        for dr, dc in STATION_AREA_DIR:
            rr0 = r0 + dr
            cc0 = c0 + dc
            rr1 = r1 + dr
            cc1 = c1 + dc
            if 0 <= rr0 < self.N and 0 <= cc0 < self.N:
                a.add((rr0, cc0))
            if 0 <= rr1 < self.N and 0 <= cc1 < self.N:
                a.add((rr1, cc1))
            
        for rr0, cc0 in list(a):
            if 0 <= rr0 < self.N and 0 <= cc0 < self.N:
                for person_idx in self.person_mapping[rr0][cc0]:
                    person_num += 1
                    h[person_idx] += 1
                    if h[person_idx] == 2:
                        expected_income += distance(self.home[person_idx], self.workplace[person_idx])
                        person_num -= 1
        assert expected_income != 0
        return expected_income, person_num
    
    def initial_build(self) -> bool:
        """
        初期段階で配置する駅と線路を配置する(home => workで各abs2以下のマス全探索)
        評価値：
            (income, stationに含まれる人の数)
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
                    expected_income, person_num = self._calclate_expected_income_initial(start_r, start_c, goal_r, goal_c)
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
        
    def build_main_section(self, start: Pos, goal: Pos) -> None:
        # 線路を配置して駅を接続する
        route = self.field.calc_rail_route(start, goal)
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
        
    def _calclate_expected_income_main(self, r0, c0) -> int:
        expected_income = 0
        person_num = 0
        for dr, dc in STATION_AREA_DIR:
            r = r0 + dr
            c = c0 + dc
            if 0 <= r < self.N and 0 <= c < self.N:
                if len(self.person_mapping[r][c]) == 0 or self.station_space[r][c]:
                    continue
                for person_idx in self.person_mapping[r][c]:
                    person_num += 1
                    if ((self.home[person_idx] == (r, c) and self.station_space[self.workplace[person_idx][0]][self.workplace[person_idx][1]])
                        or (self.workplace[person_idx] == (r, c) and self.station_space[self.home[person_idx][0]][self.home[person_idx][1]])):
                            expected_income += distance(self.home[person_idx], self.workplace[person_idx])
        assert expected_income != 0
        return expected_income, person_num
        
    def main_build(self) -> bool:
        """
        通常の建設
        評価値：
            (income, stationに含まれる人の数)
        """
        best_eval = (-1,-1) # 現時点でのincome, stationに含まれるpersonの数]
        best_place = None # (person_idx, (r, c))　駅の座標 
        
        for person_idx in range(self.M):
            if self.used_person[person_idx] or not self.next_person_candidate[person_idx]:
                continue
            
            # score_list = [((r, c), (build_cost, build_days)), ((r, c), (build_cost, build_days)), ...]
            if self.station_space[self.home[person_idx][0]][self.home[person_idx][1]]:
                score_list = self.field.calc_expected_score(self.home[person_idx], self.workplace[person_idx])
            else:
                score_list = self.field.calc_expected_score(self.workplace[person_idx], self.home[person_idx])
            
            # score: (expected_income, person_num, (r, c))
            for tmp_score in score_list:
                build_days = max(tmp_score[1][1], (tmp_score[1][0] - self.money + self.income - 1) // self.income)
                if tmp_score[1][1] == -1 or build_days > self.T - len(self.actions):
                    continue
                expected_income, person_num = self._calclate_expected_income_main(*tmp_score[0])
                expected_money = ((self.T - len(self.actions) - build_days) * expected_income) - tmp_score[1][0]
                if expected_money > 0:
                    if expected_income > best_eval[0]:
                        best_eval = (expected_income, person_num)
                        best_place = (person_idx, tmp_score[0])
                    elif expected_income == best_eval[0] and person_num > best_eval[1]:
                        best_eval = (expected_income, person_num)
                        best_place = (person_idx, tmp_score[0])
                   
        if best_place is None:
            self.build_nothing()
            self.money += self.income
            return False

        # print(best_eval, file=sys.stderr)
                  
        # 配置する
        person_idx, best_place = best_place
        if self.station_space[self.home[person_idx][0]][self.home[person_idx][1]]:
            self.build_main_section(self.home[person_idx], best_place)
        else:
            self.build_main_section(self.workplace[person_idx], best_place)
        
        self.money -= self.income
        self.calc_income_main(*best_place)
        self.money += self.income
        return True

    def solve(self) -> Result:
        # 初期処理
        if self.initial_build():
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
                self.money += self.income

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
