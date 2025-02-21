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
        self.used = [[(0, 0) for _ in range(self.N)] for _ in range(self.N)]
        self.used_pos = [[False] * N for _ in range(N)]
        self.parent = None

    def build(self, type: int, r: int, c: int) -> None:
        assert self.rail[r][c] != STATION
        if 1 <= type <= 6:
            assert self.rail[r][c] == EMPTY
        self.rail[r][c] = type
        if type == STATION:
            self.station_list.add((r, c))
            
    def setup_calc_expected_score(self) -> None:
        """
        bfsによって、self.usedを更新する
        """
        for i in range(self.N):
            for j in range(self.N):
                self.used_pos[i][j] = False
                self.used[i][j] = (0, 2600)
                
        # bfsによって求める
        queue = deque()
        self.parent = {}
        # start地点にはどうせ駅がある->すべての駅を(0,0)にすればいい
        for station_obj in self.station_list:
            queue.append(station_obj)
            self.used[station_obj[0]][station_obj[1]] = (0, 0)
            self.parent[station_obj] = None
            
        while(queue):
            r, c = queue.popleft()
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr = r + dr
                nc = c + dc
                if (0 <= nr < self.N and 0 <= nc < self.N):
                    if self.rail[nr][nc] == EMPTY:
                        if self.used[nr][nc][1] > self.used[r][c][1] + 1:
                            self.used[nr][nc] = (self.used[r][c][0] + COST_RAIL, self.used[r][c][1] + 1)
                            queue.append((nr, nc))
                            self.parent[(nr, nc)] = (r, c)
         
    
    def calc_expected_score(self, goal: Pos) -> tuple[int, int]:
        """
        start地点からgoal地点までの最適な経路を求める
        score_list = [((r, c), (build_cost, build_days)), ((r, c), (build_cost, build_days)), ...]
        """
        score_list = []                               
        for dr, dc in STATION_AREA_DIR:
            r = goal[0] + dr
            c = goal[1] + dc
            if 0 <= r < self.N and 0 <= c < self.N and (not self.used_pos[r][c]):
                self.used_pos[r][c] = True
                # railの上に駅を立てる場合
                if 1 <= self.rail[r][c] <= 6:
                    score_list.append(((r, c), (5000,1)))
                # 何もない場合
                elif self.used[r][c][1] != -1:
                    score_list.append(((r, c), (self.used[r][c][0] - COST_RAIL + COST_STATION, self.used[r][c][1])))
        return score_list
    
    def calc_rail_route(self, goal: Pos) -> list[Pos]:
        if 1 <= self.rail[goal[0]][goal[1]] <= 6:
            return [goal]
                
        parent_route = [goal]
        now = goal
        while(self.parent[now] != None):
            parent_route.append(self.parent[now])
            now = self.parent[now]
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
        self.ng_person_list = [False] * M
        
        # pos -> person_idx
        self.person_mapping = [[[] for _ in range(N)] for _ in range(N)]
        for i in range(M):
            self.person_mapping[home[i][0]][home[i][1]].append(i)
            self.person_mapping[workplace[i][0]][workplace[i][1]].append(i)
            if distance(home[i], workplace[i]) <= 10:
                self.ng_person_list[i] = True
            
        self.station_space = [[False] * N for _ in range(N)]
        self.field = Field(N)
        self.money = K
        self.income = 0
        self.actions = []

    def calc_income(self, r0: int, c0: int, r1: int, c1: int) -> None:
        self.income = 0
        neighbors = []
        h = defaultdict(lambda: 0)
        # r0, c0の駅に接続されている人
        for dr, dc in STATION_AREA_DIR:
            rr0 = r0 + dr
            cc0 = c0 + dc
            rr1 = r1 + dr
            cc1 = c1 + dc
            if 0 <= rr0 < self.N and 0 <= cc0 < self.N:
                neighbors.append((rr0, cc0))
            if 0 <= rr1 < self.N and 0 <= cc1 < self.N:
                neighbors.append((rr1, cc1))
                
        for r, c in neighbors:
            for i in self.person_mapping[r][c]:
                self.next_person_candidate[i] = True
                h[i] += 1
                if h[i] == 2:
                    self.income += distance(self.home[i], self.workplace[i])
                    self.used_person[i] = True
                
                
    def calc_income_main(self, r0: int, c0: int) -> None:
        self.income = 0
        for i in range(self.M):
            if self.used_person[i]:
                self.income += distance(self.home[i], self.workplace[i])
                self.used_person[i] = True
            else:
                if self.station_space[self.home[i][0]][self.home[i][1]]:
                    if self.station_space[self.workplace[i][0]][self.workplace[i][1]]:
                        self.used_person[i] = True
                        self.income += distance(self.home[i], self.workplace[i])
                    else:
                        self.next_person_candidate[i] = True
                else:
                    if self.station_space[self.workplace[i][0]][self.workplace[i][1]]:
                        self.next_person_candidate[i] = True
                        
                        
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
        while(self.money < COST_STATION):
            self.build_nothing()
            self.money += self.income
        for dr, dc in STATION_AREA_DIR:
            rr = r + dr
            cc = c + dc
            if 0 <= rr < self.N and 0 <= cc < self.N:
                self.station_space[rr][cc] = True
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
            for person_idx in self.person_mapping[rr0][cc0]:
                score = distance(self.home[person_idx], self.workplace[person_idx])
                if score >= 20:
                    person_num += 1
                h[person_idx] += 1
                if h[person_idx] == 2:
                    expected_income += score
                    person_num -= 2
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
        
    def build_main_section(self, goal: Pos) -> None:
        # 線路を配置して駅を接続する
        route = self.field.calc_rail_route(goal)
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
        remaining_turn = self.T - len(self.actions)
        best_eval = (-1,-1,-1) # incomeの増加分, stationに含まれるpersonの数, build_cost]
        best_place = None # (person_idx, (r, c))　駅の座標 
        self.field.setup_calc_expected_score()
        for person_idx in range(self.M):
            if self.used_person[person_idx] or not self.next_person_candidate[person_idx] or self.ng_person_list[person_idx]:
                continue
            
            # score_list = [((r, c), (build_cost, build_days)), ((r, c), (build_cost, build_days)), ...]
            if self.station_space[self.home[person_idx][0]][self.home[person_idx][1]]:
                score_list = self.field.calc_expected_score(self.workplace[person_idx])
            else:
                score_list = self.field.calc_expected_score(self.home[person_idx])
            
            # score: (expected_income, person_num, (r, c))
            for tmp_score in score_list:
                build_cost, build_days = tmp_score[1]
                build_days = max(build_days, (build_cost - self.money + self.income - 1) // self.income)
                if build_days > remaining_turn:
                    continue
                expected_income, person_num = self._calclate_expected_income_main(*tmp_score[0])
                expected_money = ((remaining_turn - build_days) * expected_income) - build_cost
                if remaining_turn <= 200:
                    # best_eval = (expected_money, build_days, expected_incomeにする)
                    if expected_money > best_eval[0]:
                        best_eval = (expected_money, build_days, expected_income)
                        best_place = (person_idx, tmp_score[0])
                    elif expected_money == best_eval[0]:
                        if build_days < best_eval[1]:
                            best_eval = (expected_money, build_days, expected_income)
                            best_place = (person_idx, tmp_score[0])
                        elif build_days == best_eval[1] and expected_income > best_eval[2]:
                            best_eval = (expected_money, build_days, expected_income)
                            best_place = (person_idx, tmp_score[0])
                        
                else:
                    if expected_money > 0:
                        if expected_income > best_eval[0]:
                            best_eval = (expected_income, person_num, build_cost)
                            best_place = (person_idx, tmp_score[0])
                        elif expected_income == best_eval[0]:
                            if person_num > best_eval[1]:
                                best_eval = (expected_income, person_num, build_cost)
                                best_place = (person_idx, tmp_score[0])
                            elif person_num == best_eval[1] and build_cost < best_eval[2]:
                                best_eval = (expected_income, person_num, build_cost)
                                best_place = (person_idx, tmp_score[0])
                                 
        if best_place is None:
            self.build_nothing()
            self.money += self.income
            return False

        # print(best_eval, file=sys.stderr)
                  
        # 配置する
        person_idx, best_place = best_place
        self.build_main_section(best_place)
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
