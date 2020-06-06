import os
import sys
import math
import random
import functools
from matplotlib import pyplot as plt
import csv
import copy


class PSO_VRPTW:
    trunk_amount = None  # 货车数量
    trunk_volumes = None  # 货车各自的容量，共k辆车
    target_amount = None  # 目的地数量
    target_sites = None  # 各目的地坐标,共有L+1个,[0]为起点坐标
    target_time_limits = None  # 各目的地时间限制[begin, end]
    target_volumes = None  # 各目的地要运的货总量 # max(trunk_volumes)>max(target_volumes)
    target_service_times = None  # 停留时长
    dist = None  # 邻接矩阵
    losses = []

    omega = 0.4  # 惯性因子
    c1, c2 = 0.1, 0.5  # 加速因子
    n = 10  # 总粒子数
    dot_v = None  # 各粒子的速度,element [Vv,Vr]对应solution
    dot_bests = None  # 各粒子的最好成绩
    dot_solutions = None  # 一个2L维的空间对应有L个发货点任务的VRP问题,每个发货点任务对应两维:完成该任务车辆的编号k该任务在k车行驶路径中的次序r
    best_solution = None  # 最好方案[Xv各任务车辆编号，Xr各任务在对应车辆路径执行次序]

    color = ['#00FFFF', '#7FFFD4', '#0000FF', '#8A2BE2', '#A52A2A', '#DEB887', '#FFFF00', '#9ACD32', '#008000',
             '#5F9EA0', '#7FFF00', '#D2691E', '#FF7F50', '#6495ED', '#DC143C', '#00FFFF', '#008B8B', '#B8860B',
             '#A9A9A9', '#006400', '#BDB76B', '#8B008B', '#556B2F', '#FF8C00', '#9932CC', '#8B0000', '#E9967A',
             '#8FBC8F', '#483D8B', '#2F4F4F', '#00CED1', '#9400D3', '#FF1493', '#00BFFF', '#696969', '#1E90FF',
             '#B22222', '#FFFAF0', '#228B22', '#FF00FF', '#DCDCDC', '#F8F8FF', '#FFD700', '#DAA520', '#808080',
             '#FF69B4', '#CD5C5C', '#4B0082', '#FFFFF0', '#F0E68C', '#E6E6FA', '#FFF0F5', '#7CFC00', '#FFFACD',
             '#ADD8E6', '#F08080', '#E0FFFF', '#FAFAD2', '#90EE90', '#D3D3D3', '#FFB6C1', '#FFA07A', '#20B2AA',
             '#87CEFA', '#778899', '#B0C4DE', '#FFFFE0', '#00FF00', '#32CD32', '#FAF0E6', '#FF00FF', '#800000',
             '#66CDAA', '#0000CD', '#BA55D3', '#9370DB', '#3CB371', '#7B68EE', '#00FA9A', '#48D1CC', '#C71585',
             '#191970', '#F5FFFA', '#FFE4E1', '#FFE4B5', '#FFDEAD', '#000080', '#FDF5E6', '#808000', '#6B8E23',
             '#FFA500', '#FF4500', '#DA70D6', '#EEE8AA', '#98FB98', '#AFEEEE', '#DB7093', '#FFEFD5', '#FFDAB9',
             '#CD853F', '#FFC0CB', '#DDA0DD', '#B0E0E6', '#800080', '#FF0000', '#BC8F8F', '#4169E1', '#8B4513',
             '#FA8072', '#FAA460', '#2E8B57', '#FFF5EE', '#A0522D', '#C0C0C0', '#87CEEB', '#6A5ACD', '#708090',
             '#FFFAFA', '#00FF7F', '#4682B4', '#D2B48C', '#008080', '#D8BFD8', '#FF6347', '#40E0D0', '#EE82EE',
             '#F5DEB3', '#FFFFFF', '#F5F5F5']
    rate = 0.10

    def __init__(self, trunk_volumes: list, target_sites: list
                 , target_time_limits: list, target_volumes: list, target_service_times):
        self.trunk_volumes = trunk_volumes
        self.target_sites = target_sites
        self.target_time_limits = target_time_limits
        self.target_volumes = target_volumes
        self.target_service_times = target_service_times
        self.trunk_amount = len(trunk_volumes)
        self.target_amount = len(target_volumes)
        #######
        self.n = 10 * len(target_service_time)  # 对于有时间窗，粒子数一般取10倍
        self.dot_solutions = [[[], []] for i in range(self.n)]  # element len=2*L
        self.dot_bests = [None for i in range(self.n)]  # L
        self.dot_v = [None for i in range(self.n)]  # 2*L
        self.dist = [(lambda x: [math.sqrt(math.pow(self.target_sites[0][x] - self.target_sites[0][y], 2)
                                           + math.pow(self.target_sites[1][x] - self.target_sites[1][y], 2))
                                 for y in range(self.target_amount)])(i)
                     for i in range(self.target_amount)]  # 邻接矩阵

    def cost(self, solution):
        """ 计算方案solution的代价大小 """
        # 满足条件：1.总容量不超过汽车容量  2.在服务时间之内
        PE, PL = 10, 100  # pe为时间成本，pl为罚金成本
        max_time = 0  # k辆车中耗时最长的
        sum_dist = 0  # 总距离
        penalty = 0  # 惩罚
        for k in range(self.trunk_amount):  # 分别计算第k辆车的代价
            pre_pos, pre_time = 0, 0  # 之前的位置（初始在原点），上一步结束的时间
            k_time = 0  # 花费时间
            counter = 0
            for i in range(1, len(solution[0])):
                # solution[0] 完成该任务车辆k编号    solution[1] 该任务在的车辆次序
                if solution[0][i] == k:  # 正好是第k辆车处理的订单
                    k_time += self.dist[pre_pos][i]
                    sum_dist += self.dist[pre_pos][i]
                    counter += 1
                    if self.target_time_limits[0][i] > k_time and counter > 1:  # 说明可以提前到达(TODO:默认速度1m/s)
                        k_time = self.target_time_limits[0][i]
                        penalty += PE * (self.target_time_limits[0][i] - k_time)
                    elif self.target_time_limits[1][i] < k_time:  # 说明迟到了(TODO:默认速度1m/s)
                        penalty += PL * (k_time - self.target_time_limits[1][i])
                    k_time += self.target_service_times[i]
                    pre_pos = i  # 记录位置
            k_time += self.dist[pre_pos][0]  # 回到起点
            sum_dist += self.dist[pre_pos][0]
            max_time = max_time if k_time < max_time else k_time
        return sum_dist + penalty  # TODO:由于sum_time暗含了sum_dist，其实只返回sum_time即可。至于回到原点的cost，不必计入

    def recode_solution(self, solution: list):
        # 由于[3,1,2,1]与[3,2,1,2]  [1,2,3,2]方案实际上相同（数字代表车辆编号），需要进行重编码，编码从0开始
        temp = copy.deepcopy(solution[0])
        i, mapp = 0, dict()
        counter = [1 for i in range(self.trunk_amount)]
        for i in range(self.target_amount):
            if solution[0][i] not in mapp:
                mapp[solution[0][i]] = len(mapp)
        for i in range(self.target_amount):  # 替换编码
            solution[1][i] = counter[mapp[solution[0][i]]]
            counter[mapp[solution[0][i]]] += 1
            solution[0][i] = mapp[solution[0][i]]
        return

    def init_solution(self):
        for i in range(self.n):
            # dot i
            self.dot_solutions[i][0].clear()
            self.dot_solutions[i][1].clear()
            self.dot_solutions[i][0].append(0)  # 代表起点
            self.dot_solutions[i][1].append(0)  # 代表起点
            mapp = dict()
            order = [0 for i in range(self.trunk_amount)]
            volumes = [0 for i in range(self.trunk_amount)]  # 记录k辆车的容量
            for j in range(1, self.target_amount):
                No_ = random.randint(0, self.trunk_amount - 1)  # 代表抽中车的序号
                counter = 0
                if No_ not in mapp:  # 如果不在map中，生成对应的车辆编号映射
                    mapp[No_] = len(mapp)
                while volumes[No_] + self.target_volumes[j] > self.trunk_volumes[No_]:  # TODO:如果超出容量限制，继续抽奖
                    No_ = random.randint(0, self.trunk_amount - 1)
                    counter += 1
                    if No_ not in mapp:  # 如果不在map中，生成对应的车辆编号映射
                        mapp[No_] = len(mapp)
                    if counter > 0xffff:
                        print('车辆容量严重不足，无法装载所有货物，无解')
                        return -1
                self.dot_solutions[i][0].append(mapp[No_])  # mapp[No_]接下第j个任务
                order[mapp[No_]] += 1  # 顺序从1开始记 i.e. 1,2,3,4,5...
                self.dot_solutions[i][1].append(order[mapp[No_]])  # 记录顺序
                volumes[No_] += self.target_volumes[j]  # 汽车容量++

    def update_best_solution(self):
        # 更新全局最优解
        min_cost = 0x7fffffff if self.best_solution is None else self.cost(self.best_solution)
        pos, i = 0, 0
        while i < len(self.dot_solutions):
            # print('solution[', i, ']=', self.cost(self.dot_solutions[i]))
            if self.cost(self.dot_solutions[i]) < min_cost:
                min_cost = self.cost(self.dot_solutions[i])
                pos = i  # 记录best solution position
            i += 1
        self.losses.append(min_cost)
        if pos != 0:
            self.best_solution = copy.deepcopy(self.dot_solutions[pos])

    def update_dot_best(self):
        # 更新每个粒子的历史最优值
        for i in range(self.n):
            if (self.dot_bests[i] is None) or self.cost(self.dot_solutions[i]) < self.cost(self.dot_bests[i]):
                self.dot_bests[i] = copy.deepcopy(self.dot_solutions[i])

    def draw_pic(self):
        # 画结果
        plt.subplot(2, 2, 1)
        for i in range(self.trunk_amount):
            # if i % 2 == 0:
            #     plt.subplot(2, 2, 1 + i // 2)
            # 原点
            plt.scatter(self.target_sites[0][0], self.target_sites[1][0], color='', marker='o', edgecolors='b', s=50)
            pre = 0
            for j in range(1, len(self.best_solution[0])):
                if self.best_solution[0][j] == i:
                    plt.plot([self.target_sites[0][pre], self.target_sites[0][j]],
                             [self.target_sites[1][pre], self.target_sites[1][j]],
                             color=self.color[i])
                    pre = j
            plt.plot([self.target_sites[0][0], self.target_sites[0][pre]],
                     [self.target_sites[1][0], self.target_sites[1][pre]],
                     color=self.color[i])
        plt.title('Cost=' + str(self.cost(self.best_solution)))
        plt.subplot(2, 2, 2)  # losses
        plt.plot(self.losses)
        plt.subplot(2, 1, 2)  # 时间轴
        for k in range(self.trunk_amount):
            # 第k辆车
            y = k * 10  # 纵轴
            time, pre, counter = 0, 0, 0
            for j in range(1, self.target_amount):
                if self.best_solution[0][j] == k:
                    time += self.dist[pre][j]
                    time = time if self.target_time_limits[0][j] < time else self.target_time_limits[0][j]
                    plt.scatter(time, y, color='', marker='o', edgecolors='b', s=50)
                    time += self.target_service_times[j]
                    plt.plot([self.target_time_limits[0][j], self.target_time_limits[1][j]], [y, y]
                             , color='r', linewidth=2.5)
                    plt.scatter(self.target_time_limits[0][j], y, color='', marker='o'
                                , edgecolors=self.color[counter], s=35)
                    plt.scatter(self.target_time_limits[1][j], y, color='', marker='o'
                                , edgecolors=self.color[counter], s=35)
                    pre = j
                    counter += 1
            plt.plot([0, time], [y, y], color='b', linewidth=1)
        plt.show()

    def dot_improve(self):
        """ 每一回合粒子向最优方案靠近，呃，这里用dot指代粒子 """
        sum_factor = self.omega + self.c1 + self.c2
        selection = [self.omega / sum_factor, (self.omega + self.c1) / sum_factor, 1]
        for i in range(self.n):
            # 更新每一个dot的solution
            temp_volumes = [0 for i in range(self.trunk_amount)]  # 方案累积容量
            for j in range(self.target_amount):
                # 朝最佳方案逼近
                num = random.random()
                if num > selection[1]:
                    self.dot_solutions[i][0][j] = self.best_solution[0][j]
                elif num > selection[2]:
                    self.dot_solutions[i][0][j] = self.dot_bests[i][0][j]
                if random.random() < self.rate:  # 变异因子
                    self.dot_solutions[i][0][j] = random.randint(0, len(self.trunk_volumes) - 1)
                while temp_volumes[self.dot_solutions[i][0][j]] + self.target_volumes[j] \
                        > self.trunk_volumes[self.dot_solutions[i][0][j]]:
                    # TODO:如果超出容量限制,只能随机一辆车。不会无限循环，否则init_solution()处就会报错
                    self.dot_solutions[i][0][j] = random.randint(0, len(self.trunk_volumes) - 1)
                temp_volumes[self.dot_solutions[i][0][j]] += self.target_volumes[j]
        # 必须满足货车容量>=任务要求，尽量满足[ET,LT]之间送货

    def run(self):
        """ 粒子群算法的入口 """
        # 粒子群划分为两两重叠的相邻子群
        # TODO:每个粒子solution初始化,可能方案无法做到K辆车都用上?
        self.init_solution()
        # 每个粒子速度v
        # 用cost评价各粒子效果
        self.dot_bests = copy.deepcopy(self.dot_solutions)  # 替换各粒子最优solution
        self.update_best_solution()  # 更新best solution
        # 迭代epoch轮
        for epoch in range(400):
            if epoch % 10 == 0:
                print('Best cost:', self.cost(self.best_solution))
            if epoch % 100 == 0:
                self.rate /= 2
            # 更新solution
            self.dot_improve()
            # 重新编码
            for i in self.dot_solutions:
                self.recode_solution(i)
            # update best solution
            self.update_dot_best()
            self.update_best_solution()
        print("Best solution:\n", self.best_solution[0], '\n', self.best_solution[1])
        self.draw_pic()


def read_in_data(path: str, row_num):
    def func_cmp(a, b):
        if a[3] == b[3]:
            return a[4] - b[4]
        return a[3] - b[3]
    # TODO:数据先按时间排序
    data = [[[], []], [], [[], []], []]  # 坐标，容量，时间起止，服务时长
    data_in = []
    with open(path) as file:
        csvf = csv.reader(file)
        csvf.__next__()  # 标题
        for i in range(row_num):
            line = csvf.__next__()  # 一行数据
            ele = [int(line[1]), int(line[2]), int(line[3]), int(line[4]), int(line[5]), int(line[6])]
            data_in.append(ele)
    data_in = sorted(data_in, key=functools.cmp_to_key(func_cmp))
    for i in data_in:
        data[0][0].append(i[0])  # 坐标x
        data[0][1].append(i[1])  # 坐标y
        data[1].append(i[2])  # 所需容量
        data[2][0].append(i[3])  # 时间起点
        data[2][1].append(i[4])  # 时间终点
        data[3].append(i[5])  # 停留时长
    return data


if __name__ == "__main__":
    data = read_in_data('./input/rc101.csv', 51)  # 读取1原点+50个订单=51
    trunk_volume = [200 for i in range(13)]  # 默认200，共10辆车
    target_site = data[0]
    target_volume = data[1]
    target_time_limit = data[2]
    target_service_time = data[3]
    pso = PSO_VRPTW(trunk_volume, target_site, target_time_limit, target_volume, target_service_time)
    pso.run()  # 计算解
    # TODO:循环使用车辆？
