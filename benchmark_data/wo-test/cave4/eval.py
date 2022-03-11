import xlrd
import scipy.io
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# matplotlib.use('qt5agg')


def get_values_at_t(time_split, value_split, t):
    values_t = np.zeros(len(time_split))
    for j in range(0, len(time_split)):
        time_j = time_split[j]
        value_j = value_split[j]

        N = time_j.size
        if t < time_j[0]:
            values_t[j] = np.nan
        elif t >= time_j[N - 1]:
            values_t[j] = value_j[N - 1]
        else:
            for k in range(1, N):
                if t < time_j[k]:
                    values_t[j] = value_j[k - 1]
                    break
                
    return values_t


def get_cost_mean_var_list(book, eval_timeline):
    sheet = book.sheet_by_index(0)
    num_rows = sheet.nrows
    num_cols = sheet.ncols

    idx_all = np.array(sheet.col_values(0))
    time_all = np.array(sheet.col_values(1))
    cost_all = np.array(sheet.col_values(2))


    split_idx = []
    for i in range(1, idx_all.size):
        if idx_all[i] == 1:
            split_idx.append(i)

    idx_split = np.hsplit(idx_all, split_idx)
    time_split = np.hsplit(time_all, split_idx)
    cost_split = np.hsplit(cost_all, split_idx)

    cost_mean = np.zeros(eval_timeline.size)
    cost_var = np.zeros(eval_timeline.size)
    for i in range(0, eval_timeline.size):
        costs_i = get_values_at_t(time_split, cost_split, eval_timeline[i])
        cost_mean[i] = np.nanmean(costs_i)
        cost_var[i] = np.nanstd(costs_i)

    return cost_mean, cost_var

def get_dura_mean_var_list(book, eval_timeline):
    sheet = book.sheet_by_index(0)

    idx_all = np.array(sheet.col_values(0))
    time_all = np.array(sheet.col_values(1))
    dura_all = np.array(sheet.col_values(3))

    split_idx = []
    for i in range(1, idx_all.size):
        if idx_all[i] == 1:
            split_idx.append(i)

    time_split = np.hsplit(time_all, split_idx)
    dura_split = np.hsplit(dura_all, split_idx)

    dura_mean = np.zeros(eval_timeline.size)
    dura_var = np.zeros(eval_timeline.size)
    for i in range(0, eval_timeline.size):
        duras_i = get_values_at_t(time_split, dura_split, eval_timeline[i])
        dura_mean[i] = np.nanmean(duras_i)
        dura_var[i] = np.nanstd(duras_i)

    return dura_mean, dura_var

fig, ax = plt.subplots()
#ax.grid()

book = xlrd.open_workbook("./f-rrtStar-wo.xlt")
eval_timeline = np.array([200, 800, 1800, 3000, 4200, 5700, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='P', s=100, label='RRT*')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dotted', marker='x', label='kRRT*-w/o', c='green', ms=8)
print("kRRT*-w/o cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT*-w/o dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

book = xlrd.open_workbook("./f-rrtStar-s.xlt")
eval_timeline = np.array([200, 700, 1700, 2900, 4000, 5500, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='X', s=100, label='RRT*-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dashdot', marker='x', label='kRRT*-S', c='green', ms=8)
print("kRRT*-S cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT*-S dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

book = xlrd.open_workbook("./f-rrtStar-w.xlt")
eval_timeline = np.array([200, 750, 1800, 2850, 4050, 5550, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='X', s=100, label='RRT*-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, marker='x', label='kRRT*-ST', c='green', ms=8)
print("kRRT*-ST cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT*-ST dura: ", dura_mean_var[0][6], dura_mean_var[1][6])


book = xlrd.open_workbook("./f-rrtSharp-wo.xlt")
eval_timeline = np.array([200, 1000, 1900, 3200, 4400, 5900, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='x', s=80,  label='RRT#')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dotted', marker='P', label='kRRT#-w/o', c='red', ms=8)
print("kRRT#-w/o cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT#-w/o dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

book = xlrd.open_workbook("./f-rrtSharp-s.xlt")
eval_timeline = np.array([200, 950, 1800, 3100, 4200, 5700, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='d', s=80,  label='RRT#-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dashdot', marker='P', label='kRRT#-S', c='red', ms=8)
print("kRRT#-S cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT#-S dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

book = xlrd.open_workbook("./f-rrtSharp-w.xlt")
eval_timeline = np.array([200, 900, 1950, 3050, 4250, 5750, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='d', s=80,  label='RRT#-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, marker='P', label='kRRT#-ST\n(Proposed)', c='red', ms=8)
print("kRRT#-ST cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT#-ST dura: ", dura_mean_var[0][6], dura_mean_var[1][6])


book = xlrd.open_workbook("./f-rrt-wo.xlt")
eval_timeline = np.array([200, 800, 1700, 2800, 4000, 5500, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='v', s=100, label='RRT')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dotted', marker='v', label='kRRT-w/o', c='blue', ms=7)
print("kRRT-w/o cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT-w/o dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

book = xlrd.open_workbook("./f-rrt-s.xlt")
eval_timeline = np.array([200, 850, 1750, 2900, 4100, 5600, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='o', s=100, label='RRT-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dashdot', marker='v', label='kRRT-S', c='blue', ms=7)
print("kRRT-S cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT-S dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

book = xlrd.open_workbook("./f-rrt-w.xlt")
eval_timeline = np.array([200, 750, 1950, 2750, 3950, 5450, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
dura_mean_var = get_dura_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='o', s=100, label='RRT-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, marker='v', label='kRRT-ST', c='blue', ms=7)
print("kRRT-ST cost: ", cost_mean_var[0][6], cost_mean_var[1][6])
print("kRRT-ST dura: ", dura_mean_var[0][6], dura_mean_var[1][6])

ax.legend(ncol=3)
ax.set_xlabel("Time (ms)", fontsize='large')
ax.set_ylabel("Best Solution Cost", fontsize='large')

# scipy.io.savemat("./cost_mean_var1.mat", {'timeline': eval_timeline, 'cost_mean': cost_mean, 'cost_var': cost_var})

plt.show()
