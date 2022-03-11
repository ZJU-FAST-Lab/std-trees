import xlrd
import scipy.io
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# matplotlib.use('qt5agg')


def get_costs_at_t(time_split, cost_split, t):
    costs_t = np.zeros(len(time_split))
    for j in range(0, len(time_split)):
        time_j = time_split[j]
        cost_j = cost_split[j]

        N = time_j.size
        if t < time_j[0]:
            costs_t[j] = np.nan
        elif t >= time_j[N - 1]:
            costs_t[j] = cost_j[N - 1]
        else:
            for k in range(1, N):
                if t < time_j[k]:
                    costs_t[j] = cost_j[k - 1]
                    break
                
    return costs_t


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
        costs_i = get_costs_at_t(time_split, cost_split, eval_timeline[i])
        cost_mean[i] = np.nanmean(costs_i)
        cost_var[i] = np.nanstd(costs_i)

    return cost_mean, cost_var

fig, ax = plt.subplots()
#ax.grid()

book = xlrd.open_workbook("./f-rrt-w.xls")
eval_timeline = np.array([500, 1000, 1500, 2000, 2500, 3000, 4000, 5500, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='o', s=100, label='RRT-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, marker='v', label='kRRT-w/', c='blue', ms=8)

book = xlrd.open_workbook("./f-rrt-wo.xls")
eval_timeline = np.array([500, 1000, 1500, 2000, 2500, 3000, 4000, 5500, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='v', s=100, label='RRT')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dashed', marker='v', label='kRRT-w/o', c='blue', ms=8)

book = xlrd.open_workbook("./f-rrtStar-w.xls")
eval_timeline = np.array([600, 1100, 1600, 2100, 2600, 3200, 4200, 5700, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='X', s=100, label='RRT*-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, marker='P', label='kRRT*-w/', c='green', ms=8)

book = xlrd.open_workbook("./f-rrtStar-wo.xls")
eval_timeline = np.array([600, 1100, 1600, 2100, 2600, 3200, 4200, 5700, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='P', s=100, label='RRT*')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dashed', marker='P', label='kRRT*-w/o', c='green', ms=8)

book = xlrd.open_workbook("./f-rrtSharp-w.xls")
eval_timeline = np.array([700, 1200, 1700, 2200, 2700, 3400, 4400, 5900, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='d', s=80,  label='RRT#-BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, marker='d', label='kRRT#-w/', c='red', ms=8)

book = xlrd.open_workbook("./f-rrtSharp-wo.xls")
eval_timeline = np.array([700, 1200, 1700, 2200, 2700, 3400, 4400, 5900, 7000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
#ax.scatter(eval_timeline, cost_mean_var[0], marker='x', s=80,  label='RRT#')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=1.5, ls='dashed', marker='d', label='kRRT#-w/o', c='red', ms=8)

ax.legend(ncol=1)
ax.set_xlabel("Time (ms)", fontsize='large')
ax.set_ylabel("Best Solution Cost", fontsize='large')

# scipy.io.savemat("./cost_mean_var1.mat", {'timeline': eval_timeline, 'cost_mean': cost_mean, 'cost_var': cost_var})

plt.show()
