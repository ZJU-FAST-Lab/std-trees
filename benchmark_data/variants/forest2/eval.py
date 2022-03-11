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
ax.grid()

book = xlrd.open_workbook("./f-one-node.xls")
eval_timeline = np.array([0, 60, 150, 300, 600, 1000, 2000, 3000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
ax.scatter(eval_timeline, cost_mean_var[0], marker='o', s=60, label='NODE')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=2)

book = xlrd.open_workbook("./f-trunk.xls")
eval_timeline = np.array([0, 100, 200, 400, 700, 1100, 2100, 3000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
ax.scatter(eval_timeline, cost_mean_var[0], marker='v', s=60, label='TRUNK')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=2)

# book = xlrd.open_workbook("./f-branch.xls")
# eval_timeline = np.array([0, 26,56, 96, 190, 280, 580, 980, 1980, 2980])
# cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
# ax.scatter(eval_timeline, cost_mean_var[0], marker='v', s=100, label='BRANCH')
# ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=2)

book = xlrd.open_workbook("./f-tree.xls")
eval_timeline = np.array([0, 80, 175, 350, 650, 1050, 2050, 3000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
ax.scatter(eval_timeline, cost_mean_var[0], marker='X', s=60, label='TREE')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=2)

book = xlrd.open_workbook("./f-small-branch.xls")
eval_timeline = np.array([0, 120, 225, 345, 750, 1150, 2150, 3000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
ax.scatter(eval_timeline, cost_mean_var[0], marker='P', s=60, label='BRANCH')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=2)

book = xlrd.open_workbook("./f-no-deform.xls")
eval_timeline = np.array([0, 60, 150, 450, 600, 1000, 2000, 3000])
cost_mean_var = get_cost_mean_var_list(book, eval_timeline)
ax.scatter(eval_timeline, cost_mean_var[0], marker='p', s=60,  label='w/o')
ax.errorbar(eval_timeline, cost_mean_var[0], yerr=cost_mean_var[1], capsize=2, lw=2)

book = xlrd.open_workbook("./f-astar.xls")
sheet = book.sheet_by_index(0)
lambda_heu_all = np.array(sheet.col_values(0))
time_all = np.array(sheet.col_values(1))
cost_all = np.array(sheet.col_values(2))
ax.scatter(time_all, cost_all, marker='x', s=50, c='b', label='Search')

ax.legend(ncol=2)
ax.set_xlabel("time (ms)", fontsize='large')
ax.set_ylabel("cost", fontsize='large')

# scipy.io.savemat("./cost_mean_var1.mat", {'timeline': eval_timeline, 'cost_mean': cost_mean, 'cost_var': cost_var})

plt.show()
