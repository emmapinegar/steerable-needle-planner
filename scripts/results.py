import numpy as np
import open3d as o3d
import os, fnmatch
import matplotlib.pyplot as plotter
from dataclasses import dataclass
import copy
import scipy
from scipy import optimize
from scipy import stats
from magnet import Magnet
from viz_utils import stats_indices, planners, viz_params, get_planner_indices, make_violin_figure


def make_time_average_figure(data, time_data, index, title, ylabel, y_min=0, y_max=2, ylog=False):
    """    
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_min (float): minimum y axis value, default=0
        y_max (float): maximum y axis value, default=100
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """

    colors = []
    labels = []
    plotter.title(title)
    lines = []

    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
        if index == stats_indices['lengths']:
            data_ind = time_data[planner_indices,stats_indices['lengths']-stats_indices['times']]/data[planner_indices,stats_indices['sg_mag']]
        else:
            data_ind = time_data[planner_indices, index - stats_indices['times']]
        if np.shape(data_ind)[0] == 0:
            continue
        colors += [planners[i].color]
        labels += [planners[i].label]
        flat = []
        for x in data_ind:
            for xi in x:
                flat.append(xi)

        time = []
        for x in time_data[planner_indices, stats_indices['times'] - stats_indices['times']]:
            for xi in x:
                time.append(xi)
        time = np.array(time)
        flat = np.array(flat)
        sortedinds = np.argsort(time)

        time = time[sortedinds]
        flat = flat[sortedinds]


        # for j in range(np.shape(planner_indices)[0]):
        #     if int(data[planner_indices[j], stats_indices['success']]) == 1:
        #         plotter.scatter(time_data[planner_indices[j], stats_indices['times'] - stats_indices['times']], data_ind[j], color=planners[i].color, alpha=viz_params['alpha']/10)

        n = 3 #window
        average = np.cumsum(flat)
        average[n:] = average[n:] - average[:-n]
        average[n-1:] = average[n-1:]/n

        averagetime = np.cumsum(time)
        averagetime[n:] = averagetime[n:] - averagetime[:-n]
        averagetime[n-1:] = averagetime[n-1:]/n

        if np.shape(average)[0] > 0:
            for j in range(0, n-1):
                average[j] = average[j]/(j+1)
                averagetime[j] = averagetime[j]/(j+1)

        print(average)
        print(flat)
        print(np.shape(average))
        print(np.shape(flat))

        # A = [time*time, time, np.ones(np.shape(time))]
        # A = np.transpose(A)
        # y = flat
        # p = np.linalg.lstsq(A, y)
        # coeff = p[0]
        # plotter.plot(time, coeff[0]*time*time + coeff[1]*time + coeff[2]*np.ones(np.shape(time)), color=planners[i].color)

        # A = [averagetime*averagetime, averagetime, np.ones(np.shape(averagetime))]
        # A = np.transpose(A)
        # y = average
        # p = np.linalg.lstsq(A, y)
        # coeff = p[0]
        # plotter.plot(averagetime, coeff[0]*averagetime*averagetime + coeff[1]*averagetime + coeff[2]*np.ones(np.shape(averagetime)), color=planners[i].color)

        line = plotter.plot(averagetime, average, color=planners[i].color, linestyle=planners[i].linestyle, label=planners[i].label, linewidth=1.75)
        # lines += [line]

    # if len(colors) > 0:
    #     plotter.setp(lines[0], color=colors[0])
    # if len(colors) > 1:      
    #     plotter.setp(lines[1], color=colors[1])
    # if len(colors) > 2:
    #     plotter.setp(lines[2], color=colors[2])
    # if len(colors) > 3:
    #     plotter.setp(lines[3], color=colors[3])


    plotter.ylabel(ylabel)
    plotter.legend()
    x_max = np.floor(np.max(data[:,stats_indices['time']]))
    plotter.xlim([0.0001, x_max])
    plotter.ylim([y_min,y_max])
    plotter.xlabel('seconds')
    if ylog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')


def make_time_figure(data, time_data, index, title, ylabel, y_min=0, y_max=2, ylog=False):
    """    
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_min (float): minimum y axis value, default=0
        y_max (float): maximum y axis value, default=100
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """

    colors = []
    labels = []
    plotter.title(title)
    lines = []

    for i in range(len(planners)):
        # print(data)
        planner_indices = get_planner_indices(data, planners[i])
        data_ind = time_data[planner_indices, index - stats_indices['times']]
        # if index == stats_indices['lengths']:
        #     data_ind = time_data[planner_indices,stats_indices['lengths']-stats_indices['times']]/data[planner_indices,stats_indices['sg_mag']]
        # else:
        #     data_ind = time_data[planner_indices, index - stats_indices['times']]
        if np.shape(data_ind)[0] == 0:
            continue
        colors += [planners[i].color]
        labels += [planners[i].label]
        flat = []
        for x in data_ind:
            for xi in x:
                flat.append(xi)

        time = []
        for x in time_data[planner_indices, stats_indices['times'] - stats_indices['times']]:
            for xi in x:
                time.append(xi)
        time = np.array(time)
        flat = np.array(flat)
        sortedinds = np.argsort(time)

        time = time[sortedinds]
        flat = flat[sortedinds]


        # for j in range(np.shape(planner_indices)[0]):
        #     if int(data[planner_indices[j], stats_indices['success']]) == 1:
        #         plotter.scatter(time_data[planner_indices[j], stats_indices['times'] - stats_indices['times']], data_ind[j], color=planners[i].color, alpha=viz_params['alpha']/10)


        n = 50 #window

        if np.shape(flat)[0] < n*5:
            n = 8

        median = np.zeros(np.shape(flat)[0]-n)
        median_time = np.zeros(np.shape(time)[0]-n)

        for j in range(np.shape(median)[0]):
            median[j] = np.median(flat[j:j+n])
            median_time[j] = np.median(time[j:j+n])


        average = np.cumsum(flat)
        average[n:] = average[n:] - average[:-n]
        average[n-1:] = average[n-1:]/n

        averagetime = np.cumsum(time)
        averagetime[n:] = averagetime[n:] - averagetime[:-n]
        averagetime[n-1:] = averagetime[n-1:]/n

        average = average[n:]
        averagetime = averagetime[n:]
        # if np.shape(average)[0] > 0:
        #     for j in range(0, n-1):
        #         average[j] = average[j]/(j+1)
        #         averagetime[j] = averagetime[j]/(j+1)

        # line = plotter.plot(median_time, median, color=planners[i].color, linestyle=planners[i].linestyle, label=planners[i].label, linewidth=1.75)
        plotter.plot(averagetime, average, color=planners[i].color, linestyle=planners[i].linestyle, label=planners[i].label, alpha=viz_params['alpha'], linewidth=1.75)
        # lines += [line]

    plotter.ylabel(ylabel)
    plotter.legend()
    x_max = np.floor(np.max(data[:,stats_indices['time']]))
    plotter.xlim([0.0001, x_max])
    plotter.ylim([y_min,y_max])
    plotter.xlabel('seconds')
    if ylog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')


def make_success_time_figure(data, time_data, index, title, ylabel, y_min=0, y_max=2, xlog=True):
    """    
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_min (float): minimum y axis value, default=0
        y_max (float): maximum y axis value, default=100
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """
    plotter.title(title)

    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
        if index == stats_indices['lengths']:
            data_ind = time_data[planner_indices,stats_indices['lengths']-stats_indices['times']]/data[planner_indices,stats_indices['sg_mag']]
        else:
            data_ind = time_data[planner_indices, index - stats_indices['times']]
        if np.shape(data_ind)[0] == 0:
            continue


        time = []
        success = []
        success_upper = []
        success_lower = []
        j = 0
        for x in time_data[planner_indices, stats_indices['times'] - stats_indices['times']]:
            if len(x) > 0:
                time.append(x[0])
                j += 1
                next_success = j/np.shape(planner_indices)[0]
                success.append(100*next_success)
                adjusted_proportion = (j + 2)/(np.shape(planner_indices)[0] + 4)
                se_1 = adjusted_proportion*(1-adjusted_proportion)
                se_2 = se_1/(np.shape(planner_indices)[0] + 4)
                standard_error = np.sqrt(se_2)
                margin_of_error = standard_error*2
                success_upper.append(100*next_success + 100*margin_of_error)
                success_lower.append(100*next_success - 100*margin_of_error)

        if len(time) > 0:
            time = np.array(time)
            sortedinds = np.argsort(time)
            time = time[sortedinds]
            plotter.plot(time, success, color=planners[i].color, label=planners[i].label, linestyle=planners[i].linestyle, linewidth=1.75)
            plotter.fill_between(time, success_upper, success_lower, color=planners[i].color, alpha=viz_params['alpha']/1.5)


    plotter.ylabel(ylabel)
    plotter.legend()
    x_max = np.floor(np.max(data[:,stats_indices['time']]))
    plotter.xlim([0.0001, x_max])
    plotter.ylim([0,100])
    plotter.xlabel('seconds')

    if xlog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')
        

def make_success_data_figure(data, time_data, index, title, ylabel, x_min=0, x_max=2, xlog=False):
    """    
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_min (float): minimum y axis value, default=0
        y_max (float): maximum y axis value, default=100
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """

    colors = []
    labels = []
    plotter.title(title)
    lines = []

    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
        if index == stats_indices['lengths']:
            data_ind = time_data[planner_indices,stats_indices['lengths']-stats_indices['times']]/data[planner_indices,stats_indices['sg_mag']]
        else:
            data_ind = time_data[planner_indices, index - stats_indices['times']]
        if np.shape(data_ind)[0] == 0:
            continue


        plot_data = []
        success = []

        j = 0
        for x in data_ind:
            if len(x) > 0:
                plot_data.append(x[-1])
                j += 1
                next_success = j/np.shape(planner_indices)[0]
                success.append(100*next_success)


        if len(plot_data) > 0:

            # colors += [planners[i].color]
            # labels += [planners[i].label]

            plot_data = np.array(plot_data)
            sortedinds = np.argsort(plot_data)

            plot_data = plot_data[sortedinds]
            line = plotter.plot(plot_data, success, color=planners[i].color, linestyle=planners[i].linestyle, label=planners[i].label, linewidth=1.75)


    plotter.ylabel('Success Percentage')
    plotter.legend()

    plotter.xlim([x_min, x_max])
    plotter.ylim([0,100])
    plotter.xlabel(ylabel)

    if xlog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')


def make_success_heat_figure(data, time_data, index, title, ylabel, y_min=0, y_max=2, xlog=True):
    """    
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_min (float): minimum y axis value, default=0
        y_max (float): maximum y axis value, default=100
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """
    plotter.title(title)


    pairs = np.unique(data[:,stats_indices['sg_index']])
    numpairs = 500#np.shape(pairs)[0]

    numplanners = np.shape(np.unique(data[:,stats_indices['planner']]))[0]
    pairs_success = np.zeros(numpairs)

    for j in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[j])

        for i in range(np.shape(planner_indices)[0]):
            if data[planner_indices[i], stats_indices['success']] == 1:
                pairs_success[i] += 1

    pairs_success = np.divide(pairs_success, numplanners)
    pairs_success = np.reshape(pairs_success, (25,-1))

    plotter.imshow(pairs_success)


def make_scaled_length_time_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0] + 8
    rows = 2
    cols = max(num_plots//rows, 1)
    fig_ind = 1

    pairs = np.unique(data[:,stats_indices['sg_index']])
    pairs_mins = np.zeros(int(np.max(pairs))+1)
    # print(np.shape(pairs_mins))
    for pair in pairs:
        # print(pair)
        pair = int(pair)
        pair_inds = np.where(data[:,stats_indices['sg_index']] == pair)[0]
        pair_data = data[pair_inds,stats_indices['ell']]
        pair_min = np.min(pair_data)
        pairs_mins[pair] = pair_min
        pair_time_data = time_data[pair_inds,stats_indices['lengths']-stats_indices['times']]
        # print(pair_time_data)
        pair_time_data = np.divide(pair_time_data,pair_min)
        # print(pair_time_data)
        time_data[pair_inds,stats_indices['lengths']-stats_indices['times']] = pair_time_data

    # print(pairs_mins)

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for k in range(np.shape(phis)[0]):
            phi = phis[k]
            phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
            phi_data = env_data[phi_inds,:]
            if np.shape(phi_data)[0] > 0:
                for i in range(np.shape(kappas)[0]):
                    kappa = kappas[i]
                    kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                    kappa_data = phi_data[kappa_inds,:]
                    for l in range(np.shape(varcurvs)[0]):
                        varcurv = varcurvs[l]
                        varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                        varcurv_data = kappa_data[varcurv_inds,:]
                        if np.shape(varcurv_data)[0] > 0:
                            # for m in range(20,20+num_plots//2): 
                            #     pair_inds = np.where(varcurv_data[:,stats_indices['sg_index']] == m)[0]
                            #     pair_data = varcurv_data[pair_inds,:]
                            #     if np.shape(pair_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            make_time_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' var = $ %d$' %varcurv, axis_label, y_min=y_min, y_max=y_max)
                            fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.05, hspace=0.3, wspace=0.2)


def make_success_time_figures(data, time_data):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0] + 8
    rows = 2
    cols = max(num_plots//rows, 1)
    fig_ind = 1
    # make figures for each of the kappa values used in experiments

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for k in range(np.shape(phis)[0]):
            phi = phis[k]
            phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
            phi_data = env_data[phi_inds,:]
            if np.shape(phi_data)[0] > 0:
                for i in range(np.shape(kappas)[0]):
                    kappa = kappas[i]
                    kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                    kappa_data = phi_data[kappa_inds,:]
                    for l in range(np.shape(varcurvs)[0]):
                        varcurv = varcurvs[l]
                        varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                        varcurv_data = kappa_data[varcurv_inds,:]
                        if np.shape(varcurv_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            make_success_time_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], stats_indices['lengths'], r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' var = $ %d$' %varcurv, r'Success Percentage', y_min=0, y_max=100)
                            fig_ind += 1
    plotter.suptitle(r'Success vs Time', fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.05, hspace=0.3, wspace=0.2)


def make_time_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0] + 8
    rows = 2
    cols = max(num_plots//rows, 1)
    fig_ind = 1

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for k in range(np.shape(phis)[0]):
            phi = phis[k]
            phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
            phi_data = env_data[phi_inds,:]
            if np.shape(phi_data)[0] > 0:
                for i in range(np.shape(kappas)[0]):
                    kappa = kappas[i]
                    kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                    kappa_data = phi_data[kappa_inds,:]
                    for l in range(np.shape(varcurvs)[0]):
                        varcurv = varcurvs[l]
                        varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                        varcurv_data = kappa_data[varcurv_inds,:]
                        if np.shape(varcurv_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            make_time_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' var = $ %d$' %varcurv, axis_label, y_min=y_min, y_max=y_max)
                            fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.05, hspace=0.3, wspace=0.2)


def make_success_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0] + 8
    rows = 2
    cols = max(num_plots//rows, 1)
    fig_ind = 1

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for k in range(np.shape(phis)[0]):
            phi = phis[k]
            phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
            phi_data = env_data[phi_inds,:]
            if np.shape(phi_data)[0] > 0:
                for i in range(np.shape(kappas)[0]):
                    kappa = kappas[i]
                    kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                    kappa_data = phi_data[kappa_inds,:]
                    for l in range(np.shape(varcurvs)[0]):
                        varcurv = varcurvs[l]
                        varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                        varcurv_data = kappa_data[varcurv_inds,:]
                        if np.shape(varcurv_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            make_success_data_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' var = $ %d$' %varcurv, axis_label, x_min =y_min, x_max = y_max)
                            fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.05, hspace=0.3, wspace=0.2)


def make_violin_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0] + 8
    rows = 2
    cols = max(num_plots//rows, 1)
    fig_ind = 1

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for k in range(np.shape(phis)[0]):
            phi = phis[k]
            phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
            phi_data = env_data[phi_inds,:]
            if np.shape(phi_data)[0] > 0:
                for i in range(np.shape(kappas)[0]):
                    kappa = kappas[i]
                    kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                    kappa_data = phi_data[kappa_inds,:]
                    for l in range(np.shape(varcurvs)[0]):
                        varcurv = varcurvs[l]
                        varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                        varcurv_data = kappa_data[varcurv_inds,:]
                        if np.shape(varcurv_data)[0] > 0:
                            plan_data_ind = np.where(varcurv_data[:,stats_indices['success']] == 1)[0]
                            plan_data = varcurv_data[plan_data_ind,:]
                            if np.shape(plan_data)[0] > 0:
                                plotter.subplot(rows, cols, fig_ind)
                                make_violin_figure(plan_data, index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' var = $ %d$' %varcurv, axis_label, y_min=y_min, y_max=y_max, ylog=False)
                                fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.05, hspace=0.3, wspace=0.2)  


def make_success_heat_figures(data, time_data):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0] + 8
    rows = 2
    cols = max(num_plots//rows, 1)
    fig_ind = 1
    # make figures for each of the kappa values used in experiments

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for k in range(np.shape(phis)[0]):
            phi = phis[k]
            phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
            phi_data = env_data[phi_inds,:]
            if np.shape(phi_data)[0] > 0:
                for i in range(np.shape(kappas)[0]):
                    kappa = kappas[i]
                    kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                    kappa_data = phi_data[kappa_inds,:]
                    for l in range(np.shape(varcurvs)[0]):
                        varcurv = varcurvs[l]
                        varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                        varcurv_data = kappa_data[varcurv_inds,:]
                        if np.shape(varcurv_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            make_success_heat_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], stats_indices['lengths'], r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' var = $ %d$' %varcurv, r'Success Percentage', y_min=0, y_max=100)
                            fig_ind += 1
    plotter.suptitle(r'Success vs Pairs', fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.025, hspace=0.3, wspace=0.2)


def make_success_brain_figures(data):
    fig = plotter.figure(figsize=[15, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    # make figures for each of the kappa values used in experiments

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        phi_data = env_data
        # for k in range(np.shape(phis)[0]):
        #     phi = phis[k]
        #     phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
        #     phi_data = env_data[phi_inds,:]
        #     if np.shape(phi_data)[0] > 0:
        for i in range(np.shape(kappas)[0]):
            kappa = kappas[i]
            kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
            kappa_data = phi_data[kappa_inds,:]
            for l in range(np.shape(varcurvs)[0]):
                varcurv = varcurvs[l]
                varcurv_inds = np.where(kappa_data[:,stats_indices['varcurv']] == varcurv)[0]
                varcurv_data = kappa_data[varcurv_inds,:]
                if np.shape(varcurv_data)[0] > 0:
                    make_success_brain_figure(varcurv_data)


def make_success_brain_figure(data):
    """    
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_min (float): minimum y axis value, default=0
        y_max (float): maximum y axis value, default=100
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """
    min_pair = int(np.min(data[:,stats_indices['sg_index']]))
    max_pair = int(np.max(data[:,stats_indices['sg_index']]))
    pairs_file = "./../data/input/remind_001_sg_pairs.txt"
    pairs = np.loadtxt(pairs_file)
    pairs = pairs[min_pair:max_pair+1, :]
    numpairs = np.shape(pairs)[0]
    print(f"min pair: {min_pair} max pair: {max_pair}")
    pairs_success = np.zeros(numpairs)


    for i in range(np.shape(data)[0]):

        if int(data[i, stats_indices['success']]) == 1:
            # print(f"index: {int(data[i,stats_indices['sg_index']])} num: {pairs_success[int(data[i,stats_indices['sg_index']])] + 1}")
            pairs_success[int(data[i,stats_indices['sg_index']]) - min_pair] = pairs_success[int(data[i,stats_indices['sg_index']]) - min_pair] + 1

    colors = [[1,0,0], [0.85, 0.85, 0.85], [0.76, 0.74, 0.88], [0.63, 0.61, 0.77], [0.56, 0.52, 0.74], [0.53, 0.48, 0.69], [0.42, 0.33, 0.70], [0.26, 0.16, 0.63], [0.16, 0.01, 0.67]]

    start = o3d.geometry.TriangleMesh.create_coordinate_frame()
    ptcs = [start]

    for i in range(numpairs):
        point = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        point.translate(pairs[i,0:3])
        point.paint_uniform_color(colors[int(pairs_success[i])])
        ptcs.append(point)

    for i in range(numpairs):
        point = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        point.translate(pairs[i,3:6])
        point.paint_uniform_color(colors[int(pairs_success[i])])
        ptcs.append(point)


    o3d.visualization.draw_geometries(ptcs)   


def plot_magnet_options(datafile):
        mat = scipy.io.loadmat(datafile)
        readtorques = mat["torques"]
        radius = mat["kappa"]
        
        radius_by_stiffness = np.zeros((3, radius.shape[1], radius.shape[2]))
        
        radius_by_stiffness[0, :, 0:3] = radius[4, :, 1:]  # this is the brain stiffness data
        radius_by_stiffness[1, :, 0:3] = radius[1, :, 1:]
        radius_by_stiffness[2, :, 0:3] = radius[5, :, 1:]


        all_torques = np.ones(np.shape(radius_by_stiffness[0,:,:]))
        all_torques[:,0] = np.multiply(readtorques[0,1],all_torques[:,0])
        all_torques[:,1] = np.multiply(readtorques[0,2],all_torques[:,1])
        all_torques[:,2] = np.multiply(readtorques[0,3],all_torques[:,2])
        all_torques[:,3] = np.multiply(0,all_torques[:,3])

        # fit_radius = np.reshape(radius_by_stiffness[0,:,:],(-1,))
        # fit_torque = np.reshape(all_torques,(-1))
        # print(fit_radius)
        # print(fit_torque)
        radius_by_stiffness_mean = np.squeeze(np.mean(radius_by_stiffness, 1))
        torques = np.array((readtorques[0,3], readtorques[0,2], readtorques[0,1], 0))
        radius_of_curvatures = np.array((radius_by_stiffness_mean[0,2], radius_by_stiffness_mean[0,1], radius_by_stiffness_mean[0,0], 0)) # only use the brain stiffness data

        # bestFit = np.array(stats.linregress(fit_torque, fit_radius))
        bestFit = np.array(stats.linregress(torques, radius_of_curvatures))
        m = bestFit[0]
        b = bestFit[1]
        torques_ = torques[:3]
        print(torques)

        distances = [43, 46, 50]
        # print(radius_by_stiffness[0,0,:])
        # print(radius_by_stiffness_mean)
        # print(radius)
        # print(mat)
        print(f"m: {m} b: {b}")
        plotter.figure()

        # plotter.scatter(distances, 1000/np.array([radius_by_stiffness[0,0,2],radius_by_stiffness[0,0,1],radius_by_stiffness[0,0,0]]), c='k', marker='o', s=10, alpha=1)
        # plotter.scatter(distances, 1000/np.array([radius_by_stiffness[0,1,2],radius_by_stiffness[0,1,1],radius_by_stiffness[0,1,0]]), c='k', marker='o', s=10, alpha=1)
        # plotter.scatter(distances, 1000/np.array([radius_by_stiffness[0,2,2],radius_by_stiffness[0,2,1],radius_by_stiffness[0,2,0]]), c='k', marker='o', s=10, alpha=1)
        # plotter.scatter(distances, 1000/np.array([radius_by_stiffness[0,3,2],radius_by_stiffness[0,3,1],radius_by_stiffness[0,3,0]]), c='k', marker='o', s=10, alpha=1)
        # plotter.scatter(distances, 1000/np.array([radius_by_stiffness[0,4,2],radius_by_stiffness[0,4,1],radius_by_stiffness[0,4,0]]), c='k', marker='o', s=10, alpha=1)
        # plotter.scatter(distances, 1000/radius_of_curvatures[:3], s=10, alpha=1)
        distances = np.linspace(20, 150, 105, endpoint=True)
        # distances = np.divide(distances, 1000)
        needle = Magnet(np.array([[0], [0], [0]]), np.array([[0, 0, 1]]), 0.0018, radius=0.001)
     
        
        colors = [ '#117733','#C33AAC','#BF0F67', '#332288'] # '#0063F8' '#40B3EC' '#44AA99' '#D46D7E' '#EF6E12'

        
        samm = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 66.03, radius=0.0254, shape='sphere')
        # sammn42 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 72.10, radius=0.0254, Br=1.30, shape='sphere')
        sammn52 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 80.84, radius=0.0254, Br=1.45, shape='sphere')
        # smallsamm = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 30.42, radius=0.01905, Br=1.30, shape='sphere')
        # smallsammn52 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 34.11, radius=0.01905, Br=1.45, shape='sphere')

        # smallcuben42 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 58.10, radius=0.02694, Br=1.30, shape='cube')
        smallcuben52 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 65.14, radius=0.02694, Br=1.45, shape='cube')
        bigcuben42 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 137.71, radius=0.03592, Br=1.30, shape='cube')
        # uniformn52 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 830, radius=0.055, Br=1.48, shape='sphere')
        # bigcuben52 = Magnet(np.array([[distances[0]], [0], [0]]), np.array([[0], [1], [0]]), 154.40, radius=0.03592, Br=1.45, shape='cube')  

        magnets = [sammn52, smallcuben52, bigcuben42, samm] 

        for i in range(len(magnets)):
            dist_points = []
            radius_points = []
            for distance in distances:
                      
                if distance > magnets[i].radius*1000 + 1:
                    magnets[i].position = np.array([[distance/1000], [0], [0]])                              #magnets[i].position + np.array([[magnets[i].radius], [0], [0]])
                    f, tau = needle.get_force_torque(magnets[i])

                    curvature = 1000/(np.linalg.norm(tau)*m + b)
                    print(f"position: {distance} {magnets[i].position.reshape((-1,))} curvature: {curvature} tau: {np.linalg.norm(tau)} m: {magnets[i].mag} {magnets[i].m.reshape((-1,))}")
                    if curvature > 12:

                        marker = 'o'
                        if magnets[i].shape == 'cube':
                            marker= 's'

                        alpha = 0.75
                        if magnets[i].Br > 0.0:
                            alpha = magnets[i].Br - 0.75

                        dist_points += [distance-magnets[i].radius*1000]
                        radius_points += [curvature]
                        # plotter.scatter(dist_points, radius_points, color=colors[i], alpha=alpha, marker=marker)


            plotter.plot(dist_points, radius_points, color=colors[i], alpha=alpha)
        
        radii = np.array([15, 25, 50, 100])
        for i in range(len(magnets)):
            dist_points = []
            radius_points = []
            for radius in radii:
                torque_mag = ((1000/radius)-b)/m
                f, tau = needle.get_force_torque(magnets[i])
                r_mag = 1000*np.cbrt((np.linalg.norm(tau)*np.linalg.norm(magnets[i].position)**3)/torque_mag)      
                if r_mag > magnets[i].radius*1000 + 10:
                    magnets[i].position = np.array([[r_mag/1000], [0], [0]])                              #magnets[i].position + np.array([[magnets[i].radius], [0], [0]])
                    f, tau = needle.get_force_torque(magnets[i])

                    curvature = 1000/(np.linalg.norm(tau)*m + b)
                    print(f"position: {r_mag} {magnets[i].position.reshape((-1,))} curvature: {curvature} tau: {np.linalg.norm(tau)} m: {magnets[i].mag} {magnets[i].m.reshape((-1,))}")
                    if curvature > 12:

                        marker = 'o'
                        if magnets[i].shape == 'cube':
                            marker= 's'

                        alpha = 0.75
                        if magnets[i].Br > 0.0:
                            alpha = magnets[i].Br - 0.75

                        dist_points += [r_mag]
                        radius_points += [curvature]
                        plotter.scatter(r_mag-magnets[i].radius*1000, curvature, color=colors[i], alpha=alpha, marker=marker)


        # plotter.scatter(torques_, 1000/np.array([radius_by_stiffness[0,0,2],radius_by_stiffness[0,0,1],radius_by_stiffness[0,0,0]]), c='k', marker='o', s=120, alpha=0.1)
        # plotter.scatter(torques_, 1000/np.array([radius_by_stiffness[0,1,2],radius_by_stiffness[0,1,1],radius_by_stiffness[0,1,0]]), c='k', marker='o', s=120, alpha=0.1)
        # plotter.scatter(torques_, 1000/np.array([radius_by_stiffness[0,2,2],radius_by_stiffness[0,2,1],radius_by_stiffness[0,2,0]]), c='k', marker='o', s=120, alpha=0.1)
        # plotter.scatter(torques_, 1000/np.array([radius_by_stiffness[0,3,2],radius_by_stiffness[0,3,1],radius_by_stiffness[0,3,0]]), c='k', marker='o', s=120, alpha=0.1)
        # plotter.scatter(torques_, 1000/np.array([radius_by_stiffness[0,4,2],radius_by_stiffness[0,4,1],radius_by_stiffness[0,4,0]]), c='k', marker='o', s=120, alpha=0.1)

        # plotter.scatter(torques[:3], 1000/radius_of_curvatures[:3], s=30, alpha=1)
        # plotter.scatter(torques_, 1000/(torques_*m + b), s=30, alpha=1)
        # extents = np.array([3e-5, 3.5e-5, 4e-5, 5e-5, 6e-5, 7e-5, 8e-5, torques_[2], torques_[1], torques_[0]])
        # plotter.plot(extents, 1000/(extents*m + b), c='k', linestyle='--')

        # print(1000/(extents*m + b))
        # print(bestFit)

        # # https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.ticklabel_format.html
        # plotter.ticklabel_format(axis='x', style='sci', scilimits=(-3,3))
        plotter.xlabel("Distance (mm)")
        plotter.ylabel("Radius of Curvature (mm)")
        plotter.show()    



if __name__=='__main__':

    files = fnmatch.filter(os.listdir('./../data/output/'), '*_stats.txt')
    data = np.empty((0,14))
    time_data = []
    def conv(x):
        x_ = x.decode()
        if len(x) > 2:
            values = np.array([float(xi) for xi in x_.strip("[,]").split(',')])
            return values
        else:
            return np.array([])
        
    convs = {0: lambda x: conv(x), 1: lambda x: conv(x), 2: lambda x: conv(x), 3: lambda x: conv(x)}
    for file in files:
        next_data = np.loadtxt('./../data/output/' + file, delimiter=',', comments='#', usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12,13))

        data = np.vstack((data, next_data))
        next_time_data = np.loadtxt('./../data/output/' + file, delimiter=',', comments='#', usecols=(14,15,16,17), converters=conv, dtype=object, quotechar='"')

        time_data.append(next_time_data)

    # plot_magnet_options("./../../data/PiGroup/curvature_pi_group_data.mat")


    # make_success_brain_figure(data)
    # make_success_brain_figures(data)

    make_success_heat_figures(data, time_data[0])


    make_success_time_figures(data, time_data[0])
    

    # make_time_figures(data, time_data[0], stats_indices['lengths'], 1, 1.05, r'Distance vs Time', r'$\ell^\prime$')
    make_scaled_length_time_figures(data, time_data[0], stats_indices['lengths'], 1, 1.05, r'Distance vs Time', r'$\ell^\prime$')
    make_time_figures(data, time_data[0], stats_indices['phis'], 0, 2.5, r'Angle vs Time', r'$\phi$')

    make_success_figures(data, time_data[0], stats_indices['lengths'], 1, 1.3, r'Success vs Distance', r'$\ell^\prime$')
    make_success_figures(data, time_data[0], stats_indices['phis'], 0, 3.14, r'Success vs Angle', r'$\phi$')

    make_violin_figures(data, time_data[0], stats_indices['ell'], 1, 1.3, r'$\ell^\prime$ ratio for Planner Variations', r'$\ell^\prime$')
    make_violin_figures(data, time_data[0], stats_indices['phi'], 0, 3.14, r'Angles', r'$\phi$')


    # make_scaled_length_time_figures(data, time_data[0], stats_indices['lengths'], 1, 1.05, r'Distance vs Time', r'$\ell^\prime$')

    plotter.show()