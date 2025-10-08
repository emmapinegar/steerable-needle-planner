import numpy as np
import open3d as o3d
import os, fnmatch
import matplotlib.pyplot as plotter
from dataclasses import dataclass
import copy

from viz_utils import stats_indices, planners, viz_params, get_planner_indices, make_violin_figure


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

        n = 25 #window
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


        line = plotter.plot(averagetime, average, color=planners[i].color, linestyle=planners[i].linestyle, linewidth=1.75)
        lines += [line]

    if len(colors) > 0:
        plotter.setp(lines[0], color=colors[0])
    if len(colors) > 1:      
        plotter.setp(lines[1], color=colors[1])
    if len(colors) > 2:
        plotter.setp(lines[2], color=colors[2])
    if len(colors) > 3:
        plotter.setp(lines[3], color=colors[3])


    plotter.ylabel(ylabel)
    plotter.legend(labels)
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
        success_upper = []
        success_lower = []
        j = 0
        for x in data_ind:
            if len(x) > 0:
                plot_data.append(x[-1])
                j += 1
                next_success = j/np.shape(planner_indices)[0]
                success.append(100*next_success)
                # adjusted_proportion = (j + 2)/(np.shape(planner_indices)[0] + 4)
                # se_1 = adjusted_proportion*(1-adjusted_proportion)
                # se_2 = se_1/(np.shape(planner_indices)[0] + 4)
                # standard_error = np.sqrt(se_2)
                # margin_of_error = standard_error*2
                # success_upper.append(100*next_success + 100*margin_of_error)
                # success_lower.append(100*next_success - 100*margin_of_error)

        if len(plot_data) > 0:

            colors += [planners[i].color]
            labels += [planners[i].label]

            plot_data = np.array(plot_data)
            sortedinds = np.argsort(plot_data)

            plot_data = plot_data[sortedinds]
            line = plotter.plot(plot_data, success, color=planners[i].color, linestyle=planners[i].linestyle, linewidth=1.75)
            # plotter.fill_between(time, success_upper, success_lower, color=planners[i].color, alpha=viz_params['alpha']/2)
            # plotter.plot(time, success_upper, color=planners[i].color, alpha=viz_params['alpha'])
            # plotter.plot(time, success_lower, color=planners[i].color, alpha=viz_params['alpha'])
            # lines += [line]

    # if len(colors) > 0:
    #     plotter.setp(lines[0], color=colors[0])
    # if len(colors) > 1:      
    #     plotter.setp(lines[1], color=colors[1])
    # if len(colors) > 2:
    #     plotter.setp(lines[2], color=colors[2])
    # if len(colors) > 3:
    #     plotter.setp(lines[3], color=colors[3])


    plotter.ylabel('Success Percentage')
    plotter.legend(labels)

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


def make_success_time_figures(data, time_data):
    fig = plotter.figure(figsize=[15, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]
    rows = 2
    cols = num_plots//rows
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
                            make_success_time_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], stats_indices['lengths'], r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' env = $ %d$' %env + r' var = $ %d$' %varcurv, r'Success Percentage', y_min=0, y_max=100)
                            fig_ind += 1
    plotter.suptitle(r'Success vs Time', fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)


def make_time_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[15, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]
    rows = 2
    cols = num_plots//rows
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
                            make_time_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' env = $ %d$' %env + r' var = $ %d$' %varcurv, axis_label, y_min=y_min, y_max=y_max)
                            fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)


def make_success_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[15, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]
    rows = 2
    cols = num_plots//rows
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
                            make_success_data_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' env = $ %d$' %env + r' var = $ %d$' %varcurv, axis_label, x_min =y_min, x_max = y_max)
                            fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)


def make_violin_figures(data, time_data, index, y_min, y_max, fig_title, axis_label):
    fig = plotter.figure(figsize=[15, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]
    rows = 2
    cols = num_plots//rows
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
                                make_violin_figure(plan_data, index, r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' env = $ %d$' %env + r' var = $ %d$' %varcurv, axis_label, y_min=y_min, y_max=y_max, ylog=False)
                                fig_ind += 1
    plotter.suptitle(fig_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)  


def make_success_heat_figures(data, time_data):
    fig = plotter.figure(figsize=[15, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]
    rows = 2
    cols = num_plots//rows
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
                            make_success_heat_figure(varcurv_data, time_data[env_inds,:][phi_inds,:][kappa_inds,:][varcurv_inds,:], stats_indices['lengths'], r'$\kappa$ = %.1f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' env = $ %d$' %env + r' var = $ %d$' %varcurv, r'Success Percentage', y_min=0, y_max=100)
                            fig_ind += 1
    plotter.suptitle(r'Success vs Pairs', fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)


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

    # make_success_brain_figure(data)
    # make_success_brain_figures(data)

    make_success_heat_figures(data, time_data[0])


    make_success_time_figures(data, time_data[0])
    make_time_figures(data, time_data[0], stats_indices['lengths'], 1, 1.3, r'Distance vs Time', r'$\ell^\prime$')
    make_time_figures(data, time_data[0], stats_indices['phis'], 0, 3.14, r'Angle vs Time', r'$\phi$')

    make_success_figures(data, time_data[0], stats_indices['lengths'], 1, 1.3, r'Success vs Distance', r'$\ell^\prime$')
    make_success_figures(data, time_data[0], stats_indices['phis'], 0, 3.14, r'Success vs Angle', r'$\phi$')

    make_violin_figures(data, time_data[0], stats_indices['ell'], 1, 1.3, r'$\ell^\prime$ ratio for Planner Variations', r'$\ell^\prime$')
    make_violin_figures(data, time_data[0], stats_indices['phi'], 0, 3.14, r'Angles', r'$\phi$')


    plotter.show()