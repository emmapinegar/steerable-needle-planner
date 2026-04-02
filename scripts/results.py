import numpy as np
import open3d as o3d
import os, fnmatch
import matplotlib.pyplot as plotter
from dataclasses import dataclass
import copy
import csv
import scipy
from scipy import optimize
from scipy import stats
from magnet import Magnet
from viz_utils import stats_indices, planners, viz_params, get_planner_indices, make_violin_figure


def get_phi_substr(maxphi):
    phi_frac = maxphi/180
    if phi_frac == 1:
        phi_str = " " + r'$ {\phi_{\Sigma max} = \pi}$'
    elif phi_frac == 0.5:
        phi_str = " " + r'$ {\phi_{\Sigma max} = \frac{\pi}{2}}$'
    elif phi_frac == 1.5:
        phi_str =" " +  r' {\phi_{\Sigma max} = 3\pi/2}'    
    
    return phi_str

def get_phi_alpha(maxphi):
    alpha = maxphi/270
    return alpha

def get_phi_width(maxphi):
    linewidth = 1.75/(maxphi/180)
    return linewidth

def get_kappa_alpha(kappa_index, kappas):
    alpha = ((kappa_index+1)/np.shape(kappas)[0])
    return alpha 

def get_kappa_linewidth(kappa_index, kappas):
    linewidth = 6*((np.shape(kappas)[0]-kappa_index-1)/(np.shape(kappas)[0])) + 1.5
    return linewidth


def make_time_figure(data, time_data, index, title, xlabel, ylabel, x_min, x_max, y_min, y_max, xlog, ylog):
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
    plotter.title(title, fontsize=viz_params['subtitlesize'])

    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
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

        window = 200
        n = window//2 #window

        if np.shape(flat)[0] < window:
            
            if np.shape(flat)[0] > n//2:
                n = n//5
            else:
                n = 0
            
        average = np.zeros(np.shape(flat))
        averagetime = np.zeros(np.shape(flat))
        for j in range(np.shape(flat)[0]):
            min_ind = max(0, j - n)
            max_ind = min(np.shape(flat)[0], j + n + 1)
            average[j] = np.average(flat[min_ind:max_ind])
            averagetime[j] = np.average(time[min_ind:max_ind])

        phi_str = get_phi_substr(data[0,stats_indices['maxphi']])

        plotter.plot(averagetime, average, color=planners[i].color, linestyle=planners[i].linestyle, label=planners[i].label + phi_str, alpha=get_phi_alpha(data[0,stats_indices['maxphi']]), linewidth=get_phi_width(data[0,stats_indices['maxphi']]), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])

    # plotter.ylabel(ylabel)
    # plotter.legend(ncols=2)
    x_max = np.floor(np.max(data[:,stats_indices['time']]))
    plotter.xlim([x_min, x_max])
    plotter.ylim([y_min, y_max])
    # plotter.xlabel(xlabel)
    if xlog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')


def make_success_time_figure(data, time_data, index, title, xlabel, ylabel, x_min, x_max, y_min, y_max, xlog, ylog):
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
    plotter.title(title, fontsize=viz_params['subtitlesize'])

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

        j = 0
        for x in time_data[planner_indices, stats_indices['times'] - stats_indices['times']]:
            if len(x) > 0:
                time.append(x[0])
                j += 1
                next_success = j*100/np.shape(planner_indices)[0]
                success.append(next_success) 

        if len(time) == 1:
            time = [time[0] - time[0]/10, time[0] - time[0]/100, time[0]]
            success = [0, 0, success[0]]

        if len(time) > 0:
            time = np.array(time)
            sortedinds = np.argsort(time)
            time = time[sortedinds]
            phi_str = get_phi_substr(data[0,stats_indices['maxphi']])           
            plotter.plot(time, success, color=planners[i].color, label=planners[i].label + phi_str, linestyle=planners[i].linestyle, alpha=get_phi_alpha(data[0,stats_indices['maxphi']]), linewidth=get_phi_width(data[0,stats_indices['maxphi']]), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])

    # plotter.ylabel(ylabel)
    # plotter.legend(ncols=2)
    x_max = 1.1*np.floor(np.max(data[:,stats_indices['time']])) 
    plotter.xlim([x_min, x_max])
    plotter.ylim([y_min,y_max])
    # plotter.xlabel(xlabel)

    if xlog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')
        

def make_success_data_figure(data, time_data, index, title, xlabel, ylabel, x_min, x_max, y_min, y_max, xlog, ylog):
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
    plotter.title(title, fontsize=viz_params['subtitlesize'])

    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])

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
                next_success = j*100/np.shape(planner_indices)[0]
                success.append(next_success)


        if len(plot_data) > 0:
            plot_min = min(plot_data)
            success = [0, 0] + success
            plot_data = [plot_min -  0.1*(plot_min-x_min), plot_min-0.01*(plot_min-x_min)] + plot_data

            plot_data = np.array(plot_data)
            sortedinds = np.argsort(plot_data)

            plot_data = plot_data[sortedinds]
            phi_str = get_phi_substr(data[0,stats_indices['maxphi']])  
            plotter.plot(plot_data, success, color=planners[i].color, linestyle=planners[i].linestyle, label=planners[i].label + phi_str, alpha=get_phi_alpha(data[0,stats_indices['maxphi']]), linewidth=get_phi_width(data[0,stats_indices['maxphi']]), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])

    # plotter.ylabel(ylabel)
    # plotter.legend(ncols=2)

    plotter.xlim([x_min, x_max])
    plotter.ylim([y_min, y_max])
    # plotter.xlabel(xlabel)

    if xlog:
        plotter.xscale('log')
    else:
        plotter.xscale('linear')


def make_success_heat_figure(data, time_data, index, title, xlabel, ylabel, x_min, x_max, y_min, y_max, xlog, ylog):
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
    numpairs = np.shape(pairs)[0]
    if not numpairs % 500 == 0:
        numpairs = (numpairs//500 + 1)*500

    numplanners = np.shape(np.where(data[:,stats_indices['sg_index']] == pairs[0])[0])[0]
    pairs_success = np.zeros(numpairs)
    min_ind = np.min(pairs)
    for i in pairs:
        pair_data = np.where(data[:,stats_indices['sg_index']] == i)[0]
        # print(pair_data)
        for next_data in pair_data:
            if data[next_data, stats_indices['success']] == 1:
                pairs_success[int(i - min_ind)] += 1

    # for j in range(len(planners)):
    #     planner_indices = get_planner_indices(data, planners[j])

    #     for i in range(np.shape(planner_indices)[0]):
    #         if data[planner_indices[i], stats_indices['success']] == 1:
    #             print(data[planner_indices[i],:])
    #             pairs_success[int(data[planner_indices[i], stats_indices['sg_index']])] += 1
    pairs_success = np.transpose(pairs_success)
    # pairs_success = np.divide(pairs_success, numplanners)
    if numpairs > 500:
        pairs_success = np.reshape(pairs_success, (50,-1))
        plotter.imshow(pairs_success,vmax=16)
    else:
        pairs_success = np.reshape(pairs_success, (50,-1))
        pairs_success = np.transpose(pairs_success)
        plotter.imshow(pairs_success,vmax=numplanners,vmin=0)
    
    plotter.xlabel(xlabel)
    plotter.ylabel(ylabel)


def make_violin_figures(data, time_data, index, y_min, y_max, figure_title, axis_label):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])

    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]*np.shape(varcurvs)[0]
    cols = np.shape(kappas)[0]
    rows = max(num_plots//cols, 1)
    fig_ind = 1
    # make figures for each of the kappa values used in experiments

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for l in range(np.shape(varcurvs)[0]):
            varcurv = varcurvs[l]
            varcurv_inds = np.where(env_data[:,stats_indices['varcurv']] == varcurv)[0]
            varcurv_data = env_data[varcurv_inds,:]
            for k in range(np.shape(phis)[0]):
                phi = phis[k]
                phi_inds = np.where(varcurv_data[:,stats_indices['maxphi']] == phi)[0]
                phi_data = varcurv_data[phi_inds,:]
                if np.shape(phi_data)[0] > 0:                
                    for i in range(np.shape(kappas)[0]):
                        kappa = kappas[i]
                        kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                        kappa_data = phi_data[kappa_inds,:]

                        if np.shape(kappa_data)[0] > 0:
                            plan_data_ind = np.where(kappa_data[:,stats_indices['success']] == 1)[0]
                            plan_data = kappa_data[plan_data_ind,:]
                            if np.shape(plan_data)[0] > 0:

                                plotter.subplot(rows, cols, fig_ind)
                                make_violin_figure(plan_data, index, r'$r_{min}$=%dmm' % kappa + r' $\phi$=$%d$' % phi + r' var=$%d$' %varcurv, axis_label, y_min=y_min, y_max=y_max, ylog=False)
                        fig_ind += 1
                    if not (fig_ind % np.shape(kappas)[0] == 1):
                        fig_ind = (l + k + 1)*np.shape(kappas)[0] + 1 
    plotter.suptitle(figure_title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.025, hspace=0.3, wspace=0.2)


def make_generic_figures(data, time_data, figure_function, figure_title, xaxis_label, yaxis_label, data_index, x_min, x_max, y_min, y_max, x_log, y_log):
    fig = plotter.figure(figsize=[17, 9])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])

    num_plots = np.shape(kappas)[0]*np.shape(varcurvs)[0]
    cols = np.shape(kappas)[0]
    rows = max(num_plots//cols, 1)
    # print(f"rows: {rows} cols: {cols}")
    fig_ind = 1
    # make figures for each of the kappa values used in experiments

    if data[0,stats_indices['multi']] == 1:
        figure_title = figure_title + " Multi-Threaded"
    else:
        figure_title = figure_title + " Single-Threaded"
    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for l in range(np.shape(varcurvs)[0]):
            varcurv = varcurvs[l]
            varcurv_inds = np.where(env_data[:,stats_indices['varcurv']] == varcurv)[0]
            varcurv_data = env_data[varcurv_inds,:]
            fig_ind = (l)*np.shape(kappas)[0] + 1 
            for k in range(np.shape(phis)[0]):
                phi = phis[k]
                phi_inds = np.where(varcurv_data[:,stats_indices['maxphi']] == phi)[0]
                phi_data = varcurv_data[phi_inds,:]
                if np.shape(phi_data)[0] > 0:                
                    for i in range(np.shape(kappas)[0]):
                        kappa = kappas[i]
                        kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                        kappa_data = phi_data[kappa_inds,:]

                        if np.shape(kappa_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            var_str = ""
                            if varcurv > 0:
                                var_str = "dynamic" 
                            figure_function(kappa_data, time_data[env_inds,:][varcurv_inds,:][phi_inds,:][kappa_inds,:], data_index, r'$r_{min}$=%d mm ' % kappa + var_str, xaxis_label, yaxis_label, x_min, x_max, y_min, y_max, x_log, y_log)
                            if fig_ind % cols == 1:
                                plotter.ylabel(yaxis_label, fontsize=viz_params['labelsize'])
                                                                
                            if fig_ind == 1:
                                plotter.legend(ncols=8, columnspacing=1.1, handlelength=2, handletextpad=0.25, borderpad=0.3, bbox_to_anchor=(-0.1, 1.05), loc='lower left', draggable=True, fontsize=viz_params['legendsize'])

                            if fig_ind > cols:
                                plotter.xlabel(xaxis_label, fontsize=viz_params['labelsize'])
                            plotter.tick_params(labelsize=viz_params['ticksize'])
                        fig_ind += 1
                    # if not (fig_ind % np.shape(kappas)[0] == 1):
                    fig_ind = (l)*np.shape(kappas)[0] + 1 
    plotter.suptitle(figure_title, fontsize=viz_params['titlesize'])
    plotter.subplots_adjust(top=0.85, bottom=0.0675, right=0.9915, left=0.0485, hspace=0.23, wspace=0.2)



def make_generic_test_figures(data, time_data, figure_function, figure_title, xaxis_label, yaxis_label, data_index, x_min, x_max, y_min, y_max, x_log, y_log):
    max_num_pairs = 500
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    pairs = np.unique(data[:,stats_indices['sg_index']])
    min_pair = (np.min(pairs)//max_num_pairs)*max_num_pairs
    # print(f"{np.max(pairs)} {np.max(pairs)+1} {(np.max(pairs)+1)//max_num_pairs} {((np.max(pairs)+1)//max_num_pairs)*max_num_pairs}")
    max_pair = ((np.max(pairs)+1)//max_num_pairs)*max_num_pairs
    if max_pair < min_pair + max_num_pairs:
        max_pair = min_pair + max_num_pairs
    # if np.max(pairs)%max_num_pairs > 0:
    #     max_pair = max_pair + max_num_pairs
    
    
    pair_start_inds = np.arange(min_pair, max_pair, step=max_num_pairs)
    num_plots = np.shape(pair_start_inds)[0]
    cols = num_plots
    rows = 1

    # make figures for each of the kappa values used in experiments
    # if np.shape(kappas)[0] > 1:
    plotter.figure(figsize=[17,9])
    line_offset = 0
    line_increment = 1
    if max_pair - min_pair > max_num_pairs:
        linewidth = 0.25
    else:
        linewidth = 1.5
    plotter.title(f"Planner Successes Planning Problems {round(min_pair)} to {round(max_pair)}", fontsize=viz_params['titlesize'])
    locations = []
    yticklabels = []
    for k in range(np.shape(kappas)[0]):
        for p in range(np.shape(planners)[0]):
            locations += [line_offset]
            yticklabels += [planners[p].label + f" " + r'$r_{min}$' + f"={round(kappas[k])}"]
            next_data = data[np.where(np.logical_and(data[:,stats_indices['planner']] == planners[p].index, data[:,stats_indices['minrad']] == kappas[k]))[0], :]
            rgrrt25_pairs = data[np.where(np.logical_and(data[:,stats_indices['success']] == 1 ,np.logical_and(data[:,stats_indices['planner']] == 1, data[:,stats_indices['minrad']] == 25)))[0], stats_indices['sg_index']]
            # print(np.shape(rgrrt25_pairs)[0])
            successful_pairs = next_data[np.where(next_data[:,stats_indices['success']] == 1)[0], stats_indices['sg_index']]
            notrgrrt_pairs =  np.setdiff1d(successful_pairs, rgrrt25_pairs)

            

            plotter.eventplot(notrgrrt_pairs, linelengths=line_increment, linewidths=linewidth, colors=planners[p].color, alpha=1, lineoffsets=line_offset)   #get_kappa_alpha(k,kappas)
            plotter.eventplot(np.intersect1d(rgrrt25_pairs, successful_pairs), linelengths=line_increment, colors=planners[p].color, alpha=0.5, lineoffsets=line_offset)
            line_offset += line_increment

    plotter.xlim([min_pair-1, max_pair+1])
    # plotter.xlim([1499, 2001])
    plotter.ylim([-0.6*line_increment, line_offset - 0.4*line_increment])
    plotter.xlabel("Planning Problem", fontsize=viz_params['labelsize'])
    plotter.ylabel("Planner", fontsize=viz_params['labelsize'])
    plotter.yticks(locations, yticklabels)
    plotter.tick_params(labelsize=viz_params['ticksize']) 
    plotter.subplots_adjust(top=0.95, bottom=0.066, right=0.985, left=0.12, hspace=0.26, wspace=0.2)


    # for k in range(np.shape(kappas)[0]):
    #     plotter.figure(figsize=[16,8])
    #     line_offset = 0
    #     line_increment = 1
    #     plotter.title(f"Planner Successes {min_pair} to {max_pair} with r = {kappas[k]}", fontsize=viz_params['titlesize'])        
    #     for p in range(np.shape(planners)[0]):
    #         next_data = data[np.where(np.logical_and(data[:,stats_indices['planner']] == planners[p].index, data[:,stats_indices['minrad']] == kappas[k]))[0], :]
    #         successful_pairs = next_data[np.where(next_data[:,stats_indices['success']] == 1)[0], stats_indices['sg_index']]

    #         plotter.eventplot(successful_pairs, linelengths=line_increment, linewidths=1, colors=planners[p].color, alpha=0.5, lineoffsets=line_offset)   #get_kappa_alpha(k,kappas)
    #         line_offset += line_increment

    #     plotter.xlim([min_pair-1, max_pair+1])
    #     # plotter.xlim([1499, 2001])
    #     plotter.ylim([-line_increment, line_offset])
    #     plotter.subplots_adjust(top=0.92, bottom=0.05, right=0.99, left=0.05, hspace=0.26, wspace=0.2)    



    for k in range(np.shape(pair_start_inds)[0]):
        # plotter.subplot(rows, cols, k+1)
        
        pair_start = pair_start_inds[k]
        pair_inds = np.where(np.logical_and(data[:,stats_indices['sg_index']] >= pair_start, data[:,stats_indices['sg_index']] < pair_start + max_num_pairs))[0]
        pair_data = data[pair_inds,:]
        success_pairs = np.where(pair_data[:,stats_indices['sg_index']])
        pair_kappas = np.unique(pair_data[:,stats_indices['minrad']])
        if np.shape(pair_data)[0] > 0:      
            fig = plotter.figure(figsize=[17, 9])          
            for i in range(np.shape(kappas)[0]):
                kappa = kappas[i]
                kappa_inds = np.where(pair_data[:,stats_indices['minrad']] == kappa)[0]
                kappa_data = pair_data[kappa_inds,:]

                if np.shape(kappa_data)[0] > 0:

                    var_str = "dynamic" 

                    for p in range(np.shape(planners)[0]):
                        planner_inds = np.where(kappa_data[:,stats_indices['planner']] == planners[p].index)[0]
                        next_time_data = time_data[pair_inds,:][kappa_inds,:][planner_inds,:]
                        next_data = kappa_data[planner_inds,:]
                        time = []
                        angle = []
                        length = []
                        success = []
                        times = []
                        angles = []
                        lengths = []
                        j = 0
    
                        for x in next_time_data[:, stats_indices['times'] - stats_indices['times']]:
                            if len(x) > 0:
                                time.append(x[0])
                                for xi in x:
                                    times.append(xi)
                                j += 1
                                next_success = j*100/max_num_pairs
                                success.append(next_success) 

                        for x in next_time_data[:, stats_indices['lengths'] - stats_indices['times']]:
                            if len(x) > 0:
                                length.append(x[-1])
                                for xi in x:
                                    lengths.append(xi)                        

                        for x in next_time_data[:, stats_indices['phis'] - stats_indices['times']]:
                            if len(x) > 0:
                                angle.append(x[-1])
                                for xi in x:
                                    angles.append(xi)

                        if len(time) == 1:
                            time = [time[0] - time[0]/10, time[0] - time[0]/100, time[0]]
                            success = [0, 0, success[0]]
                            angle = [angle[0] - angle[0]/10, angle[0] - angle[0]/100, angle[0]]
                            length = [length[0] - length[0]/10, length[0] - length[0]/100, length[0]]

                        if len(time) > 0:
                            time = np.array(time)
                            sortedinds = np.argsort(time)
                            time = time[sortedinds]
                            length = np.array(length)
                            sortedlengthinds = np.argsort(length)
                            length = length[sortedlengthinds]
                            angle = np.array(angle)
                            sortedangleinds = np.argsort(angle)
                            angle = angle[sortedangleinds]

                            times = np.array(times)
                            sortedtimeinds = np.argsort(times)
                            times = times[sortedtimeinds]
                            lengths = np.array(lengths)
                            lengths = lengths[sortedtimeinds]
                            angles = np.array(angles)
                            angles = angles[sortedtimeinds]
                            average_time = np.zeros(np.shape(times))
                            average_length = np.zeros(np.shape(lengths))
                            average_angle = np.zeros(np.shape(angles))

                            window = 200
                            n = window//2 #window

                            if np.shape(times)[0] < window:
                                
                                if np.shape(times)[0] > n//2:
                                    n =  n//5
                                else:
                                    n = 0

                            for j in range(np.shape(times)[0]):
              
                                min_ind = max(0, j - n)
                                max_ind = min(np.shape(times)[0], j + n + 1)
       
                                average_length[j] = np.average(lengths[min_ind:max_ind])
                                average_angle[j] = np.average(angles[min_ind:max_ind])
                                average_time[j] = np.average(times[min_ind:max_ind])                            


                            roc_str = f" " + r'$r_{min}$' + f"={int(kappa)}"

                            phi_str = r'$\phi_{\Sigma}$'

  
                            plotter.subplot(2,3,1)
                            plotter.plot(time, success, color=planners[p].color, label=planners[p].label + roc_str, linestyle=planners[p].linestyle, alpha=get_kappa_alpha(i, kappas), linewidth=get_kappa_linewidth(i, kappas), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])
                            plotter.title("Success vs. Time", fontsize=viz_params['subtitlesize'])
                            plotter.xlim([0.0001, 11])
                            plotter.xscale('log')
                            plotter.ylim([-1, 101])
                            plotter.ylabel("Success Percentage", fontsize=viz_params['labelsize'])
                            plotter.xlabel("Time (seconds)", fontsize=viz_params['labelsize'])
                            plotter.tick_params(labelsize=viz_params['ticksize'])

                            if np.shape(pair_kappas)[0] > 1:
                                plotter.legend(ncols=8, columnspacing=1.25, handlelength=2, handletextpad=0.5, borderpad=0.3, bbox_to_anchor=(-0.2, 1.095), loc='lower left', draggable=True, fontsize=viz_params['legendsize'])
                            else:
                                plotter.legend(ncols=8, columnspacing=1.5, handlelength=2, handletextpad=0.5, borderpad=0.3, bbox_to_anchor=(-0.1, 1.085), loc='lower left', draggable=True, fontsize=viz_params['legendsize'])

                            plotter.subplot(2,3,2)
                            plotter.plot(length, success, color=planners[p].color, label=planners[p].label + roc_str, linestyle=planners[p].linestyle, alpha=get_kappa_alpha(i, kappas), linewidth=get_kappa_linewidth(i, kappas), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])
                            plotter.title("Success vs. Relative Path Length", fontsize=viz_params['subtitlesize'])
                            plotter.xlim([0.9995, 1.04])
                            plotter.ylim([-1, 101])
                            plotter.ylabel("Success Percentage", fontsize=viz_params['labelsize'])
                            plotter.xlabel("Relative Path Length", fontsize=viz_params['labelsize'])
                            plotter.tick_params(labelsize=viz_params['ticksize'])

                            plotter.subplot(2,3,3)
                            plotter.plot(angle, success, color=planners[p].color, label=planners[p].label + roc_str, linestyle=planners[p].linestyle, alpha=get_kappa_alpha(i, kappas), linewidth=get_kappa_linewidth(i, kappas), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])
                            plotter.title(f"Success vs. {phi_str}", fontsize=viz_params['subtitlesize'])
                            plotter.xlim([0, 2])
                            plotter.ylim([-1, 101])
                            plotter.ylabel("Success Percentage", fontsize=viz_params['labelsize'])
                            plotter.xlabel(f"{phi_str} (radians)", fontsize=viz_params['labelsize'])
                            plotter.tick_params(labelsize=viz_params['ticksize'])

                            plotter.subplot(2,3,4)
                            plotter.plot(average_time, average_length, color=planners[p].color, label=planners[p].label + roc_str, linestyle=planners[p].linestyle, alpha=get_kappa_alpha(i, kappas), linewidth=get_kappa_linewidth(i, kappas), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])
                            plotter.title("Relative Path Length vs. Time", fontsize=viz_params['subtitlesize'])
                            plotter.xlim([0.001, 11])
                            plotter.xscale('log')
                            plotter.ylim([0.9995, 1.04])
                            plotter.ylabel("Relative Path Length", fontsize=viz_params['labelsize'])
                            plotter.xlabel("Time (seconds)", fontsize=viz_params['labelsize']) 
                            plotter.tick_params(labelsize=viz_params['ticksize'])
                             
                            plotter.subplot(2,3,5)
                            plotter.plot(average_time, average_angle, color=planners[p].color, label=planners[p].label + roc_str, linestyle=planners[p].linestyle, alpha=get_kappa_alpha(i, kappas), linewidth=get_kappa_linewidth(i, kappas), dash_capstyle=viz_params['capstyle'], dash_joinstyle=viz_params['dashjoinstyle'])
                            plotter.title(f"{phi_str} vs. Time", fontsize=viz_params['subtitlesize'])
                            plotter.xlim([0.001, 11])
                            plotter.xscale('log')
                            plotter.ylim([0, 2])
                            plotter.ylabel(f"{phi_str} (radians)", fontsize=viz_params['labelsize'])
                            plotter.xlabel("Time (seconds)", fontsize=viz_params['labelsize'])    
                            plotter.tick_params(labelsize=viz_params['ticksize'])                                                    

                # plotter.xlim([0.0001, 10])
                # plotter.xscale('log')
                # plotter.ylim([-1, 100])
                # plotter.ylabel(yaxis_label, fontsize=14)
                # plotter.xlabel(xaxis_label, fontsize=14)
            

            # plotter.subplot(2,3,6)
            # make_success_heat_figure(pair_data, time_data[pair_inds,:], stats_indices['sg_index'], r'Success vs. Pair', r'Goals', r'Starts', 0, 0, 0, 0, False, False)


            plotter.suptitle(f"Common Set of Planning Problems {round(pair_start)} to {round(pair_start + max_num_pairs)}", fontsize=viz_params['titlesize'])
            if np.shape(pair_kappas)[0] > 1:
                plotter.subplots_adjust(top=0.825, bottom=0.0675, right=0.985, left=0.06, hspace=0.35, wspace=0.21)
            else:
                plotter.subplots_adjust(top=0.85, bottom=0.0675, right=0.985, left=0.06, hspace=0.35, wspace=0.21)

            if np.shape(pair_start_inds)[0] > 1:
                get_pairwise_statistics(pair_data, time_data[pair_inds,:], 0.1, 1)
                # get_pairwise_statistics(pair_data, time_data[pair_inds,:], 0.75, 15)
        


def make_big_generic_figures(data, time_data, figure_function, figure_title, xaxis_label, yaxis_label, data_index, x_min, x_max, y_min, y_max, x_log, y_log):
    fig = plotter.figure(figsize=[16, 8])
    kappas = np.unique(data[:,stats_indices['minrad']])
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])

    num_plots = np.shape(kappas)[0]*np.shape(phis)[0]*np.shape(varcurvs)[0]
    cols = np.shape(kappas)[0]
    rows = max(num_plots//cols, 1)
    fig_ind = 1
    # make figures for each of the kappa values used in experiments

    for j in range(np.shape(envs)[0]):
        env = envs[j]
        env_inds = np.where(data[:,stats_indices['env']] == env)[0]
        env_data = data[env_inds,:]
        for l in range(np.shape(varcurvs)[0]):
            varcurv = varcurvs[l]
            varcurv_inds = np.where(env_data[:,stats_indices['varcurv']] == varcurv)[0]
            varcurv_data = env_data[varcurv_inds,:]
            # fig_ind = (l + k)*np.shape(kappas)[0] + 1 
            for k in range(np.shape(phis)[0]):
                phi = phis[k]
                phi_inds = np.where(varcurv_data[:,stats_indices['maxphi']] == phi)[0]
                phi_data = varcurv_data[phi_inds,:]
                if np.shape(phi_data)[0] > 0:                
                    for i in range(np.shape(kappas)[0]):
                        kappa = kappas[i]
                        kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                        kappa_data = phi_data[kappa_inds,:]

                        if np.shape(kappa_data)[0] > 0:
                            plotter.subplot(rows, cols, fig_ind)
                            figure_function(kappa_data, time_data[env_inds,:][varcurv_inds,:][phi_inds,:][kappa_inds,:], data_index, r'$r_{min}$=%dmm' % kappa + r' $\phi$=$%d$' % phi + r' var=$%d$' %varcurv, xaxis_label, yaxis_label, x_min, x_max, y_min, y_max, x_log, y_log)
                        fig_ind += 1
                    if not (fig_ind % np.shape(kappas)[0] == 1):
                        fig_ind = (l + k + 1)*np.shape(kappas)[0] + 1 
    plotter.suptitle(figure_title, fontsize=18)
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
                print(f"r: {kappa} var: {varcurv}")
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
            print(f"index: {int(data[i,stats_indices['sg_index']])} num: {pairs_success[int(data[i,stats_indices['sg_index']]) - min_pair] + 1} pair: {pairs[int(data[i,stats_indices['sg_index']]) - min_pair, :]}")
            pairs_success[int(data[i,stats_indices['sg_index']]) - min_pair] = pairs_success[int(data[i,stats_indices['sg_index']]) - min_pair] + 1

    colors = [[1,0,0], [0.75, 0.75, 0.75], [0.76, 0.74, 0.88], [0.63, 0.61, 0.77], [0.56, 0.52, 0.74], [0.53, 0.48, 0.69], [0.42, 0.33, 0.70], [0.26, 0.16, 0.63], [0.16, 0.01, 0.67]]

    start = o3d.geometry.TriangleMesh.create_coordinate_frame()
    ptcs = [start]

    for i in range(numpairs):
        if int(pairs_success[i]) > 0:
            point = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
            point.translate(pairs[i,0:3])
            point.paint_uniform_color(colors[int(pairs_success[i])])
            ptcs.append(point)

    for i in range(numpairs):
        if int(pairs_success[i]) > 0:
            point = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
            point.translate(pairs[i,3:6])
            point.paint_uniform_color(colors[int(pairs_success[i])])
            ptcs.append(point)

    for i in range(numpairs):
        point = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        point.translate(pairs[i,0:3])
        point.paint_uniform_color(colors[int(pairs_success[i])])
        ptcs.append(point)

    for i in range(numpairs):
        point = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
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
        distances = np.linspace(20, 180, 80, endpoint=True)
        # distances = np.divide(distances, 1000)
        needle = Magnet(np.array([[0], [0], [0]]), np.array([[0, 0, 1]]), 0.0018, radius=0.001)
     
        
        colors = [ '#332288', '#117733'] #,'#C33AAC','#BF0F67',  '#0063F8' '#40B3EC' '#44AA99' '#D46D7E' '#EF6E12'

        
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

        magnets = [samm, sammn52] 

        for i in range(len(magnets)):
            dist_points = []
            radius_points = []
            for distance in distances:
                      
                if distance > magnets[i].radius*1000 + 1:
                    magnets[i].position = np.array([[distance/1000], [0], [0]])                              #magnets[i].position + np.array([[magnets[i].radius], [0], [0]])
                    f, tau = needle.get_force_torque(magnets[i])

                    curvature = 1000/(np.linalg.norm(tau)*m + b)
                    # print(f"position: {distance} {magnets[i].position.reshape((-1,))} curvature: {curvature} tau: {np.linalg.norm(tau)} m: {magnets[i].mag} {magnets[i].m.reshape((-1,))}")
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
        
        radii = np.array([15, 25, 50, 100, 250])
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

def fix_lines(filename):
    newlines = []
    with open(filename) as file:
        file.readline()
        glue = ","
        
        for line in file:
            data = line.split(glue)
            data = data[0:14] + ["0"] + data[14:]
            # data[-1] += "\n"
            
            newline = glue.join(data)
            newlines += [newline]

    with open(filename, "w") as file:
        file.writelines(newlines)
        
def scale_distances(data, time_data, pairs_mins):
    pairs = np.unique(data[:,stats_indices['sg_index']])

    for pair in pairs:
        pair = int(pair)
        pair_inds = np.where(data[:,stats_indices['sg_index']] == pair)[0]
        pair_min = pairs_mins[pair]

        pair_time_data = time_data[pair_inds,stats_indices['lengths']-stats_indices['times']]

        data[pair_inds,stats_indices['ell']] = np.divide(data[pair_inds,stats_indices['ell']], pair_min)
        pair_time_data = np.divide(pair_time_data,pair_min)
        time_data[pair_inds,stats_indices['lengths']-stats_indices['times']] = pair_time_data

    return data, time_data

def get_min_ell(data, time_data):
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

    return pairs_mins

def get_success_im_diff(data, time_data):
    pairs = np.unique(data[:,stats_indices['sg_index']])
    kappas = np.unique(data[:,stats_indices['minrad']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    planners = np.unique(data[:, stats_indices['planner']])
    numpairs = int(np.max(pairs)) + 1
    if not numpairs % 500 == 0:
        numpairs = (numpairs//500 + 1)*500
    pairs_success = np.zeros((numpairs, np.shape(varcurvs)[0], np.shape(phis)[0]))
    for pair in pairs:
        pair = int(pair)
        pair_inds = np.where(data[:,stats_indices['sg_index']] == pair)[0]
        pair_data = data[pair_inds,:]
        for next_ind in pair_inds:
            next_pair = data[next_ind, :]
            if next_pair[stats_indices['success']] == 1:
                if next_pair[stats_indices['maxphi']] == phis[0]:
                    pairs_success[pair, int(next_pair[stats_indices['varcurv']]), 0] += 1
                else:
                    pairs_success[pair, int(next_pair[stats_indices['varcurv']]), 1] += 1

    
    vmax = np.max(pairs_success)
    vmin = -1
    figure = plotter.figure(figsize=[16, 8])
    plotter.subplot(1,3,1)
    plotter.imshow((pairs_success[:,0,0]-pairs_success[:,0,1]).reshape(50,-1),vmin=vmin, vmax=vmax)
    plotter.colorbar()

    plotter.subplot(1,3,2)
    plotter.imshow((pairs_success[:,1,0]-pairs_success[:,1,1]).reshape(50,-1),vmin=vmin, vmax=vmax)
    plotter.colorbar()

    plotter.subplot(1,3,3)
    plotter.imshow((pairs_success[:,0,0]+pairs_success[:,0,1]).reshape(50,-1)-(pairs_success[:,1,0]+pairs_success[:,1,1]).reshape(50,-1))
    plotter.colorbar()

def get_exhausted_planners(data, time_data):
    exhausted_planners = np.where(data[:,stats_indices['time']] < 10)[0]
    successful_planners = np.where(data[exhausted_planners, stats_indices['success']] == 1)[0]
    exhausted_pairs = np.unique(data[exhausted_planners, stats_indices['sg_index']])
    print(data[exhausted_planners, :][successful_planners, :])
    print(np.shape(exhausted_pairs))
    exhausted_rads = np.unique(data[exhausted_planners, stats_indices['minrad']])
    for rad in exhausted_rads:
        exhausted_rads = np.where(data[exhausted_planners, stats_indices['minrad']] == rad)[0]
        exhausted_rad_pairs = np.unique(data[exhausted_planners,:][exhausted_rads,stats_indices['sg_index']])
        print(np.shape(exhausted_rad_pairs))
        print(np.average(data[exhausted_planners,:][exhausted_rads,stats_indices['time']]))

def get_pairwise_statistics(data, time_data, min_prop, min_number):
    kappas = np.unique(data[:,stats_indices['minrad']])
    threads = np.unique(data[:, stats_indices['multi']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    planners_ = np.unique(data[:, stats_indices['planner']])    
    pairs = np.unique(data[:,stats_indices['sg_index']])

    good_pairs = []
    pair_sols = 0
    # plotter.figure(figsize=[16,8])
    for next_pair_arr_ind in range(np.shape(pairs)[0]):
        pair_ind = int(pairs[next_pair_arr_ind])
        pair_data_inds = np.where(data[:,stats_indices['sg_index']] == pair_ind)[0]
        pair_data = data[pair_data_inds, :]

        # np.shape(pair_data)[0] -1

        if min(min_number, min_prop*np.shape(pair_data)[0]) <= np.shape(np.where(pair_data[:,stats_indices['success']] == 1)[0])[0]:
            good_pairs += [pair_data_inds]
            pair_sols += 1
            # for next_pair_data_ind in range(np.shape(pair_data)[0]):
            #     if pair_data[next_pair_data_ind,stats_indices['success']] == 1:

            #         plotter.scatter(pair_ind, pair_data[next_pair_data_ind,stats_indices['ell']], s=6, c=planners[int(pair_data[next_pair_data_ind,stats_indices['planner']])-1].color, alpha=0.75*(pair_data[next_pair_data_ind,stats_indices['minrad']]/kappas[-1]), marker=planners[int(pair_data[next_pair_data_ind,stats_indices['planner']])-1].marker)


    # plotter.xlim([np.min(pairs)-1, np.max(pairs)+1])
    # plotter.ylim([0.9995, 1.01])
    # plotter.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95)
    # plotter.show()

    pair_plot_inds = np.reshape(np.array(good_pairs), (-1,))
    # print(np.shape(pair_plot_inds))
    print(f"problems solved: {pair_sols}/500 = {100*pair_sols/500}%")

    if np.shape(pair_plot_inds)[0] > 0:
        make_generic_test_figures(data[pair_plot_inds,:], time_data[pair_plot_inds,:], make_success_time_figure, r'Success vs. Time', r'Time (seconds)', r'Success Percentage', stats_indices['lengths'], 0.0001, 100, -1, 100, True, False)
        # get_statistics(data[pair_plot_inds,:], time_data[pair_plot_inds,:])
        # get_statistics(data, time_data)


def get_statistics(data, time_data):
    kappas = np.unique(data[:,stats_indices['minrad']])
    threads = np.unique(data[:, stats_indices['multi']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    phis = np.flip(phis)
    varcurvs = np.unique(data[:, stats_indices['varcurv']])
    planners_ = np.unique(data[:, stats_indices['planner']])
    problems = np.unique(data[:,stats_indices['sg_index']])

    max_num_problems = 500
    min_problem = (np.min(problems)//max_num_problems)*max_num_problems
    max_problem = np.max(problems)

    max_problem = ((np.max(problems)+1)//max_num_problems)*max_num_problems
    if max_problem < min_problem + max_num_problems:
        max_problem = min_problem + max_num_problems
    
    problem_start_inds = np.arange(min_problem, max_problem, step=max_num_problems)

    # make figures for each of the kappa values used in experiments
    rows = []
    for prob_start_ind in range(np.shape(problem_start_inds)[0]):
        problem_start = problem_start_inds[prob_start_ind]
        problem_inds = np.where(np.logical_and(data[:,stats_indices['sg_index']] >= problem_start, data[:, stats_indices['sg_index']] < problem_start + max_num_problems))[0]
        problem_data = data[problem_inds,:]
        for j in range(np.shape(threads)[0]):
            thread = threads[j]
            thread_inds = np.where(problem_data[:,stats_indices['multi']] == thread)[0]
            thread_data = problem_data[thread_inds,:]


            if thread > 0:
                multi_str = 'multi'
            else:
                multi_str = 'single'
            print(f"{multi_str} {thread} {np.shape(thread_data)}")
            for l in range(np.shape(varcurvs)[0]):
                runtimes = np.floor(thread_data[:,stats_indices['time']])
                average_time = np.average(runtimes)
                max_time_inds = np.where(runtimes > average_time)[0]
                if np.shape(max_time_inds)[0] > 0:
                    runtimes = runtimes[max_time_inds]
                    max_time = np.floor(np.round(np.average(runtimes)))
                else:
                    max_time = average_time

                varcurv = varcurvs[l]
                varcurv_inds = np.where(thread_data[:,stats_indices['varcurv']] == varcurv)[0]
                varcurv_data = thread_data[varcurv_inds,:]
                # fig_ind = (l + k)*np.shape(kappas)[0] + 1 
                if varcurv > 0:
                    varcurv_str = 'yes'
                else:
                    varcurv_str = 'no'
                for k in range(np.shape(phis)[0]):
                    phi = phis[k]
                    phi_inds = np.where(varcurv_data[:,stats_indices['maxphi']] == phi)[0]
                    phi_data = varcurv_data[phi_inds,:]
                    if np.shape(phi_data)[0] > 0:                
                        for i in range(np.shape(kappas)[0]):
                            kappa = kappas[i]
                            kappa_inds = np.where(phi_data[:,stats_indices['minrad']] == kappa)[0]
                            kappa_data = phi_data[kappa_inds,:]
                            
                            if np.shape(kappa_data)[0] > 0:
                                kappa_problems_solved = np.unique(kappa_data[np.where(kappa_data[:,stats_indices['success']])[0],stats_indices['sg_index']])
                                print(f"problems: {round(problem_start)} - {round(problem_start + max_num_problems)} rad: {kappa} phi: {phi} var: {varcurv} max time: {max_time} planning problems: {np.min(kappa_data[:,stats_indices['sg_index']])} - {np.max(kappa_data[:,stats_indices['sg_index']])} solved: {np.shape(kappa_problems_solved)[0]}/500 = {100*np.shape(kappa_problems_solved)[0]/500}%")
                                for p in range(np.shape(planners_)[0]):
                                    planner_ind = planners_[p]
                                    planner_inds = np.where(kappa_data[:,stats_indices['planner']] == planner_ind)[0]
                                    planner_data = kappa_data[planner_inds,:]
                                    planner_time_data = time_data[problem_inds,:][thread_inds,:][varcurv_inds,:][phi_inds,:][kappa_inds,:][planner_inds,:]

                                    # get success percentage
                                    success_inds = np.where(planner_data[:,stats_indices['success']] == 1)[0]
                                    num_success = np.shape(success_inds)[0]
                                    num_pairs = np.shape(planner_data)[0]
                                    num_pairs = max_num_problems
                                    if num_pairs > 0:
                                        success_percentage = 100*(num_success/num_pairs)

                                        # get statistics involving successful planners
                                        if success_percentage > 0:
                                            ell_improvements = np.zeros(np.shape(success_inds))
                                            angle_improvements = np.zeros(np.shape(success_inds))
                                            first_sol_times = np.zeros(np.shape(success_inds))
                                            final_sol_times = np.zeros(np.shape(success_inds))
                                            num_sols = np.zeros(np.shape(success_inds))
                                            best_ell = np.zeros(np.shape(success_inds))
                                            worst_ell = np.zeros(np.shape(success_inds))
                                            first_phi = np.zeros(np.shape(success_inds))
                                            final_phi = np.zeros(np.shape(success_inds))
                                        else:
                                            ell_improvements = 0
                                            angle_improvements = 0
                                            first_sol_times = 0
                                            final_sol_times = 0
                                            num_sols = 0
                                            best_ell = 0
                                            worst_ell = 0
                                            first_phi = 0
                                            final_phi = 0

                                        for next_ind in range(num_success):
                                            success_ind = success_inds[next_ind]
                                            ells = planner_time_data[success_ind, stats_indices['lengths'] - stats_indices['times']] 
                                            ell_improvements[next_ind] = 100*(ells[0] - ells[-1])/ ells[-1]

                                            angles = planner_time_data[success_ind, stats_indices['phis'] - stats_indices['times']] 
                                            angle_improvements[next_ind] = 100*(angles[0] - angles[-1])/angles[-1]

                                            times = planner_time_data[success_ind, stats_indices['times'] - stats_indices['times']] 
                                            first_sol_times[next_ind] = times[0]
                                            final_sol_times[next_ind] = times[-1]

                                            num_sols[next_ind] = len(times)
                                            best_ell[next_ind] = ells[-1]
                                            worst_ell[next_ind] = ells[0]
                                            final_phi[next_ind] = angles[-1]
                                            first_phi[next_ind] = angles[0]

                                            # get ell percent improvement average / median 


                                            # get average / median number of solutions per pair / planner


                                            # get average / median first solution time



                                        # get statistics involving exhausted planners
                                        exhausted_inds = np.where(planner_data[:,stats_indices['time']] < max_time)[0]
                                        exhausted_percentage = 100*(np.shape(exhausted_inds)[0]/num_pairs)
                                        if exhausted_percentage > 0:
                                            exhausted_time = np.average(planner_data[exhausted_inds,stats_indices['time']])
                                            exhausted_time_std = np.std(planner_data[exhausted_inds,stats_indices['time']])
                                        else:
                                            exhausted_time = 0
                                            exhausted_time_std = 0

                                        exhausted_successful_inds = np.where(np.logical_and(planner_data[:,stats_indices['time']] < max_time, planner_data[:,stats_indices['success']] == 1))[0]
                                        exhausted_successful_percentage = 100*(np.shape(exhausted_successful_inds)[0]/num_pairs)

                                        exhausted_str = ""
                                        if exhausted_percentage > 0:
                                            exhausted_str = f"\n\texhausted: {exhausted_percentage:.01f} exhausted successful: {exhausted_successful_percentage:.01f} exhaustion time: {exhausted_time:.02f} +/- {exhausted_time_std:.04f}"

                                        # get percent exhausted planners


                                        # get average / median planner exhaustion time

                                        # https://discuss.python.org/t/general-way-to-print-floats-without-the-0-part/53728
                                        print(f"planner: {planner_ind} success: {success_percentage:.01f} \tell %: {np.average(ell_improvements):.04f} +/- {np.std(ell_improvements):.04f} best ell: {np.average(best_ell):.04f} worst ell: {np.average(worst_ell):.04f} \tphi %: {np.average(angle_improvements):.04f} +/- {np.std(angle_improvements):.04f} final phi: {np.average(final_phi):.04f} first phi: {np.average(first_phi):.04f} \t first time: {np.average(first_sol_times):.04f} +/- {np.std(first_sol_times):.04f} final time: {np.average(final_sol_times):.04f} +/- {np.std(final_sol_times):.04f} sols: {np.average(num_sols):.02f} +/- {np.std(num_sols):.02f} {exhausted_str}")
                                        rows += [[problem_start, problem_start + max_num_problems, multi_str, varcurv_str, f'{kappa}', f'{phi}', planners[int(planner_ind)-1].label, f'{success_percentage:.04f}', f'{np.average(ell_improvements):.04f}', f'{np.std(ell_improvements):.04f}', f'{np.average(best_ell):.04f}', f'{np.average(worst_ell):.04f}', f'{np.average(angle_improvements):.04f}', f'{np.std(angle_improvements):.04f}', f'{np.average(final_phi):.04f}', f'{np.average(first_phi):.04f}', f'{np.average(first_sol_times):.04f}', f'{np.std(first_sol_times):.04f}', f'{np.average(final_sol_times):.04f}', f'{np.std(final_sol_times):.04f}', f'{np.average(num_sols):.04f}', f'{np.std(num_sols):.04f}', f'{exhausted_percentage:.04f}', f'{exhausted_successful_percentage:.04f}', f'{exhausted_time:.04f}', f'{exhausted_time_std:.04f}']]
                                    else:
                                        print(f"planner: {planner_ind} no data")
                                print()

    fields = ["problem start", "problem end", "multi", "dynamic", "radius", "phi max", "planner", "success", "ell improvement", "ell improvement +/-", "best ell", "worst ell", "phi improvement", " phi improvement +/-", "final phi", "first phi", "first solution time", "first solution time +/-", "final solution time", "final solution time +/-", "number of solutions", "number of solutions +/-", "exhausted", "successful exhausted", "exhausted time", "exhausted time +/-"]
    # with open("./../data/output/statistics.csv", "w") as file:                  # https://www.geeksforgeeks.org/python/working-csv-files-python/
    #     csvwriter = csv.writer(file, quoting=csv.QUOTE_MINIMAL)
    #     csvwriter.writerow(fields)
    #     csvwriter.writerows(rows)


def get_all_data(directory, file_spec):
    files = fnmatch.filter(os.listdir(directory), file_spec)

    data = np.empty((0,15))
    time_data = np.empty((0,4))
    for file in files:

        next_data, next_time_data = get_data(directory + file)

        data = np.vstack((data, next_data))
        time_data = np.vstack((time_data, next_time_data))

        # make_success_brain_figure(data)
        # make_success_brain_figures(data)  
        
    return data, time_data


def get_data(file):

    data = np.empty((0,15))
    time_data = np.empty((0,4))

    def conv(x):
        x_ = x.decode()
        if len(x) > 2:
            values = np.array([float(xi) for xi in x_.strip("[,]").split(',')])
            return values
        else:
            return np.array([])
        
    convs = {0: lambda x: conv(x), 1: lambda x: conv(x), 2: lambda x: conv(x), 3: lambda x: conv(x)}

    next_data = np.loadtxt(file, delimiter=',', comments='#', usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14))
    next_time_data = np.loadtxt(file, delimiter=',', comments='#', usecols=(15,16,17,18), converters=conv, dtype=object, quotechar='"')

    data = np.vstack((data, next_data))
    time_data = np.vstack((time_data, next_time_data))

    return data, time_data    


if __name__=='__main__':

    # data, time_data = get_all_data('./../data/output/', '*_stats*.txt')
    data, time_data = get_all_data('./../data/output/', '*_debug_longer.txt')

    pairs_mins = get_min_ell(data, time_data)

    data, time_data = scale_distances(data, time_data, pairs_mins)

    get_statistics(data, time_data)

    # single_inds = np.where(np.logical_and(data[:,stats_indices['varcurv']] == 0,np.logical_and(data[:,stats_indices['multi']] == 1, data[:,stats_indices['maxphi']] == 180)))[0]
    # make_generic_test_figures(data[single_inds,:], time_data[single_inds,:], make_success_time_figure, r'Success vs. Time', r'Time (seconds)', r'Success Percentage', stats_indices['lengths'], 0.0001, 100, -1, 100, True, False)


    # find the mutli threaded 180 degree set of dynamic and non dynamic limits and plot them like the common planning problems
    mutli_inds = np.where(np.logical_and(data[:,stats_indices['varcurv']] == 1,np.logical_and(data[:,stats_indices['multi']] == 1, data[:,stats_indices['maxphi']] == 180)))[0]
    # mutli_inds = np.where(np.logical_and(data[:,stats_indices['varcurv']] == 1, data[:,stats_indices['maxphi']] == 180))[0]
    make_generic_test_figures(data[mutli_inds,:], time_data[mutli_inds,:], make_success_time_figure, r'Success vs. Time', r'Time (seconds)', r'Success Percentage', stats_indices['lengths'], 0.0001, 100, -1, 100, True, False)


    # mutli_inds = np.where(np.logical_and(data[:,stats_indices['varcurv']] == 1,np.logical_and(data[:,stats_indices['multi']] == 0, data[:,stats_indices['maxphi']] == 180)))[0]
    # # mutli_inds = np.where(np.logical_and(data[:,stats_indices['varcurv']] == 1, data[:,stats_indices['maxphi']] == 180))[0]
    # make_generic_test_figures(data[mutli_inds,:], time_data[mutli_inds,:], make_success_time_figure, r'Success vs. Time', r'Time (seconds)', r'Success Percentage', stats_indices['lengths'], 0.0001, 100, -1, 100, True, False)


    # files = fnmatch.filter(os.listdir('./../data/output/'), '*_stats*.txt')
    # for file in files:
    #     data, time_data = get_data('./../data/output/' + file)

    #     # make_success_brain_figure(data)
    #     # make_success_brain_figures(data)

    #     data, time_data = scale_distances(data, time_data, pairs_mins)

    #     # make_big_generic_figures(data, time_data, make_success_heat_figure, r'Success vs. Pairs', r'Goals', r'Starts', stats_indices['lengths'], 0, 0, 0, 0, False, False)
    #     # # make_generic_figures(data, time_data,  make_success_heat_figure, r'Success vs. Pairs', r'Goals', r'Starts', stats_indices['lengths'], 0, 0, 0, 0, False, False)
    #     # fig = plotter.figure(figsize=[16, 8])
    #     # make_success_heat_figure(data, time_data, stats_indices['lengths'], r'Success vs. Pair', r'Goals', r'Starts', 0, 0, 0, 0, False, False)
    #     # plotter.colorbar()
    #     # get_success_im_diff(data, time_data)

    #     make_generic_figures(data, time_data, make_success_time_figure, r'Success vs. Time', r'Time (seconds)', r'Success Percentage', stats_indices['lengths'], 0.0001, 100, -1, 100, True, False)

    #     # make_generic_figures(data, time_data, make_time_figure, r'Relative Path Length vs. Time', r'Time (seconds)', r'Relative Path Length', stats_indices['lengths'], 0.0001, 100, 0.999, 1.1, True, False)
    #     # make_generic_figures(data, time_data, make_time_figure, r'Angle vs. Time', r'Time (seconds)', r'$\phi_{\Sigma}$ (radians)', stats_indices['phis'], 0.0001, 100, 0, 3.14, True, False)

    #     # make_generic_figures(data, time_data, make_success_data_figure, r'Success vs. Relative Path Length', r'Relative Path Length', r'Success Percentage', stats_indices['lengths'], 0.995, 1.175, -1, 100, False, False)
    #     # make_generic_figures(data, time_data, make_success_data_figure, r'Success vs. Cumulative Orientation Angle Change', r'$\phi_{\Sigma}$ (radians)', r'Success Percentage', stats_indices['phis'], 0, 3.14, -1, 100, False, False)


    #     # make_violin_figures(data, time_data, stats_indices['ell'], 0.995, 1.175, r'Relative Path Lengths for Planners', r'Relative Path Length')
    #     # make_violin_figures(data, time_data, stats_indices['phi'], 0, 3.2, r'Cumulative Orientation Angle Changes for Planners', r'$\phi$ (radians)')

    # # plot_magnet_options("./../../data/PiGroup/curvature_pi_group_data.mat")

    # # plotter.show()

    # # get_exhausted_planners(data, time_data)







    # data, time_data = get_all_data('./../data/output/', '*_test.txt')

    # # pairs_mins = get_min_ell(data, time_data)

    # data, time_data = scale_distances(data, time_data, pairs_mins)

    # get_pairwise_statistics(data, time_data)

    # # get_statistics(data, time_data)

    # # make_big_generic_figures(data, time_data, make_success_heat_figure, r'Success vs. Pairs', r'Goals', r'Starts', stats_indices['lengths'], 0, 0, 0, 0, False, False)
    # # # make_generic_figures(data, time_data,  make_success_heat_figure, r'Success vs. Pairs', r'Goals', r'Starts', stats_indices['lengths'], 0, 0, 0, 0, False, False)
    # # fig = plotter.figure(figsize=[16, 8])
    # # make_success_heat_figure(data, time_data, stats_indices['lengths'], r'Success vs. Pair', r'Goals', r'Starts', 0, 0, 0, 0, False, False)
    # # plotter.colorbar()
    # # get_success_im_diff(data, time_data)

    # make_generic_test_figures(data, time_data, make_success_time_figure, r'Success vs. Time', r'Time (seconds)', r'Success Percentage', stats_indices['lengths'], 0.0001, 100, -1, 100, True, False)

    # # make_generic_figures(data, time_data, make_time_figure, r'Relative Path Length vs. Time', r'Time (seconds)', r'Relative Path Length', stats_indices['lengths'], 0.0001, 100, 0.999, 1.1, True, False)
    # # make_generic_figures(data, time_data, make_time_figure, r'Angle vs. Time', r'Time (seconds)', r'$\phi$ (radians)', stats_indices['phis'], 0.0001, 100, 0, 3.14, True, False)

    # # make_generic_figures(data, time_data, make_success_data_figure, r'Success vs. Relative Path Length', r'Relative Path Length', r'Success Percentage', stats_indices['lengths'], 0.995, 1.175, -1, 100, False, False)
    # # make_generic_figures(data, time_data, make_success_data_figure, r'Success vs. Cumulative Orientation Angle Change', r'$\phi$ (radians)', r'Success Percentage', stats_indices['phis'], 0, 3.14, -1, 100, False, False)

    # # make_violin_figures(data, time_data, stats_indices['ell'], 0.995, 1.175, r'Relative Path Lengths for Planners', r'Relative Path Length')
    # # make_violin_figures(data, time_data, stats_indices['phi'], 0, 3.2, r'Cumulative Orientation Angle Changes for Planners', r'$\phi$ (radians)')

    plotter.show()

      