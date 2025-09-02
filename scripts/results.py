import numpy as np
import os, fnmatch
import matplotlib.pyplot as plotter
from dataclasses import dataclass

_PURPLE = '#332288'
_GREEN = '#117733'
_TEAL = '#44AA99'
_BLUE = '#40B3EC'

_ORANGE = '#F3B228'
_BLUSH = '#D46D7E'
_PINK = '#C33AAC'
_RED = '#BF0F67'


stats_indices = {'env': 0, 'sg_index': 1, 'sg_mag': 2, 'planner': 3, 'ell': 4, 'phi': 5, 'time': 6, 'success': 7, 'approx_success': 8, 'spreading': 9, 'maxphi': 10, 'maxell': 11, 'minrad': 12, 'times': 13, 'costs': 14, 'lengths': 15, 'phis': 16}
viz_params = {'alpha': 0.25, 'rotation': 10, 'width': 0.2, 'textsize': 10}


@dataclass
class Planner:
    index: int
    color: str
    label: str

rrt_info = Planner(1, _PINK, r'RGRRT')
aorrt_info = Planner(2, _ORANGE, r'AORRT')
rcs_info = Planner(3, _BLUE, r'RCS')
rcsstar_info = Planner(4, _PURPLE, r'RCS*')

rrt_spreading_info = Planner(5, _RED, r'RGRRT_s')
aorrt_spreading_info = Planner(6, _RED, r'AORRT_s')
rcs_spreading_info = Planner(7, _RED, r'RCS_s')

planners = [rrt_info, aorrt_info, rcs_info, rcsstar_info, rrt_spreading_info, aorrt_spreading_info, rcs_spreading_info]


def color_boxplot(bp, color, marker='o'):
    """
    Styles matplotlib boxplot.

    Parameters:
        bp (box plot): matplotlib boxplot to style
        color (string): color to apply to the boxplot
        marker (string): marker to use for outliers
    """
    for whisker in bp['whiskers']: whisker.set(color=color, linewidth=1)

    for cap in bp['caps']: cap.set(color=color, linewidth=1)

    for median in bp['medians']: 
        median.set(color=color, linewidth=1)
        (xl, y), (xr, _) = median.get_xydata()
        plotter.text(xl-0.05, y, '%.2f' % y, verticalalignment='center', horizontalalignment='right', fontsize=viz_params['textsize'])

    for mean in bp['means']: 
        mean.set(markerfacecolor=color, markeredgecolor='#000000', alpha=viz_params['alpha'], markersize=5)

    for flier in bp['fliers']: 
        flier.set(marker=marker, markeredgecolor=color, alpha=viz_params['alpha'])

    for box in bp['boxes']: box.set(color=color, linewidth=1)


def color_violinplot(vp, color):
    """
    Styles the violin plot.

    Parameters:
        vp (violin plot): matplotlib violin plot
        color (string): color to use for the violin plot
    """
    for body in vp['bodies']: body.set(color=color, alpha=viz_params['alpha'])

    # vp['cmeans'].set(color=color, linestyle='dotted')

    vp['cmedians'].set(color=color)

    vp['cmins'].set(color=color)

    vp['cmaxes'].set(color=color)

    vp['cbars'].set(color=color)

    # vp['cquantiles'].set(color=color)


# def analyze_pairs(data):
#     '''
#     Analyze the data by separating out the sets of starts & goals
#     '''
#     for i in range(np.shape(_BRSTARTS)[0]):
#         pair_inds = np.where(data[:,0] == i)[0]
#         make_figures(data[pair_inds])
#     plotter.show()


def get_indices(data):
    """
    Gets the indices for the 8 different planner variations from the provided data.

    Parameters:
        data (n,13): numpy array of the data saved in the experiments

    Returns:
        rgrrt, aorrt, rcs, rcsstar, rrt_spreading, aorrt_spreading, rcs_spreading (NDArray): 8 arrays containing the row indices for data gathered for the 8 planner variations
    """
    rrt = np.where(data[:,stats_indices['planner']] == rrt_info.index)[0]
    rcs = np.where(data[:,stats_indices['planner']] == rcs_info.index)[0]
    aorrt = np.where(data[:,stats_indices['planner']] == aorrt_info.index)[0]
    rcs_star = np.where(data[:,stats_indices['planner']] == rcsstar_info.index)[0]

    rrt_spread = np.where(data[:,stats_indices['planner']] == rrt_spreading_info.index)[0]
    rcs_spread = np.where(data[:,stats_indices['planner']] == rcs_spreading_info.index)[0]
    aorrt_spread = np.where(data[:,stats_indices['planner']] == aorrt_spreading_info.index)[0]

    return rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread


def get_planner_indices(data, planner:Planner):
    """
    Gets the indices for a planner in the data array.
    
    Parameters:
        data (n,13): data from the planner experiment results
        planner (Planner): planner struct to isolate

    Returns:
        indices (NDArray): indices for the rows of data belonging to the provided planner
    """
    indices = np.where(data[:,stats_indices['planner']] == planner.index)[0]
    return indices

def make_misc_figure(data, index, title, ylabel, ylog=True):
    """    
    Makes an augmented boxplot figure using the provided data.

    Parameters:
        data (n,13): data from the experiments to analyze
        index (int): index for the column of the data to be analyzed
        title (string): title for the resulting plot
        ylabel (string): label for the y axis of the plot
        y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    """
    colors = []
    labels = []

    plotter.title(title)
    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
        # if we're analyzing the path length, use the scaling to have a more informative plot
        if index == stats_indices['ell']:
            data_ind = data[planner_indices, index]/data[planner_indices, stats_indices['sg_mag']]
        else:
            data_ind = data[planner_indices, index]
        if np.shape(data_ind)[0] == 0:
            continue

        colors += [planners[i].color]
        labels += [planners[i].label]
        median = np.median(data_ind)
        mean = np.mean(data_ind)
        std = np.std(data_ind)

        _bp = plotter.boxplot(data_ind, positions=[i], widths=viz_params['width'], whis=[0, 100], notch=True, bootstrap=5000)
        color_boxplot(_bp, planners[i].color)
        plotter.hlines(mean, i-viz_params['width']/2, i+viz_params['width']/2, color=planners[i].color, linestyles='dotted')

        if mean < std:
            plotter.bar(i, std, bottom=mean, color=planners[i].color, alpha=viz_params['alpha'], width=viz_params['width'])
            plotter.bar(i, mean, bottom=0.0001, color=planners[i].color, alpha=viz_params['alpha'], width=viz_params['width'])
        else:
            plotter.bar(i, std, bottom=mean, color=planners[i].color, alpha=viz_params['alpha'], width=viz_params['width'])
            plotter.bar(i, std, bottom=mean-std, color=planners[i].color, alpha=viz_params['alpha'], width=viz_params['width'])
        
    plotter.ylabel(ylabel)
    plotter.xticks(np.arange(len(colors)), labels, rotation=viz_params['rotation'])
    plotter.xlim([-1, len(colors)-0.5])

    if ylog:
        plotter.yscale('log')



def make_violin_figure(data, index, title, ylabel, y_min=0, y_max=10, ylog=False):
    """
    Makes violin plots for the planner variations.

    Parameters:
        data (n,13): data from the experiments to analyze
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
    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
        if index == stats_indices['ell']:
            data_ind = data[planner_indices, index]/data[planner_indices, stats_indices['sg_mag']]
        else:
            data_ind = data[planner_indices, index]
        if np.shape(data_ind)[0] == 0:
            continue

        colors += [planners[i].color]
        labels += [planners[i].label]

        median = np.median(data_ind)

        _bp = plotter.violinplot(data_ind, positions=[len(colors)-1], widths=viz_params['width'], showmedians=True)
        color_violinplot(_bp, planners[i].color)

        # plotter.hlines(median, i-viz_params['width'], i+viz_params['width'], color=planners[i].color, linestyles='dashed')

        plotter.text(len(colors)-1-viz_params['width']/2, median,'%.3f' % median, horizontalalignment='right', verticalalignment='center', fontsize=viz_params['textsize'])

    plotter.ylabel(ylabel)
    plotter.xticks(np.arange(0,len(colors)), labels, rotation=viz_params['rotation'])
    plotter.xlim([-0.5, len(colors)-0.5])

    if ylog:
        plotter.yscale('log')
    else:
        plotter.yscale('linear')
        

    plotter.ylim([y_min, y_max])



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

        n = 7 #window
        average = np.cumsum(flat)
        average[n:] = average[n:] - average[:-n]
        average[n-1:] = average[n-1:]/n

        averagetime = np.cumsum(time)
        averagetime[n:] = averagetime[n:] - averagetime[:-n]
        averagetime[n-1:] = averagetime[n-1:]/n

        if np.shape(average)[0] > 0:
            for i in range(0, n-1):
                average[i] = average[i]/(i+1)
                averagetime[i] = averagetime[i]/(i+1)


        line = plotter.plot(averagetime, average, color=planners[i].color)
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
    plotter.xlim([0, x_max])
    plotter.ylim([y_min,y_max])

    if ylog:
        plotter.yscale('log')
    else:
        plotter.yscale('linear')
        

def make_success_bar(data):
    """    
    Makes a success rate plot for the planner variations with a 95% binomial confidence interval.

    Parameters:
        data (n,13): data from the experiments to analyze  
    """
    plotter.title(r'Success Rates with 95% Confidence Interval')

    successes = []
    colors = []
    labels = []
    for i in range(len(planners)):
        planner_indices = get_planner_indices(data, planners[i])
        data_ind = data[planner_indices]

        if np.shape(data_ind)[0] == 0:
            continue
        colors += [planners[i].color]
        labels += [planners[i].label]
        success, moe = get_success(data_ind)
        successes += [success]
        plotter.hlines(success, i-viz_params['width']/2, i+viz_params['width']/2, color=planners[i].color, linestyles='dotted')
        plotter.bar(i, moe*2, bottom=success - moe, color=planners[i].color, alpha=viz_params['alpha'], width=viz_params['width'], edgecolor=planners[i].color)
        plotter.text(i-viz_params['width']/1.5, success, '%.2f' % success + '%', horizontalalignment='right', verticalalignment='center', fontsize=viz_params['textsize'])

    plotter.ylabel('Success Percentage')
    plotter.xticks(np.arange(0, len(colors)), labels, rotation=viz_params['rotation'])
    plotter.xlim([-0.5,len(colors)-0.5])
    plotter.ylim(top=105, bottom=0)


def get_success(data):
    """    
    Calculates the success rates for the data and a 95% confidence interval.

    Parameters:
        data (n,13): data from the experiments to analyze, should probably be for a single planner variation
    
    Returns:
        success (float): the success rate of the planner (max 100)
        margin_of_error (float): the size of the confidence interval above and below success
    """
    plan_found = np.where(np.logical_or(data[:,stats_indices['success']] == True, data[:,stats_indices['approx_success']] == True))[0]
    success = np.shape(plan_found)[0]/np.shape(data)[0]
    adjusted_proportion = (np.shape(plan_found)[0] + 2)/(np.shape(data)[0] + 4)
    se_1 = adjusted_proportion*(1-adjusted_proportion)
    se_2 = se_1/(np.shape(data)[0] + 4)
    standard_error = np.sqrt(se_2)
    margin_of_error = standard_error*2

    return success*100, margin_of_error*100


def fix_ylabels():
    """
    fix the y labels if they end up in scientific notation or if minor ticks are there
    """
    locs, labels = plotter.yticks(minor=True)
    new_labels = []
    for loc in locs:
        new_label = str(round(loc, 4))
        new_labels += [new_label]    
    plotter.yticks(locs, new_labels)


def make_figures(data, time_data, title):
    """    
    Makes a figure with 4 subplots anaylzing different aspects of the planner variations.
    
    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array
        title (string): title for the whole figure
    """
    plan_data_ind = np.where(data[:,stats_indices['success']] == 1)[0]
    plan_data = data[plan_data_ind,:]
    num_plots = 4
    rows = 2
    fig = plotter.figure(figsize=[15, 8])

    # make violin subplot of runtimes with log scale
    plotter.subplot(rows,num_plots//rows,1)
    make_success_bar(data)
    # make_violin_figure(data, _RUNTIME, 'Run Time for Planner Variations', 'run time (seconds)', y_min=0, y_max=7.5)

    # make success bar subplot with 95% confidence interval
    plotter.subplot(rows,num_plots//rows,2)
    # make_success_bar(data, hatch)
    make_time_figure(data, time_data, stats_indices['lengths'], r'Distance vs time', r'$\ell$', y_min=1, y_max=1.25)

    # make violin subplot of total phis for planners with log scale
    plotter.subplot(rows,num_plots//rows,3)
    make_violin_figure(plan_data, stats_indices['phi'], r'Total $\phi$ for Planner Variations', r'$\phi$ (radians)', y_min=0.0, y_max=6)

    # make a violin subplot of the path length ratios for planners, no log scale
    plotter.subplot(rows,num_plots//rows,4)
    make_violin_figure(plan_data, stats_indices['ell'], r'$\ell^\prime$ ratio for Planner Variations', r'$\ell^\prime$', y_min=1, y_max=1.25, ylog=False)

    # title the whole figure and adjust the spacing of the plots and margins 
    plotter.suptitle(title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)


def get_distances(data):
    """
    Calculates the path length ratio, a better metric than just the total path length since we're considering multiple sets of starts & goals.

    Parameters:
        data (n,13): data from the experiments to analyze

    Returns:
        lproportion (n,): array of the path length ratios calculated from data
    """
    lproportion = data[:,stats_indices['ell']]/data[:,stats_indices['sg_mag']]    

    return lproportion


def get_distances_time(data, time_data):
    """    
    Calculates the path length ratio, a better metric than just the total path length since we're considering multiple sets of starts & goals.

    Parameters:
        data (n,13): data from the experiments to analyze
        time_data (n,4): timewise data from the experiments to analyze, with arrays for each element in the array

    Returns:
        lproportion (n,): array of the path length ratios calculated from data
    """
    time_data[:,stats_indices['lengths']-stats_indices['times']] = time_data[:,stats_indices['lengths']-stats_indices['times']]/data[:,stats_indices['sg_mag']]

    return time_data



if __name__=='__main__':

    files = fnmatch.filter(os.listdir('./../data/output/'), '*_stats.txt')
    data = np.empty((0,13))
    time_data = []
    def conv(x):
        x_ = x.decode()
        if len(x) > 2:
            values = np.array([float(xi) for xi in x_.strip("[,]").split(',')])
            # print(values)
            return values
        else:
            return np.array([])
        
    convs = {0: lambda x: conv(x), 1: lambda x: conv(x), 2: lambda x: conv(x), 3: lambda x: conv(x)}
    for file in files:
        next_data = np.loadtxt('./../data/output/' + file, delimiter=',', comments='#', usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12))

        data = np.vstack((data, next_data))
        next_time_data = np.loadtxt('./../data/output/' + file, delimiter=',', comments='#', usecols=(13,14,15,16), converters=conv, dtype=object, quotechar='"')

        # print(np.shape(next_time_data))
        time_data.append(next_time_data)

    kappas = np.unique(data[:,stats_indices['minrad']])
    hatches = ['O', '///', '\\\\\\',  'xxx', '.', '*', 'o']
    envs = np.unique(data[:, stats_indices['env']])
    phis = np.unique(data[:,stats_indices['maxphi']])
    # make figures for each of the kappa values used in experiments
    for i in range(np.shape(kappas)[0]):
        kappa = kappas[i]
        kappa_inds = np.where(data[:,stats_indices['minrad']] == kappa)[0]
        kappa_data = data[kappa_inds,:]
        for j in range(np.shape(envs)[0]):
            env = envs[j]
            env_inds = np.where(kappa_data[:,stats_indices['env']] == env)[0]
            env_data = kappa_data[env_inds,:]
            for k in range(np.shape(phis)[0]):
                phi = phis[k]
                phi_inds = np.where(env_data[:,stats_indices['maxphi']] == phi)[0]
                phi_data = env_data[phi_inds,:]
                if np.shape(phi_data)[0] > 0:
                    # print(np.shape(phi_data))
                    make_figures(phi_data, time_data[0][kappa_inds,:][env_inds,:][phi_inds,:], r'$\kappa$ = %.4f $mm^{-1}$' % kappa + r' $\phi = %d$' % phi + r' env = $ %d$' %env)
        # plotter.savefig('./figures/K0%.4f.pdf'% kappa )
        # analyze_pairs(data)
    plotter.show()