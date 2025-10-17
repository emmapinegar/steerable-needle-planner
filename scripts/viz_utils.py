import numpy as np
import os, fnmatch
import matplotlib.pyplot as plotter
from dataclasses import dataclass

_PURPLE = '#332288'
_GREEN = '#117733'
_TEAL = '#44AA99'
_BLUE = '#40B3EC'

_ORANGE = '#EF6E12'
_BLUSH = '#D46D7E'
_PINK = '#C33AAC'
_RED = '#BF0F67'


stats_indices = {'env': 0, 'sg_index': 1, 'sg_mag': 2, 'planner': 3, 'ell': 4, 'phi': 5, 'time': 6, 'success': 7, 'approx_success': 8, 'spreading': 9, 'maxphi': 10, 'maxell': 11, 'minrad': 12, 'varcurv':13, 'times': 14, 'costs': 15, 'lengths': 16, 'phis': 17}
viz_params = {'alpha': 0.25, 'rotation': 10, 'width': 0.2, 'textsize': 10}


@dataclass
class Planner:
    index: int
    color: str
    label: str
    linestyle: str

rrt_info = Planner(1, _PINK, r'RGRRT', '-')
aorrt_info = Planner(2, _ORANGE, r'AORRT', '-.')
rcs_info = Planner(3, _BLUE, r'RCS', '--')
rcsstar_info = Planner(4, _PURPLE, r'RCS*', ':')

rrt_spreading_info = Planner(5, _RED, r'RGRRT_s', '-')
aorrt_spreading_info = Planner(6, _RED, r'AORRT_s', '-')
rcs_spreading_info = Planner(7, _RED, r'RCS_s', '-')

planners = [rrt_info, aorrt_info, rcs_info, rcsstar_info] #, rrt_spreading_info, aorrt_spreading_info, rcs_spreading_info]



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

        _bp = plotter.violinplot(data_ind, positions=[len(colors)-1+2*viz_params['width']/3], widths=viz_params['width']*np.shape(data_ind)[0]/350, showmedians=True)
        color_violinplot(_bp, planners[i].color)

        # plotter.hlines(median, i-viz_params['width'], i+viz_params['width'], color=planners[i].color, linestyles='dashed')

        plotter.text(len(colors)-1, median,'%.3f' % median, horizontalalignment='right', verticalalignment='center', fontsize=viz_params['textsize'])

    plotter.ylabel(ylabel)
    plotter.xticks(np.arange(0,len(colors))+2*viz_params['width']/3, labels, rotation=viz_params['rotation'])
    plotter.xlim([-0.5, len(colors)-0.5])

    if ylog:
        plotter.yscale('log')
    else:
        plotter.yscale('linear')
        

    plotter.ylim([y_min, y_max])


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
        plotter.hlines(success, len(colors)-1-viz_params['width']/2, len(colors)-1+viz_params['width']/2, color=planners[i].color, linestyles='dotted')
        plotter.bar(len(colors)-1, moe*2, bottom=success - moe, color=planners[i].color, alpha=viz_params['alpha'], width=viz_params['width'], edgecolor=planners[i].color)
        plotter.text(len(colors)-1-viz_params['width']/1.5, success, '%.2f' % success + '%', horizontalalignment='right', verticalalignment='center', fontsize=viz_params['textsize'])

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