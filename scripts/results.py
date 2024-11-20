import numpy as np
import os, fnmatch
import matplotlib.pyplot as plotter

_RGRRT_NOPHI = '#332288'
_RGCON_NOPHI = '#117733'
_AORRT_NOPHI = '#44AA99'
_AOCON_NOPHI = '#40B3EC'

_RGRRT_PHI = '#F3B228'
_RGCON_PHI = '#D46D7E'
_AORRT_PHI = '#C33AAC'
_AOCON_PHI = '#BF0F67'

_COLORS = [_RGRRT_NOPHI, _RGCON_NOPHI, _AORRT_NOPHI, _AOCON_NOPHI, _RGRRT_PHI, _RGCON_PHI, _AORRT_PHI]
_LABELS = [r'RGRRT', r'AORRT', r'RCS', r'RCS*', r'RGRRT$_s$', r'AORRT$_s$', r'RCS$_s$']

_RUNTIME = 3
_TOTALPHI = 2
_L = 1
_KAPPA = 0

_SUCCESS = 4
_APPROX_SUCCESS = 5
_SPREADING = 6
_PLANNER = 7


_ALPHA = 0.25
_ROTATION = 10
_WIDTH = 0.2
_TEXTSIZE = 10

_RRT = 1
_AORRT = 2
_RCS = 3
_RCS_STAR = 4

_RRT_SPREADING = 5
_AORRT_SPREADING = 6
_RCS_SPREADING = 7

def color_boxplot(bp, color, marker='o'):
    '''
    Styles matplotlib boxplot
    Parameters:
    bp (box plot): matplotlib boxplot to style
    color (string): color to apply to the boxplot
    marker (string): marker to use for outliers
    '''
    for whisker in bp['whiskers']: whisker.set(color=color, linewidth=1)

    for cap in bp['caps']: cap.set(color=color, linewidth=1)

    for median in bp['medians']: 
        median.set(color=color, linewidth=1)
        (xl, y), (xr, _) = median.get_xydata()
        plotter.text(xl-0.05, y, '%.2f' % y, verticalalignment='center', horizontalalignment='right', fontsize=_TEXTSIZE)

    for mean in bp['means']: 
        mean.set(markerfacecolor=color, markeredgecolor='#000000', alpha=_ALPHA, markersize=5)

    for flier in bp['fliers']: 
        flier.set(marker=marker, markeredgecolor=color, alpha=_ALPHA)

    for box in bp['boxes']: box.set(color=color, linewidth=1)

def color_violinplot(vp, color, hatching='/'):
    '''
    Styles the violin plot.
    Parameters:
    vp (violin plot): matplotlib violin plot
    color (string): color to use for the violin plot
    hatching (string): hatch pattern that will be added to the background of the violinplot, may not show if saved as PDF
    '''
    for body in vp['bodies']: body.set(color=color, hatch=hatching, alpha=_ALPHA)

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
    '''
    Gets the indices for the 8 different planner variations from the provided data.
    Parameters:
    data (n,11): numpy array of the data saved in the experiments

    Returns:
    8 arrays containing the row indices for data gathered for the 8 planner variations
    rgrrt_nophi, rgcon_nophi, aorrt_nophi, aocon_nophi, rgrrt_phi, rgcon_phi, aorrt_phi, aocon_phi
    '''
    rrt = np.where(data[:,_PLANNER] == _RRT)[0]
    rcs = np.where(data[:,_PLANNER] == _RCS)[0]
    aorrt = np.where(data[:,_PLANNER] == _AORRT)[0]
    rcs_star = np.where(data[:,_PLANNER] == _RCS_STAR)[0]

    rrt_spread = np.where(data[:,_PLANNER] == _RRT_SPREADING)[0]
    rcs_spread = np.where(data[:,_PLANNER] == _RCS_SPREADING)[0]
    aorrt_spread = np.where(data[:,_PLANNER] == _AORRT_SPREADING)[0]

    return rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread


def make_misc_figure(data, index, title, ylabel, ylog=True):
    '''
    Makes an augmented boxplot figure using the provided data
    Parameters:
    data (n,11): data from the experiments to analyze
    index (int): index for the column of the data to be analyzed
    title (string): title for the resulting plot
    ylabel (string): label for the y axis of the plot
    y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    '''
    rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread = get_indices(data)
    indices = [rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread]
    if index == _L:
        ldata = get_distances(data)
    plotter.title(title)
    for i in range(len(indices)):
        # if we're analyzing the path length, use the scaling to have a more informative plot
        if index == _L:
            data_ind = ldata[indices[i]]
        else:
            data_ind = data[indices[i], index]
        if np.shape(data_ind)[0] == 0:
            continue

        color = _COLORS[i]
        median = np.median(data_ind)
        mean = np.mean(data_ind)
        std = np.std(data_ind)

        _bp = plotter.boxplot(data_ind, positions=[i], widths=_WIDTH, whis=[0, 100], notch=True, bootstrap=5000)
        color_boxplot(_bp, color)
        plotter.hlines(mean, i-_WIDTH/2, i+_WIDTH/2, color=color, linestyles='dotted')

        if mean < std:
            plotter.bar(i, std, bottom=mean, color=color, alpha=_ALPHA, width=_WIDTH)
            plotter.bar(i, mean, bottom=0.0001, color=color, alpha=_ALPHA, width=_WIDTH)
        else:
            plotter.bar(i, std, bottom=mean, color=color, alpha=_ALPHA, width=_WIDTH)
            plotter.bar(i, std, bottom=mean-std, color=color, alpha=_ALPHA, width=_WIDTH)
        
    plotter.ylabel(ylabel)
    plotter.xticks(np.arange(len(_COLORS)),_LABELS, rotation=_ROTATION)
    plotter.xlim([-1, len(_COLORS)-0.5])

    if ylog:
        plotter.yscale('log')



def make_violin_figure(data, index, title, ylabel, hatching, y_min=0, y_max=10, ylog=False):
    '''
    Makes violin plots for the planner variations.
    Parameters:
    data (n,11): data from the experiments to analyze
    index (int): index for the column of the data to be analyzed
    title (string): title for the resulting plot
    ylabel (string): label for the y axis of the plot
    hatching (string): the hatching pattern for the violin plot (Note that this may not show up when saving as a PDF)
    y_min (float): minimum y axis value, default=0
    y_max (float): maximum y axis value, default=100
    y_log (bool): if true makes the y axis scaled log, can throw off y axis limits
    '''
    rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread = get_indices(data)
    indices = [rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread]
    if index == _L:
        ldata = get_distances(data)
    plotter.title(title)
    for i in range(len(indices)):
        if index == _L:
            data_ind = ldata[indices[i]]
        else:
            data_ind = data[indices[i], index]
        if np.shape(data_ind)[0] == 0:
            continue
        color = _COLORS[i]
        median = np.median(data_ind)

        _bp = plotter.violinplot(data_ind, positions=[i], widths=_WIDTH, showmedians=True)
        color_violinplot(_bp, color, hatching=hatching)
        # plotter.hlines(median, i-_WIDTH, i+_WIDTH, color=color, linestyles='dashed')

        plotter.text(i-_WIDTH/2, median,'%.3f' % median, horizontalalignment='right', verticalalignment='center', fontsize=_TEXTSIZE)

    plotter.ylabel(ylabel)
    plotter.xticks(np.arange(0,len(_COLORS)),_LABELS, rotation=_ROTATION)
    plotter.xlim([-1, len(_COLORS)-0.5])

    if ylog:
        plotter.yscale('log')
    else:
        plotter.yscale('linear')
        

    plotter.ylim([y_min, y_max])
    fix_ylabels()



def make_success_bar(data, hatch):
    '''
    Makes a success rate plot for the planner variations with a 95% binomial confidence interval.
    Parameters:
    data (n,11): data from the experiments to analyze
    hatching (string): the hatching pattern for the bars (Note that this may not show up when saving as a PDF)    
    '''
    rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread = get_indices(data)
    indices = [rrt, aorrt, rcs, rcs_star, rrt_spread, aorrt_spread, rcs_spread]

    plotter.title(r'Success Rates with 95% Confidence Interval')

    successes = []
    for i in range(len(_COLORS)):
        data_ind = data[indices[i]]
        color = _COLORS[i]
        if np.shape(data_ind)[0] == 0:
            continue
        success, moe = get_success(data_ind)
        successes += [success]
        plotter.hlines(success, i-_WIDTH/2, i+_WIDTH/2, color=color, linestyles='dotted')
        plotter.bar(i, moe*2, bottom=success - moe, color=color, alpha=_ALPHA, width=_WIDTH, hatch=hatch, edgecolor=color)
        plotter.text(i-_WIDTH/1.5, success, '%.2f' % success + '%', horizontalalignment='right', verticalalignment='center', fontsize=_TEXTSIZE)

    plotter.ylabel('Success Percentage')
    plotter.xticks(np.arange(0,len(_COLORS)),_LABELS, rotation=_ROTATION)
    plotter.xlim([-1,len(_COLORS)-0.5])
    plotter.ylim(top=105, bottom=0) #max(10, min(60, min(successes))-10))


def get_success(data):
    '''
    Calculates the success rates for the data and a 95% confidence interval.
    Parameters:
    data (n,11): data from the experiments to analyze, should probably be for a single planner variation
    
    Returns:
    success (float): the success rate of the planner (max 100)
    margin_of_error (float): the size of the confidence interval above and below success
    '''
    plan_found = np.where(np.logical_or(data[:,_SUCCESS] == True, data[:,_APPROX_SUCCESS] == True))[0]
    success = np.shape(plan_found)[0]/np.shape(data)[0]
    adjusted_proportion = (np.shape(plan_found)[0] + 2)/(np.shape(data)[0] + 4)
    se_1 = adjusted_proportion*(1-adjusted_proportion)
    se_2 = se_1/(np.shape(data)[0] + 4)
    standard_error = np.sqrt(se_2)
    margin_of_error = standard_error*2

    return success*100, margin_of_error*100


def fix_ylabels():
    '''
    fix the y labels if they end up in scientific notation or if minor ticks are there
    '''
    locs, labels = plotter.yticks(minor=True)
    new_labels = []
    for loc in locs:
        new_label = str(round(loc, 4))
        new_labels += [new_label]    
    plotter.yticks(locs, new_labels)


def make_figures(data, title, hatch):
    '''
    Makes a figure with 4 subplots anaylzing different aspects of the planner variations.
    Parameters:
    data (n,11): data from the experiments to analyze
    title (string): title for the whole figure
    hatch (string): hatching pattern that will be used in the plots, may not show up if saving as PDF
    '''
    plan_data_ind = np.where(data[:,_SUCCESS] == 1)[0]
    plan_data = data[plan_data_ind,:]
    num_plots = 4
    rows = 2
    fig = plotter.figure(figsize=[15, 8])

    # make violin subplot of runtimes with log scale
    plotter.subplot(rows,num_plots//rows,1)
    make_violin_figure(data, _RUNTIME, 'Run Time for Planner Variations', 'run time (seconds)', hatch, y_min=0, y_max=6)

    # make success bar subplot with 95% confidence interval
    plotter.subplot(rows,num_plots//rows,2)
    make_success_bar(data, hatch)

    # make violin subplot of total phis for planners with log scale
    plotter.subplot(rows,num_plots//rows,3)
    make_violin_figure(plan_data, _TOTALPHI, r'Total $\phi$ for Planner Variations', r'$\phi$ (radians)', hatch, y_min=0.0, y_max=5)

    # make a violin subplot of the path length ratios for planners, no log scale
    plotter.subplot(rows,num_plots//rows,4)
    make_violin_figure(plan_data, _L, r'$\ell^\prime$ ratio for Planner Variations', r'$\ell^\prime$', hatch, y_min=0, y_max=300, ylog=False)

    # title the whole figure and adjust the spacing of the plots and margins 
    plotter.suptitle(title, fontsize=18)
    plotter.subplots_adjust(top=0.9, bottom=0.075, right=0.98, left=0.065, hspace=0.25, wspace=0.15)


def get_distances(data):
    '''
    Calculates the path length ratio, a better metric than just the total path length since we're considering multiple sets of starts & goals.
    Parameters:
    data (n,11): data from the experiments to analyze

    Returns:
    lproportion (n,): array of the path length ratios calculated from data
    '''
    return data[:, _L]
    # starts = _BRSTARTS
    # goals = _BRGOALS

    # lproportion = np.empty(np.shape(data[:,0]))

    # for i in range(np.shape(_BRSTARTS)[0]):
    #     distance = np.linalg.norm(starts[i] - goals[i])
    #     indices = np.where(data[:,0] == i)[0]
    #     if np.shape(indices)[0] == 0:
    #         continue
    #     lproportion[indices] = data[indices,_L]/distance

    # return lproportion




if __name__=='__main__':


    files = fnmatch.filter(os.listdir('./../data/output/'), '*stats.txt')
    data = np.empty((0,8))
    for file in files:
        next_data = np.loadtxt('./../data/output/' + file, delimiter=',', comments='#')
        data = np.vstack((data, next_data))

    kappas = np.unique(data[:,_KAPPA])
    hatches = ['O', '///', '\\\\\\',  'xxx', '.', '*', 'o']

    # make figures for each of the kappa values used in experiments
    for i in range(np.shape(kappas)[0]):
        kappa = kappas[i]
        kappa_inds = np.where(data[:,_KAPPA] == kappa)[0]
        kappa_data = data[kappa_inds,:]
        make_figures(kappa_data, r'$\kappa$ = %.4f $mm^{-1}$' % kappa, hatches[i])
        # plotter.savefig('./figures/K0%.4f.pdf'% kappa )
        # analyze_pairs(data)
    plotter.show()