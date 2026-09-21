import numpy as np
import open3d as o3d
import os, fnmatch
import matplotlib.pyplot as plotter
from dataclasses import dataclass
import copy
import csv
from scipy.cluster.vq import kmeans
from scipy.stats import rdist
from sklearn.cluster import KMeans, BisectingKMeans
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
        
def scale_distances(data, action_data, pairs_mins):
    pairs = np.unique(data[:,stats_indices['sg_index']])

    for pair in pairs:
        pair = int(pair)
        pair_inds = np.where(data[:,stats_indices['sg_index']] == pair)[0]
        pair_min = pairs_mins[pair]

        pair_action_data = action_data[pair_inds,0]

        data[pair_inds,stats_indices['ell']] = np.divide(data[pair_inds,stats_indices['ell']], pair_min)
        pair_action_data = np.divide(pair_action_data,pair_min)
        action_data[pair_inds,stats_indices['lengths']-stats_indices['times']] = pair_action_data

    return data, action_data

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
    with open("./../data/output/statistics.csv", "w") as file:                  # https://www.geeksforgeeks.org/python/working-csv-files-python/
        csvwriter = csv.writer(file, quoting=csv.QUOTE_MINIMAL)
        csvwriter.writerow(fields)
        csvwriter.writerows(rows)


def get_all_data(directory, file_spec):
    files = fnmatch.filter(os.listdir(directory), file_spec)

    data = np.empty((0,15))
    action_data = np.empty((0,3))
    radius_data = []
    for file in files:

        next_data, next_action_data, next_radius_data = get_data(directory + file)

        data = np.vstack((data, next_data))
        action_data = np.vstack((action_data, next_action_data))
        radius_data += next_radius_data

        # make_success_brain_figure(data)
        # make_success_brain_figures(data)  
        
    return data, action_data, radius_data


def get_data(file):

    data = np.empty((0,15))
    action_data = np.empty((0,3))
    radius_data = []
    
   
    def conv(x):
        x_ = x.decode()
        if len(x) > 2:
            values = np.array([float(xi) for xi in x_.strip("[,]").split(',')])
            return values
        else:
            return np.array([])
        
    convs = {0: lambda x: conv(x), 1: lambda x: conv(x), 2: lambda x: conv(x), 3: lambda x: conv(x)}

    next_data = np.loadtxt(file, delimiter=',', comments='#', usecols=(0,1,2,3,4,5,6,7,8,9,10,11,12,13,14))
    # next_file_root = #column 15
    next_action_data = np.loadtxt(file, delimiter=',', comments='#', usecols=(16,17,18), converters=conv, dtype=object, quotechar='"')
    for i in range(np.shape(next_data)[0]):
        # print(np.shape(next_data[i,:]))
        # print(np.shape(next_action_data[i,0]))
        if (next_data[i,3] < 3) and (next_data[i,3] > 1):
            for j in range(np.shape(next_action_data[i,0])[0]):
                # print(next_action_data[i,1][j])
                for k in range(round(10*next_action_data[i,0][j]/next_data[i,4])):
                    if next_action_data[i,1][j] < 100000:
                        radius_data += [1/(next_action_data[i,1][j]/1000)]
                    else:
                        radius_data += [0]
            # plotter.scatter(next_action_data[i,1], )
    data = np.vstack((data, next_data))
    action_data = np.vstack((action_data, next_action_data))

    return data, action_data, radius_data  

def max_mean_discrepancy(particles, radius_data, gamma=0.5):
    kparticleparticle = 0
    kparticledata = 0
    kdatadata = 0
    for i in range(len(particles)):
        for j in range(len(particles)):
            if not (j == i):
                kparticleparticle += np.exp(-gamma*np.linalg.norm(particles[i] - particles[j]))

        for j in range(len(radius_data)):
            kparticledata += np.exp(-gamma*np.linalg.norm(particles[i] - radius_data[j]))                

    for i in range(len(radius_data)):
        for j in range(len(radius_data)):
            if not (radius_data[j] == radius_data[i]):
                # print(f"{radius_data[i]} {radius_data[j]}")
                kdatadata += np.exp(-gamma*np.linalg.norm(radius_data[i] - radius_data[j]))

    kparticleparticle = kparticleparticle/((len(particles) - 1)*len(particles))
    kparticledata = 2*kparticledata/(len(particles)*len(radius_data))
    kdatadata = kdatadata/(len(radius_data)*(len(radius_data) - 1))

    mmd = kparticleparticle - kparticledata + kdatadata
    return mmd


if __name__=='__main__':

    data, action_data, radius_data = get_all_data('./../data/output/', '*planner_best_actions*.txt')

    plotter.figure()
    plotter.hist(radius_data, bins='doane', density=True)
    
    num_particles = [5, 4, 3, 2]
    colors = ['r', 'darkorange', 'g', 'b', 'blueviolet', 'indigo']

    print(f"primitives: {[0, 15]} mmd: {max_mean_discrepancy([0, 67], radius_data)}")

    print(f"radius data: {len(radius_data)}")
    for i in range(len(num_particles)):

        codebook, distortion = kmeans(radius_data, num_particles[i])
        codebook.sort()
        print(codebook)
        # print(distortion)
        plotter.scatter(codebook, 0*codebook + 0.2 + i*0.01, c=colors[i])
        kmeans_mmd = max_mean_discrepancy(codebook, radius_data)

        bkmeans = BisectingKMeans(n_clusters=num_particles[i]).fit(np.array(radius_data).reshape((-1,1)))
        plotter.scatter(bkmeans.cluster_centers_, 0*bkmeans.cluster_centers_ + 0.3 + i*0.01, c=colors[i])
        bkmeans_mmd = max_mean_discrepancy(bkmeans.cluster_centers_, radius_data)
        bkmeans.cluster_centers_.sort()
        print(bkmeans.cluster_centers_)
        print(f"n: {num_particles[i]} kmeans mmd: {kmeans_mmd} bkmeans mmd: {bkmeans_mmd}")

    plotter.show()


      