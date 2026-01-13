import matplotlib.pyplot as plt
import numpy as np


def plot_slices(scan_data, x, y, z, samples, cmap="gist_yarg", figname=None, mask=False):
    """
    Plots orthogonal slices of the segmented scan environment.

    Parameters:
    scan_data (n x m x p): segmented scan int array
    x (int): index for zy slice
    y (int): index for the zx slice
    z (int): index for the xy slice
    maxval (int): max value for the colormap
    cmap (string): colormap to use, default is plasma
    figname (string): name/save location for figure if desired, default is None which results in no saved figure
    mask (bool): True treats data as already masked, False (default) treats data as regular segmented data 

    """
    buffer = 1
    print(np.shape(scan_data))
    zy = np.transpose(scan_data[x,:,:])
    zx = np.transpose(scan_data[:,y,:])
    xy = np.transpose(scan_data[:,:,z])

    maxval = np.max(np.array([np.max(np.array(zy)), np.max(np.array(zx)), np.max(np.array(xy))]))

    zy_indices = [1,2,0,3]
    zx_indices = [0,2,1,3]
    xy_indices = [0,1,2,3]
    zy_samples = samples[np.where(np.abs(samples[:,0] - x) < buffer)[0], :]
    zx_samples = samples[np.where(np.abs(samples[:,1] - y) < buffer)[0], :]
    xy_samples = samples[np.where(np.abs(samples[:,2] - z) < buffer)[0], :]

    print(f"xy: {np.round(xy_samples[0,:])} {np.shape(xy_samples)} zx: {np.round(zx_samples[0,:])} {np.shape(zx_samples)} zy: {np.round(zy_samples[0,:])} {np.shape(zy_samples)}")
    print(f"statuses: {np.unique(samples[:,3])}")
    plot_a_slice(xy, xy_samples[:,xy_indices], 7, cmap, figname, "xy", "z", z, mask)
    plot_a_slice(zy, zy_samples[:,zy_indices], 7, cmap, figname, "yz", "x", x, mask)
    plot_a_slice(zx, zx_samples[:,zx_indices], 7, cmap, figname, "xz", "y", y, mask)
    plt.show()


def plot_a_slice(scan_slice, sample_slice, maxval, cmap, figname, slice_plane, slice_dir, slice_ind, mask):
    """
    Plots a single slice of the the segmented or masked brain.

    Parameters:
    scan_slice (a x b): data image
    maxval (int): max value for the colormap
    cmap (string): colormap to use when plotting
    figname (string): name/location to save image, if None does not save
    slice_plane (string): directions of the plane ordered such that [0] is along x and [1] is along y when plotted
    slice_dir (string): direction the slice was taken
    slice_ind (int): index of the slice taken
    mask (bool): True treat data as masked data, False treat as segmented data, impacts colorbar, labeling, and file name

    """
    plt.figure()
    numstatus = 10
    print(np.unique(scan_slice))
    plt.imshow(scan_slice, cmap=cmap, origin='lower', vmin=0, vmax=maxval)
    if mask:
        plt.title(f"Brain Mask {slice_plane} plane {slice_dir}-{slice_ind}")
        cbar = plt.colorbar()
        cbar.ax.set_yticks([0, 1, 4, 5])
        cbar.ax.set_yticklabels(['workspace', 'obstacles', 'start', 'goal'])

        if figname is not None:
            plt.savefig(f"{figname}-mask{slice_plane}.pdf")          
    else:
        plt.title(f"Brain Scan {slice_plane} plane {slice_dir}-{slice_ind}")
        cbar = plt.colorbar()
        cbar.ax.set_yticks([0, 1, 2, 3, 4, 5, 6, 7])
        cbar.ax.set_yticklabels(['unoccupied', 'skull', 'brain', 'ventricles', 'tumor', 'start', 'goal', 'obstacles'])

    samples_ten = sample_slice[np.where(sample_slice[:,3] == 10)[0], :]
    plt.scatter(samples_ten[:,0], samples_ten[:,1], s=5, c=samples_ten[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)
    samples_eleven = sample_slice[np.where(sample_slice[:,3] == 11)[0], :]
    plt.scatter(samples_eleven[:,0], samples_eleven[:,1], s=5, c=samples_eleven[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)
    samples_twelve = sample_slice[np.where(sample_slice[:,3] == 12)[0], :]
    plt.scatter(samples_twelve[:,0], samples_twelve[:,1], s=5, c=samples_twelve[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)
    samples_thirteen = sample_slice[np.where(sample_slice[:,3] == 13)[0], :]
    plt.scatter(samples_thirteen[:,0], samples_thirteen[:,1], s=5, c=samples_thirteen[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)
    samples_fourteen = sample_slice[np.where(sample_slice[:,3] == 14)[0], :]
    plt.scatter(samples_fourteen[:,0], samples_fourteen[:,1], s=5, c=samples_fourteen[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)




    valid_samples = sample_slice[np.where(sample_slice[:,3] == 0)[0], :]
    
    not_valid_samples = sample_slice[np.where(np.logical_and(sample_slice[:,3] > 0, sample_slice[:,3] < 10))[0], :]
    plt.scatter(not_valid_samples[:,0], not_valid_samples[:,1], s=8, c=not_valid_samples[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)
    plt.scatter(valid_samples[:,0], valid_samples[:,1], s=8, c=valid_samples[:,3], cmap="tab20b", vmin=0, vmax=numstatus, alpha=0.9)
    # sample_status = np.unique(sample_slice[:,3])
    # for status in sample_status:
    #     slice_status = sample_slice[np.where(sample_slice[:,3] == status)[0], :]
    #     plt.scatter(slice_status[:,0], slice_status[:,1], s=1, c=slice_status[:,3], cmap="rainbow", vmin=0, vmax=1)
    plt.xlabel(slice_plane[0])
    plt.ylabel(slice_plane[1])

    cbar = plt.colorbar()
    
    cbar.ax.set_yticks([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    cbar.ax.set_yticklabels(['valid', 'sample collision', 'path collision', 'propagation', 'curvature', 'ell', 'phi', 'goal', 'similar', 'cost', 'sample'])

    if figname is not None:
        plt.savefig(f"{figname}-brain{slice_plane}.pdf")   



if __name__=='__main__':
    transformation_file = "./../data/input/remind_001_obstacles.txt"
    transformation = np.loadtxt(transformation_file, max_rows=4)
    inv_transformation = np.linalg.inv(transformation)

    sample_file = "./../data/output/remind_001_samples.txt"
    cartesian_samples = np.loadtxt(sample_file)
    sample_status = np.copy(cartesian_samples[:,3])
    print(sample_status)
    cartesian_samples[:,3] = np.ones(np.shape(sample_status))
    print(sample_status)
    cartesian_samples = np.transpose(cartesian_samples)
    voxel_samples = np.matmul(inv_transformation, cartesian_samples)
    voxel_samples = np.transpose(voxel_samples)
    voxel_samples[:,3] = sample_status

    segmentation_file = "./envs/ReMIND_segmentation_001.npy"
    segmentation = np.load(segmentation_file)
    x = int(np.round(np.average(voxel_samples[:,0])))
    y = int(np.round(np.average(voxel_samples[:,1])))
    z = int(np.round(np.average(voxel_samples[:,2])))

    print(f"x: {x} y: {y} z: {z}")
    plot_slices(segmentation, x, y, z, voxel_samples)


