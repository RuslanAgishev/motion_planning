import contextlib
from matplotlib import animation as anim
import numpy as np
import time


def get_movie_writer(should_write_movie, title, movie_fps, plot_pause_len):
    """
    :param should_write_movie: Indicates whether the animation of SLAM should be written to a movie file.
    :param title: The title of the movie with which the movie writer will be initialized.
    :param movie_fps: The frame rate of the movie to write.
    :param plot_pause_len: The pause durations between the frames when showing the plots.
    :return: A movie writer that enables writing MP4 movie with the animation from SLAM.
    """

    get_ff_mpeg_writer = anim.writers['ffmpeg']
    metadata = dict(title=title, artist='matplotlib', comment='Potential Fields Formation Navigation')
    movie_fps = min(movie_fps, float(1. / plot_pause_len))

    return get_ff_mpeg_writer(fps=movie_fps, metadata=metadata)

@contextlib.contextmanager
def get_dummy_context_mgr():
    """
    :return: A dummy context manager for conditionally writing to a movie file.
    """
    yield None


# HUMAN VELOCITY CALCULATION
hum_time_array = np.ones(10)
hum_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
def hum_vel(human_pose):

    for i in range(len(hum_time_array)-1):
        hum_time_array[i] = hum_time_array[i+1]
    hum_time_array[-1] = time.time()

    for i in range(len(hum_pose_array[0])-1):
        hum_pose_array[0][i] = hum_pose_array[0][i+1]
        hum_pose_array[1][i] = hum_pose_array[1][i+1]
        hum_pose_array[2][i] = hum_pose_array[2][i+1]
    hum_pose_array[0][-1] = human_pose[0]
    hum_pose_array[1][-1] = human_pose[1]
    hum_pose_array[2][-1] = human_pose[2]

    vel_x = (hum_pose_array[0][-1]-hum_pose_array[0][0])/(hum_time_array[-1]-hum_time_array[0])
    vel_y = (hum_pose_array[1][-1]-hum_pose_array[1][0])/(hum_time_array[-1]-hum_time_array[0])
    vel_z = (hum_pose_array[2][-1]-hum_pose_array[2][0])/(hum_time_array[-1]-hum_time_array[0])

    hum_vel = np.array( [vel_x, vel_y, vel_z] )

    return hum_vel