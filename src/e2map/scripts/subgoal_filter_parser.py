import os
import argparse


e2map_pkg_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def comma_seperated(values):
    """Custom type function for pass comma included command line argument."""
    args = values.split(',')
    assert len(args) == 2, "must contain only two integer numbers."
    
    args = [int(arg) for arg in args]
    return args


def parse_configs():
    parser = argparse.ArgumentParser(description="config for subgoal coordinate filter")
    parser.add_argument('--data_path', type=str, default=os.path.join(e2map_pkg_directory, 'data'), metavar='PATH', help="the path for dataset required for vlmap creation")
    parser.add_argument('--mask_version', type=int, default=1, help='mask version | 0 | 1 |, (default: 1)')
    parser.add_argument('--lang', nargs='+', default=['table', 'sofa', 'chair'], help='default open vocabulary to query VLMap')
    parser.add_argument('--clip_version', type=str, default="ViT-B/32", help='version of CLIP')
    parser.add_argument('--min_area', type=float, default=50.0, help='minimum contour area threshold value')
    parser.add_argument('--max_area', type=float, default=8000.0, help='maximum contour area threshold value')
    parser.add_argument('--init_coord', type=comma_seperated, default=[50,50], help='initial starting coordinate of an agent')
    parser.add_argument('--margin', type=int, default=6, help='marginal value to compute subgoal\'s final coordinate, assumed as the length of an agent')
    parser.add_argument('--vis', action='store_true', help='determine whether to plot subgoals\' coordiates on the landmark indexing map')
    parser.add_argument('--do_crop', action='store_true', help='determine whether to visualize maps with cropped size (caution: only use it for visulization purpose)')
    parser.add_argument('--device', type=str, default='cuda', help='device to run CLIP')
    parser.add_argument('--goal_arrived_thresh', type=float, default=1.0, help='distance threshold to determine whether robot arrive the goal.')
    parser.add_argument('--map_resolution', type=float, default=0.1, help='real meter unit of 1 grid of the map.')
    parser.add_argument('--raycast_resolution', type=float, default=15, help='angle resolution of ray-casting algorithm')
    args = parser.parse_args()
    
    return args
