"""Detection profiles (features, color ranges, and weights) for different objects."""

data_prefix = ''
data_prefix = '/home/pi/rosbots_catkin_ws/src/quad/scripts/vision/'  # pi's ROS path


class Profile:
    def __init__(self, label, feature_file, color_ranges, colors, weights):
        """Information for Detector to recognize a specific object class."""
        self.label = label
        self.feature_file = feature_file
        self.color_ranges = color_ranges
        self.colors = colors
        self.weights = weights  # weight of corresponding colored keypoints


# HSV color ranges and representative BGR colors
purple = (255, 0, 255)
purple_range = [[145, 40, 0],
                [180, 255, 255]]

blue = (150, 0, 0)
blue_range = [[45, 0, 35],
              [55, 255, 195]]


orange = (0, 165, 255)
orange_range = [[0, 84, 60],
                [22, 252, 255]]

red = (10, 10, 255)
red_range = [[160, 15, 34],
             [180, 255, 255]]

white = (255, 255, 255)
white_range = [[1, 1, 210],
               [180, 252, 251]]

wqb = [[85, 10, 0],
       [110, 255, 255]]

bqb = [[90, 90, 49],
       [119, 120, 105]]

black = (0, 0, 0)
black_range = [[0, 0, 0],
               [180, 255, 45]]

green = (0, 255, 0)
green_range = [[45, 60, 150],
               [71, 160, 253]]

# Profiles
sim_drone = Profile('Sim Drone',
                    'data/sim2_870.dat',
                    [purple_range,  blue_range],
                    [purple,        blue],
                    [6,             4])

idea_orange = Profile('IDEA Orange',
                      # 'data/orange_all_152.dat',
                      data_prefix + 'data/io-323_2941.dat',
                      # 'data/io_full_6664.dat',
                      # 'data/io_323_4310.dat',
                      # 'data/io_323_886.dat',
                      # 'data/io_6290.dat',
                      # 'data/orange2_trained_388.dat',
                      [orange_range, white_range],
                      [orange, white],
                      [10, 4])

idea_red = Profile('IDEA Red',
                   data_prefix + 'data/ir-2.dat',
                   # 'data/io-323_2941.dat',
                   [orange_range, red_range],
                   [orange, red],
                   [5, 5])

quadbox_black = Profile('Quadbox Black',
                        # 'data/black_trained_289.dat',
                        #    'data/qb1_1998.dat',
                        data_prefix + 'data/qwc_1424.dat',
                        [black_range, bqb, green_range],
                        [red, white, green],
                        [4, 8, 9])

quadbox_white = Profile('Quadbox White',
                        data_prefix + 'data/qw_1_4144.dat',
                        # 'data/qb1_1998.dat',
                        [wqb, white_range],
                        [orange, white],
                        [6, 5])
