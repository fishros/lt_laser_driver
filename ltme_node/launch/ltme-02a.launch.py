import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
  return launch.LaunchDescription([
    launch_ros.actions.Node(
      package="ltme_node", node_executable="ltme_node", node_name=["ltme_node"],
      output="screen",
      parameters=[
        { "device_model": "LTME-02A" },

        # IP address of LTME-02A
        { "device_address": "192.168.10.160" },
        
        # Frame ID used by the published LaserScan messages
        # { "frame_id": "laser" },

        # If this option is enabled, published LaserScan messages will have their X and Z axes inverted.
        # This is mostly useful when the device is mounted upside down, as it effectively undos the inversion created by the mounting,
        # and makes it look like the scans are from a device installed in a normal, upward direction.
        # { "invert_frame": False },

        # Start and end angle of published scans
        # As LTME-02A has an FOV of 270 degrees, the minimum allowed value for angle_min is -2.356 (about -3 * pi / 4), and the maximum allowed value for angle_max is 2.356 (about 3 * pi / 4)
        # { "angle_min": -1.571 },
        # { "angle_max": 1.571 },

        # Range of angle for which data should be excluded from published scans
        # Leave these two parameters commented out if a full 270 degree FOV is desired
        # { "angle_excluded_min": -0.785 },
        # { "angle_excluded_max": 0.785 },

        # Minimum and maximum range value of published scans
        # Defaults to 0.05 and 30 respectively if not specified
        # { "range_min": 0.05 },
        # { "range_max": 30 },

        # Number of neighboring measurements to be averaged
        # Averaging reduces jitter but angular resolution will also decrease by the same factor
        # { "average_factor": 2 },

        # Adjust how data post-processing stage will filter scan artifacts caused by veiling effect
        # Valid range is between 0 and 100 inclusive, larger value leads to more aggressive filtering
        # { "shadow_filter_strength": 15 },
      ]
    )
  ])
