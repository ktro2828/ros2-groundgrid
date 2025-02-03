# ros2-groundgrid

ROS 2 support of GroundGrid: [dcmlr/groundgrid](https://github.com/dcmlr/groundgrid).

Paper: [GroundGrid: LiDAR Point Cloud Segmentation and Terrain Estimation; IEEE'2024](https://arxiv.org/abs/2405.15664)

## Parameters

| Name                                                   | Type     | Default  | Description                                                                                                                  |
| ------------------------------------------------------ | -------- | -------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `point_count_cell_variance_threshold`                  | `int`    | `10`     | If a cell has at least this much points, the variance of just that cell is used instead of the variance of 3x3 or 5x5 patch. |
| `max_ring`                                             | `int`    | `1024`   | Maximum LiDAR ring for ground detection consideration.                                                                       |
| `ground_patch_detection_minimum_threshold`             | `double` | `0.0001` | If the ground patch layer at a cell is below this value, a cell without the minimum point count can be classified as ground. |
| `distance_factor`                                      | `double` | `0.0001` | Compensates for the geometric dilution of the point density with the distance.                                               |
| `minimum_distance_factor`                              | `double` | `0.0005` | Minimum value for the distance factor.                                                                                       |
| `minimum_point_height_threshold`                       | `double` | `0.3`    | Points lower than ground height + threshold are considered ground points.                                                    |
| `minimum_point_height_obstacle_threshold`              | `double` | `0.1`    | Points lower than ground height + threshold are considered ground points [m].                                                |
| `outlier_tolerance`                                    | `double` | `0.1`    | Outlier detection tolerance [m].                                                                                             |
| `ground_patch_detection_minimum_point_count_threshold` | `double` | `0.25`   | Minimum point count for ground patch detection in percent of expected point count.                                           |
| `patch_size_change_distance`                           | `double` | `20.0`   | Distance from the center from which on the patch size is increased [m].                                                      |
| `occupied_cells_decrease_factor`                       | `double` | `5.0`    | Occupied cells decrease factor [100/x %].                                                                                    |
| `occupied_cells_point_count_factor`                    | `double` | `50.0`   | Occupied cells point count factor [100/x %].                                                                                 |
| `min_outlier_detection_ground_confidence`              | `double` | `1.25`   | Minimum ground confidence to consider lower points an outlier (5x5 patch).                                                   |
| `thread_count`                                         | `int`    | `8`      | Maximum number of threads.                                                                                                   |
