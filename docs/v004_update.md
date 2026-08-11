**Bench2Drive Benchmark**

1. We released a new evaluation set with 220 routes (44 scenarios * 5) in it. The xml file is at `leaderboard/data/bench2drive_0.0.4_val.xml`
2. Results:

| Method        | Driving Score | Success Rate | Efficiency |   Comfort | Overtaking |   Merging | Emergency_Brake |  Give_Way | Traffic_Signs |      Mean |
| ------------- | ------------: | -----------: | ---------: | --------: | ---------: | --------: | --------------: | --------: | ------------: | --------: |
| TCP           |         51.44 |        26.36 |      75.67 |     30.16 |      14.49 |     14.06 |           59.64 |     33.33 |         63.95 |     37.10 |
| ThinkTwice   |         52.84|        27.73 |      90.73 |     19.76 |      20.29 |     20.31 |           59.64 |     33.33 |         66.86 |     40.09|
| DriveAdapter |         54.96 |        30.45 |      91.82 |     20.63 |      26.09 |     21.88 |           64.90 |     33.33 |         69.18 |     43.08 |
| ADMLP         |         35.39 |         0.00 |      64.92 |     20.95 |       0.00 |      0.00 |            0.00 |      0.00 |         45.35 |      9.07 |
| UniAD         |         38.69 |        19.09 |     120.16 |     64.09 |      11.59 |     10.94 |           40.35 |     33.33 |          8.14 |     20.87 |
| VAD           |         38.65 |        17.27 |     163.74 | **67.34** |      13.04 |     17.19 |           29.82 |     16.67 |         19.77 |     19.30 |
| DriveTrans    |         60.20 |        35.91 |     168.87 |     17.22 |      30.43 |     39.06 |           49.12 |      0.00 |         61.63 |     36.05 |
| Drive-pi0     |         69.71 |        45.91 |     178.07 |     10.80 |      24.64 |     39.06 |           73.68 |     33.33 |         66.86 |     47.52 |
| Simlingo      |         86.55 |        70.45 | **251.72** |     35.47 |      63.77 |     60.94 |           91.23 | **50.00** |         90.70 |     71.34 |
| LEAD          |     **95.59** |    **87.28** |     201.55 |     20.56 |  **89.86** | **73.44** |       **98.25** | **50.00** |     **97.67** | **81.84** |

**Bench2Drive Data Collection**

1. We released a new training set collected by Think2Drive, containing 44 scenarios * 25 routes = 1100 routes. Please visit https://huggingface.co/datasets/rethinklab/Bench2Drive-V0.0.4 for more details. In the new dataset:
2. Depth is no longer stored as `.png` files, but as `.npz` files to avoid precision loss.
   The dataset is available at https://huggingface.co/datasets/rethinklab/Bench2Drive-V0.0.4-depth
   Related issue: https://github.com/Thinklab-SJTU/Bench2Drive/issues/21
3. We introduced a new modality: **3D occupancy (3D occ)**.
4. We fixed the bug in the Radar sensor position.
5. We fixed the skeleton collection bug.
   Related issue: https://huggingface.co/datasets/rethinklab/Bench2Drive-Map-V0.0.4
6. When vehicles in the scene disappear due to CARLA scenario or Traffic Manager issues, they will now be annotated.

**Visualizer**

1. We open-sourced a data visualizer under `B2DVisualize`, feel free to try it out.

**Bench2Drive Data Fixes**

1. We fixed duplicated lane points in the Map GT: https://github.com/Thinklab-SJTU/Bench2Drive/issues/149. The updated map is available at https://huggingface.co/datasets/rethinklab/Bench2Drive-Map-V0.0.4

**Bench2Drive Zoo**

1. Reduced memory usage in data preprocessing and loading during training.
   (https://github.com/Thinklab-SJTU/Bench2DriveZoo/issues/92 https://github.com/Thinklab-SJTU/Bench2DriveZoo/issues/28)
2. Fixed `ego_lcf_feat` in VAD.
   (https://github.com/Thinklab-SJTU/Bench2DriveZoo/issues/46)
3. Fixed ego pose calculation during closed-loop inference.
   (https://github.com/Thinklab-SJTU/Bench2DriveZoo/pull/102)
4. Fixed `OccHead` in UniAD.
   (https://github.com/Thinklab-SJTU/Bench2DriveZoo/issues/113)
5. Fixed duplicate `NameMapping` entries in configs.
   (https://github.com/Thinklab-SJTU/Bench2DriveZoo/issues/57)
6. Improved readability of some code.

