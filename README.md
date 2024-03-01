# USV Simulation

A generic simulation environment for surface vehicles in Gazebo. The wave simulation
plugins are provided by the [ASV Wave Sim](https://github.com/srmainwaring/asv_wave_sim).

# Ocean Current and Wind

To use the ocean current and wind plugins, two data tables must be first created 
using the Python scripts in `feature_extraction_scripts` directory. Only the model
STL and mass are required to create the tables.

For the scripts to work properly:
+ Model origin must be at CoM
+ Model must be manifold
+ A model mesh with more vertices outputs more accurate area calculation

## Steps to create the data tables
### 1. Draft calculation

By defining the STL file and mass in the `compute_draft.py` main the draft and
submerged volume are calculated. The submerged volume is used for the added mass
plugin.

```terminal
Draft:              0.4520999938249588
Calculated Volume:  0.41467056646739586
Theoretical Volume: 0.4146341463414634
```

### 2. Projection area and Torque table

By defining the STL file, draft and number of angles in the `multithread_table_calc.py` 
the two tables for the ocean current and wind are generated. Note that complex meshes 
require more time to be processed. 