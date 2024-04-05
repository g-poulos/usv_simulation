# USV Simulation

A generic simulation environment for surface vehicles in Gazebo. The wave simulation
plugins are provided by the [ASV Wave Sim](https://github.com/srmainwaring/asv_wave_sim)
package.

## Environment

+ Ubuntu 22.04 (Jammy)
+ Gazebo Sim, version 7.6.0 (Garden)
+ ROS2 Humble 

## Setup

### Build the ASV Wave Simulation package

Installation instructions [here](https://github.com/srmainwaring/asv_wave_sim?tab=readme-ov-file#installation) 

### Clone the `usv_simulation` repository

```
cd ~/gz_ws/src
git clone https://github.com/g-poulos/usv_simulation
```

### Source the ASV Wave Sim package

```
cd ~/gz_ws/src/usv_simulation
source source.bash
```

## Plugin setup

All the Python scripts used for the plugins are based on the [PyVista](https://github.com/pyvista/pyvista)
library.

### Ocean Current and Wind

To use the ocean current and wind plugins, two data tables must first be created 
using the Python scripts in `feature_extraction_scripts` directory. The **model
STL** and **mass** are required to create the tables. The final CSV files must be located 
to the model `meshes` directory.

For the scripts to work properly:
+ Model origin must be at CoM
+ Model must be manifold
+ A model mesh with more vertices outputs more accurate area calculation

### Steps to create the data tables
### 1. Draft calculation

By defining the STL file and mass in the `compute_draft.py` main, the vehicle draft and
submerged volume can be calculated. The submerged volume is used for the added mass
plugin.

```python
vereniki = "../models/vereniki/meshes/vereniki_scaled3.stl"
print(compute_draft(vereniki, 425, 1025, step_size=0.0001, plot=True))
```

### Parameters 

+ `stl_file`: the STL file of the model 
+ `model_mass`: the mass of the model 
+ `water_density`: the density of the water 
+ `step_size`: step size of the algorithm. Accuracy increases with smaller steps
+ `plot`: if **True**, plots the vehicle floating at the calculated equilibrium  

Example output:
```
Draft:              0.4520999938249588
Calculated Volume:  0.41467056646739586
Theoretical Volume: 0.4146341463414634
```

### 2. Projection Area and Torque table

By defining the `STL file`, `draft` and `number of angles` in the `multithread_table_calc.py` 
the two tables for the ocean current and wind are generated. Note that complex meshes 
require more time to be processed.

```python
draft = 0.4520999938249588
stl_file = "../models/vereniki/meshes/vereniki_scaled3.stl"
num_of_angles = 128

poly = pv.read(stl_file)
surface_part = poly.clip_closed_surface(normal=(0, 0, 1),
                                        origin=(0, 0, poly.bounds[4]+draft))
submerged_part = poly.clip_closed_surface(normal=(0, 0, -1),
                                          origin=(0, 0, poly.bounds[4]+draft))

table = calculate_force_torque_table(surface_part, num_of_angles, 16)
write_list_to_file("../models/vereniki/meshes/wind_table.csv", table)

table = calculate_force_torque_table(submerged_part, num_of_angles, 16)
write_list_to_file("../models/vereniki/meshes/current_table.csv", table)
```

### Parameters (calculate_force_torque_table) 

+ `mesh`: the part of the vehicle to calculate the table
+ `num_of_angles`: number of angles around the vehicle for which the algorithm will calculate force/torque 
                   information. More angles generate tables with greater accuracy but need more time to calculate
+ `num_of_threads`: number of threads used for the operation

### 3. Model SDF
After the tables have been created the plugins can be used by adding the following lines to the 
model SDF.

```html
<plugin filename="WaterCurrent" name="water_current::WaterCurrent">
    <link_name>platform_body</link_name>
    <current>
        <speed>
            <min>0.0</min>
            <max>0.3</max>
            <init>0.1</init>
            <stddev>0.05</stddev>
        </speed>
        <direction>
            <min>155</min>
            <max>205</max>
            <init>180</init>
            <stddev>20</stddev>
        </direction>
    </current>

    <density>1025</density>
    <res_coef>0.8</res_coef>
    <update_rate>1500</update_rate>
    <table_file>current_table.csv</table_file>
    <surface_level>1.0</surface_level>
</plugin>

<plugin filename="Wind" name="wind::Wind">
    <link_name>platform_body</link_name>
    <wind>
        <speed>
            <min>0</min>
            <max>7</max>
            <init>2</init>
            <stddev>2</stddev>
        </speed>
        <direction>
            <min>245</min>
            <max>295</max>
            <init>270</init>
            <stddev>20</stddev>
        </direction>
    </wind>
    <density>1.225</density>
    <res_coef>0.8</res_coef>
    <update_rate>1500</update_rate>
    <table_file>wind_table.csv</table_file>
</plugin>
```

### Parameters

+ The `link_name` attribute sets the link on which the plugin applies
+ The `min`, `max`, `init` and `stddev` attributes control the Integrated White Gaussian Noise that
  generates the speed and direction for both plugins  
+ The `density` attribute sets the fluid density
+ The `res_coef` attribute is the drag coefficient in the [drag equation](https://en.wikipedia.org/wiki/Drag_equation)
+ The `update_rate` attribute is the rate of messages per second for the speed and direction topics
+ The `table_file` attribute is the table file name that was generated in step 2
+ The `surface_level` attribute sets the level above which the wind plugin applies

## Added Mass 

Using the submerged volume calculated in the previous section the added mass plugin can be 
used by adding the following lines to the model SDF.

```html
<plugin filename="AddedMass" name="added_mass::AddedMass">
    <link_name>platform_body</link_name>
    <coef>0.8</coef>
    <density>1025</density>
    <sub_volume>0.41477</sub_volume>
</plugin>
```

### Parameters

+ The `link_name` attribute sets the link on which the plugin applies
+ The `res_coef` attribute is the added mass coefficient 
+ The `density` attribute sets the fluid density
+ The `sub_volume` attribute sets the submerged volume of the vehicle calculated using the  
  draft script in step 1

# Example 

### Source the workspace before first execution
```
cd ~/gz_ws/src/usv_simulation
source source.bash
```
### Using SDF world file directly

```
cd ~/gz_ws/src/usv_simulation
gz sim worlds/waves.sdf
```

### Using launch file

```
cd ~/gz_ws/src/usv_simulation/launch
gz sim launch launch_waves.py
```

The launch file only works for the triangular platform model (vereniki) and bridges GZ and ROS topics.

## Extras

+ The repository [usv_controller](https://github.com/g-poulos/usv_controller) contains:
  + Two controllers for the triangular platform 
  + The dynamic model simulation of the platform that was used to develop the `usv_simulation` package