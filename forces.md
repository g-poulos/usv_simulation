# Simulation Forces  

Grigoris Poulos 4480

## Wave Forces

The floating body on which the wave forces are applied is approximated by a triangulated 
mesh. The forces are calculated for each triangle of the body and then summed to be applied
to its center of mass.

### **1. Buoyancy** : hydrostatic force acting on a small planar surface $dS$
$$d \overrightarrow{F} = \rho g z dS \overrightarrow{n}$$

Where $\rho$ is the density of water, $z$ the depth in the water and $\overrightarrow{n}$
the normal to the surface

### **2. Damping Forces**

#### <ins>Viscous Water Resistance</ins>

Resistance as water flows across a surface. Tangential to the surface

$$\overrightarrow{F}_{vi} = \frac{1}{2} \rho C_F(R_n) S_i v_{fi} \overrightarrow{v}_{fi}$$

Where:

  + $\rho$                    : density of water
  + $C_F(R_n)$                : Viscous Drag Coefficient and $R_n$ Reynolds Number 
  + $S_i$                     : Area of application
  + $\overrightarrow{v}_{fi}$ : relative velocity of the flow 

#### Implementation 

```cpp
// Force
double fDrag = 0.5 * rho * cF * subTriProps.area
  * std::sqrt(subTriProps.vf.squared_length());
cgal::Vector3 force = subTriProps.vf * fDrag;
sumForce += force;

// Torque;
cgal::Vector3 torque = CGAL::cross_product(subTriProps.xr, force);
sumTorque += torque;
```

Where `subTriProps.xr` $=centroid - CoM$ of the triangle 

#### <ins>Pressure Drag Forces</ins>

The pressure drag forces are trying to capture both the forces allowing a boat to turn,
 limiting its ability to drift sideways, and the planing forces which push the boat out
of the water. These forces act along the normal of the surface. 

$$\overrightarrow{F}_{Di} = -(C_{PD1} \frac{v_i}{v_r} + C_{PD2}(\frac{v_i}{v_r})^2)S_i(cos\theta_i)^{f_p}\overrightarrow{n}_i$$

if $cos\theta_i$ positive

Where: 

+ $f_p$     : falloff power of the facing dot product. It controls how fast or slow the drag 
falls off when the surface's normal transitions from parallel to the point velocity, when 
the drag force should reach its peak intensity, to perpendicular to it when the drag force should vanish. 
+ $C_{PD1}$            : linear drag coefficient
+ $C_{PD2}$            : quadratic drag coefficient
+ $v_r$                : reference speed
+ $\overrightarrow{n}$ : the normal to the surface


Similarly for the suction drag we have: 

$$\overrightarrow{F}_{Di} = (C_{SD1} \frac{v_i}{v_r} + C_{SD2}(\frac{v_i}{v_r})^2)S_i(cos\theta_i)^{f_s}\overrightarrow{n}_i$$

if $cos\theta_i$ negative


Where: 

+ $f_s$     : falloff power for suction
+ $C_{SD1}$ : linear suction drag coefficient
+ $C_{SD2}$ : quadratic suction drag coefficient
+ $v_r$     : reference speed

#### Implementation 

```cpp
  double S    = subTriProps.area;
  double vp   = std::sqrt(subTriProps.vp.squared_length());
  double cosTheta = subTriProps.cosTheta;

  double v    = vp / vRDrag;
  double drag = 0.0;
  if (cosTheta >= 0.0)
  {
    drag = -(cPDrag1 * v + cPDrag2 * v * v) * S * std::pow(cosTheta, fPDrag);
  } else {
    drag =  (cSDrag1 * v + cSDrag2 * v * v) * S * std::pow(-cosTheta, fSDrag);
  }
  cgal::Vector3 force = subTriProps.normal * drag;
  sumForce += force;

  // Torque;
  cgal::Vector3 torque = CGAL::cross_product(subTriProps.xr, force);
  sumTorque += torque;
```

Where `subTriProps.xr` $=centroid - CoM$ of the triangle 

<ins>Slamming Forces/ Damping</ins>

A force to capture the violence of the response of the fluid to sudden accelerations
or penetrations, which are almost like rigid collision. The sum of all the slamming
forces on all the submerged triangles must be just enough to stop the motion on the 
boat entirely, not more.  


$$\overrightarrow{F}_{slamming} = \frac{2 \sum A_{j}^{submerged}cos\theta_j}{S^{total}} m \overrightarrow{v}$$

Where: 

+ $A_{j}^{submerged}cos\theta_j$ : is the surface area of the submerged part of a triangle 
  projected onto a plane perpedicular to the velocity $\overrightarrow{v}$
+ $S_{total}$ : the total surface area of the boat
+ $\overrightarrow{v}$ : the velocity of the boat
+ $m$ : the boat's mass 
 

#### Implementation 

```cpp
double area = this->data->area;
double subArea = this->data->submergedArea;
double rs = subArea / area;

// Force
auto& v = this->data->linVelocity;
double linSpeed = std::sqrt(v.squared_length());
double cL = - rs * (cDampL1 + cDampL2 * linSpeed);
cgal::Vector3 force = v * cL;

//Torque
auto& omega = this->data->angVelocity;
double angSpeed = std::sqrt(omega.squared_length());
double cR = - rs * (cDampR1 + cDampR2 * angSpeed);
cgal::Vector3 torque = omega * cR;

this->data->force  += force;
this->data->torque += torque;
```
## Water Current / Wind 

For the force acting on the floating body due to water current we use the drag 
equation assuming that $V$ is the relative speed between the body and the water: 

$$F_{current} = \frac{1}{2} \rho_w C S V_{rel}^2$$

Where: 

+ $\rho_w$ : the density of water
+ $C$      : the drag coefficient
+ $S$      : a reference surface area where the resistance applies
+ $V_rel$      : the relative speed between the body and the water

The same formula is used for the wind force acting on the body.

$$F_{wind} = \frac{1}{2} \rho_a C S V_{rel}^2$$

Where: 

+ $\rho_a$ : the density of air 
+ $C$      : the drag coefficient
+ $S$      : a reference surface area where the resistance applies
+ $V$      : the relative speed between the body and the wind

## Added Mass 

The added mass force is calculated using Newton's second law of motion:

$$\overrightarrow{F} = m_a \overrightarrow{a}$$

Where:

+ $m_a$ : the total mass of the displaced fluid
+ $a$   : the acceleration of the body



## Engine Thrust

After the thrust message is published a PID controller is used to apply wrenches directly to the
propeller link. The angular velocity of the blades is calculated by the equations described in
Fossen's "Guidance and Control of Ocean Vehicles" in page 246. 


$$\omega = \sqrt{\frac{T}{\rho C_T d^4}}$$

Where: 

+ $T$    : thrust 
+ $\rho$ : the desnity of water
+ $C_T$  : thurst coefficient which relates the angular velocity to actual thrust
+ $d$    : propeller diameter

More about the Thruster Gazebo Plugin [here](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1systems_1_1Thruster.html) 
 