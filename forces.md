# Simulation Forces  

Grigoris Poulos 4480

## Wave Forces

The floating body on which the wave forces are applied is approximated by a triangulated 
mesh. The forces are calculated for each triangle of the body and then summed to be applied
to its center of mass.

### **1. Buoyancy** : hydrostatic force acting on a small planar surface $dS$

$$d\overrightarrow{F} = \rho g z dS \overrightarrow{n}$$

Where $\rho$ is the density of water, $z$ the depth in the water and $\overrightarrow{n}$
the normal to the surface

### **2. Damping Forces**

#### <ins>Viscous Water Resistance</ins>

Resistance as water flows across a surface. Tangential to the surface

$$
\overrightarrow{F}_{vi}= \frac{1}{2} \rho C_F(R_n) S_i v_{fi} \overrightarrow{v}_{fi}
$$

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

$$
\overrightarrow{F}_{Di}= -(C_{PD1} \frac{v_i}{v_r} + C_{PD2}(\frac{v_i}{v_r})^2)S_i(cos\theta_i)^{f_p}\overrightarrow{n}_i
$$

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

$$
\overrightarrow{F}_{Di}= (C_{SD1} \frac{v_i}{v_r} + C_{SD2}(\frac{v_i}{v_r})^2)S_i(cos\theta_i)^{f_s}\overrightarrow{n}_i
$$

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

<ins>Damping</ins>

Forces that are used to dampen the linear and angular movement indepentantly 


$$
\overrightarrow{F}_{lin} = \overrightarrow{v} (-rs * cDampL1 + cDampL2 * linSpeed)
$$

Where: 

+ $\overrightarrow{v}$ : the linear velocity of the vehicle
+ $rs$                 : the ratio between the submerged area and the total surface area of the model 
+ $cDampL1, cDampL2$   : linear movement coefficients
+ $linSpeed$           : the linear speed of the vehicle

$$
\overrightarrow{F}_{ang} = \overrightarrow{\omega} (-rs * cDampR1 + cDampR2 * angSpeed)
$$

+ $\overrightarrow{\omega}$ : the angular velocity of the vehicle
+ $rs$                 : the ratio between the submerged area and the total surface area of the model 
+ $cDampR1, cDampR2$   : angular movement coefficients
+ $angSpeed$           : the angular speed of the vehicle



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

$$
\overrightarrow{F}_{current} = \frac{1}{2} \rho_w C S \overrightarrow{V_{rel}^2}
$$

Where: 

+ $\rho_w$ : the density of water
+ $C$      : the drag coefficient
+ $S$      : a reference surface area where the resistance applies
+ $V_rel$      : the relative speed between the body and the water

The same formula is used for the wind force acting on the body.

$$
\overrightarrow{F}_{wind} = \frac{1}{2} \rho_a C S \overrightarrow{V_{rel}^2}
$$

Where: 

+ $\rho_a$ : the density of air 
+ $C$      : the drag coefficient
+ $S$      : a reference surface area where the resistance applies
+ $V$      : the relative speed between the body and the wind

#### Implementation 

```cpp
math::Vector3d linkLinearVel = toGZVec(link.WorldLinearVelocity(_ecm));
math::Vector3d forceLinearVel = sphericalToVector(speed, 90, direction);
math::Vector3d relativeVel = forceLinearVel.operator-(linkLinearVel);

float surface = getSurface(link, _ecm, direction, wrenchFileData);

math::Vector3d forceVector = 
      0.5 * fluidDensity * resCoefficient * relativeVel * surface;
```

## Added Mass 

The added mass force is calculated using Newton's second law of motion:

$$\overrightarrow{F} = -C_a m_a \overrightarrow{a}$$

Where:

+ $C_a$ : the added mass coefficient
+ $m_a$ : the mass of the displaced fluid
+ $a$   : the acceleration of the body

Since the volume of the submerged body is known, the mass of the displaced fluid can be calculated by
multiplying this volume with the water density. We only consider the planar motion of the body for 
the added mass force, so the $z$ component of the force is always zero.

## Engine Thrust

For the engine thrust after the thrust message is published a PID controller is used to apply wrenches 
directly to the propeller link. The angular velocity of the blades is calculated by the equations described in
Fossen's "Guidance and Control of Ocean Vehicles" in page 246. 


$$\omega = \sqrt{\frac{T}{\rho C_T d^4}}$$

Where: 

+ $T$    : thrust 
+ $\rho$ : the density of water
+ $C_T$  : thurst coefficient which relates the angular velocity to actual thrust
+ $d$    : propeller diameter

More about the Thruster Gazebo Plugin [here](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1systems_1_1Thruster.html) 
 