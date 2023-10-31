#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
#include "gz/sim/Util.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
#include <gz/transport/Node.hh>
#include <gz/msgs/float.pb.h>
#include <gz/math/Vector3.hh>
#include <cmath>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Volume.hh>
#include <gz/sim/components/World.hh>


#include "WaterCurrent.hh"
#include "utility.hh"

using namespace water_current;
using namespace std;
using namespace gz;             //TODO: Clean up gz namespace stuff

class water_current::WaterCurrent::Implementation
{
public: gz::sim::Link link{gz::sim::kNullEntity};

public: gz::transport::Node node;

public: gz::transport::Node::Publisher currentSpeedPub;

public: gz::transport::Node::Publisher azimuthPub;

public: std::string magnitudeTopic = "/waterCurrent/speed";

public: std::string azimuthTopic = "/waterCurrent/azimuth";

public: IntegratedWhiteNoise azimuthDistr;

public: IntegratedWhiteNoise speedDistr;

public: float waterCurrentSpeed = 0;

public: float waterCurrentAzimuth = 0;

public: float waterCurrentElevation = 90;

public: float speedstddev = 0;

public: float azimuthstddev = 0;

public: int updateRate = 10;

public: float volumeRatio;
};

float resCoefficient = 1;

float fluidDensity = 1000;

float *area_p;

float *angle_p;


gz::math::Vector3d createForceVector(double magnitude, double elevation, double azimuth) {
    gz::math::Vector3d result;

    // Convert degrees to radians
    elevation = elevation * M_PI / 180.0;
    azimuth = azimuth * M_PI / 180.0;

    // Calculate the components
    result.X(magnitude * sin(elevation) * cos(azimuth));
    result.Y(magnitude * sin(elevation) * sin(azimuth));
    result.Z(magnitude * cos(elevation));
    return result;
}

gz::math::Vector3d toGZVec(std::optional<gz::math::Vector3<double>> vec) {
    return gz::math::Vector3d(vec->X(), vec->Y(), vec->Z());
}

float getSurface(sim::Link link, sim::EntityComponentManager &_ecm, float azimuth) {
    auto q = link.WorldPose(_ecm)->Rot().Normalized();

    // Convert quaternion to yaw
    double siny_cosp = 2 * (q.W() * q.Z() + q.X() * q.Y());
    double cosy_cosp = 1 - 2 * (q.Y() * q.Y() + q.Z() * q.Z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    if (yaw < 0)
        yaw = yaw + 2*M_PI;

    float relative_angle = (azimuth * (M_PI / 180)) - yaw;

    if (relative_angle < 0)
        relative_angle = relative_angle + 2 * M_PI;

    int closest_i = findClosest(angle_p, 256, relative_angle);     //TODO: Remove constant length

    // DEBUG
//    gzmsg << "Boat Yaw " << yaw <<
//             " Total: " << relative_angle <<
//             " A_matrix: " << angle_p[closest_i] <<
//             " Area: " << area_p[closest_i] <<std::endl;
    return area_p[closest_i];
}

gz::math::Vector3d speedToForce(gz::sim::EntityComponentManager &_ecm,
                                gz::sim::Link link,
                                float currentSpeed,
                                float direction) {

    gz::math::Vector3d linkLinearVel = toGZVec(link.WorldLinearVelocity(_ecm));
    gz::math::Vector3d wcurrentLinearVel = createForceVector(currentSpeed, 90, direction);
    gz::math::Vector3d relativeVel = wcurrentLinearVel.operator-(linkLinearVel);

    float surface = getSurface(link, _ecm, direction);

    gz::math::Vector3d wcurrentVector = 0.5 * fluidDensity * resCoefficient * relativeVel * surface;

    //DEBUG
//    float relativeVelMagnitude = sqrt(relativeVel.Dot(relativeVel));
//    gzmsg << "|WATER_CURRENT|_________________________________________\n";
//    gzmsg << "linkLinearVel     : " << linkLinearVel << std::endl;
//    gzmsg << "wcurrentLinearVel : " << wcurrentLinearVel << std::endl;
//    gzmsg << "relativeVel       : " << relativeVel << std::endl;
//    gzmsg << "Relative Vel Speed: " << relativeVelMagnitude << " m/s" << std::endl;
//    gzmsg << "Current Magnitude : " << sqrt(wcurrentVector.Dot(wcurrentVector)) << " N"<<std::endl;
//    gzmsg << "Current           : " << wcurrentVector << std::endl;

    return wcurrentVector;
}


WaterCurrent::WaterCurrent()
    : System(), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

void WaterCurrent::Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr)
{
    // Parse required elements.
    if (!_sdf->HasElement("link_name")) {
        gzerr << "No <link_name> specified" << std::endl;
        return;
    }
    gz::sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = gz::sim::Link(model.LinkByName(_ecm, linkName));

    if (!this->dataPtr->link.Valid(_ecm)) {
        gzerr << "Could not find link named [" << linkName
              << "] in model" << std::endl;
        return;
    }

    if (_sdf->HasElement("speed")) {
        this->dataPtr->waterCurrentSpeed = _sdf->Get<float>("speed");
    }

    if (_sdf->HasElement("azimuth")) {
        this->dataPtr->waterCurrentAzimuth = _sdf->Get<float>("azimuth");
    }

    if (_sdf->HasElement("elevation")) {
        this->dataPtr->waterCurrentElevation = _sdf->Get<float>("elevation");
    }

    if (_sdf->HasElement("speed_stddev")) {
        this->dataPtr->speedstddev = _sdf->Get<float>("speed_stddev");
    }

    if (_sdf->HasElement("azimuth_stddev")) {
        this->dataPtr->azimuthstddev = _sdf->Get<float>("azimuth_stddev");
    }

    if (_sdf->HasElement("res_coef")) {
        resCoefficient = _sdf->Get<float>("res_coef");
    }

    if (_sdf->HasElement("density")) {
        fluidDensity = _sdf->Get<float>("density");
    }

    if (_sdf->HasElement("update_rate")) {
        this->dataPtr->updateRate = _sdf->Get<float>("update_rate");
    }

    // Set up the publisher
    double updateRate = this->dataPtr->updateRate;
    gz::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->currentSpeedPub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->azimuthPub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->azimuthTopic, opts);

    // Set up the noise distribution
    this->dataPtr->speedDistr = IntegratedWhiteNoise(0, 0.5, 1, 2, 0.01);
    this->dataPtr->azimuthDistr = IntegratedWhiteNoise(0, 1, 30, 45, 0.01);


    // Compute surface area of application
    readAreaFile(angle_p, area_p);
}

void WaterCurrent::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

    double speed = this->dataPtr->speedDistr.getValue();
    double azimuth = this->dataPtr->azimuthDistr.getValue();

    // TODO: Remove elevation
//    double elevation = this->dataPtr->waterCurrentElevation + this->dataPtr->speedDistr.getNoise();

    if (this->dataPtr->link.WorldPose(_ecm)->Z() < 1) {
        gz::math::Vector3d current = speedToForce(_ecm, this->dataPtr->link, speed, azimuth);
        this->dataPtr->link.AddWorldForce(_ecm, current);
    }

    gz::msgs::Float forceMsg;
    forceMsg.set_data(speed);
    this->dataPtr->currentSpeedPub.Publish(forceMsg);

    gz::msgs::Float azimuthMsg;
    azimuthMsg.set_data(azimuth);
    this->dataPtr->azimuthPub.Publish(azimuthMsg);
}

GZ_ADD_PLUGIN(water_current::WaterCurrent,
              sim::System,
              WaterCurrent::ISystemConfigure,
              WaterCurrent::ISystemPreUpdate)





