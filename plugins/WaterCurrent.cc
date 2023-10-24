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

#include "WaterCurrent.hh"
#include "utility.hh"

using namespace water_current;

class water_current::WaterCurrent::Implementation
{
public: gz::sim::Link link{gz::sim::kNullEntity};

public: gz::transport::Node node;

public: gz::transport::Node::Publisher currentSpeedPub;

public: gz::transport::Node::Publisher azimuthPub;

public: std::string magnitudeTopic = "/waterCurrent/speed";

public: std::string azimuthTopic = "/waterCurrent/azimuth";

public: GaussianNoise speedDistr;

public: GaussianNoise azimuthDistr;

public: float waterCurrentSpeed = 0;

public: float waterCurrentAzimuth = 0;

public: float waterCurrentElevation = 90;
};

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

gz::math::Vector3d speedToForce(gz::sim::EntityComponentManager &_ecm,
                                gz::sim::Link link,
                                float currentSpeed,
                                float direction) {

    gz::math::Vector3d linkLinearVel = toGZVec(link.WorldLinearVelocity(_ecm));
    gz::math::Vector3d wcurrentLinearVel = createForceVector(currentSpeed, 90, direction);
    gz::math::Vector3d relativeVel = wcurrentLinearVel.operator-(linkLinearVel);

    float resistanceCoefficient = 1.2;
    float fluidDensity = 1025.0;
    float surface = 1.0;

    gz::math::Vector3d wcurrentVector = 0.5 * fluidDensity * resistanceCoefficient * relativeVel * surface;
    float relativeVelMagnitude = sqrt(relativeVel.Dot(relativeVel));

    //DEBUG
    gzmsg << "|WATER_CURRENT|_________________________________________\n";
    gzmsg << "linkLinearVel     : " << linkLinearVel << std::endl;
    gzmsg << "wcurrentLinearVel : " << wcurrentLinearVel << std::endl;
    gzmsg << "relativeVel       : " << relativeVel << std::endl;
    gzmsg << "Relative Vel Speed: " << relativeVelMagnitude << " m/s" << std::endl;
    gzmsg << "Current Magnitude : " << sqrt(wcurrentVector.Dot(wcurrentVector)) << " N"<<std::endl;
    gzmsg << "Current           : " << wcurrentVector << std::endl;

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
    if (!_sdf->HasElement("link_name"))
    {
        gzerr << "No <link_name> specified" << std::endl;
        return;
    }
    gz::sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = gz::sim::Link(model.LinkByName(_ecm, linkName));

    if (!this->dataPtr->link.Valid(_ecm))
    {
        gzerr << "Could not find link named [" << linkName
              << "] in model" << std::endl;
        return;
    }

    if (_sdf->HasElement("speed"))
    {
        this->dataPtr->waterCurrentSpeed = _sdf->Get<float>("speed");
    }

    if (_sdf->HasElement("azimuth"))
    {
        this->dataPtr->waterCurrentAzimuth = _sdf->Get<float>("azimuth");
    }

    if (_sdf->HasElement("elevation"))
    {
        this->dataPtr->waterCurrentElevation = _sdf->Get<float>("elevation");
    }

    // Set up the publisher
    double updateRate = 10;
    gz::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->currentSpeedPub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->azimuthPub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->azimuthTopic, opts);

    // Set up the noise distribution
    this->dataPtr->speedDistr = GaussianNoise(0, 0.5);
    this->dataPtr->azimuthDistr = GaussianNoise(0, 2);
}

void WaterCurrent::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

    double speed = this->dataPtr->waterCurrentSpeed + this->dataPtr->speedDistr.getNoise();
    double azimuth = this->dataPtr->waterCurrentAzimuth + this->dataPtr->azimuthDistr.getNoise();
    double elevation = this->dataPtr->waterCurrentElevation + this->dataPtr->speedDistr.getNoise();

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





