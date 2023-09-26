#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
#include "gz/sim/Util.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
#include <gz/transport/Node.hh>
#include <gz/msgs/float.pb.h>

#include "WaterCurrent.hh"
#include "utility.hh"

using namespace water_current;

class water_current::WaterCurrent::Implementation
{
public: gz::sim::Link link{gz::sim::kNullEntity};

public: gz::transport::Node node;

public: gz::transport::Node::Publisher magnitudePub;

public: gz::transport::Node::Publisher azimuthPub;

public: std::string magnitudeTopic = "/waterCurrent/magnitude";

public: std::string azimuthTopic = "/waterCurrent/azimuth";

public: GaussianNoise distr;
};

gz::math::Vector3d createForceVector(double magnitude, double elevation, double azimuth) {
    gz::math::Vector3d result;

    // Convert degrees to radians
    elevation = elevation * M_PI / 180.0;
    azimuth = azimuth * M_PI / 180.0;

    // Calculate the components
//    result.X() = magnitude * sin(elevation) * cos(azimuth);
//    result.Y() = magnitude * sin(elevation) * sin(azimuth);
//    result.Z() = magnitude * cos(elevation);

    result.X(magnitude * sin(elevation) * cos(azimuth));
    result.Y(magnitude * sin(elevation) * sin(azimuth));
    result.Z(magnitude * cos(elevation));
    return result;
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

    // Set up the publisher
    double updateRate = 10;
    gz::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->magnitudePub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->azimuthPub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->azimuthTopic, opts);

    // Set up the noise distribution
    this->dataPtr->distr = GaussianNoise(0, 10);
}

void WaterCurrent::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

//    float x_force = 1000 + this->dataPtr->distr.getNoise();
//    this->dataPtr->link.AddWorldForce(_ecm, gz::math::Vector3d(x_force, 0, 0));

    double magnitude = 1000 + this->dataPtr->distr.getNoise();
    double azimuth = 30 + this->dataPtr->distr.getNoise();
    gz::math::Vector3d current = createForceVector(magnitude, 90, azimuth);
    this->dataPtr->link.AddWorldForce(_ecm, current);
    gzmsg << "Current: " << current << std::endl;


    gz::msgs::Float forceMsg;
    forceMsg.set_data(magnitude);
    this->dataPtr->magnitudePub.Publish(forceMsg);

    gz::msgs::Float azimuthMsg;
    azimuthMsg.set_data(azimuth);
    this->dataPtr->azimuthPub.Publish(azimuthMsg);
}

GZ_ADD_PLUGIN(water_current::WaterCurrent,
              sim::System,
              WaterCurrent::ISystemConfigure,
              WaterCurrent::ISystemPreUpdate)





