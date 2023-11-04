#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/float.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/common/Mesh.hh>
#include <gz/sim/components/Volume.hh>
#include <gz/sim/components/World.hh>
#include <filesystem>

#include "WaterCurrent.hh"
#include "utility.hh"

using namespace water_current;
using namespace std;

class water_current::WaterCurrent::Implementation
{
    /// \brief the link affected by the water current
public: sim::Link link{sim::kNullEntity};

    /// \brief Transport node
public: transport::Node node;

    /// \brief Transport node publisher for the current speed
public: transport::Node::Publisher currentSpeedPub;

    /// \brief Transport node publisher for the current azimuth
public: transport::Node::Publisher azimuthPub;

    /// \brief Topic where the current speed is published
public: std::string magnitudeTopic = "/waterCurrent/speed";

    /// \brief Topic where the current azimuth is published
public: std::string azimuthTopic = "/waterCurrent/azimuth";

    /// \brief Integrated white noise distribution to generate the current's azimuth
public: IntegratedWhiteNoise azimuthDistr;

    /// \brief Integrated white noise distribution to generate the current's speed
public: IntegratedWhiteNoise speedDistr;

    /// \brief Water current minimum speed
public: float minSpeed = 0;

    /// \brief Water current maximum speed
public: float maxSpeed = 0;

    /// \brief Standard deviation for the current's speed
public: float speedstddev = 0;

    /// \brief Water current minimum azimuth
public: float maxAzimuth = 0;

    /// \brief Water current maximum azimuth
public: float minAzimuth = 0;

    /// \brief Standard deviation for the current's azimuth
public: float azimuthstddev = 0;

    /// \brief Water current's elevation (not used)
public: float elevation = 90;

    /// \brief Update rate buffer for current speed and direction
public: int updateRate = 10;

    /// \brief The density of the fluid
public: float fluidDensity = 1000;

    /// \brief Structure with the surface area information
public: surfaceData *currentSurfaceData;

    /// \brief Name of file the with the surface area information
public: std::string surfaceAreaFile = "current_surface.txt";

    /// \brief Level of the water surface below which the current applies
public: float surfaceLevel = 1.0;
};

//////////////////////////////////////////////////
WaterCurrent::WaterCurrent()
    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void WaterCurrent::Configure(const sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          sim::EntityComponentManager &_ecm,
                          sim::EventManager &_eventMgr)
{
    // Parse required elements.
    if (!_sdf->HasElement("link_name")) {
        gzerr << "No <link_name> specified" << std::endl;
        return;
    }
    sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
    this->dataPtr->link.EnableVelocityChecks(_ecm, true);

    if (!this->dataPtr->link.Valid(_ecm)) {
        gzerr << "Could not find link named [" << linkName
              << "] in model" << std::endl;
        return;
    }

    if (_sdf->HasElement("current")) {
        sdf::ElementPtr currentObjSDF = _sdf->GetElementImpl("current");

        if (currentObjSDF->HasElement("speed")) {
            sdf::ElementPtr speedObjSDF = currentObjSDF->GetElementImpl("speed");

            if (speedObjSDF->HasElement("min")) {
                this->dataPtr->minSpeed = speedObjSDF->Get<float>("min");
            }
            if (speedObjSDF->HasElement("max")) {
                this->dataPtr->maxSpeed = speedObjSDF->Get<float>("max");
            }
            if (speedObjSDF->HasElement("stddev")) {
                this->dataPtr->speedstddev = speedObjSDF->Get<float>("stddev");
            }
            gzmsg << "[Water Current] Speed: " << " Min " << this->dataPtr->minSpeed
                                               << ", Max " << this->dataPtr->maxSpeed
                                               << ", stddev " << this->dataPtr->speedstddev << std::endl;
        }

        if (currentObjSDF->HasElement("direction")) {
            sdf::ElementPtr directionObjSDF = currentObjSDF->GetElementImpl("direction");

            if (directionObjSDF->HasElement("min")) {
                this->dataPtr->minAzimuth = directionObjSDF->Get<float>("min");
            }
            if (directionObjSDF->HasElement("max")) {
                this->dataPtr->maxAzimuth = directionObjSDF->Get<float>("max");
            }
            if (directionObjSDF->HasElement("stddev")) {
                this->dataPtr->azimuthstddev = directionObjSDF->Get<float>("stddev");
            }
            gzmsg << "[Water Current] Direction: " << " Min " << this->dataPtr->minAzimuth
                                                   << ", Max " << this->dataPtr->maxAzimuth
                                                   << ", stddev " << this->dataPtr->azimuthstddev << std::endl;
        }
    }

    if (_sdf->HasElement("density"))
        this->dataPtr->fluidDensity = _sdf->Get<float>("density");
    gzmsg << "[Water Current] Density: " << this->dataPtr->fluidDensity << std::endl;

    if (_sdf->HasElement("update_rate"))
        this->dataPtr->updateRate = _sdf->Get<float>("update_rate");
    gzmsg << "[Water Current] Update Rate: " << this->dataPtr->updateRate << std::endl;

    if (_sdf->HasElement("surface_area_file"))
        this->dataPtr->surfaceAreaFile = _sdf->Get<std::string>("surface_area_file");
    gzmsg << "[WaterCurrent] Surface Area File: " << this->dataPtr->surfaceAreaFile << std::endl;

    if (_sdf->HasElement("surfaceLevel"))
        this->dataPtr->surfaceLevel = _sdf->Get<float>("surfaceLevel");
    gzmsg << "[WaterCurrent] Surface Level: " << this->dataPtr->surfaceLevel << std::endl;

    // Set up the publishers
    double updateRate = this->dataPtr->updateRate;
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->currentSpeedPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->azimuthPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->azimuthTopic, opts);

    // Set up the noise distributions
    this->dataPtr->speedDistr = IntegratedWhiteNoise(0,
                                                     this->dataPtr->speedstddev,
                                                     this->dataPtr->minSpeed,
                                                     this->dataPtr->maxSpeed,
                                                     0.01);
    this->dataPtr->azimuthDistr = IntegratedWhiteNoise(0,
                                                       this->dataPtr->azimuthstddev,
                                                       this->dataPtr->minAzimuth,
                                                       this->dataPtr->maxAzimuth,
                                                       0.01);

    // Read file with area of application
    std::string currentSurfaceAreaFile = getModelFile(_ecm, "current_surface.txt");
    this->dataPtr->currentSurfaceData = readAreaFile(currentSurfaceAreaFile);
}

//////////////////////////////////////////////////
void WaterCurrent::PreUpdate(const sim::UpdateInfo &_info,
                             sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

    // Get values for speed and direction
    double speed = this->dataPtr->speedDistr.getValue();
    double azimuth = this->dataPtr->azimuthDistr.getValue();

    if (this->dataPtr->link.WorldPose(_ecm)->Z() < this->dataPtr->surfaceLevel) {
        math::Vector3d current = calculateForce(_ecm,
                                                this->dataPtr->link,
                                                speed,
                                                azimuth,
                                                this->dataPtr->currentSurfaceData,
                                                this->dataPtr->fluidDensity);
        this->dataPtr->link.AddWorldForce(_ecm, current);
    }

    // Publish the messages
    msgs::Float forceMsg;
    forceMsg.set_data(speed);
    this->dataPtr->currentSpeedPub.Publish(forceMsg);

    msgs::Float azimuthMsg;
    azimuthMsg.set_data(azimuth);
    this->dataPtr->azimuthPub.Publish(azimuthMsg);
}

GZ_ADD_PLUGIN(water_current::WaterCurrent,
              sim::System,
              WaterCurrent::ISystemConfigure,
              WaterCurrent::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(water_current::WaterCurrent,
                    "water_current::WaterCurrent")