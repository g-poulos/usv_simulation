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
#include <gz/msgs/details/vector3d.pb.h>

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

    /// \brief Transport node publisher for the current direction
public: transport::Node::Publisher directionPub;

    /// \brief Transport node publisher for the current force
public: transport::Node::Publisher forcePub;

    /// \brief Transport node publisher for the torque force
public: transport::Node::Publisher torquePub;

    /// \brief Topic where the current speed is published
public: std::string magnitudeTopic = "/waterCurrent/speed";

    /// \brief Topic where the current direction is published
public: std::string directionTopic = "/waterCurrent/direction";

    /// \brief Topic where the current force is published
public: std::string forceTopic = "/waterCurrent/force";

    /// \brief Topic where the current torque is published
public: std::string torqueTopic = "/waterCurrent/torque";

    /// \brief Integrated white noise distribution to generate the current's direction
public: IntegratedWhiteNoise directionDistr;

    /// \brief Integrated white noise distribution to generate the current's speed
public: IntegratedWhiteNoise speedDistr;

    /// \brief Water current minimum speed
public: float minSpeed = 0;

    /// \brief Water current maximum speed
public: float maxSpeed = 0;

    /// \brief Initial current speed
public: float initSpeed = 0;

    /// \brief Standard deviation for the current's speed
public: float speedstddev = 0;

    /// \brief Water current minimum direction
public: float maxDirection = 0;

    /// \brief Water current maximum direction
public: float minDirection = 0;

    /// \brief Initial current direction
public: float initDirection = 0;

    /// \brief Standard deviation for the current's direction
public: float directionstddev = 0;

    /// \brief Time passed between iterations (overwritten in preUpdate)
public: double dt = 0.001;

    /// \brief Water current elevation (not used)
public: float elevation = 90;

    /// \brief Update rate buffer for current speed and direction
public: int updateRate = 10;

    /// \brief The density of the fluid
public: float fluidDensity = 1000;

    /// \brief Resistance coefficient of the surface
public: float resCoefficient = 1;

    /// \brief Structure with the surface area information
public: wrenchFileData *currentSurfaceData;

    /// \brief Name of file the with the surface area information
public: std::string tableFileName = "current_table.csv";

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
            if (speedObjSDF->HasElement("init")) {
                this->dataPtr->initSpeed = speedObjSDF->Get<float>("init");
            }
            if (speedObjSDF->HasElement("stddev")) {
                this->dataPtr->speedstddev = speedObjSDF->Get<float>("stddev");
            }
            gzmsg << "[WaterCurrent] Speed: " << " Min " << this->dataPtr->minSpeed
                                              << ", Max " << this->dataPtr->maxSpeed
                                              << ", Initial " << this->dataPtr->initSpeed
                                              << ", stddev " << this->dataPtr->speedstddev << std::endl;
        }

        if (currentObjSDF->HasElement("direction")) {
            sdf::ElementPtr directionObjSDF = currentObjSDF->GetElementImpl("direction");

            if (directionObjSDF->HasElement("min")) {
                this->dataPtr->minDirection = directionObjSDF->Get<float>("min");
            }
            if (directionObjSDF->HasElement("max")) {
                this->dataPtr->maxDirection = directionObjSDF->Get<float>("max");
            }
            if (directionObjSDF->HasElement("init")) {
                this->dataPtr->initDirection = directionObjSDF->Get<float>("init");
            }
            if (directionObjSDF->HasElement("stddev")) {
                this->dataPtr->directionstddev = directionObjSDF->Get<float>("stddev");
            }
            gzmsg << "[WaterCurrent] Direction: " << " Min " << this->dataPtr->minDirection
                  << ", Max " << this->dataPtr->maxDirection
                  << ", Initial " << this->dataPtr->initDirection
                  << ", stddev " << this->dataPtr->directionstddev << std::endl;
        }
    }

    if (_sdf->HasElement("density"))
        this->dataPtr->fluidDensity = _sdf->Get<float>("density");
    gzmsg << "[Water Current] Density: " << this->dataPtr->fluidDensity << std::endl;

    if (_sdf->HasElement("res_coef"))
        this->dataPtr->resCoefficient = _sdf->Get<float>("res_coef");
    gzmsg << "[Water Current] Resistance Coefficient: " << this->dataPtr->resCoefficient << std::endl;

    if (_sdf->HasElement("update_rate"))
        this->dataPtr->updateRate = _sdf->Get<float>("update_rate");
    gzmsg << "[Water Current] Update Rate: " << this->dataPtr->updateRate << std::endl;

    if (_sdf->HasElement("table_file"))
        this->dataPtr->tableFileName = _sdf->Get<std::string>("table_file");
    gzmsg << "[WaterCurrent] Table File: " << this->dataPtr->tableFileName << std::endl;

    if (_sdf->HasElement("surfaceLevel"))
        this->dataPtr->surfaceLevel = _sdf->Get<float>("surfaceLevel");
    gzmsg << "[WaterCurrent] Surface Level: " << this->dataPtr->surfaceLevel << std::endl;

    // Set up the publishers
    double updateRate = this->dataPtr->updateRate;
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->currentSpeedPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->directionPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->directionTopic, opts);

    this->dataPtr->forcePub =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->forceTopic, opts);

    this->dataPtr->torquePub =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->torqueTopic, opts);

    // Set up the noise distributions
    this->dataPtr->speedDistr = IntegratedWhiteNoise(0,
                                                     this->dataPtr->speedstddev,
                                                     this->dataPtr->minSpeed,
                                                     this->dataPtr->maxSpeed,
                                                     this->dataPtr->initSpeed,
                                                     this->dataPtr->dt);
    this->dataPtr->directionDistr = IntegratedWhiteNoise(0,
                                                         this->dataPtr->directionstddev,
                                                         this->dataPtr->minDirection,
                                                         this->dataPtr->maxDirection,
                                                         this->dataPtr->initDirection,
                                                         this->dataPtr->dt);

    // Read file with area of application
    std::string currentTablrFile = getModelFile(_ecm, this->dataPtr->tableFileName);
    this->dataPtr->currentSurfaceData = read_csv(currentTablrFile);
}

//////////////////////////////////////////////////
void WaterCurrent::PreUpdate(const sim::UpdateInfo &_info,
                             sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

    

    // Set dt / Convert nanoseconds to seconds
    auto dt = static_cast<double>(_info.dt.count())/1e9;
    this->dataPtr->dt = dt;

    // Get values for speed and direction
    double speed = this->dataPtr->speedDistr.getValue();
    double direction = this->dataPtr->directionDistr.getValue();

    wrenchData wrench = calculateWrench(_ecm,
                                        this->dataPtr->link,
                                        speed,
                                        direction,
                                        this->dataPtr->currentSurfaceData,
                                        this->dataPtr->fluidDensity,
                                        this->dataPtr->resCoefficient);

    if (this->dataPtr->link.WorldPose(_ecm)->Z() < this->dataPtr->surfaceLevel) {
        this->dataPtr->link.AddWorldWrench(_ecm, wrench.force, wrench.torque);
    }

    // Publish the messages
    msgs::Float speedMsg;
    speedMsg.set_data(speed);
    this->dataPtr->currentSpeedPub.Publish(speedMsg);

    msgs::Float directionMsg;
    directionMsg.set_data(direction);
    this->dataPtr->directionPub.Publish(directionMsg);

    msgs::Vector3d forceMsg;
    forceMsg.set_x(wrench.force.X());
    forceMsg.set_y(wrench.force.Y());
    forceMsg.set_z(wrench.force.Z());
    this->dataPtr->forcePub.Publish(forceMsg);

    msgs::Vector3d torqueMsg;
    torqueMsg.set_x(wrench.torque.X());
    torqueMsg.set_y(wrench.torque.Y());
    torqueMsg.set_z(wrench.torque.Z());
    this->dataPtr->torquePub.Publish(torqueMsg);
}

GZ_ADD_PLUGIN(water_current::WaterCurrent,
              sim::System,
              WaterCurrent::ISystemConfigure,
              WaterCurrent::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(water_current::WaterCurrent,
                    "water_current::WaterCurrent")