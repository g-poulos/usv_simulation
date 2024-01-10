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

#include "Wind.hh"
#include "utility.hh"

using namespace wind;
using namespace std;

class wind::Wind::Implementation
{
    /// \brief the link affected by the wind
public: sim::Link link{sim::kNullEntity};

    /// \brief Transport node
public: transport::Node node;

    /// \brief Transport node publisher for the wind speed
public: transport::Node::Publisher windSpeedPub;

    /// \brief Transport node publisher for the wind direction
public: transport::Node::Publisher directionPub;

    /// \brief Topic where the wind speed is published
public: std::string magnitudeTopic = "/wind/speed";

    /// \brief Topic where the wind direction is published
public: std::string directionTopic = "/wind/direction";

    /// \brief Integrated white noise distribution to generate the wind's direction
public: IntegratedWhiteNoise directionDistr;

    /// \brief Integrated white noise distribution to generate the wind's speed
public: IntegratedWhiteNoise speedDistr;

    /// \brief Wind minimum speed
public: float minSpeed = 0;

    /// \brief Wind maximum speed
public: float maxSpeed = 0;

    /// \brief Wind initial speed
public: float initSpeed = 0;

    /// \brief Standard deviation for the wind's speed
public: float speedstddev = 0;

    /// \brief Wind minimum direction
public: float minDirection = 0;

    /// \brief Wind maximum direction
public: float maxDirection = 0;

    /// \brief Wind initial direction
public: float initDirection = 0;

    /// \brief Standard deviation for the wind's direction
public: float directionstddev = 0;

    /// \brief Time passed between iterations (overwritten in preUpdate)
public: double dt = 0.01;

    /// \brief WInd elevation (not used)
public: float elevation = 90;

    /// \brief Update rate buffer for wind speed and direction
public: int updateRate = 10;

    /// \brief Air density
public: float airDensity = 1.225;

    /// \brief Resistance coefficient of the surface
public: float resCoefficient = 1;

    /// \brief Structure with the surface area information
public: wrenchFileData *windSurfaceData;

    /// \brief Name of file the with the surface area information
public: std::string tableFileName = "wind_table.csv";
};

//////////////////////////////////////////////////
Wind::Wind()
    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void Wind::Configure(const sim::Entity &_entity,
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

    if (_sdf->HasElement("wind")) {
        sdf::ElementPtr windObjSDF = _sdf->GetElementImpl("wind");

        if (windObjSDF->HasElement("speed")) {
            sdf::ElementPtr speedObjSDF = windObjSDF->GetElementImpl("speed");

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
            gzmsg << "[Wind] Speed: " << " Min " << this->dataPtr->minSpeed
                                      << ", Max " << this->dataPtr->maxSpeed
                                      << ", Initial " << this->dataPtr->initSpeed
                                      << ", stddev " << this->dataPtr->speedstddev << std::endl;
        }

        if (windObjSDF->HasElement("direction")) {
            sdf::ElementPtr directionObjSDF = windObjSDF->GetElementImpl("direction");

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
            gzmsg << "[Wind] Direction: " << " Min " << this->dataPtr->minDirection
                  << ", Max " << this->dataPtr->maxDirection
                  << ", Initial " << this->dataPtr->initDirection
                  << ", stddev " << this->dataPtr->directionstddev << std::endl;
        }
    }

    if (_sdf->HasElement("density"))
        this->dataPtr->airDensity = _sdf->Get<float>("density");
    gzmsg << "[Wind] Density: " << this->dataPtr->airDensity << std::endl;

    if (_sdf->HasElement("res_coef"))
        this->dataPtr->resCoefficient = _sdf->Get<float>("res_coef");
    gzmsg << "[Wind] Resistance Coefficient: " << this->dataPtr->resCoefficient << std::endl;

    if (_sdf->HasElement("update_rate"))
        this->dataPtr->updateRate = _sdf->Get<float>("update_rate");
    gzmsg << "[Wind] Update rate: " << this->dataPtr->updateRate << std::endl;

    if (_sdf->HasElement("table_file"))
        this->dataPtr->tableFileName = _sdf->Get<std::string>("table_file");
    gzmsg << "[Wind] Table File: " << this->dataPtr->tableFileName << std::endl;

    // Set up the publishers
    double updateRate = this->dataPtr->updateRate;
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->windSpeedPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->directionPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->directionTopic, opts);

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
    std::string windTableFile = getModelFile(_ecm, this->dataPtr->tableFileName);
    this->dataPtr->windSurfaceData = read_csv(windTableFile);
}

//////////////////////////////////////////////////
void Wind::PreUpdate(const sim::UpdateInfo &_info,
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
                                        this->dataPtr->windSurfaceData,
                                        this->dataPtr->airDensity,
                                        this->dataPtr->resCoefficient);

    this->dataPtr->link.AddWorldWrench(_ecm, wrench.force, wrench.torque);

    // Publish the messages
    msgs::Float forceMsg;
    forceMsg.set_data(speed);
    this->dataPtr->windSpeedPub.Publish(forceMsg);

    msgs::Float directionMsg;
    directionMsg.set_data(direction);
    this->dataPtr->directionPub.Publish(directionMsg);
}

GZ_ADD_PLUGIN(wind::Wind,
    sim::System,
    Wind::ISystemConfigure,
    Wind::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(wind::Wind,
                    "wind::Wind")
