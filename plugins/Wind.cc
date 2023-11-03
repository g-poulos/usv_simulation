#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
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
public: sim::Link link{sim::kNullEntity};

public: transport::Node node;

public: transport::Node::Publisher windSpeedPub;

public: transport::Node::Publisher azimuthPub;

public: std::string magnitudeTopic = "/wind/speed";

public: std::string azimuthTopic = "/wind/azimuth";

public: IntegratedWhiteNoise azimuthDistr;

public: IntegratedWhiteNoise speedDistr;

public: float minSpeed = 0;

public: float maxSpeed = 0;

public: float speedstddev = 0;

public: float minAzimuth = 0;

public: float maxAzimuth = 0;

public: float azimuthstddev = 0;

public: float elevation = 90;

public: int updateRate = 10;
};

float fluidDensity = 1.225;

surfaceData *currentSurfaceData;


Wind::Wind()
    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

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

    if (!this->dataPtr->link.Valid(_ecm)) {
        gzerr << "Could not find link named [" << linkName
              << "] in model" << std::endl;
        return;
    }

    if (_sdf->HasElement("wind")) {
        sdf::ElementPtr currentObjSDF = _sdf->GetElementImpl("wind");

        gzmsg << "Water Current Parameters" << std::endl;

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
            gzmsg << "Speed: " << " Min " << this->dataPtr->minSpeed
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
            gzmsg << "Direction: " << " Min " << this->dataPtr->minAzimuth
                  << ", Max " << this->dataPtr->maxAzimuth
                  << ", stddev " << this->dataPtr->azimuthstddev << std::endl;
        }
    }

    if (_sdf->HasElement("density")) {
        fluidDensity = _sdf->Get<float>("density");
    }

    if (_sdf->HasElement("update_rate")) {
        this->dataPtr->updateRate = _sdf->Get<float>("update_rate");
    }

    // Set up the publisher
    double updateRate = this->dataPtr->updateRate;
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->windSpeedPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->azimuthPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->azimuthTopic, opts);

    // Set up the noise distribution
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

    // Compute surface area of application
    std::string currentSurfaceAreaFile = getModelFile(_ecm, "wind_surface.txt");

    currentSurfaceData = readAreaFile(currentSurfaceAreaFile);
    for (int i =0; i<256; i++){
        gzmsg << currentSurfaceData->angle_p[i] << std::endl;
    }
}

void Wind::PreUpdate(const sim::UpdateInfo &_info,
                             sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

    double speed = this->dataPtr->speedDistr.getValue();
    double azimuth = this->dataPtr->azimuthDistr.getValue();

    // TODO: Remove elevation
//    double elevation = this->dataPtr->elevation + this->dataPtr->speedDistr.getNoise();

    math::Vector3d windForce = calculateForce(_ecm, this->dataPtr->link,
                                              speed,
                                              azimuth,
                                              currentSurfaceData,
                                              fluidDensity);
    this->dataPtr->link.AddWorldForce(_ecm, windForce);


    msgs::Float forceMsg;
    forceMsg.set_data(speed);
    this->dataPtr->windSpeedPub.Publish(forceMsg);

    msgs::Float azimuthMsg;
    azimuthMsg.set_data(azimuth);
    this->dataPtr->azimuthPub.Publish(azimuthMsg);
}

GZ_ADD_PLUGIN(wind::Wind,
    sim::System,
    Wind::ISystemConfigure,
    Wind::ISystemPreUpdate)
