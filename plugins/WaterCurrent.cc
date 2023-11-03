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
#include <gz/sim/components/Volume.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/common/MeshManager.hh>
#include <filesystem>


#include "WaterCurrent.hh"
#include "utility.hh"

using namespace water_current;
using namespace std;

class water_current::WaterCurrent::Implementation
{
public: sim::Link link{sim::kNullEntity};

public: transport::Node node;

public: transport::Node::Publisher currentSpeedPub;

public: transport::Node::Publisher azimuthPub;

public: std::string magnitudeTopic = "/waterCurrent/speed";

public: std::string azimuthTopic = "/waterCurrent/azimuth";

public: IntegratedWhiteNoise azimuthDistr;

public: IntegratedWhiteNoise speedDistr;

public: float waterCurrentMinSpeed = 0;

public: float waterCurrentMaxSpeed = 0;

public: float speedstddev = 0;

public: float waterCurrentMaxAzimuth = 0;

public: float waterCurrentMinAzimuth = 0;

public: float azimuthstddev = 0;

public: float waterCurrentElevation = 90;

public: int updateRate = 10;
};

float resCoefficient = 1;

float fluidDensity = 1000;

surfaceData *currentSurfaceData;


float getSurface(sim::Link link, sim::EntityComponentManager &_ecm, float azimuth, surfaceData* surfaceData) {
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

    int closest_i = findClosest(surfaceData->angle_p, surfaceData->size, relative_angle);

    // DEBUG
//    gzmsg << "Boat Yaw " << yaw <<
//             " Total: " << relative_angle <<
//             " A_matrix: " << angle_p[closest_i] <<
//             " Area: " << area_p[closest_i] <<std::endl;
    return surfaceData->area_p[closest_i];
}

math::Vector3d speedToForce(sim::EntityComponentManager &_ecm,
                            sim::Link link,
                            float currentSpeed,
                            float direction,
                            surfaceData* surfaceData) {

    math::Vector3d linkLinearVel = toGZVec(link.WorldLinearVelocity(_ecm));
    math::Vector3d wcurrentLinearVel = sphericalToVector(currentSpeed, 90, direction);
    math::Vector3d relativeVel = wcurrentLinearVel.operator-(linkLinearVel);

    float surface = getSurface(link, _ecm, direction, surfaceData);

    math::Vector3d wcurrentVector = 0.5 * fluidDensity * resCoefficient * relativeVel * surface;

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
    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

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

    if (!this->dataPtr->link.Valid(_ecm)) {
        gzerr << "Could not find link named [" << linkName
              << "] in model" << std::endl;
        return;
    }

    if (_sdf->HasElement("current")) {
        sdf::ElementPtr currentObjSDF = _sdf->GetElementImpl("current");

        gzmsg << "Water Current Parameters" << std::endl;

        if (currentObjSDF->HasElement("speed")) {
            sdf::ElementPtr speedObjSDF = currentObjSDF->GetElementImpl("speed");

            if (speedObjSDF->HasElement("min")) {
                this->dataPtr->waterCurrentMinSpeed = speedObjSDF->Get<float>("min");
            }
            if (speedObjSDF->HasElement("max")) {
                this->dataPtr->waterCurrentMaxSpeed = speedObjSDF->Get<float>("max");
            }
            if (speedObjSDF->HasElement("stddev")) {
                this->dataPtr->speedstddev = speedObjSDF->Get<float>("stddev");
            }
            gzmsg << "Speed: " << " Min " << this->dataPtr->waterCurrentMinSpeed
                               << ", Max " << this->dataPtr->waterCurrentMaxSpeed
                               << ", stddev " << this->dataPtr->speedstddev << std::endl;
        }

        if (currentObjSDF->HasElement("direction")) {
            sdf::ElementPtr directionObjSDF = currentObjSDF->GetElementImpl("direction");

            if (directionObjSDF->HasElement("min")) {
                this->dataPtr->waterCurrentMinAzimuth = directionObjSDF->Get<float>("min");
            }
            if (directionObjSDF->HasElement("max")) {
                this->dataPtr->waterCurrentMaxAzimuth = directionObjSDF->Get<float>("max");
            }
            if (directionObjSDF->HasElement("stddev")) {
                this->dataPtr->azimuthstddev = directionObjSDF->Get<float>("stddev");
            }
            gzmsg << "Direction: " << " Min " << this->dataPtr->waterCurrentMinAzimuth
                                   << ", Max " << this->dataPtr->waterCurrentMaxAzimuth
                                   << ", stddev " << this->dataPtr->azimuthstddev << std::endl;
        }
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
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->currentSpeedPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->magnitudeTopic, opts);

    this->dataPtr->azimuthPub =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->azimuthTopic, opts);

    // Set up the noise distribution
    this->dataPtr->speedDistr = IntegratedWhiteNoise(0,
                                                     this->dataPtr->speedstddev,
                                                     this->dataPtr->waterCurrentMinSpeed,
                                                     this->dataPtr->waterCurrentMaxSpeed,
                                                     0.01);
    this->dataPtr->azimuthDistr = IntegratedWhiteNoise(0,
                                                       this->dataPtr->azimuthstddev,
                                                       this->dataPtr->waterCurrentMinAzimuth,
                                                       this->dataPtr->waterCurrentMaxAzimuth,
                                                       0.01);

    // Compute surface area of application
    std::string currentSurfaceAreaFile = getModelFile(_ecm, "current_surface.txt");

    currentSurfaceData = readAreaFile(currentSurfaceAreaFile);
    for (int i =0; i<256; i++){
        gzmsg << currentSurfaceData->angle_p[i] << std::endl;
    }
}

void WaterCurrent::PreUpdate(const sim::UpdateInfo &_info,
                             sim::EntityComponentManager &_ecm)
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
        math::Vector3d current = speedToForce(_ecm, this->dataPtr->link, speed, azimuth, currentSurfaceData);
        this->dataPtr->link.AddWorldForce(_ecm, current);
    }

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





