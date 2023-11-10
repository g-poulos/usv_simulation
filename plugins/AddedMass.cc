#include <gz/plugin/Register.hh>
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Environment.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/transport/Node.hh"
#include <gz/msgs/float.pb.h>
#include <math.h>
#include <Eigen/Eigen>
#include <gz/msgs/details/vector3d.pb.h>

#include "AddedMass.hh"

using namespace added_mass;
using namespace gz;
using namespace sim;
using namespace systems;

class added_mass::AddedMass::Implementation {

public: sim::Link link{sim::kNullEntity};

public: Entity linkEntity;

public: transport::Node node;

public: transport::Node::Publisher publisher;

public: std::string forceTopic = "/added_mass/force";

public: Eigen::VectorXd prevState;

public: Eigen::MatrixXd Ma;
    
public: float coefficient;

};

/////////////////////////////////////////////////
void AddWorldPose(
    const gz::sim::Entity &_entity,
    gz::sim::EntityComponentManager &_ecm)
{
    if (!_ecm.Component<gz::sim::components::WorldPose>(_entity))
    {
        _ecm.CreateComponent(_entity, gz::sim::components::WorldPose());
    }
}

/////////////////////////////////////////////////
void AddWorldLinearVelocity(
    const gz::sim::Entity &_entity,
    gz::sim::EntityComponentManager &_ecm)
{
    if (!_ecm.Component<gz::sim::components::WorldLinearVelocity>(
        _entity))
    {
        _ecm.CreateComponent(_entity,
                             gz::sim::components::WorldLinearVelocity());
    }
}

/////////////////////////////////////////////////
void AddAngularVelocityComponent(
    const gz::sim::Entity &_entity,
    gz::sim::EntityComponentManager &_ecm)
{
    if (!_ecm.Component<gz::sim::components::AngularVelocity>(_entity))
    {
        _ecm.CreateComponent(_entity,
                             gz::sim::components::AngularVelocity());
    }

    // Create an angular velocity component if one is not present.
    if (!_ecm.Component<gz::sim::components::WorldAngularVelocity>(
        _entity))
    {
        _ecm.CreateComponent(_entity,
                             gz::sim::components::WorldAngularVelocity());
    }
}

/////////////////////////////////////////////////
math::Vector3d toGZVec(std::optional<math::Vector3<double>> vec) {
    return math::Vector3d(vec->X(), vec->Y(), vec->Z());
}

AddedMass::AddedMass()
    : System(), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}


void AddedMass::Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) {

    sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);
    this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
    this->dataPtr->link.EnableAccelerationChecks(_ecm, true);

    if (_sdf->HasElement("coef"))
        this->dataPtr->coefficient = _sdf->Get<float>("coef");
    gzmsg << "[AddedMass]: Coefficient: " << this->dataPtr->coefficient << std::endl;

    // Set up publisher
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(30);

    this->dataPtr->publisher =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->forceTopic, opts);

    this->dataPtr->prevState = Eigen::VectorXd::Zero(6);

    AddWorldPose(this->dataPtr->linkEntity, _ecm);
    AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);
    AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
}

void AddedMass::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
    if (_info.paused)
        return;

//    float volume = 2.1549607138964606 * 0.4;
    float volume = 0.39;
    float fluidMass = volume * 1025;

    auto dt = static_cast<double>(_info.dt.count())/1e9;
    Eigen::VectorXd stateDot = Eigen::VectorXd(6);
    Eigen::VectorXd state    = Eigen::VectorXd(6);

    auto linearVelocity = _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity);
    auto rotationalVelocity = _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->linkEntity);

    // Transform world vectors to local vectors using model rotation
    auto pose = this->dataPtr->link.WorldPose(_ecm);
    auto localLinearVelocity = pose->Rot().Inverse() * linearVelocity->Data();
    auto localRotationalVelocity = pose->Rot().Inverse() * rotationalVelocity->Data();

    state(0) = localLinearVelocity.X();
    state(1) = localLinearVelocity.Y();
    state(2) = localLinearVelocity.Z();

    state(3) = localRotationalVelocity.X();
    state(4) = localRotationalVelocity.Y();
    state(5) = localRotationalVelocity.Z();


    stateDot = ((state - this->dataPtr->prevState)/dt);
    this->dataPtr->prevState = state;

//    const Eigen::VectorXd force = this->dataPtr->Ma * stateDot * 0.8;
    const Eigen::VectorXd force = fluidMass * stateDot * this->dataPtr->coefficient;
    math::Vector3d totalForce(-force(0),  -force(1), 0);
    math::Vector3d totalTorque(0,  0, -force(5));

    if (totalForce.IsFinite()) {
        this->dataPtr->link.AddWorldForce(_ecm, pose->Rot() * totalForce);
        this->dataPtr->link.AddWorldWrench(_ecm,
                                           math::Vector3d (0, 0, 0),
                                           pose->Rot() * totalTorque);
//        this->dataPtr->link.AddWorldWrench(_ecm,
//                                           pose->Rot() * totalForce,
//                                           pose->Rot() * totalTorque);
    }

    msgs::Vector3d forceMsg;
    forceMsg.set_x(totalForce.X());
    forceMsg.set_y(totalForce.Y());
    forceMsg.set_z(totalForce.Z());
    this->dataPtr->publisher.Publish(forceMsg);

    // DEBUG
    gzmsg << "Linear Acceleration: " << stateDot(0) << " " <<
                                        stateDot(1) << " " <<
                                        stateDot(2) << " " << std::endl;
    gzmsg << "Angular Acceleration: " << stateDot(3) << " " <<
                                         stateDot(4) << " " <<
                                         stateDot(5) << " " << std::endl;
//    gzmsg << totalForce << std::endl;

}


GZ_ADD_PLUGIN(added_mass::AddedMass,
              sim::System,
              AddedMass::ISystemConfigure,
              AddedMass::ISystemPreUpdate)