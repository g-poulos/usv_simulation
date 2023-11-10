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

public: transport::Node::Publisher forcePublisher;

public: std::string forceTopic = "/added_mass/force";

public: transport::Node::Publisher torquePublisher;

public: std::string torqueTopic = "/added_mass/torque";

public: Eigen::VectorXd prevState;
    
public: float coefficient;

public: float density;

public: float volume;

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
    gzmsg << "[AddedMass]: Coefficient " << this->dataPtr->coefficient << std::endl;

    if (_sdf->HasElement("density"))
        this->dataPtr->density = _sdf->Get<float>("density");
    gzmsg << "[AddedMass]: Density " << this->dataPtr->density << std::endl;

    if (_sdf->HasElement("sub_volume"))
        this->dataPtr->volume = _sdf->Get<float>("sub_volume");
    gzmsg << "[AddedMass]: Submerged Volume " << this->dataPtr->volume << std::endl;

    // Set up publishers
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(30);

    this->dataPtr->forcePublisher =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->forceTopic, opts);

    this->dataPtr->torquePublisher =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->torqueTopic, opts);

    this->dataPtr->prevState = Eigen::VectorXd::Zero(6);

    AddWorldPose(this->dataPtr->linkEntity, _ecm);
    AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);
    AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
}

void AddedMass::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
    if (_info.paused)
        return;

    // Compute displaced fluid mass
    float fluidMass = this->dataPtr->volume * this->dataPtr->density;

    auto dt = static_cast<double>(_info.dt.count())/1e9;
    Eigen::VectorXd stateDot = Eigen::VectorXd(6);
    Eigen::VectorXd state    = Eigen::VectorXd(6);

    auto linearVelocity =
        _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity);
    auto rotationalVelocity =
        _ecm.Component<components::WorldAngularVelocity>(this->dataPtr->linkEntity);

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

    // Compute the linear and angular acceleration
    stateDot = ((state - this->dataPtr->prevState)/dt);
    this->dataPtr->prevState = state;

    // Added Mass wrench
    const Eigen::VectorXd wrench = fluidMass * stateDot * this->dataPtr->coefficient;

    // We focus on the planar motion of the body so forces acting
    // along the z axis and about the x and y axis are zero (Heave, Pitch, Roll)
    math::Vector3d force(-wrench(0), -wrench(1), 0);
    math::Vector3d torque(0, 0, -wrench(5));

    if (force.IsFinite()) {
        this->dataPtr->link.AddWorldWrench(_ecm,
                                           pose->Rot() * force,
                                           pose->Rot() * torque);
    }

    msgs::Vector3d forceMsg;
    forceMsg.set_x(force.X());
    forceMsg.set_y(force.Y());
    forceMsg.set_z(force.Z());
    this->dataPtr->forcePublisher.Publish(forceMsg);

    msgs::Vector3d torqueMsg;
    torqueMsg.set_x(torque.X());
    torqueMsg.set_y(torque.Y());
    torqueMsg.set_z(torque.Z());
    this->dataPtr->torquePublisher.Publish(torqueMsg);

    // DEBUG
//    gzmsg << "Linear Acceleration: " << stateDot(0) << " " <<
//                                        stateDot(1) << " " <<
//                                        stateDot(2) << " " << std::endl;
//    gzmsg << "Angular Acceleration: " << stateDot(3) << " " <<
//                                         stateDot(4) << " " <<
//                                         stateDot(5) << " " << std::endl;
//    gzmsg << force << std::endl;
//    gzmsg << torque << std::endl;

}


GZ_ADD_PLUGIN(added_mass::AddedMass,
              sim::System,
              AddedMass::ISystemConfigure,
              AddedMass::ISystemPreUpdate)