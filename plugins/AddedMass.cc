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

};

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

    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(30);

    this->dataPtr->publisher =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->forceTopic, opts);

    this->dataPtr->prevState = Eigen::VectorXd::Zero(3);
    this->dataPtr->Ma = Eigen::MatrixXd::Zero(3, 3);
    this->dataPtr->Ma << 532, 0, 0,
                         0, 532, 0,
                         0, 0, 1917;


    AddWorldPose(this->dataPtr->linkEntity, _ecm);
    AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);
}

void AddedMass::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
    if (_info.paused)
        return;

//    float volume = 2.1549607138964606 * 0.4;
    float volume = 0.43;
    float fluidMass = volume * 1025;
//    auto acceleration = this->dataPtr->link.WorldLinearAcceleration(_ecm);

    auto dt = static_cast<double>(_info.dt.count())/1e9;
    Eigen::VectorXd stateDot = Eigen::VectorXd(3);
    Eigen::VectorXd state    = Eigen::VectorXd(3);

    auto linearVelocity = _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity);
    auto pose = this->dataPtr->link.WorldPose(_ecm);
    auto localLinearVelocity = pose->Rot().Inverse() * linearVelocity->Data();

    state(0) = localLinearVelocity.X();
    state(1) = localLinearVelocity.Y();
    state(2) = localLinearVelocity.Z();


    stateDot = ((state - this->dataPtr->prevState)/dt);
    this->dataPtr->prevState = state;

//    const Eigen::VectorXd force = this->dataPtr->Ma * stateDot * 0.8;
    const Eigen::VectorXd force = fluidMass * stateDot * 0.8;
    math::Vector3d totalForce(-force(0),  -force(1), 0);

    float mag = sqrt(pow(totalForce.X(), 2) + pow(totalForce.Y(), 2));

    gzmsg << "Acceleration: " << stateDot << std::endl;
    gzmsg << "Mag: " << mag << std::endl;
    gzmsg << totalForce << std::endl;
    if (mag < 100000) {
        this->dataPtr->link.AddWorldForce(_ecm, pose->Rot() * totalForce);
    }

    math::Vector3d acc(stateDot(0), stateDot(1), stateDot(2));
    msgs::Vector3d forceMsg;
    forceMsg.set_x(totalForce.X());
    forceMsg.set_y(totalForce.Y());
    forceMsg.set_z(totalForce.Z());
    this->dataPtr->publisher.Publish(forceMsg);

}


GZ_ADD_PLUGIN(added_mass::AddedMass,
              sim::System,
              AddedMass::ISystemConfigure,
              AddedMass::ISystemPreUpdate)