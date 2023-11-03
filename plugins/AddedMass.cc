#include <gz/plugin/Register.hh>
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/transport/Node.hh"
#include <gz/msgs/float.pb.h>
#include <math.h>
#include <Eigen/Eigen>

#include "AddedMass.hh"

using namespace added_mass;
using namespace gz;
using namespace sim;
using namespace systems;

class added_mass::AddedMass::Implementation {

public: sim::Link link{sim::kNullEntity};

public: transport::Node node;

public: transport::Node::Publisher publisher;

public: std::string forceTopic = "/added_mass/force";

public: Eigen::VectorXd prevState;

};

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
    this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
    this->dataPtr->link.EnableAccelerationChecks(_ecm, true);

//    double updateRate = this->dataPtr->updateRate;
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(30);

    this->dataPtr->publisher =
        this->dataPtr->node.Advertise<msgs::Float>(this->dataPtr->forceTopic, opts);

    this->dataPtr->prevState = Eigen::VectorXd::Zero(3);
}

void AddedMass::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
    if (_info.paused)
        return;

    float volume = 2.1549607138964606;
    float fluidMass = volume * 1025;
//    auto acceleration = this->dataPtr->link.WorldLinearAcceleration(_ecm);

    auto dt = static_cast<double>(_info.dt.count())/1e9;
    Eigen::VectorXd stateDot = Eigen::VectorXd(3);
    Eigen::VectorXd state    = Eigen::VectorXd(3);

    state(0) = this->dataPtr->link.WorldLinearVelocity(_ecm)->X();
    state(1) = this->dataPtr->link.WorldLinearVelocity(_ecm)->Y();
    state(2) = this->dataPtr->link.WorldLinearVelocity(_ecm)->Z();


    stateDot = (state - this->dataPtr->prevState)/dt;
    this->dataPtr->prevState = state;

    const Eigen::VectorXd force = - stateDot * fluidMass;

    msgs::Float forceMsg;
//    forceMsg.set_data();
//    this->dataPtr->publisher.Publish(forceMsg);
//    std::string name = this->dataPtr->link.Name(_ecm).value();

    gzmsg << "Acceleration: " << stateDot << std::endl;


    math::Vector3d totalForce(-force(0),  -force(1), 0);
    float mag = sqrt(pow(totalForce.X(), 2) + pow(totalForce.Y(), 2));
    gzmsg << "Mag: " << mag << std::endl;
    if (mag < 1000000)
        this->dataPtr->link.AddWorldForce(_ecm, totalForce);


}


GZ_ADD_PLUGIN(added_mass::AddedMass,
              sim::System,
              AddedMass::ISystemConfigure,
              AddedMass::ISystemPreUpdate)