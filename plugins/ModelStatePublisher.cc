#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/details/vector3d.pb.h>

#include "ModelStatePublisher.hh"

using namespace model_state_publisher;
using namespace gz;

class model_state_publisher::ModelStatePublisher::Implementation {

    /// \brief The link
public: sim::Link link{sim::kNullEntity};

    /// \brief Transport node
public: transport::Node node;

    /// \brief Transport node publisher for the link linear acceleration
public: transport::Node::Publisher linearAccPublisher;

    /// \brief Topic where the linear acceleration is published
public: std::string linearAccTopic = "/acceleration/linear";

    /// \brief Transport node publisher for the link angular acceleration
public: transport::Node::Publisher angularAccPublisher;

    /// \brief Topic where the angular acceleration is published
public: std::string angularAccTopic = "/acceleration/angular";
};

ModelStatePublisher::ModelStatePublisher()
    : System(), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

/////////////////////////////////////////////////
void ModelStatePublisher::Configure(const sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                          sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr) {
    // Parse required elements
    if (!_sdf->HasElement("link_name")) {
        gzerr << "No <link_name> specified" << std::endl;
        return;
    }
    sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = sim::Link(model.LinkByName(_ecm, linkName));
    this->dataPtr->link.EnableAccelerationChecks(_ecm, true);

    this->dataPtr->linearAccTopic = "model/" + model.Name(_ecm) + "/acceleration/linear";
    this->dataPtr->angularAccTopic = "model/" + model.Name(_ecm) + "/acceleration/angular";

    // Set up publishers
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(10);

    this->dataPtr->linearAccPublisher =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->linearAccTopic, opts);

    this->dataPtr->angularAccPublisher =
        this->dataPtr->node.Advertise<msgs::Vector3d>(this->dataPtr->angularAccTopic, opts);
}

/////////////////////////////////////////////////
void ModelStatePublisher::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {
    if (_info.paused)
        return;
    auto linearAcceleration = this->dataPtr->link.WorldLinearAcceleration(_ecm);
    auto angularAcceleration = this->dataPtr->link.WorldAngularAcceleration(_ecm);

    msgs::Vector3d linearAccMsg;
    linearAccMsg.set_x(linearAcceleration->X());
    linearAccMsg.set_y(linearAcceleration->Y());
    linearAccMsg.set_z(linearAcceleration->Z());
    this->dataPtr->linearAccPublisher.Publish(linearAccMsg);

    msgs::Vector3d angularAccMsg;
    angularAccMsg.set_x(angularAcceleration->X());
    angularAccMsg.set_y(angularAcceleration->Y());
    angularAccMsg.set_z(angularAcceleration->Z());
    this->dataPtr->angularAccPublisher.Publish(angularAccMsg);
}

GZ_ADD_PLUGIN(model_state_publisher::ModelStatePublisher,
              sim::System,
              ModelStatePublisher::ISystemConfigure,
              ModelStatePublisher::ISystemPostUpdate)