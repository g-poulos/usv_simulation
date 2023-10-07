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
#include <gz/physics.hh>

#include "ModelPush.hh"
#include "utility.hh"

using namespace model_push;

class model_push::ModelPush::Implementation
{
public: gz::sim::Link link{gz::sim::kNullEntity};

public: gz::transport::Node node;

public: gz::transport::Node::Publisher forcePub;

public: std::string topicForce = "/modelPushInfo";

public: GaussianNoise distr;
};

ModelPush::ModelPush()
    : System(), dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

void ModelPush::Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr)
{
    // Parse required elements.
    if (!_sdf->HasElement("link_name"))
    {
        gzerr << "No <link_name> specified" << std::endl;
        return;
    }
    gz::sim::Model model(_entity);
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = gz::sim::Link(model.LinkByName(_ecm, linkName));

    if (!this->dataPtr->link.Valid(_ecm))
    {
        gzerr << "Could not find link named [" << linkName
              << "] in model" << std::endl;
        return;
    }

//    gz::math::Vector3 force = (this->dataPtr->link)->GetExternalForces();
    // Set up publisher

    double updateRate = 10;
    gz::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(updateRate);

    this->dataPtr->forcePub =
        this->dataPtr->node.Advertise<gz::msgs::Float>(this->dataPtr->topicForce, opts);

    // Set up the noise distribution
    this->dataPtr->distr = GaussianNoise(0, 10);
}

void ModelPush::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
    // Don't add force if the simulation is paused
    if (_info.paused) {
        return;
    }

    float x_force = 1000 + this->dataPtr->distr.getNoise();
    this->dataPtr->link.AddWorldForce(_ecm, gz::math::Vector3d(x_force, 0, 0));

    gz::msgs::Float forceMsg;
    forceMsg.set_data(x_force);
    this->dataPtr->forcePub.Publish(forceMsg);
}

GZ_ADD_PLUGIN(model_push::ModelPush,
              sim::System,
              ModelPush::ISystemConfigure,
              ModelPush::ISystemPreUpdate)





