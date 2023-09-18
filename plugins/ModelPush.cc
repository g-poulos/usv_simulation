#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
#include "gz/sim/Util.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"

#include "ModelPush.hh"

using namespace model_push;

class model_push::ModelPush::Implementation
{
    public: gz::sim::Link link{gz::sim::kNullEntity};
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
}

void ModelPush::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{

    this->dataPtr->link.AddWorldForce(_ecm, gz::math::Vector3d(1000, 0, 0));
}

GZ_ADD_PLUGIN(model_push::ModelPush,
              sim::System,
              ModelPush::ISystemConfigure,
              ModelPush::ISystemPreUpdate)





