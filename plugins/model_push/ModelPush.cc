#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"

#include "ModelPush.hh"

using namespace model_push;

std::string linkName = "default";
gz::sim::Link modelLink;


ModelPush::ModelPush() {
}

void ModelPush::Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr)
{
    gz::sim::Model model(_entity);
    linkName = _sdf->Get<std::string>("link_name");
    modelLink = gz::sim::Link(model.LinkByName(_ecm, linkName));
}

void ModelPush::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{
    gzmsg << " edw " << linkName << std::endl;
    modelLink.AddWorldForce(_ecm,
                            gz::math::Vector3d(1000, 0, 0));
}

GZ_ADD_PLUGIN(model_push::ModelPush,
              sim::System,
              ModelPush::ISystemConfigure,
              ModelPush::ISystemPreUpdate)





