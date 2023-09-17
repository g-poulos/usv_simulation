
#include <string>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>
#include "ModelPush.hh"

using namespace model_push;

//class model_push::ModelPush::Implementation
//{
//    public: std::string msg;
//};

//ModelPush::ModelPush()
//    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
//{
//}

//void ModelPush::Configure(const gz::sim::Entity &_entity,
//                          const std::shared_ptr<const sdf::Element> &_sdf,
//                          gz::sim::EntityComponentManager &_ecm,
//                          gz::sim::EventManager &_eventMgr)
//{
//
//}

void ModelPush::PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm)
{

}

GZ_ADD_PLUGIN(model_push::ModelPush,
              sim::System,
//              ModelPush::ISystemConfigure,
              ModelPush::ISystemPreUpdate)

//GZ_ADD_PLUGIN_ALIAS(model_push::ModelPush,
//                    "model_push::ModelPush")
//



