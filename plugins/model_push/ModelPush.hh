#ifndef MODEL_PLUGIN_MODELPUSH_HH_
#define MODEL_PLUGIN_MODELPUSH_HH_

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace model_push
{
	class ModelPush
        : public gz::sim::System,
          public gz::sim::ISystemConfigure,
          public gz::sim::ISystemPreUpdate
    {
        public: ModelPush();
        public: ~ModelPush() override = default;

         // Documentation inherited.
        public: void Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr) override;

         // Documentation inherited.
        public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm) override;
//        GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
    };
}

#endif