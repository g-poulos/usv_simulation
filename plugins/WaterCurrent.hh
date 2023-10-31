#ifndef WATER_CURRENT_HH_
#define WATER_CURRENT_HH_

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

using namespace gz;

namespace water_current
{
    class WaterCurrent
        : public sim::System,
          public sim::ISystemConfigure,
          public sim::ISystemPreUpdate
    {
    public: WaterCurrent();
    public: ~WaterCurrent() override = default;

        // Documentation inherited.
    public: void Configure(const sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           sim::EntityComponentManager &_ecm,
                           sim::EventManager &_eventMgr) override;

        // Documentation inherited.
    public: void PreUpdate(const sim::UpdateInfo &_info,
                           sim::EntityComponentManager &_ecm) override;
        GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
    };
}

#endif