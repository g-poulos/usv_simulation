#ifndef WIND_HH_
#define WIND_HH_

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

using namespace gz;

namespace wind
{
    class Wind
        : public sim::System,
          public sim::ISystemConfigure,
          public sim::ISystemPreUpdate
    {
    public: Wind();
    public: ~Wind() override = default;

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