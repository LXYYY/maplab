#include "rovioli/online-map-publisher-flow.h"

namespace rovioli {
OnlineMapPublisherFlow::OnlineMapPublisherFlow() {}

void OnlineMapPublisherFlow::attachToMessageFlow(
    message_flow::MessageFlow* flow) {
  CHECK_NOTNULL(flow);
  static constexpr char kSubscriberNodeName[] = "MapPublisherFlow";
  flow->registerSubscriber<message_flow_topics::MAP_UPDATES>(
      kSubscriberNodeName, message_flow::DeliveryOptions(),
      [this](const vio::MapUpdate::ConstPtr& vio_update) {
        CHECK(vio_update != nullptr);
        this->map_publisher_.apply(vio_update);
      });
}
}  // namespace rovioli
