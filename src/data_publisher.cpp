#include <data_publisher/data_publisher.hpp>
#include <memory>
#include <stdexcept>
#include <string>

#include "euroc_publisher.hpp"

namespace simulation {

std::unique_ptr<IDataPublisher> CreateDataPublisher(DatasetType type) {
  std::unique_ptr<IDataPublisher> ret;
  switch (type) {
    case DatasetType::EUROC:
      ret = std::make_unique<EurocPublisher>();
      break;
    default:
      throw std::runtime_error("Not yet implemented");
  }
  return ret;
}

}  // namespace simulation