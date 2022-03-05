#include <data_publisher/data_publisher.hpp>
#include <memory>
#include <stdexcept>
#include <string>

#include "euroc_publisher.hpp"

namespace simulation {

std::unique_ptr<IDataPublisher> CreateDataPublisher(DatasetType type,
                                                    std::string path) {
  switch (type) {
    case DatasetType::EUROC:
      return std::make_unique<EurocPublisher>(path);
      break;
    default:
      throw std::runtime_error("Not yet implemented");
  }
}

}  // namespace simulation