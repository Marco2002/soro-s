#pragma once

#include <vector>

#include "soro/utls/coroutine/generator.h"

#include "soro/si/units.h"

#include "soro/infrastructure/graph/element.h"

#include "utl/pipes/all.h"
#include "utl/pipes/iterable.h"
#include "utl/pipes/remove_if.h"

namespace soro::infra {

/**
 * section used to determine the exclusion of interlocking routes
 */
struct exclusion_section {
  using id = uint32_t;
  using ids = soro::vector<id>;

  element_id lower_end;
  element_id upper_end;
  id id_;

  exclusion_section(element_id first, element_id second, id id) {
        if(first < second) {
          lower_end = first;
          upper_end = second;
        } else {
          upper_end = first;
          lower_end = second;
        }
        id_ = id;
  }

  bool contains_end(element_id end) const {
    return lower_end == end || upper_end == end;
  }
};

}  // namespace soro::infra
