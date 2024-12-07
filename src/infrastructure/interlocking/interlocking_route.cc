#include "soro/infrastructure/interlocking/interlocking_route.h"

#include "utl/concat.h"

#include "soro/utls/std_wrapper/find.h"

#include "soro/infrastructure/path/length.h"

namespace soro::infra {

bool interlocking_route::starts_on_section(infrastructure const& infra) const {
  return this->first_node(infra)->element_->is_section_element();
}

bool interlocking_route::ends_on_section(infrastructure const& infra) const {
  return this->last_node(infra)->element_->is_section_element();
}

bool interlocking_route::valid_end(type const t) {
  return interlocking_route::valid_ends().contains(t);
}

type_set interlocking_route::valid_ends() {
  return type_set{{type::MAIN_SIGNAL, type::HALT, type::BORDER, type::BUMPER,
                   type::TRACK_END}};
}

node::idx interlocking_route::size(infrastructure const& infra) const {
  node::idx result = 0;

  if (this->station_routes_.size() == 1) {
    return this->end_offset_ - this->start_offset_;
  }

  for (auto const sr : this->station_routes_) {
    result += infra->station_routes_[sr]->size();
  }

  result = result - start_offset_ -
           (infra->station_routes_[last_sr_id()]->size() - end_offset_);

  return result;
}

bool interlocking_route::contains(station_route::id const needle_sr,
                                  node::idx const needle_idx) const {

  if (station_routes_.size() == 1) {
    return station_routes_.front() == needle_sr &&
           start_offset_ <= needle_idx && needle_idx < end_offset_;
  }

  if (station_routes_.front() == needle_sr && start_offset_ <= needle_idx) {
    return true;
  }

  if (station_routes_.back() == needle_sr && needle_idx < end_offset_) {
    return true;
  }

  return std::any_of(std::begin(station_routes_) + 1,
                     std::end(station_routes_) - 1,
                     [&](auto&& sr_id) { return sr_id == needle_sr; });
}

station_route::id interlocking_route::first_sr_id() const {
  return this->station_routes_.front();
}

station_route::id interlocking_route::sr_id(sr_offset const sr_offset) const {
  return this->station_routes_[sr_offset];
}

station_route::id interlocking_route::last_sr_id() const {
  return this->station_routes_.back();
}

station_route::ptr interlocking_route::first_sr(
    infrastructure const& infra) const {
  return infra->station_routes_[this->first_sr_id()];
}

station_route::ptr interlocking_route::sr(sr_offset const sr_offset,
                                          infrastructure const& infra) const {
  return infra->station_routes_[this->sr_id(sr_offset)];
}

station_route::ptr interlocking_route::last_sr(
    infrastructure const& infra) const {
  return infra->station_routes_[this->last_sr_id()];
}

node::ptr interlocking_route::first_node(infrastructure const& infra) const {
  return first_sr(infra)->nodes(start_offset_);
}

node::ptr interlocking_route::last_node(infrastructure const& infra) const {
  return last_sr(infra)->nodes(end_offset_ - 1);
}

bool interlocking_route::starts_on_ms(infrastructure const& infra) const {
  return first_node(infra)->is(type::MAIN_SIGNAL);
}

bool interlocking_route::ends_on_ms(infrastructure const& infra) const {
  return last_node(infra)->is(type::MAIN_SIGNAL);
}

utls::it_range<utls::id_it_ptr<station_route>>
interlocking_route::station_routes(infrastructure const& infra) const {
  return utls::make_range(
      utls::id_iterator(std::begin(station_routes_), &infra->station_routes_),
      utls::id_iterator(std::end(station_routes_), &infra->station_routes_));
}

bool interlocking_route::follows(interlocking_route const& other,
                                 infrastructure const& infra) const {
  return this->first_node(infra) == other.last_node(infra);
}

bool interlocking_route::operator==(interlocking_route const& o) const {
  return this->id_ == o.id_;
}

namespace detail {

utls::recursive_generator<route_node> from_to(
    interlocking_route::ptr const ir,
    decltype(ir->station_routes_)::const_iterator from_it, node::idx const from,
    decltype(ir->station_routes_)::const_iterator to_it, node::idx const to,
    infrastructure const& infra) {

  utls::sassert(ir->station_routes_.size() > 1,
                "Called from_to_impl with a single station route in IR {}, "
                "call from_to_single_imple instead",
                ir->id_);
  utls::sassert(from_it != std::end(ir->station_routes_),
                "Station route {} is not part of interlocking route {}, but "
                "got it for iteration.",
                *from_it, ir->id_);
  utls::sassert(to_it != std::end(ir->station_routes_),
                "Station route {} is not part of interlocking route {}, but "
                "got it for iteration.",
                *to_it, ir->id_);
  utls::sassert(std::distance(from_it, to_it) >= 0,
                "To station route is located before from station route in "
                "interlocking route iterator.");
  utls::sassert(*from_it != *to_it,
                "Don't call this with from == to, "
                "since it will yield the wrong nodes.");

  co_yield infra->station_routes_[*from_it]->from(from);
  ++from_it;

  for (; from_it != to_it; ++from_it) {
    co_yield infra->station_routes_[*from_it]->iterate();
  }

  co_yield infra->station_routes_[*to_it]->to(to);
}

}  // namespace detail

utls::recursive_generator<route_node> interlocking_route::from_to(
    station_route::id const from_sr, node::idx const from,
    station_route::id const to_sr, node::idx const to,
    infrastructure const& infra) const {

  if (this->station_routes_.size() == 1) {
    utls::sassert(from_sr == to_sr,
                  "Only one station route in interlocking route {}, but "
                  "while iterating "
                  "got from {} and to {}.",
                  this->id_, from_sr, to_sr);

    co_yield this->first_sr(infra)->from_to(from, to);
  } else {
    auto from_it = utls::find(this->station_routes_, from_sr);
    auto to_it = utls::find(this->station_routes_, to_sr);

    utls::sassert(from_it != std::end(this->station_routes_),
                  "Station route {} is not part of interlocking route {}, but "
                  "got it for iteration.",
                  from_sr, this->id_);
    utls::sassert(to_it != std::end(this->station_routes_),
                  "Station route {} is not part of interlocking route {}, but "
                  "got it for iteration.",
                  to_sr, this->id_);
    utls::sassert(std::distance(from_it, to_it) >= 0,
                  "To station route is located before from station route in "
                  "interlocking route iterator.");

    if (*from_it == *to_it) {
      co_yield infra->station_routes_[*from_it]->from_to(from, to);
    } else {
      co_yield detail::from_to(this, from_it, from, to_it, to, infra);
    }
  }
}

utls::recursive_generator<route_node> interlocking_route::to(
    station_route::id const sr_id, node::idx const to,
    infrastructure const& infra) const {
  co_yield this->from_to(this->station_routes_.front(), start_offset_, sr_id,
                         to, infra);
}

utls::recursive_generator<route_node> interlocking_route::from(
    station_route::id const sr_id, node::idx const from,
    infrastructure const& infra) const {
  co_yield this->from_to(sr_id, from, station_routes_.back(), end_offset_,
                         infra);
}

utls::recursive_generator<route_node> interlocking_route::iterate(
    infrastructure const& infra) const {
  if (station_routes_.size() == 1) {
    co_yield this->first_sr(infra)->from_to(start_offset_, end_offset_);
  } else {
    co_yield detail::from_to(this, std::cbegin(station_routes_), start_offset_,
                             std::cend(station_routes_) - 1, end_offset_,
                             infra);
  }
}

utls::generator<sub_path> interlocking_route::iterate_station_routes(
    infrastructure_t const& infra) const {
  sub_path sp;

  if (station_routes_.size() == 1) {
    auto const sr = infra.station_routes_[station_routes_.front()];
    sp = {.station_route_ = sr, .from_ = start_offset_, .to_ = end_offset_};
    co_yield sp;
  } else {
    auto const first_sr = infra.station_routes_[station_routes_.front()];

    sp = {.station_route_ = first_sr,
          .from_ = start_offset_,
          .to_ = first_sr->size()};
    co_yield sp;

    for (soro::size_t i = 1; i < station_routes_.size() - 1; ++i) {
      auto const sr = infra.station_routes_[station_routes_[i]];
      sp = {.station_route_ = sr, .from_ = 0, .to_ = sr->size()};
      co_yield sp;
    }

    auto const last_sr = infra.station_routes_[station_routes_.back()];
    sp = {.station_route_ = last_sr, .from_ = 0, .to_ = end_offset_};
    co_yield sp;
  }
}

section::ids interlocking_route::get_used_sections(infrastructure const& infra) const {
  section::ids used_sections;
  for (auto const& rn : iterate(infra)) {
    auto const& element = rn.node_->element_;
    if (!element->is_track_element()) {
      continue;
    }

    auto const& sec_ids =
        infra->graph_.element_id_to_section_ids_[element->id()];
    utls::sassert(sec_ids.size() == 1,
                  "track element with more than one section?");
    auto const section_id = sec_ids.front();

    if (used_sections.empty() || used_sections.back() != section_id) {
      used_sections.emplace_back(section_id);
    }
  }

  return used_sections;
}

utls::recursive_generator<route_node> station_route_reverse(infrastructure const& infra, station_route::id station_route_id, node::idx const from, node::idx const to) {
  utls::sassert(from < to, "From: {} is not smaller than to: {}.", from, to);

  auto const sr = infra->station_routes_[station_route_id];

  if (to >= sr->size()) {
    co_return;
  }

  route_node result;
  node::idx node_idx = to;

  while (node_idx > from) {
    --node_idx;
    result.node_ = sr->nodes(node_idx);

    co_yield result;
  }
}

exclusion_section::ids interlocking_route::get_used_exclusion_sections(infrastructure const& infra) const {
  section::ids used_sections = get_used_sections(infra);
  exclusion_section::ids used_exclusion_sections;
  std::set<element_id> handled_crosses;

  auto const add_cross_section = [&handled_crosses, &used_exclusion_sections, &infra](element_id cross_element) {
    if(handled_crosses.contains(cross_element)) {
      return;
    }
    handled_crosses.insert(cross_element);
    used_exclusion_sections.push_back(infra->exclusion_.cross_sections_.at(cross_element));
  };

  auto const first_element = first_node(infra)->element_;
  auto last_element = last_node(infra)->element_;
  if(used_sections.size() == 1) {
    auto const& exclusion_sections = infra->exclusion_.section_to_exclusion_sections_[used_sections.front()];
    size_t index_of_first = 0;
    size_t index_of_last = 0;
    for(size_t i = 0; i < exclusion_sections.size(); ++i) {
      auto const& es = infra->exclusion_.exclusion_sections_[exclusion_sections[i]];
      if(es.contains_end(first_element->id())) index_of_first = i;
      if(es.contains_end(last_element->id())) index_of_last = i;
    }
    if(index_of_first == exclusion_sections.size() - 1 &&
        !infra->exclusion_.exclusion_sections_[exclusion_sections[index_of_first-1]].contains_end(last_element->id())) {
      ++index_of_first;
    }
    if(index_of_last == exclusion_sections.size() - 1 &&
        !infra->exclusion_.exclusion_sections_[exclusion_sections[index_of_last-1]].contains_end(last_element->id())) {
      ++index_of_last;
    }
    auto const lower_bound = exclusion_sections.begin() + std::min(static_cast<long>(index_of_first), static_cast<long>(index_of_last));
    auto const upper_bound = exclusion_sections.begin() + std::max(static_cast<long>(index_of_first), static_cast<long>(index_of_last));
    used_exclusion_sections.insert(used_exclusion_sections.end(), lower_bound, upper_bound);

    // add cross sections if needed
    if(first_element->type() == type::CROSS) add_cross_section(first_element->id());
    if(last_element->type() == type::CROSS) add_cross_section(last_element->id());
    return used_exclusion_sections;
  }

  // add first exclusion section and all the following ones that are part of the same section
  if(!first_element->is_track_element()) {
    // first element is either start or end of a section => add the whole section
    auto const& exclusions = infra->exclusion_.section_to_exclusion_sections_[used_sections.front()];
    used_exclusion_sections.insert(used_exclusion_sections.end(), exclusions.begin(), exclusions.end());
    if(first_element->type() == type::CROSS) add_cross_section(first_element->id());
  } else {
    auto const& first_non_tracking_element = std::find_if(iterate(infra).begin(), iterate(infra).end(), [](auto const& rn) {
                                          return !rn.node_->element_->is_track_element();
                                        })->node_->element_;
    auto const& exclusions_by_first_section = infra->exclusion_.section_to_exclusion_sections_[used_sections.front()];
    // find first exclusion_section that contains first_element
    long index_of_match = -1;
    for(size_t i = 0; i < exclusions_by_first_section.size(); ++i) {
      auto const& es = infra->exclusion_.exclusion_sections_[exclusions_by_first_section[i]];
      if(es.contains_end(first_element->id())) {
        index_of_match = static_cast<long>(i);
        break;
      }
    }
    utls::sassert(index_of_match != -1, "couldn't find first exclusion section that contains first non track element");
    bool collect_left = infra->graph_.sections_[used_sections[0]].first_rising()->id() ==
        first_non_tracking_element->id();
    used_exclusion_sections.insert(used_exclusion_sections.end(),
                                   collect_left ? exclusions_by_first_section.begin() : (exclusions_by_first_section.begin() + index_of_match + 1),
                                   collect_left ? (exclusions_by_first_section.begin() + index_of_match + 1) : exclusions_by_first_section.end());
    if(first_non_tracking_element->type() == type::CROSS) add_cross_section(first_non_tracking_element->id());
  }

  // add all the exclusion sections in between used_sections[1, -1]
  for(soro::size_t i = 1; i < used_sections.size() - 1; ++i) {
    auto const& exclusions = infra->exclusion_.section_to_exclusion_sections_[used_sections[i]];
    used_exclusion_sections.insert(used_exclusion_sections.end(), exclusions.begin(), exclusions.end());
    if(infra->graph_.sections_[used_sections[i]].first_rising()->type() == type::CROSS) add_cross_section(infra->graph_.sections_[used_sections[i]].first_rising()->id());
    if(infra->graph_.sections_[used_sections[i]].first_falling()->type() == type::CROSS) add_cross_section(infra->graph_.sections_[used_sections[i]].first_falling()->id());
  }

  // add last exclusion section and all the exclusion section that lead to it in the last section
  if(!last_element->is_track_element()) {
    // last element is either start or end of a section => add the whole section
    auto const& exclusions = infra->exclusion_.section_to_exclusion_sections_[used_sections.back()];
    used_exclusion_sections.insert(used_exclusion_sections.end(), exclusions.begin(), exclusions.end());
    if(last_element->type() == type::CROSS) add_cross_section(first_element->id());
  } else {
    auto last_sr = station_routes_.back();
    auto reverse_iterator = station_route_reverse(infra, last_sr, station_routes_.size() == 1 ? start_offset_ : 0, end_offset_-1);
    auto const& last_non_tracking_element = std::find_if(reverse_iterator.begin(), reverse_iterator.end(), [](auto const& rn) {
                                          return !rn.node_->element_->is_track_element();
                                        })->node_->element_;
    auto const& exclusions_by_last_section = infra->exclusion_.section_to_exclusion_sections_[used_sections.back()];
    // find first exclusion_section that contains last_element
    long index_of_match = -1;
    for(size_t i = 0; i < exclusions_by_last_section.size(); ++i) {
      auto const& es = infra->exclusion_.exclusion_sections_[exclusions_by_last_section[i]];
      if(es.contains_end(last_element->id())) {
        index_of_match = static_cast<long>(i);
        break;
      }
    }
    utls::sassert(index_of_match != -1, "couldn't find last exclusion section that contains last non track element");
    bool collect_left = infra->graph_.sections_[used_sections.back()].first_rising()->id() ==
        last_non_tracking_element->id();
    used_exclusion_sections.insert(used_exclusion_sections.end(),
                                   collect_left ? exclusions_by_last_section.begin() : (exclusions_by_last_section.begin() + index_of_match + 1),
                                   collect_left ? (exclusions_by_last_section.begin() + index_of_match + 1) : exclusions_by_last_section.end());
    if(last_non_tracking_element->type() == type::CROSS) add_cross_section(last_non_tracking_element->id());
  }

  return used_exclusion_sections;
}

}  // namespace soro::infra