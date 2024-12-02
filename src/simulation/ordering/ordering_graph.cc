#include "soro/simulation/ordering/ordering_graph.h"

#include "range/v3/range/conversion.hpp"
#include "range/v3/view/filter.hpp"
#include "range/v3/view/transform.hpp"

#include "utl/concat.h"
#include "utl/erase.h"
#include "utl/erase_duplicates.h"
#include "utl/parallel_for.h"
#include "utl/timer.h"

#include "soro/utls/graph/traversal.h"
#include "soro/utls/std_wrapper/contains.h"
#include "soro/utls/std_wrapper/count_if.h"
#include "soro/utls/std_wrapper/sort.h"

#include "soro/runtime/runtime.h"
#include "soro/simulation/ordering/remove_transitive_edges.h"

namespace soro::simulation {

using namespace soro::tt;
using namespace soro::infra;
using namespace soro::runtime;

void print_ordering_graph_stats(ordering_graph const& og) {
  std::size_t edges = 0;

  // edge count e -> node count with edge count e
  std::map<std::size_t, std::size_t> in_edge_counts;
  std::map<std::size_t, std::size_t> out_edge_counts;

  for (auto const& n : og.nodes_) {
    edges += n.out_.size();
    ++in_edge_counts[n.in_.size()];
    ++out_edge_counts[n.out_.size()];
  }

  uLOG(utl::info) << "ordering graph node count: " << og.nodes_.size();
  uLOG(utl::info) << "ordering graph edge count: " << edges;

  uLOG(utl::info) << "incoming edges distribution:";
  for (auto const& [edge_count, nodes] : in_edge_counts) {
    uLOG(utl::info) << "nodes with " << edge_count << " in edges: " << nodes;
  }

  uLOG(utl::info) << "outgoing edges distribution:";
  for (auto const& [edge_count, nodes] : out_edge_counts) {
    uLOG(utl::info) << "nodes with " << edge_count << " out edges: " << nodes;
  }

  uLOG(utl::info) << "Total trips in ordering graph: "
                  << og.trip_to_nodes_.size();
}

struct route_usage {
  soro::absolute_time timestamp_{}; // time where it is begin to used
  ordering_node::id id_{ordering_node::INVALID};
};

struct usage_data {
  std::vector<std::vector<exclusion_section::id>> used_sections;
  std::vector<std::vector<route_usage*>> usages;
  std::vector<soro::data::bitvec> reachability_data;

  route_usage* find_next_usage(section::id const section, ordering_node::id const node_id) {
    auto const& section_usages = usages[section];
    for(size_t i = 0; i < section_usages.size()-1; ++i) {
      if(section_usages[i]->id_ == node_id) {
        return section_usages[i+1];
      }
    }
    return nullptr;
  }
};

void search_with_break(ordering_node::id node_id, soro::data::bitvec& handled_exclusions, ordering_node::id break_node,
                       std::vector<ordering_node>& nodes, usage_data& usage_data) {
  handled_exclusions.set(node_id, true);
  auto const used_sections = usage_data.used_sections[node_id];
  std::vector<route_usage*> next_usages;
  for(auto const section : used_sections) {
    auto next_usage = usage_data.find_next_usage(section, node_id);
    if(next_usage != nullptr) {
      next_usages.push_back(next_usage);
    }
  }



  if(!next_usages.empty()) {
    auto next_usage = (*(std::min_element(next_usages.begin(), next_usages.end(), [](auto const a, auto const b) {
                        return a->timestamp_ < b->timestamp_;
                      })))->id_;
    if(!handled_exclusions[next_usage]) {
      nodes[node_id].out_.push_back(next_usage);
      nodes[next_usage].in_.push_back(node_id);

      // search other node if not yet searched
      if(usage_data.reachability_data[next_usage].none()) {
        auto end_node = &nodes[next_usage];
        while(!end_node->out_.empty()) end_node = &nodes[end_node->out_.front()];
        soro::data::bitvec next_usage_exclusions;
        next_usage_exclusions.resize(handled_exclusions.size());
        search_with_break(end_node->id_, next_usage_exclusions, next_usage, nodes, usage_data);
      }

      auto other_exclusions = usage_data.reachability_data[next_usage];
      handled_exclusions |= other_exclusions;
    }
  }

  usage_data.reachability_data[node_id] = handled_exclusions;
  if(node_id != break_node && !nodes[node_id].in_.empty()) {
    search_with_break(nodes[node_id].in_.front(), handled_exclusions, break_node, nodes, usage_data);
  }
}

void search(ordering_node::id const node_id, soro::data::bitvec& handled_exclusions,
            std::vector<ordering_node>& nodes, usage_data& usage_data) {
  search_with_break(node_id, handled_exclusions, ordering_node::INVALID, nodes, usage_data);
}

void start_search(ordering_node::id const node_id, std::vector<ordering_node>& nodes,
                  usage_data& usage_data) {
  soro::data::bitvec handled_exclusions;
  // if the next node was already visited, then take its reachability data as handled exclusions
  if(nodes[node_id].out_.empty()) {
    handled_exclusions.resize(static_cast<unsigned int>(nodes.size()));
  } else {
    handled_exclusions = usage_data.reachability_data[nodes[node_id].out_.front()];
  }
  search(node_id, handled_exclusions, nodes, usage_data);
}

ordering_graph::ordering_graph(infra::infrastructure const& infra,
                               tt::timetable const& tt)
    : ordering_graph(infra, tt, filter{}) {}

ordering_graph::ordering_graph(const infra::infrastructure& infra,
                               const tt::timetable& tt, const soro::simulation::ordering_graph::filter& filter) {
  std::cout << "filtering trains: ";
  utl::scoped_timer const timer("creating ordering graph");
  ordering_node::id glob_current_node_id = 0;
  size_t total_number_of_nodes = 0;

  std::cout << "filtering trains: " << &infra;

  for (auto const &train : tt->trains_) {
    if (!filter.trains_.empty() && !utls::contains(filter.trains_, train.id_)) {
      continue;
    }
    for ([[maybe_unused]] auto const _ : train.departures(filter.interval_)) {
      total_number_of_nodes += train.path_.size();
    }
  }

  std::cout << "total number of nodes: " << total_number_of_nodes << std::endl;

  std::vector<route_usage> route_usages(total_number_of_nodes);
  nodes_.resize(total_number_of_nodes);
  usage_data usage_data = {
      .used_sections = std::vector<std::vector<exclusion_section::id>>(total_number_of_nodes),
      .usages = std::vector<std::vector<route_usage*>>(infra->exclusion_.exclusion_sections_.size()),
      .reachability_data = std::vector<soro::data::bitvec>(total_number_of_nodes),
  };

  // populate exclusion_sets and nodes with primitive edges
  for (auto const &train : tt->trains_) {
    if (!filter.trains_.empty() && !utls::contains(filter.trains_, train.id_)) {
      continue;
    }

    soro::vector<timestamp> times;

    for (auto const anchor : train.departures(filter.interval_)) {
      if (times.empty()) {
        times = runtime_calculation(train, infra, {type::MAIN_SIGNAL}).times_;

        utls::sasserts([&]() {
          auto const ms_count = utls::count_if(times, [](auto&& t) {
            return t.element_->is(type::MAIN_SIGNAL);
          });

          utls::sassert(
              train.path_.size() == ms_count + 1,
              "Differing amounts of interlocking routes in train path and "
              "main signals in running time calculation timestamps");
        });
      }
      if (times.empty()) {
        uLOG(utl::warn) << "no main signal in path of train " << train.id_;
        return;
      }

      ordering_node::id curr_node_id = ordering_node::INVALID;

      curr_node_id = glob_current_node_id;
      glob_current_node_id += train.path_.size();

      // add train trip to trip_to_nodes_
      trip_to_nodes_.emplace(
          train::trip{.train_id_ = train.id_, .anchor_ = anchor},
          std::pair{curr_node_id, static_cast<ordering_node::id>(
                                      curr_node_id + train.path_.size())});

      for(size_t i = 0; i < train.path_.size(); ++i) {
        std::vector<node::id> in;
        std::vector<node::id> out;
        if(i > 0) in.push_back(curr_node_id - 1);
        if(i < train.path_.size() - 1) out.push_back(curr_node_id + 1);
        nodes_[curr_node_id] = ordering_node{.id_ = curr_node_id,
                                             .ir_id_ = train.path_[i],
                                             .train_id_ = train.id_,
                                             .in_ = in,
                                             .out_ = out};

        route_usages[curr_node_id] = {
                .timestamp_ = relative_to_absolute(anchor, i == 0 ? train.first_departure() : times[i-1].departure_),
                .id_ = curr_node_id};
        ++curr_node_id;
      }
    }
  }
  // sort route_usages by from time
  utls::sort(route_usages, [](auto const& a, auto const& b) {
    return a.timestamp_ < b.timestamp_;
  });

  // populate used_sections and usages
  for (auto& usage : route_usages) {
    auto const& node = nodes_[usage.id_];
    auto const& ir = infra->interlocking_.routes_[node.ir_id_];
    // auto sections = ir.get_used_sections(infra);
    auto sections = ir.get_used_exclusion_sections(infra);

    for(auto section : sections) {
      usage_data.used_sections[node.id_].emplace_back(section);
      usage_data.usages[section].push_back(&usage);
    }
  }

  // start adding edges between train trips by searching backwards though ordering nodes
  for(auto it = route_usages.rbegin(); it != route_usages.rend(); ++it) {
    if(usage_data.reachability_data[it->id_].none()) {
      start_search(it->id_, nodes_, usage_data);
    }
  }

  print_ordering_graph_stats(*this);
}

std::span<const ordering_node> ordering_graph::trip_nodes(
    tt::train::trip const trip) const {
  auto const it = trip_to_nodes_.find(trip);

  utls::sassert(it != std::end(trip_to_nodes_),
                "could not find nodes for trip {}", trip);

  return {&nodes_[it->second.first], it->second.second - it->second.first};
}

ordering_node const& ordering_node::next(ordering_graph const& og) const {
  utls::sasserts([this, &og] {
    utls::sassert(!out_.empty(), "no next node");
    auto const& next = og.nodes_[out_.front()];
    utls::sassert(next.train_id_ == train_id_, "next node not same train");
  });

  return og.nodes_[out_.front()];
}

}  // namespace soro::simulation