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
  soro::absolute_time from_{};
  soro::absolute_time to_{};
  ordering_node::id id_{ordering_node::INVALID};
};

ordering_graph::ordering_graph(infra::infrastructure const& infra,
                               tt::timetable const& tt)
    : ordering_graph(infra, tt, filter{}) {}

ordering_graph::ordering_graph(infra::infrastructure const& infra,
                               tt::timetable const& tt, filter const& filter) {
  utl::scoped_timer const timer("creating ordering graph");
  std::vector<std::vector<route_usage>> exclusion_sets(
      infra->exclusion_.exclusion_sets_.size());

  auto const insert_into_exclusion_set = [&](route_usage const& usage,
                                             interlocking_route::id const ir_id) {
    for (auto const es_id : infra->exclusion_.irs_to_exclusion_sets_[ir_id]) {
      exclusion_sets[es_id].push_back(usage);
    }
  };

  ordering_node::id glob_current_node_id = 0;

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
      nodes_.resize(nodes_.size() + train.path_.size());

      // add train trip to trip_to_nodes_
      trip_to_nodes_.emplace(
          train::trip{.train_id_ = train.id_, .anchor_ = anchor},
          std::pair{curr_node_id, static_cast<ordering_node::id>(
                                      curr_node_id + train.path_.size())});

      // add first halt -> first main signal
      nodes_[curr_node_id] = ordering_node{.id_ = curr_node_id,
                                           .ir_id_ = train.path_.front(),
                                           .train_id_ = train.id_,
                                           .in_ = {},
                                           .out_ = {curr_node_id + 1}};

      route_usage const first_usage = {
          .from_ = relative_to_absolute(anchor, train.first_departure()),
          .to_ = relative_to_absolute(anchor, times.front().arrival_),
          .id_ = curr_node_id};

      insert_into_exclusion_set(first_usage, nodes_[curr_node_id].ir_id_);
      ++curr_node_id;

      // add trip route_usages
      auto path_idx = 1U;
      for (auto const [from_time, to_time] : utl::pairwise(times)) {
        nodes_[curr_node_id] = ordering_node{.id_ = curr_node_id,
                                             .ir_id_ = train.path_[path_idx],
                                             .train_id_ = train.id_,
                                             .in_ = {curr_node_id - 1},
                                             .out_ = {curr_node_id + 1}};

        route_usage const usage = {
            .from_ = relative_to_absolute(anchor, from_time.arrival_),
            .to_ = relative_to_absolute(anchor, from_time.departure_),
            .id_ = curr_node_id};

        insert_into_exclusion_set(usage, nodes_[curr_node_id].ir_id_);

        ++path_idx;
        ++curr_node_id;
      }

      // add last main signal -> last halt
      nodes_[curr_node_id] = ordering_node{.id_ = curr_node_id,
                                           .ir_id_ = train.path_.back(),
                                           .train_id_ = train.id_,
                                           .in_ = {curr_node_id - 1},
                                           .out_ = {}};

      route_usage const last_usage = {
          .from_ = relative_to_absolute(anchor, times.back().arrival_),
          .to_ = relative_to_absolute(anchor, train.last_arrival()),
          .id_ = curr_node_id};

      insert_into_exclusion_set(last_usage, nodes_[curr_node_id].ir_id_);
      ++curr_node_id;

      utls::sassert(path_idx == train.path_.size() - 1);
    }
  }

  for(auto const& train : tt->trains_) {
    if (!filter.trains_.empty() && !utls::contains(filter.trains_, train.id_)) {
      continue;
    }
    for (auto const anchor : train.departures(filter.interval_)) {
      auto const& trip = trip_to_nodes_[train::trip{.train_id_ = train.id_, .anchor_ = anchor}];
      std::unordered_map<ordering_node::id, bool> handled_exclusions = {};

      for(auto i = trip.second; i > trip.first; --i) {
        auto const node_id = i-1;
        auto const ir_id = nodes_[node_id].ir_id_;
        auto const& affected_exclusion_sets = infra->exclusion_.irs_to_exclusion_sets_[ir_id];
        std::unordered_map<ordering_node::id, std::pair<std::size_t, std::size_t>> added_arcs = {};

        for(auto const& affected_exclusion_set : affected_exclusion_sets) {
          soro::absolute_time self_time;
          for(auto const other : exclusion_sets[affected_exclusion_set]) {

            if(other.id_ == node_id || handled_exclusions.find(other.id_) != handled_exclusions.end()) {
              self_time = other.from_;
            }
          }

          for(auto const& other : exclusion_sets[affected_exclusion_set]) {

            if(other.id_ == node_id || handled_exclusions.find(other.id_) != handled_exclusions.end() || other.from_ < self_time) {
              continue;
            }

            // do a dfs on the other node and add all reachable nodes to the handled_exclusions
            std::stack<ordering_node::id> stack;
            stack.push(other.id_);

            while (!stack.empty()) {
              auto v = stack.top();
              stack.pop();

              if(handled_exclusions.find(v) != handled_exclusions.end()) {
                if(added_arcs.find(v) != added_arcs.end()) {
                  nodes_[node_id].out_.erase(nodes_[node_id].out_.begin() + added_arcs[v].first);
                  nodes_[v].in_.erase(nodes_[v].in_.begin() + added_arcs[v].second);
                }
              } else {
                handled_exclusions[v] = true;
                // Visit all adjacent vertices
                for (auto i : nodes_[v].out_) {
                  if (handled_exclusions.find(i) == handled_exclusions.end()) {
                    stack.push(i);
                  }
                }
              }
            }

            added_arcs[other.id_] = { nodes_[node_id].out_.size(), nodes_[other.id_].in_.size() };
            nodes_[node_id].out_.emplace_back(other.id_);
            nodes_[other.id_].in_.emplace_back(node_id);
          }
        }
      }
    }
  }

//  for (auto& node : nodes_) {
//    utl::erase_duplicates(node.out_);
//    utl::erase_duplicates(node.in_);
//  }
//
//  // Replace with new algorithm
//  remove_transitive_edges(*this);
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