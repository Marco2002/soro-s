#include <stack>

#include "doctest/doctest.h"

#include "soro/base/time.h"

#include "soro/utls/graph/has_cycle.h"
#include "soro/utls/std_wrapper/contains.h"

#include "soro/simulation/ordering/check_exclusion_paths.h"
#include "soro/simulation/ordering/ordering_graph.h"

#include "test/file_paths.h"

namespace soro::simulation::test {

using namespace soro::tt;
using namespace soro::infra;

void fill_by_dfs(ordering_graph const& og, ordering_node::id start_node, std::unordered_map<ordering_node::id, bool>& transiently_reachable_edges) {
  vector<bool> visited(og.nodes_.size(), false);
  std::stack<ordering_node::id> stack;
  stack.push(start_node);

  while (!stack.empty()) {
    auto v = stack.top();
    stack.pop();
    if(v != start_node) {
      transiently_reachable_edges[v] = true;
    }

    if (!visited[v]) {
      visited[v] = true;
    }

    // Visit all adjacent vertices
    for (auto i : og.nodes_[v].out_) {
      if (!visited[i]) {
        stack.push(i);
      }
    }
  }
}

void check_no_transient_edges_and_no_duplicates(ordering_graph const& og) {
  std::unordered_map<ordering_node::id, std::unordered_map<ordering_node::id, bool>> transiently_reachable_edges_of_node(og.nodes_.size());
  for(auto const& node : og.nodes_) {
    // find all transiently reachable nodes that are reachable from node x
    fill_by_dfs(og, node.id_, transiently_reachable_edges_of_node[node.id_]);
    // assert no duplicate edges
    std::unordered_map<ordering_node::id, int> freqMap;
    std::set<int> duplicates;
    for (const auto& elem : node.out_) {
      freqMap[elem]++;
      CHECK(freqMap[elem] == 1);
    }
    freqMap.clear();
    for (const auto& elem : node.in_) {
      freqMap[elem]++;
      CHECK(freqMap[elem] == 1);
    }
  }

  std::map<ordering_node::id, tt::train::trip> node_to_trip;
  for (auto trip : og.trip_to_nodes_) {
    for(auto i = trip.second.first; i < trip.second.second; ++i) {
      node_to_trip[i] = trip.first;
    }
  }

  // check that no transiently reachable node is directly connected (unless its direct following edge)
  for(auto const& node : og.nodes_) {
    for(auto const out_id : node.out_) {
      for(auto const reachable_node : transiently_reachable_edges_of_node[out_id]) {
        if(reachable_node.first == node.id_+1 && node_to_trip[node.id_] == node_to_trip[reachable_node.first]) {
          continue;
        }
        CHECK(!utls::contains(node.out_, reachable_node.first));
      }
    }
  }
}

void check_ordering_graph(ordering_graph const& og,
                          infrastructure const& infra) {
  // no cycles allowed in the ordering graph
  CHECK(!utls::has_cycle(
      og.nodes_, [](auto&& nodes, auto&& id) { return nodes[id].out_; }));

  // all exclusion paths have to exist
  CHECK(has_exclusion_paths(og, infra));
}

TEST_SUITE("ordering graph") {

  TEST_CASE("ordering graph, follow") {
    auto opts = soro::test::SMALL_OPTS;
    auto tt_opts = soro::test::FOLLOW_OPTS;

    opts.exclusions_ = true;
    opts.interlocking_ = true;
    opts.exclusion_graph_ = false;
    opts.layout_ = false;

    infrastructure const infra(opts);

    timetable const tt(tt_opts, infra);

    // we have two trains, with one trip each
    CHECK_EQ(tt->trains_.size(), 2);
    CHECK_EQ(tt->trains_.front().trip_count(), 1);
    CHECK_EQ(tt->trains_.back().trip_count(), 1);

    auto const& [earlier, later] =
        tt->trains_.front().first_departure() <=
                tt->trains_.back().first_departure()
            ? std::pair{tt->trains_.front(), tt->trains_.back()}
            : std::pair{tt->trains_.back(), tt->trains_.front()};

    // both take the same path
    CHECK_EQ(earlier.path_, later.path_);

    ordering_graph const og(infra, tt);

    check_no_transient_edges_and_no_duplicates(og);
    check_ordering_graph(og, infra);

    // the ordering graph has a node for every {train, interlocking route} pair
    CHECK_EQ(earlier.path_.size() + later.path_.size(), og.nodes_.size());

    auto const earlier_trip = earlier.trips().front();
    auto const later_trip = later.trips().front();

    auto const earlier_nodes = og.trip_nodes(earlier_trip);
    auto const later_nodes = og.trip_nodes(later_trip);

    // check train edges
    for (auto const [from, to] : utl::pairwise(earlier_nodes)) {
      CHECK(utls::contains(from.out_, to.id_));
      CHECK(utls::contains(to.in_, from.id_));
    }

    // check ordering edges
    for (auto idx = 0U; idx < earlier_nodes.size(); ++idx) {
      auto const& from = earlier_nodes[idx];
      auto const& to = later_nodes[idx];

      CHECK(utls::contains(from.out_, to.id_));
      CHECK(utls::contains(to.in_, from.id_));
    }
  }

  TEST_CASE("ordering graph, cross") {
    auto opts = soro::test::SMALL_OPTS;
    auto tt_opts = soro::test::CROSS_OPTS;

    opts.exclusions_ = true;
    opts.interlocking_ = true;
    opts.exclusion_graph_ = false;
    opts.layout_ = false;

    infrastructure const infra(opts);
    timetable const tt(tt_opts, infra);
    ordering_graph const og(infra, tt);

    check_no_transient_edges_and_no_duplicates(og);
    check_ordering_graph(og, infra);
  }

  TEST_CASE("de_kss graph") {
    auto opts = soro::test::DE_ISS_OPTS;
    auto tt_opts = soro::test::DE_KSS_OPTS;

    opts.exclusions_ = true;
    opts.interlocking_ = true;
    opts.exclusion_graph_ = false;
    opts.layout_ = false;

    interval const inter{.start_ = rep_to_absolute_time(1636786800),
                         .end_ = rep_to_absolute_time(1636786800) + hours{1}};

    infrastructure const infra(opts);
    timetable const tt(tt_opts, infra);

    ordering_graph const og(infra, tt, {.interval_ = inter});

    check_no_transient_edges_and_no_duplicates(og);
    check_ordering_graph(og, infra);
  }
}

}  // namespace soro::simulation::test