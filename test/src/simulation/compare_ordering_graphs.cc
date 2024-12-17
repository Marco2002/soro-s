#include "doctest/doctest.h"

#include "soro/base/time.h"

#include "soro/utls/graph/has_cycle.h"
#include "soro/utls/std_wrapper/contains.h"

#include "soro/simulation/ordering/check_exclusion_paths.h"
#include "soro/simulation/ordering/ordering_graph.h"

#include "test/file_paths.h"
#include <stack>
#include <unordered_set>

namespace soro::simulation::test {

  using namespace soro::tt;
  using namespace soro::infra;

bool check_reachability(ordering_graph const& og, ordering_node::id from, ordering_node::id to) {
  if (from == to) {
    std::cout << "Path: " << from << std::endl;
    return true; // A node is always reachable from itself
  }

  if (from >= og.nodes_.size() || to >= og.nodes_.size()) {
    return false; // Invalid node IDs
  }

  // Stack for DFS traversal
  std::stack<ordering_node::id> stack;
  // Map to track the parent of each node
  std::unordered_map<ordering_node::id, ordering_node::id> parent;
  // Set to track visited nodes
  std::unordered_set<ordering_node::id> visited;

  // Initialize the stack with the starting node
  stack.push(from);
  parent[from] = from; // The start node's parent is itself

  while (!stack.empty()) {
    auto current = stack.top();
    stack.pop();

    // If the current node is already visited, skip it
    if (visited.count(current)) {
      continue;
    }

    // Mark the current node as visited
    visited.insert(current);

    // Check if the target node is reached
    if (current == to) {
      // Reconstruct and print the path
      std::vector<ordering_node::id> path;
      for (auto node = to; node != from; node = parent[node]) {
        path.push_back(node);
      }
      path.push_back(from); // Add the start node
      std::reverse(path.begin(), path.end());

      // Print the path
      std::cout << "Path: ";
      for (auto node : path) {
        std::cout << "(" << node << "," << og.nodes_[node].ir_id_ << ") ";
      }
      std::cout << std::endl;

      return true;
    }

    // Get the current node's outgoing edges
    const auto& outgoing_edges = og.nodes_[current].out_;

    // Push all unvisited neighbors to the stack and set their parent
    for (auto neighbor : outgoing_edges) {
      if (!visited.count(neighbor)) {
        stack.push(neighbor);
        parent[neighbor] = current;
      }
    }
  }

  // If the target node was not found, return false
  std::cout << "No path found" << std::endl;
  return false;
}

void compare_graphs(ordering_graph const& og, ordering_graph const& og2) {
  CHECK_EQ(og.nodes_.size(), og2.nodes_.size());
  for(size_t i = 0; i < og.nodes_.size(); ++i) {
    auto const& node = og.nodes_[i];
    auto const& node2 = og2.nodes_[i];
    CHECK_EQ(node.id_, node2.id_);
    CHECK_EQ(node.ir_id_, node2.ir_id_);
    CHECK_EQ(node.train_id_, node2.train_id_);
    if(node.in_.size() != node2.in_.size()) {
      if(node.in_.size() > node2.in_.size()) {
        for(auto const& in : node.in_) {
          if(!utls::contains(node2.in_, in)) {
            std::cout << "node (" << in << "," << og.nodes_[in].ir_id_ << ") is in node (" << node.id_ << "," << node.ir_id_ << ") in og1 but not in og2" << std::endl;
            check_reachability(og2, in, node.id_);
          }
        }
      } else {
        for(auto const& in : node2.in_) {
          if(!utls::contains(node.in_, in)) {
            std::cout << "node (" << in << "," << og2.nodes_[in].ir_id_ << ") is in node (" << node2.id_ << "," << node2.ir_id_ << ") in og2 but not in og1" << std::endl;
            check_reachability(og, in, node.id_);
          }
        }
      }
    }
  }
}

TEST_CASE("compare graphs") {
  auto opts = soro::test::DE_ISS_OPTS;
  auto tt_opts = soro::test::DE_KSS_OPTS;

  opts.exclusions_ = true;
  opts.interlocking_ = true;
  opts.exclusion_graph_ = true;
  opts.exclusion_sets_ = true;
  opts.layout_ = false;

  interval const inter{.start_ = rep_to_absolute_time(1636786800),
                       .end_ = rep_to_absolute_time(1636786800) + hours{8}};

  infrastructure const infra(opts);
  timetable const tt(tt_opts, infra);

  ordering_graph const og(infra, tt, {.interval_ = inter});
  ordering_graph const og2(infra, tt, {.interval_ = inter}, true);

  compare_graphs(og, og2);
}

}  // namespace soro::simulation::test