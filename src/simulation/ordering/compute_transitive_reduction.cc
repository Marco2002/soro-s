#include <unordered_set>

#include "soro/simulation/ordering/compute_transitive_reduction.h"

namespace soro::simulation {
using Edge = std::tuple<ordering_node*, ordering_node*>;
using NodeOrder = std::tuple<std::vector<size_t>, std::vector<size_t>>;

struct up_down_node {
  ordering_node* node_;
  bool is_up_;
  size_t degree_;

  up_down_node(ordering_node* node, bool const is_up, size_t const degree)
      : node_(node), is_up_(is_up), degree_(degree) {}
};

struct EdgeHash {
  std::size_t operator()(Edge const& edge) const {
    // Use std::hash for pointers and combine the results
    auto const hash1 = std::hash<ordering_node*>{}(std::get<0>(edge));
    auto const hash2 = std::hash<ordering_node*>{}(std::get<1>(edge));
    return hash1 ^ (hash2 << 1);  // Combine the two hashes
  }
};

void remove_edge(ordering_node& from, ordering_node& to) {
  from.out_.erase(std::ranges::find(from.out_, to.id_));
  to.in_.erase(std::ranges::find(to.in_, from.id_));
}

NodeOrder get_topological_order(ordering_graph& dag) {
  // the algorithm used for creating a topological order of nodes is Kahn's Algorithm
  unsigned long num_of_nodes = dag.nodes_.size();

  std::vector<size_t> topological_order(num_of_nodes);
  std::vector<size_t> topological_order_reverse(num_of_nodes);
  std::queue<ordering_node const*> nodes_without_incoming_edge = {};
  std::vector<size_t> num_of_visited_edges_for_node(num_of_nodes, 0); // this map keeps track of the number of visited edges by Kahn's Algorithm for each node
  size_t current_index = 0;

  // Look for all nodes that have no incoming edges and store them in nodes_without_incoming_edge
  for(auto const& node : dag.nodes_) {
    if(node.in_.empty()) {
      nodes_without_incoming_edge.push(&node);
    }
  }
  // Kahn's Algorithm
  while(!nodes_without_incoming_edge.empty()) {
    // get the last node n from the nodes without incoming edge
    auto const& n = *nodes_without_incoming_edge.front();
    nodes_without_incoming_edge.pop();

    // set the index of the current node
    topological_order[n.id_] = current_index;
    topological_order_reverse[current_index++] = n.id_;

    // loop through each edge e of each node m that has an incoming edge from n to m
    for(auto const outgoing_edge : n.out_) {
      auto const& m = dag.nodes_[outgoing_edge];

      // check if node m has no more incoming edges and if so add it to the topological order
      if(++num_of_visited_edges_for_node[outgoing_edge] == m.in_.size()) {
        nodes_without_incoming_edge.push(&m);
      }
    }
  }

  // Check if the graph is a DAG
  if (current_index != dag.nodes_.size()) {
    for(auto const& n : dag.nodes_) {
      if(n.in_.size() > num_of_visited_edges_for_node[n.id_]) {
        std::cout << "here" << std::endl;
      }
    }

    throw std::invalid_argument("the input graph is not a dag");
  }

  return std::make_tuple(topological_order, topological_order_reverse);
}

void set_edges_in_topological_order(ordering_graph& dag, std::vector<size_t> const& to) {

  // sort outgoing and incoming edges
  for(auto& n : dag.nodes_) {
    std::sort(n.out_.begin(), n.out_.end(), [&to](auto const a, auto const b) { return to[a] < to[b]; });
    std::sort(n.in_.begin(), n.in_.end(), [&to](auto const a, auto const b) { return to[a] > to[b]; });
  }

}

void depth_first_search_visit(ordering_node const& n, LabelDiscovery& label_discover, LabelFinish& label_finish, std::vector<ordering_node const*>& post_order, long& current, size_t& order_index, std::vector<ordering_node> const& nodes) {
  label_discover[n.id_] = ++current;
  for (auto const e : n.out_) {
    if (label_discover[e] != 0) continue; // if e was visited

    depth_first_search_visit(nodes[e], label_discover, label_finish, post_order, current, order_index, nodes);
  }
  post_order[order_index++] = &n;
  label_finish[n.id_] = ++current;
}

std::tuple<std::vector<ordering_node const*>, LabelDiscovery, LabelFinish> depth_first_search(ordering_graph const& g) {
  LabelDiscovery label_discovery(g.nodes_.size());
  LabelFinish label_finish(g.nodes_.size());
  std::vector<ordering_node const*> post_order(g.nodes_.size());
  long current = 0; // keeps track of the current step for the label_discovery and label_finish
  size_t order_index = 0;

  // start DFS on all nodes without incoming edges
  for(auto const& n : g.nodes_) {
    if(n.in_.empty()) {
      depth_first_search_visit(n, label_discovery, label_finish, post_order, current, order_index, g.nodes_);
    }
  }

  if(order_index < g.nodes_.size() - 1) {
    throw std::invalid_argument( "the input graph is not a dag" );
  }

  return std::make_tuple(post_order, label_discovery, label_finish);
}

std::vector<ordering_node const*> merge_vertices(std::vector<ordering_node const*> const& post_order, size_t const d) {
  size_t const num_of_intervals = std::min(d, post_order.size());
  std::vector<const ordering_node*> g(post_order.size());

  // Calculate the width of each interval
  size_t interval_width = std::max(post_order.size() / num_of_intervals, 1ul);
  std::vector<size_t> lower_bounds(num_of_intervals+1);

  // Vector to store the intervals as pairs
  std::vector<std::pair<long, long>> intervals;

  for (size_t i = 0; i < num_of_intervals; ++i) {
    lower_bounds[i] = i * interval_width;
  }
  lower_bounds[num_of_intervals] = post_order.size(); // add one "extra" lower bound to make the loop simpler

  for(size_t i = 0; i < num_of_intervals; ++i) {
    for(size_t j = lower_bounds[i]; j < lower_bounds[i+1]; ++j) {
      g[post_order[j]->id_] = post_order[lower_bounds[i]];
    }
  }

  return g;
}

std::vector<Edge> sort_edge_tro_plus(ordering_graph& graph,
                                     std::vector<size_t> const& to) {
  set_edges_in_topological_order(graph,
                                 to);  // add sorting into topological order

  unsigned long number_of_edges = 0;
  for (auto const& node : graph.nodes_) {
    number_of_edges += node.out_.size();
  }

  std::vector<Edge> queue;
  queue.reserve(number_of_edges);

  std::vector<up_down_node> up_and_down_nodes;
  up_and_down_nodes.reserve(graph.nodes_.size() * 2);
  // divide nodes into UP-nodes and DOWN-nodes
  for (auto& node : graph.nodes_) {
    up_and_down_nodes.emplace_back(&node, true, node.in_.size());
    up_and_down_nodes.emplace_back(&node, false, node.out_.size());
  }
  // sort up and down nodes by their degree in ascending order
  std::sort(up_and_down_nodes.begin(), up_and_down_nodes.end(),
            [](up_down_node const& a, up_down_node const& b) {
              return a.degree_ < b.degree_;
            });

  std::unordered_set<Edge, EdgeHash> handled_edges(number_of_edges);

  for (auto const& up_down_node : up_and_down_nodes) {
    if (up_down_node.is_up_) {
      for (auto incoming_node : up_down_node.node_->in_) {  // loop in descending order through incoming_edges
        auto edge = std::make_tuple(&graph.nodes_[incoming_node], up_down_node.node_);
        auto [it, inserted] = handled_edges.insert(edge);
        if (inserted) {
          queue.emplace_back(std::move(edge));
        }
      }
    } else {
      for (auto outgoing_node : up_down_node.node_->out_) {  // loop in ascending order through outgoing_edges
        auto edge = std::make_tuple(up_down_node.node_, &graph.nodes_[outgoing_node]);
        auto [it, inserted] = handled_edges.insert(edge);
        if (inserted) {
          queue.emplace_back(std::move(edge));
        }
      }
    }
  }

  return queue;
}

template <size_t hash_range>
bool is_redundant_tro_plus(labeled_graph<hash_range> const& labeled_graph,
                           Edge const& edge, std::vector<size_t> const& to) {
  auto const [u, v] = edge;
  auto const u_index = to[u->id_];
  auto const v_index = to[v->id_];
  if (u->out_.size() > v->in_.size()) {
    for (auto const w : v->in_) {  // loop in descending order through incoming_edges
      if (to[w] <= u_index) break;  // add index check
      if (query_reachability(labeled_graph, *u, labeled_graph.graph_.nodes_[w])) {
        return true;
      }
    }
  } else {
    for (auto const w : u->out_) {  // loop in ascending order through outgoing_edges
      if (to[w] >= v_index) break;  // add index check
      if (query_reachability(labeled_graph, labeled_graph.graph_.nodes_[w], *v)) {
        return true;
      }
    }
  }
  return false;
}

// Algorithm TR-O-Plus
void remove_transitive_edges(ordering_graph& graph) {
  auto const hash_range = 1024;
  auto const [to, to_reverse] = get_topological_order(graph);
  auto const labeled_graph = build_labeled_graph<hash_range>(
      graph, [](ordering_node const* n) { return n->id_ % hash_range; },
      static_cast<size_t>(hash_range * 10));

  auto queue = sort_edge_tro_plus(graph, to);

  std::map<ordering_node::id, tt::train::trip> node_to_trip;
  for (auto const& trip : graph.trip_to_nodes_) {
    for (auto node = trip.second.first; node != trip.second.second; ++node) {
      node_to_trip[node] = trip.first;
    }
  }
  for (auto edge : queue) {
    auto const& from_node = *std::get<0>(edge);
    auto const& to_node = *std::get<1>(edge);
    if (is_redundant_tro_plus(labeled_graph, edge, to)
        && !(from_node.id_ == to_node.id_-1 && node_to_trip[from_node.id_] == node_to_trip[to_node.id_])) {

      remove_edge(*std::get<0>(edge), *std::get<1>(edge));
    }
  }
}

}