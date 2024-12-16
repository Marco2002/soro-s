#pragma once

#include <bitset>
#include <functional>
#include <utility>
#include <iostream>
#include <unordered_set>
#include <queue>

#include "soro/simulation/ordering/ordering_graph.h"

namespace soro::simulation {

template <size_t hash_range>
using LabelIn = std::vector<std::bitset<hash_range>>;

template <size_t hash_range>
using LabelOut = std::vector<std::bitset<hash_range>>;

using LabelDiscovery = std::vector<long>;
using LabelFinish = std::vector<long>;

template <size_t hash_range>
struct labeled_graph {
  ordering_graph& graph_;
  LabelDiscovery label_discovery_;
  LabelFinish label_finish_;
  LabelIn<hash_range> label_in_;
  LabelOut<hash_range> label_out_;

  labeled_graph(ordering_graph& graph, LabelDiscovery label_discovery, LabelFinish label_finish, LabelIn<hash_range> label_in, LabelOut<hash_range> label_out)
      : graph_(graph), label_discovery_(std::move(label_discovery)), label_finish_(std::move(label_finish)), label_in_(label_in), label_out_(label_out) {}
};

std::tuple<std::vector<ordering_node const*>, LabelDiscovery, LabelFinish> depth_first_search(ordering_graph const& g);

std::vector<ordering_node const*> merge_vertices(std::vector<ordering_node const*> const& post_order, size_t const d);

template <size_t hash_range>
void compute_label_out(ordering_graph const& graph, std::vector<ordering_node const*> const& g, std::function<size_t(ordering_node const*)> const& h, ordering_node const& n, std::vector<std::bitset<hash_range>>& label_out) {
  label_out[n.id_].set(h(g[n.id_]));
  for(auto const successor : n.out_) {
    if(label_out[successor].none()) { // if successor has not been visited
      compute_label_out<hash_range>(graph, g, h, graph.nodes_[successor], label_out);
    }
    label_out[n.id_] |= label_out[successor]; // label_out[n] = label_out[n] union label_out[successor]
  }
}

template <size_t hash_range>
void compute_label_in(ordering_graph const& graph, std::vector<ordering_node const*> const& g, std::function<size_t(ordering_node const*)> const& h, ordering_node const& n, LabelIn<hash_range>& label_in) {
  label_in[n.id_].set(h(g[n.id_]));
  for(auto const predecessor : n.in_) {
    if(label_in[predecessor].none()) { // if successor has not been visited
      compute_label_in<hash_range>(graph, g, h, graph.nodes_[predecessor], label_in);
    }
    label_in[n.id_] |= label_in[predecessor]; // label_in[n] = label_in[n] union label_in[predecessor]
  }
}

// the hash should map to values in a range from 0...hash_range-1
template <size_t hash_range> // the range is the number of values that can be possible outputs of the hash function
labeled_graph<hash_range> build_labeled_graph(ordering_graph& graph, std::function<size_t(ordering_node const*)> const& h, size_t const d) {
  LabelIn<hash_range> label_in(graph.nodes_.size());
  LabelOut<hash_range> label_out(graph.nodes_.size());

  auto [post_order, label_discovery, label_finish] = depth_first_search(graph);

  auto g = merge_vertices(post_order, d);

  for(auto n : post_order) {
    if(label_out[n->id_].none()) {
      compute_label_out<hash_range>(graph, g, h, *n, label_out);
    }
    if(label_in[n->id_].none()) {
      compute_label_in<hash_range>(graph, g, h, *n, label_in);
    }
  }

  return labeled_graph<hash_range>(graph, label_discovery, label_finish, label_in, label_out);
}

template <size_t hash_range>
bool query_reachability(const labeled_graph<hash_range>& graph, const ordering_node& u, const ordering_node& v) {
  std::vector<bool> visited(graph.graph_.nodes_.size(), false);
  return query_reachability<hash_range>(graph, u, v, visited);
}

template <size_t hash_range>
bool query_reachability(labeled_graph<hash_range> const& graph, ordering_node const& u, ordering_node const& v, std::vector<bool>& visited) {
  visited[u.id_] = true;

  if(graph.label_discovery_[u.id_] <= graph.label_discovery_[v.id_] && graph.label_finish_[v.id_] <= graph.label_finish_[u.id_]) {
    return true;
  }
  // if L_out(v) !subset_of L_out(u) or L_in(u) !subset_of L_in(v)
  if((graph.label_out_[v.id_] & graph.label_out_[u.id_]) != graph.label_out_[v.id_]
      || (graph.label_in_[u.id_] & graph.label_in_[v.id_]) != graph.label_in_[u.id_]) {
    return false;
  }
  for (auto const w : u.out_) {
    if (visited[w]) continue;

    if (query_reachability<hash_range>(graph, graph.graph_.nodes_[w], v, visited)) {
      return true;
    }
  }
  return false;
}


void remove_transitive_edges(ordering_graph& graph);
} // namespace soro::simulation