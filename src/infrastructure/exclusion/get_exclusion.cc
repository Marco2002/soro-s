#include "soro/infrastructure/exclusion/get_exclusion.h"

#include "utl/concat.h"
#include "utl/enumerate.h"
#include "utl/erase_duplicates.h"
#include "utl/parallel_for.h"
#include "utl/timer.h"

#include "soro/infrastructure/exclusion/exclusion_elements.h"
#include "soro/infrastructure/exclusion/get_exclusion_graph.h"
#include "soro/infrastructure/exclusion/read_cliques.h"
#include "soro/infrastructure/exclusion/exclusion_section.h"
#include "soro/infrastructure/infrastructure.h"

namespace soro::infra {

soro::vector<soro::vector<exclusion_set::id>> get_irs_to_exclusion_sets(
    soro::vector<interlocking_route::ids> const& exclusion_sets,
    soro::size_t const interlocking_route_count) {
  utl::scoped_timer const timer("generating irs to exclusion sets mapping");

  soro::vector<soro::vector<exclusion_set::id>> irs_to_exclusion_sets(
      interlocking_route_count);

  for (auto const [id, exclusion_set] : utl::enumerate(exclusion_sets)) {
    for (auto const ir_id : exclusion_set) {
      irs_to_exclusion_sets[ir_id].emplace_back(
          static_cast<exclusion_set::id>(id));
    }
  }

  return irs_to_exclusion_sets;
}

// for every element returns the interlocking routes using that element
soro::vector<interlocking_route::ids> get_closed_element_used_by(
    soro::vector<element::ids> const& closed_exclusion_elements,
    soro::size_t const element_count) {
  utl::scoped_timer const timer("generating element used by mapping");

  auto const ir_count = closed_exclusion_elements.size();

  using result_t = soro::vector<interlocking_route::ids>;
  result_t element_used_by(element_count);

  // Set up the thread results ...
  auto const thread_count = std::thread::hardware_concurrency();
  soro::vector<result_t> thread_results(thread_count, result_t(element_count));

  // ... thread lambda ...

  auto const generate_used_by = [&](soro::size_t const t_id) {
    auto const batch_size = ir_count / thread_count;
    auto const from = static_cast<interlocking_route::id>(t_id * batch_size);
    auto const to =
        t_id == thread_count - 1 ? ir_count : ((t_id + 1) * batch_size);

    for (interlocking_route::id ir_id = from; ir_id < to; ++ir_id) {
      for (auto const e_id : closed_exclusion_elements[ir_id]) {
        thread_results[t_id][e_id].emplace_back(ir_id);
      }
    }
  };

  // ... threads.

  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (auto t = 0U; t < thread_count; ++t) {
    threads.emplace_back(generate_used_by, t);
  }

  std::for_each(begin(threads), end(threads), [](auto& t) { t.join(); });

  timer.print("threads finished");

  // Combine all thread results

  utl::parallel_for_run(element_used_by.size(), [&](auto&& id) {
    auto const ir_id = static_cast<interlocking_route::id>(id);
    for (auto t = 0U; t < thread_count; ++t) {
      utl::concat(element_used_by[ir_id], thread_results[t][ir_id]);
    }
  });

  timer.print("thread results combined");

  utl::parallel_for_run(element_used_by.size(), [&](auto&& id) {
    auto const ir_id = static_cast<interlocking_route::id>(id);
    utl::erase_duplicates(element_used_by[ir_id]);
  });

  timer.print("duplicates removed");

  utls::ensure(element_used_by.size() == element_count,
               "Mapping has to exist for every element");

  return element_used_by;
}

std::tuple<vector<exclusion_section>, soro::vector<soro::vector<exclusion_section::id>>, std::unordered_map<element_id, exclusion_section::id>> get_exclusion_sections(const soro::vector<section>& sections) {
  soro::vector<exclusion_section> exclusion_sections = {};
  std::unordered_map<element_id, exclusion_section::id> cross_sections = {};
  soro::vector<soro::vector<exclusion_section::id>> section_to_exclusion_sections(sections.size());

  auto add_cross_section = [&exclusion_sections, &cross_sections](auto const& element, exclusion_section::id& global_id) {
    if(cross_sections.find(element->id()) == cross_sections.end()) {
      exclusion_sections.emplace_back(element->id(), element->id(), global_id);
      cross_sections[element->id()] = global_id;
      ++global_id;
    }
  };

  exclusion_section::id global_id = 0;
  for(auto const& section : sections) {
    if(section.first_rising()->type() == type::CROSS) {
      add_cross_section(section.first_rising(), global_id);
    }
    if(section.first_falling()->type() == type::CROSS) {
      add_cross_section(section.first_falling(), global_id);
    }

    auto iterator = section.iterate<direction::Rising, skip::No>();

    auto previous_element = *(iterator.begin());
    for(auto const element : iterator) {
      if(element->type() == type::MAIN_SIGNAL || element->type() == type::HALT) {
        exclusion_sections.emplace_back(previous_element->id(), element->id(), global_id);
        section_to_exclusion_sections[section.id_].emplace_back(global_id);
        ++global_id;
        previous_element = element;
      }
    }
    exclusion_sections.emplace_back(previous_element->id(), section.last_rising()->id(), global_id);
    section_to_exclusion_sections[section.id_].emplace_back(global_id);
    ++global_id;
  }

  return {exclusion_sections, section_to_exclusion_sections, cross_sections};
}

exclusion get_exclusion(infrastructure_t const& infra_t,
                        std::filesystem::path const& clique_path,
                        option<exclusion_elements> const exclusion_elements,
                        option<exclusion_graph> const exclusion_graph,
                        option<exclusion_set> const exclusion_sets) {
  infrastructure const infra(&infra_t);

  exclusion ex;

  utl::scoped_timer const timer("calculating exclusion graph");
  if (exclusion_elements) {
    ex.exclusion_elements_.closed_ = get_closed_exclusion_elements(infra);
    ex.exclusion_elements_.open_ =
        get_open_exclusion_elements(ex.exclusion_elements_.closed_, infra);
  }

  if (exclusion_elements && exclusion_graph) {
    auto const closed_element_used_by = get_closed_element_used_by(
        ex.exclusion_elements_.closed_, infra->graph_.elements_.size());

    ex.exclusion_graph_ = get_exclusion_graph(ex.exclusion_elements_.closed_,
                                              closed_element_used_by, infra);
  }
  timer.print("done calculating exclusion graph");


  if(exclusion_sets) {
    ex.exclusion_sets_ = read_cliques(clique_path);
    ex.irs_to_exclusion_sets_ = get_irs_to_exclusion_sets(
        ex.exclusion_sets_, infra->interlocking_.routes_.size());
  }

  utl::scoped_timer const timer2("calculating exclusion sections");
  auto const [exclusion_sections, section_to_exclusion_sections, cross_sections] = get_exclusion_sections(infra->graph_.sections_);
  ex.exclusion_sections_ = exclusion_sections;
  ex.section_to_exclusion_sections_ = section_to_exclusion_sections;
  ex.cross_sections_ = cross_sections;
  timer2.print("done calculating exclusion sections");
  return ex;
}

}  // namespace soro::infra
