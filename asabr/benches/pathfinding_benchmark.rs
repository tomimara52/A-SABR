//! Measures only the pathfinding (`Pathfinding::find_path`), without the route caches of
//! SPSN / VolCGR and without booking resources, so each search can be repeated.
//!
//! Plans:
//! - `100.cp`: long contacts with long delays, with a delay-heuristic row per node.
//! - tvgutil LEO Ring Roads (cubesats + ground stations, short passes), routed from the
//!   first to the last ground station: `sample1.json` with its delays zeroed, and the
//!   scenarios of `leo_scenarios/` (see its README). They have no delay matrix, so only the
//!   bucket heuristic is benchmarked on them.

use std::{
    fs::File,
    io::{BufRead, BufReader},
};

use a_sabr::{
    bundle::Bundle,
    contact_manager::segmentation::seg::SegmentationManager,
    contact_plan::{
        ContactPlan, asabr_file_lexer::parse_from_iter,
        from_tvgutil_file::TVGUtilContactPlan,
    },
    distance::{astar::AStar, sabr::SABR},
    multigraph::{Multigraph, RoutableNodeRef},
    node_manager::{
        bucket_heuristic::BucketHeuristicManager, delay_heuristic::DelayHeuristicManager,
        none::NoManagement,
    },
    pathfinding::{ContactParenting, HybridParenting, NodeParenting, Pathfinding},
    types::Date,
};
use criterion::{
    BatchSize, BenchmarkGroup, Criterion, black_box, criterion_group, criterion_main,
    measurement::WallTime,
};
use generativity::make_guard;
use serde_json::Value;

type CM = SegmentationManager;
type DelayNM = DelayHeuristicManager<NoManagement>;
type BucketNM = BucketHeuristicManager<NoManagement>;

const BUNDLE: Bundle = Bundle {
    priority: 0,
    size: 1_000,
    expiration: 24060,
};
const BUCKET_COUNTS: [usize; 3] = [64, 256, 1024];

/// A search to benchmark: plan, source and destination node IDs, and routing time.
struct Scenario {
    name: &'static str,
    plan: ContactPlan<NoManagement, CM>,
    source: usize,
    dest: usize,
    time: Date,
}

fn read_lines(path: &str) -> impl Iterator<Item = String> {
    let file = File::open(path).unwrap();
    BufReader::new(file).lines().map(|l| l.unwrap())
}

/// Drops the delay-heuristic row (`node <id> <name> [..]`) so nodes only carry `NoManagement`.
fn strip_delay_row(line: String) -> String {
    match line.trim_start().starts_with("node") {
        true => line.split_once('[').map_or(line.clone(), |(node, _)| node.to_string()),
        false => line,
    }
}

/// Routing from ground station `source` to `dest` on the tvgutil P-TVG at `path`,
/// starting at the first contact. With `zero_delays`, every contact delay is set to 0.
fn tvg_scenario(
    name: &'static str,
    path: &str,
    source: &str,
    dest: &str,
    zero_delays: bool,
) -> Scenario {
    let file = File::open(path).unwrap();
    let mut json: Value = serde_json::from_reader(BufReader::new(file)).unwrap();
    if zero_delays {
        // contact = [tx, rx, start, end, [[_, confidence, [[start, rate, delay], ..]], ..]]
        for edge in json["edges"].as_array_mut().unwrap() {
            for contact in edge["contacts"].as_array_mut().unwrap() {
                for generation in contact[4].as_array_mut().unwrap() {
                    for characteristic in generation[2].as_array_mut().unwrap() {
                        characteristic[2] = 0.0.into();
                    }
                }
            }
        }
    }
    // node names are only kept with the "debug" feature: the parser numbers the
    // vertices in the order of the JSON object, so look them up there
    let vertices = json["vertices"].as_object().unwrap();
    let id = |node: &str| {
        vertices
            .keys()
            .position(|k| k == node)
            .unwrap_or_else(|| panic!("{name}: no node {node}"))
    };
    let (source, dest) = (id(source), id(dest));
    let plan: ContactPlan<NoManagement, CM> = TVGUtilContactPlan::parse(json).unwrap();
    let time = plan.contacts.iter().map(|(c, _, _)| c.lifespan.start).min().unwrap();
    Scenario {
        name,
        plan,
        source,
        dest,
        time,
    }
}

/// Benchmarks repeated searches of `$finder` for `$scenario`, on `$plan`.
/// One search is done beforehand, so lazily computed heuristics are warm.
macro_rules! bench_find_path {
    ($group:expr, $name:expr, $scenario:expr, $plan:expr, $finder:ty) => {{
        let Scenario {
            source, dest, time, ..
        } = $scenario;
        make_guard!(id);
        let mut graph = Multigraph::new(id, $plan).unwrap();
        let source = graph.node_id_ref((*source).into()).unwrap().internal().unwrap();
        let mut dest = graph.node_id_ref((*dest).into()).unwrap().routable().unwrap();
        let mut finder = <$finder>::new();

        let found = finder
            .find_path(&mut graph, *time, source, &BUNDLE, &mut dest, None)
            .unwrap()
            .is_some();
        assert!(found, "{}: no route", $name);

        $group.bench_function($name, |b| {
            b.iter(|| {
                black_box(
                    finder
                        .find_path(
                            black_box(&mut graph),
                            black_box(*time),
                            source,
                            black_box(&BUNDLE),
                            &mut dest,
                            None,
                        )
                        .unwrap(),
                );
            })
        });
    }};
}

/// `bench_find_path` with node, hybrid and contact parenting.
macro_rules! bench_parentings {
    ($group:expr, $label:expr, $scenario:expr, $plan:expr, $NM:ty, $distance:ty) => {{
        bench_find_path!(
            $group,
            format!("Node/{}", $label),
            $scenario,
            $plan.clone(),
            NodeParenting<$distance>
        );
        bench_find_path!(
            $group,
            format!("Hybrid/{}", $label),
            $scenario,
            $plan.clone(),
            HybridParenting<$distance, $NM, CM>
        );
        bench_find_path!(
            $group,
            format!("Contact/{}", $label),
            $scenario,
            $plan.clone(),
            ContactParenting<$NM, CM, $distance, RoutableNodeRef>
        );
    }};
}

/// SABR and bucket A* (warm, then cold with node parenting) for `scenario`.
fn bench_scenario(group: &mut BenchmarkGroup<WallTime>, scenario: &Scenario) {
    bench_parentings!(group, "SABR", scenario, scenario.plan, NoManagement, SABR);

    for n_buckets in BUCKET_COUNTS {
        let bucket_plan = BucketNM::wrap_plan(scenario.plan.clone(), n_buckets);
        let label = format!("AStar-bucket{n_buckets}");
        bench_parentings!(group, label, scenario, bucket_plan, BucketNM, AStar<SABR>);
    }

    // First search with a fresh bucket table: includes computing the destination's table.
    for n_buckets in BUCKET_COUNTS {
        group.bench_function(format!("Node/AStar-bucket{n_buckets}-cold"), |b| {
            b.iter_batched_ref(
                || {
                    let plan = BucketNM::wrap_plan(scenario.plan.clone(), n_buckets);
                    let graph = unsafe { Multigraph::new_unguarded(plan) }.unwrap();
                    let source = graph.node_id_ref(scenario.source.into()).unwrap();
                    let dest = graph.node_id_ref(scenario.dest.into()).unwrap();
                    (graph, source.internal().unwrap(), dest.routable().unwrap())
                },
                |(graph, source, dest)| {
                    let mut finder = NodeParenting::<AStar<SABR>>::new();
                    black_box(
                        finder
                            .find_path(graph, scenario.time, *source, &BUNDLE, dest, None)
                            .unwrap()
                            .is_some(),
                    );
                },
                BatchSize::SmallInput,
            );
        });
    }
}

pub fn benchmark(c: &mut Criterion) {
    // ---- 100.cp ----
    let path = "benches/astar_graphs/100.cp";
    let long_delays = Scenario {
        name: "100cp",
        plan: parse_from_iter(read_lines(path).map(strip_delay_row)).unwrap(),
        source: 0,
        dest: 60,
        time: 60,
    };
    let mut group = c.benchmark_group(long_delays.name);
    bench_scenario(&mut group, &long_delays);
    let delay_plan = parse_from_iter::<DelayNM, CM>(read_lines(path)).unwrap();
    bench_parentings!(group, "AStar-delay", &long_delays, delay_plan, DelayNM, AStar<SABR>);
    group.finish();

    // ---- LEO Ring Roads: ground station to ground station through the cubesats ----
    // sample1.json has non-physical delays (up to ~5400 s, even on ISLs): zero them
    let mut leo = vec![tvg_scenario(
        "sample1",
        "benches/ptvg_files/sample1.json",
        "gs00",
        "gs39",
        true,
    )];
    // (name, first ground station, last ground station), generated by
    // leo_scenarios/generate.sh. IDs are zero-padded to the width of the largest index,
    // so up to 10 ground stations they have a single digit.
    for (name, source, dest) in [
        ("rr0_s40_g40", "gs00", "gs39"),
        ("rrs_s40_g40", "gs00", "gs39"),
        ("rrs_s80_g20", "gs00", "gs19"),
        ("rrs_s80_g80", "gs00", "gs79"),
        ("rrs_s80_g05", "gs0", "gs4"),
        ("rrs_s80_g10", "gs0", "gs9"),
    ] {
        let path = format!("benches/leo_scenarios/{name}.json");
        leo.push(tvg_scenario(name, &path, source, dest, false));
    }
    for scenario in &leo {
        let mut group = c.benchmark_group(scenario.name);
        bench_scenario(&mut group, scenario);
        group.finish();
    }
}

criterion_group! {
    name=benches;
    config=Criterion::default().sample_size(50);
    targets=benchmark
}
criterion_main!(benches);
