//! Measures only the pathfinding (`Pathfinding::find_path`), without the route caches of
//! SPSN / VolCGR and without booking resources, so each search can be repeated.
//!
//! Plans:
//! - `100.cp`: long contacts with long delays, with a delay-heuristic row per node.
//! - tvgutil LEO Ring Roads (cubesats + ground stations, short passes), routed from the
//!   first to the last ground station (see `common::LEO_PLANS`). They have no delay matrix,
//!   so only the bucket heuristic is benchmarked on them.
//!
//! This uses a single query per plan; `query_sweep` samples many.
//!
//! The expansions of one warm search of each benchmark are also counted (outside of the
//! timings, see `common::Counting`), printed as a table per plan and written to
//! `target/criterion/expansions.csv`.

mod common;

use a_sabr::{
    bundle::Bundle,
    contact_plan::ContactPlan,
    distance::{astar::AStar, sabr::SABR},
    multigraph::{Multigraph, RoutableNodeRef},
    node_manager::{bucket_heuristic::BucketHeuristicManager, none::NoManagement},
    pathfinding::{
        AStarNodeParenting, ContactParenting, HybridParenting, NodeParenting, Pathfinding,
    },
    types::Date,
};
use common::{CM, CountedContactParenting, CountedHybridParenting, CountedNodeParenting, DelayNM};
use criterion::{
    BatchSize, BenchmarkGroup, Criterion, black_box, criterion_group, criterion_main,
    measurement::WallTime,
};
use generativity::make_guard;

type BucketNM = BucketHeuristicManager<NoManagement>;

const BUNDLE: Bundle = Bundle {
    priority: 0,
    size: 1_000,
    expiration: 24060,
};
const BUCKET_COUNTS: [usize; 5] = [64, 256, 512, 1024, 4096];

/// A search to benchmark: plan, source and destination node IDs, and routing time.
struct Scenario {
    name: &'static str,
    plan: ContactPlan<NoManagement, CM>,
    source: usize,
    dest: usize,
    time: Date,
}

/// Routing from ground station `source` to `dest` on the LEO plan `name` (see
/// `common::LEO_PLANS`), starting at the first contact.
fn tvg_scenario(name: &'static str, source: &str, dest: &str) -> Scenario {
    let (_, path, zero_delays) = common::LEO_PLANS
        .into_iter()
        .find(|(n, ..)| *n == name)
        .unwrap();
    let (plan, names) = common::load_tvg(path, zero_delays);
    Scenario {
        name,
        source: common::node_index(&names, source),
        dest: common::node_index(&names, dest),
        time: common::plan_span(&plan).0,
        plan,
    }
}

/// Expansions of one search of each benchmark: (plan, benchmark, expansions).
static EXPANSIONS: std::sync::Mutex<Vec<(&str, String, usize)>> = std::sync::Mutex::new(Vec::new());

/// Prints the expansions of each plan as a table (parenting x pathfinder) and writes them
/// all to `expansions.csv`, next to the criterion reports.
fn report_expansions() {
    use std::fmt::Write as _;

    let expansions = EXPANSIONS.lock().unwrap();
    let mut plans: Vec<&str> = expansions.iter().map(|(plan, ..)| *plan).collect();
    plans.dedup();
    let mut csv = String::from("plan,parenting,pathfinder,expanded\n");
    for plan in plans {
        // (pathfinder, expansions of node, hybrid and contact parenting), in run order
        let mut rows: Vec<(&str, [Option<usize>; 3])> = Vec::new();
        for (_, name, count) in expansions.iter().filter(|(p, ..)| *p == plan) {
            let (parenting, pathfinder) = name.split_once('/').unwrap();
            let col = ["Node", "Hybrid", "Contact"]
                .iter()
                .position(|p| *p == parenting)
                .unwrap();
            let row = match rows.iter().position(|(p, _)| *p == pathfinder) {
                Some(row) => row,
                None => {
                    rows.push((pathfinder, [None; 3]));
                    rows.len() - 1
                }
            };
            rows[row].1[col] = Some(*count);
            writeln!(csv, "{plan},{parenting},{pathfinder},{count}").unwrap();
        }
        println!("\nExpanded nodes / labels, {plan}:");
        println!("{:<20}{:>10}{:>10}{:>10}", "", "Node", "Hybrid", "Contact");
        for (pathfinder, counts) in rows {
            let counts = counts.map(|c| c.map_or("-".to_string(), |c| c.to_string()));
            println!(
                "{pathfinder:<20}{:>10}{:>10}{:>10}",
                counts[0], counts[1], counts[2]
            );
        }
    }
    // the bench runs from target/<profile>/deps/
    let path = std::env::current_exe()
        .unwrap()
        .ancestors()
        .nth(3)
        .unwrap()
        .join("criterion/expansions.csv");
    std::fs::write(&path, csv).unwrap();
    println!("\nwritten to {}", path.display());
}

/// Benchmarks repeated searches of `$finder` for `$scenario`, on `$plan`.
/// One search is done beforehand, so lazily computed heuristics are warm, then a second one
/// with `$counted` (the same pathfinder counting its expansions) counts the expansions.
macro_rules! bench_find_path {
    ($group:expr, $name:expr, $scenario:expr, $plan:expr, $finder:ty, $counted:ty) => {{
        let Scenario {
            name: plan_name,
            source,
            dest,
            time,
            ..
        } = $scenario;
        make_guard!(id);
        let mut graph = Multigraph::new(id, $plan).unwrap();
        let source = graph
            .node_id_ref((*source).into())
            .unwrap()
            .internal()
            .unwrap();
        let mut dest = graph
            .node_id_ref((*dest).into())
            .unwrap()
            .routable()
            .unwrap();
        let mut finder = <$finder>::new();

        let found = finder
            .find_path(&mut graph, *time, source, &BUNDLE, &mut dest, None)
            .unwrap()
            .is_some();
        assert!(found, "{}: no route", $name);

        common::reset_expanded();
        let reached = <$counted>::new()
            .find_path(&mut graph, *time, source, &BUNDLE, &mut dest, None)
            .unwrap()
            .and_then(|output| output.full_path_rev(dest, &graph).map(|_| ()))
            .is_some();
        EXPANSIONS
            .lock()
            .unwrap()
            .push((*plan_name, $name.to_string(), common::expanded(reached)));

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

/// `bench_find_path` with node, hybrid and contact parenting. `$distance` compares the
/// paths reaching the same node (or contact), `$queue` orders the priority queue.
macro_rules! bench_parentings {
    ($group:expr, $label:expr, $scenario:expr, $plan:expr, $NM:ty, $distance:ty, $queue:ty) => {{
        bench_find_path!(
            $group,
            format!("Node/{}", $label),
            $scenario,
            $plan.clone(),
            NodeParenting<$distance, $queue>,
            CountedNodeParenting<$distance, $queue>
        );
        bench_find_path!(
            $group,
            format!("Hybrid/{}", $label),
            $scenario,
            $plan.clone(),
            HybridParenting<$distance, $NM, CM, $queue>,
            CountedHybridParenting<$distance, $NM, CM, $queue>
        );
        bench_find_path!(
            $group,
            format!("Contact/{}", $label),
            $scenario,
            $plan.clone(),
            ContactParenting<$NM, CM, $distance, RoutableNodeRef, $queue>,
            CountedContactParenting<$NM, CM, $distance, RoutableNodeRef, $queue>
        );
    }};
}

/// SABR and bucket A* (warm, then cold with node parenting) for `scenario`.
fn bench_scenario(group: &mut BenchmarkGroup<WallTime>, scenario: &Scenario) {
    bench_parentings!(
        group,
        "SABR",
        scenario,
        scenario.plan,
        NoManagement,
        SABR,
        SABR
    );

    for n_buckets in BUCKET_COUNTS {
        let bucket_plan = BucketNM::wrap_plan(scenario.plan.clone(), n_buckets);
        let label = format!("AStar-bucket{n_buckets}");
        bench_parentings!(
            group,
            label,
            scenario,
            bucket_plan,
            BucketNM,
            SABR,
            AStar<SABR>
        );
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
                    let mut finder = AStarNodeParenting::<SABR>::new();
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
    let long_delays = Scenario {
        name: "100cp",
        plan: common::load_100cp(),
        source: 0,
        dest: 60,
        time: 60,
    };
    let mut group = c.benchmark_group(long_delays.name);
    bench_scenario(&mut group, &long_delays);
    let delay_plan = common::load_100cp_delay();
    bench_parentings!(
        group,
        "AStar-delay",
        &long_delays,
        delay_plan,
        DelayNM,
        SABR,
        AStar<SABR>
    );
    group.finish();

    // ---- LEO Ring Roads: ground station to ground station through the cubesats ----
    // (name, first ground station, last ground station). IDs are zero-padded to the width
    // of the largest index, so up to 10 ground stations they have a single digit.
    let leo: Vec<Scenario> = [
        ("sample1", "gs00", "gs39"),
        ("rr0_s40_g40", "gs00", "gs39"),
        ("rrs_s40_g40", "gs00", "gs39"),
        ("rrs_s80_g20", "gs00", "gs19"),
        ("rrs_s80_g80", "gs00", "gs79"),
        ("rrs_s80_g05", "gs0", "gs4"),
        ("rrs_s80_g10", "gs0", "gs9"),
    ]
    .into_iter()
    .map(|(name, source, dest)| tvg_scenario(name, source, dest))
    .collect();
    for scenario in &leo {
        let mut group = c.benchmark_group(scenario.name);
        bench_scenario(&mut group, scenario);
        group.finish();
    }

    report_expansions();
}

criterion_group! {
    name=benches;
    config=Criterion::default().sample_size(50);
    targets=benchmark
}
criterion_main!(benches);
