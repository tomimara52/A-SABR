//! `query_sweep` with criterion: the same random queries (source, destination, start time)
//! on each plan, with every pathfinder, but timed by criterion. Each iteration runs all the
//! queries of a plan, so a benchmark measures the total time of the query set (and the
//! throughput in queries per second), not each query: use `query_sweep` for per-query
//! times, expansions and arrival checks.
//!
//! `cargo bench --bench query_benchmark [-- <filter>]`
//!
//! The queries are sampled as in `query_sweep` (see `common::plan_queries`), with the same
//! environment variables: `SWEEP_QUERIES` (default 100), `SWEEP_SEED` (default 1) and
//! `SWEEP_PLANS` (default: all). Queries without a route are included: they are part of
//! the workload.
//!
//! All the queries are searched once before measuring, so the bucket tables of their
//! destinations are already computed: only warm searches are measured.
//! Benchmarks are grouped by plan as `sweep_<plan>`, apart from `pathfinding_benchmark`'s.

mod common;

use a_sabr::{
    bundle::Bundle,
    contact_plan::ContactPlan,
    distance::{astar::AStar, sabr::SABR},
    multigraph::{Multigraph, RoutableNodeRef},
    node_manager::{bucket_heuristic::BucketHeuristicManager, none::NoManagement},
    pathfinding::{ContactParenting, HybridParenting, NodeParenting, Pathfinding},
    types::Date,
};
use common::{CM, DelayNM, Query};
use criterion::{
    BenchmarkGroup, Criterion, Throughput, black_box, criterion_group, criterion_main,
    measurement::WallTime,
};
use generativity::make_guard;

type BucketNM = BucketHeuristicManager<NoManagement>;

const BUNDLE: Bundle = Bundle {
    priority: 0,
    size: 1_000,
    expiration: Date::MAX,
};
const BUCKET_COUNTS: [usize; 5] = [64, 256, 512, 1024, 4096];

/// Benchmarks searching every query of `$queries` with `$finder` on `$plan`, after one
/// unmeasured pass over all of them.
macro_rules! bench_queries {
    ($group:expr, $name:expr, $queries:expr, $plan:expr, $finder:ty) => {{
        make_guard!(id);
        let mut graph = Multigraph::new(id, $plan).unwrap();
        let mut requests: Vec<_> = $queries
            .iter()
            .map(|query: &Query| {
                let source = graph
                    .node_id_ref(query.source.into())
                    .unwrap()
                    .internal()
                    .unwrap();
                let dest = graph
                    .node_id_ref(query.dest.into())
                    .unwrap()
                    .routable()
                    .unwrap();
                (source, dest, query.time)
            })
            .collect();
        let mut finder = <$finder>::new();

        for (source, dest, time) in &mut requests {
            finder
                .find_path(&mut graph, *time, *source, &BUNDLE, dest, None)
                .unwrap();
        }

        $group.bench_function($name, |b| {
            b.iter(|| {
                for (source, dest, time) in &mut requests {
                    black_box(
                        finder
                            .find_path(
                                black_box(&mut graph),
                                black_box(*time),
                                *source,
                                black_box(&BUNDLE),
                                dest,
                                None,
                            )
                            .unwrap(),
                    );
                }
            })
        });
    }};
}

/// `bench_queries` with node, hybrid and contact parenting. `$plan` is evaluated for each
/// of them, so each gets its own plan (a cloned bucket plan shares its tables).
macro_rules! bench_parentings {
    ($group:expr, $label:expr, $queries:expr, $plan:expr, $NM:ty, $queue:ty) => {{
        bench_queries!(
            $group,
            format!("Node/{}", $label),
            $queries,
            $plan,
            NodeParenting<SABR, $queue>
        );
        bench_queries!(
            $group,
            format!("Hybrid/{}", $label),
            $queries,
            $plan,
            HybridParenting<SABR, $NM, CM, $queue>
        );
        bench_queries!(
            $group,
            format!("Contact/{}", $label),
            $queries,
            $plan,
            ContactParenting<$NM, CM, SABR, RoutableNodeRef, $queue>
        );
    }};
}

/// SABR and bucket A* with every bucket count, on `queries`.
fn bench_plan(
    group: &mut BenchmarkGroup<WallTime>,
    queries: &[Query],
    plan: &ContactPlan<NoManagement, CM>,
) {
    group.throughput(Throughput::Elements(queries.len() as u64));
    bench_parentings!(group, "SABR", queries, plan.clone(), NoManagement, SABR);
    for n_buckets in BUCKET_COUNTS {
        bench_parentings!(
            group,
            format!("AStar-bucket{n_buckets}"),
            queries,
            BucketNM::wrap_plan(plan.clone(), n_buckets),
            BucketNM,
            AStar<SABR>
        );
    }
}

pub fn benchmark(c: &mut Criterion) {
    // ---- 100.cp: between any two nodes, also with the delay heuristic ----
    if common::plan_selected("100cp") {
        let plan = common::load_100cp();
        let nodes: Vec<usize> = (0..plan.realnodes.len()).collect();
        let queries = common::plan_queries("100cp", &plan, &nodes);
        let mut group = c.benchmark_group("sweep_100cp");
        bench_plan(&mut group, &queries, &plan);
        let delay_plan = common::load_100cp_delay();
        bench_parentings!(
            group,
            "AStar-delay",
            &queries,
            delay_plan.clone(),
            DelayNM,
            AStar<SABR>
        );
        group.finish();
    }

    // ---- LEO Ring Roads: between two ground stations ----
    for (name, path, zero_delays) in common::LEO_PLANS {
        if !common::plan_selected(name) {
            continue;
        }
        let (plan, names) = common::load_tvg(path, zero_delays);
        let queries = common::plan_queries(name, &plan, &common::ground_stations(&names));
        let mut group = c.benchmark_group(format!("sweep_{name}"));
        bench_plan(&mut group, &queries, &plan);
        group.finish();
    }
}

criterion_group! {
    name=benches;
    // each iteration runs a whole query set: 10 samples keep the run time reasonable
    config=Criterion::default().sample_size(10);
    targets=benchmark
}
criterion_main!(benches);
