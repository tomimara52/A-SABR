//! Runs many random queries (source, destination, start time) on each plan with every
//! pathfinder, and records per query the arrival time, the expansions and the search time.
//! Unlike `pathfinding_benchmark`, it does not use criterion: each search is timed directly.
//!
//! `cargo bench --bench query_sweep`
//!
//! Environment variables:
//! - `SWEEP_QUERIES` (default 100): queries per plan.
//! - `SWEEP_REPS` (default 5): warm repetitions of each search, the median is kept.
//! - `SWEEP_SEED` (default 1): seed of the query sampling.
//! - `SWEEP_PLANS`: comma-separated plans to run (default: all).
//!
//! Queries: on the LEO plans, between two different ground stations; on `100.cp`, between
//! two different nodes. The start time is uniform over the span of the plan, so some
//! queries have no route. The same queries are used for every pathfinder.
//!
//! Each query is searched once with the bucket table of its destination possibly not yet
//! computed (`first_ns`, and `first_to_dest` if it was the first query to that destination
//! on that graph), then `SWEEP_REPS` times warm (`warm_ns`, median). The expansions are
//! counted on one more, untimed, search (see `common::Counting`).
//! The rows are written to `target/query_sweep.csv`, and a summary per plan is printed:
//! medians over the queries with a route, and arrival-time mismatches with SABR (an
//! admissible heuristic must never give a later arrival).

mod common;

use std::{collections::HashSet, fmt::Write as _, hint::black_box, time::Instant};

use a_sabr::{
    bundle::Bundle,
    contact_plan::ContactPlan,
    distance::{astar::AStar, sabr::SABR},
    multigraph::{Multigraph, RoutableNodeRef},
    node_manager::{bucket_heuristic::BucketHeuristicManager, none::NoManagement},
    pathfinding::{ContactParenting, HybridParenting, NodeParenting, Pathfinding},
    types::Date,
};
use common::{
    CM, CountedContactParenting, CountedHybridParenting, CountedNodeParenting, DelayNM, Query,
    env_or,
};
use generativity::make_guard;

type BucketNM = BucketHeuristicManager<NoManagement>;

const BUNDLE: Bundle = Bundle {
    priority: 0,
    size: 1_000,
    expiration: Date::MAX,
};
const BUCKET_COUNTS: [usize; 5] = [64, 256, 512, 1024, 4096];

/// Result of one query with one pathfinder.
struct Row {
    plan: &'static str,
    query: usize,
    request: Query,
    pathfinder: String,
    parenting: &'static str,
    arrival: Option<Date>,
    expanded: usize,
    first_ns: u128,
    warm_ns: u128,
    first_to_dest: bool,
}

fn median(values: &mut [u128]) -> u128 {
    values.sort_unstable();
    values[values.len() / 2]
}

/// Runs every query of `$queries` on `$plan` with `$finder`, appending a `Row` per query.
/// `$counted` is the same pathfinder counting its expansions, kept out of the timings.
macro_rules! sweep {
    ($rows:expr, $reps:expr, $plan_name:expr, $queries:expr, $plan:expr, $label:expr, $parenting:expr, $finder:ty, $counted:ty) => {{
        make_guard!(id);
        let mut graph = Multigraph::new(id, $plan).unwrap();
        let mut finder = <$finder>::new();
        let mut counted = <$counted>::new();
        let mut seen_dests = HashSet::new();
        for (index, query) in $queries.iter().enumerate() {
            let source = graph
                .node_id_ref(query.source.into())
                .unwrap()
                .internal()
                .unwrap();
            let mut dest = graph
                .node_id_ref(query.dest.into())
                .unwrap()
                .routable()
                .unwrap();

            let start = Instant::now();
            let output = finder
                .find_path(&mut graph, query.time, source, &BUNDLE, &mut dest, None)
                .unwrap();
            let first_ns = start.elapsed().as_nanos();
            let arrival = output
                .as_ref()
                .and_then(|o| o.full_path_rev(dest, &graph))
                .and_then(|mut path| path.next())
                .map(|frag| frag.recv.end);

            let mut warm: Vec<u128> = (0..$reps)
                .map(|_| {
                    let start = Instant::now();
                    black_box(
                        finder
                            .find_path(&mut graph, query.time, source, &BUNDLE, &mut dest, None)
                            .unwrap(),
                    );
                    start.elapsed().as_nanos()
                })
                .collect();

            common::reset_expanded();
            counted
                .find_path(&mut graph, query.time, source, &BUNDLE, &mut dest, None)
                .unwrap();
            let expanded = common::expanded(arrival.is_some());

            $rows.push(Row {
                plan: $plan_name,
                query: index,
                request: *query,
                pathfinder: $label.to_string(),
                parenting: $parenting,
                arrival,
                expanded,
                first_ns,
                warm_ns: median(&mut warm),
                first_to_dest: seen_dests.insert(query.dest),
            });
        }
    }};
}

/// `sweep` with node, hybrid and contact parenting. `$plan` is evaluated for each of them,
/// so it must build a fresh plan: a cloned bucket plan shares its tables.
macro_rules! sweep_parentings {
    ($rows:expr, $reps:expr, $plan_name:expr, $queries:expr, $plan:expr, $label:expr, $NM:ty, $queue:ty) => {{
        sweep!($rows, $reps, $plan_name, $queries, $plan, $label, "Node",
            NodeParenting<SABR, $queue>, CountedNodeParenting<SABR, $queue>);
        sweep!($rows, $reps, $plan_name, $queries, $plan, $label, "Hybrid",
            HybridParenting<SABR, $NM, CM, $queue>,
            CountedHybridParenting<SABR, $NM, CM, $queue>);
        sweep!($rows, $reps, $plan_name, $queries, $plan, $label, "Contact",
            ContactParenting<$NM, CM, SABR, RoutableNodeRef, $queue>,
            CountedContactParenting<$NM, CM, SABR, RoutableNodeRef, $queue>);
    }};
}

/// SABR and bucket A* with every bucket count, on `queries`.
fn sweep_plan(
    rows: &mut Vec<Row>,
    reps: usize,
    name: &'static str,
    queries: &[Query],
    plan: &ContactPlan<NoManagement, CM>,
) {
    sweep_parentings!(
        rows,
        reps,
        name,
        queries,
        plan.clone(),
        "SABR",
        NoManagement,
        SABR
    );
    for n_buckets in BUCKET_COUNTS {
        let label = format!("AStar-bucket{n_buckets}");
        sweep_parentings!(
            rows,
            reps,
            name,
            queries,
            BucketNM::wrap_plan(plan.clone(), n_buckets),
            label,
            BucketNM,
            AStar<SABR>
        );
    }
}

fn percentile(values: &mut [f64], p: f64) -> f64 {
    values.sort_by(f64::total_cmp);
    values[((values.len() - 1) as f64 * p).round() as usize]
}

/// Prints, per pathfinder and parenting, medians over the queries with a route, compared
/// with SABR on the same query and parenting.
fn summarize(plan: &str, rows: &[Row]) {
    let rows: Vec<&Row> = rows.iter().filter(|r| r.plan == plan).collect();
    let n_queries = rows.iter().map(|r| r.query).max().map_or(0, |q| q + 1);
    let sabr = |parenting: &str, query: usize| {
        rows.iter()
            .find(|r| r.pathfinder == "SABR" && r.parenting == parenting && r.query == query)
            .unwrap()
    };
    let routed = rows
        .iter()
        .filter(|r| r.pathfinder == "SABR" && r.parenting == "Node" && r.arrival.is_some())
        .count();
    println!("\n{plan}: {n_queries} queries, {routed} with a route");
    println!(
        "{:<18}{:<9}{:>10}{:>10}{:>10}{:>12}{:>12}{:>12}{:>12}{:>10}",
        "", "", "expanded", "exp/SABR", "warm µs", "speedup", "p10", "p90", "table µs", "mismatch"
    );
    let mut labels: Vec<&str> = rows.iter().map(|r| r.pathfinder.as_str()).collect();
    labels.dedup();
    for label in labels {
        for parenting in ["Node", "Hybrid", "Contact"] {
            let group: Vec<&Row> = rows
                .iter()
                .copied()
                .filter(|r| r.pathfinder == label && r.parenting == parenting)
                .collect();
            let mismatches = group
                .iter()
                .filter(|r| r.arrival != sabr(parenting, r.query).arrival)
                .count();
            let with_route: Vec<&Row> = group
                .iter()
                .copied()
                .filter(|r| r.arrival.is_some())
                .collect();
            if with_route.is_empty() {
                continue;
            }
            let mut expanded: Vec<u128> = with_route.iter().map(|r| r.expanded as u128).collect();
            let mut warm: Vec<u128> = with_route.iter().map(|r| r.warm_ns).collect();
            let mut exp_ratio: Vec<f64> = with_route
                .iter()
                .map(|r| r.expanded as f64 / sabr(parenting, r.query).expanded.max(1) as f64)
                .collect();
            let mut speedup: Vec<f64> = with_route
                .iter()
                .map(|r| sabr(parenting, r.query).warm_ns as f64 / r.warm_ns as f64)
                .collect();
            // first search to a destination minus a warm one: computing its bucket table
            let mut table: Vec<u128> = group
                .iter()
                .filter(|r| r.first_to_dest)
                .map(|r| r.first_ns.saturating_sub(r.warm_ns))
                .collect();
            let table = match label.contains("bucket") {
                true => format!("{:.0}", median(&mut table) as f64 / 1000.0),
                false => "-".to_string(),
            };
            println!(
                "{label:<18}{parenting:<9}{:>10}{:>10.2}{:>10.1}{:>11.2}x{:>11.2}x{:>11.2}x{:>12}{:>10}",
                median(&mut expanded),
                percentile(&mut exp_ratio, 0.5),
                median(&mut warm) as f64 / 1000.0,
                percentile(&mut speedup, 0.5),
                percentile(&mut speedup, 0.1),
                percentile(&mut speedup, 0.9),
                table,
                mismatches,
            );
        }
    }
}

fn write_csv(rows: &[Row]) {
    let mut csv = String::from(
        "plan,query,pathfinder,parenting,source,dest,time,arrival,expanded,first_ns,warm_ns,first_to_dest\n",
    );
    for r in rows {
        let q = r.request;
        writeln!(
            csv,
            "{},{},{},{},{},{},{},{},{},{},{},{}",
            r.plan,
            r.query,
            r.pathfinder,
            r.parenting,
            q.source,
            q.dest,
            q.time,
            r.arrival.map_or(String::new(), |a| a.to_string()),
            r.expanded,
            r.first_ns,
            r.warm_ns,
            r.first_to_dest,
        )
        .unwrap();
    }
    // the bench runs from target/<profile>/deps/
    let path = std::env::current_exe()
        .unwrap()
        .ancestors()
        .nth(3)
        .unwrap()
        .join("query_sweep.csv");
    std::fs::write(&path, csv).unwrap();
    println!("\nwritten to {}", path.display());
}

fn main() {
    let reps = env_or("SWEEP_REPS", 5);
    let mut rows = Vec::new();
    let mut plans = Vec::new();

    // ---- 100.cp: between any two nodes, also with the delay heuristic ----
    if common::plan_selected("100cp") {
        eprintln!("sweeping 100cp");
        let plan = common::load_100cp();
        let nodes: Vec<usize> = (0..plan.realnodes.len()).collect();
        let queries = common::plan_queries("100cp", &plan, &nodes);
        sweep_plan(&mut rows, reps, "100cp", &queries, &plan);
        let delay_plan = common::load_100cp_delay();
        sweep_parentings!(
            rows,
            reps,
            "100cp",
            &queries,
            delay_plan.clone(),
            "AStar-delay",
            DelayNM,
            AStar<SABR>
        );
        plans.push("100cp");
    }

    // ---- LEO Ring Roads: between two ground stations ----
    for (name, path, zero_delays) in common::LEO_PLANS {
        if !common::plan_selected(name) {
            continue;
        }
        eprintln!("sweeping {name}");
        let (plan, names) = common::load_tvg(path, zero_delays);
        let queries = common::plan_queries(name, &plan, &common::ground_stations(&names));
        sweep_plan(&mut rows, reps, name, &queries, &plan);
        plans.push(name);
    }

    for plan in plans {
        summarize(plan, &rows);
    }
    write_csv(&rows);
}
