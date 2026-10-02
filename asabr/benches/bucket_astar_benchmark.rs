use std::{
    fs::File,
    io::{BufRead, BufReader},
};

use a_sabr::{
    bundle::Bundle,
    contact_manager::segmentation::seg::SegmentationManager,
    contact_plan::asabr_file_lexer::parse_from_iter,
    node_manager::{bucket_heuristic::BucketHeuristicManager, none::NoManagement},
    pathfinding::{destination::RoutableDest, top_level::aliases::build_astar_router},
};
use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};

type NM = BucketHeuristicManager<NoManagement>;

/// Drops the delay-heuristic row (`node <id> <name> [..]`) so nodes only carry `NoManagement`.
fn strip_delay_row(line: String) -> String {
    match line.trim_start().starts_with("node") {
        true => line.split_once('[').map_or(line.clone(), |(node, _)| node.to_string()),
        false => line,
    }
}

pub fn benchmark(c: &mut Criterion) {
    let file = File::open("benches/astar_graphs/100.cp").unwrap();
    let lines = BufReader::new(file)
        .lines()
        .map(|l| strip_delay_row(l.unwrap()));
    let contact_plan = parse_from_iter::<NoManagement, SegmentationManager>(lines).unwrap();

    let source = 0.into();
    let destinatation = 60.into();
    let bundle = Bundle {
        priority: 0,
        size: 1_000,
        expiration: 24060,
    };
    let curr_time = 60;

    let mut router_types = vec![
        "SpsnNodeParentingAStar",
        "SpsnHybridParentingAStar",
        "SpsnContactParentingAStar",
    ];

    router_types.extend([
        "VolCgrNodeParentingAStar",
        "VolCgrHybridParentingAStar",
        "VolCgrContactParentingAStar",
    ]);

    let bucket_counts = [64, 256, 1024];

    let mut group = c.benchmark_group("BucketRouters");

    for n_buckets in bucket_counts {
        for router_type in &router_types {
            group.bench_function(format!("{router_type}/{n_buckets}"), |b| {
                b.iter_batched_ref(
                    || {
                        // a fresh table per batch: the destination's table is computed
                        // lazily during the first route, inside the measured routine
                        let contact_plan = NM::wrap_plan(contact_plan.clone(), n_buckets);
                        match unsafe { build_astar_router::<3, _, _>(router_type, contact_plan) } {
                            Ok((graph, router)) => {
                                let source = graph.node_id_ref(source).unwrap().try_into().unwrap();
                                let dest = graph
                                    .node_id_ref(destinatation)
                                    .unwrap()
                                    .routable()
                                    .unwrap();
                                (graph, router, source, dest)
                            }
                            Err(err) => panic!("{}", err),
                        }
                    },
                    |(graph, router, source, dest)| {
                        for _ in 0..100 {
                            black_box(dest.route(
                                black_box(graph),
                                black_box(&bundle),
                                black_box(&mut **router),
                                black_box(curr_time),
                                black_box(*source),
                                black_box(None),
                            ))
                            .unwrap();
                        }
                    },
                    BatchSize::SmallInput,
                );
            });
        }
    }
}

criterion_group! {
    name=benches;
    config=Criterion::default().sample_size(50);
    targets=benchmark
}
criterion_main!(benches);
