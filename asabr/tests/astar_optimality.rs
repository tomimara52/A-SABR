//! `AStar<SABR>` must reach every destination as early as `SABR` (Dijkstra) does,
//! for each parenting, as long as the heuristic never overestimates the remaining delay.

use std::fmt::Write;

use a_sabr::bundle::Bundle;
use a_sabr::contact_manager::legacy::evl::EVLManager;
use a_sabr::distance::{astar::AStar, sabr::SABR};
use a_sabr::errors::ASABRError;
use a_sabr::mk_graph;
use a_sabr::node_manager::{delay_heuristic::DelayHeuristicManager, none::NoManagement};
use a_sabr::pathfinding::{ContactParenting, HybridParenting, NodeParenting, Pathfinding};
use a_sabr::types::Date;

type NM = DelayHeuristicManager<NoManagement>;
type CM = EVLManager;

const BUNDLE: Bundle = Bundle {
    priority: 0,
    size: 0,
    expiration: 1_000_000,
};

/// Small deterministic generator, so a failure can be reproduced from its seed.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self, bound: u64) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 33) % bound
    }
}

/// A contact: from, to, start, end, delay.
type Contact = (usize, usize, Date, Date, Date);

/// Builds an A-SABR contact plan. Each node's heuristic row is the minimal sum of
/// contact delays to every other node (unreachable nodes get 0): it ignores waiting
/// for contacts to open, so it never overestimates.
fn plan(nodes: usize, contacts: &[Contact]) -> String {
    const INF: Date = Date::MAX / 4;
    let mut dist = vec![vec![INF; nodes]; nodes];
    for (i, row) in dist.iter_mut().enumerate() {
        row[i] = 0;
    }
    for &(from, to, _, _, delay) in contacts {
        dist[from][to] = dist[from][to].min(delay);
    }
    for k in 0..nodes {
        for i in 0..nodes {
            for j in 0..nodes {
                dist[i][j] = dist[i][j].min(dist[i][k] + dist[k][j]);
            }
        }
    }

    let mut plan = String::new();
    for (i, row) in dist.iter().enumerate() {
        let row: Vec<String> = row
            .iter()
            .map(|&d| if d == INF { 0 } else { d }.to_string())
            .collect();
        writeln!(plan, "node {i} n{i} [{}]", row.join(", ")).unwrap();
    }
    for &(from, to, start, end, delay) in contacts {
        writeln!(plan, "contact {from} {to} {start} {end} 1 {delay}").unwrap();
    }
    plan
}

/// Random time-varying plan: contacts open at different times, so the earliest
/// arrival is often not the path with the smallest delay sum.
fn random_contacts(seed: u64, nodes: usize, count: usize) -> Vec<Contact> {
    let mut rng = Lcg(seed);
    (0..count)
        .filter_map(|_| {
            let from = rng.next(nodes as u64) as usize;
            let to = rng.next(nodes as u64) as usize;
            let start = rng.next(400) as Date;
            let end = start + 1 + rng.next(100) as Date;
            let delay = 1 + rng.next(30) as Date;
            (from != to).then_some((from, to, start, end, delay))
        })
        .collect()
}

/// Grid where each node is linked to its 4 neighbours, always available.
fn grid_contacts(width: usize, height: usize) -> Vec<Contact> {
    let id = |x: usize, y: usize| y * width + x;
    let mut contacts = Vec::new();
    for y in 0..height {
        for x in 0..width {
            if x + 1 < width {
                contacts.push((id(x, y), id(x + 1, y), 0, 100_000, 1));
                contacts.push((id(x + 1, y), id(x, y), 0, 100_000, 1));
            }
            if y + 1 < height {
                contacts.push((id(x, y), id(x, y + 1), 0, 100_000, 1));
                contacts.push((id(x, y + 1), id(x, y), 0, 100_000, 1));
            }
        }
    }
    contacts
}

/// Arrival time at `dest` from node 0 at t=0, `None` if no route.
macro_rules! arrival {
    ($graph:ident, $finder:expr, $dest:expr) => {{
        let source = $graph.node_id_ref(0.into())?.internal().unwrap();
        let mut destination = $graph.node_id_ref($dest.into())?.routable().unwrap();
        let mut finder = $finder;
        let out = finder.find_path(&mut $graph, 0, source, &BUNDLE, &mut destination, None)?;
        out.and_then(|out| out[$graph.routable_to_usize(destination)].map(|p| p.recv.end))
    }};
}

/// Checks every destination of the plan, for the three parentings.
fn check_plan(name: &str, nodes: usize, contacts: &[Contact]) -> Result<(), ASABRError> {
    let plan = plan(nodes, contacts);
    mk_graph!(graph, NM, CM, plan, raw);

    for dest in 1..nodes {
        let node_sabr = arrival!(graph, NodeParenting::<SABR>::new(), dest);
        let node_astar = arrival!(graph, NodeParenting::<AStar<SABR>>::new(), dest);
        assert_eq!(node_sabr, node_astar, "{name}, node {dest}: NodeParenting");

        let hybrid_sabr = arrival!(graph, HybridParenting::<SABR, NM, CM>::new(), dest);
        let hybrid_astar = arrival!(graph, HybridParenting::<AStar<SABR>, NM, CM>::new(), dest);
        assert_eq!(
            hybrid_sabr, hybrid_astar,
            "{name}, node {dest}: HybridParenting"
        );

        let contact_sabr = arrival!(graph, ContactParenting::<NM, CM, SABR, _>::new(), dest);
        let contact_astar = arrival!(
            graph,
            ContactParenting::<NM, CM, AStar<SABR>, _>::new(),
            dest
        );
        assert_eq!(
            contact_sabr, contact_astar,
            "{name}, node {dest}: ContactParenting"
        );
    }
    Ok(())
}

#[test]
fn astar_matches_dijkstra_on_grid() -> Result<(), ASABRError> {
    check_plan("grid 10x10", 100, &grid_contacts(10, 10))
}

#[test]
fn astar_matches_dijkstra_on_random_plans() -> Result<(), ASABRError> {
    for seed in 0..50 {
        check_plan(&format!("seed {seed}"), 20, &random_contacts(seed, 20, 150))?;
    }
    Ok(())
}
