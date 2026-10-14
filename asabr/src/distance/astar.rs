use core::{cmp::Ordering, marker::PhantomData};

use crate::{
    bundle::Bundle,
    contact_manager::ContactManager,
    multigraph::Multigraph,
    node_manager::{NodeHeuristic, NodeManager},
    pathfinding::{HybridParentingOrd, destination::FindableDest},
    paths::PathFragment,
    types::Date,
};

use super::Distance;

/// Wrap a `Distance` metric `D` with an A* heuristic: order `PathFragment`s
/// by `f = g + h` instead of `g` alone, then by `D`. `g` is the arrival time (`recv.end`)
/// and `h` comes from `NodeHeuristic::get_heuristic`, evaluated once per queued path.
#[derive(Debug, Default)]
pub struct AStar<D> {
    _phantom: PhantomData<D>,
}

impl<
    'id,
    NM: NodeManager + NodeHeuristic,
    CM: ContactManager,
    D: Distance<'id, NM, CM, De>,
    De: FindableDest<'id, NM, CM>,
> Distance<'id, NM, CM, De> for AStar<D>
{
    /// Paths reaching the same node are compared by `D` alone: the heuristic can
    /// depend on the arrival time, so comparing by `g + h` could keep a later arrival.
    #[inline(always)]
    fn cmp(
        first: &PathFragment<'id>,
        second: &PathFragment<'id>,
        graph: &Multigraph<'id, NM, CM>,
        bundle: &Bundle,
        destination: &De,
    ) -> Ordering {
        D::cmp(first, second, graph, bundle, destination)
    }

    /// The queue is ordered by `f = g + h`, computed once per queued path. Ties on `f` go
    /// to the smaller `g` (via `cmp`, i.e. `D`), otherwise a time-dependent heuristic can
    /// expand a node from a non-optimal path.
    /// Without a single target (e.g. `DestAll`), `f` is 0 and only `D` orders the queue.
    #[inline(always)]
    fn queue_priority(
        path: &PathFragment<'id>,
        graph: &Multigraph<'id, NM, CM>,
        _bundle: &Bundle,
        destination: &De,
    ) -> Date {
        let Some(target) = destination.to_id(graph) else {
            return 0;
        };
        let target_nodeid = graph.routable_index_to_nodeid(target);
        path.recv
            .end
            .saturating_add(NM::get_heuristic(path, graph, target_nodeid))
    }
}

impl<NM: NodeManager, CM: ContactManager, D: HybridParentingOrd<NM, CM>> HybridParentingOrd<NM, CM>
    for AStar<D>
{
    #[inline(always)]
    fn keep_both<'id>(
        first: &PathFragment<'id>,
        second: &PathFragment<'id>,
        graph: &Multigraph<'id, NM, CM>,
        bundle: &Bundle,
    ) -> bool {
        // If both paths should be kept is about the accumulated cost, the heuristic doesn't affect.
        D::keep_both(first, second, graph, bundle)
    }
}
