extern crate alloc;

use core::cmp::Ordering;

use crate::{
    bundle::Bundle, contact_manager::ContactManager, multigraph::Multigraph,
    node_manager::NodeManager, pathfinding::destination::FindableDest, paths::PathFragment,
    types::Date,
};

/// A* heuristic wrapper around another `Distance` metric.
pub mod astar;
/// Hop-count distance metric.
pub mod hop;
/// Priority queue used by distance-based pathfinding.
pub mod prio_queue;
/// SABR distance metric.
pub mod sabr;

/// A trait that allows RouteStages to define custom distance comparison strategies.
///
/// # Type Parameters
/// - `NM`: A type that implements the `NodeManager` trait.
/// - `CM`: A type that implements the `ContactManager` trait, representing the contact management
///   system used to manage and compare routes.
pub trait Distance<'id, NM, CM, D>
where
    Self: Sized,
    NM: NodeManager,
    CM: ContactManager,
    D: FindableDest<'id, NM, CM>,
{
    /// Compares the distances between two `RouteStage` instances.
    ///
    /// This method provides a total ordering of `RouteStage` instances based on
    /// their distances, returning an `Ordering` (`Less`, `Equal`, or `Greater`)
    /// based on whether the `first` route is shorter, equal to, or longer than
    /// the `second` route.
    ///
    /// # Parameters
    /// - `first`: The first route stage to compare.
    /// - `second`: The second route stage to compare.
    ///
    /// # Returns
    /// - `Ordering::Less` if `first` is shorter than `second`.
    /// - `Ordering::Equal` if `first` and `second` are the same.
    /// - `Ordering::Greater` if `first` is longer than `second`.
    fn cmp(
        first: &PathFragment<'id>,
        second: &PathFragment<'id>,
        graph: &Multigraph<'id, NM, CM>,
        bundle: &Bundle,
        destination: &D,
    ) -> Ordering;

    /// Priority of a path in the priority queue (smallest first), computed once when the
    /// path is inserted. Paths of equal priority are ordered by `cmp`.
    ///
    /// This is used to decide which path is expanded first: it only matters for distances
    /// that order the exploration differently from `cmp`, like `AStar`.
    /// Defaults to 0, so that only `cmp` orders the queue.
    #[inline(always)]
    fn queue_priority(
        _path: &PathFragment<'id>,
        _graph: &Multigraph<'id, NM, CM>,
        _bundle: &Bundle,
        _destination: &D,
    ) -> Date {
        0
    }
}

/// Compares two values by an extracted ordering key.
pub fn cmp_by<T, F: Fn(T) -> O, O: Ord>(a: T, b: T, f: F) -> Ordering {
    f(a).cmp(&f(b))
}
