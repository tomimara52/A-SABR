extern crate alloc;
use alloc::rc::Rc;

use crate::bundle::Bundle;
use crate::contact_manager::ContactManager;
use crate::contact_plan::asabr_file_lexer::parse_from_iter;
use crate::contact_plan::{ContactPlan, RealNode};
use crate::errors::ASABRError;
use crate::multigraph::Multigraph;
use crate::node::Node;
use crate::node_manager::NodeHeuristic;
use crate::parsing::LexFrom;
use crate::paths::PathFragment;
use crate::types::{Date, NodeID, TimeInterval};

use super::NodeManager;
use super::bucket_table::BucketTable;

/// `NodeManager` that behaves like `NoManagement` except that it 
/// divides the contact plan in buckets of equal duration and precomputes
/// reachability between those buckets
#[derive(Debug, Clone, Default)]
pub struct BucketHeuristicManager<NM: NodeManager> {
    inner: NM,
    bucket_table: Rc<BucketTable>,
}

impl<NM: NodeManager> BucketHeuristicManager<NM> {
    pub fn new(inner: NM, bucket_table: Rc<BucketTable>) -> Self {
        Self {
            inner,
            bucket_table: bucket_table.clone(),
        }
    }

    /// Builds the bucket table of `contact_plan` with `n_buckets` buckets and wraps each
    /// node manager so that they all share it. The table of each destination is computed
    /// the first time a route to it needs the heuristic.
    pub fn wrap_plan<CM: ContactManager>(
        contact_plan: ContactPlan<NM, CM>,
        n_buckets: usize,
    ) -> ContactPlan<Self, CM> {
        let table = Rc::new(BucketTable::from_plan(&contact_plan, n_buckets));

        let wrap = |Node { info, manager }: Node<NM>| Node {
            info,
            manager: Self::new(manager, table.clone()),
        };
        let ContactPlan {
            realnodes,
            vnodes,
            contacts,
        } = contact_plan;
        let realnodes = realnodes
            .into_iter()
            .map(|node| match node {
                RealNode::Inode(node) => RealNode::Inode(wrap(node)),
                RealNode::Enode(node) => RealNode::Enode(wrap(node)),
            })
            .collect();

        ContactPlan::new(realnodes, vnodes, contacts)
    }

    /// Parses an ASABR-format contact plan whose nodes carry `NM` tokens, then wraps it
    /// with a bucket table of `n_buckets` buckets (see `wrap_plan`).
    pub fn parse_from_iter<CM: ContactManager + LexFrom<str>>(
        iter: impl Iterator<Item: AsRef<str>>,
        n_buckets: usize,
    ) -> Result<ContactPlan<Self, CM>, ASABRError>
    where
        NM: LexFrom<str>,
    {
        Ok(Self::wrap_plan(parse_from_iter(iter)?, n_buckets))
    }
}

impl<NM: NodeManager> NodeManager for BucketHeuristicManager<NM> {
    fn accept(&self, bundle: &Bundle, time: TimeInterval, sender: NodeID) -> bool {
        self.inner.accept(bundle, time, sender)
    }

    fn process_delay(
        &self,
        bundle: &Bundle,
        reception: TimeInterval,
        sender: NodeID,
        nextvertex: NodeID,
    ) -> Date {
        self.inner
            .process_delay(bundle, reception, sender, nextvertex)
    }

    fn dry_run_retention(
        &self,
        bundle: &Bundle,
        reception: TimeInterval,
        sender: NodeID,
        transmission: TimeInterval,
        next: NodeID,
    ) -> bool {
        self.inner
            .dry_run_retention(bundle, reception, sender, transmission, next)
    }

    fn dry_run_multi(
        &self,
        bundle: &Bundle,
        reception: TimeInterval,
        sender: NodeID,
        transmissions: &[(TimeInterval, NodeID)],
    ) -> Option<usize> {
        self.inner
            .dry_run_multi(bundle, reception, sender, transmissions)
    }

    fn commit(
        &mut self,
        bundle: &Bundle,
        reception: TimeInterval,
        sender: NodeID,
        transmissions: &[(TimeInterval, NodeID)],
    ) -> Result<(), ASABRError> {
        self.inner.commit(bundle, reception, sender, transmissions)
    }
}

impl<NM: NodeManager> NodeHeuristic for BucketHeuristicManager<NM> {
    fn get_heuristic<'id, CM: ContactManager>(
        path: &PathFragment<'id>,
        graph: &Multigraph<'id, Self, CM>,
        target: NodeID,
    ) -> Date {
        let manager = &graph[path.rx_node].manager;
        let node = usize::from(graph.into_nodeid(path.rx_node.into()));
        let arrive_estimate = manager
            .bucket_table
            .bound(node, usize::from(target), path.recv.end);

        arrive_estimate.saturating_sub(path.recv.end)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact_manager::legacy::evl::EVLManager;
    use crate::node_manager::none::NoManagement;
    use alloc::vec::Vec;

    type NM = BucketHeuristicManager<NoManagement>;

    /// S -> R during buckets 0 and 1, S -> G during bucket 2, R -> G during bucket 3.
    const PLAN: &str = "node 0 S node 1 R node 2 G
                        contact 0 1 5 12 100 1
                        contact 0 2 22 24 100 1
                        contact 1 2 32 35 100 1";

    #[test]
    fn parse_wraps_every_node_with_shared_table() {
        let plan = NM::parse_from_iter::<EVLManager>(PLAN.lines(), 3).unwrap();
        assert_eq!(plan.realnodes.len(), 3);
        assert_eq!(plan.contacts.len(), 3);

        let managers: Vec<&NM> = plan
            .realnodes
            .iter()
            .map(|node| match node {
                RealNode::Inode(node) | RealNode::Enode(node) => &node.manager,
            })
            .collect();
        assert!(
            managers
                .windows(2)
                .all(|m| Rc::ptr_eq(&m[0].bucket_table, &m[1].bucket_table))
        );

        // plan spans [5, 35], 3 buckets of width 10: [5, 15), [15, 25), [25, 35]
        let table = &managers[0].bucket_table;
        assert_eq!(table.bound(0, 2, 10), 15);
        assert_eq!(table.bound(1, 2, 10), 25);
        // other destinations are computed on demand
        assert_eq!(table.bound(2, 0, 10), Date::MAX);
    }
}
