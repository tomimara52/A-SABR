extern crate alloc;

use crate::{
    contact_manager::ContactManager,
    contact_plan::ContactPlan,
    node_manager::NodeManager,
    types::{Date, Duration},
};
use alloc::{vec, vec::Vec};
use core::cell::OnceCell;

/// Marks a (node, bucket) from which the destination cannot be reached.
const UNREACHABLE: u16 = u16::MAX;

/// Time-bucketed reachability table used as an A* heuristic.
///
/// The plan's time span is split into `n_buckets` buckets of `width`. Inside a bucket,
/// every contact overlapping it is considered present for the whole bucket. For each
/// destination `d`, `B(n, k)` is the earliest bucket in which `d` can be reached from
/// node `n` during bucket `k`. Since this relaxes the real contact plan, the start of
/// bucket `B(n, k)` is a lower bound on the arrival date (admissible and consistent).
///
/// The per-bucket graphs are built once from the plan, and the table of a destination
/// is computed the first time it is needed (see `bound`).
///
/// Node and destination indices are positions in `ContactPlan::realnodes`, which match
/// the routable index for internal nodes.
#[derive(Debug, Clone, Default)]
pub struct BucketTable {
    t0: Date,
    width: Duration,
    n_buckets: usize,
    /// `in_edges[k][rx] = [tx_, ...]`  
    /// means there is a contact (tx -> rx) overlapping with bucket k
    in_edges: Vec<Vec<Vec<usize>>>,
    /// `buckets_per_dest[dest][k][node] = B(node, k)`, empty until `dest` is needed.
    buckets_per_dest: Vec<OnceCell<Vec<Vec<u16>>>>,
}

impl BucketTable {
    /// Splits `[t0, tf]` into buckets of `width` and builds the per-bucket graphs of
    /// `contact_plan`.
    ///
    /// # Panics
    /// If it produces more than `u16::MAX - 1` buckets.
    pub fn new<NM: NodeManager, CM: ContactManager>(
        contact_plan: &ContactPlan<NM, CM>,
        t0: Date,
        tf: Date,
        width: Duration,
    ) -> Self {
        let width = width.max(1);
        let n_buckets = (((tf - t0 + width - 1) / width) as usize).max(1);
        assert!(
            n_buckets < UNREACHABLE as usize,
            "BucketTable supports at most {} buckets",
            UNREACHABLE - 1
        );
        let mut table = BucketTable {
            t0,
            width,
            n_buckets,
            in_edges: Vec::new(),
            buckets_per_dest: vec![OnceCell::new(); contact_plan.realnodes.len()],
        };
        table.in_edges = table.build_in_edges(contact_plan);
        table
    }

    /// Splits the plan's span (earliest contact start to latest contact end) into
    /// `n_buckets` buckets of equal width.
    pub fn from_plan<NM: NodeManager, CM: ContactManager>(
        contact_plan: &ContactPlan<NM, CM>,
        n_buckets: usize,
    ) -> Self {
        let lifespans = contact_plan.contacts.iter().map(|(c, _, _)| c.lifespan);
        let t0 = lifespans.clone().map(|l| l.start).min().unwrap_or(0);
        let tf = lifespans.map(|l| l.end).max().unwrap_or(t0);
        let n_buckets = n_buckets.max(1) as Duration;
        let width = ((tf - t0).max(1) + n_buckets - 1) / n_buckets;
        Self::new(contact_plan, t0, tf, width)
    }

    /// Computes the table for a single destination ahead of time, if not done yet.
    pub fn compute_for_dest(&self, dest: usize) {
        self.dest_table(dest);
    }

    /// Computes the tables for several destinations ahead of time, if not done yet.
    pub fn compute_for_dests(&self, dests: &[usize]) {
        for &dest in dests {
            self.dest_table(dest);
        }
    }

    /// Table of `dest`, computed on first access. `None` if `dest` is not a real node.
    fn dest_table(&self, dest: usize) -> Option<&Vec<Vec<u16>>> {
        let cell = self.buckets_per_dest.get(dest)?;
        // the table for a `dest` is only computed when first needed
        Some(cell.get_or_init(|| self.compute_table(dest)))
    }

    /// Computes `B(node, k)` for every node and bucket, for one destination.
    fn compute_table(&self, dest: usize) -> Vec<Vec<u16>> {
        let n_nodes = self.buckets_per_dest.len();
        let mut table = vec![vec![UNREACHABLE; n_nodes]; self.n_buckets];

        // buffers reused across buckets
        let mut labels = vec![UNREACHABLE; n_nodes];
        let mut order: Vec<usize> = Vec::with_capacity(n_nodes);
        let mut visited = vec![false; n_nodes];
        let mut stack: Vec<usize> = Vec::new();

        for k in (0..self.n_buckets).rev() {
            // L(m) = B(m, k + 1), except the destination which is reached in bucket k
            if k + 1 < self.n_buckets {
                labels.copy_from_slice(&table[k + 1]);
            } else {
                labels.fill(UNREACHABLE);
            }
            labels[dest] = k as u16;

            // B(n, k) = min L(m) over m reachable from n inside bucket k:
            order.clear();
            order.extend((0..n_nodes).filter(|&m| labels[m] != UNREACHABLE));
            order.sort_unstable_by_key(|&m| labels[m]);
            visited.fill(false);

            let row = &mut table[k];
            for &m in &order {
                if visited[m] {
                    continue;
                }
                let label = labels[m];
                visited[m] = true;
                row[m] = label;
                stack.push(m);
                while let Some(x) = stack.pop() {
                    for &tx in &self.in_edges[k][x] {
                        if !visited[tx] {
                            visited[tx] = true;
                            row[tx] = label;
                            stack.push(tx);
                        }
                    }
                }
            }
        }

        table
    }

    /// Lower bound on the arrival date at `dest` for a bundle held at `node` at `t`.
    ///
    /// Computes the table of `dest` on first call. Returns `t` (no information) if `dest`
    /// is not a real node, and `Date::MAX` if `dest` cannot be reached anymore.
    pub fn bound(&self, node: usize, dest: usize, t: Date) -> Date {
        let Some(table) = self.dest_table(dest) else {
            return t;
        };
        // Past the last bucket we keep using it: contacts after `tf` were clamped into it.
        match table[self.bucket(t)].get(node) {
            Some(&UNREACHABLE) => Date::MAX,
            Some(&b) => t.max(self.t0 + b as Date * self.width),
            None => t,
        }
    }

    /// For every bucket `k` and receiver `rx`, the senders `tx` having at least one
    /// `tx -> rx` contact overlapping bucket `k`: `in_edges[k][rx] = [tx, ...]`.
    /// Each `tx` appears at most once per `(k, rx)`.
    fn build_in_edges<NM: NodeManager, CM: ContactManager>(
        &self,
        contact_plan: &ContactPlan<NM, CM>,
    ) -> Vec<Vec<Vec<usize>>> {
        let n_nodes = contact_plan.realnodes.len();
        let mut in_edges: Vec<Vec<Vec<usize>>> = vec![vec![Vec::new(); n_nodes]; self.n_buckets];

        // visit contacts grouped by link (tx, rx) and, within a link, by start time
        let contacts = &contact_plan.contacts;
        let mut order: Vec<usize> = (0..contacts.len()).collect();
        order.sort_unstable_by_key(|&i| {
            let (contact, tx, rx) = &contacts[i];
            (*tx, *rx, contact.lifespan.start)
        });

        let mut current_link: Option<(usize, usize)> = None;
        // first bucket not yet pushed for `current_link`, to avoid having a link twice in the same
        // bucket
        let mut next_free: usize = 0;

        for i in order {
            let (contact, tx, rx) = &contacts[i];
            let (tx, rx) = (*tx, *rx);

            // invalid indices are rejected later by `Multigraph::new`
            if tx >= n_nodes || rx >= n_nodes || contact.lifespan.end < contact.lifespan.start {
                continue;
            }

            if current_link != Some((tx, rx)) {
                current_link = Some((tx, rx));
                next_free = 0;
            }

            let b_start = self.bucket(contact.lifespan.start);
            let b_end = self.bucket(contact.lifespan.end);

            // contacts of this link are sorted by start: buckets below `next_free` are covered
            for bucket in in_edges
                .iter_mut()
                .take(b_end + 1)
                .skip(b_start.max(next_free))
            {
                bucket[rx].push(tx);
            }
            next_free = next_free.max(b_end + 1);
        }

        in_edges
    }

    /// Bucket containing `t`, clamped to `0..n_buckets`.
    fn bucket(&self, t: Date) -> usize {
        if t <= self.t0 {
            return 0;
        }
        (((t - self.t0) / self.width) as usize).min(self.n_buckets - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact_manager::legacy::evl::EVLManager;
    use crate::contact_plan::asabr_file_lexer::parse_from_iter;
    use crate::node_manager::none::NoManagement;

    const S: usize = 0;
    const R: usize = 1;
    const G: usize = 2;
    const INF: u16 = UNREACHABLE;

    /// S -> R during buckets 0 and 1, S -> G during bucket 2, R -> G during bucket 3.
    fn plan() -> ContactPlan<NoManagement, EVLManager> {
        let plan = "node 0 S node 1 R node 2 G
                    contact 0 1 5 12 100 1
                    contact 0 2 22 24 100 1
                    contact 1 2 32 35 100 1";
        parse_from_iter::<NoManagement, EVLManager>(plan.lines()).unwrap()
    }

    fn table() -> BucketTable {
        BucketTable::new(&plan(), 0, 40, 10)
    }

    #[test]
    fn in_edges_per_bucket() {
        let in_edges = table().in_edges;
        assert_eq!(in_edges[0][R], vec![S]);
        assert_eq!(in_edges[1][R], vec![S]);
        assert_eq!(in_edges[2][G], vec![S]);
        assert_eq!(in_edges[3][G], vec![R]);
        let total: usize = in_edges.iter().flatten().map(Vec::len).sum();
        assert_eq!(total, 4);
    }

    #[test]
    fn in_edges_without_duplicates() {
        let plan = "node 0 A node 1 B
                    contact 0 1 1 3 100 1
                    contact 0 1 4 6 100 1
                    contact 0 1 2 15 100 1";
        let plan = parse_from_iter::<NoManagement, EVLManager>(plan.lines()).unwrap();
        let in_edges = BucketTable::new(&plan, 0, 20, 10).in_edges;
        assert_eq!(in_edges[0][1], vec![0]);
        assert_eq!(in_edges[1][1], vec![0]);
    }

    #[test]
    fn table_values() {
        let table = table();
        let t = table.dest_table(G).unwrap();
        let column = |n: usize| t.iter().map(|row| row[n]).collect::<Vec<_>>();
        assert_eq!(column(S), vec![2, 2, 2, INF]);
        assert_eq!(column(R), vec![3, 3, 3, 3]);
        assert_eq!(column(G), vec![0, 1, 2, 3]);
    }

    #[test]
    fn bounds() {
        let table = table();
        assert_eq!(table.bound(S, G, 15), 20);
        assert_eq!(table.bound(R, G, 5), 30);
        // loose: S -> G ended at 24, but the contact counts for the whole bucket
        assert_eq!(table.bound(S, G, 25), 25);
        assert_eq!(table.bound(S, G, 35), Date::MAX);
        // past the end: R -> G is clamped into the last bucket
        assert_eq!(table.bound(R, G, 100), 100);
    }

    #[test]
    fn destinations_computed_on_demand() {
        let table = table();
        assert!(table.buckets_per_dest.iter().all(|cell| cell.get().is_none()));

        assert_eq!(table.bound(S, G, 15), 20);
        assert!(table.buckets_per_dest[G].get().is_some());
        assert!(table.buckets_per_dest[S].get().is_none());
        assert!(table.buckets_per_dest[R].get().is_none());

        table.compute_for_dests(&[S, R]);
        assert!(table.buckets_per_dest.iter().all(|cell| cell.get().is_some()));
    }

    #[test]
    fn non_real_destination_gives_no_bound() {
        let table = table();
        assert_eq!(table.bound(S, 3, 15), 15);
    }
}
