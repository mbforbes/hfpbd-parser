import json

from libpgm.orderedskeleton import OrderedSkeleton
from libpgm.nodedata import NodeData
from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.tablecpdfactorization import TableCPDFactorization

nd = NodeData()
skel = OrderedSkeleton()
nd.load("hfpbd_draft1.json")
skel.load("hfpbd_draft1.json")
bn = DiscreteBayesianNetwork(skel, nd)

evidence = {
    'u_verb': 'move',
    # 'u_lr': 'left',
    # 'u_gripper': 'true',
    # 'u_cw': 'cw',
    'u_dir': 'left',
}
# queries only look atkeys, and provide results for all values
query = {
    # 'command':'',
    'lr':'',
}

# load factorization
fn = TableCPDFactorization(bn)

# calculate probability distribution
result = fn.condprobve(query, evidence)
print result.scope
print result.vals

