import json

from libpgm.orderedskeleton import OrderedSkeleton
from libpgm.nodedata import NodeData
from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.tablecpdfactorization import TableCPDFactorization

nd = NodeData()
skel = OrderedSkeleton()
nd.load("sprinkler.json")
skel.load("sprinkler.json")
bn = DiscreteBayesianNetwork(skel, nd)

evidence = {'Grass wet':'true'}
# queries only look at keys, and provide results for all values
query = {'Rain':'', 'Sprinkler':''}

# load factorization
fn = TableCPDFactorization(bn)

# calculate probability distribution
result = fn.condprobve(query, evidence)
print result.scope
print result.vals
