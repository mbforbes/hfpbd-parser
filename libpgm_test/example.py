import json

from libpgm.orderedskeleton import OrderedSkeleton
from libpgm.nodedata import NodeData
from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.tablecpdfactorization import TableCPDFactorization

# ref: http://books.google.com/books?id=Ik_lAwAAQBAJ&pg=PT197&lpg=PT197&dq=libpgm+inference&source=bl&ots=oKww8Q9Xa5&sig=EMF2womqwDt4_c68Y4c3aa3nenE&hl=en&sa=X&ei=Q3HEU9ikFKb2igKk4IDYCA&ved=0CFIQ6AEwBw#v=onepage&q=inference&f=false
# ref: http://pythonhosted.org/libpgm/tablecpdfactorization.html?highlight=condprobve#libpgm.tablecpdfactorization.TableCPDFactorization.condprobve

# load nodedata and graphskeleton
nd = NodeData()
skel = OrderedSkeleton()
nd.load("node_data.json")
skel.load("example_graph.json")

# load evidence
# evidence = dict(Letter='weak')
# query = dict(Grade='A')

evidence = dict(Letter='weak')
query = dict(Intelligence='barbarbar')

# load bayesian network
bn = DiscreteBayesianNetwork(skel, nd)

# load factorization
fn = TableCPDFactorization(bn)

# calculate probability distribution
result = fn.condprobve(query, evidence)

# output
print json.dumps(result.vals, indent=2)
# print json.dumps(result.scope, indent=2)
# print json.dumps(result.card, indent=2)
# print json.dumps(result.stride, indent=2)
