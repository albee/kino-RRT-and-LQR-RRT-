# Define SearchNode class to represent a search node
# Oliver Jia-Richards, Keenan Albee, and Hailee Hettrick
class SearchNode(object):
    def __init__(self, state, parent=None, cost=0.0, control=0.0):

        # Store internal representations of input parameters
        self.state = state
        self.parent = parent
        self.cost = cost
        self.control = control


# Define Path class to represent a path
class Path(object):
    def __init__(self, search_node):
        # Construct path recursively
        path = []
        node = search_node
        while node is not None:
            path.append(node.state)
            node = node.parent
        path.reverse()

        self.path = path
        self.cost = search_node.cost
